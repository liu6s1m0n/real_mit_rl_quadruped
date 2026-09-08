#include "hardware/dm_motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <thread>

#include <linux/can.h>

namespace
{
constexpr float kPositionMax = 12.566F;  // DM6248P
constexpr float kVelocityMax = 20.0F;
constexpr float kTorqueMax = 120.0F;
constexpr float kKpMax = 500.0F;
constexpr float kKdMax = 5.0F;
constexpr double kFeedbackTimeout = 0.05;
constexpr auto kEnableConfirmationTimeout = std::chrono::milliseconds(100);
constexpr int kDisableAttempts = 3;

std::uint16_t floatToUint(float value, float minimum, float maximum, unsigned bits)
{
  const float normalized = (std::clamp(value, minimum, maximum) - minimum) /
    (maximum - minimum);
  const float scaled = normalized * static_cast<float>((1U << bits) - 1U);
  return static_cast<std::uint16_t>(std::lround(scaled));
}

float uintToFloat(std::uint16_t value, float minimum, float maximum, unsigned bits)
{
  return static_cast<float>(value) /
         static_cast<float>((1U << bits) - 1U) * (maximum - minimum) + minimum;
}
}  // namespace

DmMotorDriver::DmMotorDriver(
  std::string can0, std::string can1, const CalibrationArray & calibration)
: can_names_{std::move(can0), std::move(can1)}, calibration_(calibration)
{
  for (std::size_t i = 0; i < calibration_.size(); ++i) {
    const auto & address = calibration_[i].address;
    if (address.bus > 1 || address.can_id == 0 || address.can_id > 0x0F ||
      address.master_id == 0 || address.master_id > CAN_SFF_MASK ||
      indexFor(address) != i)
    {
      throw std::invalid_argument("duplicate or invalid DM motor bus/CAN mapping");
    }
    for (std::size_t j = 0; j < i; ++j) {
      const auto & previous = calibration_[j].address;
      if (address.bus == previous.bus && address.master_id == previous.master_id) {
        throw std::invalid_argument("duplicate DM motor bus/master mapping");
      }
    }
  }
}

DmMotorDriver::~DmMotorDriver()
{
  close();
}

bool DmMotorDriver::decodeFeedback(
  const canfd_frame & frame, DecodedFeedback & decoded) noexcept
{
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
    frame.len != 8)
  {
    return false;
  }

  decoded = {};
  decoded.motor_id = static_cast<std::uint16_t>(frame.data[0] & 0x0F);
  decoded.status = static_cast<std::uint8_t>(frame.data[0] >> 4);
  if (decoded.motor_id == 0) {return false;}
  const std::uint16_t q_uint =
    (static_cast<std::uint16_t>(frame.data[1]) << 8) | frame.data[2];
  const std::uint16_t dq_uint =
    (static_cast<std::uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const std::uint16_t tau_uint =
    (static_cast<std::uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];
  decoded.position = uintToFloat(q_uint, -kPositionMax, kPositionMax, 16);
  decoded.velocity = uintToFloat(dq_uint, -kVelocityMax, kVelocityMax, 12);
  decoded.torque = uintToFloat(tau_uint, -kTorqueMax, kTorqueMax, 12);
  decoded.mos_temperature_c = static_cast<float>(frame.data[6]);
  decoded.rotor_temperature_c = static_cast<float>(frame.data[7]);
  return std::isfinite(decoded.position) && std::isfinite(decoded.velocity) &&
         std::isfinite(decoded.torque);
}

bool DmMotorDriver::matchesFeedbackAddress(
  const dm1_hardware::MotorAddress & expected, std::uint16_t master_id,
  const DecodedFeedback & decoded) noexcept
{
  return expected.can_id == decoded.motor_id && expected.master_id == master_id;
}

can_frame DmMotorDriver::encodeMitFrame(
  const dm1_hardware::MotorAddress & address,
  float position, float velocity, float kp, float kd, float torque) noexcept
{
  const std::uint16_t q = floatToUint(position, -kPositionMax, kPositionMax, 16);
  const std::uint16_t dq = floatToUint(velocity, -kVelocityMax, kVelocityMax, 12);
  const std::uint16_t kp_uint = floatToUint(kp, 0.0F, kKpMax, 12);
  const std::uint16_t kd_uint = floatToUint(kd, 0.0F, kKdMax, 12);
  const std::uint16_t tau = floatToUint(torque, -kTorqueMax, kTorqueMax, 12);
  can_frame output{};
  // DM control commands use the physical motor CAN ID. Master ID is only used
  // by the motor in its response frame.
  output.can_id = address.can_id;
  output.can_dlc = 8;
  output.data[0] = static_cast<std::uint8_t>(q >> 8);
  output.data[1] = static_cast<std::uint8_t>(q);
  output.data[2] = static_cast<std::uint8_t>(dq >> 4);
  output.data[3] = static_cast<std::uint8_t>(((dq & 0x0F) << 4) | (kp_uint >> 8));
  output.data[4] = static_cast<std::uint8_t>(kp_uint);
  output.data[5] = static_cast<std::uint8_t>(kd_uint >> 4);
  output.data[6] = static_cast<std::uint8_t>(((kd_uint & 0x0F) << 4) | (tau >> 8));
  output.data[7] = static_cast<std::uint8_t>(tau);
  return output;
}

can_frame DmMotorDriver::encodeCommandFrame(
  const dm1_hardware::MotorAddress & address, std::uint8_t command) noexcept
{
  can_frame output{};
  output.can_id = address.can_id;
  output.can_dlc = 8;
  std::fill_n(output.data, 7, static_cast<std::uint8_t>(0xFF));
  output.data[7] = command;
  return output;
}

std::size_t DmMotorDriver::indexFor(
  const dm1_hardware::MotorAddress & address) const noexcept
{
  for (std::size_t i = 0; i < calibration_.size(); ++i) {
    const auto & configured = calibration_[i].address;
    if (configured.bus == address.bus && configured.can_id == address.can_id) {return i;}
  }
  return kNumJoints;
}

bool DmMotorDriver::open()
{
  if (opened_.load()) {return true;}
  if (can_names_[0].empty() || can_names_[1].empty()) {return false;}

  const bool first = buses_[0].open(
    can_names_[0], [this](const canfd_frame & frame) {receive(0, frame);}, 0);
  if (!first) {return false;}
  const bool second = buses_[1].open(
    can_names_[1], [this](const canfd_frame & frame) {receive(1, frame);}, 0);
  if (!second) {
    buses_[0].close();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshots_ = {};
  }
  opened_.store(true);
  output_disabled_.store(false);
  return true;
}

bool DmMotorDriver::pollAll()
{
  if (!opened_.load()) {return false;}
  for (const auto & item : calibration_) {
    const auto & address = item.address;
    dm1_hardware::MitFrame frame{};
    frame.bus = address.bus;
    frame.can_id = address.can_id;
    frame.master_id = address.master_id;
    if (!sendMit(frame)) {return false;}
  }
  return true;
}

bool DmMotorDriver::latest(FeedbackArray & feedback, double now_s) const
{
  if (!opened_.load() || !std::isfinite(now_s)) {return false;}
  const auto now = std::chrono::steady_clock::now();
  bool valid = true;
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < snapshots_.size(); ++i) {
    const auto & address = calibration_[i].address;
    const auto & snapshot = snapshots_[i];
    auto & sample = feedback[i];
    sample = {};
    sample.bus = address.bus;
    sample.can_id = address.can_id;
    sample.position = snapshot.position;
    sample.velocity = snapshot.velocity;
    sample.torque = snapshot.torque;
    sample.temperature_c = snapshot.mos_temperature_c;
    sample.rotor_temperature_c = snapshot.rotor_temperature_c;
    sample.fault_code = isHealthyStatus(snapshot.status) ? 0 : snapshot.status;
    sample.sequence = snapshot.sequence;
    const double age = snapshot.online ?
      std::chrono::duration<double>(now - snapshot.received_at).count() :
      std::numeric_limits<double>::infinity();
    sample.timestamp = std::max(0.0, now_s - age);
    sample.health_valid = snapshot.online && isHealthyStatus(snapshot.status) &&
      snapshot.mos_temperature_c >= -20.0F && snapshot.mos_temperature_c <= 100.0F &&
      snapshot.rotor_temperature_c >= -20.0F && snapshot.rotor_temperature_c <= 100.0F;
    if (!snapshot.online || !std::isfinite(age) || age > kFeedbackTimeout ||
      !sample.health_valid)
    {
      valid = false;
    }
  }
  return valid;
}

void DmMotorDriver::receive(std::uint8_t bus, const canfd_frame & frame) noexcept
{
  const std::uint32_t raw_id = frame.can_id;
  if (bus > 1) {return;}
  DecodedFeedback decoded;
  if (!decodeFeedback(frame, decoded)) {return;}

  // DM MIT feedback uses the configured Master ID as the CAN arbitration ID.
  // The physical motor ID is carried in D0[3:0]; D0[7:4] is the motor status.
  const std::uint16_t master_id = static_cast<std::uint16_t>(raw_id & CAN_SFF_MASK);
  const dm1_hardware::MotorAddress address{bus, decoded.motor_id, master_id};
  const std::size_t index = indexFor(address);
  if (index >= kNumJoints) {return;}
  const auto & configured = calibration_[index].address;
  if (!matchesFeedbackAddress(configured, master_id, decoded)) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  auto & snapshot = snapshots_[index];
  snapshot.position = decoded.position;
  snapshot.velocity = decoded.velocity;
  snapshot.torque = decoded.torque;
  snapshot.status = decoded.status;
  snapshot.mos_temperature_c = decoded.mos_temperature_c;
  snapshot.rotor_temperature_c = decoded.rotor_temperature_c;
  snapshot.received_at = std::chrono::steady_clock::now();
  ++snapshot.sequence;
  snapshot.online = configured.bus == bus && configured.can_id == decoded.motor_id &&
    configured.master_id == master_id;
}

bool DmMotorDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  if (!opened_.load()) {return false;}
  const dm1_hardware::MotorAddress address{frame.bus, frame.can_id, frame.master_id};
  const std::size_t index = indexFor(address);
  if (index >= kNumJoints || !std::isfinite(frame.position) ||
    !std::isfinite(frame.velocity) || !std::isfinite(frame.kp) ||
    !std::isfinite(frame.kd) || !std::isfinite(frame.torque) ||
    frame.position < -kPositionMax || frame.position > kPositionMax ||
    std::abs(frame.velocity) > kVelocityMax || frame.kp < 0.0F || frame.kp > kKpMax ||
    frame.kd < 0.0F || frame.kd > kKdMax || std::abs(frame.torque) > kTorqueMax)
  {
    return false;
  }

  const auto & configured = calibration_[index].address;
  if (frame.master_id != 0 && frame.master_id != configured.master_id) {return false;}
  const can_frame output = encodeMitFrame(
    configured, frame.position, frame.velocity, frame.kp, frame.kd, frame.torque);
  return buses_[address.bus].write(&output);
}

bool DmMotorDriver::sendCommand(
  const dm1_hardware::MotorAddress & address, std::uint8_t command)
{
  if (!opened_.load() || address.can_id > CAN_SFF_MASK) {
    return false;
  }
  const auto index = indexFor(address);
  if (index >= kNumJoints) {return false;}
  const auto & configured = calibration_[index].address;
  if ((address.master_id != 0 && address.master_id != configured.master_id) ||
    configured.master_id > CAN_SFF_MASK)
  {
    return false;
  }
  const can_frame output = encodeCommandFrame(configured, command);
  return buses_[address.bus].write(&output);
}

bool DmMotorDriver::allMotorsEnabled() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::all_of(
    snapshots_.begin(), snapshots_.end(), [](const Snapshot & snapshot) {
      return snapshot.online &&
      snapshot.status == static_cast<std::uint8_t>(MotorStatus::Enabled);
    });
}

bool DmMotorDriver::setZero(const dm1_hardware::MotorAddress & address)
{
  return sendCommand(address, 0xFE);
}

bool DmMotorDriver::enableAll()
{
  if (!opened_.load()) {return false;}
  output_disabled_.store(false);
  bool result = true;
  for (const auto & item : calibration_) {
    result = sendCommand(item.address, 0xFC) && result;
  }
  if (!result) {
    disableAll();
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + kEnableConfirmationTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!pollAll()) {
      disableAll();
      return false;
    }
    if (allMotorsEnabled()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  disableAll();
  return false;
}

void DmMotorDriver::disableAll() noexcept
{
  if (!opened_.load() || output_disabled_.load()) {return;}
  for (int attempt = 0; attempt < kDisableAttempts; ++attempt) {
    bool all_sent = true;
    for (const auto & item : calibration_) {
      all_sent = sendCommand(item.address, 0xFD) && all_sent;
    }
    if (all_sent) {
      output_disabled_.store(true);
      return;
    }
  }
}

void DmMotorDriver::close() noexcept
{
  if (!opened_.load()) {return;}
  disableAll();
  opened_.store(false);
  buses_[0].close();
  buses_[1].close();
}
