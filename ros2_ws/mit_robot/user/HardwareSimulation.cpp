#include "HardwareSimulation.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <pty.h>
#include <stdexcept>
#include <unistd.h>
#include "dm_imu/bsp_crc.h"

namespace
{
constexpr std::array<const char *, kNumJoints> kJointNames{
  "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
  "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};
using namespace dm1_hardware::mit_protocol;

std::uint16_t quantize(double value, double range, unsigned bits)
{
  return static_cast<std::uint16_t>(std::lround(
    (std::clamp(value, -range, range) + range) / (2.0 * range) * ((1U << bits) - 1U)));
}
double decode(std::uint16_t value, double minimum, double maximum, unsigned bits)
{
  return static_cast<double>(value) / ((1U << bits) - 1U) * (maximum - minimum) + minimum;
}
HardwareSimulation::Clock::duration milliseconds(double ms)
{
  return std::chrono::duration_cast<HardwareSimulation::Clock::duration>(
    std::chrono::duration<double, std::milli>(ms));
}
// Device approximation only: rated 40 rpm/30 Nm, no-load 60 rpm, peak 97 Nm.
double torqueLimit(double torque, double velocity)
{
  if (torque * velocity <= 0.0) {return 97.0;}
  constexpr double rated = 4.18879020478639, no_load = 6.28318530717959;
  const double speed = std::abs(velocity);
  if (speed <= rated) {return 97.0 + (30.0 - 97.0) * speed / rated;}
  return 30.0 * std::clamp((no_load - speed) / (no_load - rated), 0.0, 1.0);
}
}  // namespace

void HardwareSimulationOptions::validate() const
{
  if (!std::isfinite(torque_limit_nm) || torque_limit_nm <= 0.0F || torque_limit_nm > 97.0F ||
    !std::isfinite(imu_period_ms) || imu_period_ms < 1.0 || imu_period_ms > 1000.0 ||
    !std::isfinite(output_scale) || output_scale < 0.0 || output_scale > 1.0 ||
    weak_joint < -1 || weak_joint >= static_cast<int>(kNumJoints) ||
    !std::isfinite(drop_every) || drop_every < 0 || drop_every > 1000000 ||
    std::floor(drop_every) != drop_every ||
    !std::isfinite(impairment_after_s) || impairment_after_s < 0.0)
  {
    throw std::invalid_argument("invalid hardware simulation torque/rate/joint/impairment option");
  }
  for (double ms : {command_delay_ms, feedback_delay_ms, imu_delay_ms, motor_time_constant_ms}) {
    if (!std::isfinite(ms) || ms < 0.0 || ms > 1000.0) {
      throw std::invalid_argument("simulation delay/time constant must be in [0,1000] ms");
    }
  }
}

HardwareSimulation::HardwareSimulation(
  mjModel * model, mjData * data, std::recursive_mutex & mutex,
  const Calibration & calibration, const HardwareSimulationOptions & options)
: model_(model), data_(data), mutex_(mutex), calibration_(calibration), options_(options),
  imu_(model, data)
{
  options_.validate();
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto name = std::string(kJointNames[i]);
    const int joint = mj_name2id(model_, mjOBJ_JOINT, (name + "_joint").c_str());
    const int actuator = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
    if (joint < 0 || actuator < 0) {throw std::runtime_error("missing DM1 joint/actuator");}
    motors_[i].qpos = model_->jnt_qposadr[joint];
    motors_[i].dof = model_->jnt_dofadr[joint];
    motors_[i].actuator = actuator;
    motors_[i].zero_offset = calibration_[i].zero_position;
  }
  // Keep physical calf collision, but remove the enlarged home-only helpers.
  for (const char * leg : {"FR", "FL", "RR", "RL"}) {
    const int geom = mj_name2id(model_, mjOBJ_GEOM, (std::string(leg) + "_home_calf_contact").c_str());
    if (geom >= 0) {model_->geom_contype[geom] = model_->geom_conaffinity[geom] = 0;}
  }
  char device[128]{};
  if (::openpty(&master_fd_, &slave_fd_, device, nullptr, nullptr) != 0) {
    throw std::runtime_error("cannot create simulated DM IMU serial port");
  }
  imu_device_ = device;
  const int flags = ::fcntl(master_fd_, F_GETFL);
  if (flags < 0 || ::fcntl(master_fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
    ::close(master_fd_); ::close(slave_fd_);
    throw std::runtime_error("cannot configure simulated IMU serial port");
  }
}

HardwareSimulation::~HardwareSimulation()
{
  ::close(master_fd_);
  ::close(slave_fd_);
}

bool HardwareSimulation::open(std::uint8_t bus, Receiver receiver)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (bus >= receivers_.size()) {return false;}
  receivers_[bus] = std::move(receiver);
  return true;
}

void HardwareSimulation::close(std::uint8_t bus)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (bus >= receivers_.size()) {return;}
  receivers_[bus] = {};
  commands_.erase(std::remove_if(commands_.begin(), commands_.end(),
    [bus](const auto & p) {return p.bus == bus;}), commands_.end());
  feedback_.erase(std::remove_if(feedback_.begin(), feedback_.end(),
    [bus](const auto & p) {return p.bus == bus;}), feedback_.end());
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (calibration_[i].address.bus == bus) {motors_[i].enabled = false; motors_[i].applied_torque = 0;}
  }
}

bool HardwareSimulation::write(std::uint8_t bus, const can_frame & frame)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (bus >= receivers_.size() || !receivers_[bus] || frame.can_dlc != 8) {return false;}
  const bool special = std::all_of(frame.data, frame.data + 7,
    [](std::uint8_t byte) {return byte == 0xFF;});
  const auto now = Clock::now();
  if (!special) {
    ++mit_writes_;
    if (options_.drop_every > 0 && enabled_at_ != Clock::time_point{} &&
      std::chrono::duration<double>(now - enabled_at_).count() >= options_.impairment_after_s &&
      mit_writes_ % static_cast<std::size_t>(options_.drop_every) == 0) {
      ++dropped_writes_;
      return false;
    }
  }
  commands_.push_back({now + milliseconds(options_.command_delay_ms), bus, frame});
  return true;
}

void HardwareSimulation::emergencyStop()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  commands_.clear(); feedback_.clear();
  enabled_at_ = {};
  for (auto & motor : motors_) {motor.enabled = false; motor.applied_torque = 0.0;}
  mju_zero(data_->qfrc_applied, model_->nv);
}

bool HardwareSimulation::motorsEnabled() const noexcept
{
  return std::any_of(motors_.begin(), motors_.end(), [](const Motor & m) {return m.enabled;});
}

void HardwareSimulation::receiveCommand(const PendingCan & pending, Clock::time_point now)
{
  const auto & frame = pending.frame;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (calibration_[i].address.bus != pending.bus || calibration_[i].address.can_id != frame.can_id) {
      continue;
    }
    auto & motor = motors_[i];
    const auto * d = frame.data;
    if (std::all_of(d, d + 7, [](std::uint8_t byte) {return byte == 0xFF;})) {
      if (d[7] == 0xFC) {
        motor.enabled = true;
        if (enabled_at_ == Clock::time_point{}) {enabled_at_ = now;}
      } else if (d[7] == 0xFD) {
        motor.enabled = false; motor.applied_torque = 0;
        if (!motorsEnabled()) {enabled_at_ = {};}
      } else if (d[7] == 0xFE) {
        motor.zero_offset = -calibration_[i].direction * data_->qpos[motor.qpos];
      }
    } else {
      // Firmware side of the existing driver's real encodeMitFrame().
      motor.q_des = decode((d[0] << 8) | d[1], -kPositionMax, kPositionMax, 16);
      motor.dq_des = decode((d[2] << 4) | (d[3] >> 4), -kVelocityMax, kVelocityMax, 12);
      motor.kp = decode(((d[3] & 15) << 8) | d[4], 0, kKpMax, 12);
      motor.kd = decode((d[5] << 4) | (d[6] >> 4), 0, kKdMax, 12);
      motor.ff = decode(((d[6] & 15) << 8) | d[7], -kTorqueMax, kTorqueMax, 12);
    }
    feedback_.push_back({now + milliseconds(options_.feedback_delay_ms), pending.bus, feedbackFrame(i)});
    break;
  }
}

canfd_frame HardwareSimulation::feedbackFrame(std::size_t joint) const
{
  const auto & c = calibration_[joint];
  const auto & m = motors_[joint];
  const auto q = quantize(c.direction * data_->qpos[m.qpos] + m.zero_offset, kPositionMax, 16);
  const auto dq = quantize(c.direction * data_->qvel[m.dof], kVelocityMax, 12);
  const auto tau = quantize(c.direction * m.applied_torque, kTorqueMax, 12);
  canfd_frame frame{};
  frame.can_id = c.address.master_id;
  frame.len = 8;
  frame.data[0] = static_cast<std::uint8_t>((m.enabled ? 0x10 : 0) | c.address.can_id);
  frame.data[1] = q >> 8; frame.data[2] = q & 0xFF;
  frame.data[3] = dq >> 4; frame.data[4] = ((dq & 15) << 4) | (tau >> 8);
  frame.data[5] = tau & 0xFF; frame.data[6] = 25; frame.data[7] = 25;
  return frame;
}

ImuDriver::Frame HardwareSimulation::imuFrame()
{
  const auto sample = imu_.read();
  const Mat3<float> r = sample.orientation_world_from_body.toRotationMatrix();
  const Vec3<float> rpy(
    std::atan2(r(2, 1), r(2, 2)), std::asin(std::clamp(-r(2, 0), -1.0F, 1.0F)),
    std::atan2(r(1, 0), r(0, 0)));
  const std::array<Vec3<float>, 3> values{
    sample.acceleration_body, sample.angular_velocity_body, rpy * 57.29577951308232F};
  ImuDriver::Frame frame{};
  for (std::size_t record = 0; record < values.size(); ++record) {
    const auto offset = record * 19;
    frame[offset] = 0x55; frame[offset + 1] = 0xAA;
    frame[offset + 2] = 1; frame[offset + 3] = record + 1;
    for (int axis = 0; axis < 3; ++axis) {
      const float value = values[record][axis];
      std::memcpy(frame.data() + offset + 4 + axis * sizeof(float), &value, sizeof(value));
    }
    const auto crc = Get_CRC16(frame.data() + offset, 16);
    frame[offset + 16] = crc & 0xFF; frame[offset + 17] = crc >> 8;
  }
  return frame;
}

void HardwareSimulation::tick(bool advance_physics)
{
  const auto now = Clock::now();
  while (!commands_.empty() && commands_.front().ready <= now) {
    receiveCommand(commands_.front(), now);
    commands_.pop_front();
  }
  while (!feedback_.empty() && feedback_.front().ready <= now) {
    const auto pending = feedback_.front();
    feedback_.pop_front();
    if (receivers_[pending.bus]) {receivers_[pending.bus](pending.frame);}
  }
  if (now >= next_imu_ && imu_frames_.size() < 512) {
    imu_frames_.push_back({now + milliseconds(options_.imu_delay_ms), imuFrame(), 0});
    next_imu_ = now + milliseconds(options_.imu_period_ms);
  }
  while (!imu_frames_.empty() && imu_frames_.front().ready <= now) {
    auto & pending = imu_frames_.front();
    const auto count = ::write(master_fd_, pending.frame.data() + pending.written,
      pending.frame.size() - pending.written);
    if (count > 0) {
      pending.written += static_cast<std::size_t>(count);
      if (pending.written == pending.frame.size()) {imu_frames_.pop_front();}
    } else if (count < 0 && (errno == EAGAIN || errno == EINTR)) {break;}
    else {throw std::runtime_error("simulated IMU serial write failed");}
  }
  mju_zero(data_->qfrc_applied, model_->nv);
  mju_zero(data_->ctrl, model_->nu);
  const bool impairment = enabled_at_ != Clock::time_point{} &&
    std::chrono::duration<double>(now - enabled_at_).count() >= options_.impairment_after_s;
  const double blend = options_.motor_time_constant_ms == 0 ? 1.0 :
    1.0 - std::exp(-model_->opt.timestep * 1000.0 / options_.motor_time_constant_ms);
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto & m = motors_[i];
    if (!m.enabled || !advance_physics) {m.applied_torque = 0.0; continue;}
    const auto & c = calibration_[i];
    const double q = c.direction * data_->qpos[m.qpos] + m.zero_offset;
    const double dq = c.direction * data_->qvel[m.dof];
    const double target = c.direction * (m.ff + m.kp * (m.q_des - q) + m.kd * (m.dq_des - dq));
    const double scale = impairment && static_cast<int>(i) == options_.weak_joint ? options_.output_scale : 1.0;
    const double limit = torqueLimit(target, data_->qvel[m.dof]) * scale;
    m.applied_torque += blend * (std::clamp(target, -limit, limit) - m.applied_torque);
    // Saturation also applies to lagged torque when a motor loses output.
    m.applied_torque = std::clamp(m.applied_torque, -limit, limit);
    data_->qfrc_applied[m.dof] = m.applied_torque;
  }
}
