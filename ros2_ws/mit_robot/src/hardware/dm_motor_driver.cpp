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
// ---------- DM MIT 协议范围和驱动时序参数 ----------
constexpr float kPositionMax = dm1_hardware::mit_protocol::kPositionMax;  // DM6248P 位置上限。
constexpr float kVelocityMax = dm1_hardware::mit_protocol::kVelocityMax;  // 速度上限。
constexpr float kTorqueMax = dm1_hardware::mit_protocol::kTorqueMax;  // 力矩上限。
constexpr float kKpMax = dm1_hardware::mit_protocol::kKpMax;  // kp 上限。
constexpr float kKdMax = dm1_hardware::mit_protocol::kKdMax;  // kd 上限。
constexpr double kFeedbackTimeout = 0.05;  // 驱动层允许的反馈最大年龄，s。
constexpr auto kEnableConfirmationTimeout = std::chrono::milliseconds(100);  // 使能确认超时。
constexpr int kDisableAttempts = 3;  // 禁用命令的最大重试次数。

// 将指定范围内的浮点值量化为无符号整数，用于写入 MIT 数据帧。
std::uint16_t floatToUint(float value, float minimum, float maximum, unsigned bits)
{
  // 先限制到协议范围，再映射到 bits 位整数的完整区间。
  const float normalized = (std::clamp(value, minimum, maximum) - minimum) /
    (maximum - minimum);
  const float scaled = normalized * static_cast<float>((1U << bits) - 1U);
  return static_cast<std::uint16_t>(std::lround(scaled));
}

// 将 MIT 数据帧中的无符号整数还原为指定范围内的浮点值。
float uintToFloat(std::uint16_t value, float minimum, float maximum, unsigned bits)
{
  return static_cast<float>(value) /
         static_cast<float>((1U << bits) - 1U) * (maximum - minimum) + minimum;
}
}  // namespace

// 构造驱动器，保存两条 CAN 总线名称和全部电机标定信息，并校验地址唯一性。
DmMotorDriver::DmMotorDriver(
  std::string can0, std::string can1, const CalibrationArray & calibration)
: can_names_{std::move(can0), std::move(can1)}, calibration_(calibration)
{
  // 每个标定项必须使用合法总线、物理 ID、主站 ID 和有限的零位。
  for (std::size_t i = 0; i < calibration_.size(); ++i) {
    const auto & address = calibration_[i].address;
    if (address.bus > 1 || address.can_id == 0 || address.can_id > 0x0F ||
      address.master_id == 0 || address.master_id > CAN_SFF_MASK ||
      !std::isfinite(calibration_[i].zero_position) ||
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

// 析构时统一关闭电机输出和 CAN 总线。
DmMotorDriver::~DmMotorDriver()
{
  close();
}

bool DmMotorDriver::decodeFeedback(
  const canfd_frame & frame, DecodedFeedback & decoded) noexcept
{
  // 只接受普通标准 CAN 数据帧，且 DM MIT 反馈必须正好有 8 个字节。
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
    frame.len != 8)
  {
    return false;
  }

  decoded = {};
  // 第一个字节的低 4 位是物理电机 ID，高 4 位是电机状态码。
  decoded.motor_id = static_cast<std::uint16_t>(frame.data[0] & 0x0F);
  decoded.status = static_cast<std::uint8_t>(frame.data[0] >> 4);
  if (decoded.motor_id == 0) {return false;}

  // 按 DM MIT 协议的位布局拼接位置、速度和力矩的量化整数。
  const std::uint16_t q_uint =
    (static_cast<std::uint16_t>(frame.data[1]) << 8) | frame.data[2];
  const std::uint16_t dq_uint =
    (static_cast<std::uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const std::uint16_t tau_uint =
    (static_cast<std::uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];

  // 将量化整数转换为协议定义范围内的物理量。
  decoded.position = uintToFloat(q_uint, -kPositionMax, kPositionMax, 16);
  decoded.velocity = uintToFloat(dq_uint, -kVelocityMax, kVelocityMax, 12);
  decoded.torque = uintToFloat(tau_uint, -kTorqueMax, kTorqueMax, 12);
  decoded.mos_temperature_c = static_cast<float>(frame.data[6]);
  decoded.rotor_temperature_c = static_cast<float>(frame.data[7]);

  // 位置、速度和力矩必须是有限值，防止异常数据进入后续缓存。
  return std::isfinite(decoded.position) && std::isfinite(decoded.velocity) &&
         std::isfinite(decoded.torque);
}

bool DmMotorDriver::matchesFeedbackAddress(
  const dm1_hardware::MotorAddress & expected, std::uint16_t master_id,
  const DecodedFeedback & decoded) noexcept
{
  // 反馈中的物理电机 ID 和 CAN 仲裁 ID 必须同时匹配标定配置。
  return expected.can_id == decoded.motor_id && expected.master_id == master_id;
}

// 将位置、速度、增益和力矩编码为 DM MIT 的 8 字节控制帧。
can_frame DmMotorDriver::encodeMitFrame(
  const dm1_hardware::MotorAddress & address,
  float position, float velocity, float kp, float kd, float torque) noexcept
{
  // 根据各物理量的协议位宽完成量化。
  const std::uint16_t q = floatToUint(position, -kPositionMax, kPositionMax, 16);
  const std::uint16_t dq = floatToUint(velocity, -kVelocityMax, kVelocityMax, 12);
  const std::uint16_t kp_uint = floatToUint(kp, 0.0F, kKpMax, 12);
  const std::uint16_t kd_uint = floatToUint(kd, 0.0F, kKdMax, 12);
  const std::uint16_t tau = floatToUint(torque, -kTorqueMax, kTorqueMax, 12);
  can_frame output{};

  // DM 控制命令使用电机物理 CAN ID；主站 ID 只用于电机反馈帧的仲裁 ID。
  output.can_id = address.can_id;
  output.can_dlc = 8;

  // 按 MIT 协议将量化字段拆分到 8 个数据字节中。
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

// 将使能、禁用或设置零位等控制命令编码为 DM 控制帧。
can_frame DmMotorDriver::encodeCommandFrame(
  const dm1_hardware::MotorAddress & address, std::uint8_t command) noexcept
{
  can_frame output{};
  output.can_id = address.can_id;
  output.can_dlc = 8;

  // 命令帧前 7 个字节固定填充 0xFF，最后一个字节保存具体命令码。
  std::fill_n(output.data, 7, static_cast<std::uint8_t>(0xFF));
  output.data[7] = command;
  return output;
}

std::size_t DmMotorDriver::indexFor(
  const dm1_hardware::MotorAddress & address) const noexcept
{
  // 标定数组中的索引同时定义了电机反馈的内部固定顺序。
  for (std::size_t i = 0; i < calibration_.size(); ++i) {
    const auto & configured = calibration_[i].address;
    if (configured.bus == address.bus && configured.can_id == address.can_id) {return i;}
  }
  return kNumJoints;
}

bool DmMotorDriver::open()
{
  // 已打开时保持幂等；缺少任意一条 CAN 设备名称则无法启动。
  if (opened_.load()) {return true;}
  if (can_names_[0].empty() || can_names_[1].empty()) {return false;}

  const bool first = buses_[0].open(
    can_names_[0], [this](const canfd_frame & frame) {receive(0, frame);}, 0);
  // 第一条总线打开失败时，不继续尝试第二条总线。
  if (!first) {return false;}
  const bool second = buses_[1].open(
    can_names_[1], [this](const canfd_frame & frame) {receive(1, frame);}, 0);
  if (!second) {
    // 第二条总线打开失败时，回收已经打开的第一条总线。
    buses_[0].close();
    return false;
  }
  {
    // 清空接收缓存，要求启动后重新收到所有电机的反馈。
    std::lock_guard<std::mutex> lock(mutex_);
    snapshots_ = {};
  }
  opened_.store(true);
  output_disabled_.store(false);
  return true;
}

// 向所有电机发送零增益 MIT 帧，主动请求一轮反馈。
bool DmMotorDriver::pollAll()
{
  // 未打开 CAN 总线时不能轮询电机。
  if (!opened_.load()) {return false;}
  for (const auto & item : calibration_) {
    const auto & address = item.address;
    dm1_hardware::MitFrame frame{};
    frame.bus = address.bus;
    frame.can_id = address.can_id;
    frame.master_id = address.master_id;
    // 零初始化帧不会施加位置、速度或力矩控制，只用于触发反馈。
    if (!sendMit(frame)) {return false;}
  }
  return true;
}

// 读取 12 个电机的最新快照，并转换为上层使用的 MotorFeedback 数组。
bool DmMotorDriver::latest(FeedbackArray & feedback, double now_s) const
{
  // 读取接口保持非阻塞：这里只复制接收线程已经缓存的数据。
  if (!opened_.load() || !std::isfinite(now_s)) {return false;}
  const auto now = std::chrono::steady_clock::now();
  bool valid = true;
  std::lock_guard<std::mutex> lock(mutex_);

  // 按标定顺序逐个生成反馈，并检查在线状态、反馈年龄和电机健康状态。
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

    // 使用单调时钟计算快照年龄，并换算为控制层使用的时间戳。
    const double age = snapshot.online ?
      std::chrono::duration<double>(now - snapshot.received_at).count() :
      std::numeric_limits<double>::infinity();
    sample.timestamp = std::max(0.0, now_s - age);
    sample.health_valid = snapshot.online && isHealthyStatus(snapshot.status) &&
      snapshot.mos_temperature_c >= -20.0F && snapshot.mos_temperature_c <= 100.0F &&
      snapshot.rotor_temperature_c >= -20.0F && snapshot.rotor_temperature_c <= 100.0F;

    // 任意一个电机离线、反馈过期或状态/温度异常，整批反馈都判定为无效。
    if (!snapshot.online || !std::isfinite(age) || age > kFeedbackTimeout ||
      !sample.health_valid)
    {
      valid = false;
    }
  }
  return valid;
}

// 接收回调：验证 CAN 帧、匹配电机地址，并更新对应电机的最新快照。
void DmMotorDriver::receive(std::uint8_t bus, const canfd_frame & frame) noexcept
{
  const std::uint32_t raw_id = frame.can_id;
  // 回调只接受已知的两条 CAN 总线。
  if (bus > 1) {return;}
  DecodedFeedback decoded;
  // 帧格式或数据内容不正确时直接丢弃，不污染已有快照。
  if (!decodeFeedback(frame, decoded)) {return;}

  // DM MIT 反馈使用主站 ID 作为 CAN 仲裁 ID；
  // 物理电机 ID 位于 data[0] 的低 4 位，电机状态位于高 4 位。
  const std::uint16_t master_id = static_cast<std::uint16_t>(raw_id & CAN_SFF_MASK);
  const dm1_hardware::MotorAddress address{bus, decoded.motor_id, master_id};
  const std::size_t index = indexFor(address);
  // 未在标定表中登记的电机反馈不允许进入缓存。
  if (index >= kNumJoints) {return;}
  const auto & configured = calibration_[index].address;
  // 同时检查物理 ID 和主站 ID，防止不同总线或不同电机的帧串入。
  if (!matchesFeedbackAddress(configured, master_id, decoded)) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  auto & snapshot = snapshots_[index];

  // 保存当前帧的物理量和健康状态。
  snapshot.position = decoded.position;
  snapshot.velocity = decoded.velocity;
  snapshot.torque = decoded.torque;
  snapshot.status = decoded.status;
  snapshot.mos_temperature_c = decoded.mos_temperature_c;
  snapshot.rotor_temperature_c = decoded.rotor_temperature_c;
  snapshot.received_at = std::chrono::steady_clock::now();
  ++snapshot.sequence;
  // 地址已经匹配，因此该电机被标记为在线。
  snapshot.online = configured.bus == bus && configured.can_id == decoded.motor_id &&
    configured.master_id == master_id;
}

bool DmMotorDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  // 设备未打开时禁止访问底层 CAN 总线。
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
    // 所有量必须有限，并且不能超出 DM MIT 协议允许的物理范围。
    return false;
  }

  const auto & configured = calibration_[index].address;
  // master_id 为 0 时允许使用标定值；非零时必须与标定值一致。
  if (frame.master_id != 0 && frame.master_id != configured.master_id) {return false;}
  const can_frame output = encodeMitFrame(
    configured, frame.position, frame.velocity, frame.kp, frame.kd, frame.torque);
  return buses_[address.bus].write(&output);
}

// 发送一个原始控制命令，例如设置零位、使能或禁用电机。
bool DmMotorDriver::sendCommand(
  const dm1_hardware::MotorAddress & address, std::uint8_t command)
{
  // 目标地址必须属于已打开的驱动器和已登记的标定表。
  if (!opened_.load() || address.can_id > CAN_SFF_MASK) {
    return false;
  }
  const auto index = indexFor(address);
  if (index >= kNumJoints) {return false;}
  const auto & configured = calibration_[index].address;
  // 非零主站 ID 必须与标定配置一致。
  if ((address.master_id != 0 && address.master_id != configured.master_id) ||
    configured.master_id > CAN_SFF_MASK)
  {
    return false;
  }
  const can_frame output = encodeCommandFrame(configured, command);
  return buses_[address.bus].write(&output);
}

// 检查所有 12 个电机是否都在线，并且反馈状态均为已使能。
bool DmMotorDriver::allMotorsEnabled() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::all_of(
    snapshots_.begin(), snapshots_.end(), [](const Snapshot & snapshot) {
      // 只有在线且明确报告 Enabled 才算完成使能确认。
      return snapshot.online &&
      snapshot.status == static_cast<std::uint8_t>(MotorStatus::Enabled);
    });
}

// 将指定电机的当前位置写入驱动器零位。
bool DmMotorDriver::setZero(const dm1_hardware::MotorAddress & address)
{
  return sendCommand(address, 0xFE);
}

bool DmMotorDriver::enableAll()
{
  // 未打开 CAN 总线时不能执行使能流程。
  if (!opened_.load()) {return false;}
  output_disabled_.store(false);
  bool result = true;
  // 先向每台电机发送使能命令；任意发送失败都会触发整体失败。
  for (const auto & item : calibration_) {
    result = sendCommand(item.address, 0xFC) && result;
  }
  if (!result) {
    // 使能命令未能完整发出时，立即回到禁用状态。
    disableAll();
    return false;
  }

  // 发送轮询帧并等待所有电机反馈确认，避免只依赖命令发送成功。
  const auto deadline = std::chrono::steady_clock::now() + kEnableConfirmationTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!pollAll()) {
      // 轮询失败时禁止继续等待并关闭全部输出。
      disableAll();
      return false;
    }
    if (allMotorsEnabled()) {return true;}
    // 给接收线程时间处理电机反馈。
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // 超时仍未确认全部电机使能，执行安全禁用。
  disableAll();
  return false;
}

// 多次发送禁用命令，尽可能确保所有电机退出输出状态。
void DmMotorDriver::disableAll() noexcept
{
  // 已关闭或已经完成禁用时无需重复发送。
  if (!opened_.load() || output_disabled_.load()) {return;}
  for (int attempt = 0; attempt < kDisableAttempts; ++attempt) {
    bool all_sent = true;
    // 一次尝试向所有已标定电机发送禁用命令。
    for (const auto & item : calibration_) {
      all_sent = sendCommand(item.address, 0xFD) && all_sent;
    }
    if (all_sent) {
      // 只有全部禁用命令发送成功才记录完成状态。
      output_disabled_.store(true);
      return;
    }
  }
}

// 先禁用电机，再关闭两条 CAN 总线。
void DmMotorDriver::close() noexcept
{
  // close() 可以重复调用，未打开时直接返回。
  if (!opened_.load()) {return;}
  disableAll();
  opened_.store(false);
  buses_[0].close();
  buses_[1].close();
}
