/*! @file dm1_mit_interface.hpp
 *  @brief DM1 真机 MIT 协议的校准、安全和发送边界。
 *
 *  该接口不依赖具体 CAN 驱动。驱动只需实现 MitTransport，即可把控制核心
 *  产生的统一 JointCommand 映射到真实 DM1 电机；所有 12 个电机的 ID、方向
 *  和零位必须在启动时显式提供，禁止默认直连。
 */
#ifndef MYMIT_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_
#define MYMIT_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "model/robot_types.hpp"

namespace dm1_hardware
{

namespace mit_protocol
{

// DM MIT wire-format ranges shared by pre-enable validation and the CAN driver.
inline constexpr float kPositionMax = 12.566F;
inline constexpr float kVelocityMax = 20.0F;
inline constexpr float kTorqueMax = 120.0F;
inline constexpr float kKpMax = 500.0F;
inline constexpr float kKdMax = 5.0F;

}  // namespace mit_protocol

// ---------- 电机地址与标定数据 ----------

// 单个 DM1 电机在 CAN 总线上的寻址信息。
struct MotorAddress
{
  std::uint8_t bus = 0;        // CAN 总线编号，目前允许使用 0 或 1。
  std::uint16_t can_id = 0;    // 电机物理 CAN ID，范围为 [1, 0x0f]。
  std::uint16_t master_id = 0; // DM MIT 协议使用的主站/仲裁 ID，范围为 [1, 0x7ff]。
};

// 单个电机的地址、方向和零位标定参数。
struct MotorCalibration
{
  MotorAddress address{};      // 电机的 CAN 寻址信息。
  int direction = 1;           // 模型坐标到电机坐标的方向，必须为 +1 或 -1。
  float zero_position = 0.0F;  // 电机原始位置对应的模型零位，rad。
};

// 从 DM1 驱动器接收的一帧电机反馈。
struct MotorFeedback
{
  std::uint8_t bus = 0;             // 反馈帧所在的 CAN 总线编号。
  std::uint16_t can_id = 0;         // 反馈帧中的电机物理 CAN ID。
  float position = 0.0F;            // 电机原始位置，rad。
  float velocity = 0.0F;            // 电机原始速度，rad/s。
  float torque = 0.0F;              // 电机原始力矩，N*m。
  float temperature_c = 0.0F;      // 定子/驱动器温度，摄氏度。
  float voltage_v = 0.0F;           // 总线电压，V；仅在 voltage_valid 时参与校验。
  std::uint32_t fault_code = 0;     // 驱动器故障码，0 表示未报告故障。
  double timestamp = 0.0;           // 反馈采集时间戳，s。
  std::uint64_t sequence = 0;       // 递增反馈序号，用于检测旧帧或重复帧。
  bool health_valid = false;        // 驱动器健康状态是否有效。
  // DM MIT 反馈同时提供两种温度，但不一定提供总线电压字段。
  float rotor_temperature_c = 0.0F; // 转子温度，摄氏度。
  bool voltage_valid = false;       // 当前反馈是否包含可用的总线电压。
};

// 待发送的单帧 DM1 MIT 控制数据。
struct MitFrame
{
  std::uint8_t bus = 0;        // 目标 CAN 总线编号。
  std::uint16_t can_id = 0;    // 目标电机物理 CAN ID。
  std::uint16_t master_id = 0; // 目标电机使用的主站/仲裁 ID。
  float position = 0.0F;       // 转换到电机坐标后的期望位置，rad。
  float velocity = 0.0F;       // 转换到电机坐标后的期望速度，rad/s。
  float kp = 0.0F;             // MIT 位置比例增益。
  float kd = 0.0F;             // MIT 速度微分增益。
  float torque = 0.0F;         // 转换到电机坐标后的前馈力矩，N*m。
};

// MIT 帧发送抽象层。
// 具体 CAN/串口驱动实现发送和全电机关闭，接口本身不持有底层资源。
class MitTransport
{
public:
  virtual ~MitTransport() = default;
  // 发送一帧 MIT 控制命令；返回 false 表示底层发送失败。
  virtual bool sendMit(const MitFrame & frame) = 0;
  // 关闭底层已知的全部电机输出，且不得抛出异常。
  virtual void disableAll() noexcept = 0;
};

// ---------- DM1 真机 MIT 接口 ----------

/**
 * @brief 将统一关节命令转换为 DM1 MIT 帧，并在异常时关闭全部输出。
 *
 * 反馈必须包含 12 个唯一电机 ID，且在超时时间内同时有效；温度、序号、
 * 驱动器故障码以及位置/速度/力矩中的非有限值都会触发安全停机。只有当
 * 底层协议明确提供电压时才校验电压。默认只允许 30 N*m 连续力矩，97 N*m
 * 峰值力矩必须由调用方显式启用。
 */
class Dm1MitInterface
{
public:
  // 标定、反馈和命令数组的固定布局均由机器人关节数量定义。
  using CalibrationArray = std::array<MotorCalibration, kNumJoints>;  // 12 个电机的标定。
  using FeedbackArray = std::array<MotorFeedback, kNumJoints>;        // 12 个电机的反馈。
  using CommandArray = std::array<JointCommand<float>, kNumLegs>;     // 四条腿的统一命令。

  // 构造接口并校验超时参数、CAN 地址、方向和地址唯一性。
  Dm1MitInterface(
    MitTransport & transport, const CalibrationArray & calibration,
    double feedback_timeout_s = 0.05, bool allow_peak_torque = false)
  : transport_(transport), calibration_(calibration),
    feedback_timeout_s_(feedback_timeout_s), allow_peak_torque_(allow_peak_torque)
  {
    // 反馈超时必须是有限的正数，否则无法进行安全的时间检查。
    if (!std::isfinite(feedback_timeout_s_) || feedback_timeout_s_ <= 0.0) {
      throw std::invalid_argument("DM1 feedback timeout must be positive and finite");
    }

    // 逐个校验电机地址和方向，并检查同一总线上的地址不能重复。
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto & address = calibration_[i].address;
      if (address.bus > 1 || address.can_id == 0 || address.can_id > 0x0F ||
        address.master_id == 0 || address.master_id > 0x7FF)
      {
        throw std::invalid_argument(
                "DM1 physical CAN ID must be in [1,0x0f] and master ID in [1,0x7ff]");
      }
      if (calibration_[i].direction != 1 && calibration_[i].direction != -1) {
        throw std::invalid_argument("DM1 motor direction must be +1 or -1");
      }
      if (!std::isfinite(calibration_[i].zero_position)) {
        throw std::invalid_argument("DM1 motor zero position must be finite");
      }
      for (std::size_t j = 0; j < i; ++j) {
        const auto & previous = calibration_[j].address;
        if (address.bus == previous.bus &&
          (address.can_id == previous.can_id || address.master_id == previous.master_id))
        {
          throw std::invalid_argument("DM1 motor bus/CAN/master mappings must be unique");
        }
      }
    }
  }

  bool updateFeedback(const FeedbackArray & feedback, double now_s) noexcept
  {
    // 当前时间无效时，立即使反馈失效并关闭所有电机输出。
    if (!std::isfinite(now_s)) {return invalidate();}
    std::array<bool, kNumJoints> seen{};

    // 第一遍只做完整性和安全校验，避免把部分有效的反馈写入缓存。
    for (const auto & sample : feedback) {
      const auto index = indexForAddress(sample.bus, sample.can_id);
      if (index >= kNumJoints || seen[index] ||
        !std::isfinite(sample.position) || !std::isfinite(sample.velocity) ||
        !std::isfinite(sample.torque) || !std::isfinite(sample.temperature_c) ||
        !std::isfinite(sample.rotor_temperature_c) || !std::isfinite(sample.timestamp) ||
        sample.timestamp > now_s || now_s - sample.timestamp > feedback_timeout_s_ ||
        sample.sequence == 0 ||
        (feedback_sequences_[index] != 0 && sample.sequence < feedback_sequences_[index]) ||
        !sample.health_valid || sample.temperature_c<-20.0F || sample.temperature_c>100.0F ||
        sample.rotor_temperature_c<-20.0F || sample.rotor_temperature_c>100.0F ||
        (sample.voltage_valid &&
        (!std::isfinite(sample.voltage_v) || sample.voltage_v<18.0F || sample.voltage_v>60.0F)) ||
        sample.fault_code != 0)
      {
        return invalidate();
      }
      seen[index] = true;
    }

    // 第二遍保存反馈，并将位置、速度和力矩转换到模型坐标系。
    for (const auto & sample : feedback) {
      const auto index = indexForAddress(sample.bus, sample.can_id);
      feedback_[index] = sample;
      feedback_sequences_[index] = sample.sequence;
      feedback_[index].position = calibration_[index].direction *
        (sample.position - calibration_[index].zero_position);
      feedback_[index].velocity = calibration_[index].direction * sample.velocity;
      feedback_[index].torque = calibration_[index].direction * sample.torque;
    }
    feedback_valid_ = true;
    latest_feedback_time_ = now_s;
    return true;
  }

  /** @brief 返回已经转换到 DM1 模型坐标系的四腿关节反馈。 */
  std::array<JointState<float>, kNumLegs> jointStates(float timestamp) const noexcept
  {
    // 按固定的腿和关节顺序组装四条腿的统一状态。
    std::array<JointState<float>, kNumLegs> result{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      auto & state = result[leg];
      state.leg = static_cast<LegId>(leg);
      state.timestamp = timestamp;
      state.valid = feedback_valid_ && std::isfinite(timestamp);
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const auto & sample = feedback_[leg * kJointsPerLeg + joint];
        state.position[static_cast<Eigen::Index>(joint)] = sample.position;
        state.velocity[static_cast<Eigen::Index>(joint)] = sample.velocity;
        state.torque_estimate[static_cast<Eigen::Index>(joint)] = sample.torque;
      }
    }
    return result;
  }

  /** @brief 返回当前是否存在完整、有效且未超时的电机反馈。 */
  bool feedbackValid() const noexcept {return feedback_valid_;}

  /**
   * @brief Validate a complete command frame without sending or disabling.
   *
   * This is used before a physical enable so the first enabled frame cannot
   * be the operation that discovers a joint-limit or torque violation.
   */
  bool validateCommands(const CommandArray & commands, double now_s) const noexcept
  {
    if (!feedback_valid_ || !std::isfinite(now_s) || now_s < latest_feedback_time_ ||
      now_s - latest_feedback_time_ > feedback_timeout_s_)
    {
      return false;
    }

    const float torque_limit = allow_peak_torque_ ? 97.0F : 30.0F;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & command = commands[leg];
      if (command.leg != static_cast<LegId>(leg) || !command.enabled ||
        !command.position_desired.allFinite() || !command.velocity_desired.allFinite() ||
        !command.kp.allFinite() || !command.kd.allFinite() ||
        !command.torque_feedforward.allFinite() || !std::isfinite(command.timestamp) ||
        static_cast<double>(command.timestamp) > now_s + 1.0e-6 ||
        now_s - static_cast<double>(command.timestamp) > feedback_timeout_s_)
      {
        return false;
      }
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const float q = command.position_desired[static_cast<Eigen::Index>(joint)];
        const float dq = command.velocity_desired[static_cast<Eigen::Index>(joint)];
        const float kp = command.kp[static_cast<Eigen::Index>(joint)];
        const float kd = command.kd[static_cast<Eigen::Index>(joint)];
        const float tau = command.torque_feedforward[static_cast<Eigen::Index>(joint)];
        const float total_torque =
          kp * (q - feedback_[leg * kJointsPerLeg + joint].position) +
          kd * (dq - feedback_[leg * kJointsPerLeg + joint].velocity) + tau;
        const float motor_position =
          calibration_[leg * kJointsPerLeg + joint].direction * q +
          calibration_[leg * kJointsPerLeg + joint].zero_position;
        if (q < kLowerLimit[joint] || q > kUpperLimit[joint] ||
          std::abs(dq) > kVelocityLimit || kp<0.0F || kp> mit_protocol::kKpMax ||
          kd<0.0F || kd> mit_protocol::kKdMax ||
          std::abs(tau) > torque_limit ||
          motor_position<-mit_protocol::kPositionMax ||
          motor_position> mit_protocol::kPositionMax ||
          !std::isfinite(motor_position) || !std::isfinite(total_torque) ||
          std::abs(total_torque) > torque_limit)
        {
          return false;
        }
      }
    }
    return true;
  }

  // 校验反馈新鲜度和全部关节命令，通过后逐帧发送 MIT 控制数据。
  bool send(const CommandArray & commands, double now_s) noexcept
  {
    // 没有有效反馈、时间倒退或反馈过期时，禁止继续发送控制命令。
    if (!feedback_valid_ || !std::isfinite(now_s) || now_s < latest_feedback_time_ ||
      now_s - latest_feedback_time_ > feedback_timeout_s_)
    {
      transport_.disableAll();
      return false;
    }

    // 先复用无副作用校验，确保不会由首帧发送才发现协议边界错误。
    if (!validateCommands(commands, now_s)) {return fail();}

    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & command = commands[leg];
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const auto & calibration = calibration_[leg * kJointsPerLeg + joint];
        const float q = command.position_desired[static_cast<Eigen::Index>(joint)];
        const float dq = command.velocity_desired[static_cast<Eigen::Index>(joint)];
        const float kp = command.kp[static_cast<Eigen::Index>(joint)];
        const float kd = command.kd[static_cast<Eigen::Index>(joint)];
        const float tau = command.torque_feedforward[static_cast<Eigen::Index>(joint)];

        // 将模型坐标命令按方向和零位转换为电机物理坐标。
        MitFrame frame;
        frame.bus = calibration.address.bus;
        frame.can_id = calibration.address.can_id;
        frame.master_id = calibration.address.master_id;
        frame.position = calibration.direction * q + calibration.zero_position;
        frame.velocity = calibration.direction * dq;
        frame.kp = kp;
        frame.kd = kd;
        frame.torque = calibration.direction * tau;

        // 任意一帧发送失败都进入失效状态并关闭全部电机。
        if (!transport_.sendMit(frame)) {
          return fail();
        }
      }
    }
    return true;
  }

  // 立即标记反馈无效并关闭所有电机输出。
  void disable() noexcept {feedback_valid_ = false; transport_.disableAll();}

private:
  // 每条腿三个关节的模型位置下限，单位为 rad。
  static constexpr std::array<float, kJointsPerLeg> kLowerLimit{
    -1.57F, -1.367F, -0.03F};

  // 每条腿三个关节的模型位置上限，单位为 rad。
  static constexpr std::array<float, kJointsPerLeg> kUpperLimit{
    1.57F, 2.603F, 2.72F};

  // 关节速度绝对值上限，单位为 rad/s。
  static constexpr float kVelocityLimit = 4.1887902F;

  // 根据总线和物理 CAN ID 查找标定数组中的关节索引。
  // 找不到时返回 kNumJoints，供调用方统一判定为无效地址。
  std::size_t indexForAddress(std::uint8_t bus, std::uint16_t can_id) const noexcept
  {
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto & address = calibration_[i].address;
      if (address.bus == bus && address.can_id == can_id) {return i;}
    }
    return kNumJoints;
  }

  // 反馈校验失败时清空有效标志，并以故障闭锁方式关闭所有电机。
  bool invalidate() noexcept
  {
    feedback_valid_ = false;
    transport_.disableAll();
    return false;
  }

  // 命令校验或发送失败时清空有效标志，并关闭所有电机。
  bool fail() noexcept
  {
    feedback_valid_ = false;
    transport_.disableAll();
    return false;
  }

  MitTransport & transport_;  // 非拥有的 MIT 传输层引用。
  CalibrationArray calibration_;  // 12 个电机的地址、方向和零位标定。
  FeedbackArray feedback_{};  // 最近一次通过校验的电机反馈。
  std::array<std::uint64_t, kNumJoints> feedback_sequences_{};  // 每个电机最近的反馈序号。
  double feedback_timeout_s_;  // 反馈允许的最大时间间隔，s。
  double latest_feedback_time_ = 0.0;  // 最近一次完整有效反馈的时间戳，s。
  bool allow_peak_torque_ = false;  // 是否允许使用 97 N*m 峰值力矩限制。
  bool feedback_valid_ = false;  // 当前反馈是否完整、有效且未超时。
};

}  // namespace dm1_hardware

#endif  // MYMIT_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_
