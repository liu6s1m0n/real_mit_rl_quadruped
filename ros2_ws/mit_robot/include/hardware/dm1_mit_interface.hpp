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

struct MotorAddress
{
  std::uint8_t bus = 0;
  std::uint16_t can_id = 0;
  std::uint16_t master_id = 0;
};

struct MotorCalibration
{
  MotorAddress address{};
  int direction = 1;
  float zero_position = 0.0F;
};

struct MotorFeedback
{
  std::uint8_t bus = 0;
  std::uint16_t can_id = 0;
  float position = 0.0F;
  float velocity = 0.0F;
  float torque = 0.0F;
  float temperature_c = 0.0F;
  float voltage_v = 0.0F;
  std::uint32_t fault_code = 0;
  double timestamp = 0.0;
  std::uint64_t sequence = 0;
  bool health_valid = false;
  // DM MIT feedback exposes both temperatures, but not a bus-voltage field.
  float rotor_temperature_c = 0.0F;
  bool voltage_valid = false;
};

struct MitFrame
{
  std::uint8_t bus = 0;
  std::uint16_t can_id = 0;
  std::uint16_t master_id = 0;
  float position = 0.0F;
  float velocity = 0.0F;
  float kp = 0.0F;
  float kd = 0.0F;
  float torque = 0.0F;
};

class MitTransport
{
public:
  virtual ~MitTransport() = default;
  virtual bool sendMit(const MitFrame & frame) = 0;
  virtual void disableAll() noexcept = 0;
};

/**
 * @brief 将统一命令转换为 DM1 MIT 帧，并在异常时关闭全部输出。
 *
 * 反馈必须包含 12 个唯一电机 ID，且在 timeout 内同时有效；温度、序号、
 * 驱动器 fault code、位置/速度/力矩非有限值都会触发安全停机。电压只有在
 * 底层协议明确提供时才校验。默认只允许 30 Nm 连续力矩，97 Nm 峰值必须由
 * 调用方显式启用。
 */
class Dm1MitInterface
{
public:
  using CalibrationArray = std::array<MotorCalibration, kNumJoints>;
  using FeedbackArray = std::array<MotorFeedback, kNumJoints>;
  using CommandArray = std::array<JointCommand<float>, kNumLegs>;

  Dm1MitInterface(
    MitTransport & transport, const CalibrationArray & calibration,
    double feedback_timeout_s = 0.05, bool allow_peak_torque = false)
  : transport_(transport), calibration_(calibration),
    feedback_timeout_s_(feedback_timeout_s), allow_peak_torque_(allow_peak_torque)
  {
    if (!std::isfinite(feedback_timeout_s_) || feedback_timeout_s_ <= 0.0) {
      throw std::invalid_argument("DM1 feedback timeout must be positive and finite");
    }
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
    if (!std::isfinite(now_s)) {return invalidate();}
    std::array<bool, kNumJoints> seen{};
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

  /** @brief 返回已转换到 DM1 模型坐标的四腿反馈。 */
  std::array<JointState<float>, kNumLegs> jointStates(float timestamp) const noexcept
  {
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

  bool feedbackValid() const noexcept {return feedback_valid_;}

  bool send(const CommandArray & commands, double now_s) noexcept
  {
    if (!feedback_valid_ || !std::isfinite(now_s) || now_s < latest_feedback_time_ ||
      now_s - latest_feedback_time_ > feedback_timeout_s_)
    {
      transport_.disableAll();
      return false;
    }
    const float torque_limit = allow_peak_torque_ ? 97.0F : 30.0F;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & command = commands[leg];
      if (command.leg != static_cast<LegId>(leg) || !command.enabled ||
        !command.position_desired.allFinite() || !command.velocity_desired.allFinite() ||
        !command.kp.allFinite() || !command.kd.allFinite() ||
        !command.torque_feedforward.allFinite() || !std::isfinite(command.timestamp) ||
        static_cast<double>(command.timestamp) > now_s ||
        now_s - static_cast<double>(command.timestamp) > feedback_timeout_s_)
      {
        return fail();
      }
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const auto & calibration = calibration_[leg * kJointsPerLeg + joint];
        const float q = command.position_desired[static_cast<Eigen::Index>(joint)];
        const float dq = command.velocity_desired[static_cast<Eigen::Index>(joint)];
        const float kp = command.kp[static_cast<Eigen::Index>(joint)];
        const float kd = command.kd[static_cast<Eigen::Index>(joint)];
        const float tau = command.torque_feedforward[static_cast<Eigen::Index>(joint)];
        const float total_torque = kp * (q - feedback_[leg * kJointsPerLeg + joint].position) +
          kd * (dq - feedback_[leg * kJointsPerLeg + joint].velocity) + tau;
        if (q < kLowerLimit[joint] || q > kUpperLimit[joint] ||
          std::abs(dq) > kVelocityLimit || kp<0.0F || kd<0.0F ||
          std::abs(tau)> torque_limit || !std::isfinite(total_torque) ||
          std::abs(total_torque)> torque_limit)
        {
          return fail();
        }
        MitFrame frame;
        frame.bus = calibration.address.bus;
        frame.can_id = calibration.address.can_id;
        frame.master_id = calibration.address.master_id;
        frame.position = calibration.direction * q + calibration.zero_position;
        frame.velocity = calibration.direction * dq;
        frame.kp = kp;
        frame.kd = kd;
        frame.torque = calibration.direction * tau;
        if (!transport_.sendMit(frame)) {
          return fail();
        }
      }
    }
    return true;
  }

  void disable() noexcept {feedback_valid_ = false; transport_.disableAll();}

private:
  static constexpr std::array<float, kJointsPerLeg> kLowerLimit{
    -1.57F, -1.367F, -0.03F};
  static constexpr std::array<float, kJointsPerLeg> kUpperLimit{
    1.57F, 2.603F, 2.72F};
  static constexpr float kVelocityLimit = 4.1887902F;

  std::size_t indexForAddress(std::uint8_t bus, std::uint16_t can_id) const noexcept
  {
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto & address = calibration_[i].address;
      if (address.bus == bus && address.can_id == can_id) {return i;}
    }
    return kNumJoints;
  }

  bool invalidate() noexcept
  {
    feedback_valid_ = false;
    transport_.disableAll();
    return false;
  }

  bool fail() noexcept
  {
    feedback_valid_ = false;
    transport_.disableAll();
    return false;
  }

  MitTransport & transport_;
  CalibrationArray calibration_;
  FeedbackArray feedback_{};
  std::array<std::uint64_t, kNumJoints> feedback_sequences_{};
  double feedback_timeout_s_;
  double latest_feedback_time_ = 0.0;
  bool allow_peak_torque_ = false;
  bool feedback_valid_ = false;
};

}  // namespace dm1_hardware

#endif  // MYMIT_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_
