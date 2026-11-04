/*! @file dm1_mit_interface.hpp
 *  @brief DM1 真机 MIT 协议的校准、安全和发送边界。
 *
 *  该接口不依赖具体 CAN 驱动：它把控制核心产生的统一 JointCommand 校验并
 *  转换为 MIT 帧，由实现 MitTransport 的驱动发送；所有 12 个电机的 ID、方向
 *  和零位必须在启动时显式提供，禁止默认直连。
 */
#ifndef DM1_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_
#define DM1_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "common/console_log.hpp"
#include "common/robot_types.hpp"

namespace dm1_hardware
{

namespace mit_protocol
{

/// DM MIT 协议编码范围，使能前校验和 CAN 驱动编码共用。
inline constexpr float kPositionMax = 12.566F;  ///< 位置编码范围 ±P_MAX，rad。
inline constexpr float kVelocityMax = 20.0F;    ///< 速度编码范围 ±V_MAX，rad/s。
inline constexpr float kTorqueMax = 120.0F;     ///< 力矩编码范围 ±T_MAX，N*m。
inline constexpr float kKpMax = 500.0F;         ///< kp 编码上限。
inline constexpr float kKdMax = 5.0F;           ///< kd 编码上限。

}  // namespace mit_protocol

// ---------- 电机地址与标定数据 ----------

/// 单个 DM1 电机在 CAN 总线上的寻址信息。
struct MotorAddress
{
  std::uint8_t bus = 0;         ///< CAN 总线编号，目前允许使用 0 或 1。
  std::uint16_t can_id = 0;     ///< 电机物理 CAN ID，范围为 [1, 0x0f]。
  std::uint16_t master_id = 0;  ///< DM MIT 协议使用的主站/仲裁 ID，范围为 [1, 0x7ff]。
};

/// 单个电机的地址、方向和零位标定参数。
struct MotorCalibration
{
  MotorAddress address{};      ///< 电机的 CAN 寻址信息。
  int direction = 1;           ///< 模型坐标到电机坐标的方向，必须为 +1 或 -1。
  float zero_position = 0.0F;  ///< 电机原始位置对应的模型零位，rad。
};

/// 从 DM1 驱动器接收的一帧电机反馈。
struct MotorFeedback
{
  std::uint8_t bus = 0;          ///< 反馈帧所在的 CAN 总线编号。
  std::uint16_t can_id = 0;      ///< 反馈帧中的电机物理 CAN ID。
  float position = 0.0F;         ///< 电机原始位置，rad。
  float velocity = 0.0F;         ///< 电机原始速度，rad/s。
  float torque = 0.0F;           ///< 电机原始力矩，N*m。
  float temperature_c = 0.0F;    ///< 定子/驱动器温度，摄氏度。
  float voltage_v = 0.0F;        ///< 总线电压，V；仅在 voltage_valid 时参与校验。
  std::uint32_t fault_code = 0;  ///< 驱动器故障码，0 表示未报告故障。
  double timestamp = 0.0;        ///< 反馈采集时间戳，s。
  std::uint64_t sequence = 0;    ///< 递增反馈序号，用于检测旧帧或重复帧。
  bool health_valid = false;     ///< 驱动器健康状态是否有效。
  /// DM MIT 反馈同时提供两种温度，但不一定提供总线电压字段。
  float rotor_temperature_c = 0.0F;  ///< 转子温度，摄氏度。
  bool voltage_valid = false;        ///< 当前反馈是否包含可用的总线电压。
};

/// 待发送的单帧 DM1 MIT 控制数据。
struct MitFrame
{
  std::uint8_t bus = 0;         ///< 目标 CAN 总线编号。
  std::uint16_t can_id = 0;     ///< 目标电机物理 CAN ID。
  std::uint16_t master_id = 0;  ///< 目标电机使用的主站/仲裁 ID。
  float position = 0.0F;        ///< 转换到电机坐标后的期望位置，rad。
  float velocity = 0.0F;        ///< 转换到电机坐标后的期望速度，rad/s。
  float kp = 0.0F;              ///< MIT 位置比例增益。
  float kd = 0.0F;              ///< MIT 速度微分增益。
  float torque = 0.0F;          ///< 转换到电机坐标后的前馈力矩，N*m。
};

/**
 * @brief MIT 帧发送抽象层。
 *
 * 具体 CAN/串口驱动实现发送和全电机关闭，接口本身不持有底层资源。
 */
class MitTransport
{
public:
  virtual ~MitTransport() = default;
  /// 发送一帧 MIT 控制命令；返回 false 表示底层发送失败。
  virtual bool sendMit(const MitFrame & frame) = 0;
  /// 关闭底层已知的全部电机输出，且不得抛出异常。
  virtual void disableAll() noexcept = 0;
};

/// 校验 12 个电机的地址、方向和零位；标定文件加载后调用一次。
inline void validateCalibration(const std::array<MotorCalibration, kNumJoints> & calibration)
{
  for (std::size_t i = 0; i < calibration.size(); ++i) {
    const auto & address = calibration[i].address;
    if (address.bus > 1 || address.can_id == 0 || address.can_id > 0x0F ||
      address.master_id == 0 || address.master_id > 0x7FF)
    {
      throw std::invalid_argument(
              "DM1 bus must be 0/1, physical CAN ID in [1,0x0f], master ID in [1,0x7ff]");
    }
    if (calibration[i].direction != 1 && calibration[i].direction != -1) {
      throw std::invalid_argument("DM1 motor direction must be +1 or -1");
    }
    if (!std::isfinite(calibration[i].zero_position)) {
      throw std::invalid_argument("DM1 motor zero position must be finite");
    }
    for (std::size_t j = 0; j < i; ++j) {
      const auto & previous = calibration[j].address;
      if (address.bus == previous.bus &&
        (address.can_id == previous.can_id || address.master_id == previous.master_id))
      {
        throw std::invalid_argument("DM1 motor bus/CAN/master mappings must be unique");
      }
    }
  }
}

// ---------- DM1 真机 MIT 接口 ----------

/**
 * @brief 将统一关节命令转换为 DM1 MIT 帧，并拒绝不安全的输入。
 *
 * 反馈必须包含 12 个唯一电机地址；序号回退、健康标志、温度、电压或驱动器
 * 故障码异常时拒绝该反馈，是否失能由上层硬件桥决定。超过力矩上限的关节
 * 在发送帧里按比例缩小 kp/kd/前馈，不丢弃整批命令。
 */
class Dm1MitInterface
{
public:
  /// 标定、反馈和命令数组的固定布局均由机器人关节数量定义。
  using CalibrationArray = std::array<MotorCalibration, kNumJoints>;  ///< 12 个电机的标定。
  using FeedbackArray = std::array<MotorFeedback, kNumJoints>;        ///< 12 个电机的反馈。
  using CommandArray = std::array<JointCommand<float>, kNumLegs>;     ///< 四条腿的统一命令。
  using FrameArray = std::array<MitFrame, kNumJoints>;                ///< 一批 12 帧 MIT 命令。

  /// calibration 须已通过 validateCalibration()；torque_limit_nm 为每关节总力矩上限。
  explicit Dm1MitInterface(
    const CalibrationArray & calibration, float torque_limit_nm = 30.0F)
  : calibration_(calibration), torque_limit_(torque_limit_nm)
  {}

  /**
   * @brief 校验并缓存一整批 12 个电机反馈。
   *
   * 任一电机地址未知/重复、序号为 0 或回退、健康标志无效、温度/电压超范围或
   * 故障码非 0 时整批拒绝并把反馈标为无效，返回 false。通过后转换到模型坐标系。
   */
  bool updateFeedback(const FeedbackArray & feedback) noexcept
  {
    std::array<bool, kNumJoints> seen{};

    // 第一遍只做完整性和安全校验，避免把部分有效的反馈写入缓存。
    for (std::size_t sample_index = 0; sample_index < feedback.size(); ++sample_index) {
      const auto & sample = feedback[sample_index];
      const auto index = indexForAddress(sample.bus, sample.can_id);
      const auto previous_sequence = index < kNumJoints ? feedback_sequences_[index] : 0;
      const char * reason = nullptr;
      if (index >= kNumJoints) {reason = "未知电机地址";} else if (seen[index]) {
        reason = "重复电机地址";
      } else if (sample.sequence == 0) {
        reason = "sequence 为 0";
      } else if (previous_sequence != 0 && sample.sequence < previous_sequence) {
        reason = "sequence 回退";
      } else if (!sample.health_valid) {reason = "health_valid=false";}
      // 100 °C 只是警告区，DM 驱动器本身在更高温度才降额/保护；软件上限放到
      // 120 °C，避免刚跑热一点就整车失能。
      else if (sample.temperature_c < -20.0F || sample.temperature_c > 120.0F) {
        reason = "temperature 超范围";
      } else if (sample.rotor_temperature_c < -20.0F ||
        sample.rotor_temperature_c > 120.0F)
      {
        reason = "rotor_temperature 超范围";
      } else if (sample.voltage_valid && (sample.voltage_v < 18.0F || sample.voltage_v > 60.0F)) {
        reason = "voltage 超范围";
      } else if (sample.fault_code != 0) {reason = "fault_code 非 0";}

      if (reason != nullptr) {
        // 500 Hz 热路径：按秒节流，避免日志本身拖慢控制循环并放大 CAN 压力。
        static robot_log::Throttle feedback_failure_log(1000);
        if (feedback_failure_log.ready()) {
          imu_log::print(
            imu_log::Level::Warning,
            "DM1 反馈校验失败：reason=%s sample=%zu joint=%zu bus=%u can_id=0x%02x "
            "sequence=%llu previous=%llu fault_code=0x%08x health_valid=%s。\n",
            reason, sample_index, index, static_cast<unsigned int>(sample.bus),
            static_cast<unsigned int>(sample.can_id),
            static_cast<unsigned long long>(sample.sequence),
            static_cast<unsigned long long>(previous_sequence),
            static_cast<unsigned int>(sample.fault_code),
            sample.health_valid ? "true" : "false");
        }
        feedback_valid_ = false;
        return false;
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
    return true;
  }

  /** @brief 返回已经转换到 DM1 模型坐标系的四腿关节反馈。 */
  std::array<JointState<float>, kNumLegs> jointStates(float timestamp) const noexcept
  {
    std::array<JointState<float>, kNumLegs> result{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      auto & state = result[leg];
      state.leg = static_cast<LegId>(leg);
      state.timestamp = timestamp;
      state.valid = feedback_valid_;
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const auto & sample = feedback_[leg * kJointsPerLeg + joint];
        state.position[static_cast<Eigen::Index>(joint)] = sample.position;
        state.velocity[static_cast<Eigen::Index>(joint)] = sample.velocity;
        state.torque_estimate[static_cast<Eigen::Index>(joint)] = sample.torque;
      }
    }
    return result;
  }

  /** @brief 返回当前是否存在完整、有效的电机反馈。 */
  bool feedbackValid() const noexcept {return feedback_valid_;}

  /**
   * @brief 校验一整批命令的协议字段，不发送。
   *
   * 关节目标先按模型限位限幅；非有限值、超出协议范围的速度/增益/电机位置
   * 会拒绝整批。力矩超限只记录，发送时在 prepareFrames() 中按比例截断。
   */
  bool validateCommands(const CommandArray & commands) const noexcept
  {
    const auto reject = [] (
      std::size_t leg, std::size_t joint, const char * field,
      double actual, double lower, double upper) noexcept->bool {
      // 持续越界时不要每帧刷屏；按秒给一条足够定位即可。
      static robot_log::Throttle reject_log(1000);
      if (reject_log.ready()) {
        imu_log::print(
          imu_log::Level::Warning,
          "DM1 MIT command rejected: leg=%zu joint=%zu field=%s "
          "actual=%.9g allowed=[%.9g,%.9g]\n",
          leg, joint, field, actual, lower, upper);
      }
      return false;
    };

    if (!feedback_valid_) {return reject(kNumLegs, kJointsPerLeg, "feedback_valid", 0, 1, 1);}
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & command = commands[leg];
      if (!command.enabled) {return reject(leg, kJointsPerLeg, "enabled", 0, 1, 1);}
      // 优化器发散时会产生 NaN/Inf，不能进入限幅和编码。
      if (!command.position_desired.allFinite() || !command.velocity_desired.allFinite() ||
        !command.kp.allFinite() || !command.kd.allFinite() ||
        !command.torque_feedforward.allFinite())
      {
        return reject(leg, kJointsPerLeg, "non-finite", 1, 0, 0);
      }
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const auto i = static_cast<Eigen::Index>(joint);
        const float requested_q = command.position_desired[i];
        const float q = std::clamp(requested_q, kLowerLimit[joint], kUpperLimit[joint]);
        const float dq = command.velocity_desired[i];
        const float kp = command.kp[i];
        const float kd = command.kd[i];
        const float tau = command.torque_feedforward[i];
        const auto & calibration = calibration_[leg * kJointsPerLeg + joint];
        const float motor_position = calibration.direction * q + calibration.zero_position;
        if (q != requested_q) {
          static robot_log::Throttle position_limit_log(1000);
          if (position_limit_log.ready()) {
            imu_log::print(
              imu_log::Level::Warning,
              "DM1 关节目标已限幅：leg=%zu joint=%zu requested=%.6f bounded=%.6f；"
              "完整命令继续发送。\n", leg, joint,
              static_cast<double>(requested_q), static_cast<double>(q));
          }
        }
        if (std::abs(dq) > kVelocityLimit) {
          return reject(leg, joint, "velocity_desired", dq, -kVelocityLimit, kVelocityLimit);
        }
        if (kp < 0.0F || kp > mit_protocol::kKpMax) {
          return reject(leg, joint, "kp", kp, 0.0, mit_protocol::kKpMax);
        }
        if (kd < 0.0F || kd > mit_protocol::kKdMax) {
          return reject(leg, joint, "kd", kd, 0.0, mit_protocol::kKdMax);
        }
        if (std::abs(motor_position) > mit_protocol::kPositionMax) {
          return reject(
            leg, joint, "motor_position", motor_position,
            -mit_protocol::kPositionMax, mit_protocol::kPositionMax);
        }
        if (std::abs(tau) > mit_protocol::kTorqueMax) {
          logTorqueClamped(
            "torque_feedforward_protocol", leg, joint, tau, mit_protocol::kTorqueMax,
            q, dq, kp, kd, tau);
        }
        const float total_torque = totalTorque(leg, joint, q, dq, kp, kd, tau);
        if (std::abs(total_torque) > torque_limit_) {
          logTorqueClamped(
            "total_torque", leg, joint, total_torque, torque_limit_, q, dq, kp, kd, tau);
        }
      }
    }
    return true;
  }

  /// 在控制线程完成校验和力矩饱和；独立发送线程只接收完整的不可变帧批次。
  bool prepareFrames(const CommandArray & commands, FrameArray & frames) const noexcept
  {
    if (!validateCommands(commands)) {return false;}

    // 两条 CAN 共用同一个双通道 USB 适配器。按关节交错发送前后腿，避免先向
    // can0 连续灌入六帧、再向 can1 连续灌入六帧形成单通道突发。
    static constexpr std::array<std::size_t, kNumLegs> kInterleavedLegOrder{0, 2, 1, 3};
    std::size_t frame_index = 0;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      for (const std::size_t leg : kInterleavedLegOrder) {
        const auto & command = commands[leg];
        const auto & calibration = calibration_[leg * kJointsPerLeg + joint];
        const auto i = static_cast<Eigen::Index>(joint);
        // 单个目标越界只在硬件边界限幅，不阻断其他十一关节的新命令。
        const float q = std::clamp(
          command.position_desired[i], kLowerLimit[joint], kUpperLimit[joint]);
        const float dq = command.velocity_desired[i];
        const float kp = command.kp[i];
        const float kd = command.kd[i];
        const float tau = command.torque_feedforward[i];

        // 超限时把 kp/kd/前馈按同一比例缩小，使预计总力矩和协议 torque 字段都在范围内。
        float scale = 1.0F;
        const float total_torque = totalTorque(leg, joint, q, dq, kp, kd, tau);
        if (std::abs(total_torque) > torque_limit_) {
          scale = torque_limit_ / std::abs(total_torque);
        }
        if (std::abs(tau) > mit_protocol::kTorqueMax) {
          scale = std::min(scale, mit_protocol::kTorqueMax / std::abs(tau));
        }

        // 将模型坐标命令按方向和零位转换为电机物理坐标。
        MitFrame frame;
        frame.bus = calibration.address.bus;
        frame.can_id = calibration.address.can_id;
        frame.master_id = calibration.address.master_id;
        frame.position = calibration.direction * q + calibration.zero_position;
        frame.velocity = calibration.direction * dq;
        frame.kp = kp * scale;
        frame.kd = kd * scale;
        // 缩放因子的浮点舍入可能让结果比上限大 1 ULP，这里显式夹到协议编码范围。
        frame.torque = std::clamp(
          calibration.direction * tau * scale,
          -mit_protocol::kTorqueMax, mit_protocol::kTorqueMax);
        frames[frame_index++] = frame;
      }
    }
    return true;
  }

private:
  /// 每条腿三个关节的模型位置下限，单位为 rad。
  static constexpr std::array<float, kJointsPerLeg> kLowerLimit{
    -1.57F, -1.57F, -2.25F};

  /// 每条腿三个关节的模型位置上限，单位为 rad。
  static constexpr std::array<float, kJointsPerLeg> kUpperLimit{
    1.57F, 2.4F, 0.0F};

  /// 关节速度绝对值上限，单位为 rad/s。
  /// 电机额定 40 rpm，峰值/空载 60 rpm；指令速度只是前馈目标，电机内部还有
  /// 自己的限速，所以这里放宽到空载 60 rpm，避免爬行/摆动起步时被误判。
  static constexpr float kVelocityLimit = 6.2831853F;

  /// 以当前反馈估算该关节的 MIT-PD 总力矩。
  float totalTorque(
    std::size_t leg, std::size_t joint, float q, float dq, float kp, float kd,
    float tau) const noexcept
  {
    const auto & feedback = feedback_[leg * kJointsPerLeg + joint];
    return kp * (q - feedback.position) + kd * (dq - feedback.velocity) + tau;
  }

  /// 根据总线和物理 CAN ID 查找标定数组中的关节索引。
  /// 找不到时返回 kNumJoints，供调用方统一判定为无效地址。
  std::size_t indexForAddress(std::uint8_t bus, std::uint16_t can_id) const noexcept
  {
    for (std::size_t i = 0; i < calibration_.size(); ++i) {
      const auto & address = calibration_[i].address;
      if (address.bus == bus && address.can_id == can_id) {return i;}
    }
    return kNumJoints;
  }

  /// 力矩超限时只记录并在发送帧中截断；每 500 次提示一次，避免热路径刷屏。
  void logTorqueClamped(
    const char * field, std::size_t leg, std::size_t joint, float actual,
    float limit, float q, float dq, float kp, float kd, float tau) const noexcept
  {
    if ((torque_clamp_frames_++ % 500U) != 0U) {return;}
    const auto & feedback = feedback_[leg * kJointsPerLeg + joint];
    const float position_torque = kp * (q - feedback.position);
    const float velocity_torque = kd * (dq - feedback.velocity);
    imu_log::print(
      imu_log::Level::Warning,
      "DM1 MIT 力矩将截断：field=%s leg=%zu joint=%zu actual=%.4g Nm "
      "limit=%.4g Nm q_des=%.4g q=%.4g dq_des=%.4g dq=%.4g "
      "tau_pos=%.4g tau_vel=%.4g tau_ff=%.4g；该关节帧仍会发送。\n",
      field, leg, joint, static_cast<double>(actual), static_cast<double>(limit),
      static_cast<double>(q), static_cast<double>(feedback.position),
      static_cast<double>(dq), static_cast<double>(feedback.velocity),
      static_cast<double>(position_torque), static_cast<double>(velocity_torque),
      static_cast<double>(tau));
  }

  CalibrationArray calibration_;                                ///< 12 个电机的地址、方向和零位标定。
  float torque_limit_;                                          ///< 每关节总力矩上限，N*m。
  FeedbackArray feedback_{};                                    ///< 最近一次通过校验的电机反馈。
  std::array<std::uint64_t, kNumJoints> feedback_sequences_{};  ///< 每个电机最近的反馈序号。
  bool feedback_valid_ = false;                                 ///< 当前反馈是否完整、有效。
  mutable std::uint64_t torque_clamp_frames_ = 0;               ///< 力矩截断提示节流计数。
};

}  // namespace dm1_hardware

#endif  // DM1_ROBOT_HARDWARE_DM1_MIT_INTERFACE_HPP_
