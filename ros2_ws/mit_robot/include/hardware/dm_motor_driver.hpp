#ifndef MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_
#define MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dmbot_serial/protocol/socketcan.h"
#include "hardware/dm1_mit_interface.hpp"

// ============================================================
// DM1 双 CAN 总线电机驱动适配器。
// 接收回调只更新加锁保护的最新状态快照；控制线程不等待 CAN 帧，
// 总线和电机物理 ID 的查找均通过显式标定完成。
// ============================================================
class DmMotorDriver final
{
public:
  // DM1 电机反馈中的状态码。0 和 1 表示正常，其他值表示故障。
  enum class MotorStatus : std::uint8_t
  {
    Disabled = 0,       // 电机未使能。
    Enabled = 1,        // 电机已使能。
    Fault2 = 2,         // 故障码 2。
    Fault3 = 3,         // 故障码 3。
    Fault5 = 5,         // 故障码 5。
    Fault7 = 7,         // 故障码 7。
    Fault8 = 8,         // 故障码 8。
    Fault9 = 9,         // 故障码 9。
    FaultA = 0x0A,      // 故障码 A。
    FaultB = 0x0B,      // 故障码 B。
    FaultC = 0x0C,      // 故障码 C。
    FaultD = 0x0D,      // 故障码 D。
    FaultE = 0x0E,      // 故障码 E。
    UnknownFault = 0x0F // 未知或保留故障码。
  };

  // 复用 MIT 接口中定义的标定数组和反馈数组类型。
  using CalibrationArray = dm1_hardware::Dm1MitInterface::CalibrationArray;
  using FeedbackArray = dm1_hardware::Dm1MitInterface::FeedbackArray;

  // 单帧 DM MIT 反馈解码后的原始数据，不包含标定和时间信息。
  struct DecodedFeedback
  {
    std::uint16_t motor_id = 0;        // 反馈帧中携带的物理电机 ID。
    std::uint8_t status = 0;           // 电机状态码。
    float position = 0.0F;              // 原始位置，rad。
    float velocity = 0.0F;              // 原始速度，rad/s。
    float torque = 0.0F;                // 原始力矩，N*m。
    float mos_temperature_c = 0.0F;    // MOS/驱动器温度，摄氏度。
    float rotor_temperature_c = 0.0F;  // 转子温度，摄氏度。
  };

  // 绑定两条 CAN 总线和 12 个电机的标定信息。
  DmMotorDriver(std::string can0, std::string can1, const CalibrationArray & calibration);
  // 关闭 CAN 总线并确保电机输出被禁用。
  ~DmMotorDriver();

  // 驱动器持有互斥状态和底层总线，禁止复制，避免重复管理硬件资源。
  DmMotorDriver(const DmMotorDriver &) = delete;
  DmMotorDriver & operator=(const DmMotorDriver &) = delete;

  /** @brief 打开两条 SocketCAN 总线，并清空启动前的旧反馈缓存。 */
  bool open();
  /** @brief 只打开指定电机所在的 SocketCAN 总线。 */
  bool openSingle(const dm1_hardware::MotorAddress & address);
  /** @brief 向全部电机发送零增益 MIT 帧，请求更新反馈。 */
  bool pollAll();
  /** @brief 非阻塞读取全部电机的最新反馈，并检查反馈是否在线且未超时。 */
  bool latest(FeedbackArray & feedback, double now_s) const;
  /** @brief 非阻塞读取指定电机的最新反馈，并检查其是否新鲜健康。 */
  bool latestOne(
    const dm1_hardware::MotorAddress & address, dm1_hardware::MotorFeedback & feedback,
    double now_s) const;
  /** @brief 返回指定电机最近反馈是否明确报告为 Enabled。 */
  bool isEnabled(const dm1_hardware::MotorAddress & address) const noexcept;
  /** @brief 校验并发送一帧已经转换到电机坐标的 MIT 命令。 */
  bool sendMit(const dm1_hardware::MitFrame & frame);
  /** @brief 向指定电机发送设置当前位置为零位的命令。 */
  bool setZero(const dm1_hardware::MotorAddress & address);
  /** @brief 使能全部电机，并等待反馈确认每台电机均已使能。 */
  bool enableAll();
  /** @brief 只使能指定电机，并等待该电机反馈确认。 */
  bool enableOne(const dm1_hardware::MotorAddress & address);
  /** @brief 只禁用指定电机。 */
  bool disableOne(const dm1_hardware::MotorAddress & address) noexcept;
  /** @brief 多次尝试发送禁用命令，关闭全部电机输出。 */
  void disableAll() noexcept;
  /** @brief 关闭电机输出并关闭两条 CAN 总线。 */
  void close() noexcept;
  /** @brief 返回两条 CAN 总线是否已经打开。 */
  bool isOpen() const noexcept {return opened_.load();}

  /** @brief 解码 8 字节 DM MIT 状态帧，但不应用方向和零位标定。 */
  static bool decodeFeedback(const canfd_frame & frame, DecodedFeedback & decoded) noexcept;
  /** @brief 判断电机状态是否为正常的未使能或已使能状态。 */
  static bool isHealthyStatus(std::uint8_t status) noexcept
  {
    return status == static_cast<std::uint8_t>(MotorStatus::Disabled) ||
           status == static_cast<std::uint8_t>(MotorStatus::Enabled);
  }
  /** @brief 检查反馈中的电机 ID、主站 ID 是否匹配期望地址。 */
  static bool matchesFeedbackAddress(
    const dm1_hardware::MotorAddress & expected, std::uint16_t master_id,
    const DecodedFeedback & decoded) noexcept;
  /** @brief 将浮点 MIT 命令量编码为 8 字节 CAN 帧。 */
  static can_frame encodeMitFrame(
    const dm1_hardware::MotorAddress & address,
    float position, float velocity, float kp, float kd, float torque) noexcept;
  /** @brief 将使能/禁用/置零等控制命令编码为 CAN 帧。 */
  static can_frame encodeCommandFrame(
    const dm1_hardware::MotorAddress & address, std::uint8_t command) noexcept;

private:
  // 单个电机最近一次接收的反馈快照，由 mutex_ 保护。
  struct Snapshot
  {
    float position = 0.0F;  // 最近一次原始位置，rad。
    float velocity = 0.0F;  // 最近一次原始速度，rad/s。
    float torque = 0.0F;  // 最近一次原始力矩，N*m。
    std::chrono::steady_clock::time_point received_at{};  // 接收时间。
    std::uint64_t sequence = 0;  // 接收序号，递增用于判断数据是否更新。
    std::uint8_t status = 0;  // 最近一次电机状态码。
    float mos_temperature_c = 0.0F;  // MOS/驱动器温度，摄氏度。
    float rotor_temperature_c = 0.0F;  // 转子温度，摄氏度。
    bool online = false;  // 是否收到与标定地址匹配的反馈。
  };

  // 接收线程回调：解码、校验地址，并更新对应电机的快照。
  void receive(std::uint8_t bus, const canfd_frame & frame) noexcept;
  // 发送一个原始电机控制命令，例如使能、禁用或设置零位。
  bool sendCommand(const dm1_hardware::MotorAddress & address, std::uint8_t command);
  // 写入单个电机的协议参数；单电机测试只用于切换到 MIT 模式。
  bool writeParameter(
    const dm1_hardware::MotorAddress & address, std::uint8_t register_id,
    std::uint32_t value);
  // 检查 12 个电机是否都在线且已经进入使能状态。
  bool allMotorsEnabled() const noexcept;
  // 按总线和物理 CAN ID 查找标定数组中的电机索引。
  std::size_t indexFor(const dm1_hardware::MotorAddress & address) const noexcept;
  std::string can_names_[2];  // 两条 CAN 总线的设备名称。
  CalibrationArray calibration_{};  // 12 个电机的地址和标定信息。
  damiao::SocketCAN buses_[2];  // 两条底层 SocketCAN 总线对象。
  mutable std::mutex mutex_;  // 保护 snapshots_ 的读写互斥量。
  std::array<Snapshot, kNumJoints> snapshots_{};  // 12 个电机的最新反馈快照。
  std::array<bool, 2> active_buses_{{false, false}};  // 当前实际打开的总线。
  std::optional<dm1_hardware::MotorAddress> single_target_;  // 单电机模式的唯一目标。
  std::atomic_bool opened_{false};  // CAN 总线是否已打开。
  std::atomic_bool output_disabled_{false};  // 是否已经完成禁用输出。
};

#endif  // MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_
