/*! @file HardwareSimulation.hpp
 *  @brief 硬件在环仿真：在真实 CAN/IMU 驱动之下用 MuJoCo 替换物理设备。
 *
 *  电机驱动、MIT 编解码、IMU 串口解析和硬件桥全部沿用真机代码；
 *  这里只模拟电机固件（MIT 帧解码、PD 力矩、反馈帧）和 IMU 串口字节流（伪终端）。
 */
#ifndef DM1_ROBOT_SIMULATION_HARDWARE_SIMULATION_HPP_
#define DM1_ROBOT_SIMULATION_HARDWARE_SIMULATION_HPP_

#include "common/control_config.h"
#include <array>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>
#include "hardware/dm_motor_driver.hpp"
#include "sensor/imu.hpp"
#include "sensor/imu_driver.hpp"

/// 硬件在环仿真参数；延迟和故障注入都是压力测试假设，不是辨识出的 DM1 设备参数。
struct HardwareSimulationOptions
{
  bool enabled = false;                       ///< 是否启用硬件在环仿真。
  std::string calibration_path;               ///< 电机标定文件路径。
  std::string algorithm_config_directory = "config/opendog";  ///< OpenDog 配置目录。
  float torque_limit_nm = control_config::hardware::torque_limit;  ///< 默认值与 hardware_main 相同。
  double command_delay_ms = 1.0;              ///< CAN 命令到达“电机”的延迟。
  double feedback_delay_ms = 1.0;             ///< 电机反馈回到驱动的延迟。
  double imu_period_ms = 2.0;                 ///< IMU 输出周期。
  double imu_delay_ms = 2.0;                  ///< IMU 数据写入串口的延迟。
  double motor_time_constant_ms = 4.0;        ///< 电机力矩一阶滞后时间常数，0 表示无滞后。
  int weak_joint = -1;                        ///< 输出被削弱的关节下标，-1 表示不注入。
  double output_scale = 1.0;                  ///< weak_joint 的输出力矩比例，[0, 1]。
  double impairment_after_s = 0.0;            ///< 首次使能电机后多久开始注入故障，s。
  double drop_every = 0;                      ///< 每第 N 次 MIT 写入丢弃一次；0 表示不丢帧。
  /// 检查参数范围，非法时抛出 std::invalid_argument。
  void validate() const;
};

/**
 * @brief 替换 DmMotorDriver 底层 CAN 传输的 MuJoCo 设备模型。
 *
 * 写入的 CAN 帧按命令延迟排队，到期后按电机固件语义解码；电机根据 MIT PD+前馈
 * 计算力矩写入 qfrc_applied，并按反馈延迟回送反馈帧。IMU 数据经伪终端以真实串口
 * 帧格式输出，由真机 IMU 驱动读取。
 */
class HardwareSimulation final : public DmMotorDriver::CanTransport
{
public:
  using Clock = std::chrono::steady_clock;
  using Calibration = dm1_hardware::Dm1MitInterface::CalibrationArray;  ///< 12 个电机标定。
  /// 绑定 MuJoCo 模型/数据和共享互斥锁，查找关节与执行器并创建 IMU 伪终端。
  HardwareSimulation(mjModel * model, mjData * data, std::recursive_mutex & mutex,
    const Calibration & calibration, const HardwareSimulationOptions & options);
  ~HardwareSimulation() override;
  /// 模拟 IMU 串口的设备路径（伪终端 slave 端）。
  const std::string & imuDevice() const noexcept {return imu_device_;}
  bool open(std::uint8_t bus, Receiver receiver) override;  ///< 注册总线接收回调。
  bool write(std::uint8_t bus, const can_frame & frame) override;  ///< 写入一帧并按延迟排队。
  void close(std::uint8_t bus) override;  ///< 清空该总线队列并失能其电机。
  /// 在共享的物理/渲染锁内调用，每个物理步一次。
  void tick(bool advance_physics);
  /// GUI 暂停/复位时立即在设备侧失能；之后硬件桥会重启。
  void emergencyStop();
  /// 是否有任一电机处于使能状态。
  bool motorsEnabled() const noexcept;

private:
  /// 单个模拟电机的固件状态。
  struct Motor
  {
    int qpos = -1, dof = -1, actuator = -1;  ///< MuJoCo 中的 qpos/自由度/执行器地址。
    bool enabled = false;                    ///< 是否已使能。
    double zero_offset = 0.0;                ///< 电机零位偏置，置零命令会更新它。
    double q_des = 0.0, dq_des = 0.0, kp = 0.0, kd = 0.0, ff = 0.0;  ///< 最近一次 MIT 命令。
    double applied_torque = 0.0;             ///< 经滞后和饱和后实际施加的力矩。
  };
  /// 尚未到达电机的 CAN 命令。
  struct PendingCan
  {
    Clock::time_point ready;  ///< 到达时间。
    std::uint8_t bus;         ///< 总线编号。
    can_frame frame;          ///< 原始帧。
  };
  /// 尚未送达驱动的电机反馈。
  struct PendingFeedback
  {
    Clock::time_point ready;  ///< 送达时间。
    std::uint8_t bus;         ///< 总线编号。
    canfd_frame frame;        ///< 反馈帧。
  };
  /// 尚未完整写入伪终端的 IMU 帧。
  struct PendingImu
  {
    Clock::time_point ready;  ///< 可写入时间。
    ImuDriver::Frame frame;   ///< 57 字节 IMU 帧。
    std::size_t written = 0;  ///< 已写入的字节数。
  };
  /// 按电机固件语义处理一帧命令（使能/失能/置零/MIT），并排队一帧反馈。
  void receiveCommand(const PendingCan & pending, Clock::time_point now);
  /// 按 DM MIT 反馈格式编码指定关节的当前状态。
  canfd_frame feedbackFrame(std::size_t joint) const;
  /// 用仿真 IMU 数据生成一帧 DM IMU 串口帧。
  ImuDriver::Frame imuFrame();
  mjModel * model_;                         ///< MuJoCo 模型。
  mjData * data_;                           ///< MuJoCo 数据。
  std::recursive_mutex & mutex_;            ///< 与物理/渲染线程共享的锁。
  Calibration calibration_;                 ///< 电机标定。
  HardwareSimulationOptions options_;       ///< 仿真参数。
  SimImu imu_;                              ///< 仿真 IMU 传感器读取。
  std::array<Motor, kNumJoints> motors_{};  ///< 12 个模拟电机。
  std::array<Receiver, 2> receivers_{};     ///< 两条总线的接收回调。
  std::deque<PendingCan> commands_;         ///< 命令延迟队列。
  std::deque<PendingFeedback> feedback_;    ///< 反馈延迟队列。
  std::deque<PendingImu> imu_frames_;       ///< IMU 输出队列。
  int master_fd_ = -1, slave_fd_ = -1;      ///< IMU 伪终端两端。
  std::string imu_device_;                  ///< 伪终端 slave 设备路径。
  Clock::time_point next_imu_{}, enabled_at_{};  ///< 下一帧 IMU 时间 / 首次使能时间。
  std::size_t mit_writes_ = 0;              ///< MIT 写入计数，用于丢帧注入。
};
#endif
