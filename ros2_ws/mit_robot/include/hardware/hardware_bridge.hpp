// ============================================================
// DM1 真机硬件桥接接口。
// 负责连接真实硬件、执行启动安全检查，并驱动 RobotRunner 控制管线。
// ============================================================
#ifndef MYMIT_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_
#define MYMIT_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_

#include <atomic>
#include <chrono>

#include "RobotRunner.hpp"
#include "hardware/dm1_hardware_io.hpp"
#include "teleop/keyboard_teleop.hpp"

class HardwareBridge
{
public:
  // 真机控制循环的时间参数、启动策略和输出安全开关。
  struct Options
  {
    float control_time_step = 0.002F;
    double feedback_timeout_s = 0.05;
    // 普通启动只检查当前位置是否落在该窗口内；写入电机零位必须显式使用
    // --set-zero 维护操作。
    float startup_zero_tolerance_rad = 0.05F;
    float startup_stationary_velocity_rad_s = 0.05F;
    // 在写入零位或使能物理输出前，要求连续多个采样周期保持稳定；
    // 对 CAN/IMU 来说，仅有一个正常帧是不够的。
    int startup_stable_samples = 3;
    bool enable_output = false;
    bool keyboard_control = false;
    bool request_stand_up = false;
    bool set_zero = false;
    int max_consecutive_cycle_overruns = 3;
    int max_cycle_overruns_in_window = 5;
    double cycle_overrun_window_s = 1.0;
  };

  // 构造硬件桥，并绑定电机标定参数、控制周期和反馈超时配置。
  HardwareBridge(
    dm1_hardware::Dm1HardwareIo & hardware,
    const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
    const Options & options);

  // 打开硬件并运行主控制循环，直到收到停止请求或发生安全故障。
  int run(const std::atomic_bool & stop_requested);

private:
  // 执行启动时的静止、水平和电机零位检查；必要时显式重置电机零位。
  bool performStartupZeroCalibration(
    const std::chrono::steady_clock::time_point & start,
    const std::atomic_bool & stop_requested);
  // 检查 IMU 与全部电机反馈是否满足静止、水平启动条件。
  bool stationaryLevelSample(
    const dm1_hardware::HardwareSample & sample) const noexcept;
  // 检查全部电机位置是否落在给定的零位误差窗口内。
  bool motorPositionsWithin(
    const dm1_hardware::HardwareSample & sample,
    float tolerance_rad) const noexcept;
  // 报告所有超出零位误差窗口或缺少有效反馈的电机。
  void reportInvalidMotorPositions(
    const dm1_hardware::HardwareSample & sample,
    float tolerance_rad) const;
  // 运行键盘遥操作循环；该模式下电机默认锁定，输出必须显式解锁。
  int runKeyboardControl(
    const std::chrono::steady_clock::time_point & start,
    const std::atomic_bool & stop_requested);

  dm1_hardware::Dm1HardwareIo & hardware_;
  dm1_hardware::Dm1MitInterface::CalibrationArray calibration_;
  Options options_;
  RobotRunner runner_;
  dm1_hardware::Dm1MitInterface mit_;
};

#endif  // MYMIT_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_
