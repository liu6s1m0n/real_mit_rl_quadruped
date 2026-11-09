/*! @file hardware_bridge.hpp
 *  @brief DM1 真机硬件桥接接口。
 *
 *  负责连接真实硬件、执行启动检查，并驱动 OpenDog 控制算法。
 */
#ifndef DM1_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_
#define DM1_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_

#include "common/control_config.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "control/opendog_robot_runner.hpp"
#include "hardware/dm1_hardware_io.hpp"
#include "teleop/keyboard_teleop.hpp"

/**
 * @brief 真机主循环：读硬件 → 处理键盘命令 → 运行算法 → 校验并下发 MIT 帧。
 */
class HardwareBridge
{
public:
  /// 真机控制循环的时间参数、启动策略和输出安全开关。
  struct Options
  {
    float control_time_step = float(control_config::control_time_step);  ///< 控制周期，s。
    double feedback_timeout_s = control_config::hardware::feedback_timeout;  ///< 电机反馈超时，s。
    /// 关节力矩饱和上限；电机连续额定 30 Nm、峰值 97 Nm，默认 30 Nm。
    /// 超过该值的关节会把 kp/kd/前馈按同一比例缩小到上限，其余关节不受影响，
    /// 整批 12 帧照常发送（丢帧会让电机收不到命令而超时失能）。
    float torque_limit_nm = control_config::hardware::torque_limit;
    /// 启动检查时判定关节静止的速度阈值，rad/s。
    float startup_stationary_velocity_rad_s = control_config::hardware::startup_stationary_velocity;
    /// false 时为只读模式：电机保持失能，只持续监测 IMU。
    bool keyboard_control = false;
    bool set_zero = false;  ///< true 时启动检查通过后把当前位置写为电机零位。
    /// OpenDog 配置目录（task/reference/gait/URDF）。
    std::string algorithm_config_directory = "config/opendog";
    /// 可选的窗口/测试命令输入，与键盘命令走同一个仲裁器。
    std::function<std::vector<teleop::OperatorCommand>()> command_source;
    /// 允许的连续控制周期超时次数，超过即安全停机。
    int max_consecutive_cycle_overruns = control_config::hardware::maximum_consecutive_overruns;
    /// 统计窗口内允许的周期超时次数。
    int max_cycle_overruns_in_window = control_config::hardware::maximum_overruns_in_window;
    double cycle_overrun_window_s = control_config::hardware::overrun_window;  ///< 超时统计窗口，s。
  };

  /// 构造硬件桥，并绑定电机标定参数、控制周期和反馈超时配置。
  HardwareBridge(
    dm1_hardware::Dm1HardwareIo & hardware,
    const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
    const Options & options);

  /// 打开硬件并运行主控制循环，直到收到停止请求或发生安全故障。
  /// 返回 0 表示正常退出，非 0 退出码的含义见 hardware_bridge.cpp 末尾。
  int run(const std::atomic_bool & stop_requested);

private:
  /// 启动执行一轮静止/水平/反馈健康检查；--set-zero 直接写入电机零位。
  bool performStartupZeroCalibration(
    const std::chrono::steady_clock::time_point & start,
    const std::atomic_bool & stop_requested);
  /// 检查 IMU 与全部电机反馈是否满足静止、水平启动条件。
  bool stationaryLevelSample(
    const dm1_hardware::HardwareSample & sample) const noexcept;
  /// 运行键盘遥操作循环；该模式下电机默认锁定，输出必须显式解锁。
  int runKeyboardControl(
    const std::chrono::steady_clock::time_point & start,
    const std::atomic_bool & stop_requested);

  dm1_hardware::Dm1HardwareIo & hardware_;                       ///< 真机或仿真硬件。
  dm1_hardware::Dm1MitInterface::CalibrationArray calibration_;  ///< 12 个电机标定。
  Options options_;                                              ///< 运行参数。
  OpenDogRobotRunner runner_;                                    ///< OpenDog 算法运行器。
  dm1_hardware::Dm1MitInterface mit_;                            ///< 命令校验与 MIT 帧转换。
};

#endif  // DM1_ROBOT_HARDWARE_HARDWARE_BRIDGE_HPP_
