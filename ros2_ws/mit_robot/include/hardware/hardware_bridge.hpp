/*! @file hardware_bridge.hpp
 *  @brief 真实 DM1 设备循环与 RobotRunner 控制管线之间的桥。
 */
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
  struct Options
  {
    float control_time_step = 0.002F;
    double feedback_timeout_s = 0.05;
    // Normal startup only checks this window. Writing motor parameters is a
    // separate, explicit --set-zero maintenance operation.
    float startup_zero_tolerance_rad = 0.05F;
    float startup_stationary_velocity_rad_s = 0.05F;
    bool enable_output = false;
    bool keyboard_control = false;
    bool request_stand_up = false;
    bool set_zero = false;
    int max_consecutive_cycle_overruns = 3;
    int max_cycle_overruns_in_window = 5;
    double cycle_overrun_window_s = 1.0;
  };

  HardwareBridge(
    dm1_hardware::Dm1HardwareIo & hardware,
    const dm1_hardware::Dm1MitInterface::CalibrationArray & calibration,
    const Options & options);

  int run(const std::atomic_bool & stop_requested);

private:
  bool performStartupZeroCalibration(
    const std::chrono::steady_clock::time_point & start);
  bool stationaryLevelSample(
    const dm1_hardware::HardwareSample & sample) const noexcept;
  bool motorPositionsWithin(
    const dm1_hardware::HardwareSample & sample,
    float tolerance_rad) const noexcept;
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
