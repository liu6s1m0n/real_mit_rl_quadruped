// Physical-device replacement below the real CAN and IMU drivers.
#ifndef MYMIT_ROBOT_USER_HARDWARE_SIMULATION_HPP_
#define MYMIT_ROBOT_USER_HARDWARE_SIMULATION_HPP_

#include <array>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>
#include "hardware/dm_motor_driver.hpp"
#include "sensor/imu.hpp"
#include "sensor/imu_driver.hpp"

struct HardwareSimulationOptions
{
  bool enabled = false;
  std::string calibration_path;
  float torque_limit_nm = 88.0F;  // Same default as hardware_main.
  // Unmeasured stress assumptions, not identified DM1 device parameters.
  double command_delay_ms = 1.0;
  double feedback_delay_ms = 1.0;
  double imu_period_ms = 2.0;
  double imu_delay_ms = 2.0;
  double motor_time_constant_ms = 4.0;
  int weak_joint = -1;
  double output_scale = 1.0;
  double impairment_after_s = 0.0;  // Seconds since first motor enable.
  double drop_every = 0;  // Every Nth MIT write; 0 disables injection.
  void validate() const;
};

class HardwareSimulation final : public DmMotorDriver::CanTransport
{
public:
  using Clock = std::chrono::steady_clock;
  using Calibration = dm1_hardware::Dm1MitInterface::CalibrationArray;
  HardwareSimulation(mjModel * model, mjData * data, std::recursive_mutex & mutex,
    const Calibration & calibration, const HardwareSimulationOptions & options);
  ~HardwareSimulation() override;
  const std::string & imuDevice() const noexcept {return imu_device_;}
  bool open(std::uint8_t bus, Receiver receiver) override;
  bool write(std::uint8_t bus, const can_frame & frame) override;
  void close(std::uint8_t bus) override;
  // Call under the shared physics/render mutex, once per physics tick.
  void tick(bool advance_physics);
  // Immediate device-side disable for GUI pause/reset; bridge is then restarted.
  void emergencyStop();
  std::size_t mitWrites() const noexcept {return mit_writes_;}
  std::size_t droppedWrites() const noexcept {return dropped_writes_;}
  bool motorsEnabled() const noexcept;

private:
  struct Motor
  {
    int qpos = -1, dof = -1, actuator = -1;
    bool enabled = false;
    double zero_offset = 0.0;
    double q_des = 0.0, dq_des = 0.0, kp = 0.0, kd = 0.0, ff = 0.0;
    double applied_torque = 0.0;
  };
  struct PendingCan
  {
    Clock::time_point ready;
    std::uint8_t bus;
    can_frame frame;
  };
  struct PendingFeedback
  {
    Clock::time_point ready;
    std::uint8_t bus;
    canfd_frame frame;
  };
  struct PendingImu
  {
    Clock::time_point ready;
    ImuDriver::Frame frame;
    std::size_t written = 0;
  };
  void receiveCommand(const PendingCan & pending, Clock::time_point now);
  canfd_frame feedbackFrame(std::size_t joint) const;
  ImuDriver::Frame imuFrame();
  mjModel * model_;
  mjData * data_;
  std::recursive_mutex & mutex_;
  Calibration calibration_;
  HardwareSimulationOptions options_;
  SimImu imu_;
  std::array<Motor, kNumJoints> motors_{};
  std::array<Receiver, 2> receivers_{};
  std::deque<PendingCan> commands_;
  std::deque<PendingFeedback> feedback_;
  std::deque<PendingImu> imu_frames_;
  int master_fd_ = -1, slave_fd_ = -1;
  std::string imu_device_;
  Clock::time_point next_imu_{}, enabled_at_{};
  std::size_t mit_writes_ = 0, dropped_writes_ = 0;
};
#endif
