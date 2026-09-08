#ifndef MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_
#define MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "dmbot_serial/protocol/socketcan.h"
#include "hardware/dm1_mit_interface.hpp"

/**
 * @brief Non-blocking dual-SocketCAN adapter for the twelve DM motors.
 *
 * The receive callbacks only update a locked snapshot.  Control code never
 * waits for a CAN frame and all bus/physical-ID lookups are explicit.
 */
class DmMotorDriver final
{
public:
  enum class MotorStatus : std::uint8_t
  {
    Disabled = 0,
    Enabled = 1,
    Fault2 = 2,
    Fault3 = 3,
    Fault5 = 5,
    Fault7 = 7,
    Fault8 = 8,
    Fault9 = 9,
    FaultA = 0x0A,
    FaultB = 0x0B,
    FaultC = 0x0C,
    FaultD = 0x0D,
    FaultE = 0x0E,
    UnknownFault = 0x0F
  };

  using CalibrationArray = dm1_hardware::Dm1MitInterface::CalibrationArray;
  using FeedbackArray = dm1_hardware::Dm1MitInterface::FeedbackArray;

  struct DecodedFeedback
  {
    std::uint16_t motor_id = 0;
    std::uint8_t status = 0;
    float position = 0.0F;
    float velocity = 0.0F;
    float torque = 0.0F;
    float mos_temperature_c = 0.0F;
    float rotor_temperature_c = 0.0F;
  };

  DmMotorDriver(std::string can0, std::string can1, const CalibrationArray & calibration);
  ~DmMotorDriver();

  DmMotorDriver(const DmMotorDriver &) = delete;
  DmMotorDriver & operator=(const DmMotorDriver &) = delete;

  bool open();
  bool pollAll();
  bool latest(FeedbackArray & feedback, double now_s) const;
  bool sendMit(const dm1_hardware::MitFrame & frame);
  bool setZero(const dm1_hardware::MotorAddress & address);
  bool enableAll();
  void disableAll() noexcept;
  void close() noexcept;
  bool isOpen() const noexcept {return opened_.load();}

  /** Decode the eight-byte DM MIT status frame without applying calibration. */
  static bool decodeFeedback(const canfd_frame & frame, DecodedFeedback & decoded) noexcept;
  static bool isHealthyStatus(std::uint8_t status) noexcept
  {
    return status == static_cast<std::uint8_t>(MotorStatus::Disabled) ||
           status == static_cast<std::uint8_t>(MotorStatus::Enabled);
  }
  static bool matchesFeedbackAddress(
    const dm1_hardware::MotorAddress & expected, std::uint16_t master_id,
    const DecodedFeedback & decoded) noexcept;
  static can_frame encodeMitFrame(
    const dm1_hardware::MotorAddress & address,
    float position, float velocity, float kp, float kd, float torque) noexcept;
  static can_frame encodeCommandFrame(
    const dm1_hardware::MotorAddress & address, std::uint8_t command) noexcept;

private:
  struct Snapshot
  {
    float position = 0.0F;
    float velocity = 0.0F;
    float torque = 0.0F;
    std::chrono::steady_clock::time_point received_at{};
    std::uint64_t sequence = 0;
    std::uint8_t status = 0;
    float mos_temperature_c = 0.0F;
    float rotor_temperature_c = 0.0F;
    bool online = false;
  };

  void receive(std::uint8_t bus, const canfd_frame & frame) noexcept;
  bool sendCommand(const dm1_hardware::MotorAddress & address, std::uint8_t command);
  bool allMotorsEnabled() const noexcept;
  std::size_t indexFor(const dm1_hardware::MotorAddress & address) const noexcept;
  std::string can_names_[2];
  CalibrationArray calibration_{};
  damiao::SocketCAN buses_[2];
  mutable std::mutex mutex_;
  std::array<Snapshot, kNumJoints> snapshots_{};
  std::atomic_bool opened_{false};
  std::atomic_bool output_disabled_{false};
};

#endif  // MYMIT_ROBOT_HARDWARE_DM_MOTOR_DRIVER_HPP_
