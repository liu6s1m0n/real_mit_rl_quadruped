/**
 * @file keyboard_teleop.hpp
 * @brief Non-blocking, RAII-managed terminal keyboard producer.
 */
#ifndef MYMIT_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_
#define MYMIT_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "teleop/operator_command.hpp"

namespace teleop
{

class KeyboardTeleop
{
public:
  KeyboardTeleop() = default;
  ~KeyboardTeleop();

  KeyboardTeleop(const KeyboardTeleop &) = delete;
  KeyboardTeleop & operator=(const KeyboardTeleop &) = delete;

  /** Start raw terminal capture. Returns false when stdin is not a TTY. */
  bool start();
  void stop() noexcept;
  /** Inject a key received by the MuJoCo/GLFW window. */
  void injectKey(int key);
  /** Consume terminal commands without consuming window commands. */
  std::vector<OperatorCommand> consume();
  /** Consume commands produced by the focused MuJoCo/GLFW window. */
  std::vector<OperatorCommand> consumeWindow();
  bool available() const noexcept {return available_;}
  bool healthy() const noexcept {return !available_ || thread_alive_;}

private:
  void readLoop() noexcept;
  void enqueue(std::deque<OperatorCommand> & queue, OperatorCommand command) noexcept;

  std::atomic_bool stopping_{false};
  std::atomic_bool thread_alive_{false};
  bool available_ = false;
  bool terminal_configured_ = false;
  std::thread thread_;
  mutable std::mutex mutex_;
  std::deque<OperatorCommand> terminal_commands_;
  std::deque<OperatorCommand> window_commands_;
  struct termios_state;
  termios_state * terminal_state_ = nullptr;
};

}  // namespace teleop

#endif  // MYMIT_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_
