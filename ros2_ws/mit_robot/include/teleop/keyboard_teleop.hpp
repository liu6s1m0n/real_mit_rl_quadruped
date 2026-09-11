/**
 * @file keyboard_teleop.hpp
 * @brief 由 RAII 管理的非阻塞终端键盘输入生产器。
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

  /** 启动原始终端捕获；当标准输入不是 TTY 时返回 false。 */
  bool start();
  void stop() noexcept;
  /** 注入由 MuJoCo/GLFW 窗口接收到的按键。 */
  void injectKey(int key);
  /** 获取终端命令，但不获取窗口命令。 */
  std::vector<OperatorCommand> consume();
  /** 获取当前聚焦的 MuJoCo/GLFW 窗口产生的命令。 */
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
