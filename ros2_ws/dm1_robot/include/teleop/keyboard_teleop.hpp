/**
 * @file keyboard_teleop.hpp
 * @brief 由 RAII 管理的非阻塞终端键盘输入生产器。
 */
#ifndef DM1_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_
#define DM1_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "teleop/operator_command.hpp"

namespace teleop
{

/**
 * @brief 后台线程读取原始终端按键，转换为 OperatorCommand 排队。
 *
 * 终端命令和 MuJoCo/GLFW 窗口命令分别排队；析构时恢复终端设置。
 */
class KeyboardTeleop
{
public:
  KeyboardTeleop() = default;
  /// 停止读取线程并恢复终端设置。
  ~KeyboardTeleop();

  KeyboardTeleop(const KeyboardTeleop &) = delete;
  KeyboardTeleop & operator=(const KeyboardTeleop &) = delete;

  /** 启动原始终端捕获；当标准输入不是 TTY 时返回 false。 */
  bool start();
  /** 停止读取线程并恢复终端设置，可重复调用。 */
  void stop() noexcept;
  /** 注入由 MuJoCo/GLFW 窗口接收到的按键。 */
  void injectKey(int key);
  /** 获取终端命令，但不获取窗口命令。 */
  std::vector<OperatorCommand> consume();
  /** 获取当前聚焦的 MuJoCo/GLFW 窗口产生的命令。 */
  std::vector<OperatorCommand> consumeWindow();
  /** 终端捕获是否已启动。 */
  bool available() const noexcept {return available_;}
  /** 未启用终端，或读取线程仍存活时返回 true。 */
  bool healthy() const noexcept {return !available_ || thread_alive_;}

private:
  /// 读取线程主循环：poll 标准输入并解码按键。
  void readLoop() noexcept;
  /// 给命令编号并加入队列。
  void enqueue(std::deque<OperatorCommand> & queue, OperatorCommand command) noexcept;

  std::atomic_bool stopping_{false};               ///< 请求读取线程退出。
  std::atomic_bool thread_alive_{false};           ///< 读取线程是否仍在运行。
  bool available_ = false;                         ///< 终端捕获是否已启动。
  bool terminal_configured_ = false;               ///< 是否已修改终端为原始模式。
  std::thread thread_;                             ///< 读取线程。
  mutable std::mutex mutex_;                       ///< 保护两个命令队列。
  std::deque<OperatorCommand> terminal_commands_;  ///< 终端按键命令。
  std::deque<OperatorCommand> window_commands_;    ///< 窗口按键命令。
  struct termios_state;                            ///< 保存原终端设置（避免在头文件引入 termios.h）。
  termios_state * terminal_state_ = nullptr;       ///< 原终端设置。
};

}  // namespace teleop

#endif  // DM1_ROBOT_TELEOP_KEYBOARD_TELEOP_HPP_
