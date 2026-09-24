// ============================================================
// 键盘遥操作输入实现。
// 负责配置原始终端、后台读取按键，并将终端和窗口命令分别放入队列。
// ============================================================

#include "teleop/keyboard_teleop.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <memory>

namespace teleop
{

// ---------- 终端状态保存 ----------

struct KeyboardTeleop::termios_state
{
  termios original{};
};

// 析构函数：停止输入线程，并恢复终端的原始配置。
KeyboardTeleop::~KeyboardTeleop()
{
  stop();
}

// 启动终端按键捕获：检查标准输入、切换终端模式，并启动后台读取线程。
bool KeyboardTeleop::start()
{
  // 已经存在可连接线程时，不重复启动。
  if (thread_.joinable()) {return available_;}

  // 只有交互式终端才能进行键盘遥操作；重定向或管道输入保持电机锁定。
  if (!::isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "keyboard teleop disabled: stdin is not a TTY; motors remain locked\n");
    return false;
  }

  // 保存终端原始配置，便于 stop() 时恢复。
  auto state = std::make_unique<termios_state>();
  if (::tcgetattr(STDIN_FILENO, &state->original) != 0) {
    std::fprintf(stderr, "keyboard teleop disabled: unable to read terminal settings\n");
    return false;
  }

  // 关闭规范模式和回显，使按键无需回车即可立即读取且不会显示在终端上。
  termios raw = state->original;
  // 保持 ISIG 启用，使 Ctrl+C 仍然能够到达进程级 SIGINT 处理器。
  raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;

  // 应用非规范终端配置。
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    std::fprintf(stderr, "keyboard teleop disabled: unable to configure raw terminal\n");
    return false;
  }

  // 记录终端已配置，初始化线程状态并启动读取线程。
  terminal_state_ = state.release();
  terminal_configured_ = true;
  available_ = true;
  stopping_.store(false);
  thread_alive_.store(true);
  thread_ = std::thread(&KeyboardTeleop::readLoop, this);
  return true;
}

// 停止终端输入：等待读取线程退出，并恢复终端原始配置。
void KeyboardTeleop::stop() noexcept
{
  // 通知后台线程退出；poll() 的超时机制保证线程能够定期检查该标志。
  stopping_.store(true);
  if (thread_.joinable()) {thread_.join();}

  // 恢复终端设置并释放保存的终端状态。
  if (terminal_configured_ && terminal_state_ != nullptr) {
    ::tcsetattr(STDIN_FILENO, TCSANOW, &terminal_state_->original);
    delete terminal_state_;
    terminal_state_ = nullptr;
    terminal_configured_ = false;
  }
  available_ = false;
}

// 注入窗口收到的按键，并将其放入独立的窗口命令队列。
void KeyboardTeleop::injectKey(int key)
{
  auto command = decodeKey(key);
  if (command.has_value()) {
    // 窗口输入拥有独立的队列和健康状态域。终端 EOF 绝不能导致当前聚焦的
    // MuJoCo 窗口不可用；关闭期间的窗口按键也不能被误认为终端输入。
    enqueue(window_commands_, *command);
  }
}

// 取出当前积累的所有终端命令，并清空终端命令队列。
std::vector<OperatorCommand> KeyboardTeleop::consume()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<OperatorCommand> result;
  result.reserve(terminal_commands_.size());
  while (!terminal_commands_.empty()) {
    result.push_back(terminal_commands_.front());
    terminal_commands_.pop_front();
  }
  return result;
}

// 取出当前积累的所有窗口命令，并清空窗口命令队列。
std::vector<OperatorCommand> KeyboardTeleop::consumeWindow()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<OperatorCommand> result;
  result.reserve(window_commands_.size());
  while (!window_commands_.empty()) {
    result.push_back(window_commands_.front());
    window_commands_.pop_front();
  }
  return result;
}

// 为命令分配序号，并以线程安全的方式追加到指定队列。
void KeyboardTeleop::enqueue(
  std::deque<OperatorCommand> & queue, OperatorCommand command) noexcept
{
  static std::atomic_uint64_t sequence{0};
  command.sequence = ++sequence;
  std::lock_guard<std::mutex> lock(mutex_);

  // 队列达到上限时丢弃最旧命令，优先保留最新的操作输入。
  constexpr std::size_t kMaximumQueuedCommands = 128;
  if (queue.size() >= kMaximumQueuedCommands) {queue.pop_front();}
  queue.push_back(command);
}

// 后台读取循环：轮询标准输入，解码有效按键并放入终端命令队列。
void KeyboardTeleop::readLoop() noexcept
{
  const char * exit_reason = "程序请求停止";
  int exit_error = 0;
  short exit_revents = 0;
  while (!stopping_.load()) {
    // 使用有限超时轮询，使线程既不会永久阻塞，也能及时响应停止请求。
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    /*等待标准输入，最多等待 100 ms。
      这样既不会永久阻塞，也能定期检查 stopping_。*/
    const int result = ::poll(&descriptor, 1, 100);
    if (result < 0) {
      // 被信号中断时继续读取；其他轮询错误会结束线程。
      if (errno == EINTR) {continue;}
      exit_reason = "poll() 失败";
      exit_error = errno;
      break;
    }
    // 超时但没有输入时，回到循环顶部检查 stopping_。
    if (result == 0) {continue;}

    // 输入描述符发生错误、挂起或失效时，结束读取线程。
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      exit_reason = "终端描述符错误、挂断或失效";
      exit_revents = descriptor.revents;
      break;
    }

    // 每次读取一个原始终端字节。
    unsigned char byte = 0;
    const ssize_t count = ::read(STDIN_FILENO, &byte, sizeof(byte));
    if (count == 0) {
      // 返回 0 表示标准输入已到达 EOF。
      exit_reason = "read() 返回 EOF";
      break;
    }
    if (count < 0) {
      // 可重试的读取错误继续循环，其他错误结束线程。
      if (errno == EINTR || errno == EAGAIN) {continue;}
      exit_reason = "read() 失败";
      exit_error = errno;
      break;
    }

    // 将原始按键转换为操作员命令；未知按键不会进入队列。
    const auto command = decodeKey(byte);
    if (command.has_value()) {enqueue(terminal_commands_, *command);}
  }

  if (!stopping_.load()) {
    std::fprintf(
      stderr,
      "键盘输入线程异常停止：reason=%s errno=%d(%s) revents=0x%x。\n",
      exit_reason, exit_error, exit_error == 0 ? "none" : std::strerror(exit_error),
      static_cast<unsigned int>(exit_revents));
  }
  // 线程退出前发布存活状态，供外部检测键盘输入故障。
  thread_alive_.store(false);
}

}  // teleop 命名空间
