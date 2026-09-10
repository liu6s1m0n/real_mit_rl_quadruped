#include "teleop/keyboard_teleop.hpp"

#include <cerrno>
#include <cstdio>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <memory>

namespace teleop
{

struct KeyboardTeleop::termios_state
{
  termios original{};
};

KeyboardTeleop::~KeyboardTeleop()
{
  stop();
}

bool KeyboardTeleop::start()
{
  if (thread_.joinable()) {return available_;}
  if (!::isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "keyboard teleop disabled: stdin is not a TTY; motors remain locked\n");
    return false;
  }

  auto state = std::make_unique<termios_state>();
  if (::tcgetattr(STDIN_FILENO, &state->original) != 0) {
    std::fprintf(stderr, "keyboard teleop disabled: unable to read terminal settings\n");
    return false;
  }
  termios raw = state->original;
  // Keep ISIG enabled so Ctrl+C still reaches the process-level SIGINT handler.
  raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    std::fprintf(stderr, "keyboard teleop disabled: unable to configure raw terminal\n");
    return false;
  }
  terminal_state_ = state.release();
  terminal_configured_ = true;
  available_ = true;
  stopping_.store(false);
  thread_alive_.store(true);
  thread_ = std::thread(&KeyboardTeleop::readLoop, this);
  return true;
}

void KeyboardTeleop::stop() noexcept
{
  stopping_.store(true);
  if (thread_.joinable()) {thread_.join();}
  if (terminal_configured_ && terminal_state_ != nullptr) {
    ::tcsetattr(STDIN_FILENO, TCSANOW, &terminal_state_->original);
    delete terminal_state_;
    terminal_state_ = nullptr;
    terminal_configured_ = false;
  }
  available_ = false;
}

void KeyboardTeleop::injectKey(int key)
{
  auto command = decodeKey(key);
  if (command.has_value()) {
    // Window input has its own queue and health domain. A terminal EOF must
    // never make a focused MuJoCo window unusable, and a window key must not
    // be mistaken for terminal input during shutdown.
    enqueue(window_commands_, *command);
  }
}

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

void KeyboardTeleop::enqueue(
  std::deque<OperatorCommand> & queue, OperatorCommand command) noexcept
{
  static std::atomic_uint64_t sequence{0};
  command.sequence = ++sequence;
  std::lock_guard<std::mutex> lock(mutex_);
  constexpr std::size_t kMaximumQueuedCommands = 128;
  if (queue.size() >= kMaximumQueuedCommands) {queue.pop_front();}
  queue.push_back(command);
}

void KeyboardTeleop::readLoop() noexcept
{
  while (!stopping_.load()) {
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, 100);
    if (result < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    if (result == 0) {continue;}
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      break;
    }
    unsigned char byte = 0;
    const ssize_t count = ::read(STDIN_FILENO, &byte, sizeof(byte));
    if (count == 0) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) {continue;}
      break;
    }
    const auto command = decodeKey(byte);
    if (command.has_value()) {enqueue(terminal_commands_, *command);}
  }
  thread_alive_.store(false);
}

}  // namespace teleop
