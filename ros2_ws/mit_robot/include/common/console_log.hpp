/*! @file console_log.hpp
 *  @brief 工程统一的终端/非终端日志输出。
 */

#ifndef MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_
#define MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_

#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <unistd.h>

namespace robot_log
{

enum class Level
{
  Info,
  Warning,
  Error
};

inline bool colorEnabled() noexcept
{
  static const bool enabled =
    std::getenv("NO_COLOR") == nullptr && ::isatty(STDERR_FILENO) != 0;
  return enabled;
}

inline const char * prefix(Level level) noexcept
{
  if (!colorEnabled()) {return "";}
  switch (level) {
    case Level::Info: return "\033[37m";
    case Level::Warning: return "\033[33m";
    case Level::Error: return "\033[31m";
  }
  return "";
}

inline const char * suffix() noexcept
{
  return colorEnabled() ? "\033[0m" : "";
}

inline void print(Level level, const char * format, ...) noexcept
{
  std::fputs(prefix(level), stderr);

  va_list arguments;
  va_start(arguments, format);
  std::vfprintf(stderr, format, arguments);
  va_end(arguments);

  std::fputs(suffix(), stderr);
  std::fflush(stderr);
}

/**
 * @brief 时间节流器：把高频循环里的同类日志压到每 interval_ms 最多一条。
 *
 * 500 Hz 控制回路里每帧写终端会拖慢循环，进而让 CAN 发送排队、反馈变旧，
 * 形成"越打印越坏"的正反馈。热路径日志必须经过本类。
 * 典型用法：函数内 `static robot_log::Throttle throttle(1000);`。
 */
class Throttle
{
public:
  explicit Throttle(std::int64_t interval_ms = 1000) noexcept
  : interval_ms_(interval_ms) {}

  /** 距上次放行已超过间隔时返回 true，并刷新计时；否则返回 false。 */
  bool ready() noexcept
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_.time_since_epoch().count() != 0 &&
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_).count() <
      interval_ms_)
    {
      return false;
    }
    last_ = now;
    return true;
  }

private:
  std::int64_t interval_ms_;
  std::chrono::steady_clock::time_point last_{};
};

}  // namespace robot_log

// Temporary source compatibility for the existing hardware/IMU call sites.
// New code should use robot_log directly; this alias can be removed after
// downstream users have migrated.
namespace imu_log = robot_log;

#endif  // MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_
