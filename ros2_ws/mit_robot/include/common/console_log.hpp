/*! @file console_log.hpp
 *  @brief 工程统一的终端/非终端日志输出。
 */

#ifndef MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_
#define MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_

#include <cstdarg>
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

}  // namespace robot_log

// Temporary source compatibility for the existing hardware/IMU call sites.
// New code should use robot_log directly; this alias can be removed after
// downstream users have migrated.
namespace imu_log = robot_log;

#endif  // MYMIT_ROBOT_COMMON_CONSOLE_LOG_HPP_
