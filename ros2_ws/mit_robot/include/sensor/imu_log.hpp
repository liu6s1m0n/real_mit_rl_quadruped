/*! @file imu_log.hpp
 *  @brief IMU 日志颜色与非终端输出处理。
 */

#ifndef MYMIT_ROBOT_SENSOR_IMU_LOG_HPP_
#define MYMIT_ROBOT_SENSOR_IMU_LOG_HPP_

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include <unistd.h>

namespace imu_log
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

}  // namespace imu_log

#endif  // MYMIT_ROBOT_SENSOR_IMU_LOG_HPP_
