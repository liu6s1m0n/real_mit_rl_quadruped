#ifndef DMBOT_SERIAL_PROTOCOL_TERMINAL_COLORS_H_
#define DMBOT_SERIAL_PROTOCOL_TERMINAL_COLORS_H_

#include <cstdio>
#include <cstdlib>

#include <unistd.h>

namespace damiao
{

  enum class TerminalColor
  {
    Yellow,
    Red,
    White
  };

  inline bool terminalColorsEnabled(FILE * stream) noexcept
  {
    return std::getenv("NO_COLOR") == nullptr && stream != nullptr &&
           ::isatty(::fileno(stream)) != 0;
  }

  inline const char * terminalColor(TerminalColor color, FILE * stream) noexcept
  {
    if (!terminalColorsEnabled(stream)) {return "";}
    switch (color) {
      case TerminalColor::Yellow: return "\033[33m";
      case TerminalColor::Red: return "\033[31m";
      case TerminalColor::White: return "\033[37m";
    }
    return "";
  }

  inline const char * terminalColorReset(FILE * stream) noexcept
  {
    return terminalColorsEnabled(stream) ? "\033[0m" : "";
  }

}  // namespace damiao

#endif  // DMBOT_SERIAL_PROTOCOL_TERMINAL_COLORS_H_
