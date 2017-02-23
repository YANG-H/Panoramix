#include "pch.hpp"

#include "clock.hpp"

namespace pano {
namespace misc {

Clock::Clock(const std::string &msg) : _message(msg) {
  std::cout << "[" << _message << "] Started." << std::endl;
  _startTime = std::chrono::system_clock::now();
}
Clock::~Clock() {
  auto duration = std::chrono::system_clock::now() - _startTime;
  std::cout << "[" << _message << "] Stopped. Time Elapsed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
                   .count()
            << " ms" << std::endl;
}

std::string CurrentTimeString(bool tagified) {
  auto curTime =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string timeStr = std::ctime(&curTime);
  if (tagified) {
    std::replace(timeStr.begin(), timeStr.end(), '\\', '.');
    std::replace(timeStr.begin(), timeStr.end(), '\n', '_');
    std::replace(timeStr.begin(), timeStr.end(), '/', '.');
    std::replace(timeStr.begin(), timeStr.end(), ':', '_');
    std::replace(timeStr.begin(), timeStr.end(), ' ', '_');
  }
  return timeStr;
}
}
}
