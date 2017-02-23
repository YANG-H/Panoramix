#pragma once

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

namespace pano {
namespace misc {
class Clock {
public:
  Clock(const std::string &msg);
  ~Clock();

private:
  std::chrono::system_clock::time_point _startTime;
  std::string _message;
};

#define SetClock() pano::misc::Clock clock##__COUNTER__(__FUNCTION__)

std::string CurrentTimeString(bool tagified = false);

// time_cost
template <class DurationT = std::chrono::milliseconds, class FunT>
auto TimeCost(FunT &&fun) {
  auto start_time = std::chrono::high_resolution_clock::now();
  std::forward<FunT>(fun)();
  auto d = std::chrono::high_resolution_clock::now() - start_time;
  return std::chrono::duration_cast<DurationT>(d);
}

namespace {
inline const char *period_str(const std::nano &) { return "nanoseconds"; }
inline const char *period_str(const std::micro &) { return "microseconds"; }
inline const char *period_str(const std::milli &) { return "milliseconds"; }
inline const char *period_str(const std::ratio<1> &) { return "seconds"; }
inline const char *period_str(const std::ratio<60> &) { return "minutes"; }
inline const char *period_str(const std::ratio<3600> &) { return "hours"; }
template <intmax_t Nx, intmax_t Dx>
inline std::string period_str(const std::ratio<Nx, Dx> &) {
  return "(x" + std::to_string(Nx) + "/" + std::to_string(Dx) + ") seconds";
}
}

template <class RepT, class PeriodT>
std::ostream &operator<<(std::ostream &os,
                         const std::chrono::duration<RepT, PeriodT> &d) {
  return os << d.count() << " " << period_str(PeriodT());
}
}
}
