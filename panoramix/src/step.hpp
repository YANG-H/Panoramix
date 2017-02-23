#pragma once

#include "file.hpp"

namespace pano {
namespace misc {
// StepConfig
class StepConfig {
public:
  std::string name;
  bool rerun;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_point_start,
      time_point_end;

public:
  StepConfig() : rerun(false) {}
  StepConfig(const std::string &n) : name(n), rerun(false) {}

private:
  struct Performer {
    std::function<bool()> begin_fun;
    std::function<void()> end_fun;
    template <class FunT> void operator()(FunT fun) const {
      if (begin_fun()) {
        fun();
        end_fun();
      }
    }
  };

public:
  template <class... DataTs>
  bool begin(const std::string &id, DataTs &... data) {
    bool run = rerun || !misc::LoadCache(id, name, data...);
    if (!run) {
      return false;
    }
    time_point_start = std::chrono::high_resolution_clock::now();
    return true;
  }
  template <class... DataTs> void end(const std::string &id, DataTs &... data) {
    time_point_end = std::chrono::high_resolution_clock::now();
    std::cout << "time cost of step [" << name << "] is: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     time_point_end - time_point_start)
              << std::endl;
    misc::SaveCache(id, name, data...);
  }
  template <class... DataTs>
  Performer perform(const std::string &id, DataTs &... data) {
    return Performer{
        [this, id, &data...]() -> bool { return this->begin(id, data...); },
        [this, id, &data...]() { this->end(id, data...); }};
  }

  template <class Archive> void serialize(Archive &ar) {
    ar(name, rerun, time_point_start, time_point_end);
  }
};
}
}