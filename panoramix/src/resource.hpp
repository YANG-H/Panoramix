#pragma once

#include "basic_types.hpp"

namespace pano {
namespace gui {

// resource making
struct Resource {
  virtual bool isNull() const = 0;
  virtual void initialize() = 0;
  virtual bool isInitialized() const = 0;
  virtual bool bind() = 0;
  virtual void destroy() = 0;
  virtual ~Resource() {}
};

using ResourcePtr = std::shared_ptr<Resource>;
ResourcePtr MakeResource(const core::Image &tex);

struct ResourceStore {
  static void set(const std::string &name, ResourcePtr r);
  template <class T> static void set(const std::string &name, const T &data) {
    set(name, MakeResource(data));
  }
  static ResourcePtr get(const std::string &name);
  static bool has(const std::string &name);
  static void clear();
};
}
}
