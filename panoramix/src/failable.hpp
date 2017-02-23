#pragma once

#include <cassert>
#include <utility>

namespace pano {
namespace core {
// failable result
template <class T> class Failable {
public:
  Failable() : _exist(false) {}

  Failable(nullptr_t) : _exist(false) {}
  Failable &operator=(nullptr_t) {
    _exist = false;
    return *this;
  }

  Failable(T &&v) : _exist(true), _temp(std::move(v)) {}
  Failable &operator=(T &&v) {
    _temp = std::move(v);
    _exist = true;
    return *this;
  }

  Failable(Failable &&opt) { swap(opt); }
  Failable &operator=(Failable &&opt) {
    swap(opt);
    return *this;
  }

  Failable(const Failable &) = delete;
  Failable &operator=(const Failable &) = delete;

  inline void swap(Failable &opt) {
    std::swap(_exist, opt._exist);
    std::swap(_temp, opt._temp);
  }

  inline bool null() const { return !_exist; }
  inline bool failed() const { return null(); }
  inline const T &ref() const { return _temp; }

  inline T unwrap() {
    if (!_exist) {
      std::cout << "unwrapping a failed Failable<T>!!!!!!!" << std::endl;
    }
    assert(_exist);
    _exist = false;
    return std::move(_temp);
  }
  template <class TT> inline T unwrap(TT &&defaultValue) {
    bool e = _exist;
    _exist = false;
    return e ? std::move(_temp) : std::forward<TT>(defaultValue);
  }
  inline operator T() { return unwrap(); }

private:
  bool _exist;
  T _temp;
};

template <class T, class = std::enable_if_t<!std::is_reference<T>::value>>
inline Failable<T> AsResult(T &&v) {
  return Failable<T>(std::move(v));
}
}
}

namespace std {
template <class T>
void swap(pano::core::Failable<T> &a, pano::core::Failable<T> &b) {
  a.swap(b);
}
}
