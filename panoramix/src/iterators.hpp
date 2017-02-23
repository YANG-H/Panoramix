#pragma once

#include <cassert>
#include <chrono>
#include <iostream>
#include <iterator>
#include <iterator>
#include <map>
#include <set>
#include <stack>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include "meta.hpp"

namespace pano {
namespace core {

//// ITERATORS
template <class IterT, class T>
struct IsIteratorOfType
    : std::is_same<typename std::iterator_traits<IterT>::value_type, T> {};

template <class ContainerT, class T>
struct IsContainerOfType
    : IsIteratorOfType<decltype(std::begin(std::declval<ContainerT>())), T> {};

// IsIterator
template <class T, class = void> struct IsIterator : no {};
template <class T>
struct IsIterator<T, void_t<typename T::iterator_category,
                            typename T::value_type, typename T::difference_type,
                            typename T::pointer, typename T::reference>> : yes {
};
template <class T> struct IsIterator<T *> : yes {};

template <class ContainerT, class FunT>
constexpr auto MakeTransformRange(ContainerT &&c, FunT f);
template <class ContainerT1, class ContainerT2>
constexpr auto MakeConcatedRange(ContainerT1 &&c1, ContainerT2 &&c2);
template <class ContainerT, class PredT>
constexpr auto MakeConditionalRange(ContainerT &&c, PredT pred);

// Range
template <class IterT> struct Range {
  IterT b, e;
  Range(IterT bb, IterT ee) : b(bb), e(ee) {}
  template <class ContainerT>
  explicit Range(ContainerT &&cont) : b(std::begin(cont)), e(std::end(cont)) {}

  IterT begin() const { return b; }
  IterT end() const { return e; }
  IterT cbegin() const { return b; }
  IterT cend() const { return e; }

  decltype(auto) operator[](size_t i) const { return *(b + i); }
  decltype(auto) operator()(size_t i) const { return *(b + i); }

  template <class FunT> void forEach(FunT &&fun) const {
    IterT i = b;
    while (i != e) {
      fun(*i);
      ++i;
    }
  }

  // (0, 1), (0, 2), ..., (0, n-1), (1, 2), (1, 3), ... (n-2, n-1)
  template <class FunT> void forEachTwo(FunT &&fun) const {
    for (IterT i = b; i != e; ++i) {
      for (IterT j = std::next(i); j != e; ++j) {
        fun(*i, *j);
      }
    }
  }

  template <class FunT> auto transform(FunT &&fun) const {
    return MakeTransformRange(*this, std::forward<FunT>(fun));
  }
  template <class PredT> auto filter(PredT &&pred) const {
    return MakeConditionalRange(*this, std::forward<PredT>(pred));
  }

  template <class ContainerT> ContainerT evalAs() const {
    return ContainerT(b, e);
  }
  auto evalAsStdVector() const {
    using ValueType = typename std::iterator_traits<IterT>::value_type;
    return std::vector<ValueType>(b, e);
  }

  bool operator==(const Range &r) const { return b == r.b && e == r.e; }
  bool operator!=(const Range &r) const { return !(*this == r); }
};

// MakeRange
template <class IterT> Range<IterT> MakeRange(IterT b, IterT e) {
  return Range<IterT>(b, e);
}
template <class ContainerT> auto MakeRange(ContainerT &&cont) {
  return MakeRange(std::begin(cont), std::end(cont));
}

// IotaIterator
template <class T>
class IotaIterator : public std::iterator<std::random_access_iterator_tag, T, T,
                                          const T *, const T &> {
public:
  explicit IotaIterator(T v) : value(v){}
  const T &operator*() const { return value; }
  const T *operator->() const { return &value; }

  IotaIterator &operator++() {
    ++value;
    return *this;
  }
  IotaIterator &operator++(int) {
    auto tmp = *this;
    ++value;
    return tmp;
  }
  IotaIterator &operator--() {
    --value;
    return *this;
  }
  IotaIterator &operator--(int) {
    auto tmp = *this;
    --value;
    return tmp;
  }

  IotaIterator &operator+=(difference_type off) { // increment by integer
    value += off;
    return (*this);
  }
  IotaIterator
  operator+(difference_type off) const { // return this + integer
    return (IotaIterator(value + off));
  }

  IotaIterator &operator-=(difference_type off) { // decrement by integer
    current -= off;
    return (*this);
  }
  IotaIterator
  operator-(difference_type off) const { // return this - integer
    return (IotaIterator(value - off));
  }

public:
  T value;
};

template <class T>
constexpr bool operator==(const IotaIterator<T> &i1,
                          const IotaIterator<T> &i2) {
  return i1.value == i2.value;
}
template <class T>
constexpr bool operator!=(const IotaIterator<T> &i1,
                          const IotaIterator<T> &i2) {
  return !(i1 == i2);
}
template <class T>
constexpr bool operator<(const IotaIterator<T> &i1, const IotaIterator<T> &i2) {
  return i1.value < i2.value;
}
template <class T>
constexpr auto operator-(const IotaIterator<T> &i1, const IotaIterator<T> &i2) {
  return i1.value - i2.value;
}
// MakeIotaIterator
template <class T> constexpr auto MakeIotaIterator(T v) {
  return IotaIterator<T>(v);
}
// MakeIotaRange
template <class T>
constexpr auto MakeIotaRange(T v) {
  return MakeRange(IotaIterator<T>(0), IotaIterator<T>(v));
}
// MakeIotaRange
template <class T>
constexpr auto MakeIotaRange(T start, T end) {
  return MakeRange(IotaIterator<T>(start), IotaIterator<T>(end));
}


// TransformIterator
template <class T, class IterT, class FunT>
class TransformIterator
    : public std::iterator<
          typename std::iterator_traits<IterT>::iterator_category, T,
          typename std::iterator_traits<IterT>::difference_type> {
  static_assert(IsIterator<IterT>::value, "IterT must be an iterator type");

public:
  using difference_type = typename std::iterator_traits<IterT>::difference_type;

  constexpr TransformIterator() : current(), fun() {}
  template <class F>
  constexpr TransformIterator(IterT it, F &&f)
      : current(it), fun(std::forward<F>(f)) {}
  constexpr IterT base() const { return current; }

  decltype(auto) operator*() const { return fun(*current); }
  decltype(auto) operator-> () const { return &(**this); }

  TransformIterator &operator++() {
    ++current;
    return *this;
  }
  TransformIterator &operator++(int) {
    auto tmp = *this;
    ++current;
    return tmp;
  }
  TransformIterator &operator--() {
    --current;
    return *this;
  }
  TransformIterator &operator--(int) {
    auto tmp = *this;
    --current;
    return tmp;
  }

  TransformIterator &operator+=(difference_type off) { // increment by integer
    current += off;
    return (*this);
  }
  TransformIterator
  operator+(difference_type off) const { // return this + integer
    return (TransformIterator(current + off, fun));
  }

  TransformIterator &operator-=(difference_type off) { // decrement by integer
    current -= off;
    return (*this);
  }
  TransformIterator
  operator-(difference_type off) const { // return this - integer
    return (TransformIterator(current - off, fun));
  }

protected:
  IterT current;
  FunT fun;
};

template <class T, class IterT, class FunT>
constexpr bool operator==(const TransformIterator<T, IterT, FunT> &i1,
                          const TransformIterator<T, IterT, FunT> &i2) {
  return i1.base() == i2.base();
}
template <class T, class IterT, class FunT>
constexpr bool operator!=(const TransformIterator<T, IterT, FunT> &i1,
                          const TransformIterator<T, IterT, FunT> &i2) {
  return !(i1 == i2);
}
template <class T, class IterT, class FunT>
constexpr bool operator<(const TransformIterator<T, IterT, FunT> &i1,
                         const TransformIterator<T, IterT, FunT> &i2) {
  return i1.base() < i2.base();
}
template <class T, class IterT, class FunT>
constexpr auto operator-(const TransformIterator<T, IterT, FunT> &i1,
                         const TransformIterator<T, IterT, FunT> &i2) {
  return i1.base() - i2.base();
}

// MakeTransformIterator
template <class IterT, class FunT>
constexpr auto MakeTransformIterator(IterT it, FunT f) {
  using value_t = std::decay_t<decltype(f(*it))>;
  return TransformIterator<value_t, IterT, FunT>(it, f);
}

// MakeTransformRange
template <class IterT, class FunT>
constexpr auto MakeTransformRange(IterT b, IterT e, FunT f) {
  using value_t = std::decay_t<decltype(f(*b))>;
  return MakeRange(TransformIterator<value_t, IterT, FunT>(b, f),
                   TransformIterator<value_t, IterT, FunT>(e, f));
}
template <class ContainerT, class FunT>
constexpr auto MakeTransformRange(ContainerT &&c, FunT f) {
  static_assert(IsContainer<std::decay_t<ContainerT>>::value,
                "ContainerT must be a container");
  return MakeTransformRange(std::begin(c), std::end(c), f);
}

// ConcatedIterator
template <class IterT1, class IterT2>
class ConcatedIterator
    : public std::iterator<
          std::forward_iterator_tag,
          typename std::iterator_traits<IterT1>::value_type,
          std::common_type_t<
              typename std::iterator_traits<IterT1>::difference_type,
              typename std::iterator_traits<IterT2>::difference_type>,
          std::common_type_t<typename std::iterator_traits<IterT1>::pointer,
                             typename std::iterator_traits<IterT2>::pointer>,
          std::common_type_t<
              typename std::iterator_traits<IterT1>::reference,
              typename std::iterator_traits<IterT2>::reference>> {
public:
  constexpr ConcatedIterator(IterT1 i1, IterT1 e1, IterT2 i2, IterT2 e2)
      : iter1(i1), end1(e1), iter2(i2), end2(e2) {}

  ConcatedIterator &operator++() {
    if (iter1 == end1) {
      assert(iter2 != end2);
      ++iter2;
    } else {
      ++iter1;
    }
    return *this;
  }

  decltype(auto) operator*() const { return iter1 == end1 ? *iter2 : *iter1; }
  decltype(auto) operator-> () const {
    return iter1 == end1 ? &(*iter2) : &(*iter1);
  }

public:
  IterT1 iter1, end1;
  IterT2 iter2, end2;
};

template <class IterT1, class IterT2>
constexpr bool operator==(const ConcatedIterator<IterT1, IterT2> &i1,
                          const ConcatedIterator<IterT1, IterT2> &i2) {
  return std::make_tuple(i1.iter1, i1.end1, i1.iter2, i1.end2) ==
         std::make_tuple(i2.iter1, i2.end1, i2.iter2, i2.end2);
}
template <class IterT1, class IterT2>
constexpr bool operator!=(const ConcatedIterator<IterT1, IterT2> &i1,
                          const ConcatedIterator<IterT1, IterT2> &i2) {
  return !(i1 == i2);
}



// MakeConcatedRange
template <class IterT1, class IterT2>
constexpr auto MakeConcatedRange(IterT1 b1, IterT1 e1, IterT2 b2, IterT2 e2) {
  using ValueT1 = typename std::iterator_traits<IterT1>::value_type;
  using ValueT2 = typename std::iterator_traits<IterT2>::value_type;
  static_assert(std::is_same<ValueT1, ValueT2>::value,
                "Value types mismatched!");
  return MakeRange(ConcatedIterator<IterT1, IterT2>(b1, e1, b2, e2),
                   ConcatedIterator<IterT1, IterT2>(e1, e1, e2, e2));
}
template <class ContainerT1, class ContainerT2>
constexpr auto MakeConcatedRange(ContainerT1 &&c1, ContainerT2 &&c2) {
  return MakeConcatedRange(std::begin(c1), std::end(c1), std::begin(c2),
                           std::end(c2));
}

// StepIterator
template <class IterT>
class StepIterator
    : public std::iterator<
          typename std::iterator_traits<IterT>::iterator_category,
          typename std::iterator_traits<IterT>::value_type,
          typename std::iterator_traits<IterT>::difference_type,
          typename std::iterator_traits<IterT>::pointer,
          typename std::iterator_traits<IterT>::reference> {
  using difference_type = typename std::iterator_traits<IterT>::difference_type;

public:
  StepIterator(IterT it, difference_type s) : _it(it), _step(s) {}

  const IterT &base() const { return _iter; }

  decltype(auto) operator*() const { return *_iter; }
  decltype(auto) operator-> () const { return &(**this); }

  StepIterator &operator++() {
    std::advance(_it, _step);
    return *this;
  }
  StepIterator &operator++(int) {
    auto tmp = *this;
    std::advance(_it, _step);
    return tmp;
  }
  StepIterator &operator--() {
    std::advance(_it, -_step);
    return *this;
  }
  StepIterator &operator--(int) {
    auto tmp = *this;
    std::advance(_it, -_step);
    return tmp;
  }

  StepIterator &operator+=(difference_type off) { // increment by integer
    std::advance(_it, _step * off);
    return (*this);
  }
  StepIterator operator+(difference_type off) const { // return this + integer
    return (StepIterator(_it + _step * off, _step));
  }

  StepIterator &operator-=(difference_type off) { // decrement by integer
    std::advance(_it, -_step * off);
    return (*this);
  }
  StepIterator operator-(difference_type off) const { // return this - integer
    return (StepIterator(_it - _step * off, _step));
  }

private:
  IterT _it;
  typename std::iterator_traits<IterT>::difference_type _step;
};

template <class IterT>
constexpr bool operator==(const StepIterator<IterT> &i1,
                          const StepIterator<IterT> &i2) {
  return i1.base() == i2.base();
}
template <class IterT>
constexpr bool operator!=(const StepIterator<IterT> &i1,
                          const StepIterator<IterT> &i2) {
  return !(i1 == i2);
}
template <class IterT>
constexpr bool operator<(const StepIterator<IterT> &i1,
                         const StepIterator<IterT> &i2) {
  return i1.base() < i2.base();
}
template <class IterT>
constexpr auto operator-(const StepIterator<IterT> &i1,
                         const StepIterator<IterT> &i2) {
  return i1.base() - i2.base();
}

// element of container MUST support PredT(ele) -> bool
// ConditionalIterator will automatically skip elements which DO NOT satisfy
// PredT in iteration
template <class IterT, class PredT>
class ConditionalIterator
    : public std::iterator<
          std::forward_iterator_tag,
          typename std::iterator_traits<IterT>::value_type,
          typename std::iterator_traits<IterT>::difference_type,
          typename std::iterator_traits<IterT>::pointer,
          typename std::iterator_traits<IterT>::reference> {

public:
  using Iterator = IterT;
  ConditionalIterator(Iterator it_, Iterator end_, PredT pred_ = PredT())
      : _it(it_), _end(end_), _pred(pred_) {
    if (_it != _end && !_pred(*_it))
      ++(*this);
  }
  ConditionalIterator &operator++() {
    assert(_it != _end);
    ++_it;
    while (_it != _end && !_pred(*_it))
      ++_it;
    return *this;
  }

  reference operator*() const { return *_it; }
  pointer operator->() const { return &(*_it); }
  bool operator==(const ConditionalIterator &i) const { return _it == i._it; }
  bool operator!=(const ConditionalIterator &i) const { return !(*this == i); }
  Iterator internalIterator() const { return _it; }

private:
  IterT _it;
  IterT _end;
  PredT _pred;
};

// MakeConditionalRange
template <class IterT, class PredT>
constexpr auto MakeConditionalRange(IterT b, IterT e, PredT pred) {
  return MakeRange(ConditionalIterator<IterT, PredT>(b, e, pred),
                   ConditionalIterator<IterT, PredT>(e, e, pred));
}
template <class ContainerT, class PredT>
constexpr auto MakeConditionalRange(ContainerT &&c, PredT pred) {
  return MakeConditionalRange(std::begin(c), std::end(c), pred);
}
}
}