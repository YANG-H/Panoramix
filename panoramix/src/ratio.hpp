#pragma once

#include <cstdint>
#include <ratio>
#include <type_traits>

namespace pano {
namespace core {

// ratio
template <class T, class S> struct Ratio {
  inline Ratio() : denominator(1) {}
  inline Ratio(const T &c) : numerator(c), denominator(1) {}
  inline Ratio(const T &c, const S &s) : numerator(c), denominator(s) {}
  template <intmax_t A, intmax_t B>
  inline Ratio(std::ratio<A, B> r) : numerator(r.num), denominator(r.den) {}
  inline T value() const { return numerator / denominator; }
  inline T value(const T &retIfDenomIsZero) const {
    return denominator == 0 ? retIfDenomIsZero : (numerator / denominator);
  }
  T numerator;
  S denominator;
};

template <class T> struct IsRatio : std::false_type {};

template <class T, class S> struct IsRatio<Ratio<T, S>> : std::true_type {};

template <class Archive, class T, class S>
inline void serialize(Archive &ar, Ratio<T, S> &r) {
  ar(r.numerator, r.denominator);
}

template <class T, class S>
inline Ratio<T, S> MakeRatio(const T &c, const S &s) {
  return Ratio<T, S>(c, s);
}

namespace {
enum RatioOperationType { Plus, Minus, Mult, Div };
template <class T1, class S1, class T2, class S2, RatioOperationType>
struct RatioOperationResult {};
template <class T1, class S1, class T2, class S2>
struct RatioOperationResult<T1, S1, T2, S2, Plus> {
  using type = Ratio<decltype(std::declval<T1>() * std::declval<S2>() +
                              std::declval<T2>() * std::declval<S1>()),
                     decltype(std::declval<S1>() * std::declval<S2>())>;
};
template <class T1, class S1, class T2, class S2>
struct RatioOperationResult<T1, S1, T2, S2, Minus> {
  using type = Ratio<decltype(std::declval<T1>() * std::declval<S2>() -
                              std::declval<T2>() * std::declval<S1>()),
                     decltype(std::declval<S1>() * std::declval<S2>())>;
};
template <class T1, class S1, class T2, class S2>
struct RatioOperationResult<T1, S1, T2, S2, Mult> {
  using type = Ratio<decltype(std::declval<T1>() * std::declval<T2>()),
                     decltype(std::declval<S1>() * std::declval<S2>())>;
};
template <class T1, class S1, class T2, class S2>
struct RatioOperationResult<T1, S1, T2, S2, Div> {
  using type = Ratio<decltype(std::declval<T1>() * std::declval<S2>()),
                     decltype(std::declval<S1>() * std::declval<T2>())>;
};
}

template <class T, class S> inline Ratio<T, S> operator-(const Ratio<T, S> &r) {
  return Ratio<T, S>(-r.numerator, r.denominator);
}

// ratio and ratio
template <class T1, class S1, class T2, class S2>
inline typename RatioOperationResult<T1, S1, T2, S2, Plus>::type
operator+(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return MakeRatio(a.numerator * b.denominator + b.numerator * a.denominator,
                   a.denominator * b.denominator);
}
template <class T1, class S1, class T2, class S2>
inline typename RatioOperationResult<T1, S1, T2, S2, Minus>::type
operator-(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return MakeRatio(a.numerator * b.denominator - b.numerator * a.denominator,
                   a.denominator * b.denominator);
}
template <class T1, class S1, class T2, class S2>
inline typename RatioOperationResult<T1, S1, T2, S2, Mult>::type
operator*(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return MakeRatio(a.numerator * b.numerator, a.denominator * b.denominator);
}
template <class T1, class S1, class T2, class S2>
inline typename RatioOperationResult<T1, S1, T2, S2, Div>::type
operator/(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return MakeRatio(a.numerator * b.denominator, a.denominator * b.numerator);
}

// ratio and scalar
template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline typename RatioOperationResult<T1, S1, T2, S1, Plus>::type
operator+(const Ratio<T1, S1> &a, const T2 &b) {
  return MakeRatio(a.numerator + b * a.denominator, a.denominator);
}
template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline typename RatioOperationResult<T1, S1, T2, S1, Minus>::type
operator-(const Ratio<T1, S1> &a, const T2 &b) {
  return MakeRatio(a.numerator - b * a.denominator, a.denominator);
}
template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline typename RatioOperationResult<T1, S1, T2, S1, Mult>::type
operator*(const Ratio<T1, S1> &a, const T2 &b) {
  return MakeRatio(a.numerator * b, a.denominator);
}
template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline typename RatioOperationResult<T1, S1, T2, S1, Div>::type
operator/(const Ratio<T1, S1> &a, const T2 &b) {
  return MakeRatio(a.numerator, a.denominator * b);
}

// compare
template <class T1, class S1, class T2, class S2>
inline bool operator==(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return a.numerator * b.denominator == a.denominator * b.numerator;
}

template <class T1, class S1, class T2, class S2>
inline bool operator<(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return a.denominator * b.denominator >= 0
             ? (a.numerator * b.denominator < a.denominator * b.numerator)
             : (a.numerator * b.denominator > a.denominator * b.numerator);
}

template <class T1, class S1, class T2, class S2>
inline bool operator>(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return a.denominator * b.denominator >= 0
             ? (a.numerator * b.denominator > a.denominator * b.numerator)
             : (a.numerator * b.denominator < a.denominator * b.numerator);
}

template <class T1, class S1, class T2, class S2>
inline bool operator<=(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return !(a > b);
}

template <class T1, class S1, class T2, class S2>
inline bool operator>=(const Ratio<T1, S1> &a, const Ratio<T2, S2> &b) {
  return !(a < b);
}

// compare with scalar
template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline bool operator==(const Ratio<T1, S1> &a, const T2 &b) {
  return a.numerator == a.denominator * b;
}

template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline bool operator<(const Ratio<T1, S1> &a, const T2 &b) {
  return a.denominator >= 0 ? (a.numerator < a.denominator * b)
                            : (a.numerator > a.denominator * b);
}

template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline bool operator>(const Ratio<T1, S1> &a, const T2 &b) {
  return a.denominator >= 0 ? (a.numerator > a.denominator * b)
                            : (a.numerator < a.denominator * b);
}

template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline bool operator<=(const Ratio<T1, S1> &a, const T2 &b) {
  return !(a > b);
}

template <class T1, class S1, class T2,
          class = std::enable_if_t<!IsRatio<T2>::value>>
inline bool operator>=(const Ratio<T1, S1> &a, const T2 &b) {
  return !(a < b);
}

using Rational = Ratio<double, double>;

template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>>
inline Ratio<T, T> MakeRational(const T &v) {
  return std::isinf(v)
             ? (v > 0 ? Ratio<T, T>(1.0, 0.0) : Ratio<T, T>(-1.0, 0.0))
             : (std::isnan(v) ? Ratio<T, T>(0.0, 0.0) : Ratio<T, T>(v, 1.0));
}
}
}

// some std math functions
namespace std {

template <class T, class S>
inline pano::core::Ratio<T, S> abs(const pano::core::Ratio<T, S> &r) {
  return pano::core::MakeRatio(abs(r.numerator), abs(r.denominator));
}

template <class T, class S>
inline bool isinf(const pano::core::Ratio<T, S> &r) {
  return r.denominator == 0.0 && r.numerator != 0.0;
}

template <class T, class S>
inline bool isnan(const pano::core::Ratio<T, S> &r) {
  return r.denominator == 0.0 && r.numerator == 0.0;
}
}
