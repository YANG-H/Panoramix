#pragma once

#include <string>
#include <vector>

namespace pano {
namespace core {

template <class T> struct IsDecorated : no {};

// decorated without functional
template <class T, class D> struct Decorated {
  using ComponentType = T;
  using DecorationType = D;
  D decoration;
  T component;
  operator T() const { return component; }
  template <class K> Decorated<std::decay_t<K>, D> converted(K &&k) const {
    return {decoration, std::forward<K>(k)};
  }
};
template <class T, class D> struct IsDecorated<Decorated<T, D>> : yes {};

template <class T, class D>
inline Decorated<std::decay_t<T>, std::decay_t<D>> DecorateAs(T &&comp, D &&d) {
  return Decorated<std::decay_t<T>, std::decay_t<D>>{std::forward<D>(d),
                                                     std::forward<T>(comp)};
}
template <class T, class D>
inline bool operator==(const Decorated<T, D> &a, const Decorated<T, D> &b) {
  return a.component == b.component;
}
template <class T, class D>
inline bool operator<(const Decorated<T, D> &a, const Decorated<T, D> &b) {
  return a.component < b.component;
}
template <class Archive, class T, class D>
inline void serialize(Archive &ar, Decorated<T, D> &c) {
  ar(c.decorated, c.component);
}

// something classified
template <class T, class ClassT = int> struct Classified {
  using ComponentType = T;
  using DecorationType = ClassT;
  ClassT claz;
  T component;
  template <class K>
  Classified<std::decay_t<K>, DecorationType> converted(K &&k) const {
    return {claz, std::forward<K>(k)};
  }
};
template <class T, class ClassT>
struct IsDecorated<Classified<T, ClassT>> : yes {};

template <class T, class ClassT>
inline Classified<std::decay_t<T>, std::decay_t<ClassT>>
ClassifyAs(T &&comp, ClassT &&claz) {
  return Classified<std::decay_t<T>, std::decay_t<ClassT>>{
      std::forward<ClassT>(claz), std::forward<T>(comp)};
}
template <class T, class ClassT>
inline std::vector<Classified<T, ClassT>>
ClassifyEachAs(const std::vector<T> &comps, const ClassT &claz) {
  std::vector<Classified<T, ClassT>> cs(comps.size());
  for (int i = 0; i < comps.size(); i++)
    cs[i] = ClassifyAs(comps[i], claz);
  return cs;
}
template <class T, class ClassT>
inline std::vector<Classified<T, ClassT>> ClassifyEachAs(std::vector<T> &&comps,
                                                         const ClassT &claz) {
  std::vector<Classified<T, ClassT>> cs(comps.size());
  for (int i = 0; i < comps.size(); i++)
    cs[i] = ClassifyAs(std::move(comps[i]), claz);
  return cs;
}
template <class T, class ClassT>
inline bool operator==(const Classified<T, ClassT> &a,
                       const Classified<T, ClassT> &b) {
  return a.claz == b.claz && a.component == b.component;
}
template <class Archive, class T, class ClassT>
inline void serialize(Archive &ar, Classified<T, ClassT> &c) {
  ar(c.claz, c.component);
}

// something noted
template <class T> struct Noted {
  using ComponentType = T;
  using DecorationType = std::string;
  std::string note;
  T component;
  template <class K> Noted<std::decay_t<K>> converted(K &&k) const {
    return {note, std::forward<K>(k)};
  }
};
template <class T> struct IsDecorated<Noted<T>> : yes {};

template <class T, class StringT>
inline Noted<std::decay_t<T>> NoteAs(T &&comp, StringT &&note) {
  return Noted<std::decay_t<T>>{std::forward<StringT>(note),
                                std::forward<T>(comp)};
}
template <class T>
inline bool operator==(const Noted<T> &a, const Noted<T> &b) {
  return a.note == b.note && a.component == b.component;
}
template <class Archive, class T>
inline void serialize(Archive &ar, Noted<T> &c) {
  ar(c.note, c.component);
}

// something scored for sorting
template <class T, class S = double> struct Scored {
  using ComponentType = T;
  using DecorationType = S;
  template <class K> using Converted = Scored<K, S>;
  S score;
  T component;
  const S &weight() const { return score; }
  S &weight() { return score; }
  template <class K> Scored<std::decay_t<K>, S> converted(K &&k) const {
    return Scored<std::decay_t<K>, S>{score, std::forward<K>(k)};
  }
};
template <class T, class S> struct IsDecorated<Scored<T, S>> : yes {};

template <class T, class S = double> using Weighted = Scored<T, S>;

template <class T, class S>
inline Scored<std::decay_t<T>, std::decay_t<S>> ScoreAs(T &&comp, S &&score) {
  return Scored<std::decay_t<T>, std::decay_t<S>>{std::forward<S>(score),
                                                  std::forward<T>(comp)};
}
template <class T, class S>
inline Scored<std::decay_t<T>, std::decay_t<S>> WeightAs(T &&comp, S &&score) {
  return Scored<std::decay_t<T>, std::decay_t<S>>{std::forward<S>(score),
                                                  std::forward<T>(comp)};
}

// note this!!!
template <class T, class S>
inline bool operator==(const Scored<T, S> &a, const Scored<T, S> &b) {
  return a.score == b.score && a.component == b.component;
}

// score comparison
template <class T, class S>
inline bool operator<(const Scored<T, S> &a, const Scored<T, S> &b) {
  return a.score < b.score;
}
template <class T, class S>
inline bool operator>(const Scored<T, S> &a, const Scored<T, S> &b) {
  return a.score > b.score;
}
template <class T, class S>
inline bool operator<=(const Scored<T, S> &a, const Scored<T, S> &b) {
  return a.score <= b.score;
}
template <class T, class S>
inline bool operator>=(const Scored<T, S> &a, const Scored<T, S> &b) {
  return a.score >= b.score;
}
template <class Archive, class T, class S>
inline void serialize(Archive &ar, Scored<T, S> &c) {
  ar(c.score, c.component);
}

// bounded
template <class T, class B = double> struct Bounded {
  using ComponentType = T;
  using DecorationType = std::pair<B, B>;
  template <class K> using Converted = Bounded<K, B>;
  std::pair<B, B> bound;
  T component;
  const B &lowerBound() const { return bound.first; }
  const B &upperBound() const { return bound.second; }
  template <class K> Bounded<std::decay_t<K>, B> converted(K &&k) const {
    return {bound, std::forward<K>(k)};
  }
};
template <class T, class B> struct IsDecorated<Bounded<T, B>> : yes {};

template <class T, class B1, class B2>
inline Bounded<std::decay_t<T>, std::decay_t<B1>> BoundAs(T &&comp, B1 &&lb,
                                                          B2 &&ub) {
  return Bounded<std::decay_t<T>, std::decay_t<B1>>{
      std::make_pair(std::forward<B1>(lb), std::forward<B2>(ub)),
      std::forward<T>(comp)};
}

template <class Archive, class T, class B>
inline void serialize(Archive &ar, Bounded<T, B> &c) {
  ar(c.bound, c.component);
}

// something enabled
template <class T> struct Enabled {
  using ComponentType = T;
  using DecorationType = bool;
  template <class K> using Converted = Enabled<K>;
  bool enabled;
  T component;
  template <class K> Enabled<std::decay_t<K>> converted(K &&k) const {
    return {enabled, std::forward<K>(k)};
  }
};
template <class T> struct IsDecorated<Enabled<T>> : yes {};

template <class T>
inline Enabled<std::decay_t<T>> EnableAs(T &&comp, bool e = true) {
  return Enabled<std::decay_t<T>>{e, std::forward<T>(comp)};
}
template <class T>
inline bool operator==(const Enabled<T> &a, const Enabled<T> &b) {
  return a.enabled == b.enabled && a.component == b.component;
}
template <class Archive, class T>
inline void serialize(Archive &ar, Enabled<T> &c) {
  ar(c.enabled, c.component);
}
}
}
