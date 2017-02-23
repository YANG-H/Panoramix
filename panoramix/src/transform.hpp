#pragma once

#include "geometry.hpp"

namespace pano {
namespace gui {

using namespace ::pano::core;

// transformed in 3d space
template <class T, class E = double> struct TransformedIn3D {
  T component;
  Mat<E, 4, 4> mat4;

  TransformedIn3D() : mat4(Mat<E, 4, 4>::eye()) {}
  TransformedIn3D(const T &c, const Mat<E, 4, 4> &m = Mat<E, 4, 4>::eye())
      : component(c), mat4(m) {}
  TransformedIn3D(T &&c, const Mat<E, 4, 4> &m = Mat<E, 4, 4>::eye())
      : component(std::move(c)), mat4(m) {}

  TransformedIn3D &translate(const Vec<E, 3> &t) {
    mat4 = MakeMat4Translate(t) * mat4;
    return *this;
  }
  TransformedIn3D &rotate(const Vec<E, 3> &axis, E angle) {
    mat4 = MakeMat4Rotate(axis, angle) * mat4;
    return *this;
  }
  TransformedIn3D &scale(E s) {
    mat4 = MakeMat4Scale(s) * mat4;
    return *this;
  }
  TransformedIn3D &scale(E sx, E sy, E sz) {
    mat4 = MakeMat4Scale(sx, sy, sz) * mat4;
    return *this;
  }
  TransformedIn3D &reset() {
    mat4 = Mat<E, 4, 4>::eye();
    return *this;
  }

  Point<E, 3> toWorld(const Point<E, 3> &p) const {
    Vec<E, 4> c = mat4 * cat(p, 1.0);
    return Point<E, 3>(c[0] / c[3], c[1] / c[3], c[2] / c[3]);
  }
  Point<E, 3> toLocal(const Point<E, 3> &p) const {
    Vec<E, 4> c = mat4 * cat(p, 1.0);
    Vec<E, 4> localc;
    bool solvable = cv::solve(mat4, c, localc);
    assert(solvable);
    return Point<E, 3>(localc[0] / localc[3], localc[1] / localc[3],
                       localc[2] / localc[3]);
  }

  template <class = std::enable_if_t<std::is_same<T, Line<E, 3>>::value>>
  Point<E, 3> first() const {
    return toWorld(component.first);
  }
  template <class = std::enable_if_t<std::is_same<T, Line<E, 3>>::value>>
  Point<E, 3> second() const {
    return toWorld(component.second);
  }
  template <class = std::enable_if_t<std::is_same<T, Box<E, 3>>::value>>
  Point<E, 3> corner(std::initializer_list<bool> isMaxes) const {
    return toWorld(component.corner(isMaxes));
  }
};

template <class E = double, class T>
TransformedIn3D<std::decay_t<T>, E> MakeTransformableIn3D(T &&c) {
  return TransformedIn3D<std::decay_t<T>, E>(std::forward<T>(c));
}

template <class E = double, class T>
TransformedIn3D<std::decay_t<T>, E>
AsInLocalCoordinates(T &&c, const Vec<E, 3> &x, const Vec<E, 3> &y,
                     const Vec<E, 3> &z, const Point<E, 3> &o) {
  return TransformedIn3D<std::decay_t<T>, E>(std::forward<T>(c),
                                             MakeMat4LocalToWorld(x, y, z, o));
}

template <class T, class E>
inline bool operator==(const TransformedIn3D<T, E> &a,
                       const TransformedIn3D<T, E> &b) {
  return a.component == b.component && a.mat4 == b.mat4;
}
template <class Archive, class T, class E>
inline void serialize(Archive &ar, TransformedIn3D<T, E> &b) {
  ar(b.component, b.mat4);
}
}
}
