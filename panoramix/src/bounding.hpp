#include "decorate.hpp"
#include "geometry.hpp"
#include "image.hpp"
#include "math.hpp"

namespace pano {
namespace core {

// sphere
template <class T, int N> class Sphere {
public:
  Sphere() {}
  Sphere(const Point<T, N> &c, const T &r) : center(c), radius(r) {}

public:
  Point<T, N> center;
  T radius;
};
template <class T, int N>
bool operator==(const Sphere<T, N> &a, const Sphere<T, N> &b) {
  return a.center == b.center && a.radius == b.radius;
}
template <class Archive, class T, int N>
void serialize(Archive &ar, Sphere<T, N> &s) {
  ar(s.center, s.radius);
}

using Circle = Sphere<double, 2>;
using Sphere2 = Sphere<double, 2>;
using Sphere3 = Sphere<double, 3>;

template <int N = 3, class T = double> const Sphere<T, N> &UnitSphere() {
  static const Sphere<T, N> _us = {Point<T, N>(), static_cast<T>(1.0)};
  return _us;
}

namespace {
template <class T, int N>
Vec<T, N> AllMinOf(const Vec<T, N> &v1, const Vec<T, N> &v2) {
  Vec<T, N> v;
  for (int i = 0; i < N; i++)
    v[i] = v1[i] < v2[i] ? v1[i] : v2[i];
  return v;
}
template <class T, int N>
Vec<T, N> AllMaxOf(const Vec<T, N> &v1, const Vec<T, N> &v2) {
  Vec<T, N> v;
  for (int i = 0; i < N; i++)
    v[i] = v1[i] < v2[i] ? v2[i] : v1[i];
  return v;
}
}

// box
template <class T, int N> class Box {
public:
  using Type = T;
  static const int Dimension = N;

public:
  Box() : isNull(true) {}
  Box(const Point<T, N> &c1, const Point<T, N> &c2)
      : minCorner(AllMinOf(c1, c2)), maxCorner(AllMaxOf(c1, c2)),
        isNull(false) {}
  template <class = std::enable_if_t<N == 1>>
  Box(const T &c1, const T &c2)
      : minCorner(std::min(c1, c2)), maxCorner(std::max(c1, c2)),
        isNull(false) {}

  Point<T, N> corner(std::initializer_list<bool> isMaxes) const {
    Point<T, N> c;
    auto it = isMaxes.begin();
    for (int i = 0; i < N; i++) {
      c[i] = (*it) ? maxCorner[i] : minCorner[i];
      ++it;
    }
    return c;
  }

  Vec<T, N> size() const { return maxCorner - minCorner; }
  T size(size_t i) const { return maxCorner[i] - minCorner[i]; }
  T volume() const {
    T v = 1.0;
    for (int i = 0; i < N; i++) {
      v *= (maxCorner[i] - minCorner[i]);
    }
    return v;
  }

  Point<T, N> center() const { return (maxCorner + minCorner) * (0.5); }
  Sphere<T, N> outerSphere() const {
    return Sphere<T, N>{center(),
                        static_cast<T>(norm(maxCorner - minCorner) / 2.0)};
  }
  Box &expand(const Vec<T, N> &s) {
    minCorner -= s;
    maxCorner += s;
    return *this;
  }
  Box &expand(const T &s) {
    for (int i = 0; i < N; i++) {
      minCorner[i] -= s;
      maxCorner[i] += s;
    }
    return *this;
  }

  bool contains(const Point<T, N> &p) const {
    if (isNull)
      return false;
    for (int i = 0; i < N; i++) {
      if (minCorner[i] > p[i] || maxCorner[i] < p[i])
        return false;
    }
    return true;
  }
  bool contains(const Box &b) const {
    return b.isNull ? true : contains(b.minCorner) && contains(b.maxCorner);
  }
  bool operator!=(const Box &b) const { return !(*this == b); }

  Box &operator|=(const Box &b) {
    if (isNull) {
      *this = b;
      return *this;
    }
    if (b.isNull)
      return *this;
    minCorner = AllMinOf(minCorner, b.minCorner);
    maxCorner = AllMaxOf(maxCorner, b.maxCorner);
    return *this;
  }

public:
  Point<T, N> minCorner, maxCorner;
  bool isNull;
};
template <class T, int N>
bool operator==(const Box<T, N> &a, const Box<T, N> &b) {
  return a.isNull ? b.isNull : (!b.isNull && a.minCorner == b.minCorner &&
                                a.maxCorner == b.maxCorner);
}
template <class Archive, class T, int N>
void serialize(Archive &ar, Box<T, N> &b) {
  ar(b.isNull, b.minCorner, b.maxCorner);
}
template <class T, int N>
Box<T, N> operator|(const Box<T, N> &b1, const Box<T, N> &b2) {
  Box<T, N> b12 = b1;
  return b12 |= b2;
}
using Box1 = Box<double, 1>;
using Box2 = Box<double, 2>;
using Box3 = Box<double, 3>;

template <int N = 3, class T = double> const Box<T, N> &UnitBox() {
  static const Box<T, N> _ub(Point<T, N>(), Point<T, N>::ones());
  return _ub;
}

template <class T> struct IsBox : std::false_type {};

template <class T, int N> struct IsBox<Box<T, N>> : std::true_type {};

// for scalars
template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline Box<T, 1> BoundingBox(const T &t) {
  return Box<T, 1>(Point<T, 1>(t), Point<T, 1>(t));
}

template <class T> inline Box<T, 2> BoundingBox(const std::complex<T> &c) {
  return Box<T, 2>(Point<T, 2>(c.real(), c.imag()),
                   Point<T, 2>(c.real(), c.imag()));
}

template <class T, int N> inline Box<T, N> BoundingBox(const Box<T, N> &b) {
  return b;
}

template <class T, int N> inline Box<T, N> BoundingBox(const Point<T, N> &p) {
  return Box<T, N>(p, p);
}

template <class T, int N> inline Box<T, N> BoundingBox(const HPoint<T, N> &hp) {
  return BoundingBox(hp.value());
}

inline Box2 BoundingBox(const Pixel &p) {
  return Box2(Point2(static_cast<double>(p.x), static_cast<double>(p.y)),
              Point2(static_cast<double>(p.x), static_cast<double>(p.y)));
}

inline Box2 BoundingBox(const KeyPoint &p) {
  return Box2(Point2(p.pt.x, p.pt.y), Point2(p.pt.x, p.pt.y));
}

template <class PointT> inline auto BoundingBox(const Line<PointT> &l) {
  return BoundingBox(l.first) | BoundingBox(l.second);
}

template <class PointT>
inline auto BoundingBox(const PositionOnLine<PointT> &p) {
  return BoundingBox(p.position);
}

inline Box2 BoundingBox(const Image &im) {
  return Box2(Point2(0, 0), Point2(im.cols, im.rows));
}

template <class T, int N> inline Box<T, N> BoundingBox(const Sphere<T, N> &s) {
  return Box<T, N>(s.center, s.center).expand(s.radius);
}

template <class PointT, class DirT>
inline auto BoundingBox(const Polygon<PointT, DirT> &p) {
  return BoundingBoxOfRange(b.corners.begin(), b.corners.end());
}

template <class PointT, class DirT>
inline Box3 BoundingBox(const SingleViewPolygon<PointT, DirT> &spp) {
  std::vector<PointT> cs(spp.corners.size());
  for (int i = 0; i < spp.corners.size(); i++) {
    cs[i] =
        Intersection(Ray<PointT, DirT>(spp.projectionCenter,
                                       spp.corners[i] - spp.projectionCenter),
                     spp.plane);
  }
  return BoundingBoxOfContainer(cs);
}

// pointers
template <class T>
inline auto BoundingBox(T const *const ptr) -> decltype(BoundingBox(*ptr)) {
  return BoundingBox(*ptr);
}

template <class T>
inline auto BoundingBox(std::shared_ptr<T> ptr) -> decltype(BoundingBox(*ptr)) {
  return BoundingBox(*ptr);
}

template <class T>
inline auto BoundingBox(std::unique_ptr<T> ptr) -> decltype(BoundingBox(*ptr)) {
  return BoundingBox(*ptr);
}

template <class T>
inline auto BoundingBox(std::weak_ptr<T> ptr) -> decltype(BoundingBox(*ptr)) {
  return BoundingBox(*ptr);
}

// decorators
template <class T, class C>
inline auto BoundingBox(const Classified<T, C> &c)
    -> decltype(BoundingBox(c.component)) {
  return BoundingBox(c.component);
}

template <class T>
inline auto BoundingBox(const Noted<T> &n)
    -> decltype(BoundingBox(n.component)) {
  return BoundingBox(n.component);
}

template <class T, class S>
inline auto BoundingBox(const Scored<T, S> &s)
    -> decltype(BoundingBox(s.component)) {
  return BoundingBox(s.component);
}

template <class T, class D>
inline auto BoundingBox(const Decorated<T, D> &s)
    -> decltype(BoundingBox(s.component)) {
  return BoundingBox(s.component);
}

// return null box if s.enabled == false
template <class T> inline auto BoundingBox(const Enabled<T> &s) {
  using BoxType = decltype(BoundingBox(s.component));
  return s.enabled ? BoundingBox(s.component) : BoxType();
}

// bounding box of range
template <class IterT> auto BoundingBoxOfRange(IterT begin, IterT end);

// bounding box of container
template <class ContainerT>
inline auto BoundingBoxOfContainer(const ContainerT &cont) {
  return BoundingBoxOfRange(std::begin(cont), std::end(cont));
}

template <class T>
inline auto BoundingBoxOfContainer(std::initializer_list<T> ilist) {
  return BoundingBoxOfRange(ilist.begin(), ilist.end());
}

// bounding box of pair-range
template <class PairIterT>
auto BoundingBoxOfPairRange(PairIterT begin, PairIterT end);

// the default bounding box functor
struct DefaultBoundingBoxFunctor {
  template <class T> inline auto operator()(const T &t) const {
    return BoundingBox(t);
  }
};

// the default influence box functor
template <class ScalarT> struct DefaultInfluenceBoxFunctor {
  inline explicit DefaultInfluenceBoxFunctor(const ScalarT &extSz = 0)
      : extendedSize(extSz) {}
  template <class T>
  inline auto operator()(const T &t) const
      -> decltype(BoundingBox(std::declval<T>())) {
    return BoundingBox(t).expand(extendedSize);
  }
  const ScalarT extendedSize;
};
}
}