#pragma once

#include "math.hpp"

namespace pano {
namespace core {

// key point
using KeyPoint = ::cv::KeyPoint;

// size
template <class T> using Size_ = ::cv::Size_<T>;
using Size = Size_<float>;
using Sizei = Size_<int>;

template <class T> double Argument(const Vec<T, 2> &d) {
  return std::atan2(d(1), d(0));
}
template <class T> Vec<T, 2> Direction(double angle) {
  return Vec<T, 2>(cos(angle), sin(angle));
}

// geographic coordinate
class GeoCoord {
public:
  explicit GeoCoord(double longi = 0.0, double lati = 0.0)
      : longitude(longi), latitude(lati) {}
  template <class T>
  GeoCoord(const Vec<T, 3> &d)
      : longitude(std::atan2(d(1), d(0))),
        latitude(std::atan(d(2) / std::sqrt((d(1) * d(1)) + (d(0) * d(0))))) {}
  template <class T = double> Vec<T, 3> toVector() const {
    return Vec<T, 3>(static_cast<T>(cos(longitude) * cos(latitude)),
                     static_cast<T>(sin(longitude) * cos(latitude)),
                     static_cast<T>(sin(latitude)));
  }

  bool operator==(const GeoCoord &b) const {
    return longitude == b.longitude && latitude == b.latitude;
  }
  bool operator!=(const GeoCoord &b) const { return !(*this == b); }
  template <class Archive> void serialize(Archive &ar) {
    ar(longitude, latitude);
  }

public:
  double longitude; // - M_PI ~ + M_PI
  double latitude;  // - M_PI_2 ~ + M_PI_2
};

// ray
template <class PointT, class DirT = PointT> class Ray {
public:
  Ray() {}
  Ray(const PointT &a, const DirT &d) : anchor(a), direction(d) {}

  bool operator==(const Ray &b) const {
    return anchor == b.anchor && direction == b.direction;
  }
  bool operator!=(const Ray &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) {
    ar(anchor, direction);
  }

  template <class B = void> auto coeffs() const -> decltype(GetCoeffs(*this)) {
    return GetCoeffs(*this);
  }

public:
  PointT anchor;
  DirT direction;
};
using Ray2 = Ray<Vec2>;
using Ray3 = Ray<Vec3>;
template <class T> Vec<T, 3> GetCoeffs(const Ray<Vec<T, 2>> &line) {
  return Vec<T, 3>{line.direction[1], -line.direction[0],
                   -(line.direction[1] * line.anchor[0] -
                     line.direction[0] * line.anchor[1])};
}
template <class T> Ray<Vec<T, 2>> Ray2FromCoeffs(const Vec<T, 3> &c) {
  T d = c[0] * c[0] + c[1] * c[1];
  Point<T, 2> anchor(-c[2] * c[0] / d, -c[2] * c[1] / d);
  Vec<T, 2> dir(c[1], -c[0]);
  return Ray<Vec<T, 2>>(anchor, dir);
}

// plane
template <class PointT, class DirT = PointT> class Plane {
public:
  Plane() {}
  Plane(const PointT &a, const DirT &n) : anchor(a), normal(n) {}
  auto root() const {
    return normal * (anchor.dot(normal)) / norm(normal) / norm(normal);
  }
  auto signedDistanceTo(const PointT &p) const {
    return (p - anchor).dot(normalize(normal));
  }
  auto distanceTo(const PointT &p) const {
    return abs((p - anchor).dot(normalize(normal)));
  }

  bool operator==(const Plane &b) const {
    return anchor == b.anchor && normal == b.normal;
  }
  bool operator!=(const Plane &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) { ar(anchor, normal); }

public:
  PointT anchor;
  DirT normal;
};
using Plane3 = Plane<Vec<double, 3>>;

template <class T>
Plane<Vec<T, 3>> Plane3From3Points(const Point<T, 3> &a, const Point<T, 3> &b,
                                   const Point<T, 3> &c) {
  return Plane<Vec<T, 3>>(a, normalize((b - a).cross(c - a)));
}
// ax + by + cz = 1
template <class T> Plane<Vec<T, 3>> Plane3FromEquation(T a, T b, T c) {
  T k = (a * a + b * b + c * c);
  return Plane<Vec<T, 3>>(Point<T, 3>(a, b, c) / k,
                          normalize(Vec<T, 3>(a, b, c)));
}
template <class T> Plane<Vec<T, 3>> Plane3FromEquation(const Vec<T, 3> &equ) {
  return Plane3FromEquation(equ[0], equ[1], equ[2]);
}
template <class T> Vec<T, 3> Plane3ToEquation(const Plane<Vec<T, 3>> &p) {
  auto dotv = p.anchor.dot(p.normal);
  assert(dotv != 0.0);
  return p.normal / dotv;
}

// line
template <class PointT> class Line {
public:
  Line() {}
  Line(const PointT &f, const PointT &s) : first(f), second(s) {}
  auto center() const { return (first + second) / 2.0; }
  auto length() const { return norm(first - second); }
  auto direction() const { return second - first; }
  Line reversed() const { return Line(second, first); }
  Ray<PointT, PointT> ray() const {
    return Ray<PointT, PointT>(first, second - first);
  }

  template <class T>
  std::enable_if_t<std::is_scalar<T>::value, Line &>
  operator*=(const T &factor) {
    first *= factor;
    second *= factor;
    return *this;
  }

  template <class T>
  std::enable_if_t<std::is_scalar<T>::value, Line &>
  operator/=(const T &factor) {
    first /= factor;
    second /= factor;
    return *this;
  }

  Line &operator+=(const PointT &trans) {
    first += trans;
    second += trans;
    return *this;
  }
  Line &operator-=(const PointT &trans) {
    first -= trans;
    second -= trans;
    return *this;
  }

  bool operator==(const Line &b) const {
    return first == b.first && second == b.second;
  }
  bool operator!=(const Line &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) { ar(first, second); }

public:
  PointT first, second;
};

template <class PointT, class T>
std::enable_if_t<std::is_scalar<T>::value, Line<PointT>>
operator*(const Line<PointT> &line, const T &factor) {
  return Line<PointT>(line.first * factor, line.second * factor);
}
template <class PointT, class T>
std::enable_if_t<std::is_scalar<T>::value, Line<PointT>>
operator/(const Line<PointT> &line, const T &factor) {
  return Line<PointT>(line.first / factor, line.second / factor);
}
template <class PointT, class T>
std::enable_if_t<std::is_scalar<T>::value, Line<PointT>>
operator*(const T &factor, const Line<PointT> &line) {
  return Line<PointT>(line.first * factor, line.second * factor);
}
template <class PointT>
Line<PointT> operator+(const Line<PointT> &line, const PointT &trans) {
  return Line<PointT>(line.first + trans, line.second + trans);
}
template <class PointT>
Line<PointT> operator-(const Line<PointT> &line, const PointT &trans) {
  return Line<PointT>(line.first - trans, line.second - trans);
}
using Line2 = Line<Point<double, 2>>;
using Line3 = Line<Point<double, 3>>;

template <class PointT> Line<PointT> normalize(const Line<PointT> &line) {
  return Line<PointT>(normalize(line.first), normalize(line.second));
}

template <class T> struct IsLine : std::false_type {};

template <class PointT> struct IsLine<Line<PointT>> : std::true_type {};

// position on line/infline
template <class PointT> class PositionOnLine {
public:
  PositionOnLine() {}
  PositionOnLine(const Line<PointT> &line, double r)
      : ratio(r), position(line.first + (line.second - line.first) * ratio) {}
  PositionOnLine(const Ray<PointT, PointT> &line, double r)
      : ratio(r), position(line.anchor + line.direction * ratio) {}

  bool operator==(const PositionOnLine &b) const {
    return ratio == b.ratio && position == b.position;
  }
  bool operator!=(const PositionOnLine &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) { ar(ratio, position); }

public:
  double ratio; // [0 ~ 1] on line, or [-inf, +inf] on infinite line
  // position = line.first + (line.second - line.fist) * ratio
  // or       = line.anchor + line.direction * ratio
  PointT position;
};
using PositionOnLine2 = PositionOnLine<Vec<double, 2>>;
using PositionOnLine3 = PositionOnLine<Vec<double, 3>>;

// chain
template <class PointT> class Chain {
public:
  Chain() : closed(true) {}
  explicit Chain(const std::vector<PointT> &ps, bool c = true)
      : points(ps), closed(c) {}
  explicit Chain(std::vector<PointT> &&ps, bool c = true)
      : points(std::move(ps)), closed(c) {}
  template <class IterT>
  Chain(IterT psBegin, IterT psEnd, bool c = true)
      : points(psBegin, psEnd), closed(c) {}

  const PointT &at(size_t i) const { return points.at(i % points.size()); }
  const PointT &prev(size_t i) const {
    return points.at((i - 1 + points.size()) % points.size());
  }
  const PointT &next(size_t i) const {
    return points.at((i + 1) % points.size());
  }

  Line<PointT> edge(size_t i) const { return Line<PointT>(at(i), next(i)); }

  const PointT &operator[](size_t i) const { return points[i]; }
  PointT &operator[](size_t i) { return points[i]; }

  size_t size() const { return points.size(); }
  void clear() { points.clear(); }
  bool empty() const { return points.empty(); }

  void append(const PointT &p) { points.push_back(p); }
  void insert(size_t i, const PointT &p) {
    points.insert(points.begin() + (i % points.size()), p);
  }

  double length() const {
    if (points.empty())
      return 0.0;
    double len = 0;
    for (int i = 0; i + 1 < points.size(); i++) {
      len += norm(points[i] - points[i + 1]);
    }
    if (closed) {
      len += norm(points.front() - points.back());
    }
    return len;
  }

  template <class PointT2, class = std::enable_if_t<
                               !std::is_same<PointT, PointT2>::value &&
                               std::is_convertible<PointT, PointT2>::value>>
  operator Chain<PointT2>() const {
    Chain<PointT2> c;
    c.points.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
      c.points[i] = points[i];
    }
    c.closed = closed;
    return c;
  }

  template <class FunT> void fixedStepSample(double stepLen, FunT &&fun) const {
    if (empty()) {
      return;
    }
    fun(points.front());
    auto last = points.front();
    for (int i = 1; i < points.size(); i++) {
      double dist = Distance(last, points[i]);
      auto dir = normalize(points[i] - last);
      while (dist >= stepLen) {
        auto p = last + dir * stepLen;
        fun(p);
        dist -= stepLen;
        last = p;
      }
    }
  }

  bool operator==(const Chain &b) const {
    return points == b.points && closed == b.closed;
  }
  bool operator!=(const Chain &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) { ar(points, closed); }

public:
  std::vector<PointT> points;
  bool closed;
};

template <class PointT> Chain<PointT> normalize(const Chain<PointT> &c) {
  auto r = c;
  for (auto &p : c.points) {
    p = normalize(p);
  }
  return r;
}
using Chain2i = Chain<Point<int, 2>>;
using Chain2 = Chain<Point<double, 2>>;
using Chain3 = Chain<Point<double, 3>>;

// polygon
template <class PointT, class DirT = PointT> class Polygon {
public:
  Polygon() {}
  Polygon(const std::vector<PointT> &cs, const DirT &n)
      : corners(cs), normal(n) {}
  Polygon(std::vector<PointT> &&cs, const DirT &n)
      : corners(std::move(cs)), normal(n) {}
  Polygon(const Chain<PointT> &c)
      : corners(c.points),
        normal(normalize((corners.at(0) - corners.at(1))
                             .cross(corners.at(1) - corners.at(2)))) {}
  Polygon(Chain<PointT> &&c)
      : corners(std::move(c.points)),
        normal(normalize((corners.at(0) - corners.at(1))
                             .cross(corners.at(1) - corners.at(2)))) {}
  Plane<PointT, DirT> plane() const {
    return Plane<PointT, DirT>(corners.front(), normal);
  }
  Chain<PointT> boundary() const { return Chain<PointT>{corners, true}; }

  template <
      class PointT2, class DirT2,
      class = std::enable_if_t<(!std::is_same<PointT, PointT2>::value ||
                                !std::is_same<DirT, DirT2>::value) &&
                               std::is_constructible<PointT, PointT2>::value &&
                               std::is_convertible<DirT, DirT2>::value>>
  operator Polygon<PointT2, DirT2>() const {
    std::vector<PointT2> ps(corners.size());
    for (int i = 0; i < corners.size(); i++)
      ps[i] = corners[i];
    return Polygon<PointT2, DirT2>(std::move(ps), normal);
  }

  template <class = std::enable_if_t<N == 3>> auto area() const {
    return Area(*this);
  }

  bool operator==(const Polygon &b) const {
    return corners == b.corners && normal == b.normal;
  }
  bool operator!=(const Polygon &b) const { return !(*this == b); }

  template <class Archive> void serialize(Archive &ar) { ar(corners, normal); }

public:
  std::vector<PointT> corners;
  DirT normal;
};
using Polygon2 = Polygon<Point<double, 2>>;
using Polygon2f = Polygon<Point<float, 2>>;
using Polygon3 = Polygon<Point<double, 3>>;
using Polygon3f = Polygon<Point<float, 3>>;

template <class PointT>
Polygon<PointT> MakeTriangle(const PointT &p1, const PointT &p2,
                             const PointT &p3) {
  return Polygon<PointT>{{p1, p2, p3}, (p2 - p1).cross(p3 - p1)};
}

inline double PointTest(const Polygon2f &poly, const Point2f &p) {
  return cv::pointPolygonTest(poly.corners, p, true);
}
inline bool Contains(const Polygon2f &poly, const Point2f &p) {
  return cv::pointPolygonTest(poly.corners, p, false) >= 0;
}

// layered polygons
template <class PointT, class DirT = PointT> class LayeredShape {
public:
  Polygon<PointT, DirT> layer(size_t i) const {
    return Polygon<PointT, DirT>(layers.at(i), normal);
  }
  size_t size() const { return layers.size(); }
  bool empty() const { return layers.empty(); }

  template <class Archive> void serialize(Archive &ar) { ar(layers, normal); }

public:
  std::vector<std::vector<PointT>> layers;
  DirT normal;
};
using LayeredShape3 = LayeredShape<Point<double, 3>>;

// SingleViewPolygon
template <class PointT, class DirT = PointT> class SingleViewPolygon {
public:
  template <class Archive> void serialize(Archive &ar) {
    ar(corners, projection_center, plane);
  }

public:
  std::vector<PointT> corners;
  PointT projection_center;
  Plane<PointT, DirT> plane;
};
using SingleViewPolygon3 = SingleViewPolygon<Point<double, 3>>;
}
}