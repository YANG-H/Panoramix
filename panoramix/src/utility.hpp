#pragma once

#include <Eigen/Dense>
#include <iterator>

#include "rtree.h"

#include "basic_types.hpp"

namespace pano {
namespace core {

// PrintTo
inline std::ostream &PrintTo(std::ostream &os) { return os; }
template <class T, class... Ts>
inline std::ostream &PrintTo(std::ostream &os, const T &arg,
                             const Ts &... args) {
  os << arg;
  return PrintTo(os, args...);
}
// PrintToWithSeperator
template <class SepT>
inline std::ostream &PrintToWithSeperator(std::ostream &os, SepT &&sep) {
  return os;
}
template <class SepT, class T>
inline std::ostream &PrintToWithSeperator(std::ostream &os, SepT &&sep,
                                          const T &arg) {
  return os << arg;
}
template <class SepT, class T, class... Ts>
inline std::ostream &PrintToWithSeperator(std::ostream &os, SepT &&sep,
                                          const T &arg, const Ts &... args) {
  os << arg << sep;
  return PrintToWithSeperator(os, sep, args...);
}

// Print
template <class... Ts> inline std::ostream &Print(const Ts &... args) {
  return PrintTo(std::cout, args...);
}
// PrintWithSeperator
template <class SepT, class... Ts>
inline std::ostream &PrintWithSeperator(SepT &&sep, const Ts &... args) {
  return PrintToWithSeperator(std::cout, sep, args...);
}
// Println
template <class... Ts> inline std::ostream &Println(const Ts &... args) {
  return Print(args...) << '\n';
}
// PrintWithSeperator
template <class SepT, class... Ts>
inline std::ostream &PrintlnWithSeperator(SepT &&sep, const Ts &... args) {
  return PrintToWithSeperator(std::cout, sep, args...) << '\n';
}

// test value
// can be used to check whether NaN exists by invoking: HasValue(a, std::isnan)
template <class TesterT> inline bool HasValue(double v, TesterT tester) {
  return tester(v);
}

template <class T, int N, class TesterT>
inline bool HasValue(const Vec<T, N> &v, TesterT tester) {
  for (int i = 0; i < N; i++) {
    if (tester(v[i]))
      return true;
  }
  return false;
}

template <class T, class S, class TesterT>
inline bool HasValue(const Ratio<T, S> &r, TesterT tester) {
  return HasValue(r.numerator, tester) || HasValue(r.denominator, tester);
}

template <class PointT, class TesterT>
inline bool HasValue(const Line<PointT> &v, TesterT tester) {
  return HasValue(v.first, tester) || HasValue(v.second, tester);
}

template <class PointT, class DirT, class TesterT>
inline bool HasValue(const Ray<PointT, DirT> &v, TesterT tester) {
  return HasValue(v.anchor, tester) || HasValue(v.direction, tester);
}

template <class PointT, class DirT, class TesterT>
inline bool HasValue(const Plane<PointT, DirT> &p, TesterT tester) {
  return HasValue(p.anchor, tester) || HasValue(p.normal, tester);
}

template <class T, int N, class TesterT>
inline bool HasValue(const Sphere<T, N> &s, TesterT tester) {
  return HasValue(s.center, tester) || HasValue(s.radius, tester);
}

template <class T, int N, class TesterT>
inline bool HasValue(const Box<T, N> &b, TesterT tester) {
  return HasValue(b.minCorner, tester) || HasValue(b.maxCorner, tester);
}

template <class T1, class T2, class TesterT>
inline bool HasValue(const std::pair<T1, T2> &p, TesterT tester) {
  return HasValue(p.first, tester) || HasValue(p.second, tester);
}

template <class T, class AllocT, class TesterT>
inline bool HasValue(const std::vector<T, AllocT> &v, TesterT tester) {
  for (auto &&e : v) {
    if (HasValue(e, tester))
      return true;
  }
  return false;
}

template <class T, class TesterT>
inline bool HasValue(std::initializer_list<T> list, TesterT tester) {
  for (auto &&e : list) {
    if (HasValue(e, tester))
      return true;
  }
  return false;
}

template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>>
inline bool IsInfOrNaN(const T &v) {
  return std::isinf(v) || std::isnan(v);
}

template <class T> inline T NonZeroize(T d, T epsilon = 1e-6) {
  assert(epsilon >= 0);
  return -epsilon < d && d < 0 ? -epsilon
                               : (0 < d && d < +epsilon ? +epsilon : d);
}

// contains
template <class ContainerT, class ValueT,
          class = std::enable_if_t<IsContainer<ContainerT>::value>>
inline bool Contains(const ContainerT &c, const ValueT &v) {
  return std::find(std::begin(c), std::end(c), v) != std::end(c);
}

template <class T, class ValueT>
inline bool Contains(std::initializer_list<T> ilist, const ValueT &v) {
  return std::find(ilist.begin(), ilist.end(), v) != ilist.end();
}

template <class KeyT, class ValueT, class PredT, class AllocT>
inline bool Contains(const std::map<KeyT, ValueT, PredT, AllocT> &m,
                     const KeyT &k) {
  return m.find(k) != m.end();
}

template <class KeyT, class PredT, class AllocT>
inline bool Contains(const std::set<KeyT, PredT, AllocT> &m, const KeyT &k) {
  return m.find(k) != m.end();
}

template <class KeyT, class ValueT, class HasherT, class KeyeqT, class AllocT>
inline bool
Contains(const std::unordered_map<KeyT, ValueT, HasherT, KeyeqT, AllocT> &m,
         const KeyT &k) {
  return m.find(k) != m.end();
}

template <class KeyT, class HasherT, class KeyeqT, class AllocT>
inline bool Contains(const std::unordered_set<KeyT, HasherT, KeyeqT, AllocT> &m,
                     const KeyT &k) {
  return m.find(k) != m.end();
}

template <class T>
inline std::enable_if_t<std::is_integral<T>::value, bool>
Contains(const Size_<T> &sz, const Pixel &pixel) {
  return 0 <= pixel.x && pixel.x < sz.width && 0 <= pixel.y &&
         pixel.y < sz.height;
}

// all same
template <class IterT, class EqualT = std::equal_to<void>>
inline bool AllSameInRange(IterT begin, IterT end, EqualT eq = EqualT()) {
  for (auto i = begin; i != end; ++i) {
    if (!eq(*i, *begin))
      return false;
  }
  return true;
}

template <class ContainerT, class EqualT = std::equal_to<void>>
inline bool AllSameInContainer(const ContainerT &c, EqualT eq = EqualT()) {
  return AllSameInRange(std::begin(c), std::end(c), eq);
}

template <class T, class EqualT = std::equal_to<void>>
inline bool AllSameInContainer(std::initializer_list<T> ilist,
                               EqualT eq = EqualT()) {
  return AllSameInRange(ilist.begin(), ilist.end(), eq);
}

// zero
template <class T> inline bool IsFuzzyZero(T t, T epsilon = 1e-8) {
  return t < epsilon && t > -epsilon;
}

// a safer mean
template <class T> inline T Mean(T a, T b) { return a + (b - a) / 2; }

// squared
template <class T> inline T Square(T v) { return v * v; }

// gaussian
template <class T, class K> inline T Gaussian(T x, K sigma) {
  return std::exp(-Square(x / sigma) / 2.0);
}

// pitfall
template <class T, class K> inline T Pitfall(T x, K sigma) {
  return abs(x) <= sigma ? Square(x / sigma) : 1;
}

// entropy [-factor, 0]
template <class IterT, class T = double>
inline T EntropyOfRange(IterT begin, IterT end, const T &factor = 1.0) {
  T e = 0;
  while (begin != end) {
    auto &v = *begin;
    auto ve = (v * log2(v));
    e -= (std::isnan(ve) ? 0.0 : ve);
    ++begin;
  }
  return e * factor;
}

template <class ContainerT, class T = double>
inline T EntropyOfContainer(const ContainerT &cont, const T &factor = 1.0) {
  return EntropyOfRange(std::begin(cont), std::end(cont), factor);
}

// mean squared deviation
template <class IterT> auto MeanSquaredDeviation(IterT begin, IterT end) {
  using T = typename std::iterator_traits<IterT>::value_type;
  auto n = std::distance(begin, end);
  T mean = std::accumulate(begin, end, T(0)) / double(n);
  T msd = 0;
  while (begin != end) {
    msd += Square(*begin - mean);
    ++begin;
  }
  return msd / double(n);
}

template <class ContT> auto MeanSquaredDeviationOfContainer(const ContT &cont) {
  return MeanSquaredDeviation(std::begin(cont), std::end(cont));
}
template <class IterT, class T>
auto MeanSquaredDeviation(IterT begin, IterT end,
                          std::initializer_list<T> vals) {
  auto n = std::distance(begin, end);
  T msd = 0;
  while (begin != end) {
    auto min_dev = std::numeric_limits<T>::max();
    for (auto val : vals) {
      auto dev = Square(*begin - val);
      if (dev < min_dev) {
        min_dev = dev;
      }
    }
    msd += min_dev;
    ++begin;
  }
  return msd / double(n);
}
template <class ContT, class T>
auto MeanSquaredDeviationOfContainer(const ContT &cont,
                                     std::initializer_list<T> vals) {
  return MeanSquaredDeviation(std::begin(cont), std::end(cont), vals);
}

template <class IterT, class T, class K>
auto MeanSquaredDeviation(IterT begin, IterT end, std::initializer_list<T> vals,
                          K clamp) {
  auto n = std::distance(begin, end);
  T msd = 0;
  while (begin != end) {
    auto min_dev = std::numeric_limits<T>::max();
    for (auto val : vals) {
      auto diff = abs(*begin - val);
      if (clamp < diff) {
        diff = clamp;
      }
      auto dev = Square(diff);
      if (dev < min_dev) {
        min_dev = dev;
      }
    }
    msd += min_dev;
    ++begin;
  }
  return msd / double(n);
}
template <class ContT, class T, class K>
auto MeanSquaredDeviationOfContainer(const ContT &cont,
                                     std::initializer_list<T> vals, K clamp) {
  return MeanSquaredDeviation(std::begin(cont), std::end(cont), vals, clamp);
}

// Normalize
template <class IterT> void Normalize(IterT begin, IterT end) {
  double norm = 0.0;
  for (IterT i = begin; i != end; ++i) {
    norm += Square(*i);
  }
  norm = sqrt(norm);
  for (IterT i = begin; i != end; ++i) {
    (*i) /= norm;
  }
}

// L2Distance
template <class IterT1, class IterT2>
auto L2Distance(IterT1 begin1, IterT1 end1, IterT2 begin2, IterT2 end2) {
  assert(std::distance(begin1, end1) == std::distance(begin2, end2));
  typename std::iterator_traits<IterT1>::value_type d = 0.0;
  while (begin1 != end1 && begin2 != end2) {
    d += Square(*begin1 - *begin2);
    ++begin1;
    ++begin2;
  }
  return sqrt(d);
}

// degrees to radians
template <class T> inline double DegreesToRadians(T degrees) {
  return degrees * M_PI / 180.0;
}

/// distance functions
// for real numbers
template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline T Distance(T a, T b) {
  return std::abs(a - b);
}

// for complex numbers
template <class T>
inline T Distance(const std::complex<T> &a, const std::complex<T> &b) {
  return std::abs(a - b);
}

template <class T, int N>
inline T Distance(const Point<T, N> &a, const Point<T, N> &b) {
  return norm(a - b);
}

inline double Distance(const Pixel &a, const Pixel &b) {
  return sqrt(Square(a.x - b.x) + Square(a.y - b.y));
}

inline double Distance(const KeyPoint &a, const KeyPoint &b) {
  return sqrt(Square(a.pt.x - b.pt.x) + Square(a.pt.y - b.pt.y));
}

template <class T, int N>
inline Ratio<T, T> Distance(const HPoint<T, N> &a, const HPoint<T, N> &b) {
  THERE_ARE_BUGS_HERE("not precise!");
  return norm(a - b);
}

template <class PointT>
inline auto Distance(const PositionOnLine<PointT> &a,
                     const PositionOnLine<PointT> &b) {
  return norm(a.position - b.position);
}

inline double Distance(const cv::Scalar &a, const cv::Scalar &b) {
  return norm(a - b);
}

// standard distance functor
struct DefaultDistanceFunctor {
  template <class T>
  inline auto operator()(const T &a, const T &b) const
      -> decltype(Distance(std::declval<T>(), std::declval<T>())) {
    return Distance(a, b);
  }
};

// AccumulatedLength
template <class IterT, class T = double>
inline T AccumulatedLength(IterT begin, IterT end, T base = (T)0) {
  for (auto it = begin, it2 = std::next(begin); it2 != end; ++it, ++it2) {
    base += Distance(*it, *it2);
  }
  return base;
}

// tools
template <class T, class K = double>
inline bool FuzzyEquals(const T &a, const T &b,
                        const K &epsilon = std::numeric_limits<K>::epsilon()) {
  return Distance(a, b) <= epsilon; // not only numerics
}

template <class T, class K = double>
inline int DiracDelta(const T &v,
                      const K &epsilon = std::numeric_limits<K>::epsilon()) {
  return std::abs(v) <= epsilon ? 1 : 0;
}

// [low, high] for float v, [low, high) for nonfloat v
template <class T, class K1, class K2>
inline bool IsBetween(const T &v, const K1 &low, const K2 &high);

// [low, high]
template <class T, class K1, class K2>
inline T BoundBetween(const T &v, const K1 &low, const K2 &high) {
  if (v < low)
    return low;
  return v < high ? v : high;
}

// returns [low, high)
template <class T>
inline T WrapBetween(const T &input, const T &low, const T &high);

template <class T, int N>
T EncodeSubscriptToIndex(const Point<T, N> &subscript,
                         const Vec<T, N> &dimension);

inline int EncodeSubscriptToIndex(const Pixel &p, const Sizei &size) {
  return p.x * size.height + p.y;
}

template <class T, int N>
inline Point<T, N> DecodeIndexToSubscript(T index, const Vec<T, N> &dimension);

inline Pixel DecodeIndexToSubscript(int index, const Sizei &size) {
  return Pixel(index / size.height, index % size.height);
}

// for vectors
template <class T> inline Vec<T, 2> PerpendicularDirection(const Vec<T, 2> &d) {
  return Vec<T, 2>(-d(1), d(0));
}

template <class T>
inline Vec<T, 2> LeftPerpendicularDirectiion(const Vec<T, 2> &d) {
  return Vec<T, 2>(-d(1), d(0));
}

template <class T>
inline Vec<T, 2> RightPerpendicularDirectiion(const Vec<T, 2> &d) {
  return Vec<T, 2>(d(1), -d(0));
}

template <class T>
inline std::pair<Vec<T, 3>, Vec<T, 3>>
ProposeXYDirectionsFromZDirection(const Vec<T, 3> &z);

// returns [0, pi]
template <class T, int N>
inline T AngleBetweenDirected(const Vec<T, N> &v1, const Vec<T, N> &v2) {
  auto s = v1.dot(v2) / norm(v1) / norm(v2);
  return s >= 1.0 - 1e-9 ? 0.0 : (s <= -1.0 + 1e-9 ? M_PI : acos(s));
}

// returns [0, pi/2]
template <class T, int N>
inline T AngleBetweenUndirected(const Vec<T, N> &v1, const Vec<T, N> &v2) {
  auto s = abs(v1.dot(v2) / norm(v1) / norm(v2));
  return s >= 1.0 ? 0.0 : acos(s);
}

template <class T>
inline T SignedAngle(const Vec<T, 2> &from, const Vec<T, 2> &to,
                     bool clockwiseAsPositive = true) {
  double angle = atan2(-from(0) * to(1) + to(0) * from(1),
                       from(1) * to(1) + from(0) * to(0));
  return clockwiseAsPositive ? angle : -angle;
}

template <class T, int N>
inline bool IsFuzzyParallel(const Vec<T, N> &v1, const Vec<T, N> &v2,
                            const T &epsilon = 0.1) {
  auto s = v1.dot(v2) / norm(v1) / norm(v2);
  return s >= 1.0 - epsilon || s <= -1.0 + epsilon;
}

template <class T, int N>
inline bool IsFuzzyPerpendicular(const Vec<T, N> &v1, const Vec<T, N> &v2,
                                 const T &epsilon = 0.1) {
  auto s = v1.dot(v2) / norm(v1) / norm(v2);
  return abs(s) <= epsilon;
}

template <class PointIterT, class T>
inline bool IsFuzzyColinear(PointIterT points_begin, PointIterT points_end,
                            const T &epsilon = 0.1);

// for lines and points
// returns projection position
template <class T, int N>
PositionOnLine<Point<T, N>>
ProjectionOfPointOnLine(const Point<T, N> &p, const Line<Point<T, N>> &line);

// returns (distance, nearest point)
template <class T, int N>
std::pair<T, Point<T, N>> DistanceFromPointToLine(const Point<T, N> &p,
                                                  const Ray<Point<T, N>> &line);

template <class T, int N>
inline T Distance(const Point<T, N> &p, const Ray<Point<T, N>> &line) {
  return DistanceFromPointToLine(p, line).first;
}
template <class T, int N>
inline T Distance(const Ray<Point<T, N>> &line, const Point<T, N> &p) {
  return DistanceFromPointToLine(p, line).first;
}

// returns signed distance
template <class T>
T SignedDistanceFromPointToLine(const Point<T, 2> &p,
                                const Ray<Point<T, 2>> &line);

// returns (distance, nearest position)
template <class T, int N>
std::pair<T, PositionOnLine<Point<T, N>>>
DistanceFromPointToLine(const Point<T, N> &p, const Line<Point<T, N>> &line);

template <class T, int N>
inline T Distance(const Point<T, N> &p, const Line<Point<T, N>> &line) {
  return DistanceFromPointToLine(p, line).first;
}
template <class T, int N>
inline T Distance(const Line<Point<T, N>> &line, const Point<T, N> &p) {
  return DistanceFromPointToLine(p, line).first;
}

// returns (distance, (nearest point on line1, nearest point on line2))
// see http://geomalgorithms.com/a07-_distance.html for explainations
template <class T, int N>
std::pair<T, std::pair<Point<T, N>, Point<T, N>>>
DistanceBetweenTwoLines(const Ray<Point<T, N>> &line1,
                        const Ray<Point<T, N>> &line2, T *lambda1 = nullptr,
                        T *lambda2 = nullptr);

template <class T, int N>
inline T Distance(const Ray<Point<T, N>> &line1,
                  const Ray<Point<T, N>> &line2) {
  return DistanceBetweenTwoLines(line1, line2).first;
}

template <class T>
inline Point<T, 2> Intersection(const Ray<Point<T, 2>> &line1,
                                const Ray<Point<T, 2>> &line2);

// returns (distance, (nearest position on line1, nearest position on line2))
// see http://geomalgorithms.com/a07-_distance.html for explainations
template <class T, int N>
std::pair<T,
          std::pair<PositionOnLine<Point<T, N>>, PositionOnLine<Point<T, N>>>>
DistanceBetweenTwoLines(const Line<Point<T, N>> &line1,
                        const Line<Point<T, N>> &line2);

template <class T, int N>
inline T Distance(const Line<Point<T, N>> &line1,
                  const Line<Point<T, N>> &line2) {
  return DistanceBetweenTwoLines(line1, line2).first;
}

// intersecton between line and plane
template <class T, int N>
PositionOnLine<Point<T, N>>
IntersectionOfLineAndPlane(const Ray<Point<T, N>> &line,
                           const Plane<Point<T, N>> &plane) {
  T lambda = (plane.anchor - line.anchor).dot(plane.normal) /
             line.direction.dot(plane.normal);
  return PositionOnLine<Point<T, N>>(line, lambda);
}

template <class T, int N>
inline Point<T, N> Intersection(const Ray<Point<T, N>> &line,
                                const Plane<Point<T, N>> &plane) {
  return IntersectionOfLineAndPlane(line, plane).position;
}

template <class T, int N>
inline Point<T, N> Intersection(const Plane<Point<T, N>> &plane,
                                const Ray<Point<T, N>> &line) {
  return IntersectionOfLineAndPlane(line, plane).position;
}

// intersection o
template <class T>
Failable<Ray<Point<T, 3>>>
IntersectionOfPlaneAndPlane(const Plane<Point<T, 3>> &p1,
                            const Plane<Point<T, 3>> &p2);

// triangulate polygon
// VertPoint2GetterT: (Vert) -> Point2
// AddTriFaceFunT: (Vert, Vert, Vert) -> void
template <class VertIterT, class VertPoint2GetterT, class AddTriFaceFunT>
int TriangulatePolygon(VertIterT vertsBegin, VertIterT vertsEnd,
                       VertPoint2GetterT &&getPoint2, AddTriFaceFunT &&addFace);

// area of polygon
template <class T> T Area(const Polygon<Point<T, 3>> &polygon);

// intersection of line and polygon
template <class T>
Failable<Point<T, 3>>
IntersectionOfLineAndPolygon(const Ray<Point<T, 3>> &ray,
                             const Polygon<Point<T, 3>> &polygon,
                             T epsilon = 0);

// distance from point to plane
template <class T, int N>
inline std::pair<T, Point<T, N>>
DistanceFromPointToPlane(const Point<T, N> &p, const Plane<Point<T, N>> &plane);

template <class T, int N>
Vec<T, N> BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
    const Ray<Point<T, N>> &line, const Point<T, N> *cornersData);

template <class T, int N>
inline Vec<T, N> BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
    const Ray<Point<T, N>> &line, const std::array<Point<T, N>, N> &corners);

// eigen vectors and eigen values from points
template <class T, int N>
std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const Point<T, N> *ptsData, size_t n);

template <class T, int N, int M>
inline std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const Point<T, N> (&ptsData)[M]);

template <class T, int N>
inline std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const std::vector<Point<T, N>> &pts);

// rotate a vec3 toward another with given rotation angle
// resulted vec is normalized
template <class T>
inline Vec<T, 3> RotateDirection(const Vec<T, 3> &originalDirection,
                                 const Vec<T, 3> &toDirection, double angle);

// on left
template <class T>
inline bool IsOnLeftSide(const Point<T, 2> &p, const Point<T, 2> &a,
                         const Point<T, 2> &b);

// area of triangle
template <class T> inline T AreaOfTriangle(const T &a, const T &b, const T &c);

template <class T, int N>
inline T AreaOfTriangle(const Point<T, N> &a, const Point<T, N> &b,
                        const Point<T, N> &c);

// in triangle
template <class T>
inline bool IsInTriangle(const Point<T, 2> &p, const Point<T, 2> &a,
                         const Point<T, 2> &b, const Point<T, 2> &c);

template <int N, class T, int M>
inline Vec<T, N> TransformCoordinate(const Point<T, M> &p,
                                     const std::vector<Vec<T, M>> &axis,
                                     const Point<T, M> &origin = Point<T, M>());

// generate Fibonacci Directions
template <class AddVec3FunT>
inline void GenerateFibonacciDirections(int n, AddVec3FunT &&addVec3);

// mesh makers
// make tetrahedron
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTetrahedron(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace);

// make quad faced cube
template <class AddVertex3FunT, class AddQuadFaceFunT>
void MakeQuadFacedCube(AddVertex3FunT &&addVertex, AddQuadFaceFunT &&addFace);

// make tri faced cube
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriFacedCube(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace);

// make quad faced sphere
template <class AddVertex3FunT, class AddQuadFaceFunT>
void MakeQuadFacedSphere(AddVertex3FunT &&addVertex, AddQuadFaceFunT &&addFace,
                         int m, int n);

// make tri faced sphere
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriFacedSphere(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace,
                        int m, int n);

// make an icosahedron
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeIcosahedron(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace);

// make mesh from simple mesh file
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriMeshFromSMFFile(AddVertex3FunT &&addVertex,
                            AddTriFaceFunT &&addFace,
                            const std::string &fileName);
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace core {

// bounding box of range
template <class IterT> auto BoundingBoxOfRange(IterT begin, IterT end) {
  using BoxType = decltype(BoundingBox(*begin));
  BoxType box; // a null box
  while (begin != end) {
    box |= BoundingBox(*begin);
    ++begin;
  }
  return box;
}

// bounding box of pair-range
template <class PairIterT>
auto BoundingBoxOfPairRange(PairIterT begin, PairIterT end) {
  using BoxType = decltype(BoundingBox((*begin).second));
  BoxType box;
  while (begin != end) {
    box |= BoundingBox((*begin).second);
    ++begin;
  }
  return box;
}

namespace {
// [low, high] for float v
template <class T, class K1, class K2>
inline bool IsBetweenPrivate(const T &v, const K1 &low, const K2 &high,
                             const std::true_type &floatBnd) {
  return !(v < low) && !(high < v);
}
// [low, high) for nonfloat v
template <class T, class K1, class K2>
inline bool IsBetweenPrivate(const T &v, const K1 &low, const K2 &high,
                             const std::false_type &floatBnd) {
  return !(v < low) && v < high;
}
}

// [low, high] for float v, [low, high) for nonfloat v
template <class T, class K1, class K2>
inline bool IsBetween(const T &v, const K1 &low, const K2 &high) {
  return IsBetweenPrivate(v, low, high, std::is_floating_point<T>());
}

namespace {
template <class T>
T WrapBetweenPrivate(const T &input, const T &low, const T &high,
                     const std::false_type &) {
  if (low >= high)
    return input;
  if (low <= input && input < high)
    return input;
  const auto sz = high - low;
  auto result = input - int((input - low) / sz) * sz + (input < low ? sz : 0);
  return result == high ? low : result;
}

template <class T>
T WrapBetweenPrivate(const T &input, const T &low, const T &high,
                     const std::true_type &) {
  if (low >= high)
    return input;
  if (low <= input && input < high)
    return input;
  const auto sz = high - low;
  auto result = (input - low) % sz + low + (input < low ? sz : 0);
  return result == high ? low : result;
}
}

// returns [low, high)
template <class T>
inline T WrapBetween(const T &input, const T &low, const T &high) {
  return WrapBetweenPrivate(input, low, high, std::is_integral<T>());
}

template <class T, int N>
T EncodeSubscriptToIndex(const Point<T, N> &subscript,
                         const Vec<T, N> &dimension) {
  T index = subscript[0];
  for (int i = 1; i < N; i++) {
    index = index * dimension[i] + subscript[i];
  }
  return index;
}

namespace {
template <class T, int N>
Point<T, N> DecodeIndexToSubscriptPrivate(T index, const Vec<T, N> &dimension,
                                          const std::false_type &) {
  Point<T, N> subscript;
  for (int i = N - 1; i >= 0; i--) {
    subscript[i] = WrapBetween(index, T(0.0), dimension[i]);
    index = (index - subscript[i]) / dimension[i];
  }
  return subscript;
}

template <class T, int N>
Point<T, N> DecodeIndexToSubscriptPrivate(T index, const Vec<T, N> &dimension,
                                          const std::true_type &) {
  Point<T, N> subscript;
  for (int i = N - 1; i >= 0; i--) {
    subscript[i] = index % dimension[i];
    index = index / dimension[i];
  }
  return subscript;
}
}

template <class T, int N>
inline Point<T, N> DecodeIndexToSubscript(T index, const Vec<T, N> &dimension) {
  return DecodeIndexToSubscriptPrivate(index, dimension, std::is_integral<T>());
}

template <class PointIterT, class T>
inline bool IsFuzzyColinear(PointIterT points_begin, PointIterT points_end,
                            const T &epsilon) {
  if (points_begin == points_end) {
    return true;
  }
  using PointT = typename std::iterator_traits<PointIterT>::value_type;
  PointT dir = PointT();
  PointT first_point = *points_begin;
  while (points_begin != points_end && dir == PointT()) {
    dir = *points_begin - first_point;
    ++points_begin;
  }
  while (points_begin != points_end) {
    if (!IsFuzzyParallel(dir, *points_begin - first_point, epsilon)) {
      return false;
    }
    ++points_begin;
  }
  return true;
}

template <class T>
inline std::pair<Vec<T, 3>, Vec<T, 3>>
ProposeXYDirectionsFromZDirection(const Vec<T, 3> &z) {
  auto y = z.cross(Vec<T, 3>(1, 0, 0));
  if (y(0) == 0 && y(1) == 0 && y(2) == 0) {
    y = z.cross(Vec<T, 3>(0, 1, 0));
  }
  auto x = y.cross(z);
  return std::make_pair(normalize(x), normalize(y));
}

// for lines and points
// returns projection position
template <class T, int N>
PositionOnLine<Point<T, N>>
ProjectionOfPointOnLine(const Point<T, N> &p, const Line<Point<T, N>> &line) {
  Vec<T, N> lineDir = line.direction();
  lineDir /= norm(lineDir);
  T projRatio = (p - line.first).dot(lineDir) / line.length();
  return PositionOnLine<Point<T, N>>(line, projRatio);
}

// returns (distance, nearest point)
template <class T, int N>
std::pair<T, Point<T, N>>
DistanceFromPointToLine(const Point<T, N> &p, const Ray<Point<T, N>> &line) {
  Vec<T, N> lineDir = line.direction;
  lineDir /= norm(lineDir);
  auto root = (p - line.anchor).dot(lineDir) * lineDir + line.anchor;
  return std::make_pair(norm(p - root), root);
}

// returns signed distance
template <class T>
T SignedDistanceFromPointToLine(const Point<T, 2> &p,
                                const Ray<Point<T, 2>> &line) {
  auto coeffs = GetCoeffs(line);
  return (coeffs[0] * p[0] + coeffs[1] * p[1] + coeffs[2]) /
         sqrt(Square(coeffs[0]) + Square(coeffs[1]));
}

// returns (distance, nearest position)
template <class T, int N>
std::pair<T, PositionOnLine<Point<T, N>>>
DistanceFromPointToLine(const Point<T, N> &p, const Line<Point<T, N>> &line) {
  Vec<T, N> lineDir = line.direction();
  lineDir /= norm(lineDir);
  T projRatio = (p - line.first).dot(lineDir) / line.length();
  projRatio = BoundBetween(projRatio, 0, 1);
  PositionOnLine<Point<T, N>> pos(line, projRatio);
  return std::make_pair(norm(p - pos.position), pos);
}

// returns (distance, (nearest point on line1, nearest point on line2))
// see http://geomalgorithms.com/a07-_distance.html for explainations
template <class T, int N>
std::pair<T, std::pair<Point<T, N>, Point<T, N>>>
DistanceBetweenTwoLines(const Ray<Point<T, N>> &line1,
                        const Ray<Point<T, N>> &line2, T *lambda1, T *lambda2) {

  auto u = normalize(line1.direction);
  auto v = normalize(line2.direction);
  auto w = line1.anchor - line2.anchor;
  auto a = u.dot(u); // always >= 0
  auto b = u.dot(v);
  auto c = v.dot(v); // always >= 0
  auto d = u.dot(w);
  auto e = v.dot(w);

  T P = a * c - b * b;
  if (P == 0) {
    auto ppair = std::make_pair(
        line1.anchor, DistanceFromPointToLine(line1.anchor, line2).second);
    return std::make_pair(Distance(ppair.first, ppair.second), ppair);
  }

  T sc = (b * e - c * d) / P;
  T tc = (a * e - b * d) / P;

  if (lambda1) {
    *lambda1 = sc / norm(line1.direction);
  }
  if (lambda2) {
    *lambda2 = tc / norm(line2.direction);
  }

  auto p1 = line1.anchor + sc * u;
  auto p2 = line2.anchor + tc * v;
  return std::make_pair(Distance(p1, p2), std::make_pair(p1, p2));
}

template <class T>
inline Point<T, 2> Intersection(const Ray<Point<T, 2>> &line1,
                                const Ray<Point<T, 2>> &line2) {
  auto eq1 = GetCoeffs(line1);
  auto eq2 = GetCoeffs(line2);
  auto hinterp = eq1.cross(eq2);
  if (hinterp == Origin<3, T>()) { // lines overlapped
    hinterp[0] = -eq1[1];
    hinterp[1] = eq1[0];
  }
  return Point<T, 2>(hinterp[0], hinterp[1]) / NonZeroize(hinterp[2], 1e-10);
}

// returns (distance, (nearest position on line1, nearest position on line2))
// see http://geomalgorithms.com/a07-_distance.html for explainations
template <class T, int N>
std::pair<T,
          std::pair<PositionOnLine<Point<T, N>>, PositionOnLine<Point<T, N>>>>
DistanceBetweenTwoLines(const Line<Point<T, N>> &line1,
                        const Line<Point<T, N>> &line2) {

  auto u = line1.direction();
  auto v = line2.direction();
  auto w = line1.first - line2.first;
  auto a = u.dot(u); // always >= 0
  auto b = u.dot(v);
  auto c = v.dot(v); // always >= 0
  auto d = u.dot(w);
  auto e = v.dot(w);
  auto D = a * c - b * b; // always >= 0
  T sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
  T tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

  static const double SMALL_NUM = 0;
  // compute the line parameters of the two closest points
  if (D <= SMALL_NUM) { // the lines are almost parallel
    sN = 0.0;           // force using point P0 on segment S1
    sD = 1.0;           // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  } else { // get the closest points on the infinite lines
    sN = (b * e - c * d);
    tN = (a * e - b * d);
    if (sN < 0.0) { // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) { // sc > 1  => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) { // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) { // tc > 1  => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  // finally do the division to get sc and tc
  sc = (IsBetween(sN, -SMALL_NUM, +SMALL_NUM) ? 0.0 : sN / sD);
  tc = (IsBetween(tN, -SMALL_NUM, +SMALL_NUM) ? 0.0 : tN / tD);

  PositionOnLine<Point<T, N>> pos1(line1, sc);
  PositionOnLine<Point<T, N>> pos2(line2, tc);
  auto dist = norm(pos1.position - pos2.position);
  return std::make_pair(dist, std::make_pair(pos1, pos2));
}

// intersection o
template <class T>
Failable<Ray<Point<T, 3>>>
IntersectionOfPlaneAndPlane(const Plane<Point<T, 3>> &p1,
                            const Plane<Point<T, 3>> &p2) {
  Vec<T, 3> dir = p1.normal.cross(p2.normal);
  Mat<T, 3, 3> mat(p1.normal[0], p1.normal[1], p1.normal[2], p2.normal[0],
                   p2.normal[1], p2.normal[2], dir[0], dir[1], dir[2]);
  Point3 anchor;
  bool success = cv::solve(
      mat, Vec3(p1.anchor.dot(p1.normal), p2.anchor.dot(p2.normal), 0), anchor);
  if (!success)
    return nullptr;
  return Ray<Point<T, 3>>(anchor, dir);
}

// triangulate polygon
// VertPoint2GetterT: (Vert) -> Point2
// AddTriFaceFunT: (Vert, Vert, Vert) -> void
template <class VertIterT, class VertPoint2GetterT, class AddTriFaceFunT>
int TriangulatePolygon(VertIterT vertsBegin, VertIterT vertsEnd,
                       VertPoint2GetterT &&getPoint2,
                       AddTriFaceFunT &&addFace) {

  using VHandleT = typename std::iterator_traits<VertIterT>::value_type;
  std::deque<std::vector<VHandleT>> vhGroupQ;
  vhGroupQ.emplace_back(vertsBegin, vertsEnd);

  int faceNum = 0;

  while (!vhGroupQ.empty()) {
    std::vector<VHandleT> is = vhGroupQ.front();
    vhGroupQ.pop_front();

    if (is.size() <= 2)
      continue;

    if (is.size() == 3) {
      addFace(is[0], is[1], is[2]);
      faceNum++;
    } else {
      // leftmost
      int leftmostII = 0;
      auto leftmostP = getPoint2(is[leftmostII]);
      for (int i = 0; i < is.size(); i++) {
        auto p = getPoint2(is[i]);
        if (p[0] < leftmostP[0]) {
          leftmostII = i;
          leftmostP = p;
        }
      }

      int leftmostPrevII = (leftmostII + is.size() - 1) % is.size();
      int leftmostNextII = (leftmostII + 1) % is.size();
      auto a = getPoint2(is[leftmostPrevII]);
      auto b = getPoint2(is[leftmostNextII]);

      int innerLeftmostII = -1;
      decltype(a) innerLeftmostP;
      for (int i = 0; i < is.size(); i++) {
        if (abs(i - leftmostII) <= 1 ||
            i == 0 && leftmostII == (is.size()) - 1 ||
            i == (is.size()) - 1 && leftmostII == 0)
          continue;
        auto p = getPoint2(is[i]);
        if (IsInTriangle(p, a, leftmostP, b)) {
          if (innerLeftmostII == -1) {
            innerLeftmostII = i;
            innerLeftmostP = p;
          } else if (p[0] < innerLeftmostP[0]) {
            innerLeftmostII = i;
            innerLeftmostP = p;
          }
        }
      }

      int split1 = leftmostII;
      int split2 = innerLeftmostII;
      if (innerLeftmostII < 0) {
        split1 = leftmostPrevII;
        split2 = leftmostNextII;
      }

      assert(split1 != split2);

      std::vector<VHandleT> part1, part2;

      for (int i = split1; i != split2; i = (i + 1) % is.size())
        part1.push_back(is[i]);
      part1.push_back(is[split2]);
      for (int i = split2; i != split1; i = (i + 1) % is.size())
        part2.push_back(is[i]);
      part2.push_back(is[split1]);

      assert(part1.size() >= 3);
      assert(part2.size() >= 3);

      is.clear();

      vhGroupQ.push_back(part1);
      vhGroupQ.push_back(part2);
    }
  }

  return faceNum;
}

// area of polygon
template <class T> T Area(const Polygon<Point<T, 3>> &polygon) {
  T area = 0.0;
  Vec<T, 3> x, y;
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(polygon.normal);
  TriangulatePolygon(polygon.corners.begin(), polygon.corners.end(),
                     [&](const Point<T, 3> &v) {
                       auto proj = v - v.dot(polygon.normal) * polygon.normal;
                       return Vec<T, 2>(proj.dot(x), proj.dot(y));
                     },
                     [&area](const Point<T, 3> &a, const Point<T, 3> &b,
                             const Point<T, 3> &c) {
                       double aa = AreaOfTriangle<T, 3>(a, b, c);
                       area += IsInfOrNaN(aa) ? 0.0 : aa;
                     });
  return area;
}

// intersection of line and polygon
template <class T>
Failable<Point<T, 3>>
IntersectionOfLineAndPolygon(const Ray<Point<T, 3>> &ray,
                             const Polygon<Point<T, 3>> &polygon, T epsilon) {
  Vec<T, 3> x, y;
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(polygon.normal);
  bool hasIntersection = false;
  Point<T, 3> intersection;
  TriangulatePolygon(polygon.corners.begin(), polygon.corners.end(),
                     [&](const Point<T, 3> &v) {
                       auto proj = v - v.dot(polygon.normal) * polygon.normal;
                       return Vec<T, 2>(proj.dot(x), proj.dot(y));
                     },
                     [&hasIntersection, &intersection, &ray,
                      epsilon](const Point<T, 3> &V1, const Point<T, 3> &V2,
                               const Point<T, 3> &V3) {
                       if (hasIntersection) {
                         return;
                       }
                       Vec<T, 3> e1 = V2 - V1, e2 = V3 - V1; // Edge1, Edge2
                       auto &O = ray.anchor;
                       auto &D = ray.direction;
                       auto P = D.cross(e2);
                       // Begin calculating determinant - also used to calculate
                       // u parameter
                       // if determinant is near zero, ray lies in plane of
                       // triangle
                       T det = e1.dot(P);
                       // NOT CULLING
                       if (det > -epsilon && det < epsilon)
                         return;
                       auto inv_det = 1.f / det;
                       // calculate distance from V1 to ray origin
                       auto t = O - V1;
                       // Calculate u parameter and test bound
                       auto u = t.dot(P) * inv_det;
                       // The intersection lies outside of the triangle
                       if (u < 0.f || u > 1.f)
                         return;
                       // Prepare to test v parameter
                       auto Q = t.cross(e1);
                       // Calculate V parameter and test bound
                       auto v = D.dot(Q) * inv_det;
                       // The intersection lies outside of the triangle
                       if (v < 0.f || u + v > 1.f)
                         return;
                       T lambda = e2.dot(Q) * inv_det;

                       if (lambda > epsilon) { // ray intersection
                         intersection = ray.anchor + ray.direction * lambda;
                         hasIntersection = true;
                       }
                     });

  if (hasIntersection) {
    return std::move(intersection);
  }
  return nullptr;
}

// distance from point to plane
template <class T, int N>
inline std::pair<T, Point<T, N>>
DistanceFromPointToPlane(const Point<T, N> &p,
                         const Plane<Point<T, N>> &plane) {
  auto signedDist = plane.signedDistanceTo(p);
  return std::make_pair(abs(signedDist),
                        p - signedDist * normalize(plane.normal));
}

template <class T, int N>
Vec<T, N> BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
    const Ray<Point<T, N>> &line, const Point<T, N> *cornersData) {
  using namespace Eigen;
  Map<const Matrix<T, N, N, Eigen::ColMajor>> corners((const T *)cornersData, N,
                                                      N);
  Map<const Matrix<T, N, 1>> D(line.direction.val, N, 1);
  Map<const Matrix<T, N, 1>> A(line.anchor.val, N, 1);

  Matrix<T, N, N> M =
      -(corners.colwise() - corners.col(0)); // 0 : p1-p2 : p1-p3
  M.col(0) = D;                              // d : p1-p2 : p1-p3

  Vec<T, N> coord;
  Map<Matrix<T, N, 1>> X(coord.val, N, 1);
  X = M.fullPivLu().solve(corners.col(0) - A);
  T depthOfLineIntersection = X(0);
  X(0) = 1.0f;
  for (int i = 1; i < N; i++) {
    X(0) -= X(i);
  }
  return coord;
}

template <class T, int N>
inline Vec<T, N> BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
    const Ray<Point<T, N>> &line, const std::array<Point<T, N>, N> &corners) {
  return BarycentricCoordinatesOfLineAndPlaneUnitIntersection(line,
                                                              corners.data());
}

// eigen vectors and eigen values from points
template <class T, int N>
std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const Point<T, N> *ptsData, size_t n) {
  using namespace Eigen;
  Map<const Matrix<T, Dynamic, N, RowMajor>> mat((const T *)ptsData, n, N);
  // std::cout << "points data mat: " << std::endl << mat << std::endl;
  auto centered = (mat.rowwise() - mat.colwise().mean()).eval();
  auto cov = ((centered.adjoint() * centered) / double(mat.rows() - 1)).eval();
  EigenSolver<decltype(cov)> es(cov);
  std::array<Scored<Vec<T, N>, T>, N> result;
  for (int i = 0; i < N; i++) {
    for (int k = 0; k < N; k++) {
      result[i].component(k) = es.eigenvectors()(i, k).real();
    }
    result[i].score = es.eigenvalues()(i).real();
  }
  return result;
}

template <class T, int N, int M>
inline std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const Point<T, N> (&ptsData)[M]) {
  return EigenVectorAndValuesFromPoints((const Point<T, N> *)ptsData, M);
}

template <class T, int N>
inline std::array<Scored<Vec<T, N>, T>, N>
EigenVectorAndValuesFromPoints(const std::vector<Point<T, N>> &pts) {
  return EigenVectorAndValuesFromPoints((const Point<T, N> *)pts.data(),
                                        pts.size());
}

// rotate a vec3 toward another with given rotation angle
// resulted vec is normalized
template <class T>
inline Vec<T, 3> RotateDirection(const Vec<T, 3> &originalDirection,
                                 const Vec<T, 3> &toDirection, double angle) {
  Vec<T, 3> tovec =
      normalize(originalDirection.cross(toDirection).cross(originalDirection));
  return normalize(normalize(originalDirection) * cos(angle) +
                   tovec * sin(angle));
}

// on left
template <class T>
inline bool IsOnLeftSide(const Point<T, 2> &p, const Point<T, 2> &a,
                         const Point<T, 2> &b) {
  T data[9] = {a[0], a[1], 1, b[0], b[1], 1, p[0], p[1], 1};
  T tmp1 = data[0 * 3 + 0] * (data[1 * 3 + 1] * data[2 * 3 + 2] -
                              data[1 * 3 + 2] * data[2 * 3 + 1]);
  T tmp2 = data[0 * 3 + 1] * (data[1 * 3 + 0] * data[2 * 3 + 2] -
                              data[1 * 3 + 2] * data[2 * 3 + 0]);
  T tmp3 = data[0 * 3 + 2] * (data[1 * 3 + 0] * data[2 * 3 + 1] -
                              data[1 * 3 + 1] * data[2 * 3 + 0]);
  return tmp1 - tmp2 + tmp3 > 0;
}

// area of triangle
template <class T> inline T AreaOfTriangle(const T &a, const T &b, const T &c) {
  T p = (a + b + c) / 2.0;
  return sqrt(p * (p - a) * (p - b) * (p - c));
}

template <class T, int N>
inline T AreaOfTriangle(const Point<T, N> &a, const Point<T, N> &b,
                        const Point<T, N> &c) {
  return AreaOfTriangle<T>(Distance(a, b), Distance(b, c), Distance(c, a));
}

// in triangle
template <class T>
inline bool IsInTriangle(const Point<T, 2> &p, const Point<T, 2> &a,
                         const Point<T, 2> &b, const Point<T, 2> &c) {
  bool lab = IsOnLeftSide(p, a, b);
  bool lbc = IsOnLeftSide(p, b, c);
  bool lca = IsOnLeftSide(p, c, a);
  return lab == lbc && lbc == lca;
}

template <int N, class T, int M>
inline Vec<T, N> TransformCoordinate(const Point<T, M> &p,
                                     const std::vector<Vec<T, M>> &axis,
                                     const Point<T, M> &origin) {
  assert(axis.size() >= N);
  Vec<T, N> c;
  for (int i = 0; i < N; i++)
    c[i] = (p - origin).dot(normalize(axis.at(i)));
  return c;
}

// generate Fibonacci Directions
template <class AddVec3FunT>
inline void GenerateFibonacciDirections(int n, AddVec3FunT &&addVec3) {
  static const float ga = M_PI * (-1.0f + std::sqrt(5.0f)) / 2.0f;
  for (int i = 0; i < n; i++) {
    auto d =
        GeoCoord(ga * (float)i, asin(-1.0f + 2.0f * float(i) / n)).toVector();
    addVec3(d[0], d[1], d[2]);
  }
}

// mesh makers
// make tetrahedron
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTetrahedron(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace) {
  auto v1 = addVertex(0.0f, 0.0f, 0.0f);
  auto v2 = addVertex(0.0f, 0.0f, 1.0f);
  auto v3 = addVertex(0.0f, 1.0f, 0.0f);
  auto v4 = addVertex(1.0f, 0.0f, 0.0f);

  addFace(v1, v2, v3);
  addFace(v1, v4, v2);
  addFace(v1, v3, v4);
  addFace(v2, v4, v3);
}

// make quad faced cube
template <class AddVertex3FunT, class AddQuadFaceFunT>
void MakeQuadFacedCube(AddVertex3FunT &&addVertex, AddQuadFaceFunT &&addFace) {
  /*
      4 ----- 5
     /       /|
    0 ----- 1 |
    |	      | |
    | 7	  | 6  -- x
    |	      |/
    3 ----- 2
   /
  y

  */
  auto v1 = addVertex(0.0f, 1.0f, 1.0f);
  auto v2 = addVertex(1.0f, 1.0f, 1.0f);
  auto v3 = addVertex(1.0f, 1.0f, 0.0f);
  auto v4 = addVertex(0.0f, 1.0f, 0.0f);

  auto v5 = addVertex(0.0f, 0.0f, 1.0f);
  auto v6 = addVertex(1.0f, 0.0f, 1.0f);
  auto v7 = addVertex(1.0f, 0.0f, 0.0f);
  auto v8 = addVertex(0.0f, 0.0f, 0.0f);

  addFace(v1, v2, v3, v4);
  addFace(v2, v6, v7, v3);
  addFace(v6, v5, v8, v7);
  addFace(v5, v1, v4, v8);
  addFace(v5, v6, v2, v1);
  addFace(v4, v3, v7, v8);
}

// make tri faced cube
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriFacedCube(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace) {
  auto v1 = addVertex(0.0f, 1.0f, 1.0f);
  auto v2 = addVertex(1.0f, 1.0f, 1.0f);
  auto v3 = addVertex(1.0f, 1.0f, 0.0f);
  auto v4 = addVertex(0.0f, 1.0f, 0.0f);

  auto v5 = addVertex(0.0f, 0.0f, 1.0f);
  auto v6 = addVertex(1.0f, 0.0f, 1.0f);
  auto v7 = addVertex(1.0f, 0.0f, 0.0f);
  auto v8 = addVertex(0.0f, 0.0f, 0.0f);

  addFace(v1, v2, v3);
  addFace(v1, v3, v4);

  addFace(v2, v6, v7);
  addFace(v2, v7, v3);

  addFace(v6, v5, v8);
  addFace(v6, v8, v7);

  addFace(v5, v1, v4);
  addFace(v5, v4, v8);

  addFace(v5, v6, v2);
  addFace(v5, v2, v1);

  addFace(v4, v3, v7);
  addFace(v4, v7, v8);
}

// make quad faced sphere
template <class AddVertex3FunT, class AddQuadFaceFunT>
void MakeQuadFacedSphere(AddVertex3FunT &&addVertex, AddQuadFaceFunT &&addFace,
                         int m, int n) {
  using ThisVertHandle = decltype(addVertex(0.0f, 0.0f, 0.0f));
  std::vector<std::vector<ThisVertHandle>> vhs(
      m, std::vector<ThisVertHandle>(n - 1));
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n - 1; j++) {
      double xratio = 1.0f - 1.0f / (n - 1) * j;
      double yratio = 1.0f / (m - 1) * i;
      double xangle = M_PI * 2 * xratio;
      double yangle = M_PI * yratio - M_PI_2;
      vhs[i][j] = addVertex(sin(xangle - M_PI_2) * cos(yangle),
                            cos(xangle - M_PI_2) * cos(yangle), sin(yangle));
    }
  }
  for (int i = 1; i < m; i++) {
    for (int j = 1; j < n - 1; j++) {
      addFace(vhs[i][j], vhs[i][j - 1], vhs[i - 1][j - 1], vhs[i - 1][j]);
    }
    addFace(vhs[i][0], vhs[i][n - 2], vhs[i - 1][n - 2], vhs[i - 1][0]);
  }
}

// make tri faced sphere
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriFacedSphere(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace,
                        int m, int n) {
  using VertHandle = decltype(addVertex(0.0f, 0.0f, 0.0f));
  std::vector<std::vector<VertHandle>> vhs(m, std::vector<VertHandle>(n - 1));
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n - 1; j++) {
      double xratio = 1.0f - 1.0f / (n - 1) * j;
      double yratio = 1.0f / (m - 1) * i;
      double xangle = M_PI * 2 * xratio;
      double yangle = M_PI * yratio - M_PI_2;
      vhs[i][j] = addVertex(sin(xangle - M_PI_2) * cos(yangle),
                            cos(xangle - M_PI_2) * cos(yangle), sin(yangle));
    }
  }
  for (int i = 1; i < m; i++) {
    for (int j = 1; j < n - 1; j++) {
      addFace(vhs[i][j], vhs[i][j - 1], vhs[i - 1][j - 1]);
      addFace(vhs[i][j], vhs[i - 1][j - 1], vhs[i - 1][j]);
    }
    addFace(vhs[i][0], vhs[i][n - 2], vhs[i - 1][n - 2]);
    addFace(vhs[i][j], vhs[i - 1][j - 1], vhs[i - 1][j]);
  }
}

// make an icosahedron
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeIcosahedron(AddVertex3FunT &&addVertex, AddTriFaceFunT &&addFace) {
  using VertHandle = decltype(addVertex(0.0f, 0.0f, 0.0f));
  // create a basic icosahedron
  static const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
  VertHandle vhs[] = {
      addVertex(-1, t, 0.0f),  addVertex(1, t, 0.0f),   addVertex(-1, -t, 0.0f),
      addVertex(1, -t, 0.0f),  addVertex(0.0f, -1, t),  addVertex(0.0f, 1, t),
      addVertex(0.0f, -1, -t), addVertex(0.0f, 1, -t),  addVertex(t, 0.0f, -1),
      addVertex(t, 0.0f, 1),   addVertex(-t, 0.0f, -1), addVertex(-t, 0.0f, 1)};
  addFace(vhs[0], vhs[11], vhs[5]);
  addFace(vhs[0], vhs[5], vhs[1]);
  addFace(vhs[0], vhs[1], vhs[7]);
  addFace(vhs[0], vhs[7], vhs[10]);
  addFace(vhs[0], vhs[10], vhs[11]);
  addFace(vhs[1], vhs[5], vhs[9]);
  addFace(vhs[5], vhs[11], vhs[4]);
  addFace(vhs[11], vhs[10], vhs[2]);
  addFace(vhs[10], vhs[7], vhs[6]);
  addFace(vhs[7], vhs[1], vhs[8]);
  addFace(vhs[3], vhs[9], vhs[4]);
  addFace(vhs[3], vhs[4], vhs[2]);
  addFace(vhs[3], vhs[2], vhs[6]);
  addFace(vhs[3], vhs[6], vhs[8]);
  addFace(vhs[3], vhs[8], vhs[9]);
  addFace(vhs[4], vhs[9], vhs[5]);
  addFace(vhs[2], vhs[4], vhs[11]);
  addFace(vhs[6], vhs[2], vhs[10]);
  addFace(vhs[8], vhs[6], vhs[7]);
  addFace(vhs[9], vhs[8], vhs[1]);
}

// make mesh from simple mesh file
template <class AddVertex3FunT, class AddTriFaceFunT>
void MakeTriMeshFromSMFFile(AddVertex3FunT &&addVertex,
                            AddTriFaceFunT &&addFace,
                            const std::string &fileName) {
  using VertHandle = decltype(addVertex(0.0f, 0.0f, 0.0f));
  std::vector<VertHandle> vhs;
  std::vector<std::array<int, 3>> fvids;
  int minFvid = std::numeric_limits<int>::max();
  std::ifstream ifs(fileName);
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    char tag;
    iss >> tag;
    if (tag == 'v') {
      float x, y, z;
      iss >> x >> y >> z;
      vhs.push_back(addVertex(x, y, z));
    } else if (tag == 'f') {
      int a, b, c;
      iss >> a >> b >> c;
      fvids.push_back({{a, b, c}});
      minFvid = std::min({minFvid, a, b, c});
    }
  }
  // install faces
  for (auto &fvs : fvids) {
    addFace(vhs[fvs[0] - minFvid], vhs[fvs[1] - minFvid],
            vhs[fvs[2] - minFvid]);
  }
}

// make mesh from simple obj file
template <class AddVertex3FunT, class AddFaceFunT>
void MakeMeshFromObjFile(AddVertex3FunT &&addVertex, AddFaceFunT &&addFace,
                         const std::string &fname) {
  std::ifstream ifs(fname);
  if (ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.empty()) {
        continue;
      }
      std::istringstream ss(line);
      std::string token;
      ss >> token;
      if (token == "v") {
        float x, y, z;
        ss >> x >> y >> z;
        addVertex(x, y, z);
      } else if (token == "f") {
        std::vector<int> corners;
        while (ss >> token) {
          if (token.empty()) {
            continue;
          }
          int vid = -1;
          size_t p = token.find_first_of('/');
          if (p == std::string::npos) {
            vid = std::stoi(token);
          } else {
            vid = std::stoi(token.substr(0, p));
          }
          assert(vid != -1);
          corners.push_back(vid - 1);
        }
        if (!corners.empty()) {
          addFace(std::move(corners));
        }
      }
    }
  }
}
}
}
