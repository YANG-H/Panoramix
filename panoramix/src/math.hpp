#pragma once

#include <cstdint>
#include <numeric>
#include <vector>

#include <opencv2/opencv.hpp>

#include "meta.hpp"
#include "ratio.hpp"

namespace cv {
template <class T, int N>
inline bool operator<(const Vec<T, N> &a, const Vec<T, N> &b) {
  for (int i = 0; i < N; i++) {
    if (a[i] > b[i]) {
      return false;
    } else if (a[i] < b[i]) {
      return true;
    }
  }
  return false;
}
}

namespace pano {
namespace core {
// vectors/points
template <class T, int N> using Vec = ::cv::Vec<T, N>;
using Vec2 = Vec<double, 2>;
using Vec3 = Vec<double, 3>;
using Vec4 = Vec<double, 4>;
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;
using Vec4i = Vec<int, 4>;
using Vec2ub = Vec<uint8_t, 2>;
using Vec3ub = Vec<uint8_t, 3>;
using Vec4ub = Vec<uint8_t, 4>;
template <class T, int N> using Point = ::cv::Vec<T, N>;
using Point2 = Point<double, 2>;
using Point3 = Point<double, 3>;
using Point4 = Point<double, 4>;
using Point2f = Point<float, 2>;
using Point3f = Point<float, 3>;
using Point4f = Point<float, 4>;
using Point2i = Point<int, 2>;
using Point3i = Point<int, 3>;
using Point4i = Point<int, 4>;

using Vec5 = Vec<double, 5>;
using Vec5f = Vec<float, 5>;
using Vec7 = Vec<double, 7>;
using Vec7f = Vec<float, 7>;

template <class T, int N> struct MarkedAsNonContainer<Point<T, N>> : yes {};

// matrix
template <class T, int M, int N> using Mat = ::cv::Matx<T, M, N>;
using Mat3 = Mat<double, 3, 3>;
using Mat4 = Mat<double, 4, 4>;
using Mat3f = Mat<float, 3, 3>;
using Mat4f = Mat<float, 4, 4>;

template <class T, int M, int N>
struct MarkedAsNonContainer<Mat<T, M, N>> : yes {};

using ::cv::norm;
template <class T> inline T normalize(const T &d) { return d / norm(d); }

template <int N = 3, class T = double> inline const Point<T, N> &Origin() {
  static const Point<T, N> _origin;
  return _origin;
}

template <int N = 3, class T = double> inline const Vec<T, N> &X() {
  static const Vec<T, N> _x(1);
  return _x;
}

template <int N = 3, class T = double> inline const Vec<T, N> &Y() {
  static const Vec<T, N> _y(0, 1);
  return _y;
}

template <int N = 3, class T = double> inline const Vec<T, N> &Z() {
  static const Vec<T, N> _z(0, 0, 1);
  return _z;
}

template <class To, class From, int N>
inline Vec<To, N> ecast(const Vec<From, N> &v) {
  return v;
}

template <class To, class From, int N>
inline std::vector<Vec<To, N>> ecast(const std::vector<Vec<From, N>> &v) {
  std::vector<Vec<To, N>> result(v.size());
  for (int i = 0; i < v.size(); i++) {
    result[i] = v[i];
  }
  return result;
}

template <class T, int M, int N>
inline Vec<T, M + N> cat(const Mat<T, M, 1> &a, const Mat<T, N, 1> &b) {
  Vec<T, M + N> ab;
  std::copy(a.val, a.val + M, ab.val);
  std::copy(b.val, b.val + N, ab.val + M);
  return ab;
}

template <class T, int M>
inline Vec<T, M + 1> cat(const Mat<T, M, 1> &a, const T &b) {
  Vec<T, M + 1> ab;
  std::copy(a.val, a.val + M, ab.val);
  ab[M] = b;
  return ab;
}

template <class T, int M>
inline Vec<T, M + 1> cat(const T &a, const Mat<T, M, 1> &b) {
  Vec<T, M + 1> ab;
  ab.val[0] = a;
  std::copy(b.val, b.val + M, ab.val + 1);
  return ab;
}

// dense mat
template <class T> using DenseMat = ::cv::Mat_<T>;
using DenseMati = DenseMat<int>;
using DenseMatd = DenseMat<double>;

// sparse mat
template <class T> using SparseMat = ::cv::SparseMat_<T>;
using SparseMatd = SparseMat<double>;
template <class T> struct SparseMatElement {
  using ValueType = T;
  int row, col;
  T value;
  inline SparseMatElement() : row(-1), col(-1) {}
  inline SparseMatElement(int r, int c, T v) : row(r), col(c), value(v) {}
};
using SparseMatElementd = SparseMatElement<double>;
template <class SparseMatElementIterT,
          class T = typename std::iterator_traits<
              SparseMatElementIterT>::value_type::ValueType>
inline SparseMat<T> MakeSparseMatFromElements(int row, int col,
                                              SparseMatElementIterT &&begin,
                                              SparseMatElementIterT &&end) {
  int dims[] = {row, col};
  SparseMat<T> mat(2, dims);
  while (begin != end) {
    mat.ref(begin->row, begin->col) = begin->value;
    ++begin;
  }
  return mat;
}

// homogeneous point
template <class T, int N> using HPoint = Ratio<Point<T, N>, T>;
template <class T, int N>
Vec<T, N + 1> VectorFromHPoint(const HPoint<T, N> &p, const T &scale = 1.0) {
  Vec<T, N + 1> v;
  std::copy(p.numerator.val, p.numerator.val + N, v.val);
  v[N] = p.denominator * scale;
  return v;
}
template <class T, int N>
HPoint<T, N - 1> HPointFromVector(const Vec<T, N> &v, const T &scale = 1.0) {
  HPoint<T, N - 1> hp;
  std::copy(v.val, v.val + N - 1, hp.numerator.val);
  hp.denominator = v[N - 1] / scale;
  return hp;
}
template <class T, int N> inline Ratio<T, T> norm(const HPoint<T, N> &p) {
  return Ratio<T, T>(norm(p.numerator), p.denominator);
}
template <class T, int N>
inline Ratio<T, T> dot(const HPoint<T, N> &a, const HPoint<T, N> &b) {
  return Ratio<T, T>(a.numerator.dot(b.numerator),
                     a.denominator * b.denominator);
}
using HPoint2 = HPoint<double, 2>;
using HPoint3 = HPoint<double, 3>;
using HPoint4 = HPoint<double, 4>;

// matrix transform
template <class T> Mat<T, 3, 3> MakeMat3Rotate(const Vec<T, 3> &axis, T angle) {
  auto a = normalize(axis);
  double l = a[0], m = a[1], n = a[2];
  double cosv = cos(angle), sinv = sin(angle);
  return Mat<T, 3, 3>(l * l * (1 - cosv) + cosv, m * l * (1 - cosv) - n * sinv,
                      n * l * (1 - cosv) + m * sinv,
                      l * m * (1 - cosv) + n * sinv, m * m * (1 - cosv) + cosv,
                      n * m * (1 - cosv) - l * sinv,
                      l * n * (1 - cosv) - m * sinv,
                      m * n * (1 - cosv) + l * sinv, n * n * (1 - cosv) + cosv);
}

template <class T> Mat<T, 4, 4> MakeMat4Rotate(const Vec<T, 3> &axis, T angle) {
  auto a = normalize(axis);
  double l = a[0], m = a[1], n = a[2];
  double cosv = cos(angle), sinv = sin(angle);
  return Mat<T, 4, 4>(
      l * l * (1 - cosv) + cosv, m * l * (1 - cosv) - n * sinv,
      n * l * (1 - cosv) + m * sinv, 0, l * m * (1 - cosv) + n * sinv,
      m * m * (1 - cosv) + cosv, n * m * (1 - cosv) - l * sinv, 0,
      l * n * (1 - cosv) - m * sinv, m * n * (1 - cosv) + l * sinv,
      n * n * (1 - cosv) + cosv, 0, 0, 0, 0, 1);
}

template <class T> Mat<T, 4, 4> MakeMat4Translate(const Vec<T, 3> &trans) {
  return Mat<T, 4, 4>(1, 0, 0, trans[0], 0, 1, 0, trans[1], 0, 0, 1, trans[2],
                      0, 0, 0, 1);
}

template <class T> Mat<T, 4, 4> MakeMat4Scale(const T &scale) {
  return Mat<T, 4, 4>(scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0,
                      1);
}

template <class T> Mat<T, 4, 4> MakeMat4Scale(T sx, T sy, T sz) {
  return Mat<T, 4, 4>(sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1);
}

template <class T>
Mat<T, 4, 4>
MakeMat4LocalToWorld(const Vec<T, 3> &localx, const Vec<T, 3> &localy,
                     const Vec<T, 3> &localz, const Point<T, 3> &localo) {
  return Mat<T, 4, 4>(localx[0], localy[0], localz[0], localo[0], localx[1],
                      localy[1], localz[1], localo[1], localx[2], localy[2],
                      localz[2], localo[2], 0, 0, 0, 1)
      .t();
}

// camera functions with matrix
// make a lookat view matrix
template <class T>
Mat<T, 4, 4> MakeMat4LookAt(const Vec<T, 3> &eye, const Vec<T, 3> &center,
                            const Vec<T, 3> &up) {
  Vec<T, 3> zaxis = (center - eye);
  zaxis /= core::norm(zaxis);
  Vec<T, 3> xaxis = up.cross(zaxis);
  xaxis /= core::norm(xaxis);
  Vec<T, 3> yaxis = zaxis.cross(xaxis);
  Mat<T, 4, 4> m(-xaxis(0), yaxis(0), -zaxis(0), 0, -xaxis(1), yaxis(1),
                 -zaxis(1), 0, -xaxis(2), yaxis(2), -zaxis(2), 0,
                 xaxis.dot(eye), -yaxis.dot(eye), zaxis.dot(eye), 1);
  return m.t();
}

// make a perspective projection matrix
template <class T>
Mat<T, 4, 4> MakeMat4Perspective(const T &fovyRadians, const T &aspect,
                                 const T &nearZ, const T &farZ) {
  T cotan = T(1.0) / std::tan(fovyRadians / 2.0);
  Mat<T, 4, 4> m(cotan / aspect, 0, 0, 0, 0, cotan, 0, 0, 0, 0,
                 (farZ + nearZ) / (nearZ - farZ), -1, 0, 0,
                 (2 * farZ * nearZ) / (nearZ - farZ), 0);
  return m.t();
}

template <class T>
Mat<T, 4, 4> MakeMat4Perspective(const T &fx, const T &fy, const T &cx,
                                 const T &cy, const T &nearZ, const T &farZ) {
  Mat<T, 4, 4> m(fx / cx, 0, 0, 0, 0, fy / cy, 0, 0, 0, 0,
                 (farZ + nearZ) / (nearZ - farZ), -1, 0, 0,
                 (2 * farZ * nearZ) / (nearZ - farZ), 0);
  return m.t();
}
}
}


namespace std {
template <class T, int N, int M>
const T *begin(const pano::core::Mat<T, N, M> &v) {
  return v.val;
}

template <class T, int N, int M>
const T *end(const pano::core::Mat<T, N, M> &v) {
  return v.val + N * M;
}

template <class T, int N, int M>
const T *cbegin(const pano::core::Mat<T, N, M> &v) {
  return v.val;
}

template <class T, int N, int M>
const T *cend(const pano::core::Mat<T, N, M> &v) {
  return v.val + N * M;
}

template <class T, int N, int M> T *begin(pano::core::Mat<T, N, M> &v) {
  return v.val;
}

template <class T, int N, int M> T *end(pano::core::Mat<T, N, M> &v) {
  return v.val + N * M;
}
}
