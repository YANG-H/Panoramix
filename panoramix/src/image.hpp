#pragma once

#include "geometry.hpp"

namespace cv {
template <class T>
inline bool operator<(const Point_<T> &a, const Point_<T> &b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}
}

namespace pano {
namespace core {

using Image = cv::Mat;

template <class T> using Image_ = cv::Mat_<T>;
using Imageb = Image_<bool>;
using Image3b = Image_<Vec<bool, 3>>;
using Imageub = Image_<uint8_t>;
using Image3ub = Image_<Vec<uint8_t, 3>>;
using Imagei = Image_<int>;
using Image3i = Image_<Vec<int, 3>>;
using Imagef = Image_<float>;
using Image3f = Image_<Vec<float, 3>>;
using Imaged = Image_<double>;
using Image3d = Image_<Vec<double, 3>>;
using Image5d = Image_<Vec<double, 5>>;
using Image6d = Image_<Vec<double, 6>>;
using Image7d = Image_<Vec<double, 7>>;

template <> struct MarkedAsNonContainer<Image> : yes {};
template <class T> struct MarkedAsNonContainer<Image_<T>> : yes {};

namespace {
template <class To, class From>
inline Image_<To> VecCastPrivate(const Image_<From> &im, std::false_type) {
  Image_<To> cim(im.size());
  for (auto it = im.begin(); it != im.end(); ++it) {
    cim(it.pos()) = static_cast<To>(*it);
  }
  return cim;
}
template <class To>
inline Image_<To> VecCastPrivate(const Image_<To> &v, std::true_type) {
  return v.clone();
}

template <class To, class From, int N>
inline Image_<Vec<To, N>> VecCastPrivate(const Image_<Vec<From, N>> &im,
                                         std::false_type) {
  Image_<Vec<To, N>> cim(im.size());
  for (auto it = im.begin(); it != im.end(); ++it) {
    cim(it.pos()) = ecast<To>(*it);
  }
  return cim;
}
template <class To, int N>
inline Image_<Vec<To, N>> VecCastPrivate(const Image_<Vec<To, N>> &v,
                                         std::true_type) {
  return v.clone();
}
}

template <class To, class From> inline Image_<To> ecast(const Image_<From> &v) {
  return VecCastPrivate<To>(
      v, std::integral_constant<bool, std::is_same<To, From>::value>());
}

template <class To, class From, int N>
inline Image_<Vec<To, N>> ecast(const Image_<Vec<From, N>> &v) {
  return VecCastPrivate<To>(
      v, std::integral_constant<bool, std::is_same<To, From>::value>());
}

using Pixel = cv::Point;
template <class T> inline Vec<T, 2> ecast(const Pixel &p) {
  return Vec<T, 2>(static_cast<T>(p.x), static_cast<T>(p.y));
}

template <class T = Vec<uint8_t, 3>>
inline Image_<T> ImageRead(const std::string &filename) {
  return cv::imread(filename);
}
template <class T>
inline bool ImageWrite(const std::string &filename, const Image_<T> &im) {
  return cv::imwrite(filename, im);
}

template <class T = int> inline T Area(const Image &im) {
  return im.cols * im.rows;
}
template <class T = double> inline Point<T, 2> Center(const Image &im) {
  return Point<T, 2>(im.cols / 2.0, im.rows / 2.0);
}

void ClipToSquare(Image &im);
Imageb ClipToDisk(Image &im);
Imageb Rotate(Image &im, double angle);

template <class T> inline void ReverseCols(Image_<T> &im) {
  for (int i = 0; i < im.cols / 2; i++) {
    for (int k = 0; k < im.rows; k++) {
      std::swap(im(k, i), im(k, im.cols - 1 - i));
    }
  }
}

template <class T> inline void ReverseRows(Image_<T> &im) {
  for (int i = 0; i < im.rows / 2; i++) {
    for (int k = 0; k < im.cols; k++) {
      std::swap(im(i, k), im(im.rows - 1 - i, k));
    }
  }
}

void ResizeToWidth(Image &im, int width);
void ResizeToHeight(Image &im, int height);
void ResizeToArea(Image &im, int area);
void ResizeToMakeWidthUnder(Image &im, int widthUpperBound);
void ResizeToMakeHeightUnder(Image &im, int heightUpperBound);
bool MayBeAPanorama(const Image &im);
bool MakePanorama(Image &im, int horiCenter = -1, bool *extendedOnTop = nullptr,
                  bool *extendedOnBottom = nullptr);

std::pair<Pixel, Pixel> MinMaxLocOfImage(const Image &im);
std::pair<double, double> MinMaxValOfImage(const Image &im);
template <class T> inline T Mean(const Image_<T> &im, const Imageub &mask) {
  T sum = T();
  int count = 0;
  for (auto it = im.begin(); it != im.end(); ++it) {
    if (!mask.empty() && !mask(it.pos())) {
      continue;
    }
    sum += (*it);
    count++;
  }
  return sum / count;
}

template <class T> inline Pixel ToPixel(const Point<T, 2> &p) {
  return Pixel(static_cast<int>(p[0]), static_cast<int>(p[1]));
}
template <class T> inline Pixel RoundToPixel(const Point<T, 2> &p) {
  return Pixel(static_cast<int>(std::round(p[0])),
               static_cast<int>(std::round(p[1])));
}

inline int Sub2Ind(const Pixel &p, int w, int h) { return p.x * h + p.y; }
inline int Sub2Ind(const Pixel &p, const Sizei &sz) {
  return Sub2Ind(p, sz.width, sz.height);
}
inline Pixel Ind2Sub(int ind, int w, int h) { return Pixel(ind / h, ind % h); }
inline Pixel Ind2Sub(int ind, const Sizei &sz) {
  return Ind2Sub(ind, sz.width, sz.height);
}

Pixel PixelFromGeoCoord(const GeoCoord &p, int longidiv, int latidiv);
GeoCoord GeoCoordFromPixel(const Pixel &pixel, int longidiv, int latidiv);

// non maxima suppression
void NonMaximaSuppression(const Image &src, Image &dst, int sz = 50,
                          std::vector<Pixel> *pixels = nullptr,
                          const Imageb &mask = Imageb());
}
}
