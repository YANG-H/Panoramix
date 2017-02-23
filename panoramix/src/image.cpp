#include "pch.hpp"

#include "image.hpp"
#include "macros.hpp"

namespace pano {
namespace core {

void ClipToSquare(Image &image) {
  int mindim = std::min(image.cols, image.rows);
  image = image(
      cv::Range(image.rows / 2 - mindim / 2, image.rows / 2 + mindim / 2),
      cv::Range(image.cols / 2 - mindim / 2, image.cols / 2 + mindim / 2));
}

Imageb ClipToDisk(Image &im) { NOT_IMPLEMENTED_YET(); }

Imageb Rotate(Image &im, double angle) {
  int len = std::max(im.cols, im.rows);
  cv::Mat r =
      cv::getRotationMatrix2D(Center<float>(im), angle / M_PI * 180, 1.0);
  Imageb mask(im.size(), true);
  cv::warpAffine(im, im, r, cv::Size(len, len));
  cv::warpAffine(mask, mask, r, cv::Size(len, len));
  return mask;
}

void ResizeToWidth(Image &im, int width) {
  if (im.cols == width)
    return;
  cv::resize(im, im,
             cv::Size(width, static_cast<int>(im.rows * width / im.cols)));
}

void ResizeToHeight(Image &im, int height) {
  if (im.rows == height)
    return;
  cv::resize(im, im,
             cv::Size(static_cast<int>(im.cols * height / im.rows), height));
}

void ResizeToArea(Image &im, int area) {
  if (im.rows * im.cols == area)
    return;
  double ratio = sqrt(area / double(im.rows * im.cols));
  cv::resize(im, im, cv::Size(static_cast<int>(im.cols * ratio),
                              static_cast<int>(im.rows * ratio)));
}

void ResizeToMakeWidthUnder(Image &im, int widthUpperBound) {
  if (im.cols <= widthUpperBound)
    return;
  cv::resize(im, im,
             cv::Size(widthUpperBound,
                      static_cast<int>(im.rows * widthUpperBound / im.cols)));
}

void ResizeToMakeHeightUnder(Image &im, int heightUpperBound) {
  if (im.rows <= heightUpperBound)
    return;
  cv::resize(im, im,
             cv::Size(static_cast<int>(im.cols * heightUpperBound / im.rows),
                      heightUpperBound));
}

bool MayBeAPanorama(const Image &im) {
  if (abs(im.cols - im.rows * 2) > im.rows / 10.0f)
    return false;
  // check boundary pixels
  /* for (int x = 0; x < im.cols; x++){
  const uchar* p1 = im.ptr(0, x);
  const uchar* p2 = im.ptr(im.rows - 1, x);
  for (int k = 0; k < im.elemSize(); k++){
  NOT_IMPLEMENTED_YET();
  }
  }*/
  // NOT_IMPLEMENTED_YET();
  THERE_ARE_BUGS_HERE("check continuity on borders");
  return true;
}

bool MakePanorama(Image &im, int horiCenter, bool *extendedOnTop,
                  bool *extendedOnBottom) {
  if (im.cols < im.rows * 2)
    return false;
  if (im.cols == im.rows * 2) {
    if (extendedOnTop)
      *extendedOnTop = false;
    if (extendedOnBottom)
      *extendedOnBottom = false;
    return true;
  }
  if (horiCenter == -1) {
    horiCenter = im.rows / 2;
  }
  Image pim = Image::zeros(im.cols / 2, im.cols, im.type());
  if (pim.rows / 2.0 < horiCenter) {
    return false;
  }

  if (extendedOnTop)
    *extendedOnTop = horiCenter < pim.rows / 2.0 - 1.0;
  if (extendedOnBottom)
    *extendedOnBottom = (im.rows - horiCenter) < pim.rows / 2.0 - 1.0;
  im.copyTo(pim(
      cv::Rect(0, std::round(pim.rows / 2.0 - horiCenter), pim.cols, im.rows)));
  im = pim;
  return true;
}

std::pair<Pixel, Pixel> MinMaxLocOfImage(const Image &im) {
  Pixel minLoc, maxLoc;
  double minVal, maxVal;
  cv::minMaxLoc(im, &minVal, &maxVal, &minLoc, &maxLoc);
  return std::make_pair(minLoc, maxLoc);
}

std::pair<double, double> MinMaxValOfImage(const Image &im) {
  Pixel minLoc, maxLoc;
  double minVal, maxVal;
  cv::minMaxLoc(im, &minVal, &maxVal, &minLoc, &maxLoc);
  return std::make_pair(minVal, maxVal);
}

Pixel PixelFromGeoCoord(const GeoCoord &p, int longidiv, int latidiv) {
  int longtid = static_cast<int>((p.longitude + M_PI) * longidiv / M_PI / 2);
  int latid = static_cast<int>((p.latitude + M_PI_2) * latidiv / M_PI);
  longtid = (longtid % longidiv + longidiv) % longidiv;
  latid = (latid % latidiv + latidiv) % latidiv;
  return Pixel(longtid, latid);
}

GeoCoord GeoCoordFromPixel(const Pixel &pixel, int longidiv, int latidiv) {
  return GeoCoord{pixel.x * M_PI * 2 / longidiv - M_PI,
                  pixel.y * M_PI / latidiv - M_PI_2};
}

void NonMaximaSuppression(const Image &src, Image &dst, int sz,
                          std::vector<Pixel> *pixels, const Imageb &mask) {

  const int M = src.rows;
  const int N = src.cols;
  const bool masked = !mask.empty();

  cv::Mat block = 255 * cv::Mat_<uint8_t>::ones(Size(2 * sz + 1, 2 * sz + 1));
  dst = cv::Mat::zeros(src.size(), src.type());

  // iterate over image blocks
  for (int m = 0; m < M; m += sz + 1) {
    for (int n = 0; n < N; n += sz + 1) {
      cv::Point ijmax;
      double vcmax, vnmax;

      // get the maximal candidate within the block
      cv::Range ic(m, std::min(m + sz + 1, M));
      cv::Range jc(n, std::min(n + sz + 1, N));
      cv::minMaxLoc(src(ic, jc), NULL, &vcmax, NULL, &ijmax,
                    masked ? mask(ic, jc) : cv::noArray());
      cv::Point cc = ijmax + cv::Point(jc.start, ic.start);

      // search the neighbours centered around the candidate for the true maxima
      cv::Range in(std::max(cc.y - sz, 0), std::min(cc.y + sz + 1, M));
      cv::Range jn(std::max(cc.x - sz, 0), std::min(cc.x + sz + 1, N));

      // mask out the block whose maxima we already know
      cv::Mat_<uint8_t> blockmask;
      block(cv::Range(0, in.size()), cv::Range(0, jn.size())).copyTo(blockmask);
      cv::Range iis(ic.start - in.start,
                    std::min(ic.start - in.start + sz + 1, in.size()));
      cv::Range jis(jc.start - jn.start,
                    std::min(jc.start - jn.start + sz + 1, jn.size()));
      blockmask(iis, jis) =
          cv::Mat_<uint8_t>::zeros(Size(jis.size(), iis.size()));
      minMaxLoc(src(in, jn), NULL, &vnmax, NULL, &ijmax,
                masked ? mask(in, jn).mul(blockmask) : blockmask);
      cv::Point cn = ijmax + cv::Point(jn.start, in.start);

      // if the block centre is also the neighbour centre, then it's a local
      // maxima
      if (vcmax > vnmax) {
        std::memcpy(dst.ptr(cc.y, cc.x), src.ptr(cc.y, cc.x), src.elemSize());
        if (pixels) {
          pixels->push_back(cc);
        }
      }
    }
  }
}
}
}