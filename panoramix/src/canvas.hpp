#pragma once

#include "basic_types.hpp"
#include "meta.hpp"
#include "utility.hpp"

#include "color.hpp"

namespace pano {
namespace gui {

using namespace core;

struct PaintingOptions {
  Color color;
  int thickness;
  int lineType;
  int shift;
  float alpha; // for new image
  ColorTable colorTable;
  float fontScale;
};

namespace {
inline void DrawRay2OnImage(Image &im, const Ray2 &ray,
                            const PaintingOptions &po) {
  Point2 imCenter(im.cols / 2.0, im.rows / 2.0);
  auto d = DistanceFromPointToLine(imCenter, ray);
  const Point2 &root = d.second;
  auto dir = normalize(ray.direction);
  double scale = norm(imCenter) * 2;
  Pixel p1 = ToPixel(root - dir * scale);
  Pixel p2 = ToPixel(root + dir * scale);
  cv::clipLine(cv::Rect(0, 0, im.cols, im.rows), p1, p2);
  cv::line(im, p1, p2, po.color, po.thickness, po.lineType, po.shift);
}
}

// 2d
template <class T> class Canvas {
public:
  Canvas(Image_<T> &im) : _image(im) { resetPaintingOptions(); }
  Canvas(Image_<T> &&im) : _image(im) { resetPaintingOptions(); }

  void resetPaintingOptions() {
    _paintingOptions.color = gui::White;
    _paintingOptions.thickness = 1;
    _paintingOptions.lineType = 8;
    _paintingOptions.shift = 0;
    _paintingOptions.alpha = 0.5f;
    _paintingOptions.colorTable = ColorTableDescriptor::AllColors;
    _paintingOptions.fontScale = 0.5;
  }

  Image_<T> &image() { return _image; }

  PaintingOptions &paintingOptions() { return _paintingOptions; }
  const PaintingOptions &paintingOptions() const { return _paintingOptions; }

  Canvas &color(const Color &c) {
    _paintingOptions.color = c;
    return *this;
  }
  Canvas &thickness(int t) {
    _paintingOptions.thickness = t;
    return *this;
  }
  Canvas &alpha(float a) {
    _paintingOptions.alpha = a;
    return *this;
  }
  Canvas &colorTable(const ColorTable &ct) {
    _paintingOptions.colorTable = ct;
    return *this;
  }

  template <class TT> inline Canvas &add(const Point<TT, 2> &p) {
    int x = static_cast<int>(std::round(p[0]));
    int y = static_cast<int>(std::round(p[1]));
    auto pp = Pixel(x, y);
    if (Contains(image().size(), pp)) {
      image()(pp) = paintingOptions().color;
    }
    return *this;
  }

  inline Canvas &add(const Pixel &pp) {
    if (Contains(image().size(), pp)) {
      image()(pp) = paintingOptions().color;
    }
    return *this;
  }

  template <class TT> inline Canvas &add(const HPoint<TT, 2> &p) {
    return add(p.value());
  }

  // lines
  template <class TT> inline Canvas &add(const Line<Point<TT, 2>> &line) {
    cv::line(image(), cv::Point(static_cast<int>(line.first(0)),
                                static_cast<int>(line.first(1))),
             cv::Point(static_cast<int>(line.second(0)),
                       static_cast<int>(line.second(1))),
             paintingOptions().color, paintingOptions().thickness,
             paintingOptions().lineType, paintingOptions().shift);
    return *this;
  }

  inline Canvas &add(const Ray2 &ray) {
    DrawRay2OnImage(image(), ray, paintingOptions());
    return *this;
  }

  template <class TT> inline Canvas &add(const Ray<Point<TT, 2>> &line) {
    return add(Ray2(core::ecast<double>(line.anchor),
                    core::ecast<double>(line.direction)));
  }

  Canvas &add(const Circle &c) {
    cv::circle(image(), (cv::Point)c.center, c.radius, paintingOptions().color,
               paintingOptions().thickness, paintingOptions().lineType,
               paintingOptions().shift);
    return *this;
  }

  template <class TT> inline Canvas &add(const Chain<Point<TT, 2>> &c) {
    for (int i = 0; i < (c.closed ? c.size() : (c.size() - 1)); i++) {
      add(c.edge(i));
    }
    return *this;
  }

  // keypoints
  inline Canvas &add(const KeyPoint &p) {
    cv::drawKeypoints(image(), std::vector<KeyPoint>(1, p), image(),
                      paintingOptions().color);
    return *this;
  }

  inline Canvas &add(const std::vector<KeyPoint> &ps) {
    cv::drawKeypoints(image(), ps, image(), paintingOptions().color);
    return *this;
  }

  // classified thing
  template <class T> inline Canvas &add(const Classified<T> &thing) {
    paintingOptions().color = paintingOptions().colorTable[thing.claz];
    return add(thing.component);
  }

  // noted thing
  template <class T> inline Canvas &add(const Noted<T> &thing) {
    auto center = core::BoundingBox(thing.component).center();
    cv::putText(image(), thing.note, cv::Point(static_cast<int>(center[0]),
                                               static_cast<int>(center[1])),
                1, paintingOptions().fontScale, paintingOptions().color);
    return add(thing.component);
  }

  // enabled thing
  template <class T> inline Canvas &add(const Enabled<T> &thing) {
    if (thing.enabled) {
      return add(thing.component);
    }
    return *this;
  }

  // colored
  template <class T> inline Canvas &add(const Colored<T> & t) {
    paintingOptions().color = t.color;
    return add(t.component);
  }

  // enabled thing
  template <class T, class S> inline Canvas &add(const Scored<T, S> &thing) {
    if (thing.score > 0) {
      return add(thing.component);
    }
    return *this;
  }

  // image
  inline Canvas &add(const Image &im) {
    cv::addWeighted(image(), (1.0f - paintingOptions().alpha), im,
                    paintingOptions().alpha, 0.0, image());
    return *this;
  }

  template <class TT> inline Canvas &add(const Image_<TT> &im) {
    cv::addWeighted(image(), (1.0f - paintingOptions().alpha), im,
                    paintingOptions().alpha, 0.0, image());
    return *this;
  }

  inline Canvas &add(const Imagei &im) {
    return add(paintingOptions().colorTable(im));
  }

  // containers
  template <class T, class = std::enable_if_t<IsContainer<T>::value>>
  inline Canvas &add(const T &a) {
    for (auto &e : a)
      add(e);
    return *this;
  }

  template <class T> inline Canvas &add(std::initializer_list<T> a) {
    for (auto &e : a)
      add(e);
    return *this;
  }

  inline const Canvas &maxHeight(int mh) {
    core::ResizeToMakeHeightUnder(_image, mh);
    return *this;
  }
  inline const Canvas &maxWidth(int mw) {
    core::ResizeToMakeWidthUnder(_image, mw);
    return *this;
  }
  inline const Canvas &maxWidthAndHeight(int mw, int mh) {
    core::ResizeToMakeHeightUnder(_image, mh);
    core::ResizeToMakeWidthUnder(_image, mw);
    return *this;
  }

  const Canvas &show(int delay = 0,
                     const std::string &winName = "Canvas") const {
    static int id = 0;
    core::Image im = _image.clone();
    if (im.channels() > 3) {
      std::vector<cv::Mat> cs(3);
      for (int i = 0; i < 3; i++)
        cv::extractChannel(im, cs[i], i);
      cv::merge(cs, im);
    }
    cv::imshow(winName, im);
		if(delay != -1){
			cv::waitKey(delay);
		}
    return *this;
  }

  void saveAs(const std::string &filename) const {
    cv::imwrite(filename, _image);
  }

private:
  PaintingOptions _paintingOptions;
  Image_<T> _image;
};

template <class T> inline Canvas<T> AsCanvas(Image_<T> &im) {
  return Canvas<T>(im);
}
template <class T> inline Canvas<T> AsCanvas(Image_<T> &&im) {
  return Canvas<T>(std::move(im));
}

template <class T> inline Canvas<T> MakeCanvas(const Image_<T> &im) {
  return Canvas<T>(im.clone());
}

using Canvas3ub = Canvas<Vec3ub>;




}
}
