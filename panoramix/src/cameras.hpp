#pragma once

#include "basic_types.hpp"
#include "line_detection.hpp"
#include "manhattan.hpp"

namespace pano {
namespace core {

// perspective camera
class PerspectiveCamera {
public:
  PerspectiveCamera();
  PerspectiveCamera(int w, int h);
  PerspectiveCamera(int w, int h, const Point2 &pp, const Vec2 &focalxy,
                    const Point3 &eye = Point3(0, 0, 0),
                    const Point3 &center = Point3(1, 0, 0),
                    const Vec3 &up = Vec3(0, 0, -1), double nearPlane = 0.01,
                    double farPlane = 1e4);
  PerspectiveCamera(int w, int h, const Point2 &pp, double focal,
                    const Point3 &eye = Point3(0, 0, 0),
                    const Point3 &center = Point3(1, 0, 0),
                    const Vec3 &up = Vec3(0, 0, -1), double nearPlane = 0.01,
                    double farPlane = 1e4);

  Sizei screenSize() const {
    return Sizei(static_cast<int>(_screenW), static_cast<int>(_screenH));
  }
  double screenWidth() const { return _screenW; }
  double screenHeight() const { return _screenH; }
  const Point2 &principlePoint() const { return _principlePoint; }
  double aspect() const { return double(_screenW) / double(_screenH); }
  double focalX() const { return _focalxy[0]; }
  double focalY() const { return _focalxy[1]; }
  double focal() const {
    assert(_focalxy[0] == _focalxy[1]);
    return _focalxy[0];
  }
  const Point3 &eye() const { return _eye; }
  const Point3 &center() const { return _center; }
  const Vec3 &up() const { return _up; }
  double nearPlane() const { return _near; }
  double farPlane() const { return _far; }
  Point2 toScreen(const Point3 &p3d) const;
  bool isVisibleOnScreen(const Point3 &p3d) const;
  HPoint2 toScreenInHPoint(const Point3 &p3d) const;
  Point3 toSpace(const Point2 &p2d) const;
  Point3 toSpace(const Pixel &p) const { return toSpace(Point2(p.x, p.y)); }
  Vec3 direction(const Point2 &p2d) const { return toSpace(p2d) - _eye; }
  Vec3 direction(const Pixel &p) const { return direction(Point2(p.x, p.y)); }

  const Mat4 &viewMatrix() const { return _viewMatrix; }
  const Mat4 &projectionMatrix() const { return _projectionMatrix; }
  const Mat4 &viewProjectionMatrix() const { return _viewProjectionMatrix; }

  // operations
  void resizeScreen(const Size &sz, bool updateMat = true);
  void setPrinciplePoint(const Point2 &pp, bool updateMat = true);
  void setFocalX(double fx, bool updateMat = true);
  void setFocalY(double fy, bool updateMat = true);
  void setFocal(double f, bool updateMat = true);
  void setEye(const Point3 &e, bool updateMat = true);
  void setCenter(const Point3 &c, bool updateMat = true);
  void setUp(const Vec3 &up, bool updateMat = true);
  void setNearAndFarPlanes(double nearPlane, double farPlane,
                           bool updateMat = true);

  // advanced functions
  Vec3 forward() const { return normalize(_center - _eye); }
  Vec3 backward() const { return normalize(_eye - _center); }
  Vec3 upward() const { return normalize(_up); }
  Vec3 downward() const { return -upward(); }
  Vec3 leftward() const { return normalize(_up.cross(forward())); }
  Vec3 rightward() const { return -leftward(); }

  void focusOn(const Sphere3 &target, bool updateMat = true);
  void translate(const Vec3 &t, const Sphere3 &target, bool updateMat = true);
  void moveEyeWithCenterFixed(const Vec3 &t, const Sphere3 &target,
                              bool distanceFixed = false, bool updateMat = true,
                              bool upFixed = true);
  void moveCenterWithEyeFixed(const Vec3 &t, bool updateMat = true);

  // compare
  bool operator==(const PerspectiveCamera &cam) const;

protected:
  void updateMatrices();

protected:
  double _screenW, _screenH;
  Point2 _principlePoint;
  Vec2 _focalxy;
  double _near, _far;
  Vec3 _eye, _center, _up;
  Mat4 _viewMatrix, _projectionMatrix, _viewProjectionMatrix;

  template <class Archive> void serialize(Archive &ar) {
    ar(_screenW, _screenH);
    ar(_principlePoint);
    ar(_focalxy, _near, _far);
    ar(_eye, _center, _up);
    ar(_viewMatrix, _projectionMatrix, _viewProjectionMatrix);
  }
  friend class cereal::access;
};

// panoramic camera
class PanoramicCamera {
public:
  explicit PanoramicCamera(double focal = 250,
                           const Point3 &eye = Vec3(0, 0, 0),
                           const Point3 &center = Vec3(1, 0, 0),
                           const Vec3 &up = Vec3(0, 0, 1));

  Sizei screenSize() const {
    return Sizei(static_cast<int>(_focal * 2 * M_PI),
                 static_cast<int>(_focal * M_PI));
  }
  double focal() const { return _focal; }
  const Point3 &eye() const { return _eye; }
  const Point3 &center() const { return _center; }
  const Vec3 &up() const { return _up; }
  Point2 toScreen(const Point3 &p3d) const;
  bool isVisibleOnScreen(const Point3 &p3d) const { return true; }
  HPoint2 toScreenInHPoint(const Point3 &p3d) const {
    return HPoint2(toScreen(p3d), 1.0);
  }
  Point3 toSpace(const Point2 &p2d) const;
  Point3 toSpace(const Pixel &p) const { return toSpace(Vec2(p.x, p.y)); }
  Point3 toSpace(const HPoint2 &p) const { return toSpace(p.value()); }
  Vec3 direction(const Point2 &p2d) const;
  Vec3 direction(const Pixel &p) const { return direction(Point2(p.x, p.y)); }

private:
  double _focal;
  Vec3 _eye, _center, _up;
  Vec3 _xaxis, _yaxis, _zaxis;

  template <class Archive> void serialize(Archive &ar) {
    ar(_focal);
    ar(_eye, _center, _up);
    ar(_xaxis, _yaxis, _zaxis);
  }
  friend class cereal::access;
};

// partial panoramic camera
class PartialPanoramicCamera {
public:
  explicit PartialPanoramicCamera(int w = 500, int h = 500, double focal = 250,
                                  const Point3 &eye = Vec3(0, 0, 0),
                                  const Point3 &center = Vec3(1, 0, 0),
                                  const Vec3 &up = Vec3(0, 0, -1));
  explicit PartialPanoramicCamera(const PanoramicCamera &panoCam, int w = 500,
                                  int h = 500);

  Sizei screenSize() const {
    return Size(static_cast<int>(_screenW), static_cast<int>(_screenH));
  }
  double focal() const { return _focal; }
  const Point3 &eye() const { return _eye; }
  const Point3 &center() const { return _center; }
  const Vec3 &up() const { return _up; }
  Point2 toScreen(const Point3 &p3d) const;
  bool isVisibleOnScreen(const Point3 &p3d) const;
  HPoint2 toScreenInHPoint(const Point3 &p3d) const {
    return HPoint2(toScreen(p3d), 1.0);
  }
  Point3 toSpace(const Point2 &p2d) const;
  Point3 toSpace(const Pixel &p) const { return toSpace(Vec2(p.x, p.y)); }
  Vec3 direction(const Point2 &p2d) const;
  Vec3 direction(const Pixel &p) const { return direction(Point2(p.x, p.y)); }
  PanoramicCamera toPanoramic() const {
    return PanoramicCamera(_focal, _eye, _center, _up);
  }

private:
  double _screenW, _screenH;
  double _focal;
  Vec3 _eye, _center, _up;
  Vec3 _xaxis, _yaxis, _zaxis;

  template <class Archive> void serialize(Archive &ar) {
    ar(_screenW, _screenH);
    ar(_focal);
    ar(_eye, _center, _up);
    ar(_xaxis, _yaxis, _zaxis);
  }
  friend class cereal::access;
};

namespace {
template <class T> struct IsCameraImpl {
  template <class TT>
  static auto test(int) -> decltype(
      std::declval<TT>().eye(),
      std::declval<TT>().toScreen(std::declval<core::Point3>()),
      std::declval<TT>().toScreenInHPoint(std::declval<core::Point3>()),
      std::declval<TT>().isVisibleOnScreen(std::declval<core::Point3>()),
      std::declval<TT>().toSpace(std::declval<core::Point2>()),
      std::declval<TT>().toSpace(std::declval<core::Pixel>()),
      std::declval<TT>().screenSize(), std::true_type()) {
    return std::true_type();
  }
  template <class> static std::false_type test(...) {
    return std::false_type();
  }
  static const bool value =
      std::is_same<decltype(test<T>(0)), std::true_type>::value;
};
}

// judge whether T is a camera type
template <class T>
struct IsCamera : std::integral_constant<bool, IsCameraImpl<T>::value> {};

// sample image from image using camera conversion
template <class OutCameraT, class InCameraT> class CameraSampler {
  static_assert(IsCamera<OutCameraT>::value && IsCamera<InCameraT>::value,
                "OutCameraT and InCameraT should both be cameras!");

public:
  template <class OCamT, class ICamT>
  CameraSampler(OCamT &&outCam, ICamT &&inCam)
      : _outCam(std::forward<OCamT>(outCam)),
        _inCam(std::forward<ICamT>(inCam)) {
    assert(outCam.eye() == inCam.eye());
    auto outCamSize = _outCam.screenSize();
    _mapx = cv::Mat::zeros(outCamSize, CV_32FC1);
    _mapy = cv::Mat::zeros(outCamSize, CV_32FC1);
    for (int j = 0; j < outCamSize.height; j++) {
      for (int i = 0; i < outCamSize.width; i++) {
        Vec2 screenp(i, j);
        Vec3 p3 = _outCam.toSpace(screenp);
        if (!_inCam.isVisibleOnScreen(p3)) {
          _mapx.at<float>(j, i) = -1;
          _mapy.at<float>(j, i) = -1;
          continue;
        }
        Vec2 screenpOnInCam = _inCam.toScreen(p3);
        _mapx.at<float>(j, i) = static_cast<float>(screenpOnInCam(0));
        _mapy.at<float>(j, i) = static_cast<float>(screenpOnInCam(1));
      }
    }
  }

  Image operator()(const Image &inputIm, int borderMode = cv::BORDER_REPLICATE,
                   const cv::Scalar &borderValue = cv::Scalar(0, 0, 0,
                                                              0)) const {
    Image outputIm;
    cv::remap(inputIm, outputIm, _mapx, _mapy,
              inputIm.channels() <= 4 ? cv::INTER_LINEAR : cv::INTER_NEAREST,
              borderMode, borderValue);
    return outputIm;
  }

  template <class T>
  Image_<T> operator()(const Image_<T> &inputIm,
                        int borderMode = cv::BORDER_REPLICATE,
                        const T &borderValue = T()) const {
    Image_<T> outputIm;
    cv::remap(inputIm, outputIm, _mapx, _mapy, cv::INTER_NEAREST, borderMode,
              borderValue);
    return outputIm;
  }

  template <class T, int N, class = std::enable_if_t<(N <= 4)>>
  Image_<Vec<T, N>>
  operator()(const Image_<Vec<T, N>> &inputIm,
             int borderMode = cv::BORDER_REPLICATE,
             const Vec<T, N> &borderValue = Vec<T, N>()) const {
    Image outputIm;
    cv::Scalar bv;
    for (int i = 0; i < N; i++) {
      bv[i] = borderValue[i];
    }
    cv::remap(inputIm, outputIm, _mapx, _mapy, cv::INTER_NEAREST, borderMode,
              bv);
    return outputIm;
  }

  template <class T, int N, class = std::enable_if_t<(N > 4)>, class = void>
  Image_<Vec<T, N>>
  operator()(const Image_<Vec<T, N>> &inputIm,
             int borderMode = cv::BORDER_REPLICATE,
             const Vec<T, N> &borderValue = Vec<T, N>()) const {
    std::vector<Image> channels;
    cv::split(inputIm, channels);
    for (int i = 0; i < N; i++) {
      auto &c = channels[i];
      cv::remap(c, c, _mapx, _mapy, cv::INTER_NEAREST, borderMode,
                borderValue[i]);
    }
    Image_<Vec<T, N>> result;
    cv::merge(channels, result);
    return result;
  }

private:
  OutCameraT _outCam;
  InCameraT _inCam;
  cv::Mat _mapx, _mapy;
};

template <class OutCameraT, class InCameraT>
CameraSampler<std::decay_t<OutCameraT>, std::decay_t<InCameraT>>
MakeCameraSampler(OutCameraT &&outCam, InCameraT &&inCam) {
  return CameraSampler<std::decay_t<OutCameraT>, std::decay_t<InCameraT>>(
      std::forward<OutCameraT>(outCam), std::forward<InCameraT>(inCam));
}


// create horizontal cameras
std::vector<PerspectiveCamera>
CreateHorizontalPerspectiveCameras(const PanoramicCamera &panoCam, int num = 16,
                                   int width = 500, int height = 500,
                                   double focal = 250.0);

std::vector<PerspectiveCamera> CreateHorizontalPerspectiveCameras(
    const PanoramicCamera &panoCam, const std::vector<Vec3> &dirs,
    int width = 500, int height = 500, double focal = 250.0,
    double angleThreshold = 0.1);

std::vector<PerspectiveCamera>
CreatePanoContextCameras(const PanoramicCamera &panoCam, int width = 500,
                         int height = 500, double focal = 250.0);

// create cameras toward cubic faces
std::vector<PerspectiveCamera>
CreateCubicFacedCameras(const PanoramicCamera &panoCam, int width = 500,
                        int height = 500, double focal = 250.0);

// view class
template <class CameraT, class ImageT = Image>
class View {
  static_assert(IsCamera<CameraT>::value, "CameraT MUST BE a camera!");
public:
  ImageT image;
  CameraT camera;

public:
  View() {}
  View(const ImageT &im, const CameraT &cam) : image(im), camera(cam) {}
  explicit View(const CameraT &cam)
      : image(ImageT::zeros(cam.screenSize())), camera(cam) {}
  View(const View<CameraT, Image> &v) : image(v.image), camera(v.camera) {}
  template <class T>
  View(const View<CameraT, Image_<T>> &v) : image(v.image), camera(v.camera) {}

public:
  auto size() const {return image.size(); }
  decltype(auto) at(const Vec3 &p3d) const {
    return image(ToPixel(camera.toScreen(p3d)));
  }
  decltype(auto) operator()(const Vec3 &p3d) const { return at(p3d); }
  decltype(auto) operator()(const Vec3 &p3d) {
    return image(ToPixel(camera.toScreen(p3d)));
  }

  template <
      class AnotherCameraT,
      class = std::enable_if_t<IsCamera<std::decay_t<AnotherCameraT>>::value>>
  View<std::decay_t<AnotherCameraT>, ImageT>
  sampled(AnotherCameraT &&cam) const {
    View<std::decay_t<AnotherCameraT>, ImageT> v;
    v.image = MakeCameraSampler(cam, camera)(image);
    v.camera = std::forward<AnotherCameraT>(cam);
    return v;
  }

  template <class Archiver> void serialize(Archiver &ar) { ar(image, camera); }
};

template <class CameraT, class ImageT>
View<CameraT, ImageT> MakeView(const ImageT &im, const CameraT &c) {
  assert(im.size() == c.screenSize());
  return View<CameraT, ImageT>(im, c);
}
template <class T, class CameraT>
View<CameraT, Image_<T>> MakeView(const CameraT &c) {
  return View<CameraT, Image_<T>>(c);
}

template <class OutCameraT, class InCameraT, class T,
          class = std::enable_if_t<IsCamera<std::decay_t<InCameraT>>::value &&
                                   IsCamera<std::decay_t<OutCameraT>>::value>>
View<OutCameraT, Image_<T>>
Combine(const OutCameraT &camera,
        const std::vector<View<InCameraT, Image_<T>>> &views) {
  if (views.empty()) {
    return View<OutCameraT, Image_<T>>();
  }
  Image_<T> converted = Image_<T>::zeros(camera.screenSize());
  static const int channels = cv::DataType<T>::channels;
  Imagef counts(camera.screenSize(), 0.0f);
  for (int i = 0; i < views.size(); i++) {
    auto sampler = MakeCameraSampler(camera, views[i].camera);
    auto piece = sampler(views[i].image, cv::BORDER_CONSTANT);
    converted += piece;
    counts += sampler(Imagef::ones(views[i].image.size()), cv::BORDER_CONSTANT);
  }
  View<OutCameraT, Image_<T>> v;
  v.camera = camera;
  v.image = Image_<T>::zeros(camera.screenSize());
  for (auto it = converted.begin(); it != converted.end(); ++it) {
    float count = counts(it.pos());
    v.image(it.pos()) = *it / std::max(count, 1.0f);
  }
  return v;
}

template <class OutCameraT, class InCameraT, class T, class W,
          class = std::enable_if_t<IsCamera<std::decay_t<InCameraT>>::value &&
                                   IsCamera<std::decay_t<OutCameraT>>::value>>
View<OutCameraT, Image_<T>>
Combine(const OutCameraT &camera,
        const std::vector<Weighted<View<InCameraT, Image_<T>>, W>> &views) {
  if (views.empty()) {
    return View<OutCameraT, Image_<T>>();
  }
  static const int channels = cv::DataType<T>::channels;
  using channel_type = typename cv::DataType<T>::channel_type;
  Image_<T> converted = Image_<T>::zeros(camera.screenSize());
  Imagef counts(camera.screenSize(), 0.0f);
  for (int i = 0; i < views.size(); i++) {
    auto sampler = MakeCameraSampler(camera, views[i].component.camera);
    auto piece = sampler(views[i].component.image, cv::BORDER_CONSTANT);
    for (auto it = converted.begin(); it != converted.end(); ++it) {
      *it += (piece(it.pos()) * views[i].weight());
    }
    counts +=
        sampler(Imagef(views[i].component.image.size(), views[i].weight()),
                cv::BORDER_CONSTANT);
  }
  View<OutCameraT, Image_<T>> v;
  v.camera = camera;
  v.image = Image_<T>::zeros(camera.screenSize());
  for (auto it = converted.begin(); it != converted.end(); ++it) {
    float count = counts(it.pos());
    v.image(it.pos()) = *it / std::max(count, 1.0f);
  }
  return v;
}

template <class CameraT, class InCameraT, class T>
T MeanInMask(const View<InCameraT, Image_<T>> &source,
             const View<CameraT, Imageub> &maskView) {
  Image_<T> converted = source.sampled(maskView.camera).image;
  int votes = 0;
  T featureSum;
  for (auto it = maskView.image.begin(); it != maskView.image.end(); ++it) {
    if (!*it) {
      continue;
    }
    featureSum += converted(it.pos());
    votes += 1;
  }
  return featureSum * (1.0 / std::max(votes, 1));
}

using PerspectiveView = View<PerspectiveCamera>;
using PanoramicView = View<PanoramicCamera>;
using PartialPanoramicView = View<PartialPanoramicCamera>;

PanoramicCamera CreatePanoramicCamera(const Image &panorama,
                                      const Point3 &eye = Point3(0, 0, 0),
                                      const Point3 &center = Point3(1, 0, 0),
                                      const Vec3 &up = Vec3(0, 0, 1));

PerspectiveCamera CreatePerspeciveCamera(const Point3 &eye,
                                         const Point3 &center,
                                         const Sizei &ssize, double focal,
                                         const Point2 &pp);

Failable<PerspectiveCamera> CreatePerspectiveCamera(
    const Image &perspectiveImage, const Point3 &eye = Point3(0, 0, 0),
    const Point3 &center = Point3(1, 0, 0), const Vec3 &up = Vec3(0, 0, -1),
    const LineSegmentExtractor &lse = LineSegmentExtractor(),
    const VanishingPointsDetector &vpd = VanishingPointsDetector(),
    std::vector<Classified<Line3>> *line3s = nullptr,
    std::vector<Classified<Line2>> *line2s = nullptr,
    std::vector<Vec3> *vps = nullptr, double *focal = nullptr);

PerspectiveCamera CreatePerspectiveCamera(
    const Image &perspectiveImage, const std::vector<HPoint2> &vps,
    const Point3 &eye = Point3(0, 0, 0), const Point3 &center = Point3(1, 0, 0),
    const Vec3 &up = Vec3(0, 0, -1));

// create panoramic view
inline PanoramicView CreatePanoramicView(const Image &panorama,
                                         const Point3 &eye = Point3(0, 0, 0),
                                         const Point3 &center = Point3(1, 0, 0),
                                         const Vec3 &up = Vec3(0, 0, 1)) {
  return PanoramicView(panorama,
                       CreatePanoramicCamera(panorama, eye, center, up));
}
template <class T>
View<PanoramicCamera, Image_<T>> CreatePanoramicView(
    const Image_<T> &panorama, const Point3 &eye = Point3(0, 0, 0),
    const Point3 &center = Point3(1, 0, 0), const Vec3 &up = Vec3(0, 0, 1)) {
  return View<PanoramicCamera, Image_<T>>(
      panorama, CreatePanoramicCamera(panorama, eye, center, up));
}

// create perspective view
inline Failable<PerspectiveView> CreatePerspectiveView(
    const Image &perspectiveImage, const Point3 &eye = Point3(0, 0, 0),
    const Point3 &center = Point3(1, 0, 0), const Vec3 &up = Vec3(0, 0, -1),
    const LineSegmentExtractor &lse = LineSegmentExtractor(),
    const VanishingPointsDetector &vpd = VanishingPointsDetector(),
    std::vector<Classified<Line3>> *line3s = nullptr,
    std::vector<Classified<Line2>> *line2s = nullptr,
    std::vector<Vec3> *vps = nullptr, double *focal = nullptr) {
  auto cam = CreatePerspectiveCamera(perspectiveImage, eye, center, up, lse,
                                     vpd, line3s, line2s, vps, focal);
  if (cam.failed())
    return nullptr;
  return PerspectiveView(perspectiveImage, cam.unwrap());
}
template <class T>
Failable<View<PerspectiveCamera, Image_<T>>> CreatePerspectiveView(
    const Image_<T> &perspectiveImage, const Point3 &eye = Point3(0, 0, 0),
    const Point3 &center = Point3(1, 0, 0), const Vec3 &up = Vec3(0, 0, -1),
    const LineSegmentExtractor &lse = LineSegmentExtractor(),
    const VanishingPointsDetector &vpd = VanishingPointsDetector(),
    std::vector<Classified<Line3>> *line3s = nullptr,
    std::vector<Classified<Line2>> *line2s = nullptr,
    std::vector<Vec3> *vps = nullptr, double *focal = nullptr) {
  auto cam = CreatePerspectiveCamera(perspectiveImage, eye, center, up, lse,
                                     vpd, line3s, line2s, vps, focal);
  if (cam.failed())
    return nullptr;
  return View<PerspectiveCamera, Image_<T>>(perspectiveImage, cam.unwrap());
}

inline PerspectiveView CreatePerspectiveView(
    const Image &perspectiveImage, const std::vector<HPoint2> &vps,
    const Point3 &eye = Point3(0, 0, 0), const Point3 &center = Point3(1, 0, 0),
    const Vec3 &up = Vec3(0, 0, -1)) {
  return PerspectiveView(
      perspectiveImage,
      CreatePerspectiveCamera(perspectiveImage, vps, eye, center, up));
}
template <class T>
View<PerspectiveCamera, Image_<T>> CreatePerspectiveView(
    const Image_<T> &perspectiveImage, const std::vector<HPoint2> &vps,
    const Point3 &eye = Point3(0, 0, 0), const Point3 &center = Point3(1, 0, 0),
    const Vec3 &up = Vec3(0, 0, -1)) {
  return View<PerspectiveCamera, Image_<T>>(
      perspectiveImage,
      CreatePerspectiveCamera(perspectiveImage, vps, eye, center, up));
}

template <class CameraT, class = std::enable_if_t<IsCamera<CameraT>::value>>
Failable<View<CameraT>> CreateView(const Image &image,
                                   const Point3 &eye = Point3(0, 0, 0),
                                   const Point3 &center = Point3(1, 0, 0),
                                   const Vec3 &up = Vec3(0, 0, -1));

template <class CameraT, class T,
          class = std::enable_if_t<IsCamera<CameraT>::value>>
Failable<View<CameraT, Image_<T>>>
CreateView(const Image_<T> &image, const Point3 &eye = Point3(0, 0, 0),
           const Point3 &center = Point3(1, 0, 0),
           const Vec3 &up = Vec3(0, 0, -1)) {
  auto v = CreateView<CameraT>((const Image &)image, eye, center, up);
  if (v.failed())
    return nullptr;
  return View<CameraT, Image_<T>>(image, v.unwrap.camera);
}

template <>
inline Failable<View<PanoramicCamera>>
CreateView(const Image &image, const Point3 &eye, const Point3 &center,
           const Vec3 &up) {
  return CreatePanoramicView(image, eye, center, up);
}

template <>
inline Failable<View<PerspectiveCamera>>
CreateView(const Image &image, const Point3 &eye, const Point3 &center,
           const Vec3 &up) {
  return CreatePerspectiveView(image, eye, center, up);
}
}
}
