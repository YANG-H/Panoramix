#include "pch.hpp"

#include "cameras.hpp"
#include "utility.hpp"

namespace pano {
namespace core {

PerspectiveCamera::PerspectiveCamera()
    : _screenW(500), _screenH(500), _principlePoint(250, 250),
      _focalxy(250, 250), _eye(0, 0, 0), _center(1, 0, 0), _up(0, 0, -1),
      _near(0.01), _far(1e4) {
  updateMatrices();
}

PerspectiveCamera::PerspectiveCamera(int w, int h)
    : _screenW(w), _screenH(h), _principlePoint(w / 2.0, h / 2.0),
      _focalxy(250, 250), _eye(0, 0, 0), _center(1, 0, 0), _up(0, 0, -1),
      _near(0.01), _far(1e4) {
  updateMatrices();
}

PerspectiveCamera::PerspectiveCamera(int w, int h, const Point2 &pp,
                                     const Vec2 &focalxy, const Vec3 &eye,
                                     const Vec3 &center, const Vec3 &up,
                                     double n, double f)
    : _screenW(w), _screenH(h), _principlePoint(pp), _focalxy(focalxy),
      _eye(eye), _center(center), _up(up), _near(n), _far(f) {
  updateMatrices();
}

PerspectiveCamera::PerspectiveCamera(int w, int h, const Point2 &pp,
                                     double focal, const Vec3 &eye,
                                     const Vec3 &center, const Vec3 &up,
                                     double n, double f)
    : _screenW(w), _screenH(h), _principlePoint(pp), _focalxy(focal, focal),
      _eye(eye), _center(center), _up(up), _near(n), _far(f) {
  updateMatrices();
}

void PerspectiveCamera::updateMatrices() {
  _viewMatrix = MakeMat4LookAt(_eye, _center, _up);
  _projectionMatrix =
      MakeMat4Perspective(_focalxy[0], _focalxy[1], _principlePoint[0],
                          _principlePoint[1], _near, _far);

  _viewProjectionMatrix = _projectionMatrix * _viewMatrix;
}

Point2 PerspectiveCamera::toScreen(const Point3 &p3) const {
  // THERE_ARE_BUGS_HERE("the input is not a point but a direction ?!!!!
  // CONFUSION");
  Vec4 p4(p3(0), p3(1), p3(2), 1);
  Vec4 position = _viewProjectionMatrix * p4;
  double xratio = position(0) / position(3) / 2;
  double yratio = position(1) / position(3) / 2;
  double x = (xratio + 0.5) * 2.0 * _principlePoint[0];
  double y =
      2.0 * _principlePoint[1] - (yratio + 0.5) * 2.0 * _principlePoint[1];
  return Vec2(x, y);
}

bool PerspectiveCamera::isVisibleOnScreen(const Point3 &p3d) const {
  Vec4 p4(p3d(0), p3d(1), p3d(2), 1);
  Vec4 position = _viewProjectionMatrix * p4;
  return position(3) > 0 && position(2) > 0;
}

HPoint2 PerspectiveCamera::toScreenInHPoint(const Point3 &p3) const {
  // THERE_ARE_BUGS_HERE("the input is not a point but a direction ?!!!!
  // CONFUSION");
  Vec4 p4(p3(0), p3(1), p3(2), 1);
  Vec4 position = _viewProjectionMatrix * p4;
  double xratio = position(0) / 2;
  double yratio = position(1) / 2;
  double zratio = position(3);

  double x = (xratio + 0.5 * zratio) * /*_screenW*/ (2.0 * _principlePoint[0]);
  double y = /*_screenH*/ (2.0 * _principlePoint[1]) * zratio -
             (yratio + 0.5 * zratio) * /*_screenH*/ (2.0 * _principlePoint[1]);
  return HPoint2({x, y}, zratio);
}

Point3 PerspectiveCamera::toSpace(const Point2 &p2d) const {
  double xratio = (p2d(0) / /*_screenW*/ (2.0 * _principlePoint[0]) - 0.5) * 2;
  double yratio = ((/*_screenH*/ (2.0 * _principlePoint[1]) - p2d(1)) /
                       /*_screenH*/ (2.0 * _principlePoint[1]) -
                   0.5) *
                  2;
  Vec4 position(xratio, yratio, 1, 1);
  Vec4 realPosition;
  bool solvable =
      cv::solve(_viewProjectionMatrix, position,
                realPosition); // _viewProjectionMatrixInv * position;
  assert(solvable);
  return Vec3(realPosition(0) / realPosition(3),
              realPosition(1) / realPosition(3),
              realPosition(2) / realPosition(3));
}

void PerspectiveCamera::resizeScreen(const Size &sz, bool updateMat) {
  if (_screenH == sz.height && _screenW == sz.width)
    return;
  auto offset = _principlePoint - Point2(_screenW / 2.0, _screenH / 2.0);
  _principlePoint = Point2(sz.width / 2.0, sz.height / 2.0) + offset;
  _screenH = sz.height;
  _screenW = sz.width;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setPrinciplePoint(const Point2 &pp, bool updateMat) {
  if (_principlePoint == pp)
    return;
  _principlePoint = pp;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setFocalX(double fx, bool updateMat) {
  if (fx == _focalxy[0])
    return;
  _focalxy[0] = fx;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setFocalY(double fy, bool updateMat) {
  if (fy == _focalxy[1])
    return;
  _focalxy[1] = fy;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setFocal(double f, bool updateMat) {
  if (f == _focalxy[0] && f == _focalxy[1])
    return;
  _focalxy[0] = f;
  _focalxy[1] = f;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setEye(const Vec3 &e, bool updateMat) {
  if (_eye == e)
    return;
  _eye = e;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setCenter(const Vec3 &c, bool updateMat) {
  if (_center == c)
    return;
  _center = c;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setUp(const Vec3 &up, bool updateMat) {
  if (_up == up)
    return;
  _up = up;
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::setNearAndFarPlanes(double n, double f,
                                            bool updateMat) {
  if (_near == n && _far == f)
    return;
  _near = n;
  _far = f;
  if (updateMat)
    updateMatrices();
}

inline void AdjustNearAndFar(double &n, double &f, const Sphere3 &target,
                             const Vec3 &eye) {
  n = BoundBetween(norm(target.center - eye) - target.radius, 1e-2, 1e8);
  f = BoundBetween(norm(target.center - eye) + target.radius, 1e2, 1e8);
}

void PerspectiveCamera::focusOn(const Sphere3 &target, bool updateMat) {
  _center = target.center;
  auto eyedirection = _eye - _center;
  eyedirection = eyedirection / core::norm(eyedirection) * target.radius * 0.8;
  _eye = _center + eyedirection;
  AdjustNearAndFar(_near, _far, target, _eye);
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::translate(const Vec3 &t, const Sphere3 &target,
                                  bool updateMat) {
  _eye += t;
  _center += t;
  AdjustNearAndFar(_near, _far, target, _eye);
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::moveEyeWithCenterFixed(const Vec3 &t,
                                               const Sphere3 &target,
                                               bool distanceFixed,
                                               bool updateMat, bool upFixed) {
  double dist = norm(_eye - _center);
  _eye += t;
  if (distanceFixed) {
    _eye = normalize(_eye - _center) * dist + _center;
  }
  if (!upFixed) {
    _up = rightward().cross(forward());
  }
  AdjustNearAndFar(_near, _far, target, _eye);
  if (updateMat)
    updateMatrices();
}

void PerspectiveCamera::moveCenterWithEyeFixed(const Vec3 &t, bool updateMat) {
  float sc = std::max(screenSize().width, screenSize().height) * 0.2;
  Vec3 tt = t * norm(eye() - center()) / sc;

  Vec3 xv = rightward();
  Vec3 yv = upward();
  auto xyTrans = xv * tt[0] + yv * tt[1];
  double r =
      (norm(eye() - center()) - tt[2]) / norm(eye() + xyTrans - center());
  _center = (center() + xyTrans - eye()) * r + eye();
  _up = normalize(yv);
  if (updateMat)
    updateMatrices();
}

bool PerspectiveCamera::operator==(const PerspectiveCamera &cam) const {
  /* ar(_screenW, _screenH);
   ar(_principlePoint);
   ar(_focalxy, _near, _far);
   ar(_eye, _center, _up);
   ar(_viewMatrix, _projectionMatrix, _viewProjectionMatrix);*/
  return std::tie(_screenW, _screenH, _principlePoint, _focalxy, _near, _far,
                  _eye, _center, _up, _viewMatrix, _projectionMatrix,
                  _viewProjectionMatrix) ==
         std::tie(cam._screenW, cam._screenH, cam._principlePoint, cam._focalxy,
                  cam._near, cam._far, cam._eye, cam._center, cam._up,
                  cam._viewMatrix, cam._projectionMatrix,
                  cam._viewProjectionMatrix);
}

PanoramicCamera::PanoramicCamera(double focal, const Vec3 &eye,
                                 const Vec3 &center, const Vec3 &up)
    : _focal(focal), _eye(eye), _center(center), _up(up) {
  _xaxis = (_center - _eye);
  _xaxis /= core::norm(_xaxis);
  _yaxis = _up.cross(_xaxis);
  _yaxis /= core::norm(_yaxis);
  _zaxis = _xaxis.cross(_yaxis);
}

Point2 PanoramicCamera::toScreen(const Point3 &p3) const {
  double xx = (p3 - _eye).dot(_xaxis);
  double yy = (p3 - _eye).dot(_yaxis);
  double zz = (p3 - _eye).dot(_zaxis);
  GeoCoord pg = core::Vec3(xx, yy, zz);
  auto sz = screenSize();
  double x = (pg.longitude + M_PI) / 2.0 / M_PI * sz.width;
  double y = (pg.latitude + M_PI_2) / M_PI * sz.height;
  return Point2(x, y);
}

Point3 PanoramicCamera::toSpace(const Point2 &p2d) const {
  return direction(p2d) + _eye;
}

Vec3 PanoramicCamera::direction(const Point2 &p2d) const {
  auto sz = screenSize();
  double longi = p2d(0) / double(sz.width) * 2 * M_PI - M_PI;
  double lati = p2d(1) / double(sz.height) * M_PI - M_PI_2;
  Vec3 dd = (GeoCoord(longi, lati).toVector());
  return dd(0) * _xaxis + dd(1) * _yaxis + dd(2) * _zaxis;
}

PartialPanoramicCamera::PartialPanoramicCamera(int w, int h, double focal,
                                               const Vec3 &eye,
                                               const Vec3 &center,
                                               const Vec3 &up)
    : _screenW(w), _screenH(h), _focal(focal), _eye(eye), _center(center),
      _up(up) {
  _xaxis = (_center - _eye);
  _xaxis /= core::norm(_xaxis);
  _yaxis = _up.cross(_xaxis);
  _yaxis /= core::norm(_yaxis);
  _zaxis = normalize(_xaxis.cross(_yaxis));
}

PartialPanoramicCamera::PartialPanoramicCamera(const PanoramicCamera &panoCam,
                                               int w, int h)
    : _screenW(w), _screenH(h), _focal(panoCam.focal()), _eye(panoCam.eye()),
      _center(panoCam.center()), _up(panoCam.up()) {
  _xaxis = (_center - _eye);
  _xaxis /= core::norm(_xaxis);
  _yaxis = _up.cross(_xaxis);
  _yaxis /= core::norm(_yaxis);
  _zaxis = normalize(_xaxis.cross(_yaxis));
}

Vec2 PartialPanoramicCamera::toScreen(const Vec3 &p3) const {
  double xx = (p3 - _eye).dot(_xaxis);
  double yy = (p3 - _eye).dot(_yaxis);
  double zz = (p3 - _eye).dot(_zaxis);
  GeoCoord pg = core::Vec3(xx, yy, zz);

  double halfLongitudeAngleBound = _screenW / 2.0 / _focal;
  double halfLatitudeAngleBound = _screenH / 2.0 / _focal;

  double x = (pg.longitude + halfLongitudeAngleBound) * _focal;
  double y = (pg.latitude + halfLatitudeAngleBound) * _focal;

  return Vec2(x, y);
}

bool PartialPanoramicCamera::isVisibleOnScreen(const Vec3 &p3) const {
  double xx = (p3 - _eye).dot(_xaxis);
  double yy = (p3 - _eye).dot(_yaxis);
  double zz = (p3 - _eye).dot(_zaxis);
  GeoCoord pg = core::Vec3(xx, yy, zz);

  double halfLongitudeAngleBound = _screenW / 2.0 / _focal;
  double halfLatitudeAngleBound = _screenH / 2.0 / _focal;

  double x = (pg.longitude + halfLongitudeAngleBound) * _focal;
  double y = (pg.latitude + halfLatitudeAngleBound) * _focal;

  return IsBetween(x, 0, _screenW) && IsBetween(y, 0, _screenH);
}

Point3 PartialPanoramicCamera::toSpace(const Point2 &p2d) const {
  return direction(p2d) + _eye;
}

Vec3 PartialPanoramicCamera::direction(const Point2 &p2d) const {
  double halfLongitudeAngleBound = _screenW / 2.0 / _focal;
  double halfLatitudeAngleBound = _screenH / 2.0 / _focal;

  double longi = p2d(0) / _focal - halfLongitudeAngleBound;
  double lati = p2d(1) / _focal - halfLatitudeAngleBound;

  Vec3 dd = (GeoCoord(longi, lati).toVector());
  return dd(0) * _xaxis + dd(1) * _yaxis + dd(2) * _zaxis;
}

namespace {

inline double UniformSphericalAngleToScreenLength(double angle, double focal) {
  assert(angle <= M_PI && angle >= 0);
  double len = sqrt(2.0 * (1.0 - cos(angle))) * focal;
  assert(len >= 0);
  return len;
}

inline double UniformSphericalScreenLengthToAngle(double len, double focal) {
  double angle = acos(1.0 - Square(len / focal) / 2.0);
  // assert(angle <= M_PI && angle >= 0);
  return angle;
}
}

// UniformSphericalCamera::UniformSphericalCamera(double focal, double
// angleRadius,
//    const Point3 & eye /*= Origin()*/, const Point3 & center /*= X()*/, const
//    Vec3 & up /*= -Z()*/)
//    : _focal(focal),
//    _screenRadius(UniformSphericalAngleToScreenLength(angleRadius, focal)),
//    _eye(eye), _center(center), _up(up) {
//    _xaxis = (_center - _eye); _xaxis /= core::norm(_xaxis);
//    _yaxis = _up.cross(_xaxis); _yaxis /= core::norm(_yaxis);
//    _zaxis = normalize(_xaxis.cross(_yaxis));
//}

// Point2 UniformSphericalCamera::toScreen(const Point3 & p3) const {
//    double yy = (p3 - _eye).dot(_yaxis);
//    double zz = (p3 - _eye).dot(_zaxis);
//    double a = sqrt(yy * yy + zz * zz);
//    yy /= a;
//    zz /= a;
//    double theta = AngleBetweenDirected(p3 - _eye, _center - _eye);
//    double len = UniformSphericalAngleToScreenLength(theta, _focal);
//    return Point2(_screenRadius + yy * len, _screenRadius + zz * len);
//}

// bool UniformSphericalCamera::isVisibleOnScreen(const Point3 & p3d) const {
//    Point2 p2 = toScreen(p3d);
//    return p2[0] >= 0 && p2[0] <= 2 * _screenRadius && p2[1] >= 0 && p2[1] <=
//    2 * _screenRadius;
//}

// Point3 UniformSphericalCamera::toSpace(const Point2 & p2d) const {
//    Vec2 v2 = p2d - Point2(_screenRadius, _screenRadius);
//    Vec3 v3 = normalize(_yaxis * v2[0] + _zaxis * v2[1]);
//    double len = Distance(p2d, Point2(_screenRadius, _screenRadius));
//    double theta = UniformSphericalScreenLengthToAngle(len, _focal);
//    return v3 * sin(theta) * Distance(_eye, _center) + _center * cos(theta);
//}

std::vector<PerspectiveCamera>
CreateHorizontalPerspectiveCameras(const PanoramicCamera &panoCam, int num,
                                   int width, int height, double focal) {
  std::vector<PerspectiveCamera> cams(num);
  Vec3 x, y;
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(-panoCam.up());
  for (int i = 0; i < num; i++) {
    double angle = M_PI * 2.0 / num * i;
    Vec3 direction = sin(angle) * x + cos(angle) * y;
    cams[i] = PerspectiveCamera(width, height, Point2(width, height) / 2.0,
                                focal, panoCam.eye(), panoCam.eye() + direction,
                                -panoCam.up(), 0.01, 1e4);
  }
  return cams;
}

std::vector<PerspectiveCamera> CreateHorizontalPerspectiveCameras(
    const PanoramicCamera &panoCam, const std::vector<Vec3> &dirs, int width,
    int height, double focal, double angleThreshold) {
  std::vector<PerspectiveCamera> cams;
  for (int i = 0; i < dirs.size(); i++) {
    if (AngleBetweenUndirected(dirs[i], panoCam.up()) < angleThreshold) {
      continue;
    }
    cams.emplace_back(width, height, Point2(width, height) / 2.0, focal,
                      panoCam.eye(), panoCam.eye() + dirs[i], -panoCam.up(),
                      0.01, 1e4);
    cams.emplace_back(width, height, Point2(width, height) / 2.0, focal,
                      panoCam.eye(), panoCam.eye() - dirs[i], -panoCam.up(),
                      0.01, 1e4);
  }
  return cams;
}

std::vector<PerspectiveCamera>
CreatePanoContextCameras(const PanoramicCamera &panoCam, int width, int height,
                         double focal) {
  double fov = M_PI / 3;
  std::vector<double> xh(12), yh(12);
  for (int i = 0; i < xh.size(); i++) {
    xh[i] = M_PI / 6 * i - M_PI;
    yh[i] = 0;
  }

  std::vector<double> xp, yp;
  xp = {-3 / 3.0, -2 / 3.0, -1 / 3.0, +0 / 3.0, +1 / 3.0, +2 / 3.0,
        -3 / 3.0, -2 / 3.0, -1 / 3.0, +0 / 3.0, +1 / 3.0, +2 / 3.0};
  yp = {1 / 4.0,  1 / 4.0,  1 / 4.0,  1 / 4.0,  1 / 4.0,  1 / 4.0,
        -1 / 4.0, -1 / 4.0, -1 / 4.0, -1 / 4.0, -1 / 4.0, -1 / 4.0};
  for (int i = 0; i < xp.size(); i++) {
    xp[i] *= M_PI;
    yp[i] *= M_PI;
  }

  Vec3 x, y, z = normalize(-panoCam.up());
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(z);
  std::vector<Vec3> directions(24);
  for (int i = 0; i < 12; i++) {
    auto &d = directions[i];
    double xv = xp[i], yv = yp[i];
    d = cos(xv) * cos(yv) * x + sin(xv) * cos(yv) * y + sin(yv) * z;
    auto &d2 = directions[i + 12];
    xv = xh[i], yv = yh[i];
    d2 = cos(xv) * cos(yv) * x + sin(xv) * cos(yv) * y + sin(yv) * z;
  }

  std::vector<PerspectiveCamera> pcams;
  pcams.reserve(directions.size());
  for (const Vec3 &d : directions) {
    PerspectiveCamera cam(width, height, Point2(width, height) / 2.0, focal,
                          panoCam.eye(), panoCam.eye() - d, z, 0.01, 1e4);
    pcams.push_back(cam);
  }
  return pcams;
}

std::vector<PerspectiveCamera>
CreateCubicFacedCameras(const PanoramicCamera &panoCam, int width /* = 500*/,
                        int height /* = 500*/, double focal /* = 250.0*/) {
  Vec3 x, y;
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(-panoCam.up());
  std::vector<core::PerspectiveCamera> cams = {
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{1, 0, 0},
                              -panoCam.up()),
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{0, 1, 0},
                              -panoCam.up()),
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{-1, 0, 0},
                              -panoCam.up()),
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{0, -1, 0},
                              -panoCam.up()),
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{0, 0, 1}, x),
      core::PerspectiveCamera(width, height, Point2(width, height) / 2.0, focal,
                              panoCam.eye(), panoCam.eye() + Vec3{0, 0, -1},
                              x)};
  return cams;
}

PerspectiveCamera CreatePerspeciveCamera(const Point3 &eye,
                                         const Point3 &center,
                                         const Sizei &ssize, double focal,
                                         const Point2 &pp) {
  Vec3 x, y;
  std::tie(x, y) = ProposeXYDirectionsFromZDirection(center - eye);
  return PerspectiveCamera(ssize.width, ssize.height, pp, focal, eye, center,
                           y);
}

PanoramicCamera CreatePanoramicCamera(const Image &panorama, const Point3 &eye,
                                      const Point3 &center, const Vec3 &up) {
  assert(abs(panorama.cols - panorama.rows * 2) < panorama.rows / 10.0f);
  return PanoramicCamera(panorama.cols / M_PI / 2.0, eye, center, up);
}

Failable<PerspectiveCamera>
CreatePerspectiveCamera(const Image &perspectiveImage, const Point3 &eye,
                        const Point3 &center, const Vec3 &up,
                        const LineSegmentExtractor &lse,
                        const VanishingPointsDetector &vpd,
                        std::vector<Classified<Line3>> *line3sPtr,
                        std::vector<Classified<Line2>> *line2sPtr,
                        std::vector<Vec3> *vpsPtr, double *focalPtr) {

  auto lines = lse(perspectiveImage, 2);
  std::vector<HPoint2> vps;
  double focal;
  std::vector<int> lineClasses;
  int maxTry = 500;
  while (vps.empty()) {
    auto result = vpd(lines, perspectiveImage.size());
    if (--maxTry < 0)
      break;
    if (result.failed()) {
      std::srand(clock());
      continue;
    }
    std::tie(vps, focal, lineClasses) = result.unwrap();
  }
  if (vps.size() < 3) {
    std::cerr << "vps.size() < 3!" << std::endl;
    return nullptr;
  }

  Point2 principlePoint;
  std::tie(principlePoint, std::ignore) = ComputePrinciplePointAndFocalLength(
      vps[0].value(), vps[1].value(), vps[2].value());

  // only reserve first 3 vps
  vps.erase(vps.begin() + 3, vps.end());
  for (auto &c : lineClasses) {
    if (c >= 3)
      c = -1;
  }

  PerspectiveCamera camera(perspectiveImage.cols, perspectiveImage.rows,
                           principlePoint, focal, eye, center, up);

  if (line3sPtr) {
    std::vector<Classified<Line3>> line3s(lines.size());
    for (int i = 0; i < lines.size(); i++) {
      line3s[i].component.first = camera.toSpace(lines[i].first);
      line3s[i].component.second = camera.toSpace(lines[i].second);
      line3s[i].claz = lineClasses[i];
    }
    *line3sPtr = std::move(line3s);
  }
  if (line2sPtr) {
    auto line2s = ClassifyEachAs(lines, -1);
    for (int i = 0; i < lines.size(); i++)
      line2s[i].claz = lineClasses[i];
    *line2sPtr = std::move(line2s);
  }
  if (vpsPtr) {
    std::vector<Vec3> vp3s(vps.size());
    for (int i = 0; i < vps.size(); i++) {
      vp3s[i] = normalize(camera.toSpace(vps[i].value()));
    }
    *vpsPtr = std::move(vp3s);
  }
  if (focalPtr) {
    *focalPtr = focal;
  }

  return std::move(camera);
}

PerspectiveCamera CreatePerspectiveCamera(const Image &perspectiveImage,
                                          const std::vector<HPoint2> &vps,
                                          const Point3 &eye,
                                          const Point3 &center,
                                          const Vec3 &up) {

  double focal;
  Point2 principlePoint;
  std::tie(principlePoint, focal) = ComputePrinciplePointAndFocalLength(
      vps[0].value(), vps[1].value(), vps[2].value());

  return PerspectiveCamera(perspectiveImage.cols, perspectiveImage.rows,
                           principlePoint, focal, eye, center, up);
}
}
}