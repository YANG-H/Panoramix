#pragma once

#include "basic_types.hpp"
#include "cameras.hpp"

namespace pano {
namespace core {

// ComputeSpatialRegionProperties
std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PerspectiveCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr = nullptr,
    std::vector<Vec3> *ncentersPtr = nullptr,
    std::vector<double> *areasPtr = nullptr);
std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PanoramicCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr = nullptr,
    std::vector<Vec3> *ncentersPtr = nullptr,
    std::vector<double> *areasPtr = nullptr);
std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PartialPanoramicCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr = nullptr,
    std::vector<Vec3> *ncentersPtr = nullptr,
    std::vector<double> *areasPtr = nullptr);

// PerfectRegionMaskView
View<PartialPanoramicCamera, Imageub>
PerfectRegionMaskView(const std::vector<std::vector<Vec3>> &contours,
                      const Vec3 &center, double focal);
inline View<PartialPanoramicCamera, Imageub>
PerfectRegionMaskView(const std::vector<Vec3> &contours, const Vec3 &center,
                      double focal) {
  return PerfectRegionMaskView(std::vector<std::vector<Vec3>>{contours}, center,
                               focal);
}

// ForEachPixelWithinViewCone
template <class T, class FunT>
void ForEachPixelWithinViewCone(const View<PanoramicCamera, Image_<T>> &view,
                                const Vec3 &center, double angle_radius,
                                FunT fun);
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace core {
template <class T, class FunT>
void ForEachPixelWithinViewCone(const View<PanoramicCamera, Image_<T>> &view,
                                const Vec3 &center, double angle_radius,
                                FunT fun) {
  double R = view.camera.focal(); // the radius of sphere
  double alpha = angle_radius;
  double phi = AngleBetweenDirected(
      center, view.camera.up()); // angle from center to up direction

  // line equation of the 2d projection of the circle plane is
  // sin(\phi) x + cos(\phi) y = R cos(\alpha)
  // -> x = (R cos(\alpha) - cos(\phi) y) / sin(\phi)

  // half span angle
  // \beta = acos(x / \sqrt(R^2 - y^2))

  Point2 pcenter = view.camera.toScreen(center);
  for (int row = 0; row < view.image.rows; row++) {
    double vertical_angle = (row / double(view.image.rows) - 0.5) * M_PI;
    double y = R * sin(vertical_angle);
    double x = (R * cos(alpha) - cos(phi) * y) / sin(phi);

    double circle_radius_this_row = cos(vertical_angle) * R;
    assert(circle_radius_this_row >= 0);
    double beta = 0.0;
    if (x >= circle_radius_this_row) {
      beta = 0.0;
    } else if (x <= -circle_radius_this_row) {
      beta = M_PI;
    } else {
      beta = acos(x / circle_radius_this_row);
    }
    if (beta == 0) {
      continue;
    }

    double half_span_cols = beta * R;
    for (int col = static_cast<int>(pcenter[0] - half_span_cols);
         col <= static_cast<int>(pcenter[0] + half_span_cols); col++) {
      fun(Pixel(WrapBetween(col, 0, view.image.cols), row));
    }
  }
}
}
}

