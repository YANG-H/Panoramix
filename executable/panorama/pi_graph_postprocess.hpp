#pragma once

#include "pi_graph_solve.hpp"

namespace pano {
namespace experimental {

// get the CompactModel
std::vector<Polygon3> CompactModel(const PICGDeterminablePart &dp,
                                   const PIConstraintGraph &cg,
                                   const PIGraph<PanoramicCamera> &mg, double distThres);

template <class T> void FillHoles(Image_<T> &im, const Imageb &holeMask) {
  // fill the holes
  for (auto it = im.begin(); it != im.end(); ++it) {
    auto p = it.pos();
    if (!holeMask(p)) {
      continue;
    }
    int located = 0;
    T valueSum = 0;
    static const int thres = 100;
    for (int d = 1;; d++) {
      if (located >= thres) {
        break;
      }
      for (int dd = -d; dd <= d; dd++) {
        if (located >= thres) {
          break;
        }
        Pixel ps[] = {Pixel(p.x + dd, p.y + d), Pixel(p.x + dd, p.y - d),
                      Pixel(p.x + d, p.y + dd), Pixel(p.x - d, p.y + dd)};
        for (auto pp : ps) {
          if (!IsBetween(pp.y, 0, im.rows)) {
            continue;
          }
          pp.x = WrapBetween(pp.x, 0, im.cols);
          auto anotherValue = im(pp);
          if (!holeMask(pp)) {
            valueSum += anotherValue;
            located++;
          }
        }
      }
    }
    *it = valueSum / std::max(located, 1);
  }
}

std::vector<Vec3> ComputeSegNormals(const PICGDeterminablePart &dp,
                                    const PIConstraintGraph &cg,
                                    const PIGraph<PanoramicCamera> &mg, bool smoothed);

template <class CameraT>
Image3d SurfaceNormalMap(const CameraT &cam, const PICGDeterminablePart &dp,
                         const PIConstraintGraph &cg, const PIGraph<PanoramicCamera> &mg,
                         bool smoothed) {
  auto seg2normal = ComputeSegNormals(dp, cg, mg, smoothed);
  Image3d snm(cam.screenSize());
  for (auto it = snm.begin(); it != snm.end(); ++it) {
    auto p = it.pos();
    auto dir = normalize(cam.toSpace(p));
    auto pp = ToPixel(mg.view.camera.toScreen(dir));
    pp.x = WrapBetween(pp.x, 0, mg.segs.cols);
    pp.y = BoundBetween(pp.y, 0, mg.segs.rows - 1);
    int seg = mg.segs(pp);
    *it = seg2normal[seg];
  }
  return snm;
}

std::vector<Plane3> ComputeSegPlanes(const PICGDeterminablePart &dp,
                                     const PIConstraintGraph &cg,
                                     const PIGraph<PanoramicCamera> &mg, bool smoothed);

template <class CameraT>
Imaged SurfaceDepthMap(const CameraT &cam, const PICGDeterminablePart &dp,
                       const PIConstraintGraph &cg, const PIGraph<PanoramicCamera> &mg,
                       bool smoothed = true) {
  auto seg2plane = ComputeSegPlanes(dp, cg, mg, true);
  Imaged depths(cam.screenSize(), 0.0);
  double minv = std::numeric_limits<double>::max();
  double maxv = 0.0;
  for (auto it = depths.begin(); it != depths.end(); ++it) {
    auto pos = it.pos();
    int seg = mg.segs(pos);
    if (!mg.seg2control[seg].used) {
      *it = -1;
      continue;
    }
    auto &plane = seg2plane[seg];
    if (plane.normal == Origin()) {
      continue;
    }
    Vec3 dir = normalize(cam.toSpace(pos));
    double depth = norm(Intersection(Ray3(Origin(), dir), plane));
    if (depth < minv) {
      minv = depth;
    }
    if (depth > maxv) {
      maxv = depth;
    }
    *it = depth;
  }
  // fill the holes
  std::vector<double> ordered(depths.begin(), depths.end());
  ordered.erase(std::remove(ordered.begin(), ordered.end(), 0.0),
                ordered.end());
  std::sort(ordered.begin(), ordered.end());
  double validMax =
      std::min(ordered[ordered.size() * 0.99] * 1.1, ordered.back());
  double validMin =
      std::max(ordered[ordered.size() * 0.01] * 0.9, ordered.front());
  FillHoles(depths, ((depths < validMin) | (depths > validMax)) & depths != -1);

  return depths;
}
}
}