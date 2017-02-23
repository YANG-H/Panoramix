#include "pch.hpp"


//#include <quickshift_common.h>

#include <VPCluster.h>
#include <VPSample.h>

#include "MSAC.h"

#include "cameras.hpp"
#include "containers.hpp"
#include "geo_context.hpp"
#include "utility.hpp"

#include "clock.hpp"
#include "eigen.hpp"
#include "matlab_api.hpp"

namespace pano {
namespace core {

Image_<Vec<double, 7>>
ComputeRawIndoorGeometricContextHedau(misc::Matlab &matlab, const Image &im) {
  matlab << "clear;";
  matlab.setVar("im", im);

  matlab << "[~, ~, slabelConfMap] = gc(im);";

  Image_<Vec<double, 7>> gc = matlab.var("slabelConfMap");
  assert(gc.channels() == 7);
  assert(gc.size() == im.size());
  return gc;
}

Image5d MergeGeometricContextLabelsHoiem(const Image7d &rawgc) {
  Image5d result(rawgc.size(), Vec<double, 5>());
  for (auto it = result.begin(); it != result.end(); ++it) {
    auto &p = rawgc(it.pos());
    auto &resultv = *it;
    // 0: ground, 1,2,3: vertical, 4:clutter, 5:poros, 6: sky
    resultv[ToUnderlying(GeometricContextIndex::FloorOrGround)] += p[0];
    resultv[ToUnderlying(GeometricContextIndex::ClutterOrPorous)] +=
        (p[4] + p[5]);
    resultv[ToUnderlying(GeometricContextIndex::Vertical)] +=
        (p[1] + p[2] + p[3]);
    resultv[ToUnderlying(GeometricContextIndex::CeilingOrSky)] += p[6];
    resultv[ToUnderlying(GeometricContextIndex::Other)] += 0.0;
    assert(IsFuzzyZero(
        std::accumulate(std::begin(resultv.val), std::end(resultv.val), 0.0) -
            1.0,
        1e-2));
  }
  return result;
}

Image5d MergeGeometricContextLabelsHedau(const Image7d &rawgc) {
  Image5d result(rawgc.size(), Vec<double, 5>());
  for (auto it = result.begin(); it != result.end(); ++it) {
    auto &p = rawgc(it.pos());
    auto &resultv = *it;
    // 0: front, 1: left, 2: right, 3: floor, 4: ceiling, 5: clutter, 6: unknown
    resultv[ToUnderlying(GeometricContextIndex::FloorOrGround)] += (p[3]);
    resultv[ToUnderlying(GeometricContextIndex::ClutterOrPorous)] += p[5];
    resultv[ToUnderlying(GeometricContextIndex::Vertical)] +=
        (p[0] + p[1] + p[2]);
    resultv[ToUnderlying(GeometricContextIndex::CeilingOrSky)] += p[4];
    resultv[ToUnderlying(GeometricContextIndex::Other)] += p[6];
    assert(IsFuzzyZero(
        std::accumulate(std::begin(resultv.val), std::end(resultv.val), 0.0) -
            1.0,
        1e-2));
  }
  return result;
}

Image5d ComputeIndoorGeometricContextHedau(misc::Matlab &matlab,
                                           const Image &im) {
  auto rawgc = ComputeRawIndoorGeometricContextHedau(matlab, im);
  return MergeGeometricContextLabelsHedau(rawgc);
}

Image6d MergeGeometricContextLabelsHedau(const Image7d &rawgc,
                                         const Vec3 &forward,
                                         const Vec3 &hvp1) {
  Image6d result(rawgc.size(), Vec<double, 6>());
  double angle = AngleBetweenUndirected(forward, hvp1);
  for (auto it = result.begin(); it != result.end(); ++it) {
    auto &p = rawgc(it.pos());
    auto &resultv = *it;
    // 0: front, 1: left, 2: right, 3: floor, 4: ceiling, 5: clutter, 6: unknown
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::FloorOrGround)] +=
        (p[3]);
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::ClutterOrPorous)] +=
        p[5];

    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Vertical1)] +=
        p[0] * Gaussian(angle, DegreesToRadians(20));
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Vertical2)] +=
        (p[1] + p[2]) * Gaussian(angle - M_PI_2, DegreesToRadians(20));

    // resultv[ToUnderlying(GeometricContextIndex::Vertical)] += (p[0] + p[1] +
    // p[2]);
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::CeilingOrSky)] += p[4];
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Other)] += p[6];
    // assert(IsFuzzyZero(std::accumulate(std::begin(resultv.val),
    // std::end(resultv.val), 0.0) - 1.0, 1e-2));
  }
  return result;
}

Image6d MergeGeometricContextLabelsHoiem(const Image7d &rawgc,
                                         const Vec3 &forward,
                                         const Vec3 &hvp1) {
  Image6d result(rawgc.size(), Vec<double, 6>());
  double angle = AngleBetweenUndirected(forward, hvp1);
  for (auto it = result.begin(); it != result.end(); ++it) {
    auto &p = rawgc(it.pos());
    auto &resultv = *it;
    // 0: ground, 1,2,3: vertical, 4:clutter, 5:poros, 6: sky
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::FloorOrGround)] +=
        p[0];
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::ClutterOrPorous)] +=
        (p[4] + p[5]);

    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Vertical1)] +=
        p[2] * (1.0 - angle / M_PI_2);
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Vertical2)] +=
        (p[1] + p[3]) * angle / M_PI_2;

    // resultv[ToUnderlying(GeometricContextIndexWithHorizontalOrientations::Vertical)]
    // += (p[1] + p[2] + p[3]);
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::CeilingOrSky)] += p[6];
    resultv[ToUnderlying(
        GeometricContextIndexWithHorizontalOrientations::Other)] += 0.0;
    // assert(IsFuzzyZero(std::accumulate(std::begin(resultv.val),
    // std::end(resultv.val), 0.0) - 1.0, 1e-2));
  }
  return result;
}

Image6d ComputeIndoorGeometricContextHedau(misc::Matlab &matlab, const Image &im,
                                const Vec3 &forward, const Vec3 &hvp1) {
  auto rawgc = ComputeRawIndoorGeometricContextHedau(matlab, im);
  return MergeGeometricContextLabelsHedau(rawgc, forward, hvp1);
}

Image3d ConvertToImage3d(const Image5d &gc) {
  Image3d vv(gc.size());
  std::vector<Vec3> colors = {Vec3(0, 0, 1), Vec3(0, 1, 0), Vec3(1, 0, 0),
                              normalize(Vec3(1, 1, 1))};
  for (auto it = gc.begin(); it != gc.end(); ++it) {
    const Vec5 &v = *it;
    Vec3 color;
    for (int i = 0; i < 4; i++) {
      color += colors[i] * v[i];
    }
    vv(it.pos()) = color;
  }
  return vv;
}

std::vector<Scored<Chain2>> DetectOcclusionBoundary(misc::Matlab &matlab,
                                                    const Image &im) {
  matlab << "clear";
  matlab << "oldpath = cd('F:\\MATLAB\\miaomiaoliu.occ\\occ');";
  matlab.setVar("im", im);
  matlab << "[bndinfo, score] = detect_occlusion_yh(im);";
  matlab << "inds = bndinfo.edges.indices;";
  matlab << "cd(oldpath);";
  auto inds = matlab.var("inds");
  auto score = matlab.var("score");
  assert(inds.isCell());
  int ninds = inds.length();
  assert(score.length() == ninds);
  std::vector<Scored<Chain2>> bd;
  bd.reserve(ninds);
  for (int i = 0; i < ninds; i++) {
    auto chain = inds.cell(i);
    assert(chain.is<uint32_t>());
    double s = score.at(i);
    Chain2 b;
    b.closed = false;
    b.points.reserve(chain.length());
    for (int j = 0; j < chain.length(); j++) {
      int index = chain.at<uint32_t>(j);
      --index;
      int y = index % im.rows;
      int x = index / im.rows;
      if (!b.empty()) {
        double d = Distance(b.points.back(), Point2(x, y));
        assert(d < 50);
      }
      b.append(Point2(x, y));
    }
    bd.push_back(ScoreAs(std::move(b), s));
  }
  return bd;
}
}
}
