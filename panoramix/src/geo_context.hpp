#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>

#include "matlab_api.hpp"

namespace pano {
namespace core {

/// geometric context estimator
Image7d ComputeRawIndoorGeometricContextHedau(misc::Matlab &matlab,
                                              const Image &im);

// GeometricContextIndex
enum class GeometricContextIndex : size_t {
  FloorOrGround = 0,
  CeilingOrSky = 1,
  Vertical = 2,
  ClutterOrPorous = 3,
  Other = 4
};

// MergeGeometricContextLabelsXXX
Image5d MergeGeometricContextLabelsHoiem(const Image7d &rawgc);
Image5d MergeGeometricContextLabelsHedau(const Image7d &rawgc);

// ComputeGeometricContext
Image5d ComputeIndoorGeometricContextHedau(misc::Matlab &matlab,
                                           const Image &im);

inline GeometricContextIndex MaxGeometricIndex(const Vec5 &gcv) {
  return (GeometricContextIndex)(std::max_element(gcv.val, gcv.val + 5) -
                                 gcv.val);
}

// GeometricContextIndexWithHorizontalOrientations
enum class GeometricContextIndexWithHorizontalOrientations : size_t {
  FloorOrGround = 0,
  CeilingOrSky = 1,
  Vertical1 = 2,
  Vertical2 = 3,
  ClutterOrPorous = 4,
  Other = 5
};

// MergeGeometricContextLabelsXXX
Image6d MergeGeometricContextLabelsHedau(const Image7d &rawgc,
                                         const Vec3 &forward, const Vec3 &hvp1);
Image6d MergeGeometricContextLabelsHoiem(const Image7d &rawgc,
                                         const Vec3 &forward, const Vec3 &hvp1);

// ComputeGeometricContext
Image6d ComputeIndoorGeometricContextHedau(misc::Matlab &matlab,
                                           const Image &im, const Vec3 &forward,
                                           const Vec3 &hvp1);
}
}
