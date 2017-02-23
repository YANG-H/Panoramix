#pragma once

#include "basic_types.hpp"

namespace pano {
namespace core {

class PerspectiveCamera;
class PanoramicCamera;

// find 3 orthogonal directions
Failable<std::vector<Vec3>> FindOrthogonalPrinicipleDirections(
    const std::vector<Vec3> &directions, int longitudeDivideNum = 1000,
    int latitudeDivideNum = 500, bool allowMoreThan2HorizontalVPs = false,
    const Vec3 &verticalSeed = Vec3(0, 0, 1));

int NearestDirectionId(const std::vector<Vec3> &directions,
                       const Vec3 &verticalSeed = Vec3(0, 0, 1));

// estimate vanishing points
std::vector<Vec3> EstimateVanishingPointsAndClassifyLines(
    const PerspectiveCamera &cams, std::vector<Classified<Line2>> &lineSegments,
    DenseMatd *lineVPScores = nullptr);
std::vector<Vec3> EstimateVanishingPointsAndClassifyLines(
    const std::vector<PerspectiveCamera> &cams,
    std::vector<std::vector<Classified<Line2>>> &lineSegments,
    std::vector<DenseMatd> *lineVPScores = nullptr);

std::vector<Vec3> EstimateVanishingPointsAndClassifyLines(
    std::vector<Classified<Line3>> &lines, DenseMatd *lineVPScores = nullptr,
    bool dontClassifyUmbiguiousLines = false);

// [vert, horiz1, horiz2, other]
std::vector<int> OrderVanishingPoints(std::vector<Vec3> &vps,
                                      const Vec3 &verticalSeed = Z());

namespace {
template <class T>
inline int AssignNewClass(const std::vector<int> &new2old,
                          std::vector<Classified<T>> &cs) {
  for (auto &c : cs) {
    for (int i = 0; i < new2old.size(); i++) {
      if (new2old[i] == c.claz) {
        c.claz = i;
        break;
      }
    }
  }
  return 0;
}
}
// [vert, horiz1, horiz2, other]
template <class T, class... Ts>
std::vector<int> OrderVanishingPoints(std::vector<Vec3> &vps,
                                      const Vec3 &verticalSeed,
                                      std::vector<Classified<T>> &first,
                                      std::vector<Classified<Ts>> &... others) {
  auto new2old = OrderVanishingPoints(vps, verticalSeed);
  int dummy[] = {AssignNewClass(new2old, first),
                 AssignNewClass(new2old, others)...};
  return new2old;
}

// compute pp and focal from 3 orthogonal vps
std::pair<Point2, double>
ComputePrinciplePointAndFocalLength(const Point2 &vp1, const Point2 &vp2,
                                    const Point2 &vp3);

// ComputePrinciplePointAndFocalLengthCandidates
std::vector<Scored<std::pair<Point2, double>>>
ComputePrinciplePointAndFocalLengthCandidates(
    const std::vector<std::vector<Line2>> &lineGroups);

// 2d vanishing point detection
class VanishingPointsDetector {
public:
  enum Algorithm { Naive, TardifSimplified, MATLAB_PanoContext };
  struct Params {
    inline Params(Algorithm algo = Naive, double maxPPOffsetRatio = 2.0,
                  double minFocalRatio = 0.05, double maxFocalRatio = 20.0)
        : maxPrinciplePointOffsetRatio(maxPPOffsetRatio),
          minFocalLengthRatio(minFocalRatio),
          maxFocalLengthRatio(maxFocalRatio), algorithm(algo) {}

    double maxPrinciplePointOffsetRatio;
    double minFocalLengthRatio, maxFocalLengthRatio;
    Algorithm algorithm;
    template <class Archive> inline void serialize(Archive &ar) {
      ar(maxPrinciplePointOffsetRatio, minFocalLengthRatio, maxFocalLengthRatio,
         algorithm);
    }
  };

public:
  inline explicit VanishingPointsDetector(const Params &params = Params())
      : _params(params) {}
  const Params &params() const { return _params; }
  Params &params() { return _params; }

  // accepts (lines, projection center)
  // returns (>= 3 vanishing points (the first 3 vanishing points should be the
  // Manhattan VPs), the focal length, line classes)
  Failable<std::tuple<std::vector<HPoint2>, double, std::vector<int>>>
  operator()(const std::vector<Line2> &lines, const Sizei &imSize) const;
  Failable<std::tuple<std::vector<HPoint2>, double>>
  operator()(std::vector<Classified<Line2>> &lines, const Sizei &imSize) const;

  template <class Archive> inline void serialize(Archive &ar) { ar(_params); }

private:
  Params _params;
};

/// homography estimation
std::pair<Failable<double>, Failable<double>>
ComputeFocalsFromHomography(const Mat3 &H);

// ComputeOrientationMaps
Imagei ComputeOrientationMaps(const std::vector<Classified<Line2>> &lines,
                              const std::vector<HPoint2> &vps,
                              const Sizei &imSize);

// ConvertToImage3d
Image3d ConvertToImage3d(const Image5d &gc);
}
}
