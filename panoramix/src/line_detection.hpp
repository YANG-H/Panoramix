#pragma once

#include "basic_types.hpp"

namespace pano {
namespace core {

// line extractor
class LineSegmentExtractor {
public:
  using Feature = std::vector<Line2>;
  enum Algorithm { GradientGrouping, LSD };
  struct Params {
    inline Params()
        : minLength(15), xBorderWidth(1), yBorderWidth(1), numDirs(8),
          algorithm(LSD) {}
    int minLength;
    int xBorderWidth, yBorderWidth;
    int numDirs;
    Algorithm algorithm;
    template <class Archive> inline void serialize(Archive &ar) {
      ar(minLength, xBorderWidth, yBorderWidth, numDirs, algorithm);
    }
  };

public:
  inline explicit LineSegmentExtractor(const Params &params = Params())
      : _params(params) {}
  const Params &params() const { return _params; }
  Params &params() { return _params; }
  Feature operator()(const Image &im) const;
  Feature operator()(const Image &im, int pyramidHeight,
                     int minSize = 100) const;
  template <class Archive> inline void serialize(Archive &ar) { ar(_params); }

private:
  Params _params;
};

// compute line intersections
std::vector<HPoint2> ComputeLineIntersections(
    const std::vector<Line2> &lines,
    std::vector<std::pair<int, int>> *lineids = nullptr,
    bool suppresscross = true,
    double minDistanceBetweenLinePairs = std::numeric_limits<double>::max());

std::vector<Vec3>
ComputeLineIntersections(const std::vector<Line3> &lines,
                         std::vector<std::pair<int, int>> *lineids = nullptr,
                         double minAngleDistanceBetweenLinePairs = M_PI);

// classify lines in 2d
DenseMatd ClassifyLines(std::vector<Classified<Line2>> &lines,
                        const std::vector<HPoint2> &vps,
                        double angleThreshold = M_PI / 3.0, double sigma = 0.1,
                        double scoreThreshold = 0.8,
                        double avoidVPDistanceThreshold = -1.0);

// classify lines in 3d
DenseMatd ClassifyLines(std::vector<Classified<Line3>> &lines,
                        const std::vector<Vec3> &vps, double angleThreshold,
                        double sigma, double scoreThreshold = 0.8,
                        double avoidVPAngleThreshold = M_PI / 18.0,
                        double scoreAdvatangeRatio = 0.0);

// MergeLines
std::vector<Line3> MergeLines(const std::vector<Line3> &lines,
                              double angleThres = 0.03,
                              double mergeAngleThres = 0.0);

// compute straightness of points
std::pair<double, Ray2>
ComputeStraightness(const std::vector<std::vector<Pixel>> &edges,
                    double *interArea = nullptr, double *interLen = nullptr);
}
}
