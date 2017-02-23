#include "pch.hpp"

#include <VPCluster.h>
#include <VPSample.h>

#include "MSAC.h"

#include "cameras.hpp"
#include "containers.hpp"
#include "manhattan.hpp"
#include "utility.hpp"

#include "clock.hpp"
#include "eigen.hpp"
#include "matlab_api.hpp"

namespace pano {
namespace core {

namespace {

inline double LatitudeFromLongitudeAndNormalVector(double longitude,
                                                   const Vec3 &normal) {
  // normal(0)*cos(long)*cos(la) + normal(1)*sin(long)*cos(lat) +
  // normal(2)*sin(la) = 0
  // normal(0)*cos(long) + normal(1)*sin(long) + normal(2)*tan(la) = 0
  return -atan((normal(0) * cos(longitude) + normal(1) * sin(longitude)) /
               normal(2));
}

inline double Longitude1FromLatitudeAndNormalVector(double latitude,
                                                    const Vec3 &normal) {
  double a = normal(1) * cos(latitude);
  double b = normal(0) * cos(latitude);
  double c = -normal(2) * sin(latitude);
  double sinLong =
      (a * c + sqrt(Square(a * c) -
                    (Square(a) + Square(b)) * (Square(c) - Square(b)))) /
      (Square(a) + Square(b));
  return asin(sinLong);
}

inline double Longitude2FromLatitudeAndNormalVector(double latitude,
                                                    const Vec3 &normal) {
  double a = normal(1) * cos(latitude);
  double b = normal(0) * cos(latitude);
  double c = -normal(2) * sin(latitude);
  double sinLong =
      (a * c - sqrt(Square(a * c) -
                    (Square(a) + Square(b)) * (Square(c) - Square(b)))) /
      (Square(a) + Square(b));
  return asin(sinLong);
}

inline double UnOrthogonality(const Vec3 &v1, const Vec3 &v2, const Vec3 &v3) {
  return norm(Vec3(v1.dot(v2), v2.dot(v3), v3.dot(v1)));
}
}

Failable<std::vector<Vec3>> FindOrthogonalPrinicipleDirections(
    const std::vector<Vec3> &intersections, int longitudeDivideNum,
    int latitudeDivideNum, bool allowMoreThan2HorizontalVPs,
    const Vec3 &verticalSeed) {

  std::vector<Vec3> vps(3);

  // collect votes of intersection directions
  Imagef votePanel = Imagef::zeros(longitudeDivideNum, latitudeDivideNum);
  for (const Vec3 &p : intersections) {
    Pixel pixel =
        PixelFromGeoCoord(GeoCoord(p), longitudeDivideNum, latitudeDivideNum);
    votePanel(pixel.x, pixel.y) += 1.0;
  }
  cv::GaussianBlur(votePanel, votePanel,
                   cv::Size((longitudeDivideNum / 50) * 2 + 1,
                            (latitudeDivideNum / 50) * 2 + 1),
                   4, 4, cv::BORDER_REPLICATE);

  // set the direction with the max votes as the first vanishing point
  double minVal = 0, maxVal = 0;
  int maxIndex[] = {-1, -1};
  cv::minMaxIdx(votePanel, &minVal, &maxVal, 0, maxIndex);
  cv::Point maxPixel(maxIndex[0], maxIndex[1]);

  vps[0] = GeoCoordFromPixel(maxPixel, longitudeDivideNum, latitudeDivideNum)
               .toVector();
  const Vec3 &vec0 = vps[0];

  // iterate locations orthogonal to vps[0]
  double maxScore = -1;
  for (int x = 0; x < longitudeDivideNum; x++) {
    double longt1 = double(x) / longitudeDivideNum * M_PI * 2 - M_PI;
    double lat1 = LatitudeFromLongitudeAndNormalVector(longt1, vec0);
    Vec3 vec1 = GeoCoord(longt1, lat1).toVector();
    Vec3 vec1rev = -vec1;
    Vec3 vec2 = vec0.cross(vec1);
    Vec3 vec2rev = -vec2;

    double score = 0;
    for (const Vec3 &v : {vec1, vec1rev, vec2, vec2rev}) {
      Pixel pixel =
          PixelFromGeoCoord(GeoCoord(v), longitudeDivideNum, latitudeDivideNum);
      score += votePanel(WrapBetween(pixel.x, 0, longitudeDivideNum),
                         WrapBetween(pixel.y, 0, latitudeDivideNum));
    }
    if (score > maxScore) {
      maxScore = score;
      vps[1] = vec1;
      vps[2] = vec2;
    }
  }

  if (UnOrthogonality(vps[0], vps[1], vps[2]) >= 0.1) {
    // failed, then use y instead of x
    maxScore = -1;
    for (int y = 0; y < latitudeDivideNum; y++) {
      double lat1 = double(y) / latitudeDivideNum * M_PI - M_PI_2;
      double longt1s[] = {Longitude1FromLatitudeAndNormalVector(lat1, vec0),
                          Longitude2FromLatitudeAndNormalVector(lat1, vec0)};
      for (double longt1 : longt1s) {
        Vec3 vec1 = GeoCoord(longt1, lat1).toVector();
        Vec3 vec1rev = -vec1;
        Vec3 vec2 = vec0.cross(vec1);
        Vec3 vec2rev = -vec2;
        Vec3 vecs[] = {vec1, vec1rev, vec2, vec2rev};

        double score = 0;
        for (Vec3 &v : vecs) {
          Pixel pixel = PixelFromGeoCoord(GeoCoord(v), longitudeDivideNum,
                                          latitudeDivideNum);
          score += votePanel(WrapBetween(pixel.x, 0, longitudeDivideNum),
                             WrapBetween(pixel.y, 0, latitudeDivideNum));
        }
        if (score > maxScore) {
          maxScore = score;
          vps[1] = vec1;
          vps[2] = vec2;
        }
      }
    }
  }

  if (UnOrthogonality(vps[0], vps[1], vps[2]) >= 0.1) {
    return nullptr; // failed
  }

  // make vps[0] the vertical vp
  {
    int vertVPId = -1;
    double minAngle = std::numeric_limits<double>::max();
    for (int i = 0; i < vps.size(); i++) {
      double a = AngleBetweenUndirected(vps[i], verticalSeed);
      if (a < minAngle) {
        vertVPId = i;
        minAngle = a;
      }
    }
    std::swap(vps[0], vps[vertVPId]);
  }

  if (allowMoreThan2HorizontalVPs) {
    // find more horizontal vps
    double nextMaxScore = maxScore * 0.5; // threshold
    double minAngleToCurHorizontalVPs = DegreesToRadians(30);
    for (int x = 0; x < longitudeDivideNum; x++) {
      double longt1 = double(x) / longitudeDivideNum * M_PI * 2 - M_PI;
      double lat1 = LatitudeFromLongitudeAndNormalVector(longt1, vps[0]);
      Vec3 vec1 = GeoCoord(longt1, lat1).toVector();
      Vec3 vec1rev = -vec1;
      Vec3 vec2 = vps[0].cross(vec1);
      Vec3 vec2rev = -vec2;

      double score = 0;
      for (const Vec3 &v : {vec1, vec1rev, vec2, vec2rev}) {
        Pixel pixel = PixelFromGeoCoord(GeoCoord(v), longitudeDivideNum,
                                        latitudeDivideNum);
        score += votePanel(WrapBetween(pixel.x, 0, longitudeDivideNum),
                           WrapBetween(pixel.y, 0, latitudeDivideNum));
      }

      bool tooCloseToExistingVP = false;
      for (int i = 0; i < vps.size(); i++) {
        if (AngleBetweenUndirected(vps[i], vec1) <
            minAngleToCurHorizontalVPs) {
          tooCloseToExistingVP = true;
          break;
        }
      }
      if (tooCloseToExistingVP)
        continue;

      if (score > nextMaxScore) {
        nextMaxScore = score;
        vps.push_back(vec1);
        vps.push_back(vec2);
      }
    }
  }

  return std::move(vps);
}

int NearestDirectionId(const std::vector<Vec3> &directions,
                       const Vec3 &verticalSeed) {
  int vid = -1;
  double minAngle = M_PI;
  for (int i = 0; i < directions.size(); i++) {
    double angle = AngleBetweenUndirected(directions[i], verticalSeed);
    if (angle < minAngle) {
      vid = i;
      minAngle = angle;
    }
  }
  return vid;
}

std::vector<Vec3> EstimateVanishingPointsAndClassifyLines(
    const PerspectiveCamera &cam, std::vector<Classified<Line2>> &lineSegments,
    DenseMatd *lineVPScores) {
  std::vector<Vec3> lineIntersections;

  int linesNum = 0;
  std::vector<Line2> pureLines(lineSegments.size());
  linesNum += lineSegments.size();
  for (int k = 0; k < pureLines.size(); k++) {
    pureLines[k] = lineSegments[k].component;
  }
  auto inters = ComputeLineIntersections(pureLines, nullptr, true,
                                         std::numeric_limits<double>::max());
  // insert line intersections
  for (auto &p : inters) {
    lineIntersections.push_back(normalize(cam.toSpace(p.value())));
  }

  auto vanishingPoints =
      FindOrthogonalPrinicipleDirections(lineIntersections, 1000, 500, true)
          .unwrap();

  // project lines to space
  std::vector<Classified<Line3>> spatialLineSegments;
  spatialLineSegments.reserve(linesNum);
  for (const auto &line : lineSegments) {
    auto &p1 = line.component.first;
    auto &p2 = line.component.second;
    auto pp1 = cam.toSpace(p1);
    auto pp2 = cam.toSpace(p2);
    Classified<Line3> cline3;
    cline3.claz = -1;
    cline3.component = Line3{pp1, pp2};
    spatialLineSegments.push_back(cline3);
  }

  // classify lines
  auto scores = ClassifyLines(spatialLineSegments, vanishingPoints, M_PI / 3.0,
                              0.1, 0.8, M_PI / 18.0);
  if (lineVPScores) {
    *lineVPScores = scores;
  }

  int ii = 0;
  for (int j = 0; j < lineSegments.size(); j++) {
    lineSegments[j].claz = spatialLineSegments[ii].claz;
    ii++;
  }

  return vanishingPoints;
}

std::vector<Vec3> EstimateVanishingPointsAndClassifyLines(
    const std::vector<PerspectiveCamera> &cams,
    std::vector<std::vector<Classified<Line2>>> &lineSegments,
    std::vector<DenseMatd> *lineVPScores) {

  assert(cams.size() == lineSegments.size());
  std::vector<Vec3> lineIntersections;

  int linesNum = 0;
  for (int i = 0; i < cams.size(); i++) {
    std::vector<Line2> pureLines(lineSegments[i].size());
    linesNum += lineSegments[i].size();
    for (int k = 0; k < pureLines.size(); k++) {
      pureLines[k] = lineSegments[i][k].component;
    }
    auto inters = ComputeLineIntersections(pureLines, nullptr, true,
                                           std::numeric_limits<double>::max());
    // insert line intersections
    for (auto &p : inters) {
      lineIntersections.push_back(normalize(cams[i].toSpace(p.value())));
    }
  }

  auto vanishingPoints =
      FindOrthogonalPrinicipleDirections(lineIntersections, 1000, 500, true)
          .unwrap();

  // project lines to space
  std::vector<Classified<Line3>> spatialLineSegments;
  spatialLineSegments.reserve(linesNum);
  for (int i = 0; i < cams.size(); i++) {
    for (const auto &line : lineSegments[i]) {
      auto &p1 = line.component.first;
      auto &p2 = line.component.second;
      auto pp1 = cams[i].toSpace(p1);
      auto pp2 = cams[i].toSpace(p2);
      Classified<Line3> cline3;
      cline3.claz = -1;
      cline3.component = Line3{pp1, pp2};
      spatialLineSegments.push_back(cline3);
    }
  }

  // classify lines
  auto scores = ClassifyLines(spatialLineSegments, vanishingPoints, M_PI / 3.0,
                              0.1, 0.8, M_PI / 18.0);
  assert(scores.rows == spatialLineSegments.size() &&
         scores.cols == vanishingPoints.size());
  if (lineVPScores) {
    auto &s = *lineVPScores;
    s.resize(cams.size());
    int ii = 0;
    for (int i = 0; i < cams.size(); i++) {
      scores(cv::Range(ii, ii + lineSegments[i].size()), cv::Range::all())
          .copyTo(s[i]);
      ii += lineSegments[i].size();
    }
  }

  int ii = 0;
  for (int i = 0; i < lineSegments.size(); i++) {
    for (int j = 0; j < lineSegments[i].size(); j++) {
      lineSegments[i][j].claz = spatialLineSegments[ii].claz;
      ii++;
    }
  }

  return vanishingPoints;
}

std::vector<Vec3>
EstimateVanishingPointsAndClassifyLines(std::vector<Classified<Line3>> &lines,
                                        DenseMatd *lineVPScores,
                                        bool dontClassifyUmbiguiousLines) {
  std::vector<Vec3> lineIntersections;

  std::vector<Line3> pureLines(lines.size());
  for (int i = 0; i < lines.size(); i++) {
    pureLines[i] = lines[i].component;
  }
  auto inters = ComputeLineIntersections(pureLines, nullptr);

  auto vanishingPoints =
      FindOrthogonalPrinicipleDirections(inters, 1000, 500, true).unwrap();
  OrderVanishingPoints(vanishingPoints);

  auto scores =
      ClassifyLines(lines, vanishingPoints, M_PI / 3.0, 0.1, 0.8, M_PI / 18.0,
                    dontClassifyUmbiguiousLines ? 0.1 : 0.0);
  assert(scores.rows == lines.size() && scores.cols == vanishingPoints.size());

  if (lineVPScores) {
    *lineVPScores = std::move(scores);
  }

  return vanishingPoints;
}

std::vector<int> OrderVanishingPoints(std::vector<Vec3> &vps,
                                      const Vec3 &verticalSeed /*= Z()*/) {
  assert(vps.size() >= 1);
  std::vector<int> new2old(vps.size());
  std::iota(new2old.begin(), new2old.end(), 0);
  int vertId = NearestDirectionId(vps, verticalSeed);
  std::swap(vps[0], vps[vertId]);
  std::swap(new2old[0], new2old[vertId]);
  for (int i = 1; i < vps.size(); i++) {
    if (IsFuzzyPerpendicular(vps[0], vps[i])) {
      std::swap(vps[1], vps[i]);
      std::swap(new2old[1], new2old[i]);
      break;
    }
  }
  for (int i = 2; i < vps.size(); i++) {
    if (IsFuzzyPerpendicular(vps[0], vps[i]) &&
        IsFuzzyPerpendicular(vps[1], vps[i])) {
      std::swap(vps[2], vps[i]);
      std::swap(new2old[2], new2old[i]);
      break;
    }
  }
  return new2old;
}

#pragma region VanishingPointsDetector

namespace {

template <class T>
inline Vec<T, 3> PerpendicularRootOfLineEquation(const Vec<T, 3> &lineeq) {
  auto &a = lineeq[0];
  auto &b = lineeq[1];
  auto &c = lineeq[2];
  return Vec<T, 3>(-a * c, -b * c, a * a + b * b);
}
}

std::pair<Point2, double>
ComputePrinciplePointAndFocalLength(const Point2 &vp1, const Point2 &vp2,
                                    const Point2 &vp3) {
  auto lambda =
      (vp1 - vp3).dot(vp2 - vp3) / ((vp1(0) - vp2(0)) * (vp1(1) - vp3(1)) -
                                    (vp1(0) - vp3(0)) * (vp1(1) - vp2(1)));
  Point2 pp = vp3 + PerpendicularDirection(vp1 - vp2) * lambda;
  double focalLength = sqrt(abs(-(vp1 - pp).dot(vp2 - pp)));
  return std::make_pair(pp, focalLength);
}

std::vector<Scored<std::pair<Point2, double>>>
ComputePrinciplePointAndFocalLengthCandidates(
    const std::vector<std::vector<Line2>> &lineGroups) {
  //// [Estimate PP & Focal Candidates from 2D Line Groups]
  Box2 box;
  for (auto &g : lineGroups) {
    box |= BoundingBoxOfContainer(g);
  }
  double scale = box.outerSphere().radius;

  std::vector<Scored<std::pair<Point2, double>>> ppFocalCandidates;
  ppFocalCandidates.reserve(lineGroups.size() * 3);

  for (auto & group : lineGroups) {
    // collect edge intersections in each face
    std::vector<Point2> interps;

  }

  // 
  NOT_IMPLEMENTED_YET();
}



namespace {

Imaged LinesVotesToPoints(const std::vector<HPoint2> &points,
                          const std::vector<Line2> &lines) {

  SetClock();

  static const double angleThreshold = M_PI / 3.0;
  static const double sigma = 0.1;

  size_t nlines = lines.size();
  size_t npoints = points.size();
  Imaged votes = Imaged::zeros(nlines, npoints);
  for (auto it = votes.begin(); it != votes.end(); ++it) {
    auto &line = lines[it.pos().y];
    const HPoint2 &point = points[it.pos().x];
    Vec2 mid2vp = (point - HPoint2(line.center())).value();
    // project point on line
    double proj = mid2vp.dot(normalize(line.direction()));
    bool liesOnLine = abs(proj) <= line.length() / 2.0;
    double angle = AngleBetweenUndirected(mid2vp, line.direction());

    double score = 0.0;
    if (angle >= angleThreshold)
      continue;
    if (liesOnLine)
      continue;
    score = exp(-Square(angle / angleThreshold) / sigma / sigma / 2.0);
    if (std::isinf(score) || std::isnan(score)) {
      score = 0.0;
    }
    *it = score;
  }
  return votes;
}

std::vector<int> ClassifyLines(const Imaged &votes,
                               double scoreThreshold = 0.5) {

  SetClock();

  std::vector<int> lineClasses(votes.rows, -1);
  int nlines = votes.rows;
  int npoints = votes.cols;
  for (size_t i = 0; i < nlines; i++) {
    // classify lines
    lineClasses[i] = -1;
    double curscore = scoreThreshold;
    for (int j = 0; j < npoints; j++) {
      if (votes(i, j) >= curscore) {
        lineClasses[i] = j;
        curscore = votes(i, j);
      }
    }
  }
  return lineClasses;
}

std::vector<std::pair<Point2, double>>
ComputeProjectionCenterAndFocalLength(const std::vector<Point2> &vp1s,
                                      const std::vector<Point2> &vp2s,
                                      const Point2 &vp3) {
  assert(vp1s.size() == vp2s.size());
  std::vector<std::pair<Point2, double>> ppAndFocals(vp1s.size());

  using namespace Eigen;
  Array<double, Eigen::Dynamic, 2> vp1m, vp2m;
  vp1m.resize(vp1s.size(), 2);
  vp2m.resize(vp2s.size(), 2);

  for (int i = 0; i < vp1s.size(); i++) {
    vp1m(i, 0) = vp1s[i][0];
    vp1m(i, 1) = vp1s[i][1];
    vp2m(i, 0) = vp2s[i][0];
    vp2m(i, 1) = vp2s[i][1];
  }

  Array<double, 1, 2> vp3m;
  vp3m(0, 0) = vp3[0];
  vp3m(0, 1) = vp3[1];

  auto lambdaUppers = (vp1m.rowwise() - vp3m)
                          .cwiseProduct(vp2m.rowwise() - vp3m)
                          .rowwise()
                          .sum();
  auto lambdaLowers =
      (vp1m.col(0) - vp2m.col(0)).cwiseProduct(vp1m.col(1) - vp3m(1)) -
      (vp1m.col(0) - vp3m(0)).cwiseProduct(vp1m.col(1) - vp2m.col(1));
  auto lambdas = lambdaUppers / lambdaLowers;

  Matrix<double, 2, 2> perpendicular;
  perpendicular.setZero();
  perpendicular(0, 1) = 1;
  perpendicular(1, 0) = -1;

  Array<double, Eigen::Dynamic, 2> pps =
      (((vp1m - vp2m).matrix() * perpendicular).array().colwise() * lambdas)
          .rowwise() +
      vp3m;
  auto vp1_pp = vp1m - pps;
  auto vp2_pp = vp2m - pps;
  Array<double, Eigen::Dynamic, 1> focalLengths =
      sqrt(-(vp1_pp.col(0) * vp2_pp.col(0) + vp1_pp.col(1) + vp2_pp.col(1)));

  for (int i = 0; i < vp1s.size(); i++) {
    ppAndFocals[i].first = {pps(i, 0), pps(i, 1)};
    ppAndFocals[i].second = focalLengths(i);
  }
  return ppAndFocals;
}

// make infinite points finite
// merge close points
void RefineIntersections(
    std::vector<HPoint2> &intersections,
    std::vector<std::pair<int, int>> *intersectionMakerLineIds = nullptr,
    double distanceThreshold = 1.0) {

  SetClock();

  static const bool useVotes = false;

  if (useVotes) {

    THERE_ARE_BUGS_HERE("returns nothing!!!!");

    static const double fakeFocal = 500;
    PanoramicCamera cam(fakeFocal);
    core::Imaged votes = core::Imaged::zeros(cam.screenSize());

    for (auto &inter : intersections) {
      Vec3 dir(inter.numerator[0], inter.numerator[1],
               inter.denominator * fakeFocal);
      for (auto &p : {cam.toScreen(dir), cam.toScreen(-dir)}) {
        int x = p[0], y = p[1];
        if (x < 0)
          x = 0;
        if (x >= votes.cols)
          x = votes.cols - 1;
        if (y < 0)
          y = 0;
        if (y >= votes.rows)
          y = votes.rows - 1;
        votes(y, x) += 1.0;
      }
    }

    cv::GaussianBlur(votes, votes, cv::Size(5, 5), 0.1);
    std::vector<Pixel> points;
    NonMaximaSuppression(votes, votes, 50, &points);
    intersections.clear();
    for (auto &p : points) {
      Vec3 dir = cam.toSpace(p);
      intersections.emplace_back(Point2(dir[0], dir[1]), dir[2] / fakeFocal);
    }

    assert(!intersectionMakerLineIds);
  } else {
    for (auto &hp : intersections) {
      if (hp.denominator == 0.0)
        hp.denominator = 1e-5;
    }

    std::vector<HPoint2> mergedIntersections;
    mergedIntersections.reserve(intersections.size());
    std::vector<std::pair<int, int>> mergedIntersectionMakerLineIds;
    mergedIntersectionMakerLineIds.reserve(intersections.size());

    RTreeWrapper<HPoint2, DefaultInfluenceBoxFunctor<double>> rtreeRecorder(
        DefaultInfluenceBoxFunctor<double>(distanceThreshold * 2.0));
    for (int i = 0; i < intersections.size(); i++) {
      if (rtreeRecorder.contains(
              intersections[i],
              [&distanceThreshold](const HPoint2 &a, const HPoint2 &b) {
                return Distance(a.value(), b.value()) < distanceThreshold;
              })) {
        continue;
      }
      rtreeRecorder.insert(intersections[i]);
      mergedIntersections.push_back(intersections[i]);
      if (intersectionMakerLineIds)
        mergedIntersectionMakerLineIds.push_back(
            (*intersectionMakerLineIds)[i]);
    }

    intersections = std::move(mergedIntersections);
    if (intersectionMakerLineIds)
      *intersectionMakerLineIds = std::move(mergedIntersectionMakerLineIds);
  }
}

std::vector<Vec3>
RefineIntersectionsAndProjectToSpace(const std::vector<HPoint2> &intersections,
                                     double fakeFocal, double angleThres) {
  std::vector<Vec3> dirs;
  dirs.reserve(intersections.size());

  RTreeWrapper<Vec3, DefaultInfluenceBoxFunctor<double>> rtreeRecorder(
      DefaultInfluenceBoxFunctor<double>(angleThres * 2.0));
  for (int i = 0; i < intersections.size(); i++) {
    Vec3 inter = normalize(VectorFromHPoint(intersections[i], fakeFocal));
    if (rtreeRecorder.contains(
            inter, [&angleThres](const Vec3 &a, const Vec3 &b) {
              return AngleBetweenDirected(a, b) < angleThres;
            })) {
      continue;
    }
    rtreeRecorder.insert(inter);
    dirs.push_back(inter);
  }
  return dirs;
}

Imaged GetLineLengthRatios(const std::vector<Line2> &lines) {
  // get max line length
  double maxLineLen = 0;
  for (auto &line : lines) {
    if (line.length() > maxLineLen)
      maxLineLen = line.length();
  }
  Imaged lineLengthRatios(lines.size(), 1);
  for (int i = 0; i < lines.size(); i++) {
    lineLengthRatios(i) = lines[i].length() / maxLineLen;
  }
  return lineLengthRatios;
}

inline HPoint2 ProjectOnToImagePlane(const Vec3 &d, const Point2 &pp,
                                     double focal) {
  return HPointFromVector(d, focal) + HPoint2(pp);
}

int AppendTheBestNPPAndFocalData(
    const std::vector<Point2> &vp2cands,
    const std::vector<int> &vp2candIdInRemainedIntersections,
    const std::vector<Point2> &vp3cands,
    const std::vector<int> &vp3candIdInRemainedIntersections,
    const Point2 &vp1p, int vp1id, const Imaged &votesPanel,
    const Imaged &votesRemainedPanel, const std::vector<Line2> &lines,
    double maxPrinciplePointOffset, double minFocalLength,
    double maxFocalLength, std::vector<std::pair<Point2, double>> &ppAndFocals,
    std::vector<float> &scores, int N) {

  auto ppAndFocalsThisTime =
      ComputeProjectionCenterAndFocalLength(vp2cands, vp3cands, vp1p);

  // compute scores
  std::cout << ".";
  std::vector<float> scoresThisTime(ppAndFocalsThisTime.size(), 0);

  for (int i = 0; i < ppAndFocalsThisTime.size(); i++) {
    int vp2candId = vp2candIdInRemainedIntersections[i];
    int vp3candId = vp3candIdInRemainedIntersections[i];

    auto &principlePoint = ppAndFocalsThisTime[i].first;
    auto &focalLength = ppAndFocalsThisTime[i].second;
    if (std::isnan(focalLength) || std::isinf(focalLength)) {
      continue;
    }
    if (!(norm(principlePoint) < maxPrinciplePointOffset &&
          IsBetween(focalLength, minFocalLength, maxFocalLength))) {
      continue;
    }
    // get votes from each line
    float score = 0.0;
    for (int lineId = 0; lineId < lines.size(); lineId++) {
      double voteForVP1 = votesPanel(lineId, vp1id);
      double voteForVP2 = votesRemainedPanel(lineId, vp2candId);
      double voteForVP3 = votesRemainedPanel(lineId, vp3candId);
      score += std::max({voteForVP1, voteForVP2, voteForVP3});
    }
    scoresThisTime[i] = score;
  }

  // sort
  std::vector<int> sortedPPAndFocalIds(scoresThisTime.size());
  for (int i = 0; i < scoresThisTime.size(); i++)
    sortedPPAndFocalIds[i] = i;

  std::sort(sortedPPAndFocalIds.begin(), sortedPPAndFocalIds.end(),
            [&scoresThisTime](int a, int b) {
              return scoresThisTime[a] > scoresThisTime[b];
            });

  int keptSize = std::min<size_t>(N, ppAndFocalsThisTime.size());
  scores.reserve(scores.size() + scoresThisTime.size());
  ppAndFocals.reserve(ppAndFocals.size() + ppAndFocalsThisTime.size());

  for (int i = 0; i < keptSize; i++) {
    scores.push_back(scoresThisTime[sortedPPAndFocalIds[i]]);
    ppAndFocals.push_back(ppAndFocalsThisTime[sortedPPAndFocalIds[i]]);
  }
  return scoresThisTime.size();
}

std::tuple<std::vector<HPoint2>, double, std::vector<int>>
EstimateVanishingPointsWithProjectionCenterAtOrigin(
    const std::vector<Line2> &lines, double minFocalLength,
    double maxFocalLength, double maxPrinciplePointOffset, bool &succeed) {

  SetClock();

  // get max line length
  auto lineLengthRatios = GetLineLengthRatios(lines);

  std::vector<int> lineClasses(lines.size(), -1);

  // get all intersections
  std::vector<std::pair<int, int>> intersectionMakerLineIds;
  auto intersections =
      ComputeLineIntersections(lines, &intersectionMakerLineIds, true,
                               std::numeric_limits<double>::max());

  std::cout << "intersection num: " << intersections.size() << std::endl;
  RefineIntersections(intersections, &intersectionMakerLineIds);
  std::cout << "intersection num: " << intersections.size() << std::endl;

  // nlines x npoints (without consideration of line length ratios)
  Imaged votesPanel = LinesVotesToPoints(intersections, lines);

  // vote all lines for all intersections (with consideration of line length
  // ratios)
  std::vector<double> votesForIntersections(intersections.size(), 0.0);
  for (int i = 0; i < intersections.size(); i++) {
    votesForIntersections[i] = votesPanel.col(i).dot(lineLengthRatios);
  }

  // get vp1
  auto intersectionIdWithMaxVotes =
      std::distance(votesForIntersections.begin(),
                    std::max_element(votesForIntersections.begin(),
                                     votesForIntersections.end()));
  const HPoint2 &vp1 = intersections[intersectionIdWithMaxVotes];
  std::cout << "vp1: " << vp1.value() << std::endl;
  std::cout << "score: " << votesForIntersections[intersectionIdWithMaxVotes]
            << std::endl;

  // classify lines for vp1 to be 0, and collect remained lines
  std::vector<Line2> remainedLines;
  remainedLines.reserve(lines.size() / 2);
  for (int i = 0; i < lines.size(); i++) {
    if (votesPanel(i, intersectionIdWithMaxVotes) > 0.8) {
      lineClasses[i] = 0;
    } else {
      remainedLines.push_back(lines[i]);
    }
  }

  if (remainedLines.empty()) {
    std::cout << "no remained lines to locate other two vps, failed"
              << std::endl;
    succeed = false;
    return std::tuple<std::vector<HPoint2>, double, std::vector<int>>();
  }

  // std::cout << "remained lines num: " << remainedLines.size() << std::endl;

  // get remained intersections
  std::vector<std::pair<int, int>> remainedIntersectionMakerLineIds;
  auto remainedIntersections =
      ComputeLineIntersections(remainedLines, &remainedIntersectionMakerLineIds,
                               true, std::numeric_limits<double>::max());

  // std::cout << "remained intersection num: " << remainedLines.size() <<
  // std::endl;
  RefineIntersections(remainedIntersections, &remainedIntersectionMakerLineIds);
  // std::cout << "remained intersection num: " << remainedLines.size() <<
  // std::endl;

  // vote remained lines for remained intersections
  std::vector<double> votesForRemainedIntersections(
      remainedIntersections.size(), 0.0);
  // remainedlines x remainedpoints
  Imaged votesRemainedPanel = LinesVotesToPoints(
      remainedIntersections, lines); // all lines participated!!!!!
  for (int i = 0; i < remainedIntersections.size(); i++) {
    votesForRemainedIntersections
        [i] = // cv::sum(votesRemainedPanel.col(i)).val[0];
        votesRemainedPanel.col(i).dot(lineLengthRatios);
  }
  // sort remained intersections by votes
  std::vector<int> orderedRemainedIntersectionIDs(remainedIntersections.size());
  std::iota(orderedRemainedIntersectionIDs.begin(),
            orderedRemainedIntersectionIDs.end(), 0);
  std::sort(orderedRemainedIntersectionIDs.begin(),
            orderedRemainedIntersectionIDs.end(),
            [&votesForRemainedIntersections](int id1, int id2) {
              return votesForRemainedIntersections[id1] >
                     votesForRemainedIntersections[id2];
            });

  //// traverse other vp pairs
  double curMaxScore = 0.0;
  HPoint2 vp2, vp3;
  double curFocal;
  Point2 curPrinciplePoint;

  Point2 vp1ccenter = vp1.value() / 2.0;
  double vp1cdist = norm(vp1).value();

  int maxNum = 500000;
  int count = 0;

  for (int k = 0; k < orderedRemainedIntersectionIDs.size() * 2; k++) {

    // Clock clock(std::to_string(k) + "-th guessing of vp triplets");

    if (count >= maxNum)
      break;

    int lb = std::max(0, k - (int)orderedRemainedIntersectionIDs.size());
    int ub = std::min((int)orderedRemainedIntersectionIDs.size(), k);

    for (int i = lb; i < ub; i++) {
      int j = k - 1 - i;
      if (j <= i)
        continue;

      assert(IsBetween(i, 0, orderedRemainedIntersectionIDs.size()));
      assert(IsBetween(j, 0, orderedRemainedIntersectionIDs.size()));

      if (count >= maxNum)
        break;

      auto &vp2cand = remainedIntersections[orderedRemainedIntersectionIDs[i]];
      auto &vp3cand = remainedIntersections[orderedRemainedIntersectionIDs[j]];

      if (Distance(vp2cand.value(), vp1.value()) < minFocalLength)
        continue;
      if (Distance(vp2cand.value(), vp1ccenter) <
          vp1cdist / 2.0 - minFocalLength)
        continue;

      HPoint2 vp12center = (vp1 + vp2cand) / 2.0;
      double vp12dist = norm(vp1 - vp2cand).value();

      if (Distance(vp3cand.value(), vp1.value()) < minFocalLength ||
          Distance(vp2cand.value(), vp3cand.value()) < minFocalLength)
        continue;
      if (Distance(vp3cand.value(), vp12center.value()) < vp12dist / 2.0)
        continue;

      double focalLength;
      Point2 principlePoint;
      std::tie(principlePoint, focalLength) =
          ComputePrinciplePointAndFocalLength(vp1.value(), vp2cand.value(),
                                              vp3cand.value());

      if (std::isnan(focalLength) || std::isinf(focalLength))
        continue;

      if (norm(principlePoint) < maxPrinciplePointOffset &&
          IsBetween(focalLength, minFocalLength, maxFocalLength)) {
        count++;

        // get votes from each line
        double score = 0.0;
        for (int lineId = 0; lineId < lines.size(); lineId++) {
          double voteForVP1 = votesPanel(lineId, intersectionIdWithMaxVotes);
          double voteForVP2 = votesRemainedPanel(lineId, i);
          double voteForVP3 = votesRemainedPanel(lineId, j);
          score += std::max({voteForVP1, voteForVP2, voteForVP3});
        }

        if (score > curMaxScore) {
          curMaxScore = score;
          vp2 = vp2cand;
          vp3 = vp3cand;
          curFocal = focalLength;
          curPrinciplePoint = principlePoint;
        }
      }
    }
  }

  if (count == 0) {
    std::cout << "no valid vps, failed" << std::endl;
    succeed = false;
    return std::tuple<std::vector<HPoint2>, double, std::vector<int>>();
  }

  std::vector<HPoint2> vps = {{vp1, vp2, vp3}};

  std::cout << "vp2: " << vp2.value() << "   "
            << "vp3: " << vp3.value() << std::endl;

  lineClasses = ClassifyLines(LinesVotesToPoints(vps, lines), 0.8);
  succeed = true;
  return std::make_tuple(std::move(vps), curFocal, std::move(lineClasses));
}
}

namespace {

// estimate vanishing points using classified lines
HPoint2 EstimateVanishingPointsFromLines(const std::vector<Line2> &lines,
                                         double *score = nullptr) {
  assert(!lines.empty());
  if (lines.size() == 1) {
    if (score) {
      *score = -0.1;
    }
    return HPoint2(lines.front().direction(), 0.0);
  }
  if (lines.size() == 2) {
    if (score) {
      *score = 0.0;
    }
    auto eq1 = cat(lines[0].first, 1.0).cross(cat(lines[0].second, 1.0));
    auto eq2 = cat(lines[1].first, 1.0).cross(cat(lines[1].second, 1.0));
    return HPointFromVector(eq1.cross(eq2));
  }

  using namespace Eigen;

  //// 0    1   2   3
  //// [e1x e1y e2x e2y]
  Map<const Array<double, 4, Eigen::Dynamic>> lineMat(
      (const double *)lines.data(), 4, lines.size());
  auto e1x = lineMat.row(0).eval();
  auto e1y = lineMat.row(1).eval();
  auto e2x = lineMat.row(2).eval();
  auto e2y = lineMat.row(3).eval();

  int lineNum = lines.size();
  auto costFunction = [lineNum, &e1x, &e1y, &e2x, &e2y](const VectorXd &x,
                                                        VectorXd &v) {
    // x: input, v: output
    assert(x.size() == 2);
    double vx(cos(x(0)) * sin(x(1))), vy(sin(x[0]) * sin(x[1])), vz(cos(x[1]));
    //                                                             2
    //(e1y vx - e2y vx - e1x vy + e2x vy + e1x e2y vz - e2x e1y vz)
    //--------------------------------------------------------------
    //                            2                           2
    //    (e1x vz - 2 vx + e2x vz)  + (e1y vz - 2 vy + e2y vz)
    auto top = abs(e1y * vx - e2y * vx - e1x * vy + e2x * vy + e1x * e2y * vz -
                   e2x * e1y * vz)
                   .eval();
    auto bottom = sqrt((e1x * vz - vx * 2.0 + e2x * vz).square() +
                       (e1y * vz - 2.0 * vy + e2y * vz).square())
                      .eval();
    v = (top / bottom).matrix().transpose();
  };

  auto functor =
      misc::MakeGenericNumericDiffFunctor<double>(costFunction, 2, lineNum);
  LevenbergMarquardt<decltype(functor)> lm(std::move(functor));

  // calc initial vp
  Vec3 initialVP =
      cat(lines[0].first, 1.0)
          .cross(cat(lines[0].second, 1.0))
          .cross(cat(lines[1].first, 1.0).cross(cat(lines[1].second, 1.0)));
  initialVP /= norm(initialVP);
  VectorXd x(2);
  x << atan2(initialVP[1], initialVP[0]), acos(initialVP[2]);
  lm.minimize(x);

  if (score) {
    VectorXd scores;
    costFunction(x, scores);
    *score = scores.squaredNorm();
  }

  double vx(cos(x(0)) * sin(x(1))), vy(sin(x[0]) * sin(x[1])), vz(cos(x[1]));
  return HPoint2(Point2(vx, vy), vz);
}
}

Failable<std::tuple<std::vector<HPoint2>, double, std::vector<int>>>
VanishingPointsDetector::operator()(const std::vector<Line2> &lines,
                                    const Sizei &imSize) const {

  Point2 projCenter(imSize.width / 2.0, imSize.height / 2.0);
  double imScale = sqrt(imSize.area());
  double minFocalLength = imScale * _params.minFocalLengthRatio;
  double maxFocalLength = imScale * _params.maxFocalLengthRatio;
  double maxPrinciplePointOffset =
      imScale * _params.maxPrinciplePointOffsetRatio;

  if (_params.algorithm == Naive) {
    std::vector<Line2> offsetedLines = lines;
    for (auto &line : offsetedLines) {
      line.first -= projCenter;
      line.second -= projCenter;
    }
    bool ok = false;
    auto results = EstimateVanishingPointsWithProjectionCenterAtOrigin(
        offsetedLines, minFocalLength, maxFocalLength, maxPrinciplePointOffset,
        ok);
    if (!ok) {
      return nullptr;
    }
    for (HPoint2 &vp : std::get<0>(results)) {
      vp = vp + HPoint2(projCenter);
    }
    return std::move(results);
  } else if (_params.algorithm == MATLAB_PanoContext) {
    misc::Matlab matlab;
    // install lines
    Imaged linesData(lines.size(), 4);
    for (int i = 0; i < lines.size(); i++) {
      linesData(i, 0) = lines[i].first[0];
      linesData(i, 1) = lines[i].first[1];
      linesData(i, 2) = lines[i].second[0];
      linesData(i, 3) = lines[i].second[1];
    }
    matlab.setVar("linesData", linesData);
    matlab.setVar("projCenter", cv::Mat(projCenter));
    // convert to struct array
    matlab << "[vp, f, lineclasses] = panoramix_wrapper_vpdetection(linesData, "
              "projCenter');";

    Imaged vpData = matlab.var("vp").toCVMat(false);
    Imaged focal = matlab.var("f").toCVMat(false);
    Imaged lineClassesData = matlab.var("lineclasses").toCVMat(false);
    if (!(vpData.cols == 2 && vpData.rows == 3)) {
      return nullptr;
    }
    assert(lineClassesData.cols * lineClassesData.rows == lines.size());
    std::vector<HPoint2> vps = {HPoint2({vpData(0, 0), vpData(0, 1)}, 1.0),
                                HPoint2({vpData(1, 0), vpData(1, 1)}, 1.0),
                                HPoint2({vpData(2, 0), vpData(2, 1)}, 1.0)};
    std::vector<int> lineClasses(lines.size(), -1);
    for (int i = 0; i < lines.size(); i++) {
      lineClasses[i] = (int)std::round(lineClassesData(i) - 1);
    }
    return std::make_tuple(std::move(vps), focal(0), std::move(lineClasses));
  } else /*if (_params.algorithm == TardifSimplified)*/ {

    std::vector<std::vector<Line2>> lineClusters;
    std::vector<double> clusterInitialScores;
    std::vector<int> intLabels;
    int classNum = 0;

    {
      std::vector<std::vector<float> *> pts(lines.size());
      for (int i = 0; i < lines.size(); i++) {
        pts[i] = new std::vector<float>{
            (float)lines[i].first[0], (float)lines[i].first[1],
            (float)lines[i].second[0], (float)lines[i].second[1]};
      }

      std::vector<std::vector<float> *> *mModels =
          VPSample::run(&pts, 5000, 2, 0, 3);
      std::vector<unsigned int> labels;
      std::vector<unsigned int> labelCount;
      classNum = VPCluster::run(labels, labelCount, &pts, mModels, 2, 2);

      assert(classNum >= 3);

      for (unsigned int i = 0; i < mModels->size(); ++i)
        delete (*mModels)[i];
      delete mModels;
      for (auto &p : pts) {
        delete p;
      }

      // estimate vps
      assert(lines.size() == labels.size());
      lineClusters.resize(classNum);
      clusterInitialScores.resize(classNum, 0.0);
      intLabels.resize(labels.size());
      for (int i = 0; i < labels.size(); i++) {
        lineClusters[labels[i]].push_back(lines[i]);
        clusterInitialScores[labels[i]] += lines[i].length();
        intLabels[i] = labels[i];
      }
    }

    // initial vps
    std::vector<HPoint2> vps(classNum);
    for (int i = 0; i < lineClusters.size(); i++) {
      if (lineClusters[i].empty())
        continue;
      vps[i] = EstimateVanishingPointsFromLines(lineClusters[i]);
    }

    // find class with maximum initial score as the first class
    int firstClass =
        std::distance(clusterInitialScores.begin(),
                      std::max_element(clusterInitialScores.begin(),
                                       clusterInitialScores.end()));

    int finalClasses[3] = {firstClass, -1, -1};
    double finalFocal = 1.0;
    Point2 finalPP;
    double minViolation = std::numeric_limits<double>::max();

    double logMinFocal = std::log10(minFocalLength);
    double logMaxFocal = std::log10(maxFocalLength);
    double logMiddleFocal = (logMinFocal + logMaxFocal) / 2.0;

    // select the other two classes by traversing
    using namespace Eigen;
    Map<const Array<double, 4, Eigen::Dynamic>> lineMat1(
        (const double *)lineClusters[firstClass].data(), 4,
        lineClusters[firstClass].size());
    for (int i = 0; i < classNum; i++) {
      if (i == firstClass)
        continue;
      Map<const Array<double, 4, Eigen::Dynamic>> lineMat2(
          (const double *)lineClusters[i].data(), 4, lineClusters[i].size());
      for (int j = i + 1; j < classNum; j++) {
        if (j == firstClass)
          continue;
        Map<const Array<double, 4, Eigen::Dynamic>> lineMat3(
            (const double *)lineClusters[j].data(), 4, lineClusters[j].size());

        int ids[] = {firstClass, i, j};

        // calc initial vp
        VectorXd x(6);
        for (int k = 0; k < 3; k++) {
          auto &lineCluster = lineClusters[ids[k]];
          Vec3 initialVP = normalize(VectorFromHPoint(vps[ids[k]]));
          x(k * 2) = atan2(initialVP[1], initialVP[0]);
          x(k * 2 + 1) = acos(initialVP[2]);
        }

        int startPoses[] = {0, lineMat1.cols(),
                            lineMat1.cols() + lineMat2.cols()};
        auto costFunction = [&lineMat1, &lineMat2, &lineMat3, &startPoses,
                             logMiddleFocal, &projCenter, &vps,
                             &ids](const VectorXd &x, VectorXd &v) {
          // x: input, v: output
          assert(x.size() == 6);
          v.resize(lineMat1.cols() + lineMat2.cols() + lineMat3.cols());

          Map<const Array<double, 4, Eigen::Dynamic>> *lineMats[] = {
              &lineMat1, &lineMat2, &lineMat3};
          Vec3 currentVPs[3];
          double maxOutAngle = 0.0;
          for (int k = 0; k < 3; k++) {
            currentVPs[k] =
                Vec3(cos(x(k * 2)) * sin(x(k * 2 + 1)),
                     sin(x[k * 2]) * sin(x[k * 2 + 1]), cos(x[k * 2 + 1]));
            maxOutAngle = std::max(
                maxOutAngle, AngleBetweenUndirected(
                                 VectorFromHPoint(vps[ids[k]]), currentVPs[k]));
          }
          for (int k = 0; k < 3; k++) {
            auto &lineMat = *lineMats[k];
            double vx = currentVPs[k][0], vy = currentVPs[k][1],
                   vz = currentVPs[k][2];
            auto e1x = lineMat.row(0);
            auto e1y = lineMat.row(1);
            auto e2x = lineMat.row(2);
            auto e2y = lineMat.row(3);

            //                                                             2
            //(e1y vx - e2y vx - e1x vy + e2x vy + e1x e2y vz - e2x e1y vz)
            //--------------------------------------------------------------
            //                            2                           2
            //    (e1x vz - 2 vx + e2x vz)  + (e1y vz - 2 vy + e2y vz)
            auto top = abs(e1y * vx - e2y * vx - e1x * vy + e2x * vy +
                           e1x * e2y * vz - e2x * e1y * vz)
                           .eval();
            auto bottom = sqrt((e1x * vz - vx * 2.0 + e2x * vz).square() +
                               (e1y * vz - 2.0 * vy + e2y * vz).square())
                              .eval();
            v.middleRows(startPoses[k], lineMats[k]->cols()) =
                (top / bottom).matrix().transpose();
          }

          Point2 pp;
          double focal;
          std::tie(pp, focal) = ComputePrinciplePointAndFocalLength(
              Vec2(currentVPs[0][0], currentVPs[0][1]) / currentVPs[0][2],
              Vec2(currentVPs[1][0], currentVPs[1][1]) / currentVPs[1][2],
              Vec2(currentVPs[2][0], currentVPs[2][1]) / currentVPs[2][2]);

          if (std::isinf(focal) || std::isnan(focal)) {
            // v.bottomRows(1)(0) = 1;
            v *= 1e6;
          } else {
            static const double angleThreshold = M_PI / 20.0;
            // v.bottomRows(1)(0) = 1;
            // v.bottomRows(1)(0) = 10 * (Distance(pp, projCenter)) + 20.0 *
            // abs(log10(focal) - logMiddleFocal);
            /*if (maxOutAngle > angleThreshold){
                v *= (maxOutAngle / angleThreshold);
            }*/
          }
        };

        auto functor = misc::MakeGenericNumericDiffFunctor<double>(
            costFunction, 6,
            lineMat1.cols() + lineMat2.cols() + lineMat3.cols());
        LevenbergMarquardt<decltype(functor)> lm(functor);

        VectorXd oldx = x;
        lm.minimize(x);
        /*if (oldx == x){
            std::cout << "NO OPTIMIZATION AT ALL !!!!!!!!!!!!!!!!!!!!!!!" <<
        std::endl;
        }*/

        // get the optimized vps
        Vec3 currentVPs[3];
        for (int k = 0; k < 3; k++) {
          currentVPs[k] =
              Vec3(cos(x(k * 2)) * sin(x(k * 2 + 1)),
                   sin(x[k * 2]) * sin(x[k * 2 + 1]), cos(x[k * 2 + 1]));
        }

        Point2 pp;
        double focal;
        std::tie(pp, focal) = ComputePrinciplePointAndFocalLength(
            Vec2(currentVPs[0][0], currentVPs[0][1]) / currentVPs[0][2],
            Vec2(currentVPs[1][0], currentVPs[1][1]) / currentVPs[1][2],
            Vec2(currentVPs[2][0], currentVPs[2][1]) / currentVPs[2][2]);

        if (std::isnan(focal) || std::isinf(focal))
          continue;

        if (!IsBetween(focal, minFocalLength, maxFocalLength))
          continue;

        double ppViolation = Distance(pp, projCenter);

        auto minmaxScore =
            std::minmax({clusterInitialScores[i], clusterInitialScores[j]});
        double violation =
            (ppViolation > maxPrinciplePointOffset
                 ? (80.0 * (ppViolation - maxPrinciplePointOffset) /
                    maxPrinciplePointOffset)
                 : 0.0) -
            100 * (minmaxScore.first + minmaxScore.second) /
                clusterInitialScores[firstClass];

        if (violation < minViolation) {
          minViolation = violation;
          for (int k = 0; k < 3; k++) {
            vps[ids[k]] = HPointFromVector(currentVPs[k]);
          }
          finalClasses[1] = i;
          finalClasses[2] = j;
          finalFocal = focal;
          finalPP = pp;
        }
      }
    }

    if (finalClasses[1] == -1 || finalClasses[2] == -1) {
      return nullptr;
    }

    for (int k = 0; k < 3; k++) {
      std::swap(vps[k], vps[finalClasses[k]]);
      std::cout << "vp[" << k << "] = " << vps[k].value() << std::endl;
    }
    std::cout << "focal = " << finalFocal << std::endl;

    for (int &l : intLabels) {
      for (int k = 0; k < 3; k++) {
        if (l == k) {
          l = finalClasses[k];
          break;
        } else if (l == finalClasses[k]) {
          l = k;
          break;
        }
      }
    }
    return std::make_tuple(std::move(vps), finalFocal, std::move(intLabels));
  }
}

Failable<std::tuple<std::vector<HPoint2>, double>> VanishingPointsDetector::
operator()(std::vector<Classified<Line2>> &lines, const Sizei &imSize) const {
  std::vector<HPoint2> vps;
  double focalLength = 0.0;
  std::vector<int> lineClassies;
  std::vector<Line2> plines(lines.size());
  for (int i = 0; i < lines.size(); i++)
    plines[i] = lines[i].component;
  auto result = (*this)(plines, imSize);
  if (result.null()) {
    return nullptr;
  }
  std::tie(vps, focalLength, lineClassies) = result.unwrap();
  assert(lineClassies.size() == lines.size());
  for (int i = 0; i < lines.size(); i++)
    lines[i].claz = lineClassies[i];
  return std::make_tuple(std::move(vps), focalLength);
}

#pragma endregion VanishingPointsDetector

std::pair<Failable<double>, Failable<double>>
ComputeFocalsFromHomography(const Mat3 &H) {
  /// from OpenCV code

  const double *h = reinterpret_cast<const double *>(H.val);

  double d1, d2; // Denominators
  double v1, v2; // Focal squares value candidates
  double f0 = 0.0, f1 = 0.0;

  std::pair<Failable<double>, Failable<double>> results;

  d1 = h[6] * h[7];
  d2 = (h[7] - h[6]) * (h[7] + h[6]);
  v1 = -(h[0] * h[1] + h[3] * h[4]) / d1;
  v2 = (h[0] * h[0] + h[3] * h[3] - h[1] * h[1] - h[4] * h[4]) / d2;
  if (v1 < v2)
    std::swap(v1, v2);
  if (v1 > 0 && v2 > 0)
    results.second = sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
  else if (v1 > 0)
    results.second = sqrt(v1);

  d1 = h[0] * h[3] + h[1] * h[4];
  d2 = h[0] * h[0] + h[1] * h[1] - h[3] * h[3] - h[4] * h[4];
  v1 = -h[2] * h[5] / d1;
  v2 = (h[5] * h[5] - h[2] * h[2]) / d2;
  if (v1 < v2)
    std::swap(v1, v2);
  if (v1 > 0 && v2 > 0)
    results.first = sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
  else if (v1 > 0)
    results.first = sqrt(v1);

  return results;
}

std::tuple<std::vector<Point2>, std::vector<Point2>, std::vector<Point2>,
           std::vector<int>, int>
sample_line(const std::vector<Line2> &lines,
            const std::vector<int> &lineClasses) {

  int sample_rate = 10; // sample every 5 pixel on line
  std::vector<Point2> ls[3];
  int count = 0;
  std::vector<int> lsclass;

  for (int i = 0; i < lines.size(); i++) {
    int n_sample = ceil(norm(lines[i].first - lines[i].second) / sample_rate);
    int lclass = lineClasses[i];

    /*ls(i).sample = [...
    linspace(lines(i).point1(1), lines(i).point2(1), n_sample)' ...
    linspace(lines(i).point1(2), lines(i).point2(2), n_sample)' ];*/
    Point2 eps;
    eps[0] = (lines[i].first - lines[i].second).val[0] / n_sample;
    eps[1] = (lines[i].first - lines[i].second).val[1] / n_sample;
    Point2 curr = lines[i].second;

    if (lclass == -1) {
      continue;
    }

    for (int j = 0; j < n_sample; j++) {
      ls[lclass].push_back(curr);
      lsclass.push_back(lclass);
      count++;
      curr = curr + eps;
    }

    /*ls(i).lineclass = repmat(lines(i).lineclass, n_sample, 1);*/
  }

  return std::make_tuple(ls[0], ls[1], ls[2], lsclass, count);
}

Point2 move_line_towards_vp(Point2 curp1, Point2 curp2, Point2 vp, int amount,
                            bool &atvp, Point2 &newp2) {
  Point2 vec1 = curp1 - vp;
  Point2 vec2 = curp2 - vp;
  double len1 = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]);
  double len2 = sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);
  Point2 norm1;
  norm1[0] = vec1[0] / len1 * amount;
  norm1[1] = vec1[1] / len1 * amount;
  Point2 newp1;
  Point2 norm2;
  double ratio21 = len2 / len1;
  norm2[0] = vec2[0] / len2 * amount * ratio21;
  norm2[1] = vec2[1] / len2 * amount * ratio21;

  if (len1 < abs(amount)) {
    newp1 = curp1;
    newp2 = curp2;
    atvp = 1;
  } else {

    newp1 = curp1 + norm1;
    newp2 = curp2 + norm2;
    atvp = 0;
  }

  return newp1;
};

std::vector<cv::Point2f> getpoly(const Line2 &line,
                                 const Point2 &vanishingPoints, int to_away,
                                 const cv::Size &imageSize,
                                 std::vector<Point2> sample) {
  cv::Point2f p1;
  p1.x = line.first[0];
  p1.y = line.first[1];
  cv::Point2f p2;
  p2.x = line.second[0];
  p2.y = line.second[1];

  cv::Point2f curp1 = p1;
  cv::Point2f curp2 = p2;
  int moveAmount = 64;

  std::vector<cv::Point2f> result;

  //	Imagei oo = Imagei::ones(imageSize) * -1;;

  while (moveAmount >= 1) {
    bool atvp = 0;
    Point2 hcurp1;
    Point2 hcurp2;

    hcurp1[0] = curp1.x;
    hcurp1[1] = curp1.y;
    hcurp2[0] = curp2.x;
    hcurp2[1] = curp2.y;

    Point2 hnewp2;
    Point2 hnewp1 = move_line_towards_vp(hcurp1, hcurp2, vanishingPoints,
                                         to_away * moveAmount, atvp, hnewp2);
    cv::Point2f newp1;
    cv::Point2f newp2;
    newp1.x = hnewp1[0];
    newp1.y = hnewp1[1];
    newp2.x = hnewp2[0];
    newp2.y = hnewp2[1];

    bool failed = 0;
    if (atvp == 1) {

      failed = 1;
    } else if ((hnewp1[0] > imageSize.width || hnewp1[0] < 1 ||
                hnewp1[1] > imageSize.height || hnewp1[1] < 1) ||
               (hnewp2[0] > imageSize.width || hnewp2[0] < 1 ||
                hnewp2[1] > imageSize.height || hnewp2[1] < 1)) {

      failed = 1;
    } else {
      std::vector<cv::Point2f> poly;
      poly.push_back(p1);
      poly.push_back(p2);
      cv::Point2f pt;
      for (int i = 0; i < sample.size(); i++) {
        pt.x = (sample[i])[0];
        pt.y = (sample[i])[1];
        poly.push_back(newp2);
        poly.push_back(newp1);
        poly.push_back(p1);
        double isstop = cv::pointPolygonTest(poly, pt, false);

        if (isstop > 0) {
          failed = 1;
          break;
        }
      }
    }

    if (failed == 1) {

      moveAmount = moveAmount / 2;
    } else {
      curp1 = newp1;
      curp2 = newp2;
    }
  }
  // cv::Point** pts;
  // int* npts;
  // int ncontours;
  // cv::Mat IMG = cv::Mat::zeros(imageSize, CV_32SC1);
  //
  // cv::Point offset = cv::Point();
  ///*cv::fillPoly(IMG, pts, npts, ncontours, 225, 8, 0, offset);
  // cv::fillPoly()*/

  result.push_back(p1);
  result.push_back(p2);

  result.push_back(curp2);
  result.push_back(curp1);

  return result;
}

void myfillPoly(cv::Mat &img, std::vector<std::vector<Point<int32_t, 2>>> &pts,
                int label) {
  for (auto &ps : pts) {
    cv::fillConvexPoly(img, ps, label);
  }
}

Imagei ExtractOrientationMaps(const cv::Size &imageSize,
                              const std::vector<Line2> &lines,
                              const std::array<HPoint2, 3> &vanishingPoints,
                              const std::vector<int> &lineClasses) {

  // fill with -1 as default
  Imagei omap = Imagei::ones(imageSize) * -1;
  std::vector<std::vector<Point<int32_t, 2>>> lineextimg[3][3][2];

  std::vector<Point2> ls[3];
  std::vector<int> lsclass;
  int lsSize;

  std::tie(ls[0], ls[1], ls[2], lsclass, lsSize) =
      sample_line(lines, lineClasses);

  size_t lnum = lines.size();
  // size_t lnum = 40;
  for (int i = 0; i < lnum; i++) {
    int lclass = lineClasses[i];
    if (lclass == -1)
      continue;
    for (int j = 0; j < 3; j++) {
      if (j == lclass) {
        ;
      } else {
        Point2 Vp = vanishingPoints[j].value();
        // Vp[0];
        int orient;
        for (int k = 0; k < 3; k++) {
          if (k != lclass && k != j)
            orient = k;
        }

        std::vector<cv::Point2f> poly1 =
            getpoly(lines[i], Vp, -1, imageSize, ls[orient]);
        std::vector<Point<int32_t, 2>> ppoly1;
        std::vector<Point<int32_t, 2>> ppoly2;
        for (int ii = 0; ii < poly1.size(); ii++) {
          Point<int32_t, 2> temppoint;
          temppoint[0] = poly1.at(ii).x;
          temppoint[1] = poly1.at(ii).y;
          ppoly1.push_back(temppoint);
        }
        lineextimg[lclass][j][0].push_back(ppoly1);
        std::vector<cv::Point2f> poly2 =
            getpoly(lines[i], Vp, 1, imageSize, ls[orient]);
        for (int ii = 0; ii < poly1.size(); ii++) {
          Point<int32_t, 2> temppoint;
          temppoint[0] = poly2.at(ii).x;
          temppoint[1] = poly2.at(ii).y;
          ppoly2.push_back(temppoint);
        }
        lineextimg[lclass][j][1].push_back(ppoly2);
        // std::cout << "i =  " << i << "," << j << std::endl;
        /*if (i == 4 && j == 0)
        int ass = 0;*/
      }
    }
  }

  Imagei omap1 = Imagei::ones(imageSize) * -1;
  Imagei omap2 = Imagei::ones(imageSize) * -1;
  Imagei omap3 = Imagei::ones(imageSize) * -1;
  Imagei omap12 = Imagei::ones(imageSize) * -1;
  Imagei omap21 = Imagei::ones(imageSize) * -1;
  Imagei omap10 = Imagei::ones(imageSize) * -1;
  Imagei omap01 = Imagei::ones(imageSize) * -1;
  Imagei omap02 = Imagei::ones(imageSize) * -1;
  Imagei omap20 = Imagei::ones(imageSize) * -1;
  myfillPoly(omap12, lineextimg[1][2][0], 0);
  myfillPoly(omap12, lineextimg[1][2][1], 0);
  myfillPoly(omap21, lineextimg[2][1][0], 0);
  myfillPoly(omap21, lineextimg[2][1][1], 0);
  myfillPoly(omap10, lineextimg[1][0][0], 2);
  myfillPoly(omap10, lineextimg[1][0][1], 2);
  myfillPoly(omap01, lineextimg[0][1][0], 2);
  myfillPoly(omap01, lineextimg[0][1][1], 2);
  myfillPoly(omap20, lineextimg[2][0][0], 1);
  myfillPoly(omap20, lineextimg[2][0][1], 1);
  myfillPoly(omap02, lineextimg[0][2][0], 1);
  myfillPoly(omap02, lineextimg[0][2][1], 1);

  for (int j = 0; j < imageSize.height; j++) {
    for (int i = 0; i < imageSize.width; i++) {
      if (omap12[j][i] == 0 && omap21[j][i] == 0) {
        omap1[j][i] = 0;
      }
    }
  }
  for (int j = 0; j < imageSize.height; j++) {
    for (int i = 0; i < imageSize.width; i++) {
      if (omap02[j][i] == 1 && omap20[j][i] == 1) {
        omap3[j][i] = 1;
      }
    }
  }
  for (int j = 0; j < imageSize.height; j++) {
    for (int i = 0; i < imageSize.width; i++) {
      if (omap10[j][i] == 2 && omap01[j][i] == 2) {
        omap2[j][i] = 2;
      }
    }
  }

  Image3d omapImage1 = Image3d::zeros(omap.size());
  Image3d omapImage2 = Image3d::zeros(omap.size());
  Image3d omapImage3 = Image3d::zeros(omap.size());

  for (int x = 0; x < omap.cols; x++) {
    for (int y = 0; y < omap.rows; y++) {
      Vec3 color(0, 0, 0);
      int omapValue = omap1(y, x);
      if (omapValue >= 0 && omapValue < 3)
        color[omapValue] = 255;
      omapImage1(y, x) = color;
    }
  }
  for (int x = 0; x < omap.cols; x++) {
    for (int y = 0; y < omap.rows; y++) {
      Vec3 color(0, 0, 0);
      int omapValue = omap2(y, x);
      if (omapValue >= 0 && omapValue < 3)
        color[omapValue] = 255;
      omapImage2(y, x) = color;
    }
  }
  for (int x = 0; x < omap.cols; x++) {
    for (int y = 0; y < omap.rows; y++) {
      Vec3 color(0, 0, 0);
      int omapValue = omap3(y, x);
      if (omapValue >= 0 && omapValue < 3)
        color[omapValue] = 255;
      omapImage3(y, x) = color;
    }
  }
  // cv::imshow("Orientation Map1", omapImage1);
  // cv::imshow("Orientation Map2", omapImage2);
  // cv::imshow("Orientation Map3", omapImage3);
  // cv::waitKey();
  for (int j = 0; j < imageSize.height; j++) {
    for (int i = 0; i < imageSize.width; i++) {
      if (omap1[j][i] == 0 && omap2[j][i] == -1 && omap3[j][i] == -1) {
        omap[j][i] = 0;
      }
      if (omap1[j][i] == -1 && omap2[j][i] == 2 && omap3[j][i] == -1) {
        omap[j][i] = 2;
      }
      if (omap1[j][i] == -1 && omap2[j][i] == -1 && omap3[j][i] == 1) {
        omap[j][i] = 1;
      }
    }
  }

  // Imagei ao23 = (lineextimg[1][2][0] | lineextimg[1][2][1]);
  // Imagei ao32 = (lineextimg[2][1][0] | lineextimg[2][1][1]);
  // Imagei ao13 = (lineextimg[0][2][0] | lineextimg[0][2][1]);
  // Imagei ao31 = (lineextimg[2][0][0] | lineextimg[2][0][1]);
  // Imagei ao12 = (lineextimg[0][1][0] | lineextimg[0][1][1]);
  // Imagei ao21 = (lineextimg[1][0][0] | lineextimg[1][0][1]);
  // Imagei aa23 = (lineextimg[1][2][0] & lineextimg[1][2][1]);
  // Imagei aa32 = (lineextimg[2][1][0] & lineextimg[2][1][1]);
  // Imagei aa13 = (lineextimg[0][2][0] & lineextimg[0][2][1]);
  // Imagei aa31 = (lineextimg[2][0][0] & lineextimg[2][0][1]);
  // Imagei aa12 = (lineextimg[0][1][0] & lineextimg[0][1][1]);
  // Imagei aa21 = (lineextimg[1][0][0] & lineextimg[1][0][1]);
  //
  //
  //
  // Imagei Aa1  = ao23 & ao32;
  // Imagei Aa2 = ao13 & ao31;
  // Imagei Aa3  = ao12 & ao21;

  // Imagei b1  = Aa1 &~Aa2 &~Aa3;
  // Imagei b2 = ~Aa1 &Aa2 &~Aa3;
  // Imagei b3 = ~Aa1 &~Aa2 &Aa3;

  ////omap = cat(3, b{ 1 }, b{ 2 }, b{ 3 });

  // Imagei testmap = Imagei::ones(imageSize) * -1;
  // std::vector<std::vector<Point<int32_t, 2>>> polys = {
  //	{
  //	{ 0, 0 },
  //	{ 20, 100 },
  //	{ 50, 100 },
  //	{ 150, 20 }
  //},
  //{
  //	{ 0, 15 },
  //	{ 20, 100 },
  //	{ 50, 100 },
  //	{ 150, 20 }
  //} };
  //// fill with 1
  // myfillPoly(testmap, polys, 1);
  // Image3d testmapImage = Image3d::zeros(omap.size());
  // for (int x = 0; x < omap.cols; x++) {
  //	for (int y = 0; y < omap.rows; y++) {
  //		Vec3 color(0, 0, 0);
  //		int omapValue = testmap(y, x);
  //		if (omapValue >= 0 && omapValue < 3)
  //			color[omapValue] = 255;
  //		testmapImage(y, x) = color;
  //	}
  //}
  // cv::imshow("Orientation Map test", testmapImage);

  return omap;
}

Imagei ComputeOrientationMaps(const std::vector<Classified<Line2>> &lines,
                              const std::vector<HPoint2> &vps,
                              const Sizei &imSize) {
  SetClock();
  std::array<HPoint2, 3> vanishingPoints = {{vps[0], vps[1], vps[2]}};
  std::vector<Line2> ls;
  ls.reserve(lines.size());
  std::vector<int> lcs;
  lcs.reserve(lines.size());
  for (int i = 0; i < lines.size(); i++) {
    if (lines[i].claz == -1 || lines[i].claz >= 3)
      continue;
    ls.push_back(lines[i].component);
    lcs.push_back(lines[i].claz);
  }
  return ExtractOrientationMaps(imSize, ls, vanishingPoints, lcs);
}
}
}
