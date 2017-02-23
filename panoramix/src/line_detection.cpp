#include "pch.hpp"

extern "C" {
#include <lsd.h>
}

#include "clock.hpp"
#include "containers.hpp"
#include "line_detection.hpp"
#include "utility.hpp"

namespace pano {
namespace core {

#pragma region LineSegmentExtractor

namespace {

void ExtractLines(const cv::Mat &im, std::vector<Line2> &lines, int minlen,
                  int xborderw, int yborderw, int numDir) {

  std::cout << "image processing..." << std::endl;

  cv::Mat gim;
  cv::cvtColor(im, gim, CV_BGR2GRAY);
  int h = gim.rows;
  int w = gim.cols;

  cv::Mat dx, dy;
  cv::Mat ggim;
  cv::GaussianBlur(gim, ggim, cv::Size(7, 7), 1.5);
  cv::Sobel(ggim, dx, CV_64F, 1, 0);
  cv::Sobel(ggim, dy, CV_64F, 0, 1);

  cv::Mat imCanny;
  cv::Canny(gim, imCanny, 5, 20);

  std::cout << "gradient binning..." << std::endl;

  cv::Mat imBinIds(im.size(), CV_32SC1); // int32_t
  std::vector<std::set<int>> pixelIdsSet(numDir);

  for (int x = 0; x < imCanny.cols; x++) {
    for (int y = 0; y < imCanny.rows; y++) {
      if (imCanny.at<uchar>(y, x) > 0) {
        double a = atan(dy.at<double>(y, x) / dx.at<double>(y, x));
        if (isnan(a)) { // NaN
          continue;
        }
        // compute bin id
        int binId = int((a / M_PI + 0.5) * numDir);
        if (binId == -1)
          binId = 0;
        if (binId == numDir)
          binId = numDir - 1;

        int pixelId = y + imCanny.rows * x;

        imBinIds.at<int32_t>(y, x) = binId;

        pixelIdsSet[(binId + numDir - 1) % numDir].insert(pixelId);
        pixelIdsSet[binId].insert(pixelId);
        pixelIdsSet[(binId + 1) % numDir].insert(pixelId);
      } else {
        imBinIds.at<int32_t>(y, x) = -1;
      }
    }
  }

  std::vector<int> xs, ys, ids;
  xs.reserve(512);
  ys.reserve(512);
  ids.reserve(512);

  std::cout << "collecting pixels.." << std::endl;

  for (int binId = 0; binId < numDir; binId++) {
    // search in bins
    auto pixelIdNotSearchedYet = pixelIdsSet[binId];

    while (true) {
      if (pixelIdNotSearchedYet.empty())
        break;
      int rootId = *pixelIdNotSearchedYet.begin();

      // BFS
      xs.clear();
      ys.clear();
      ids.clear();
      int x = rootId / imCanny.rows;
      int y = rootId - imCanny.rows * x;
      xs.push_back(x);
      ys.push_back(y);
      ids.push_back(rootId);

      // used for search only
      pixelIdNotSearchedYet.erase(rootId);
      int head = 0;

      static const int xdirs[] = {1, 1, 0, -1, -1, -1, 0, 1};
      static const int ydirs[] = {0, 1, 1, 1, 0, -1, -1, -1};
      while (true) {
        if (head == xs.size())
          break;
        x = xs[head];
        y = ys[head];
        for (int k = 0; k < 8; k++) {
          int nx = x + xdirs[k];
          int ny = y + ydirs[k];
          int npixelId = ny + imCanny.rows * nx;
          if (pixelIdNotSearchedYet.find(npixelId) !=
              pixelIdNotSearchedYet.end()) {
            xs.push_back(nx);
            ys.push_back(ny);
            ids.push_back(npixelId);
            pixelIdNotSearchedYet.erase(npixelId);
          }
        }
        head++;
      }

      int edgeSize = (int)xs.size();
      if (edgeSize < minlen)
        continue;

      cv::Mat xsmat(xs), ysmat(ys);
      double meanx = cv::mean(xsmat).val[0], meany = cv::mean(ysmat).val[0];
      cv::Mat zmx = xsmat - meanx, zmy = ysmat - meany;

      cv::Mat v, lambda;
      Mat<double, 2, 2> D;
      D(0, 0) = cv::sum(zmx.mul(zmx)).val[0];
      D(0, 1) = D(1, 0) = cv::sum(zmx.mul(zmy)).val[0];
      D(1, 1) = cv::sum(zmy.mul(zmy)).val[0];
      cv::eigen(D, lambda, v);

      double theta = atan2(v.at<double>(0, 1), v.at<double>(0, 0));
      double confidence = std::numeric_limits<double>::max();
      if (lambda.at<double>(1) > 0) {
        confidence = lambda.at<double>(0) / lambda.at<double>(1);
      }

      // build line
      if (confidence >= 200) { ////
        for (int pid : ids) {
          pixelIdsSet[binId].erase(pid);
          pixelIdsSet[(binId - 1 + numDir) % numDir].erase(pid);
          pixelIdsSet[(binId + 1) % numDir].erase(pid);
        }

        auto xends = std::minmax_element(xs.begin(), xs.end());
        auto yends = std::minmax_element(ys.begin(), ys.end());
        double minx = *xends.first, maxx = *xends.second;
        double miny = *yends.first, maxy = *yends.second;

        if (maxx <= xborderw || minx >= w - xborderw || maxy <= yborderw ||
            miny >= h - yborderw)
          continue;

        double len =
            sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny));
        double x1 = meanx - cos(theta) * len / 2;
        double x2 = meanx + cos(theta) * len / 2;
        double y1 = meany - sin(theta) * len / 2;
        double y2 = meany + sin(theta) * len / 2;

        lines.push_back({Vec2(x1, y1), Vec2(x2, y2)});
      }
    }
  }

  std::cout << "done" << std::endl;
}

void ExtractLinesUsingLSD(const cv::Mat &im, std::vector<Line2> &lines,
                          double minlen, int xbwidth, int ybwidth,
                          std::vector<double> *lineWidths = nullptr,
                          std::vector<double> *anglePrecisions = nullptr,
                          std::vector<double> *negLog10NFAs = nullptr) {

  cv::Mat gim;
  cv::cvtColor(im, gim, CV_BGR2GRAY);
  gim.convertTo(gim, CV_64FC1);
  // cv::imshow("gim", gim);
  // cv::waitKey();

  int h = gim.rows;
  int w = gim.cols;

  double *imgData = new double[h * w];
  for (int x = 0; x < w; x++) {
    for (int y = 0; y < h; y++) {
      imgData[x + y * w] = gim.at<double>(cv::Point(x, y));
    }
  }

  int nOut;

  // x1,y1,x2,y2,width,p,-log10(NFA)
  double *linesData = lsd(&nOut, imgData, w, h);

  lines.clear();
  lines.reserve(nOut);
  if (lineWidths) {
    lineWidths->clear();
    lineWidths->reserve(nOut);
  }
  if (anglePrecisions) {
    anglePrecisions->clear();
    anglePrecisions->reserve(nOut);
  }
  if (negLog10NFAs) {
    negLog10NFAs->clear();
    negLog10NFAs->reserve(nOut);
  }
  for (int i = 0; i < nOut; i++) {
    Line2 line = {Point2(linesData[7 * i + 0], linesData[7 * i + 1]),
                  Point2(linesData[7 * i + 2], linesData[7 * i + 3])};
    if (line.length() < minlen)
      continue;
    if (line.first[0] <= xbwidth && line.second[0] <= xbwidth ||
        line.first[0] >= w - xbwidth && line.second[0] >= w - xbwidth ||
        line.first[1] <= ybwidth && line.second[1] <= ybwidth ||
        line.first[1] >= h - ybwidth && line.second[1] >= h - ybwidth) {
      continue;
    }
    lines.emplace_back(line);
    if (lineWidths) {
      lineWidths->push_back(linesData[7 * i + 4]);
    }
    if (anglePrecisions) {
      anglePrecisions->push_back(linesData[7 * i + 5]);
    }
    if (negLog10NFAs) {
      negLog10NFAs->push_back(linesData[7 * i + 6]);
    }
  }

  delete[] linesData;
  delete[] imgData;
}
}

LineSegmentExtractor::Feature LineSegmentExtractor::
operator()(const Image &im) const {
  LineSegmentExtractor::Feature lines;
  if (_params.algorithm == LSD) {
    ExtractLinesUsingLSD(im, lines, _params.minLength, _params.xBorderWidth,
                         _params.yBorderWidth);
  } else if (_params.algorithm == GradientGrouping) {
    ExtractLines(im, lines, _params.minLength, _params.xBorderWidth,
                 _params.yBorderWidth, _params.numDirs);
  }
  return lines;
}

LineSegmentExtractor::Feature LineSegmentExtractor::
operator()(const Image &im, int pyramidHeight, int minSize) const {
  Feature lines;
  Image image = im.clone();
  for (int i = 0; i < pyramidHeight; i++) {
    if (image.cols < minSize || image.rows < minSize)
      break;
    Feature ls = (*this)(image);
    for (auto &l : ls) {
      lines.push_back(l * (double(im.cols) / image.cols));
    }
    cv::pyrDown(image, image);
  }
  return lines;
}

#pragma endregion LineSegmentExtractor

std::vector<HPoint2>
ComputeLineIntersections(const std::vector<Line2> &lines,
                         std::vector<std::pair<int, int>> *lineids,
                         bool suppresscross, double minDistanceOfLinePairs) {

  SetClock();

  std::vector<HPoint2> hinterps;

  size_t lnum = lines.size();
  for (int i = 0; i < lnum; i++) {
    Vec3 eqi = cv::Vec3d(lines[i].first[0], lines[i].first[1], 1)
                   .cross(cv::Vec3d(lines[i].second[0], lines[i].second[1], 1));
    for (int j = i + 1; j < lnum; j++) {
      if (minDistanceOfLinePairs < std::numeric_limits<double>::max()) {
        if (Distance(lines[i], lines[j]) < minDistanceOfLinePairs)
          continue;
      }

      Vec3 eqj =
          cv::Vec3d(lines[j].first[0], lines[j].first[1], 1)
              .cross(cv::Vec3d(lines[j].second[0], lines[j].second[1], 1));
      Vec3 interp = eqi.cross(eqj);
      if (interp[0] == 0 && interp[1] == 0 &&
          interp[2] == 0) { // lines overlapped
        interp[0] = -eqi[1];
        interp[1] = eqi[0];
      }
      interp /= norm(interp);

      if (suppresscross) {
        auto &a1 = lines[i].first;
        auto &a2 = lines[i].second;
        auto &b1 = lines[j].first;
        auto &b2 = lines[j].second;
        double q = a1[0] * b1[1] - a1[1] * b1[0] - a1[0] * b2[1] +
                   a1[1] * b2[0] - a2[0] * b1[1] + a2[1] * b1[0] +
                   a2[0] * b2[1] - a2[1] * b2[0];
        double t = (a1[0] * b1[1] - a1[1] * b1[0] - a1[0] * b2[1] +
                    a1[1] * b2[0] + b1[0] * b2[1] - b1[1] * b2[0]) /
                   q;
        if (t > 0 && t < 1 && t == t)
          continue;
      }
      hinterps.push_back(HPointFromVector(interp));
      if (lineids)
        lineids->emplace_back(i, j);
    }
  }

  return hinterps;
}

std::vector<Vec3>
ComputeLineIntersections(const std::vector<Line3> &lines,
                         std::vector<std::pair<int, int>> *lineids,
                         double minAngleDistanceBetweenLinePairs) {
  SetClock();

  std::vector<Vec3> interps;
  size_t lnum = lines.size();
  for (int i = 0; i < lnum; i++) {
    Vec3 ni = normalize(lines[i].first.cross(lines[i].second));

    for (int j = i + 1; j < lnum; j++) {
      auto nearest = DistanceBetweenTwoLines(lines[i], lines[j]).second;
      if (minAngleDistanceBetweenLinePairs < M_PI &&
          AngleBetweenDirected(nearest.first.position,
                               nearest.second.position) <
              minAngleDistanceBetweenLinePairs)
        continue;

      Vec3 nj = normalize(lines[j].first.cross(lines[j].second));
      Vec3 interp = ni.cross(nj);

      if (norm(interp) < 1e-5) {
        continue;
      }

      interp /= norm(interp);
      interps.push_back(interp);
      if (lineids)
        lineids->emplace_back(i, j);
    }
  }

  return interps;
}

DenseMatd ClassifyLines(std::vector<Classified<Line2>> &lines,
                        const std::vector<HPoint2> &vps, double angleThreshold,
                        double sigma, double scoreThreshold,
                        double avoidVPDistanceThreshold) {

  size_t nlines = lines.size();
  size_t npoints = vps.size();
  DenseMatd linescorestable(nlines, npoints, 0.0);
  for (int i = 0; i < nlines; i++) {
    auto &line = lines[i];
    // classify lines
    line.claz = -1;

    // get score based on angle
    for (int j = 0; j < vps.size(); j++) {
      auto &point = vps[j];
      double angle =
          std::min(AngleBetweenDirected(
                       line.component.direction(),
                       (point - HPoint2(line.component.center())).numerator),
                   AngleBetweenDirected(
                       -line.component.direction(),
                       (point - HPoint2(line.component.center())).numerator));
      double score = exp(-(angle / angleThreshold) * (angle / angleThreshold) /
                         sigma / sigma / 2);
      if (avoidVPDistanceThreshold >= 0.0 &&
          Distance(point.value(), line.component) < avoidVPDistanceThreshold) {
        linescorestable(i, j) = -1.0;
      } else {
        linescorestable(i, j) = (angle > angleThreshold) ? 0 : score;
      }
    }

    double curscore = scoreThreshold;
    for (int j = 0; j < vps.size(); j++) {
      if (linescorestable(i, j) > curscore) {
        line.claz = j;
        curscore = linescorestable(i, j);
      }
    }
  }

  return linescorestable;
}

DenseMatd ClassifyLines(std::vector<Classified<Line3>> &lines,
                        const std::vector<Vec3> &vps, double angleThreshold,
                        double sigma, double scoreThreshold,
                        double avoidVPAngleThreshold,
                        double scoreAdvatangeRatio) {

  size_t nlines = lines.size();
  size_t npoints = vps.size();

  DenseMatd linescorestable(nlines, npoints, 0.0);

  for (size_t i = 0; i < nlines; i++) {
    const Vec3 &a = lines[i].component.first;
    const Vec3 &b = lines[i].component.second;
    Vec3 normab = a.cross(b);
    normab /= norm(normab);

    // get score based on angle
    for (int j = 0; j < npoints; j++) {
      const Vec3 &point = vps[j];
      double angle = abs(asin(normab.dot(point)));
      double score = exp(-(angle / angleThreshold) * (angle / angleThreshold) /
                         sigma / sigma / 2);
      if (avoidVPAngleThreshold >= 0.0 &&
          std::min(Distance(normalize(point), normalize(lines[i].component)),
                   Distance(normalize(-point),
                            normalize(lines[i].component))) <
              2.0 * sin(avoidVPAngleThreshold /
                        2.0)) { // avoid that a line belongs to its nearby vp
        linescorestable(i, j) = -1.0;
      } else {
        linescorestable(i, j) = (angle > angleThreshold) ? 0 : score;
      }
    }

    // classify lines
    lines[i].claz = -1;
    std::vector<double> scores(npoints, 0.0);
    for (int j = 0; j < npoints; j++) {
      scores[j] = linescorestable(i, j);
    }
    int maxClaz =
        std::max_element(scores.begin(), scores.end()) - scores.begin();
    std::sort(scores.begin(), scores.end(), std::greater<>());
    if (scores.front() >= scoreThreshold &&
        (npoints <= 1 || scores.front() - scores[1] >= scoreAdvatangeRatio)) {
      lines[i].claz = maxClaz;
    }
  }

  return linescorestable;
}

std::vector<Line3> MergeLines(const std::vector<Line3> &lines,
                              double angleThres, double mergeAngleThres) {

  assert(angleThres < M_PI_4);
  RTreeMap<Vec3, int> lineNormals;
  std::vector<std::pair<std::vector<int>, Vec3>> groups;
  for (int i = 0; i < lines.size(); i++) {
    auto &line = lines[i];
    auto n = normalize(line.first.cross(line.second));

    int nearestGroupId = -1;
    double minAngle = angleThres;
    lineNormals.search(
        BoundingBox(n).expand(angleThres * 3),
        [&nearestGroupId, &minAngle, &n](const std::pair<Vec3, int> &ln) {
          double angle = AngleBetweenUndirected(ln.first, n);
          if (angle < minAngle) {
            minAngle = angle;
            nearestGroupId = ln.second;
          }
          return true;
        });

    if (nearestGroupId != -1) { // is in some group
      groups[nearestGroupId].first.push_back(i);
    } else { // create a new group
      groups.emplace_back(std::vector<int>{i}, n);
      lineNormals.emplace(n, groups.size() - 1);
    }
  }

  // optimize normals
  auto calcWeightedNormal = [&lines](int lineid) {
    auto &line = lines[lineid];
    return normalize(line.first.cross(line.second)) *
           AngleBetweenDirected(line.first, line.second);
  };
  for (auto &g : groups) {
    auto &lineids = g.first;
    Vec3 nsum = calcWeightedNormal(lineids.front());
    for (int i = 1; i < g.first.size(); i++) {
      Vec3 n = calcWeightedNormal(lineids[i]);
      if (n.dot(nsum) < 0) {
        n = -n;
      }
      nsum += n;
    }
    g.second = normalize(nsum);
  }

  std::vector<Line3> merged;
  merged.reserve(groups.size() * 2);

  // merge the group
  for (auto &g : groups) {
    auto &normal = g.second;
    Vec3 x, y;
    std::tie(x, y) = ProposeXYDirectionsFromZDirection(normal);
    assert(x.cross(y).dot(normal) > 0);

    std::vector<std::pair<double, bool>> angleRangeEnds;
    angleRangeEnds.reserve(g.first.size() * 2);
    for (int i : g.first) {
      auto &line = lines[i];
      auto &n = normalize(line.first.cross(line.second));

      double angleFrom = atan2(line.first.dot(y), line.first.dot(x));
      double angleTo = atan2(line.second.dot(y), line.second.dot(x));
      assert(IsBetween(angleFrom, -M_PI, M_PI) &&
             IsBetween(angleTo, -M_PI, M_PI));

      if (n.dot(normal) < 0) {
        std::swap(angleFrom, angleTo);
      }

      if (angleFrom < angleTo) {
        angleRangeEnds.emplace_back(angleFrom, true);
        angleRangeEnds.emplace_back(angleTo, false);
      } else {
        angleRangeEnds.emplace_back(angleFrom, true);
        angleRangeEnds.emplace_back(angleTo + M_PI * 2, false);
      }
    }

    std::sort(
        angleRangeEnds.begin(), angleRangeEnds.end(),
        [](const std::pair<double, bool> &a, const std::pair<double, bool> &b) {
          return a.first < b.first;
        });

    int occupation = 0;
    bool occupied = false;

    std::vector<double> mergedAngleRanges;
    for (auto &stop : angleRangeEnds) {
      if (stop.second) {
        occupation++;
      } else {
        occupation--;
      }
      assert(occupation >= 0);
      if (!occupied && occupation > 0) {
        mergedAngleRanges.push_back(stop.first);
        occupied = true;
      } else if (occupied && occupation == 0) {
        mergedAngleRanges.push_back(stop.first);
        occupied = false;
      }
    }

    // merge ranges
    if (mergeAngleThres > 0) {
      std::vector<double> mergedAngleRanges2;
      mergedAngleRanges2.reserve(mergedAngleRanges.size());
      for (int i = 0; i < mergedAngleRanges.size(); i += 2) {
        if (mergedAngleRanges2.empty()) {
          mergedAngleRanges2.push_back(mergedAngleRanges[i]);
          mergedAngleRanges2.push_back(mergedAngleRanges[i + 1]);
          continue;
        }
        double &lastEnd = mergedAngleRanges2.back();
        if (mergedAngleRanges[i] >= lastEnd &&
                mergedAngleRanges[i] - lastEnd < mergeAngleThres ||
            mergedAngleRanges[i] < lastEnd &&
                mergedAngleRanges[i] + M_PI * 2 - lastEnd < mergeAngleThres) {
          lastEnd = mergedAngleRanges[i + 1];
        } else {
          mergedAngleRanges2.push_back(mergedAngleRanges[i]);
          mergedAngleRanges2.push_back(mergedAngleRanges[i + 1]);
        }
      }
      mergedAngleRanges = std::move(mergedAngleRanges2);
    }

    assert(mergedAngleRanges.size() % 2 == 0);

    // make lines
    for (int i = 0; i < mergedAngleRanges.size(); i += 2) {
      Vec3 from = x * cos(mergedAngleRanges[i]) + y * sin(mergedAngleRanges[i]);
      Vec3 to =
          x * cos(mergedAngleRanges[i + 1]) + y * sin(mergedAngleRanges[i + 1]);
      merged.emplace_back(from, to);
    }
  }

  return merged;
}

namespace {

std::pair<double, double> ComputeSpanningArea(const Point2 &a, const Point2 &b,
                                              const Ray2 &line) {
  auto ad = SignedDistanceFromPointToLine(a, line);
  auto bd = SignedDistanceFromPointToLine(b, line);
  auto ap = DistanceFromPointToLine(a, line).second;
  auto bp = DistanceFromPointToLine(b, line).second;
  auto len = norm(ap - bp);
  if (ad * bd >= 0) {
    return std::make_pair(len * std::abs(ad + bd) / 2.0, len);
  }
  ad = abs(ad);
  bd = abs(bd);
  return std::make_pair((ad * ad + bd * bd) * len / (ad + bd) / 2.0, len);
}
}

std::pair<double, Ray2>
ComputeStraightness(const std::vector<std::vector<Pixel>> &edges,
                    double *interleavedArea, double *interleavedLen) {

  std::vector<Point<float, 2>> points;
  for (auto &e : edges) {
    for (auto &p : e) {
      points.push_back(Point<float, 2>(p.x, p.y));
    }
  }

  cv::Vec4f line;
  cv::fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);
  auto fittedLine = Ray2({line[2], line[3]}, {line[0], line[1]});
  double interArea = 0;
  double interLen = 0;
  for (auto &e : edges) {
    for (int i = 0; i < e.size() - 1; i++) {
      double area, len;
      std::tie(area, len) = ComputeSpanningArea(
          ecast<double>(e[i]), ecast<double>(e[i + 1]), fittedLine);
      interArea += area;
      interLen += len;
    }
  }

  if (interleavedArea)
    *interleavedArea = interArea;
  if (interleavedLen)
    *interleavedLen = interLen;
  double straightness = Gaussian(interArea / interLen, 1.0);
  if (edges.size() == 1 && edges.front().size() == 2) {
    assert(FuzzyEquals(straightness, 1.0, 0.01) &&
           "simple line should has the best straightness..");
  }

  return std::make_pair(straightness, fittedLine);
}
}
}