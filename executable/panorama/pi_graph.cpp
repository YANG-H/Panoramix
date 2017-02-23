#include "pch.hpp"

#include "algorithms.hpp"
#include "basic_types.hpp"
#include "cameras.hpp"
#include "containers.hpp"
#include "geo_context.hpp"
#include "image.hpp"
#include "line_detection.hpp"
#include "pi_graph.hpp"
#include "segmentation.hpp"
#include "utility.hpp"

#include "canvas.hpp"

#include "pi_graph.hpp"

namespace pano {

namespace experimental {

// AllAlong
template <class ContT>
inline bool AllAlong(const ContT &pts, const Vec3 &from, const Vec3 &to,
                     double angleThres) {
  auto n = normalize(from.cross(to));
  return core::AllOf<Vec3>(pts, [&n, angleThres](const Vec3 &p) {
    return abs(M_PI_2 - AngleBetweenDirected(n, p)) < angleThres;
  });
}

// RectifyPixel
inline bool RectifyPixel(Pixel &p, const PanoramicCamera &cam) {
  if (p.y < 0 || p.y >= cam.screenSize().height) {
    return false;
  }
  p.x = (p.x + cam.screenSize().width) % cam.screenSize().width;
  return true;
}
inline bool RectifyPixel(Pixel &p, const PerspectiveCamera &cam) {
  return p.x >= 0 && p.x < cam.screenSize().width && p.y >= 0 &&
         p.y < cam.screenSize().height;
}

// ForEachNeighborhoodPixel
template <class FunT>
inline void ForEachNeighborhoodPixel(const Pixel &p, int dx1, int dx2, int dy1,
                                     int dy2, const PanoramicCamera &cam,
                                     FunT fun) {
  int w = cam.screenSize().width;
  int h = cam.screenSize().height;
  for (int xx = p.x + dx1; xx <= p.x + dx2; xx++) {
    int x = xx;
    if (x < 0) {
      x += w;
    }
    x = x % w;
    for (int yy = p.y + dy1; yy <= p.y + dy2; yy++) {
      int y = yy;
      if (y < 0 || y >= h) {
        continue;
      }
      fun(Pixel(x, y));
    }
  }
}
template <class FunT>
inline void ForEachNeighborhoodPixel(const Pixel &p, int dx1, int dx2, int dy1,
                                     int dy2, const PerspectiveCamera &cam,
                                     FunT fun) {
  int w = cam.screenSize().width;
  int h = cam.screenSize().height;
  for (int x = p.x + dx1; x <= p.x + dx2; x++) {
    if (x < 0 || x >= w) {
      continue;
    }
    for (int y = p.y + dy1; y <= p.y + dy2; y++) {
      if (y < 0 || y >= h) {
        continue;
      }
      fun(Pixel(x, y));
    }
  }
}

inline double ColorDistance(const Vec3 &a, const Vec3 &b, bool useYUV) {
  static const Mat3 RGB2YUV(0.299, 0.587, 0.114, -0.14713, -0.28886, 0.436,
                            0.615, -0.51499, -0.10001);
  static const Mat3 BGR2VUY = RGB2YUV.t();
  return useYUV ? norm(BGR2VUY * (a - b)) * 3.0 : norm(a - b);
}

// for edge weight computation
// measure pixel distance
inline double PixelDiff(const Image3ub &im, const cv::Point &p1,
                        const cv::Point &p2, bool useYUV) {
  assert(im.depth() == CV_8U && im.channels() == 3);
  auto &c1 = im(p1);
  auto &c2 = im(p2);
  return ColorDistance(c1, c2, useYUV);
}

inline double DistanceAngleFromPointToLine(const Point3 &p, const Line3 &l) {
  auto n = DistanceFromPointToLine(p, l);
  return AngleBetweenDirected(p, n.second.position);
}

double PixelWeight(const PanoramicCamera &cam, const Pixel &p) {
  return cos((p.y - cam.screenSize().height / 2.0) / cam.screenSize().height *
             M_PI);
}
double PixelWeight(const PerspectiveCamera &cam, const Pixel &p) { return 1.0; }

int SegmentationForPIGraph(const PanoramicView &view,
                           const std::vector<Classified<Line3>> &lines,
                           Imagei &segs, double lineExtendAngle, double sigma,
                           double c, double minSize,
                           int widthThresToRemoveThinRegions) {

  Image3ub im = view.image;
  int width = im.cols;
  int height = im.rows;
  Image smoothed;
  if (sigma > 0) {
    cv::GaussianBlur(im, smoothed, cv::Size(5, 5), sigma);
  } else {
    smoothed = im.clone();
  }

  // register lines in RTree
  RTreeMap<Vec3, int> linesRTree;
  static const double lineSampleAngle = DegreesToRadians(2);
  for (int i = 0; i < lines.size(); i++) {
    if (lines[i].claz == -1) { // don't consider clutter lines here
      continue;
    }
    auto &line = lines[i].component;
    double spanAngle = AngleBetweenDirected(line.first, line.second);
    for (double a = 0.0; a <= spanAngle + lineSampleAngle / 2;
         a += lineSampleAngle) {
      auto direction = RotateDirection(line.first, line.second, a);
      linesRTree.emplace(normalize(direction), i);
    }
  }

  // register pixel ind -> spatial direction
  std::vector<Vec3> ind2dir(width * height);
  for (auto it = im.begin(); it != im.end(); ++it) {
    auto p = it.pos();
    Vec3 dir = normalize(view.camera.toSpace(p));
    ind2dir[Sub2Ind(p, width, height)] = dir;
  }

  // pixel graph
  using Vertex = double;
  std::vector<Vertex> vertices(width * height);
  // float radius = width / 2.0 / M_PI;
  for (int y = 0; y < height; y++) {
    // float longitude = (y - height / 2.0f) / radius;
    // float area = cos(longitude);
    // if (area < 1e-9) {
    //  area = 1e-9;
    //}
    for (int x = 0; x < width; x++) {
      Pixel p(x, y);
      auto &v = vertices[Sub2Ind(p, width, height)];
      v = PixelWeight(view.camera, p);
    }
  }
  // collect edges
  struct Edge {
    int ind1, ind2;
    double weight;
  };
  std::vector<Edge> edges;
  edges.reserve(4 * width * height);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Pixel p(x, y);
      Vec3 dir = ind2dir[Sub2Ind(p, width, height)];
      std::set<int> nearbyLines;
      linesRTree.search(BoundingBox(dir).expand(lineSampleAngle * 3),
                        [&nearbyLines](const std::pair<Vec3, int> &lineSample) {
                          nearbyLines.insert(lineSample.second);
                          return true;
                        });

      static const int dx[] = {1, 0, 1, -1};
      static const int dy[] = {0, 1, 1, 1};
      static const double offset[] = {1.0, 1.0, 1.1, 1.1};
      for (int k = 0; k < 4; k++) {
        int xx = x + dx[k];
        int yy = y + dy[k];
        if (yy < 0 || yy >= height) {
          continue;
        }
        xx = (xx + width) % width;
        Pixel p2(xx, yy);
        Vec3 dir2 = ind2dir[Sub2Ind(p2, width, height)];

        bool isSeperatedByLine = false;
        for (int lineid : nearbyLines) {
          auto line = normalize(lines[lineid].component);
          Vec3 normal = normalize(line.first.cross(line.second));
          if (dir.dot(normal) * dir2.dot(normal) < -1e-10 &&
              DistanceAngleFromPointToLine(dir, line) < lineExtendAngle &&
              DistanceAngleFromPointToLine(dir2, line) < lineExtendAngle) {
            isSeperatedByLine = true;
            break;
          }
        }

        if (isSeperatedByLine) {
          continue;
        }

        Edge edge;
        edge.ind1 = Sub2Ind(p, width, height);
        edge.ind2 = Sub2Ind(p2, width, height);
        edge.weight = PixelDiff(smoothed, p, p2, false) * offset[k];
        edges.push_back(edge);
      }
    }
  }
  // add polar pixel connections
  for (int x1 = 0; x1 < width; x1++) {
    for (int x2 = x1 + 1; x2 < width; x2++) {
      for (int y : {0, height - 1}) {
        Pixel p1(x1, y), p2(x2, y);
        Edge edge;
        edge.ind1 = Sub2Ind(p1, width, height);
        edge.ind2 = Sub2Ind(p2, width, height);
        edge.weight = PixelDiff(smoothed, p1, p2, false);
        edges.push_back(edge);
      }
    }
  }

  // segmentation
  std::sort(edges.begin(), edges.end(), [](const Edge &e1, const Edge &e2) {
    return e1.weight < e2.weight;
  });

  std::vector<double> thresholds(vertices.size(), c);
  MergeFindSet<Vertex> mfset(vertices.begin(), vertices.end());
  for (int i = 0; i < edges.size(); i++) {
    const Edge &edge = edges[i];
    int a = mfset.find(edge.ind1);
    int b = mfset.find(edge.ind2);
    if (a == b) {
      continue;
    }
    if (edge.weight <= thresholds[a] && edge.weight <= thresholds[b]) {
      mfset.join(a, b);
      a = mfset.find(a);
      thresholds[a] = edge.weight + c / mfset.data(a);
    }
  }
  while (true) {
    bool merged = false;
    for (int i = 0; i < edges.size(); i++) {
      const Edge &edge = edges[i];
      int a = mfset.find(edge.ind1);
      int b = mfset.find(edge.ind2);
      if (a == b) {
        continue;
      }
      if (mfset.data(a) < minSize || mfset.data(b) < minSize) {
        mfset.join(a, b);
        merged = true;
      }
    }
    if (!merged) {
      break;
    }
  }

  int numCCs = mfset.setsCount();
  std::unordered_map<int, int> compIntSet;
  segs = Imagei(height, width);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = mfset.find(Sub2Ind(Pixel(x, y), width, height));
      if (compIntSet.find(comp) == compIntSet.end()) {
        compIntSet.insert(std::make_pair(comp, (int)compIntSet.size()));
      }
      segs(Pixel(x, y)) = compIntSet[comp];
    }
  }

  bool mergeThinRegions = false;
  if (!mergeThinRegions) {
    return numCCs;
  }

  // now lets remove thin regions
  Imageb insiders(segs.size(), false);
  for (auto it = insiders.begin(); it != insiders.end(); ++it) {
    auto p = it.pos();
    int seg = segs(p);
    bool isInside = true;
    for (int x = -widthThresToRemoveThinRegions;
         x <= widthThresToRemoveThinRegions; x++) {
      if (!isInside) {
        break;
      }
      int xx = p.x + x;
      xx = (xx + width) % width;
      for (int y = -widthThresToRemoveThinRegions;
           y <= widthThresToRemoveThinRegions; y++) {
        if (!isInside) {
          break;
        }
        int yy = p.y + y;
        if (yy < 0 || yy > height - 1) {
          continue;
        }
        if (segs(yy, xx) != seg) {
          isInside = false;
        }
      }
    }
    *it = isInside;
  }

  DenseMatd precompte(widthThresToRemoveThinRegions * 2 + 1,
                      widthThresToRemoveThinRegions * 2 + 1, 0.0);
  for (int x = -widthThresToRemoveThinRegions;
       x <= widthThresToRemoveThinRegions; x++) {
    for (int y = -widthThresToRemoveThinRegions;
         y <= widthThresToRemoveThinRegions; y++) {
      precompte(x + widthThresToRemoveThinRegions,
                y + widthThresToRemoveThinRegions) = sqrt(x * x + y * y);
    }
  }

  std::vector<std::map<int, double>> distanceTable(width * height);
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();
    if (insiders(p)) // consider near-boundary pixels only
      continue;
    auto &dtable = distanceTable[p.x * height + p.y];

    Vec3 dir = ind2dir[Sub2Ind(p, width, height)];
    std::set<int> nearbyLines;
    linesRTree.search(BoundingBox(dir).expand(std::max(
                          lineSampleAngle * 3, widthThresToRemoveThinRegions *
                                                   3 / view.camera.focal())),
                      [&nearbyLines](const std::pair<Vec3, int> &lineSample) {
                        nearbyLines.insert(lineSample.second);
                        return true;
                      });

    for (int x = -widthThresToRemoveThinRegions;
         x <= widthThresToRemoveThinRegions; x++) {
      for (int y = -widthThresToRemoveThinRegions;
           y <= widthThresToRemoveThinRegions; y++) {
        int xx = p.x + x;
        xx = (xx + width) % width;
        int yy = p.y + y;
        if (yy < 0 || yy > height - 1) {
          continue;
        }
        Pixel curp(xx, yy);
        double distance = precompte(x + widthThresToRemoveThinRegions,
                                    y + widthThresToRemoveThinRegions);

        // check whether this is cut by line
        bool isSeperatedByLine = false;
        if (curp != p) {
          Vec3 curDir = ind2dir[Sub2Ind(curp, width, height)];
          for (int lineid : nearbyLines) {
            auto line = normalize(lines[lineid].component);
            Vec3 normal = normalize(line.first.cross(line.second));
            if (dir.dot(normal) * curDir.dot(normal) < -1e-10 &&
                IsBetween(ProjectionOfPointOnLine(dir, line).ratio, 0.0, 1.0) &&
                IsBetween(ProjectionOfPointOnLine(curDir, line).ratio, 0.0,
                          1.0)) {
              isSeperatedByLine = true;
              break;
            }
          }
        }
        if (isSeperatedByLine) {
          continue;
        }

        if (insiders(curp)) { // curp is at certain regions
          int segid = segs(curp);
          if (!Contains(dtable, segid) || dtable.at(segid) > distance) {
            dtable[segid] = distance;
          }
        } else {
          auto &curdtable = distanceTable[curp.x * height + curp.y];
          // use dtable to update curdtable
          for (auto &dd : dtable) {
            if (!Contains(curdtable, dd.first) ||
                curdtable.at(dd.first) > dd.second + distance) {
              curdtable[dd.first] = dd.second + distance;
            }
          }
          // use curdtable to update dtable
          for (auto &dd : curdtable) {
            if (!Contains(dtable, dd.first) ||
                dtable.at(dd.first) > dd.second + distance) {
              dtable[dd.first] = dd.second + distance;
            }
          }
        }
      }
    }
  }

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int id = x * height + y;
      if (distanceTable[id].empty()) {
        continue;
      }
      int minDistSegId = -1;
      double minDist = std::numeric_limits<double>::infinity();
      for (auto &dd : distanceTable[id]) {
        if (dd.second < minDist) {
          minDistSegId = dd.first;
          minDist = dd.second;
        }
      }
      if (minDistSegId == -1) {
        minDistSegId = numCCs++;
      }
      segs(y, x) = minDistSegId;
    }
  }

  return DensifySegmentation(segs, true);
}

#ifdef KKK
PIGraph<PanoramicCamera> BuildPIGraph(
    const PanoramicView &view, const std::vector<Vec3> &vps, int verticalVPId,
    const Imagei &segs, const std::vector<Classified<Line3>> &lines,
    double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
    double bndPieceBoundToLineAngleThres, double intersectionAngleThreshold,
    double incidenceAngleAlongDirectionThreshold,
    double incidenceAngleVerticalDirectionThreshold) {

  static constexpr bool _inPerspectiveMode =
      std::is_same<std::decay_t<decltype(view.camera)>,
                   PerspectiveCamera>::value;
  assert(incidenceAngleVerticalDirectionThreshold >
         bndPieceBoundToLineAngleThres);

  PIGraph<PanoramicCamera> mg;
  mg.view = view;
  int width = view.image.cols;
  int height = view.image.rows;
  assert(vps.size() >= 3);
  mg.vps = std::vector<Vec3>(vps.begin(), vps.begin() + 3);
  mg.verticalVPId = verticalVPId;

  mg.segs = segs;
  int nsegs = MinMaxValOfImage(segs).second + 1;
  mg.nsegs = nsegs;

  // init segs

  mg.seg2areaRatio.resize(nsegs);
  mg.seg2bnds.resize(nsegs);
  mg.seg2center.resize(nsegs);
  mg.seg2control.resize(nsegs);
  mg.seg2linePieces.resize(nsegs);
  mg.seg2contours.resize(nsegs);

  mg.fullArea = 0.0;
  for (auto it = mg.segs.begin(); it != mg.segs.end(); ++it) {
    double weight = PixelWeight(view.camera, it.pos());
    mg.seg2areaRatio[*it] += weight;
    mg.fullArea += weight;
  }
  for (int i = 0; i < nsegs; i++) {
    mg.seg2areaRatio[i] /= mg.fullArea;
    auto &control = mg.seg2control[i];
    control.orientationClaz = control.orientationNotClaz = -1;
    control.used = true;
  }
  for (int i = 0; i < nsegs; i++) {
    Image regionMask = (mg.segs == i);

    // find contour of the region
    std::vector<std::vector<Pixel>> contours;
    cv::findContours(regionMask, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                              // outer contours
    if (contours.empty()) {
      continue;
    }

    Vec3 centerDirection(0, 0, 0);
    std::vector<Vec3> directions;
    directions.reserve(CountOf<Pixel>(contours));
    for (auto &cs : contours) {
      for (auto &c : cs) {
        directions.push_back(normalize(view.camera.toSpace(c)));
        centerDirection += directions.back();
      }
    }
    if (centerDirection != Origin()) {
      centerDirection /= norm(centerDirection);
    }

    if (_inPerspectiveMode) {
      // perspecive mode
      mg.seg2contours[i].resize(contours.size());
      mg.seg2center[i] = centerDirection;
      for (int j = 0; j < contours.size(); j++) {
        auto &cs = mg.seg2contours[i][j];
        cs.reserve(contours[j].size());
        for (auto &p : contours[j]) {
          cs.push_back(normalize(view.camera.toSpace(p)));
        }
      }

    } else {

      // panoramic mode
      // get max angle distance from center direction
      double radiusAngle = 0.0;
      for (auto &d : directions) {
        double a = AngleBetweenDirected(centerDirection, d);
        if (radiusAngle < a) {
          radiusAngle = a;
        }
      }

      // perform a more precise sample !
      int newSampleSize = view.camera.focal() * radiusAngle * 2 + 2;
      PartialPanoramicCamera sCam(
          newSampleSize, newSampleSize, view.camera.focal(), view.camera.eye(),
          centerDirection,
          ProposeXYDirectionsFromZDirection(centerDirection).second);
      Imagei sampledSegmentedRegions =
          MakeCameraSampler(sCam, view.camera)(segs);

      // collect better contours
      contours.clear();
      regionMask = (sampledSegmentedRegions == i);
      cv::findContours(
          regionMask, contours, CV_RETR_EXTERNAL,
          CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                   // outer contours
      std::sort(contours.begin(), contours.end(),
                [](const std::vector<Pixel> &ca, const std::vector<Pixel> &cb) {
                  return ca.size() > cb.size();
                });

      auto iter = std::find_if(
          contours.begin(), contours.end(),
          [](const std::vector<Pixel> &c) { return c.size() <= 2; });
      contours.erase(iter, contours.end());

      mg.seg2contours[i].resize(contours.size());
      mg.seg2center[i] = Origin();
      for (int j = 0; j < contours.size(); j++) {
        auto &cs = mg.seg2contours[i][j];
        cs.reserve(contours[j].size());
        for (auto &p : contours[j]) {
          cs.push_back(normalize(sCam.toSpace(p)));
          mg.seg2center[i] += cs.back();
        }
      }
      if (mg.seg2center[i] != Origin()) {
        mg.seg2center[i] /= norm(mg.seg2center[i]);
      } else {
        mg.seg2center[i] = normalize(centerDirection);
      }
    }
  }

  // init lines
  mg.lines = lines;
  for (auto &l : mg.lines) {
    if (l.claz >= mg.vps.size()) {
      l.claz = -1;
    }
    l.component = normalize(l.component);
  }
  int nlines = lines.size();
  mg.line2linePieces.resize(nlines);
  mg.line2lineRelations.resize(nlines);
  mg.line2used.resize(nlines, true);

  // analyze segs, bnds, juncs ...
  // collect real border pixels
  std::set<Pixel> realBorderPixels;
  if (_inPerspectiveMode) {
    for (int x = 0; x < width; x++) {
      realBorderPixels.insert(Pixel(x, 0));
      realBorderPixels.insert(Pixel(x, height - 1));
    }
    for (int y = 0; y < height; y++) {
      realBorderPixels.insert(Pixel(0, y));
      realBorderPixels.insert(Pixel(width - 1, y));
    }
  }

  std::map<std::set<int>, std::vector<int>> segs2juncs;
  std::vector<std::vector<int>> seg2juncs(nsegs);
  std::map<std::pair<int, int>, std::set<Pixel>> segpair2pixels;

  std::map<Pixel, std::set<int>> pixel2segs;
  std::map<Pixel, int> pixel2junc;

  std::vector<Pixel> juncPositions;
  std::vector<std::vector<int>> junc2segs;

  std::cout << "recording pixels" << std::endl;
  assert(segs.size() == view.camera.screenSize());
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();

    // seg ids related
    // std::set<int> idset = {segs(p), segs(Pixel((p.x + 1) % segs.cols, p.y))};
    // if (p.y <
    //    height - 1) { // note that the top/bottom borders cannot be crossed!
    //  idset.insert(segs(Pixel(p.x, p.y + 1)));
    //  idset.insert(segs(Pixel((p.x + 1) % segs.cols, p.y + 1)));
    //}

    std::set<int> idset;
    ForEachNeighborhoodPixel(
        p, 0, 1, 0, 1, view.camera,
        [&idset, &segs](const Pixel &np) { idset.insert(segs(np)); });

    if (idset.size() <= 1) {
      continue;
    }

    // meet a bnd or a junc (and bnd) pixel
    pixel2segs[p] = std::move(idset);
    auto &relatedsegs = pixel2segs[p];

    // register this pixel as a bnd candidate for bnd of related segids
    for (auto ii = relatedsegs.begin(); ii != relatedsegs.end(); ++ii) {
      for (auto jj = std::next(ii); jj != relatedsegs.end(); ++jj) {
        segpair2pixels[MakeOrderedPair(*ii, *jj)].insert(p);
      }
    }

    // meet a junc
    if (relatedsegs.size() >= 3) {
      // create a new junc
      juncPositions.push_back(p);
      int newjuncid = juncPositions.size() - 1;
      pixel2junc[p] = newjuncid;
      segs2juncs[relatedsegs].push_back(newjuncid);
      for (int segid : relatedsegs) {
        seg2juncs[segid].push_back(newjuncid);
      }
      junc2segs.emplace_back(relatedsegs.begin(), relatedsegs.end());
    }
  }

  // now we have juncs
  mg.junc2positions.resize(juncPositions.size());
  for (int i = 0; i < juncPositions.size(); i++) {
    mg.junc2positions[i] = normalize(view.camera.toSpace(juncPositions[i]));
  }
  mg.junc2bnds.resize(juncPositions.size());

  std::cout << "connecting boundaries" << std::endl;

  // connect different junctions using allbndpixels
  // and thus generate seperated bnds
  std::vector<std::vector<Pixel>> bndPixels;
  for (int i = 0; i < juncPositions.size(); i++) {
    auto &juncpos = juncPositions[i];
    auto &relatedSegIds = junc2segs[i];

    for (int ii = 0; ii < relatedSegIds.size(); ii++) {
      for (int jj = ii + 1; jj < relatedSegIds.size(); jj++) {
        int segi = relatedSegIds[ii];
        int segj = relatedSegIds[jj];

        const auto &pixelsForThisSegPair =
            segpair2pixels.at(MakeOrderedPair(segi, segj));

        std::vector<Pixel> pixelsForThisBnd;
        std::set<Pixel> visitedPixels;

        // use dfs
        int saySegIIsOnLeft = 0, saySegIIsOnRight = 0;
        pixelsForThisBnd.push_back(juncpos);
        visitedPixels.insert(juncpos);

        int lastDirId = 0;
        while (true) {

          auto &curp = pixelsForThisBnd.back();

          // find another junc!
          if (pixelsForThisBnd.size() > 1 && Contains(pixel2junc, curp) &&
              i < pixel2junc.at(curp)) {
            int tojuncid = pixel2junc.at(curp);

            // make a new bnd!
            bndPixels.push_back(std::move(pixelsForThisBnd));
            int newbndid = bndPixels.size() - 1;
            mg.bnd2juncs.emplace_back(i, tojuncid);

            if (saySegIIsOnLeft < saySegIIsOnRight) {
              assert(saySegIIsOnLeft == 0);
              mg.bnd2segs.emplace_back(segj, segi); // left, right
            } else {
              assert(saySegIIsOnRight == 0);
              mg.bnd2segs.emplace_back(segi, segj);
            }
            saySegIIsOnLeft = saySegIIsOnRight = 0;

            mg.junc2bnds[i].push_back(newbndid);
            mg.junc2bnds[tojuncid].push_back(newbndid);

            mg.seg2bnds[segi].push_back(newbndid);
            mg.seg2bnds[segj].push_back(newbndid);

            break;
          }

          // * - * - *
          // | d | a |
          // * -[*]- *
          // | c | b |
          // * - * - *
          static const int dxs[] = {1, 0, -1, 0};
          static const int dys[] = {0, 1, 0, -1};

          static const int leftdxs[] = {1, 1, 0, 0};
          static const int leftdys[] = {0, 1, 1, 0};

          static const int rightdxs[] = {1, 0, 0, 1};
          static const int rightdys[] = {1, 1, 0, 0};

          bool hasMore = false;
          for (int kk = 0; kk < 4; kk++) {
            int k = (lastDirId + kk + 3) % 4;
            auto nextp = curp + Pixel(dxs[k], dys[k]);

            if (!RectifyPixel(nextp, view.camera)) { // note that the top/bottom
                                                     // borders cannot be
                                                     // crossed!
              continue;
            }
            if (!Contains(pixelsForThisSegPair, nextp)) {
              continue;
            }
            if (Contains(visitedPixels, nextp)) {
              continue;
            }

            auto rightp = curp + Pixel(rightdxs[k], rightdys[k]);
            RectifyPixel(rightp, view.camera);
            auto leftp = curp + Pixel(leftdxs[k], leftdys[k]);
            RectifyPixel(leftp, view.camera);
            if (Contains(segs, rightp) && Contains(segs, leftp)) {
              if (segs(rightp) == segi && segs(leftp) == segj) {
                saySegIIsOnRight++;
              } else if (segs(rightp) == segj && segs(leftp) == segi) {
                saySegIIsOnLeft++;
              } else {
                continue;
              }
            }

            pixelsForThisBnd.push_back(nextp);
            visitedPixels.insert(nextp);
            lastDirId = k;
            hasMore = true;
            break;
          }

          if (!hasMore) {
            break;
          }
        }
      }
    }
  }

  // split bnds into pieces
  std::cout << "splitting boundaries" << std::endl;
  std::vector<std::vector<Vec3>> bnd2SmoothedDirs(bndPixels.size());
  const double bndSampleAngle = 1.1 / mg.view.camera.focal();
  for (int i = 0; i < bndPixels.size(); i++) {
    assert(bndPixels[i].size() >= 2);
    auto &smoothedDirs = bnd2SmoothedDirs[i];
    for (int j = 0; j < bndPixels[i].size() - 1; j++) {
      auto &pixel = bndPixels[i][j];
      auto dir = normalize(view.camera.toSpace(pixel));
      if (smoothedDirs.empty() ||
          AngleBetweenDirected(smoothedDirs.back(), dir) >= bndSampleAngle) {
        smoothedDirs.push_back(dir);
      }
    }
    smoothedDirs.push_back(normalize(view.camera.toSpace(bndPixels[i].back())));
    assert(smoothedDirs.size() >= 2);
  }
  mg.bnd2bndPieces.resize(mg.bnd2segs.size());
  for (int i = 0; i < bnd2SmoothedDirs.size(); i++) {
    const auto &dirs = bnd2SmoothedDirs[i];
    assert(dirs.size() > 0);

    std::vector<Vec3> curPiece = {dirs.front()};
    for (int j = 1; j <= dirs.size(); j++) {
      if (j < dirs.size() && AllAlong(curPiece, curPiece.front(), dirs[j],
                                      bndPieceSplitAngleThres)) {
        curPiece.push_back(dirs[j]);
      } else {
        if (curPiece.size() >= 2) {
          mg.bndPiece2dirs.push_back(std::move(curPiece));
          mg.bndPiece2bnd.push_back(i);
          int bndPieceId = mg.bndPiece2dirs.size() - 1;
          mg.bnd2bndPieces[i].push_back(bndPieceId);
          curPiece.clear();
        }
        if (j < dirs.size()) {
          curPiece.push_back(dirs[j]);
        }
      }
    }
  }

  // bndPieces properties
  // classes
  std::cout << "classifying boundary pieces" << std::endl;
  mg.bndPiece2classes.resize(mg.bndPiece2dirs.size(), -1);
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    auto &piece = mg.bndPiece2dirs[i];
    assert(piece.size() > 1);
    Vec3 center = piece[piece.size() / 2];
    mg.bndPiece2classes[i] = -1;
    assert(vps.size() >= 3);
    for (int j = 0; j < 3; j++) {
      if (AllAlong(piece, center, vps[j], bndPieceClassifyAngleThres)) {
        mg.bndPiece2classes[i] = j;
        break;
      }
    }
  }
  // lengths
  mg.bndPiece2length.resize(mg.bndPiece2dirs.size(), 0);
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    auto &piece = mg.bndPiece2dirs[i];
    for (int j = 1; j < piece.size(); j++) {
      mg.bndPiece2length[i] += AngleBetweenDirected(piece[j - 1], piece[j]);
    }
  }
  mg.bndPiece2linePieces.resize(mg.bndPiece2dirs.size());
  mg.bndPiece2segRelation.resize(mg.bndPiece2dirs.size(), SegRelation::Unknown);

  // register bndPiece dirs in RTree
  RTreeMap<Vec3, int> bndPieceRTree;
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    for (auto &d : mg.bndPiece2dirs[i]) {
      bndPieceRTree.emplace(normalize(d), i);
    }
  }
  RTreeMap<Vec3, int> lineRTree;
  const double lineSampleAngle = bndPieceBoundToLineAngleThres / 5.0;
  std::vector<std::vector<Vec3>> lineSamples(mg.lines.size());
  for (int i = 0; i < mg.lines.size(); i++) {
    auto &line = mg.lines[i];
    double angle =
        AngleBetweenDirected(line.component.first, line.component.second);
    for (double a = 0; a <= angle; a += lineSampleAngle) {
      Vec3 sample = normalize(
          RotateDirection(line.component.first, line.component.second, a));
      lineSamples[i].push_back(sample);
      lineRTree.emplace(sample, i);
    }
  }

  // split lines to linePieces
  for (int i = 0; i < mg.lines.size(); i++) {
    Vec3 lineRotNormal = normalize(
        mg.lines[i].component.first.cross(mg.lines[i].component.second));
    auto &samples = lineSamples[i];
    int lastDetectedBndPiece = -1;
    int lastDetectedSeg = -1;
    std::vector<Vec3> collectedSamples;
    for (int j = 0; j <= samples.size(); j++) {
      double minAngleDist = bndPieceBoundToLineAngleThres;
      int nearestBndPiece = -1;
      int nearestSeg = -1;
      if (j < samples.size()) {
        auto &d = samples[j];
        bndPieceRTree.search(
            BoundingBox(d).expand(
                std::max(bndSampleAngle, bndPieceBoundToLineAngleThres) * 3),
            [&d, &minAngleDist,
             &nearestBndPiece](const std::pair<Vec3, int> &bndPieceD) {
              double angleDist = AngleBetweenDirected(bndPieceD.first, d);
              if (angleDist < minAngleDist) {
                minAngleDist = angleDist;
                nearestBndPiece = bndPieceD.second;
              }
              return true;
            });
        auto pixel = ToPixel(view.camera.toScreen(d));
        pixel.x = WrapBetween(pixel.x, 0, width);
        pixel.y = BoundBetween(pixel.y, 0, height - 1);
        nearestSeg = segs(pixel);
      }
      bool neighborChanged =
          (nearestBndPiece != lastDetectedBndPiece ||
           (nearestBndPiece == -1 && lastDetectedSeg != nearestSeg) ||
           j == samples.size() - 1) &&
          !(lastDetectedBndPiece == -1 && lastDetectedSeg == -1);
      if (neighborChanged) {
        double len = AngleBetweenDirected(collectedSamples.front(),
                                            collectedSamples.back());
        if (collectedSamples.size() >= 2) {
          if (lastDetectedBndPiece != -1) { // line piece bound to bnd piece
            mg.linePiece2bndPiece.push_back(lastDetectedBndPiece);
            int linePieceId = mg.linePiece2bndPiece.size() - 1;
            mg.line2linePieces[i].push_back(linePieceId);
            mg.linePiece2line.push_back(i);
            mg.linePiece2samples.push_back(std::move(collectedSamples));
            mg.linePiece2length.push_back(len);
            mg.linePiece2seg.push_back(-1);
            mg.linePiece2segLineRelation.push_back(SegLineRelation::Unknown);
            auto &bndPieceDirs = mg.bndPiece2dirs[lastDetectedBndPiece];
            assert(bndPieceDirs.size() > 1);
            Vec3 bndRotNormal = bndPieceDirs.front().cross(bndPieceDirs.back());
            mg.linePiece2bndPieceInSameDirection.push_back(
                lineRotNormal.dot(bndRotNormal) > 0);
            mg.bndPiece2linePieces[lastDetectedBndPiece].push_back(linePieceId);
          } else { // line piece bound to seg
            mg.linePiece2bndPiece.push_back(-1);
            int linePieceId = mg.linePiece2bndPiece.size() - 1;
            mg.line2linePieces[i].push_back(linePieceId);
            mg.linePiece2line.push_back(i);
            mg.linePiece2samples.push_back(std::move(collectedSamples));
            mg.linePiece2length.push_back(len);
            mg.linePiece2seg.push_back(lastDetectedSeg);
            mg.seg2linePieces[lastDetectedSeg].push_back(linePieceId);
            mg.linePiece2segLineRelation.push_back(SegLineRelation::Unknown);
            mg.linePiece2bndPieceInSameDirection.push_back(true);
          }
        }
        collectedSamples.clear();
      }
      if (j < samples.size()) {
        collectedSamples.push_back(samples[j]);
      }

      lastDetectedBndPiece = nearestBndPiece;
      lastDetectedSeg = nearestSeg;
    }
  }

  // build line relations
  for (int i = 0; i < mg.lines.size(); i++) {
    for (int j = i + 1; j < mg.lines.size(); j++) {
      auto &linei = mg.lines[i].component;
      int clazi = mg.lines[i].claz;
      Vec3 ni = normalize(linei.first.cross(linei.second));
      auto &linej = mg.lines[j].component;
      int clazj = mg.lines[j].claz;
      Vec3 nj = normalize(linej.first.cross(linej.second));

      auto nearest = DistanceBetweenTwoLines(linei, linej);
      double d = AngleBetweenDirected(nearest.second.first.position,
                                        nearest.second.second.position);

      if (clazi == clazj && clazi >= 0) { // incidences for classified lines
        auto conCenter = normalize(nearest.second.first.position +
                                   nearest.second.second.position);
        auto conDir =
            (nearest.second.first.position - nearest.second.second.position);

        auto &vp = vps[clazi];

        if (AngleBetweenDirected(vp, conCenter) < intersectionAngleThreshold)
          continue;

        if (d < incidenceAngleAlongDirectionThreshold &&
            AngleBetweenUndirected(ni, nj) <
                incidenceAngleVerticalDirectionThreshold) {
          // LineRelationData lrd;
          // lrd.type = LineRelationData::Type::Incidence;
          // lrd.normalizedRelationCenter = conCenter;
          // lrd.junctionWeight = 5.0;

          if (HasValue(conCenter, IsInfOrNaN<double>))
            continue;

          mg.lineRelations.push_back(LineRelation::Unknown);
          mg.lineRelation2lines.emplace_back(i, j);
          int lineRelationId = mg.lineRelation2lines.size() - 1;
          mg.line2lineRelations[i].push_back(lineRelationId);
          mg.line2lineRelations[j].push_back(lineRelationId);
          mg.lineRelation2anchor.push_back(conCenter);
          mg.lineRelation2weight.push_back(5.0);
          mg.lineRelation2IsIncidence.push_back(true);

          // mg.addConstraint(std::move(lrd), lhs[i], lhs[j]);
        }

      } else if (clazi != clazj && clazi >= 0 &&
                 clazj >= 0) { // intersections for classified lines
        if (d < intersectionAngleThreshold) {
          auto conCenter = normalize(ni.cross(nj));

          if (Distance(conCenter, linei) > intersectionAngleThreshold * 4 ||
              Distance(conCenter, linej) > intersectionAngleThreshold * 4)
            continue;

          // LineRelationData lrd;
          // lrd.type = LineRelationData::Type::Intersection;
          // lrd.normalizedRelationCenter = conCenter;
          // lrd.junctionWeight = 3.0;

          if (HasValue(conCenter, IsInfOrNaN<double>))
            continue;

          mg.lineRelations.push_back(LineRelation::Unknown);
          mg.lineRelation2lines.emplace_back(i, j);
          int lineRelationId = mg.lineRelation2lines.size() - 1;
          mg.line2lineRelations[i].push_back(lineRelationId);
          mg.line2lineRelations[j].push_back(lineRelationId);
          mg.lineRelation2anchor.push_back(conCenter);
          mg.lineRelation2weight.push_back(3.0);
          mg.lineRelation2IsIncidence.push_back(false);

          // mg.addConstraint(std::move(lrd), lhs[i], lhs[j]);
        }
      }
    }
  }

  // line relation weights
  // compute junction weights
  static const double angleThreshold = M_PI / 32;
  static const double sigma = 0.1;

  enum LineVotingDirection : int {
    TowardsVanishingPoint = 0,
    TowardsOppositeOfVanishingPoint = 1
  };
  enum class JunctionType : int { L, T, Y, W, X };
  for (int i = 0; i < mg.lineRelation2lines.size(); i++) {
    // std::cout << lr.topo.hd.id << std::endl;
    if (mg.lineRelation2IsIncidence[i]) {
      mg.lineRelation2weight[i] = IncidenceJunctionWeight(false);
    } else {
      Mat<float, 3, 2> votingData;
      std::fill(std::begin(votingData), std::end(votingData), 0);

      for (int lineid = 0; lineid < mg.lines.size(); lineid++) {
        auto &line = mg.lines[lineid].component;
        int claz = mg.lines[lineid].claz;
        if (claz == -1 || claz >= 3)
          continue;

        auto &vp = vps[claz];
        Vec3 center = normalize(line.center());

        Vec3 center2vp = normalize(center.cross(vp));
        Vec3 center2pos = normalize(center.cross(mg.lineRelation2anchor[i]));

        double angle = AngleBetweenUndirected(center2vp, center2pos);
        double angleSmall = angle > M_PI_2 ? (M_PI - angle) : angle;
        if (IsInfOrNaN(angleSmall))
          continue;

        assert(angleSmall >= 0 && angleSmall <= M_PI_2);

        double angleScore =
            exp(-(angleSmall / angleThreshold) * (angleSmall / angleThreshold) /
                sigma / sigma / 2);

        auto proj = ProjectionOfPointOnLine(mg.lineRelation2anchor[i], line);
        double projRatio = BoundBetween(proj.ratio, 0.0, 1.0);

        Vec3 lined = line.first.cross(line.second);
        double lineSpanAngle = AngleBetweenDirected(line.first, line.second);
        if (AngleBetweenDirected(center2vp, lined) <
            M_PI_2) { // first-second-vp
          votingData(claz, TowardsVanishingPoint) +=
              angleScore * lineSpanAngle * (1 - projRatio);
          votingData(claz, TowardsOppositeOfVanishingPoint) +=
              angleScore * lineSpanAngle * projRatio;
        } else { // vp-first-second
          votingData(claz, TowardsOppositeOfVanishingPoint) +=
              angleScore * lineSpanAngle * (1 - projRatio);
          votingData(claz, TowardsVanishingPoint) +=
              angleScore * lineSpanAngle * projRatio;
        }
      }
      mg.lineRelation2weight[i] = std::max(
          0.1f, ComputeIntersectionJunctionWeightWithLinesVotes(votingData));
    }
  }

  // normalize all directions
  for (auto &ds : mg.bndPiece2dirs) {
    for (auto &d : ds) {
      d = normalize(d);
    }
  }
  for (auto &d : mg.junc2positions) {
    d = normalize(d);
  }
  for (auto &ds : mg.linePiece2samples) {
    for (auto &d : ds) {
      d = normalize(d);
    }
  }
  for (auto &d : mg.lineRelation2anchor) {
    d = normalize(d);
  }
  for (auto &l : mg.lines) {
    l.component = normalize(l.component);
  }
  for (auto &d : mg.seg2center) {
    d = normalize(d);
  }
  for (auto &dss : mg.seg2contours) {
    for (auto &ds : dss) {
      for (auto &d : ds) {
        d = normalize(d);
      }
    }
  }
  for (auto &vp : mg.vps) {
    vp = normalize(vp);
  }

  return mg;
}

#endif

PIGraph<PanoramicCamera> BuildPIGraph(
    const PanoramicView &view, const std::vector<Vec3> &vps, int verticalVPId,
    const Imagei &segs, const std::vector<Classified<Line3>> &lines,
    double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
    double bndPieceBoundToLineAngleThres, double intersectionAngleThreshold,
    double incidenceAngleAlongDirectionThreshold,
    double incidenceAngleVerticalDirectionThreshold) {

  static constexpr bool _inPerspectiveMode =
      std::is_same<std::decay_t<decltype(view.camera)>,
                   PerspectiveCamera>::value;
  assert(incidenceAngleVerticalDirectionThreshold >
         bndPieceBoundToLineAngleThres);

  PIGraph<PanoramicCamera> mg;
  mg.view = view;
  int width = view.image.cols;
  int height = view.image.rows;
  assert(vps.size() >= 3);
  mg.vps = std::vector<Vec3>(vps.begin(), vps.begin() + 3);
  mg.verticalVPId = verticalVPId;

  mg.segs = segs;
  int nsegs = MinMaxValOfImage(segs).second + 1;
  mg.nsegs = nsegs;

  // init segs

  mg.seg2areaRatio.resize(nsegs);
  mg.seg2bnds.resize(nsegs);
  mg.seg2center.resize(nsegs);
  mg.seg2control.resize(nsegs);
  mg.seg2linePieces.resize(nsegs);
  mg.seg2contours.resize(nsegs);

  mg.fullArea = 0.0;
  for (auto it = mg.segs.begin(); it != mg.segs.end(); ++it) {
    double weight = PixelWeight(view.camera, it.pos());
    mg.seg2areaRatio[*it] += weight;
    mg.fullArea += weight;
  }
  for (int i = 0; i < nsegs; i++) {
    mg.seg2areaRatio[i] /= mg.fullArea;
    auto &control = mg.seg2control[i];
    control.orientationClaz = control.orientationNotClaz = -1;
    control.used = true;
  }
  for (int i = 0; i < nsegs; i++) {
    Image regionMask = (mg.segs == i);

    // find contour of the region
    std::vector<std::vector<Pixel>> contours;
    cv::findContours(regionMask, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                              // outer contours
    if (contours.empty()) {
      continue;
    }

    Vec3 centerDirection(0, 0, 0);
    std::vector<Vec3> directions;
    directions.reserve(CountOf<Pixel>(contours));
    for (auto &cs : contours) {
      for (auto &c : cs) {
        directions.push_back(normalize(view.camera.toSpace(c)));
        centerDirection += directions.back();
      }
    }
    if (centerDirection != Origin()) {
      centerDirection /= norm(centerDirection);
    }

    if (_inPerspectiveMode) {
      // perspecive mode
      mg.seg2contours[i].resize(contours.size());
      mg.seg2center[i] = centerDirection;
      for (int j = 0; j < contours.size(); j++) {
        auto &cs = mg.seg2contours[i][j];
        cs.reserve(contours[j].size());
        for (auto &p : contours[j]) {
          cs.push_back(normalize(view.camera.toSpace(p)));
        }
      }

    } else {

      // panoramic mode
      // get max angle distance from center direction
      double radiusAngle = 0.0;
      for (auto &d : directions) {
        double a = AngleBetweenDirected(centerDirection, d);
        if (radiusAngle < a) {
          radiusAngle = a;
        }
      }

      // perform a more precise sample !
      int newSampleSize = view.camera.focal() * radiusAngle * 2 + 2;
      PartialPanoramicCamera sCam(
          newSampleSize, newSampleSize, view.camera.focal(), view.camera.eye(),
          centerDirection,
          ProposeXYDirectionsFromZDirection(centerDirection).second);
      Imagei sampledSegmentedRegions =
          MakeCameraSampler(sCam, view.camera)(segs);

      // collect better contours
      contours.clear();
      regionMask = (sampledSegmentedRegions == i);
      cv::findContours(
          regionMask, contours, CV_RETR_EXTERNAL,
          CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                   // outer contours
      std::sort(contours.begin(), contours.end(),
                [](const std::vector<Pixel> &ca, const std::vector<Pixel> &cb) {
                  return ca.size() > cb.size();
                });

      auto iter = std::find_if(
          contours.begin(), contours.end(),
          [](const std::vector<Pixel> &c) { return c.size() <= 2; });
      contours.erase(iter, contours.end());

      mg.seg2contours[i].resize(contours.size());
      mg.seg2center[i] = Origin();
      for (int j = 0; j < contours.size(); j++) {
        auto &cs = mg.seg2contours[i][j];
        cs.reserve(contours[j].size());
        for (auto &p : contours[j]) {
          cs.push_back(normalize(sCam.toSpace(p)));
          mg.seg2center[i] += cs.back();
        }
      }
      if (mg.seg2center[i] != Origin()) {
        mg.seg2center[i] /= norm(mg.seg2center[i]);
      } else {
        mg.seg2center[i] = normalize(centerDirection);
      }
    }
  }

  // init lines
  mg.lines = lines;
  for (auto &l : mg.lines) {
    if (l.claz >= mg.vps.size()) {
      l.claz = -1;
    }
    l.component = normalize(l.component);
  }
  int nlines = lines.size();
  mg.line2linePieces.resize(nlines);
  mg.line2lineRelations.resize(nlines);
  mg.line2used.resize(nlines, true);

  // analyze segs, bnds, juncs ...
  // collect real border pixels
  std::set<Pixel> realBorderPixels;
  if (_inPerspectiveMode) {
    for (int x = 0; x < width; x++) {
      realBorderPixels.insert(Pixel(x, 0));
      realBorderPixels.insert(Pixel(x, height - 1));
    }
    for (int y = 0; y < height; y++) {
      realBorderPixels.insert(Pixel(0, y));
      realBorderPixels.insert(Pixel(width - 1, y));
    }
  }

  std::map<std::set<int>, std::vector<int>> segs2juncs;
  std::vector<std::vector<int>> seg2juncs(nsegs);
  std::map<std::pair<int, int>, std::set<Pixel>> segpair2pixels;

  std::map<Pixel, std::set<int>> pixel2segs;
  std::map<Pixel, int> pixel2junc;

  std::vector<Pixel> juncPositions;
  std::vector<std::vector<int>> junc2segs;

  std::cout << "recording pixels" << std::endl;
  assert(segs.size() == view.camera.screenSize());
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();

    std::set<int> idset;
    ForEachNeighborhoodPixel(
        p, 0, 1, 0, 1, view.camera,
        [&idset, &segs](const Pixel &np) { idset.insert(segs(np)); });

    if (idset.size() <= 1) {
      continue;
    }

    // meet a bnd or a junc (and bnd) pixel
    pixel2segs[p] = std::move(idset);
    auto &relatedsegs = pixel2segs[p];

    // register this pixel as a bnd candidate for bnd of related segids
    for (auto ii = relatedsegs.begin(); ii != relatedsegs.end(); ++ii) {
      for (auto jj = std::next(ii); jj != relatedsegs.end(); ++jj) {
        segpair2pixels[MakeOrderedPair(*ii, *jj)].insert(p);
      }
    }

    // meet a junc
    if (relatedsegs.size() >= 3) {
      // create a new junc
      juncPositions.push_back(p);
      int newjuncid = juncPositions.size() - 1;
      pixel2junc[p] = newjuncid;
      segs2juncs[relatedsegs].push_back(newjuncid);
      for (int segid : relatedsegs) {
        seg2juncs[segid].push_back(newjuncid);
      }
      junc2segs.emplace_back(relatedsegs.begin(), relatedsegs.end());
    }
  }

  // now we have juncs
  mg.junc2positions.resize(juncPositions.size());
  for (int i = 0; i < juncPositions.size(); i++) {
    mg.junc2positions[i] = normalize(view.camera.toSpace(juncPositions[i]));
  }
  mg.junc2bnds.resize(juncPositions.size());

  std::cout << "connecting boundaries" << std::endl;

  // connect different junctions using allbndpixels
  // and thus generate seperated bnds
  std::vector<std::vector<Pixel>> bndPixels;
  for (int i = 0; i < juncPositions.size(); i++) {
    auto &juncpos = juncPositions[i];
    auto &relatedSegIds = junc2segs[i];

    for (int ii = 0; ii < relatedSegIds.size(); ii++) {
      for (int jj = ii + 1; jj < relatedSegIds.size(); jj++) {
        int segi = relatedSegIds[ii];
        int segj = relatedSegIds[jj];

        const auto &pixelsForThisSegPair =
            segpair2pixels.at(MakeOrderedPair(segi, segj));

        std::vector<Pixel> pixelsForThisBnd;
        std::set<Pixel> visitedPixels;

        // use dfs
        int saySegIIsOnLeft = 0, saySegIIsOnRight = 0;
        pixelsForThisBnd.push_back(juncpos);
        visitedPixels.insert(juncpos);

        int lastDirId = 0;
        while (true) {

          auto &curp = pixelsForThisBnd.back();

          // find another junc!
          if (pixelsForThisBnd.size() > 1 && Contains(pixel2junc, curp) &&
              i < pixel2junc.at(curp)) {
            int tojuncid = pixel2junc.at(curp);

            // make a new bnd!
            bndPixels.push_back(std::move(pixelsForThisBnd));
            int newbndid = bndPixels.size() - 1;
            mg.bnd2juncs.emplace_back(i, tojuncid);

            if (saySegIIsOnLeft < saySegIIsOnRight) {
              assert(saySegIIsOnLeft == 0);
              mg.bnd2segs.emplace_back(segj, segi); // left, right
            } else {
              assert(saySegIIsOnRight == 0);
              mg.bnd2segs.emplace_back(segi, segj);
            }
            saySegIIsOnLeft = saySegIIsOnRight = 0;

            mg.junc2bnds[i].push_back(newbndid);
            mg.junc2bnds[tojuncid].push_back(newbndid);

            mg.seg2bnds[segi].push_back(newbndid);
            mg.seg2bnds[segj].push_back(newbndid);

            break;
          }

          // * - * - *
          // | d | a |
          // * -[*]- *
          // | c | b |
          // * - * - *
          static const int dxs[] = {1, 0, -1, 0};
          static const int dys[] = {0, 1, 0, -1};

          static const int leftdxs[] = {1, 1, 0, 0};
          static const int leftdys[] = {0, 1, 1, 0};

          static const int rightdxs[] = {1, 0, 0, 1};
          static const int rightdys[] = {1, 1, 0, 0};

          bool hasMore = false;
          for (int kk = 0; kk < 4; kk++) {
            int k = (lastDirId + kk + 3) % 4;
            auto nextp = curp + Pixel(dxs[k], dys[k]);

            if (!RectifyPixel(nextp, view.camera)) { // note that the top/bottom
                                                     // borders cannot be
                                                     // crossed!
              continue;
            }
            if (!Contains(pixelsForThisSegPair, nextp)) {
              continue;
            }
            if (Contains(visitedPixels, nextp)) {
              continue;
            }

            auto rightp = curp + Pixel(rightdxs[k], rightdys[k]);
            RectifyPixel(rightp, view.camera);
            auto leftp = curp + Pixel(leftdxs[k], leftdys[k]);
            RectifyPixel(leftp, view.camera);
            if (Contains(segs.size(), rightp) && Contains(segs.size(), leftp)) {
              if (segs(rightp) == segi && segs(leftp) == segj) {
                saySegIIsOnRight++;
              } else if (segs(rightp) == segj && segs(leftp) == segi) {
                saySegIIsOnLeft++;
              } else {
                continue;
              }
            }

            pixelsForThisBnd.push_back(nextp);
            visitedPixels.insert(nextp);
            lastDirId = k;
            hasMore = true;
            break;
          }

          if (!hasMore) {
            break;
          }
        }
      }
    }
  }

  // split bnds into pieces
  std::cout << "splitting boundaries" << std::endl;
  std::vector<std::vector<Vec3>> bnd2SmoothedDirs(bndPixels.size());
  const double bndSampleAngle = 1.1 / mg.view.camera.focal();
  for (int i = 0; i < bndPixels.size(); i++) {
    assert(bndPixels[i].size() >= 2);
    auto &smoothedDirs = bnd2SmoothedDirs[i];
    for (int j = 0; j < bndPixels[i].size() - 1; j++) {
      auto &pixel = bndPixels[i][j];
      auto dir = normalize(view.camera.toSpace(pixel));
      if (smoothedDirs.empty() ||
          AngleBetweenDirected(smoothedDirs.back(), dir) >= bndSampleAngle) {
        smoothedDirs.push_back(dir);
      }
    }
    smoothedDirs.push_back(normalize(view.camera.toSpace(bndPixels[i].back())));
    assert(smoothedDirs.size() >= 2);
  }
  mg.bnd2bndPieces.resize(mg.bnd2segs.size());
  for (int i = 0; i < bnd2SmoothedDirs.size(); i++) {
    const auto &dirs = bnd2SmoothedDirs[i];
    assert(dirs.size() > 0);

    std::vector<Vec3> curPiece = {dirs.front()};
    for (int j = 1; j <= dirs.size(); j++) {
      if (j < dirs.size() && AllAlong(curPiece, curPiece.front(), dirs[j],
                                      bndPieceSplitAngleThres)) {
        curPiece.push_back(dirs[j]);
      } else {
        if (curPiece.size() >= 2) {
          mg.bndPiece2dirs.push_back(std::move(curPiece));
          mg.bndPiece2bnd.push_back(i);
          int bndPieceId = mg.bndPiece2dirs.size() - 1;
          mg.bnd2bndPieces[i].push_back(bndPieceId);
          curPiece.clear();
        }
        if (j < dirs.size()) {
          curPiece.push_back(dirs[j]);
        }
      }
    }
  }

  // bndPieces properties
  // classes
  std::cout << "classifying boundary pieces" << std::endl;
  mg.bndPiece2classes.resize(mg.bndPiece2dirs.size(), -1);
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    auto &piece = mg.bndPiece2dirs[i];
    assert(piece.size() > 1);
    Vec3 center = piece[piece.size() / 2];
    mg.bndPiece2classes[i] = -1;
    assert(vps.size() >= 3);
    for (int j = 0; j < 3; j++) {
      if (AllAlong(piece, center, vps[j], bndPieceClassifyAngleThres)) {
        mg.bndPiece2classes[i] = j;
        break;
      }
    }
  }
  // lengths
  mg.bndPiece2length.resize(mg.bndPiece2dirs.size(), 0);
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    auto &piece = mg.bndPiece2dirs[i];
    for (int j = 1; j < piece.size(); j++) {
      mg.bndPiece2length[i] += AngleBetweenDirected(piece[j - 1], piece[j]);
    }
  }
  mg.bndPiece2linePieces.resize(mg.bndPiece2dirs.size());
  mg.bndPiece2segRelation.resize(mg.bndPiece2dirs.size(), SegRelation::Unknown);

  // register bndPiece dirs in RTree
  RTreeMap<Vec3, int> bndPieceRTree;
  for (int i = 0; i < mg.bndPiece2dirs.size(); i++) {
    for (auto &d : mg.bndPiece2dirs[i]) {
      bndPieceRTree.emplace(normalize(d), i);
    }
  }
  RTreeMap<Vec3, int> lineRTree;
  const double lineSampleAngle = bndPieceBoundToLineAngleThres / 5.0;
  std::vector<std::vector<Vec3>> lineSamples(mg.lines.size());
  for (int i = 0; i < mg.lines.size(); i++) {
    auto &line = mg.lines[i];
    double angle =
        AngleBetweenDirected(line.component.first, line.component.second);
    for (double a = 0; a <= angle; a += lineSampleAngle) {
      Vec3 sample = normalize(
          RotateDirection(line.component.first, line.component.second, a));
      lineSamples[i].push_back(sample);
      lineRTree.emplace(sample, i);
    }
  }

  // split lines to linePieces
  for (int i = 0; i < mg.lines.size(); i++) {
    Vec3 lineRotNormal = normalize(
        mg.lines[i].component.first.cross(mg.lines[i].component.second));
    auto &samples = lineSamples[i];
    int lastDetectedBndPiece = -1;
    int lastDetectedSeg = -1;
    std::vector<Vec3> collectedSamples;
    for (int j = 0; j <= samples.size(); j++) {
      double minAngleDist = bndPieceBoundToLineAngleThres;
      int nearestBndPiece = -1;
      int nearestSeg = -1;
      if (j < samples.size()) {
        auto &d = samples[j];
        bndPieceRTree.search(
            BoundingBox(d).expand(
                std::max(bndSampleAngle, bndPieceBoundToLineAngleThres) * 3),
            [&d, &minAngleDist,
             &nearestBndPiece](const std::pair<Vec3, int> &bndPieceD) {
              double angleDist = AngleBetweenDirected(bndPieceD.first, d);
              if (angleDist < minAngleDist) {
                minAngleDist = angleDist;
                nearestBndPiece = bndPieceD.second;
              }
              return true;
            });
        auto pixel = ToPixel(view.camera.toScreen(d));
        pixel.x = WrapBetween(pixel.x, 0, width);
        pixel.y = BoundBetween(pixel.y, 0, height - 1);
        nearestSeg = segs(pixel);
      }
      bool neighborChanged =
          (nearestBndPiece != lastDetectedBndPiece ||
           (nearestBndPiece == -1 && lastDetectedSeg != nearestSeg) ||
           j == samples.size() - 1) &&
          !(lastDetectedBndPiece == -1 && lastDetectedSeg == -1);
      if (neighborChanged) {
        double len = AngleBetweenDirected(collectedSamples.front(),
                                            collectedSamples.back());
        if (collectedSamples.size() >= 2) {
          if (lastDetectedBndPiece != -1) { // line piece bound to bnd piece
            mg.linePiece2bndPiece.push_back(lastDetectedBndPiece);
            int linePieceId = mg.linePiece2bndPiece.size() - 1;
            mg.line2linePieces[i].push_back(linePieceId);
            mg.linePiece2line.push_back(i);
            mg.linePiece2samples.push_back(std::move(collectedSamples));
            mg.linePiece2length.push_back(len);
            mg.linePiece2seg.push_back(-1);
            mg.linePiece2segLineRelation.push_back(SegLineRelation::Unknown);
            auto &bndPieceDirs = mg.bndPiece2dirs[lastDetectedBndPiece];
            assert(bndPieceDirs.size() > 1);
            Vec3 bndRotNormal = bndPieceDirs.front().cross(bndPieceDirs.back());
            mg.linePiece2bndPieceInSameDirection.push_back(
                lineRotNormal.dot(bndRotNormal) > 0);
            mg.bndPiece2linePieces[lastDetectedBndPiece].push_back(linePieceId);
          } else { // line piece bound to seg
            mg.linePiece2bndPiece.push_back(-1);
            int linePieceId = mg.linePiece2bndPiece.size() - 1;
            mg.line2linePieces[i].push_back(linePieceId);
            mg.linePiece2line.push_back(i);
            mg.linePiece2samples.push_back(std::move(collectedSamples));
            mg.linePiece2length.push_back(len);
            mg.linePiece2seg.push_back(lastDetectedSeg);
            mg.seg2linePieces[lastDetectedSeg].push_back(linePieceId);
            mg.linePiece2segLineRelation.push_back(SegLineRelation::Unknown);
            mg.linePiece2bndPieceInSameDirection.push_back(true);
          }
        }
        collectedSamples.clear();
      }
      if (j < samples.size()) {
        collectedSamples.push_back(samples[j]);
      }

      lastDetectedBndPiece = nearestBndPiece;
      lastDetectedSeg = nearestSeg;
    }
  }

  // build line relations
  for (int i = 0; i < mg.lines.size(); i++) {
    for (int j = i + 1; j < mg.lines.size(); j++) {
      auto &linei = mg.lines[i].component;
      int clazi = mg.lines[i].claz;
      Vec3 ni = normalize(linei.first.cross(linei.second));
      auto &linej = mg.lines[j].component;
      int clazj = mg.lines[j].claz;
      Vec3 nj = normalize(linej.first.cross(linej.second));

      auto nearest = DistanceBetweenTwoLines(linei, linej);
      double d = AngleBetweenDirected(nearest.second.first.position,
                                        nearest.second.second.position);

      if (clazi == clazj && clazi >= 0) { // incidences for classified lines
        auto conCenter = normalize(nearest.second.first.position +
                                   nearest.second.second.position);
        auto conDir =
            (nearest.second.first.position - nearest.second.second.position);

        auto &vp = vps[clazi];

        if (AngleBetweenDirected(vp, conCenter) < intersectionAngleThreshold)
          continue;

        if (d < incidenceAngleAlongDirectionThreshold &&
            AngleBetweenUndirected(ni, nj) <
                incidenceAngleVerticalDirectionThreshold) {
          // LineRelationData lrd;
          // lrd.type = LineRelationData::Type::Incidence;
          // lrd.normalizedRelationCenter = conCenter;
          // lrd.junctionWeight = 5.0;

          if (HasValue(conCenter, IsInfOrNaN<double>))
            continue;

          mg.lineRelations.push_back(LineRelation::Unknown);
          mg.lineRelation2lines.emplace_back(i, j);
          int lineRelationId = mg.lineRelation2lines.size() - 1;
          mg.line2lineRelations[i].push_back(lineRelationId);
          mg.line2lineRelations[j].push_back(lineRelationId);
          mg.lineRelation2anchor.push_back(conCenter);
          mg.lineRelation2weight.push_back(5.0);
          mg.lineRelation2IsIncidence.push_back(true);

          // mg.addConstraint(std::move(lrd), lhs[i], lhs[j]);
        }

      } else if (clazi != clazj && clazi >= 0 &&
                 clazj >= 0) { // intersections for classified lines
        if (d < intersectionAngleThreshold) {
          auto conCenter = normalize(ni.cross(nj));

          if (Distance(conCenter, linei) > intersectionAngleThreshold * 4 ||
              Distance(conCenter, linej) > intersectionAngleThreshold * 4)
            continue;

          // LineRelationData lrd;
          // lrd.type = LineRelationData::Type::Intersection;
          // lrd.normalizedRelationCenter = conCenter;
          // lrd.junctionWeight = 3.0;

          if (HasValue(conCenter, IsInfOrNaN<double>))
            continue;

          mg.lineRelations.push_back(LineRelation::Unknown);
          mg.lineRelation2lines.emplace_back(i, j);
          int lineRelationId = mg.lineRelation2lines.size() - 1;
          mg.line2lineRelations[i].push_back(lineRelationId);
          mg.line2lineRelations[j].push_back(lineRelationId);
          mg.lineRelation2anchor.push_back(conCenter);
          mg.lineRelation2weight.push_back(3.0);
          mg.lineRelation2IsIncidence.push_back(false);

          // mg.addConstraint(std::move(lrd), lhs[i], lhs[j]);
        }
      }
    }
  }

  // line relation weights
  // compute junction weights
  static const double angleThreshold = M_PI / 32;
  static const double sigma = 0.1;

  enum LineVotingDirection : int {
    TowardsVanishingPoint = 0,
    TowardsOppositeOfVanishingPoint = 1
  };
  enum class JunctionType : int { L, T, Y, W, X };
  for (int i = 0; i < mg.lineRelation2lines.size(); i++) {
    // std::cout << lr.topo.hd.id << std::endl;
    if (mg.lineRelation2IsIncidence[i]) {
      mg.lineRelation2weight[i] = IncidenceJunctionWeight(false);
    } else {
      Mat<float, 3, 2> votingData;
      std::fill(std::begin(votingData), std::end(votingData), 0);

      for (int lineid = 0; lineid < mg.lines.size(); lineid++) {
        auto &line = mg.lines[lineid].component;
        int claz = mg.lines[lineid].claz;
        if (claz == -1 || claz >= 3)
          continue;

        auto &vp = vps[claz];
        Vec3 center = normalize(line.center());

        Vec3 center2vp = normalize(center.cross(vp));
        Vec3 center2pos = normalize(center.cross(mg.lineRelation2anchor[i]));

        double angle = AngleBetweenUndirected(center2vp, center2pos);
        double angleSmall = angle > M_PI_2 ? (M_PI - angle) : angle;
        if (IsInfOrNaN(angleSmall))
          continue;

        assert(angleSmall >= 0 && angleSmall <= M_PI_2);

        double angleScore =
            exp(-(angleSmall / angleThreshold) * (angleSmall / angleThreshold) /
                sigma / sigma / 2);

        auto proj = ProjectionOfPointOnLine(mg.lineRelation2anchor[i], line);
        double projRatio = BoundBetween(proj.ratio, 0.0, 1.0);

        Vec3 lined = line.first.cross(line.second);
        double lineSpanAngle = AngleBetweenDirected(line.first, line.second);
        if (AngleBetweenDirected(center2vp, lined) <
            M_PI_2) { // first-second-vp
          votingData(claz, TowardsVanishingPoint) +=
              angleScore * lineSpanAngle * (1 - projRatio);
          votingData(claz, TowardsOppositeOfVanishingPoint) +=
              angleScore * lineSpanAngle * projRatio;
        } else { // vp-first-second
          votingData(claz, TowardsOppositeOfVanishingPoint) +=
              angleScore * lineSpanAngle * (1 - projRatio);
          votingData(claz, TowardsVanishingPoint) +=
              angleScore * lineSpanAngle * projRatio;
        }
      }
      mg.lineRelation2weight[i] = std::max(
          0.1f, ComputeIntersectionJunctionWeightWithLinesVotes(votingData));
    }
  }

  // normalize all directions
  for (auto &ds : mg.bndPiece2dirs) {
    for (auto &d : ds) {
      d = normalize(d);
    }
  }
  for (auto &d : mg.junc2positions) {
    d = normalize(d);
  }
  for (auto &ds : mg.linePiece2samples) {
    for (auto &d : ds) {
      d = normalize(d);
    }
  }
  for (auto &d : mg.lineRelation2anchor) {
    d = normalize(d);
  }
  for (auto &l : mg.lines) {
    l.component = normalize(l.component);
  }
  for (auto &d : mg.seg2center) {
    d = normalize(d);
  }
  for (auto &dss : mg.seg2contours) {
    for (auto &ds : dss) {
      for (auto &d : ds) {
        d = normalize(d);
      }
    }
  }
  for (auto &vp : mg.vps) {
    vp = normalize(vp);
  }

  return mg;
}



//PIGraph<PanoramicCamera>
//BuildPIGraph(const PanoramicView &view, const std::vector<Vec3> &vps,
//             int verticalVPId, const std::vector<Classified<Line3>> &lines,
//             double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
//             double bndPieceBoundToLineAngleThres,
//             double intersectionAngleThreshold,
//             double incidenceAngleAlongDirectionThreshold,
//             double incidenceAngleVerticalDirectionThreshold) {
//
//  Imagei segs;
//  int nsegs = SegmentationForPIGraph(view, lines, segs, DegreesToRadians(5));
//
//  RemoveThinRegionInSegmentation(segs, 1, true);
//  RemoveSmallRegionInSegmentation(segs, 100, true);
//  RemoveDanglingPixelsInSegmentation(segs, true);
//  nsegs = DensifySegmentation(segs, true);
//
//  return BuildPIGraph(view, vps, verticalVPId, segs, lines,
//                      bndPieceSplitAngleThres, bndPieceClassifyAngleThres,
//                      bndPieceBoundToLineAngleThres, intersectionAngleThreshold,
//                      incidenceAngleAlongDirectionThreshold,
//                      incidenceAngleVerticalDirectionThreshold);
//}

PIGraph<PerspectiveCamera> BuildPIGraph(
    const PerspectiveView &view, const std::vector<Vec3> &vps, int verticalVPId,
    const Imagei &segs, const std::vector<Classified<Line3>> &lines,
    double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
    double bndPieceBoundToLineAngleThres, double intersectionAngleThreshold,
    double incidenceAngleAlongDirectionThreshold,
    double incidenceAngleVerticalDirectionThreshold) {


    // todo

  return PIGraph<PerspectiveCamera>();
}







// PerfectSegMaskView
namespace details {
template <class CameraT>
View<PartialPanoramicCamera, Imageub>
PerfectSegMaskViewImpl(const PIGraph<CameraT> &mg, int seg, double focal) {
  auto &contours = mg.seg2contours[seg];
  double radiusAngle = 0.0;
  for (auto &cs : contours) {
    for (auto &c : cs) {
      double angle = AngleBetweenDirected(mg.seg2center[seg], c);
      if (angle > radiusAngle) {
        radiusAngle = angle;
      }
    }
  }
  int ppcSize = std::ceil(2 * radiusAngle * focal);
  Vec3 x;
  std::tie(x, std::ignore) =
      ProposeXYDirectionsFromZDirection(mg.seg2center[seg]);
  PartialPanoramicCamera ppc(ppcSize, ppcSize, focal, Point3(0, 0, 0),
                             mg.seg2center[seg], x);
  Imageub mask = Imageub::zeros(ppc.screenSize());

  // project contours to ppc
  std::vector<std::vector<Point2i>> contourProjs(contours.size());
  for (int k = 0; k < contours.size(); k++) {
    auto &contourProj = contourProjs[k];
    contourProj.reserve(contours[k].size());
    for (auto &d : contours[k]) {
      contourProj.push_back(ecast<int>(ppc.toScreen(d)));
    }
  }
  cv::fillPoly(mask, contourProjs, (uint8_t)1);
  return View<PartialPanoramicCamera, Imageub>{mask, ppc};
}
}

View<PartialPanoramicCamera, Imageub>
PerfectSegMaskView(const PIGraph<PanoramicCamera> &mg, int seg, double focal) {
  return details::PerfectSegMaskViewImpl(mg, seg, focal);
}

View<PartialPanoramicCamera, Imageub>
PerfectSegMaskView(const PIGraph<PerspectiveCamera> &mg, int seg,
                   double focal) {
  return details::PerfectSegMaskViewImpl(mg, seg, focal);
}

// Junction weights
float IncidenceJunctionWeight(bool acrossViews) {
  return acrossViews ? 10.0f : 7.0f;
}
float OutsiderIntersectionJunctionWeight() { return 2.0f; }
float ComputeIntersectionJunctionWeightWithLinesVotes(
    const Mat<float, 3, 2> &v) {
  double junctionWeight = 0.0;
  // Y
  double Y = 0.0;
  for (int s = 0; s < 2; s++) {
    Y += v(0, s) * v(1, s) * v(2, s) *
         DiracDelta(v(0, 1 - s) + v(1, 1 - s) + v(2, 1 - s), 1e-4);
  }

  // W
  double W = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j)
        continue;
      int k = 3 - i - j;
      for (int s = 0; s < 2; s++) {
        W += v(i, s) * v(j, 1 - s) * v(k, 1 - s) *
             DiracDelta(v(i, 1 - s) + v(j, s) + v(k, s));
      }
    }
  }

  // K
  double K = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j)
        continue;
      int k = 3 - i - j;
      K +=
          v(i, 0) * v(i, 1) * v(j, 0) * v(k, 1) * DiracDelta(v(j, 1) + v(k, 0));
      K +=
          v(i, 0) * v(i, 1) * v(j, 1) * v(k, 0) * DiracDelta(v(j, 0) + v(k, 1));
    }
  }

  // compute X junction
  double X = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j)
        continue;
      int k = 3 - i - j;
      X +=
          v(i, 0) * v(i, 1) * v(j, 0) * v(j, 1) * DiracDelta(v(k, 0) + v(k, 1));
    }
  }

  // compute T junction
  double T = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j)
        continue;
      int k = 3 - i - j;
      T +=
          v(i, 0) * v(i, 1) * v(j, 0) * DiracDelta(v(j, 1) + v(k, 0) + v(k, 1));
      T +=
          v(i, 0) * v(i, 1) * v(j, 1) * DiracDelta(v(j, 0) + v(k, 0) + v(k, 1));
    }
  }

  // compute L junction
  double L = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j)
        continue;
      int k = 3 - i - j;
      for (int a = 0; a < 2; a++) {
        int nota = 1 - a;
        for (int b = 0; b < 2; b++) {
          int notb = 1 - b;
          L += v(i, a) * v(j, b) *
               DiracDelta(v(i, nota) + v(j, notb) + v(k, 0) + v(k, 1));
        }
      }
    }
  }

  // std::cout << " Y-" << Y << " W-" << W << " K-" << K <<
  //    " X-" << X << " T-" << T << " L-" << L << std::endl;
  static const double threshold = 1e-4;
  if (Y > threshold) {
    junctionWeight += 5.0;
  } else if (W > threshold) {
    junctionWeight += 5.0;
  } else if (L > threshold) {
    junctionWeight += 4.0;
  } else if (K > threshold) {
    junctionWeight += 3.0;
  } else if (X > threshold) {
    junctionWeight += 5.0;
  } else if (T > threshold) {
    junctionWeight += 0.0;
  }

  return junctionWeight;
}
}
}