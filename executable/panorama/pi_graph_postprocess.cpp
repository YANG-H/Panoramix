#include "pch.hpp"

#include "geo_context.hpp"
#include "line_detection.hpp"
#include "segmentation.hpp"

#include "pi_graph_postprocess.hpp"

namespace pano {
namespace experimental {
std::vector<Polygon3> CompactModel(const PICGDeterminablePart &dp,
                                   const PIConstraintGraph &cg,
                                   const PIGraph<PanoramicCamera> &mg,
                                   double distThres) {

  struct Corner {
    int junction; //
    int bndpiece; // the head of the bndpiece if junction == -1
    Vec3 dir;
    bool isJunction() const { return junction != -1; }
    bool isHeadOfBndPiece() const { return junction == -1; }
    bool operator<(const Corner &c) const {
      return std::tie(junction, bndpiece) < std::tie(c.junction, c.bndpiece);
    }
    bool operator==(const Corner &c) const {
      return std::tie(junction, bndpiece) == std::tie(c.junction, c.bndpiece);
    }
  };

  std::vector<Corner> corners;
  corners.reserve(mg.njuncs() + mg.nbndPieces());

  // add junction corners
  std::vector<int> junc2corner(mg.njuncs(), -1);
  for (int junc = 0; junc < mg.njuncs(); junc++) {
    Corner c;
    c.junction = junc;
    c.bndpiece = -1;
    c.dir = normalize(mg.junc2positions[junc]);
    corners.push_back(c);
    junc2corner[junc] = corners.size() - 1;
  }

  // add bndpiece head corners
  std::vector<int> bndPiece2corner(mg.nbndPieces(), -1);
  for (int bnd = 0; bnd < mg.nbnds(); bnd++) {
    auto &bps = mg.bnd2bndPieces[bnd];
    assert(!bps.empty());
    for (int k = 1; k < bps.size(); k++) {
      int bp = bps[k];
      Corner c;
      c.junction = -1;
      c.bndpiece = bp;
      c.dir = normalize(mg.bndPiece2dirs[bp].front());
      corners.push_back(c);
      bndPiece2corner[bp] = corners.size() - 1;
    }
  }

  const int ncorners = corners.size();

  // collect corner2segs
  std::vector<std::vector<int>> corner2segs(ncorners);
  for (int corner = 0; corner < ncorners; corner++) {
    auto &c = corners[corner];
    if (c.isJunction()) {
      std::set<int> relatedSegs;
      for (int bnd : mg.junc2bnds[c.junction]) {
        relatedSegs.insert(mg.bnd2segs[bnd].first);
        relatedSegs.insert(mg.bnd2segs[bnd].second);
      }
      corner2segs[corner] =
          std::vector<int>(relatedSegs.begin(), relatedSegs.end());
    } else {
      int bnd = mg.bndPiece2bnd[c.bndpiece];
      corner2segs[corner] = {mg.bnd2segs[bnd].first, mg.bnd2segs[bnd].second};
    }
  }

  // collect seg2corners
  std::vector<std::vector<int>> seg2corners(mg.nsegs);
  std::vector<std::vector<bool>> seg2bpCornersReversed(mg.nsegs);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    std::set<int> bndsVisited;

    for (int firstBnd : mg.seg2bnds[seg]) {
      if (Contains(bndsVisited, firstBnd)) {
        continue;
      }

      bool firstBndReversed = mg.bnd2segs[firstBnd].second != seg;

      std::vector<int> orderedBnds = {firstBnd};
      std::vector<bool> bndReversed = {firstBndReversed};

      int fromJunc = mg.bnd2juncs[firstBnd].first;
      int toJunc = mg.bnd2juncs[firstBnd].second;
      if (firstBndReversed) {
        std::swap(fromJunc, toJunc);
      }

      std::vector<int> orderedJuncs = {fromJunc, toJunc};
      // the order should be [junc0, bnd0, junc1, bnd1, ..., ] -> [junc0 ,...

      // lastBndRightHand

      while (orderedJuncs.back() != orderedJuncs.front()) {
        auto &bndCands = mg.junc2bnds[orderedJuncs.back()];
        // int bndSelected = -1;
        // int nextJuncSelected = -1;
        bool foundBnd = false;
        for (int bnd : bndCands) {
          if (mg.bnd2segs[bnd].first != seg && mg.bnd2segs[bnd].second != seg) {
            continue;
          }
          if (Contains(orderedBnds, bnd)) {
            continue;
          }
          orderedBnds.push_back(bnd);
          bool reversed = mg.bnd2segs[bnd].second != seg;
          bndReversed.push_back(reversed);
          int nextJunc =
              reversed ? mg.bnd2juncs[bnd].first : mg.bnd2juncs[bnd].second;
          orderedJuncs.push_back(nextJunc);
          foundBnd = true;
          break;
        }
        // assert(foundBnd);
        if (!foundBnd) {
          break;
        }
      }

      std::vector<int> cs;
      std::vector<bool> creversed;
      for (int i = 0; i < orderedBnds.size(); i++) {
        int junc = orderedJuncs[i];
        int bnd = orderedBnds[i];
        bool rev = bndReversed[i];

        assert(junc2corner[junc] != -1);
        cs.push_back(junc2corner[junc]);
        creversed.push_back(false);
        auto &bps = mg.bnd2bndPieces[bnd];
        if (!rev) {
          for (int k = 1; k < bps.size(); k++) {
            int bp = bps[k];
            assert(bndPiece2corner[bp] != -1);
            cs.push_back(bndPiece2corner[bp]);
            creversed.push_back(false);
          }
        } else {
          for (int k = bps.size() - 1; k > 0; k--) {
            int bp = bps[k];
            assert(bndPiece2corner[bp] != -1);
            cs.push_back(bndPiece2corner[bp]);
            creversed.push_back(true);
          }
        }
      }

      if (cs.size() > seg2corners[seg].size()) {
        for (int i = 0; i < orderedBnds.size(); i++) {
          int bnd = orderedBnds[i];
          bndsVisited.insert(bnd);
        }
        seg2corners[seg] = std::move(cs);
        seg2bpCornersReversed[seg] = std::move(creversed);
      }
    }
  }

  // fix the surfaces
  std::vector<bool> seg2determined(mg.nsegs, false);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    seg2determined[seg] = true;
  }

  std::vector<Plane3> seg2plane(mg.nsegs);
  std::vector<std::map<int, double>> corner2segDepths(ncorners);

  // initialize seg2planes
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    seg2plane[seg] = cg.entities[ent].supportingPlane.reconstructed;
  }

  int t = 0;
  while (true) {

    std::cout << "expanding segs - " << (t++) << std::endl;

    // update corner2segDepths using seg2plane
    for (int corner = 0; corner < ncorners; corner++) {
      auto &segs = corner2segs[corner];
      auto &segDepths = corner2segDepths[corner];
      for (int seg : segs) {
        auto &plane = seg2plane[seg];
        if (plane.normal != Origin()) {
          double depth =
              norm(Intersection(Ray3(Origin(), corners[corner].dir), plane));
          segDepths[seg] = depth;
        }
      }

      if (segDepths.empty()) {
        continue;
      }

      // cluster these depths
      std::vector<double> groupedDepths;
      std::map<int, int> seg2group;
      for (auto &segAndDepth : segDepths) {
        int seg = segAndDepth.first;
        double depth = segAndDepth.second;
        double minDepthDiff = distThres;
        for (int g = 0; g < groupedDepths.size();
             g++) { // find the group with min depth diff
          double depthDiff = abs(depth - groupedDepths[g]);
          if (depthDiff < minDepthDiff) {
            seg2group[seg] = g;
            minDepthDiff = depthDiff;
          }
        }
        if (!Contains(seg2group,
                      seg)) { // if not group is found, add a new group
          groupedDepths.push_back(depth);
          seg2group[seg] = groupedDepths.size() - 1;
        }
      }

      // update each seg depth using group depth
      for (auto &segAndDepth : segDepths) {
        segAndDepth.second = groupedDepths[seg2group.at(segAndDepth.first)];
      }
    }

    // update seg planes
    bool hasPlanesUpdated = false;
    for (int seg = 0; seg < mg.nsegs; seg++) {
      if (!mg.seg2control[seg].used) {
        continue;
      }
      auto &plane = seg2plane[seg];
      if (plane.normal == Origin()) {
        auto &cs = seg2corners[seg];
        std::vector<std::vector<Point3>> candContourTable;
        for (int k = 0; k < cs.size(); k++) {
          int corner = cs[k];
          auto dir = normalize(corners[corner].dir);
          std::vector<Point3> candContour;
          for (auto &dcand : corner2segDepths[corner]) {
            double depth = dcand.second;
            candContour.push_back(dir * depth);
          }
          if (!candContour.empty()) {
            candContourTable.push_back(std::move(candContour));
          }
        }
        if (candContourTable.size() >= 3) {
          std::vector<int> candContourIds(candContourTable.size());
          std::iota(candContourIds.begin(), candContourIds.end(), 0);
          // make cand contour with fewer cands forward
          std::sort(candContourIds.begin(), candContourIds.end(),
                    [&candContourTable](int a, int b) {
                      return candContourTable[a].size() <
                             candContourTable[b].size();
                    });
          // iterate all candidate planes to find the best that fit the most
          Plane3 bestPlane;
          double bestScore = 0.0;
          for (auto &p1 : candContourTable[candContourIds[0]]) {
            for (auto &p2 : candContourTable[candContourIds[1]]) {
              for (auto &p3 : candContourTable[candContourIds[2]]) {
                auto candPlane = Plane3From3Points(p1, p2, p3);
                if (candPlane.distanceTo(Origin()) < 0.5) {
                  continue;
                }
                double candPlaneScore = 0.0;
                // is this candPlane fit other contour cands?
                for (int i = 3; i < candContourIds.size(); i++) {
                  for (auto &pRest : candContourTable[candContourIds[i]]) {
                    if (DistanceFromPointToPlane(pRest, candPlane).first <
                        distThres) {
                      candPlaneScore += 1.0;
                    }
                  }
                }
                for (int c : cs) {
                  double depth = norm(
                      Intersection(Ray3(Origin(), corners[c].dir), candPlane));
                  if (depth > 2.0) {
                    candPlaneScore -= 3.0;
                  }
                }
                if (std::any_of(mg.vps.begin(), mg.vps.end(),
                                [&candPlane](const Vec3 &vp) {
                                  return AngleBetweenUndirected(
                                             vp, candPlane.normal) <
                                         DegreesToRadians(5);
                                })) {
                  candPlaneScore + 0.5;
                }
                if (candPlaneScore > bestScore) {
                  bestScore = candPlaneScore;
                  bestPlane = candPlane;
                }
              }
            }
          }
          // find a best plane
          if (bestPlane.normal != Origin()) {
            plane = bestPlane;
            hasPlanesUpdated = true;
          }
        }
      }
    }

    if (!hasPlanesUpdated) {
      break;
    }
  }

  // make polygons
  std::vector<Polygon3> seg2compact(mg.nsegs);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    auto &plane = seg2plane[seg];
    if (plane.normal == Origin()) {
      continue;
    }
    auto &compact = seg2compact[seg];
    auto &cs = seg2corners[seg];
    if (cs.empty()) {
      continue;
    }
    compact.corners.resize(cs.size());
    compact.normal = plane.normal;
    for (int k = 0; k < cs.size(); k++) {
      int corner = cs[k];
      auto dir = normalize(corners[corner].dir);
      // find most fit depth in corner2segDepths
      double depth = -1.0;
      double minDist = std::numeric_limits<double>::infinity();
      for (auto &depthCand : corner2segDepths[corner]) {
        Point3 p = dir * depthCand.second;
        double dist = plane.distanceTo(p);
        if (dist < minDist) {
          depth = depthCand.second;
          minDist = dist;
        }
      }
      // corner2segDepths[corner] is empty, then compute depth directly
      if (depth > 0) {
        compact.corners[k] = dir * depth;
      } else {
        compact.corners[k] = Intersection(Ray3(Origin(), dir), plane);
      }
    }
    if (compact.normal.dot(compact.corners.front()) < 0) {
      compact.normal = -compact.normal;
    }
  }

  return seg2compact;
}

std::vector<Vec3> ComputeSegNormals(const PICGDeterminablePart &dp,
                                    const PIConstraintGraph &cg,
                                    const PIGraph<PanoramicCamera> &mg, bool smoothed) {
  std::vector<Vec3> seg2normal(mg.nsegs);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    if (!mg.seg2control[seg].used) {
      continue;
    }
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    auto &plane = cg.entities[ent].supportingPlane.reconstructed;
    auto n = normalize(plane.normal);
    seg2normal[seg] = n;
  }
  if (smoothed) {
    std::map<int, std::set<int>> seg2segs;
    for (int x = 0; x < mg.segs.cols; x++) {
      for (int y = 0; y < mg.segs.rows; y++) {
        Pixel p1(x, y);
        for (int dx = -2; dx <= 2; dx++) {
          for (int dy = -2; dy <= 2; dy++) {
            Pixel p2(x + dx, y + dy);
            if (p2 == p1) {
              continue;
            }
            p2.x = (p2.x + mg.segs.cols) % mg.segs.cols;
            if (!Contains(mg.segs.size(), p2)) {
              continue;
            }
            int seg1 = mg.segs(p1);
            int seg2 = mg.segs(p2);
            seg2segs[seg1].insert(seg2);
            seg2segs[seg2].insert(seg1);
          }
        }
      }
    }
    while (true) {
      bool hasUpdate = false;
      for (int seg = 0; seg < mg.nsegs; seg++) {
        if (!mg.seg2control[seg].used) {
          continue;
        }
        auto &n = seg2normal[seg];
        if (n != Origin()) {
          continue;
        }
        Vec3 mergedNormal;
        for (int anotherSeg : seg2segs[seg]) {
          mergedNormal += seg2normal[anotherSeg];
        }
        if (mergedNormal != Origin()) {
          n = normalize(mergedNormal);
          hasUpdate = true;
        }
      }
      if (!hasUpdate) {
        break;
      }
    }
  }
  return seg2normal;
}

std::vector<Plane3> ComputeSegPlanes(const PICGDeterminablePart &dp,
                                     const PIConstraintGraph &cg,
                                     const PIGraph<PanoramicCamera> &mg, bool smoothed) {
  std::vector<Plane3> seg2planes(mg.nsegs);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    if (!mg.seg2control[seg].used) {
      continue;
    }
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    auto &plane = cg.entities[ent].supportingPlane.reconstructed;
    seg2planes[seg] = plane;
  }
  if (smoothed) {
    std::map<int, std::set<int>> seg2segs;
    for (int x = 0; x < mg.segs.cols; x++) {
      for (int y = 0; y < mg.segs.rows; y++) {
        Pixel p1(x, y);
        for (int dx = -2; dx <= 2; dx++) {
          for (int dy = -2; dy <= 2; dy++) {
            Pixel p2(x + dx, y + dy);
            if (p2 == p1) {
              continue;
            }
            p2.x = (p2.x + mg.segs.cols) % mg.segs.cols;
            if (!Contains(mg.segs.size(), p2)) {
              continue;
            }
            int seg1 = mg.segs(p1);
            int seg2 = mg.segs(p2);
            seg2segs[seg1].insert(seg2);
            seg2segs[seg2].insert(seg1);
          }
        }
      }
    }
    while (true) {
      bool hasUpdate = false;
      for (int seg = 0; seg < mg.nsegs; seg++) {
        if (!mg.seg2control[seg].used) {
          continue;
        }
        if (seg2planes[seg].normal != Origin()) {
          continue;
        }
        Vec3 planeEq;
        int count = 0;
        for (int anotherSeg : seg2segs[seg]) {
          if (seg2planes[anotherSeg].normal != Origin()) {
            planeEq += Plane3ToEquation(seg2planes[anotherSeg]);
            count++;
          }
        }
        if (planeEq != Origin()) {
          planeEq = planeEq / count;
          hasUpdate = true;
          seg2planes[seg] = Plane3FromEquation(planeEq);
        }
      }
      if (!hasUpdate) {
        break;
      }
    }
  }
  return seg2planes;
}
}
}