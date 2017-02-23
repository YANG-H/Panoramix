#include "pch.hpp"

#include "clock.hpp"
#include "geo_context.hpp"
#include "line_detection.hpp"
#include "segmentation.hpp"
#include "utility.hpp"

#include "pi_graph_control.hpp"

namespace pano {
namespace experimental {

std::vector<int> CollectSegsIntersectingDirection(
    const Vec3 &direction, bool alsoConsiderBackward,
    const PIGraph<PanoramicCamera> &mg, double rangeAngle) {
  // find peaky regions
  std::vector<int> peakySegs;
  for (int i = 0; i < mg.nsegs; i++) {
    double radiusAngle = 0.0;
    auto &contours = mg.seg2contours[i];
    for (auto &cs : contours) {
      for (auto &c : cs) {
        double angle = AngleBetweenDirected(mg.seg2center[i], c);
        if (angle > radiusAngle) {
          radiusAngle = angle;
        }
      }
    }

    if ((alsoConsiderBackward
             ? AngleBetweenUndirected(direction, mg.seg2center[i])
             : AngleBetweenDirected(direction, mg.seg2center[i])) >
        radiusAngle) {
      continue;
    }

    float ppcFocal = 100.0f;
    int ppcSize = 2 * radiusAngle * ppcFocal + 10;
    Vec3 x;
    std::tie(x, std::ignore) =
        ProposeXYDirectionsFromZDirection(mg.seg2center[i]);
    PartialPanoramicCamera ppc(ppcSize, ppcSize, ppcFocal, Point3(0, 0, 0),
                               mg.seg2center[i], x);
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

    // intersection test
    auto p = ToPixel(ppc.toScreen(direction));
    auto p2 = ToPixel(ppc.toScreen(-direction));

    int dilateSize = ppcFocal * rangeAngle;
    bool intersected = false;
    for (int x = -dilateSize; x <= dilateSize; x++) {
      if (intersected)
        break;
      for (int y = -dilateSize; y <= dilateSize; y++) {
        if (intersected)
          break;
        auto pp = Pixel(p.x + x, p.y + y);
        if (Contains(mask.size(), pp) && mask(pp)) {
          peakySegs.push_back(i);
          intersected = true;
        }
        if (alsoConsiderBackward) {
          auto pp2 = Pixel(p2.x + x, p2.y + y);
          if (Contains(mask.size(), pp2) && mask(pp2)) {
            peakySegs.push_back(i);
            intersected = true;
          }
        }
      }
    }
  }

  return peakySegs;
}

namespace {

// returns false if confliction occurs
bool MakeRegionPlaneUsable(int seg, bool usable, PIGraph<PanoramicCamera> &mg) {
  auto &p = mg.seg2control[seg];
  if (p.used == usable)
    return true;
  if (p.used && !usable) {
    p.used = false;
    p.orientationClaz = p.orientationNotClaz = -1;
    return true;
  }
  p.orientationClaz = p.orientationNotClaz = -1;
  return true;
}

// returns false if confliction occurs
bool MakeRegionPlaneToward(int seg, int normalVPId,
                           PIGraph<PanoramicCamera> &mg) {
  auto &p = mg.seg2control[seg];
  if (!p.used)
    return true;
  assert(normalVPId != -1);
  if (p.orientationClaz != -1) {
    if (p.orientationClaz != normalVPId)
      return false;
    return true;
  }
  if (p.orientationNotClaz == -1) {
    p.orientationClaz = normalVPId;
    return true;
  }
  auto &dir = mg.vps[p.orientationNotClaz];
  if (IsFuzzyPerpendicular(mg.vps[normalVPId], dir)) {
    p.orientationClaz = normalVPId;
    p.orientationNotClaz = -1;
    return true;
  }
  return false;
}

// returns false if confliction occurs
bool MakeRegionPlaneAlsoAlong(int seg, int alongVPId,
                              PIGraph<PanoramicCamera> &mg) {
  auto &p = mg.seg2control[seg];
  if (!p.used)
    return true;
  assert(alongVPId != -1);
  auto &dir = mg.vps[alongVPId];
  if (p.orientationClaz != -1) {
    auto &normal = mg.vps[p.orientationClaz];
    return IsFuzzyPerpendicular(normal, dir);
  }
  if (p.orientationNotClaz == -1) {
    p.orientationNotClaz = alongVPId;
    return true;
  }
  if (p.orientationNotClaz == alongVPId)
    return true;

  auto newNormal = dir.cross(mg.vps[p.orientationNotClaz]);
  double minAngle = M_PI;
  for (int i = 0; i < mg.vps.size(); i++) {
    double angle = AngleBetweenUndirected(mg.vps[i], newNormal);
    if (angle < minAngle) {
      p.orientationClaz = i;
      minAngle = angle;
    }
  }
  if (p.orientationClaz != -1) {
    p.orientationNotClaz = -1;
    return true;
  }
  return false;
}
}

void AttachPrincipleDirectionConstraints(PIGraph<PanoramicCamera> &mg) {
  SetClock();
  std::vector<std::vector<int>> peakySegs(mg.vps.size());
  for (int i = 0; i < mg.vps.size(); i++) {
    // peakySegs[i] = CollectSegsIntersectingDirection(mg.vps[i], true, mg, M_PI
    // / 30.0);
    for (auto p : {ToPixel(mg.view.camera.toScreen(mg.vps[i])),
                   ToPixel(mg.view.camera.toScreen(-mg.vps[i]))}) {
      p.x = WrapBetween(p.x, 0, mg.segs.cols);
      p.y = BoundBetween(p.y, 0, mg.segs.rows - 1);
      int seg = mg.segs(p);
      peakySegs[i].push_back(seg);
    }
  }
  for (int i = 0; i < mg.vps.size(); i++) {
    auto &vp = mg.vps[i];
    auto &segs = peakySegs[i];
    for (auto seg : segs) {
      if (!mg.seg2control[seg].used) {
        continue;
      }
      bool b = MakeRegionPlaneToward(seg, i, mg);
      for (int lp : mg.seg2linePieces[seg]) {
        if (mg.linePiece2samples[lp].size() < 2)
          continue;
        int &lc = mg.lines[mg.linePiece2line[lp]].claz;
        if (lc == i) {
          lc = -1;
        }
      }
    }
  }
}

void AttachWallConstraints(PIGraph<PanoramicCamera> &mg, double rangeAngle) {
  SetClock();
  auto &vertical = mg.vps[mg.verticalVPId];

  std::vector<int> horizontalSegs;
  for (int seg = 0; seg < mg.nsegs; seg++) {
    if (!mg.seg2control[seg].used)
      continue;
    auto &contours = mg.seg2contours[seg];
    bool intersected = false;
    for (auto &cs : contours) {
      if (intersected)
        break;
      for (auto &c : cs) {
        double angle = M_PI_2 - AngleBetweenUndirected(c, vertical);
        if (angle <= rangeAngle) {
          intersected = true;
          break;
        }
      }
    }
    if (intersected) {
      horizontalSegs.push_back(seg);
    }
  }
  for (auto h : horizontalSegs) {
    MakeRegionPlaneAlsoAlong(h, mg.verticalVPId, mg);
  }
}

void DisableTopSeg(PIGraph<PanoramicCamera> &mg) {
  int topSeg = mg.segs(Pixel(0, 0));
  mg.seg2control[topSeg].used = false;
  for (int bnd : mg.seg2bnds[topSeg]) {
    bool onLeft = mg.bnd2segs[bnd].first == topSeg;
    for (int bp : mg.bnd2bndPieces[bnd]) {
      mg.bndPiece2segRelation[bp] =
          onLeft ? SegRelation::RightIsFront : SegRelation::LeftIsFront;
    }
  }
}

void DisableBottomSeg(PIGraph<PanoramicCamera> &mg) {
  int bottomSeg = mg.segs(Pixel(0, mg.segs.rows - 1));
  mg.seg2control[bottomSeg].used = false;
  for (int bnd : mg.seg2bnds[bottomSeg]) {
    bool onLeft = mg.bnd2segs[bnd].first == bottomSeg;
    for (int bp : mg.bnd2bndPieces[bnd]) {
      mg.bndPiece2segRelation[bp] =
          onLeft ? SegRelation::RightIsFront : SegRelation::LeftIsFront;
    }
  }
}

namespace {
template <class CameraT>
void AttachGCConstraintsTemplated(PIGraph<PanoramicCamera> &mg,
                                  const View<CameraT, Image5d> &gc,
                                  double clutterThres, double wallThres,
                                  bool onlyConsiderBottomHalf) {

  auto up = normalize(mg.vps[mg.verticalVPId]);
  if (up.dot(-mg.view.camera.up()) < 0) {
    up = -up;
  }

  auto gcMeanOnSegs = CollectFeatureMeanOnSegs(mg, gc.camera, gc.image);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    auto &c = mg.seg2control[seg];
    if (!c.used) {
      continue;
    }
    auto &gcMean = gcMeanOnSegs[seg];
    double gcMeanSum =
        std::accumulate(std::begin(gcMean.val), std::end(gcMean.val), 0.0);
    assert(gcMeanSum <= 1.1);
    if (gcMeanSum == 0.0) {
      continue;
    }

    std::vector<size_t> orderedIds(
        std::distance(std::begin(gcMean), std::end(gcMean)));
    std::iota(orderedIds.begin(), orderedIds.end(), 0ull);
    std::sort(orderedIds.begin(), orderedIds.end(),
              [&gcMean](size_t a, size_t b) { return gcMean[a] > gcMean[b]; });

    size_t maxid = orderedIds.front();
    if (maxid == ToUnderlying(GeometricContextIndex::ClutterOrPorous) &&
        gcMean[maxid] < clutterThres) {
      maxid = orderedIds[1];
    }

    if (maxid == ToUnderlying(GeometricContextIndex::FloorOrGround) &&
        (!onlyConsiderBottomHalf || mg.seg2center[seg].dot(up) < 0)) { // lower
      MakeRegionPlaneToward(seg, mg.verticalVPId, mg);
      continue;
    }
    if (maxid == ToUnderlying(GeometricContextIndex::CeilingOrSky)) {
      MakeRegionPlaneToward(seg, mg.verticalVPId, mg);
      continue;
    }
    double wallScore = gcMean[ToUnderlying(GeometricContextIndex::Vertical)];
    if (wallScore > wallThres &&
        (!onlyConsiderBottomHalf || mg.seg2center[seg].dot(up) < 0)) {
      MakeRegionPlaneAlsoAlong(seg, mg.verticalVPId, mg);
      continue;
    }
  }
}
}

void AttachGCConstraints(PIGraph<PanoramicCamera> &mg,
                         const View<PanoramicCamera, Image5d> &gc,
                         double clutterThres, double wallThres,
                         bool onlyConsiderBottomHalf) {
  AttachGCConstraintsTemplated(mg, gc, clutterThres, wallThres,
                               onlyConsiderBottomHalf);
}

void AttachGCConstraints(PIGraph<PanoramicCamera> &mg,
                         const View<PerspectiveCamera, Image5d> &gc,
                         double clutterThres, double wallThres,
                         bool onlyConsiderBottomHalf) {
  AttachGCConstraintsTemplated(mg, gc, clutterThres, wallThres,
                               onlyConsiderBottomHalf);
}
}
}