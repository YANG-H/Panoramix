#pragma once

#include "basic_types.hpp"
#include "cameras.hpp"
#include "utility.hpp"

#include "color.hpp"
#include "shader.hpp"

namespace pano {
namespace experimental {

using namespace pano::core;

struct SegControl {
  int orientationClaz, orientationNotClaz;
  bool used;
  int dof() const {
    if (orientationClaz != -1)
      return 1;
    if (orientationNotClaz != -1)
      return 2;
    return 3;
  }
  bool operator==(const SegControl &c) const {
    return std::tie(orientationClaz, orientationNotClaz, used) ==
           std::tie(c.orientationClaz, c.orientationNotClaz, used);
  }
  template <class Archiver> void serialize(Archiver &ar) {
    ar(orientationClaz, orientationNotClaz, used);
  }
};

// whether a bnd is an occlusion
enum class SegRelation { Connected, LeftIsFront, RightIsFront, Unknown };

// whether a line attaches a seg
enum class SegLineRelation { Attached, Detached, Unknown };

// whether two line connects
enum class LineRelation { Attached, Detached, Unknown };

template <class CameraT> struct PIGraph {
  View<CameraT> view;
  std::vector<Vec3> vps;
  int verticalVPId;

  // seg
  Imagei segs;
  int nsegs;
  std::vector<std::vector<int>> seg2bnds;
  std::vector<std::vector<int>> seg2linePieces;
  std::vector<SegControl> seg2control;
  std::vector<double> seg2areaRatio;
  double fullArea;
  std::vector<Vec3> seg2center;
  std::vector<std::vector<std::vector<Vec3>>> seg2contours;

  // linePiece
  std::vector<std::vector<Vec3>> linePiece2samples;
  std::vector<double> linePiece2length;
  std::vector<int> linePiece2line;
  std::vector<int> linePiece2seg; // could be -1
  std::vector<SegLineRelation>
      linePiece2segLineRelation; // for linePice2seg only, as for
                                 // linePIece2bndPiece, see the
                                 // bndPiece2segRelation

  std::vector<int> linePiece2bndPiece; // could be -1 (either 2seg or 2bndPiece)
  std::vector<bool> linePiece2bndPieceInSameDirection;
  int nlinePieces() const { return linePiece2samples.size(); }

  // line
  std::vector<Classified<Line3>> lines;
  std::vector<std::vector<int>> line2linePieces;
  std::vector<std::vector<int>> line2lineRelations;
  std::vector<bool> line2used;
  int nlines() const { return lines.size(); }

  // lineRelation
  std::vector<LineRelation> lineRelations;
  std::vector<Vec3> lineRelation2anchor;
  std::vector<std::pair<int, int>> lineRelation2lines;
  std::vector<double> lineRelation2weight;
  std::vector<bool> lineRelation2IsIncidence;
  int nlineRelations() const { return lineRelation2anchor.size(); }

  // bndPiece (a STRAIGHT boundary piece in a bnd)
  std::vector<std::vector<Vec3>> bndPiece2dirs; // continuous
  std::vector<double> bndPiece2length;
  std::vector<int> bndPiece2classes;
  std::vector<int> bndPiece2bnd;
  std::vector<std::vector<int>> bndPiece2linePieces;
  std::vector<SegRelation> bndPiece2segRelation;
  int nbndPieces() const { return bndPiece2dirs.size(); }

  // bnd (a CONTINUOUS boundary between TWO segs)
  std::vector<std::vector<int>> bnd2bndPieces; // continuously connected
  std::vector<std::pair<int, int>> bnd2segs;   // left , right
  std::vector<std::pair<int, int>> bnd2juncs;  // from, to
  int nbnds() const { return bnd2bndPieces.size(); }

  // junc (junction of >=3 bnds)
  std::vector<Vec3> junc2positions;
  std::vector<std::vector<int>> junc2bnds;
  int njuncs() const { return junc2positions.size(); }

  template <class Archiver> void serialize(Archiver &ar) {
    ar(view, vps, verticalVPId);
    ar(segs, nsegs, seg2bnds, seg2linePieces, seg2control, seg2areaRatio,
       fullArea, seg2center, seg2contours);
    ar(linePiece2samples, linePiece2length, linePiece2line, linePiece2seg,
       linePiece2segLineRelation, linePiece2bndPiece,
       linePiece2bndPieceInSameDirection);
    ar(lines, line2linePieces, line2lineRelations);
    ar(lineRelations, lineRelation2anchor, lineRelation2lines,
       lineRelation2weight, lineRelation2IsIncidence);
    ar(bndPiece2dirs, bndPiece2length, bndPiece2classes, bndPiece2bnd,
       bndPiece2linePieces, bndPiece2segRelation);
    ar(bnd2bndPieces, bnd2segs, bnd2juncs);
    ar(junc2positions, junc2bnds);
  }
};

int SegmentationForPIGraph(const PanoramicView &view,
                           const std::vector<Classified<Line3>> &lines,
                           Imagei &segs,
                           double lineExtendAngle = DegreesToRadians(5),
                           double sigma = 10.0, double c = 1.0,
                           double minSize = 200,
                           int widthThresToRemoveThinRegions = 2);

PIGraph<PanoramicCamera> BuildPIGraph(
    const PanoramicView &view, const std::vector<Vec3> &vps, int verticalVPId,
    const Imagei &segs, const std::vector<Classified<Line3>> &lines,
    double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
    double bndPieceBoundToLineAngleThres, double intersectionAngleThreshold,
    double incidenceAngleAlongDirectionThreshold,
    double incidenceAngleVerticalDirectionThreshold);

PIGraph<PerspectiveCamera> BuildPIGraph(
    const PerspectiveView &view, const std::vector<Vec3> &vps, int verticalVPId,
    const Imagei &segs, const std::vector<Classified<Line3>> &lines,
    double bndPieceSplitAngleThres, double bndPieceClassifyAngleThres,
    double bndPieceBoundToLineAngleThres, double intersectionAngleThreshold,
    double incidenceAngleAlongDirectionThreshold,
    double incidenceAngleVerticalDirectionThreshold);

// PerfectSegMaskView
View<PartialPanoramicCamera, Imageub>
PerfectSegMaskView(const PIGraph<PanoramicCamera> &mg, int seg,
                   double focal = 100.0);
View<PartialPanoramicCamera, Imageub>
PerfectSegMaskView(const PIGraph<PerspectiveCamera> &mg, int seg,
                   double focal = 100.0);

// CollectFeatureMeanOnSegs
template <class T, int N, class PIGraphCameraT, class CameraT>
std::vector<Vec<T, N>>
CollectFeatureMeanOnSegs(const PIGraph<PIGraphCameraT> &mg, const CameraT &pcam,
                         const Image_<Vec<T, N>> &feature) {
  std::vector<Vec<T, N>> featureMeanTable(mg.nsegs);
  for (int i = 0; i < mg.nsegs; i++) {
    auto regionMaskView = PerfectSegMaskView(mg, i, 100.0);
    if (regionMaskView.image.empty()) {
      continue;
    }
    auto sampler = MakeCameraSampler(regionMaskView.camera, pcam);
    auto featureOnRegion = sampler(feature);
    int votes = 0;
    Vec<T, N> featureSum;
    for (auto it = regionMaskView.image.begin();
         it != regionMaskView.image.end(); ++it) {
      if (!*it) {
        continue;
      }
      featureSum += featureOnRegion(it.pos());
      votes += 1;
    }
    auto featureMean = featureSum / std::max(votes, 1);
    featureMeanTable[i] = featureMean;
  }
  return featureMeanTable;
}

// JunctionWeights

// 7.0, 10.0
float IncidenceJunctionWeight(bool acrossViews);
float OutsiderIntersectionJunctionWeight();
// [0.0 ~ 5.0]
float ComputeIntersectionJunctionWeightWithLinesVotes(
    const Mat<float, 3, 2> &votes);
}
}