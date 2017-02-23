#include "pch.hpp"

#include "containers.hpp"
#include "factor_graph.hpp"
#include "geo_context.hpp"
#include "line_detection.hpp"
#include "segmentation.hpp"
#include "utility.hpp"

#include "canvas.hpp"

#include "pi_graph_annotation.hpp"
#include "pi_graph_occlusion.hpp"
#include "pi_graph_vis.hpp"

namespace pano {
namespace experimental {

namespace {
template <class ContT>
inline bool AllAlong(const ContT &pts, const Vec3 &from, const Vec3 &to,
                     double angleThres) {
  auto n = normalize(from.cross(to));
  return core::AllOf<Vec3>(pts, [&n, angleThres](const Vec3 &p) {
    return abs(M_PI_2 - AngleBetweenDirected(n, p)) < angleThres;
  });
}
}

struct SegLabel {
  int orientationClaz, orientationNotClaz;
  inline int dof() const {
    if (orientationClaz != -1) {
      return 1;
    }
    if (orientationNotClaz != -1) {
      return 2;
    }
    return 3;
  }
  inline bool operator==(const SegLabel &sl) const {
    return std::tie(orientationClaz, orientationNotClaz) ==
           std::tie(sl.orientationClaz, sl.orientationNotClaz);
  }
  inline bool operator<(const SegLabel &sl) const {
    return std::tie(orientationClaz, orientationNotClaz) <
           std::tie(sl.orientationClaz, sl.orientationNotClaz);
  }
};

struct LineLabel {
  bool connectLeft, connectRight;
  bool operator==(LineLabel ll) const {
    return connectLeft == ll.connectLeft && connectRight == ll.connectRight;
  }
  bool operator!=(LineLabel ll) const { return !(*this == ll); }
  LineLabel operator!() const { return LineLabel{!connectLeft, !connectRight}; }
  LineLabel leftRightSwapped() const {
    return LineLabel{connectRight, connectLeft};
  }
  template <class Archiver> void serialize(Archiver &ar) {
    ar(connectLeft, connectRight);
  }
};
struct LineLabelCost {
  double connectLeftConnectRight;
  double connectLeftDisconnectRight;
  double disconnectLeftConnectRight;
  double disconnectAll;
  template <class Archiver> void serialize(Archiver &ar) {
    ar(connectLeftConnectRight, connectLeftDisconnectRight,
       disconnectLeftConnectRight, disconnectAll);
  }
};

// assume that all oclcusions are described by lines
void DetectOcclusions(PIGraph<PanoramicCamera> &mg,
                      double minAngleSizeOfLineInTJunction,
                      double lambdaShrinkForHLineDetectionInTJunction,
                      double lambdaShrinkForVLineDetectionInTJunction,
                      double angleSizeForPixelsNearLines) {

  int width = mg.segs.cols;
  int height = mg.segs.rows;

  RTreeMap<Vec3, std::pair<Line3, int>> lineSamplesTree;
  for (int i = 0; i < mg.nlines(); i++) {
    auto &line = mg.lines[i].component;
    double spanAngle = AngleBetweenDirected(line.first, line.second);
    for (double a = 0.0; a < spanAngle;
         a += angleSizeForPixelsNearLines / 3.0) {
      Vec3 sample1 = normalize(RotateDirection(line.first, line.second, a));
      Vec3 sample2 = normalize(RotateDirection(
          line.first, line.second, a + angleSizeForPixelsNearLines / 3.0));
      lineSamplesTree.emplace(normalize(sample1 + sample2),
                              std::make_pair(Line3(sample1, sample2), i));
    }
  }

  // collect lines' nearby pixels and segs
  std::vector<std::set<Pixel>> line2nearbyPixels(mg.nlines());
  std::vector<std::map<int, Vec3>> line2nearbySegsWithLocalCenterDir(
      mg.nlines());
  std::vector<std::map<int, bool>> line2nearbySegsWithOnLeftFlag(mg.nlines());
  std::vector<std::map<int, double>> line2nearbySegsWithWeight(mg.nlines());

  for (auto it = mg.segs.begin(); it != mg.segs.end(); ++it) {
    Pixel p = it.pos();
    double weight = cos((p.y - (height - 1) / 2.0) / (height - 1) * M_PI);
    Vec3 dir = normalize(mg.view.camera.toSpace(p));
    int seg = *it;
    lineSamplesTree.search(
        BoundingBox(dir).expand(angleSizeForPixelsNearLines * 3),
        [&mg, &dir, &line2nearbyPixels, &line2nearbySegsWithLocalCenterDir,
         &line2nearbySegsWithWeight, angleSizeForPixelsNearLines, p, seg,
         weight](const std::pair<Vec3, std::pair<Line3, int>> &lineSample) {
          auto line = normalize(mg.lines[lineSample.second.second].component);
          auto dirOnLine = DistanceFromPointToLine(dir, lineSample.second.first)
                               .second.position;
          double angleDist = AngleBetweenDirected(dir, dirOnLine);
          double lambda = ProjectionOfPointOnLine(dir, line)
                              .ratio; // the projected position on line
          if (angleDist < angleSizeForPixelsNearLines &&
              IsBetween(lambda, 0.0, 1.0)) {
            line2nearbyPixels[lineSample.second.second].insert(p);
            auto &localCenterDir =
                line2nearbySegsWithLocalCenterDir[lineSample.second.second]
                                                 [seg];
            localCenterDir += dir * weight;
            double &segWeight =
                line2nearbySegsWithWeight[lineSample.second.second][seg];
            segWeight +=
                weight *
                Gaussian(
                    lambda - 0.5,
                    0.1); // the closer to the center, the more important it is!
          }
          return true;
        });
  }
  for (int i = 0; i < mg.nlines(); i++) {
    auto &nearbySegsWithLocalCenterDir = line2nearbySegsWithLocalCenterDir[i];
    auto &line = mg.lines[i].component;
    Vec3 lineRight = line.first.cross(line.second);
    for (auto &segWithDir : nearbySegsWithLocalCenterDir) {
      Vec3 centerDir = normalize(segWithDir.second);
      bool onLeft = (centerDir - line.first).dot(lineRight) < 0;
      line2nearbySegsWithOnLeftFlag[i][segWithDir.first] = onLeft;
    }
  }

  auto paintLineWithNearbyPixels = [&mg, &line2nearbyPixels,
                                    &line2nearbySegsWithOnLeftFlag](
      Image3f &pim, int line, const gui::Color &leftColor,
      const gui::Color &rightColor) {
    auto &pixels = line2nearbyPixels[line];
    auto &nearbySegsWithLeftFlags = line2nearbySegsWithOnLeftFlag[line];
    for (auto &p : pixels) {
      int seg = mg.segs(p);
      bool onLeft = nearbySegsWithLeftFlags.at(seg);
      if (onLeft) {
        gui::AsCanvas(pim).color(leftColor).add(p);
      } else {
        gui::AsCanvas(pim).color(rightColor).add(p);
      }
    }
  };
  if (false) {
    auto pim = PrintPIGraph(mg, ConstantFunctor<gui::Color>(gui::White),
                            ConstantFunctor<gui::Color>(gui::Transparent),
                            ConstantFunctor<gui::Color>(gui::LightGray), 1, 2);
    for (int i = 0; i < mg.nlines(); i++) {
      paintLineWithNearbyPixels(pim, i, gui::Green, gui::Blue);
    }
    gui::AsCanvas(pim).show();
  }

  core::FactorGraph fg;

  std::cout << "building factor graph" << std::endl;

  std::vector<std::vector<LineLabel>> line2allowedLabels(mg.nlines());
  std::vector<int> line2vh(mg.nlines(), -1);
  std::map<int, int> vh2line;
  for (int line = 0; line < mg.nlines(); line++) {
    if (mg.lines[line].claz == -1) {
      continue;
    }
    auto &l = mg.lines[line].component;
    std::vector<LineLabel> allowedLabels = {
        {true, true}, {true, false}, {false, true}, {false, false}};
    auto vh = fg.addVar(fg.addVarCategory(
        allowedLabels.size(), AngleBetweenDirected(l.first, l.second)));
    line2vh[line] = vh;
    vh2line[vh] = line;
    line2allowedLabels[line] = std::move(allowedLabels);
  }

  // factors preparation

  // orientation consistency term between seg and line

  std::vector<LineLabelCost> line2labelCost(mg.nlines(), {0, 1e-8, 1e-8, 1.0});

  for (int line = 0; line < mg.nlines(); line++) {
    auto vh = line2vh[line];
    if (vh == -1) {
      continue;
    }
    int claz = mg.lines[line].claz;
    auto &l = mg.lines[line].component;
    const auto &lps = mg.line2linePieces[line];
    auto &labelCosts = line2labelCost[line];

    // consider nearby segs
    for (auto &segAndWeight : line2nearbySegsWithWeight[line]) {
      int seg = segAndWeight.first;
      double weight = segAndWeight.second;
      bool onLeft = line2nearbySegsWithOnLeftFlag[line].at(seg);
      auto &segControl = mg.seg2control[seg];
      if (segControl.orientationClaz != -1 &&
          segControl.orientationClaz == claz) {
        if (onLeft) {
          labelCosts.connectLeftConnectRight += weight;
          labelCosts.connectLeftDisconnectRight += weight;
        } else {
          labelCosts.connectLeftConnectRight += weight;
          labelCosts.disconnectLeftConnectRight += weight;
        }
      }
    }
  }

  // {line-lineRelation-line} -> linePiece -> bndPiece, occlusion hints
  // suggested by line's t-junction
  std::vector<std::pair<int, int>> tjunctionLines; // [hline, vline]
  std::vector<bool> tjunctionVLineLiesOnTheRightOfHLine;
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1 = -1, line2 = -1;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    // whether this forms a t-junction?
    if (mg.lines[line1].claz == mg.lines[line2].claz) {
      continue;
    }
    auto l1 = normalize(mg.lines[line1].component);
    Vec3 n1 = normalize(l1.first.cross(l1.second));
    auto l2 = normalize(mg.lines[line2].component);
    Vec3 n2 = normalize(l2.first.cross(l2.second));
    if (AngleBetweenUndirected(n1, n2) < DegreesToRadians(3)) {
      continue;
    }

    if (AngleBetweenDirected(l1.first, l1.second) <
            minAngleSizeOfLineInTJunction ||
        AngleBetweenDirected(l2.first, l2.second) <
            minAngleSizeOfLineInTJunction) {
      continue;
    }

    int hline = -1, vline = -1;
    double lambda1 = 0, lambda2 = 0;
    DistanceBetweenTwoLines(l1.ray(), l2.ray(), &lambda1, &lambda2);
    const double shrinkRatio = lambdaShrinkForHLineDetectionInTJunction,
                 shrinkRatio2 = lambdaShrinkForVLineDetectionInTJunction;
    if (lambda1 < (1.0 - shrinkRatio) && lambda1 > shrinkRatio &&
        (lambda2 < -shrinkRatio2 || lambda2 > (1.0 + shrinkRatio2))) {
      hline = line1;
      vline = line2;
    } else if (lambda2 < (1.0 - shrinkRatio) && lambda2 > shrinkRatio &&
               (lambda1 < -shrinkRatio2 || lambda1 > (1.0 + shrinkRatio2))) {
      hline = line2;
      vline = line1;
    } else {
      continue;
    }

    // Q: what if three lines form a T structure, and the center lies between
    // the two hlines?
    // A: if the two hlines are close and are colinear, then they should have
    // been merged after the line extraction step

    tjunctionLines.emplace_back(hline, vline);
    auto vlcenter = normalize(mg.lines[vline].component.center());
    auto &hl = mg.lines[hline].component;
    auto hlright = hl.first.cross(hl.second);
    bool vlIsOnRightOfHl = (vlcenter - hl.first).dot(hlright) > 0;
    tjunctionVLineLiesOnTheRightOfHLine.push_back(vlIsOnRightOfHl);
  }

  for (int i = 0; i < tjunctionLines.size(); i++) {
    auto &labelCosts = line2labelCost[tjunctionLines[i].first];
    auto &vl = mg.lines[tjunctionLines[i].second].component;
    double vllen = AngleBetweenDirected(vl.first, vl.second);
    double weight = Gaussian(vllen, DegreesToRadians(10)) * DegreesToRadians(5);
    if (tjunctionVLineLiesOnTheRightOfHLine[i]) {
      labelCosts.disconnectLeftConnectRight += vllen;
    } else {
      labelCosts.connectLeftDisconnectRight += vllen;
    }
  }

  if (false) {
    auto pim = PrintPIGraph(
        mg, ConstantFunctor<gui::Color>(gui::White),
        [&line2labelCost, &mg](int lp) { return gui::Transparent; },
        ConstantFunctor<gui::Color>(gui::LightGray), 2, 3);
    const auto drawLine = [&mg, &pim](
        const Line3 &line, const gui::Color &color, const std::string &text) {
      double angle = AngleBetweenDirected(line.first, line.second);
      std::vector<Pixel> ps;
      for (double a = 0.0; a <= angle; a += 0.01) {
        ps.push_back(ToPixel(mg.view.camera.toScreen(
            RotateDirection(line.first, line.second, a))));
      }
      for (int i = 1; i < ps.size(); i++) {
        auto p1 = ps[i - 1];
        auto p2 = ps[i];
        if (Distance(p1, p2) >= pim.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, pim.cols, pim.rows), p1, p2);
        cv::line(pim, p1, p2, (cv::Scalar)color / 255.0, 2);
      }
      cv::circle(pim, ps.back(), 3.0, (cv::Scalar)color / 255.0, 2);
      cv::putText(pim, text, ps.back(), 1, 1.0, color);
    };
    auto randctable =
        gui::CreateRandomColorTableWithSize(tjunctionLines.size());
    for (int line = 0; line < line2labelCost.size(); line++) {
      auto &labelCosts = line2labelCost[line];
      double connectLeftCost = labelCosts.connectLeftConnectRight +
                               labelCosts.connectLeftDisconnectRight;
      double connectRightCost = labelCosts.connectLeftConnectRight +
                                labelCosts.disconnectLeftConnectRight;
      double allCost = labelCosts.connectLeftConnectRight +
                       labelCosts.connectLeftDisconnectRight +
                       labelCosts.disconnectLeftConnectRight +
                       labelCosts.disconnectAll;
      gui::Color color = Vec3(0.0, connectRightCost, connectLeftCost) / allCost;
      drawLine(mg.lines[line].component, color * 0.5, std::to_string(line));
    }
    gui::AsCanvas(pim).show();
  }

  // make factors
  // data term
  for (int line = 0; line < mg.nlines(); line++) {
    auto vh = line2vh[line];
    if (vh == -1) {
      continue;
    }
    int fc = fg.addFactorCategory(
        [&line2labelCost, &mg, &line2allowedLabels,
         line](const std::vector<int> &varlabels, void *givenData) -> double {
          size_t nvar = varlabels.size();
          assert(nvar == 1);
          auto &labelCosts = line2labelCost[line];

          double costSum = labelCosts.connectLeftConnectRight +
                           labelCosts.connectLeftDisconnectRight +
                           labelCosts.disconnectLeftConnectRight +
                           labelCosts.disconnectAll /* + 1e-3*/;

          const LineLabel &label = line2allowedLabels[line][varlabels[0]];

          double ret = 0.0;
          if (label.connectLeft && label.connectRight) {
            ret += labelCosts.connectLeftConnectRight;
          } else if (label.connectLeft && !label.connectRight) {
            ret += labelCosts.connectLeftDisconnectRight;
          } else if (!label.connectLeft && label.connectRight) {
            ret += labelCosts.disconnectLeftConnectRight;
          } else if (!label.connectLeft && !label.connectRight) {
            ret += labelCosts.disconnectAll;
          }

          return ret / costSum * 1.0;
        },
        1.0);
    fg.addFactor(fc, {vh});
  }

  // smooth term
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (!mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1, line2;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    auto vh1 = line2vh[line1], vh2 = line2vh[line2];
    if (vh1 == -1 || vh2 == -1) {
      continue;
    }
    auto &l1 = mg.lines[line1].component;
    auto &l2 = mg.lines[line2].component;
    auto nearest = DistanceBetweenTwoLines(l1, l2);
    double angleDist = AngleBetweenDirected(nearest.second.first.position,
                                            nearest.second.second.position);
    double weight = Gaussian(angleDist, DegreesToRadians(5));
    bool sameDirection =
        l1.first.cross(l1.second).dot(l2.first.cross(l2.second)) > 0;

    int fc = fg.addFactorCategory(
        [&line2allowedLabels, line1, line2, sameDirection,
         weight](const std::vector<int> &varlabels, void *givenData) -> double {
          size_t nvar = varlabels.size();
          assert(nvar == 2);
          const LineLabel &label1 = line2allowedLabels[line1][varlabels[0]];
          const LineLabel &label2 = line2allowedLabels[line2][varlabels[1]];
          if ((sameDirection && label1 != label2) ||
              (!sameDirection && label1 != label2.leftRightSwapped())) {
            return weight;
          }
          return 0.0;
        },
        1.0);
    fg.addFactor(fc, {vh1, vh2});
  }

  // solve
  std::cout << "solving factor graph" << std::endl;

  std::vector<int> bestLabels;
  double minEnergy = std::numeric_limits<double>::infinity();
  fg.solve(20, 10, [&bestLabels,
                    &minEnergy](int epoch, double energy, double denergy,
                                const std::vector<int> &results) -> bool {
    std::cout << "epoch: " << epoch << "\t energy: " << energy << std::endl;
    if (energy < minEnergy) {
      bestLabels = results;
      minEnergy = energy;
    }
    if (denergy / std::max(energy, 1.0) >= 1e-3) {
      return false;
    }
    return true;
  });

  if (true) {
    auto pim = PrintPIGraph(
        mg,
        [&line2nearbySegsWithOnLeftFlag, &line2allowedLabels, &line2vh,
         &bestLabels, &mg](int seg) -> gui::Color {
          return gui::White;
          // int line = 960;
          bool isSomeBackSeg = false;
          bool isSomeFrontSeg = false;
          for (int line = 0; line < mg.nlines(); line++) {
            auto vh = line2vh[line];
            if (vh == -1) {
              continue;
            }
            auto &lineLabel = line2allowedLabels[line][bestLabels[vh]];
            if (lineLabel.connectLeft && lineLabel.connectRight) {
              continue;
            }
            if (!lineLabel.connectLeft && !lineLabel.connectRight) {
              continue;
            }
            auto &nearbySegs = line2nearbySegsWithOnLeftFlag[line];
            if (Contains(nearbySegs, seg)) {
              if (nearbySegs.at(seg)) { // left
                (lineLabel.connectLeft ? isSomeFrontSeg : isSomeBackSeg) = true;
                continue;
              } else {
                (lineLabel.connectRight ? isSomeFrontSeg : isSomeBackSeg) =
                    true;
                continue;
              }
            }
          }
          if (isSomeFrontSeg && !isSomeBackSeg) {
            return gui::Gray;
          }
          if (isSomeBackSeg && !isSomeFrontSeg) {
            return gui::Black;
          }
          if (isSomeBackSeg && isSomeFrontSeg) {
            return gui::Red;
          }
          return gui::White;
        },
        [&line2labelCost, &mg](int lp) { return gui::Transparent; },
        ConstantFunctor<gui::Color>(gui::LightGray), 2, 3);
    auto drawLine = [&mg, &pim](const Line3 &line, const std::string &text,
                                gui::Color &color) {
      double angle = AngleBetweenDirected(line.first, line.second);
      std::vector<Pixel> ps;
      for (double a = 0.0; a <= angle; a += 0.01) {
        ps.push_back(ToPixel(mg.view.camera.toScreen(
            RotateDirection(line.first, line.second, a))));
      }
      for (int i = 1; i < ps.size(); i++) {
        auto p1 = ps[i - 1];
        auto p2 = ps[i];
        if (Distance(p1, p2) >= pim.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, pim.cols, pim.rows), p1, p2);
        cv::line(pim, p1, p2, (cv::Scalar)color / 255.0, 2);
      }
      cv::circle(pim, ps.back(), 2.0, (cv::Scalar)color / 255.0, 2);
      cv::putText(pim, text, ps.back(), 1, 0.7, color);
    };
    for (int line = 0; line < line2labelCost.size(); line++) {
      auto vh = line2vh[line];
      if (vh == -1) {
        continue;
      }
      // if (line != 94 && line != 1006) {
      //    continue;
      //}
      const LineLabel &label = line2allowedLabels[line][bestLabels[vh]];
      if (label.connectLeft && !label.connectRight) {
        drawLine(mg.lines[line].component, std::to_string(line),
                 gui::Color(gui::Green));
      } else if (!label.connectLeft && label.connectRight) {
        drawLine(mg.lines[line].component, std::to_string(line),
                 gui::Color(gui::Blue));
      } else if (!label.connectLeft && !label.connectRight) {
        drawLine(mg.lines[line].component, std::to_string(line),
                 gui::Color(gui::Red));
      } else {
        drawLine(mg.lines[line].component, std::to_string(line),
                 gui::Color(gui::Black));
      }
    }
    gui::AsCanvas(pim).show(0, "line labels");
  }

  // fill back labels
  // initialize all seg-seg, seg-line and line-line relations
  for (int bp = 0; bp < mg.nbndPieces(); bp++) {
    mg.bndPiece2segRelation[bp] = SegRelation::Connected;
  }
  for (int lp = 0; lp < mg.nlinePieces(); lp++) {
    mg.linePiece2segLineRelation[lp] = SegLineRelation::Attached;
  }
  for (int lr = 0; lr < mg.nlineRelations(); lr++) {
    mg.lineRelations[lr] = LineRelation::Attached;
  }

  auto &bpsegrelation1102 = mg.bndPiece2segRelation[1102];

  // set relations according to the occluding lines
  std::map<int, LineLabel> cutline2label;
  for (int line = 0; line < mg.nlines(); line++) {
    auto vh = line2vh[line];
    if (vh == -1) {
      continue;
    }
    const LineLabel &label = line2allowedLabels[line][bestLabels[vh]];
    if (label.connectLeft && label.connectRight) {
      continue;
    }
    if (!label.connectLeft && !label.connectRight) {
      for (int lp : mg.line2linePieces[line]) {
        mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
      }
      continue;
    }

    cutline2label.emplace(line, label);
    std::set<int> leftSegs, rightSegs;
    for (auto &pp : line2nearbySegsWithOnLeftFlag[line]) {
      int seg = pp.first;
      bool onLeft = pp.second;
      if (onLeft) {
        leftSegs.insert(seg);
      } else {
        rightSegs.insert(seg);
      }
    }

    // disconnect line-seg
    if (label.connectLeft && !label.connectRight) {
      for (int lp : mg.line2linePieces[line]) {
        int seg = mg.linePiece2seg[lp];
        if (seg != -1 && Contains(rightSegs, seg)) {
          mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
        }
      }
    } else if (!label.connectLeft && label.connectRight) {
      for (int lp : mg.line2linePieces[line]) {
        int seg = mg.linePiece2seg[lp];
        if (seg != -1 && Contains(leftSegs, seg)) {
          mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
        }
      }
    }

    // disconnect seg-seg
    static const double affectSegRangeRatio = 1.0;
    if (label.connectLeft && !label.connectRight) { // connect left side only
      for (int frontSeg : leftSegs) {
        for (int bnd : mg.seg2bnds[frontSeg]) {
          int anotherSeg = mg.bnd2segs[bnd].first;
          if (anotherSeg == frontSeg) {
            anotherSeg = mg.bnd2segs[bnd].second;
          }
          if (Contains(rightSegs, anotherSeg)) {
            bool frontSegIsOnBndLeft = frontSeg == mg.bnd2segs[bnd].first;
            if (frontSegIsOnBndLeft) {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::LeftIsFront;
                }
              }
            } else {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::RightIsFront;
                }
              }
            }
          }
        }
      }
    } else if (!label.connectLeft &&
               label.connectRight) { // connect right side only
      for (int frontSeg : rightSegs) {
        for (int bnd : mg.seg2bnds[frontSeg]) {
          int anotherSeg = mg.bnd2segs[bnd].first;
          if (anotherSeg == frontSeg) {
            anotherSeg = mg.bnd2segs[bnd].second;
          }
          if (Contains(leftSegs, anotherSeg)) {
            bool frontSegIsOnBndLeft = frontSeg == mg.bnd2segs[bnd].first;
            if (frontSegIsOnBndLeft) {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::LeftIsFront;
                }
              }
            } else {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::RightIsFront;
                }
              }
            }
          }
        }
      }
    }

    // disconnect those escaped...
    for (int lp : mg.line2linePieces[line]) {
      int bp = mg.linePiece2bndPiece[lp];
      if (bp != -1) {
        int seg1, seg2;
        std::tie(seg1, seg2) = mg.bnd2segs[mg.bndPiece2bnd[bp]];
        bool seg1left = Contains(leftSegs, seg1);
        bool seg1right = Contains(rightSegs, seg1);
        bool seg2left = Contains(leftSegs, seg2);
        bool seg2right = Contains(rightSegs, seg2);
        if (mg.bndPiece2segRelation[bp] != SegRelation::Connected) {
          continue;
        }
        if (label.connectLeft && !label.connectRight) {
          if (seg1left && seg2right) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else if (seg1right && seg2left) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        } else if (!label.connectLeft && label.connectRight) {
          if (seg1left && seg2right) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else if (seg1right && seg2left) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        }
      }
    }

    // disconnect line-line
    for (int lr : mg.line2lineRelations[line]) {
      int anotherLine = mg.lineRelation2lines[lr].first;
      if (anotherLine == line) {
        anotherLine = mg.lineRelation2lines[lr].second;
      }
      auto anotherLineCenter =
          normalize(mg.lines[anotherLine].component.center());
      auto &thisLine = mg.lines[line].component;
      auto thisLineRightSide = thisLine.first.cross(thisLine.second);
      bool anotherLineIsOnTheRightSide =
          (anotherLineCenter - thisLine.first).dot(thisLineRightSide) > 0;
      if (!label.connectLeft && !anotherLineIsOnTheRightSide ||
          !label.connectRight && anotherLineIsOnTheRightSide) {
        mg.lineRelations[lr] = LineRelation::Detached;
      }
    }
  }

  // cut all other relations using cutlines
  if (true) {
    RTreeMap<Box3, int> cutLinePiecesTree;
    static const double cutLinePieceAngle = DegreesToRadians(3);
    for (auto &lineLabelPair : cutline2label) {
      int line = lineLabelPair.first;
      auto &l = mg.lines[line].component;
      double spanAngle = AngleBetweenDirected(l.first, l.second);
      Vec3 lastDir = l.first;
      for (double a = cutLinePieceAngle; a <= spanAngle;
           a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(l.first, l.second, a);
        Line3 linePieceInst(normalize(lastDir), normalize(dir));
        cutLinePiecesTree.emplace(
            BoundingBox(linePieceInst).expand(cutLinePieceAngle), line);
        lastDir = dir;
      }
    }
    const auto isCut = [&cutLinePiecesTree,
                        &mg](const Line3 &connection) -> bool {
      bool isCutBySomeLine = false;
      double spanAngle =
          AngleBetweenDirected(connection.first, connection.second);
      for (double a = 0.0; a <= spanAngle; a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(connection.first, connection.second, a);
        cutLinePiecesTree.search(
            BoundingBox(normalize(dir)).expand(3 * cutLinePieceAngle),
            [&connection, &isCutBySomeLine,
             &mg](const std::pair<Box3, int> &cutLinePieceInst) {
              auto &cutLine = mg.lines[cutLinePieceInst.second].component;
              Vec3 vertDir = normalize(cutLine.first.cross(cutLine.second));
              if (connection.first.dot(vertDir) *
                      connection.second.dot(vertDir) >
                  1e-5) {
                return true;
              }
              auto interp = normalize(
                  Intersection(connection.ray(), Plane3(Origin(), vertDir)));
              if (cutLine.first.cross(interp).dot(
                      cutLine.second.cross(interp)) > 1e-5) {
                return true;
              }
              isCutBySomeLine = true;
              return false;
            });
        if (isCutBySomeLine) {
          break;
        }
      }
      return isCutBySomeLine;
    };

    // line-line
    for (int line = 0; line < mg.nlines(); line++) {
      if (Contains(cutline2label, line)) {
        continue;
      }
      // seg-bnd-bndpiece-line
      // seg-line
      for (int lp : mg.line2linePieces[line]) {
        if (mg.linePiece2segLineRelation[lp] == SegLineRelation::Detached) {
          continue;
        }
        int bp = mg.linePiece2bndPiece[lp];
        if (bp != -1) {
          if (mg.bndPiece2segRelation[bp] != SegRelation::Connected &&
              mg.bndPiece2segRelation[bp] != SegRelation::Unknown) {
            continue;
          }
          int seg1, seg2;
          std::tie(seg1, seg2) = mg.bnd2segs[mg.bndPiece2bnd[bp]];
          bool cutRelationToLeft = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg1])));
          bool cutRelationToRight = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg2])));
          if (cutRelationToLeft && !cutRelationToRight) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else if (!cutRelationToLeft && cutRelationToRight) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else if (cutRelationToLeft && cutRelationToRight) {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        } else {
          int seg = mg.linePiece2seg[lp];
          assert(seg != -1);
          bool cutRelation = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg])));
          if (cutRelation) {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        }
      }
      // line-line
      for (int lr : mg.line2lineRelations[line]) {
        int anotherLine = mg.lineRelation2lines[lr].first;
        if (anotherLine == line) {
          anotherLine = mg.lineRelation2lines[lr].second;
        }
        auto &line1 = mg.lines[line].component;
        auto &line2 = mg.lines[anotherLine].component;
        auto nearest = DistanceBetweenTwoLines(line1, line2).second;
        bool cutRelation = isCut(
            normalize(Line3(nearest.first.position, nearest.second.position)));
        if (line == 94 && anotherLine == 1006 ||
            line == 1006 && anotherLine == 94) {
          std::cout << "cutRelation of 94 and 1006 is " << cutRelation
                    << std::endl;
        }
        if (cutRelation) {
          mg.lineRelations[lr] = LineRelation::Detached;
        }
      }
    }
  }
}

std::vector<LineSidingWeight> ComputeLinesSidingWeights(
    const PIGraph<PanoramicCamera> &mg,
    double minAngleSizeOfLineInTJunction /*= DegreesToRadians(3)*/,
    double lambdaShrinkForHLineDetectionInTJunction /*= 0.2*/,
    double lambdaShrinkForVLineDetectionInTJunction /*= 0.1*/,
    double angleSizeForPixelsNearLines /*= DegreesToRadians(2)*/,
    std::vector<std::map<int, double>> *line2leftSegsWithWeightPtr,
    std::vector<std::map<int, double>> *line2rightSegsWithWeightPtr) {

  std::vector<std::map<int, double>> line2leftSegsWithWeight(mg.nlines());
  std::vector<std::map<int, double>> line2rightSegsWithWeight(mg.nlines());

  for (int line = 0; line < mg.nlines(); line++) {
    auto &l = mg.lines[line].component;
    int claz = mg.lines[line].claz;
    if (claz == -1) {
      continue;
    }
    for (int vpid = 0; vpid < mg.vps.size(); vpid++) {
      if (vpid == claz) {
        continue;
      }
      Vec3 vp = mg.vps[vpid];
      if (vp.dot(normalize(l.center())) < 0) {
        vp = -vp;
      }

      Vec3 lineRight = l.first.cross(l.second);
      bool onLeft = (vp - l.first).dot(lineRight) < 0;

      double lineAngleToVP = std::min(AngleBetweenDirected(l.first, vp),
                                      AngleBetweenDirected(l.second, vp));
      double sweepAngle =
          std::min(angleSizeForPixelsNearLines, lineAngleToVP - 1e-4);
      std::vector<Vec3> sweepQuad = {normalize(l.first), normalize(l.second),
                                     RotateDirection(l.second, vp, sweepAngle),
                                     RotateDirection(l.first, vp, sweepAngle)};
      Vec3 z = normalize(l.center());
      Vec3 y = normalize(normalize(l).direction());
      Vec3 x = normalize(y.cross(z));
      const double focal = mg.view.camera.focal() * 1.2;
      int w = std::ceil(tan(sweepAngle + 0.01) * focal * 2 * 1.5);
      int h = std::ceil(
          (2 * tan(AngleBetweenDirected(l.first, l.second) / 2.0) + 0.01) *
          focal * 1.5);
      PerspectiveCamera pc(w, h, Point2(w / 2.0, h / 2.0), focal, Origin(), z,
                           y);

      Imagei sampledSegs = MakeCameraSampler(pc, mg.view.camera)(mg.segs);

      std::vector<Point2i> quadProjs(4);
      for (int i = 0; i < 4; i++) {
        quadProjs[i] = pc.toScreen(sweepQuad[i]);
        // assert(IsBetween(quadProjs[i][0], 0, w));
        // assert(IsBetween(quadProjs[i][1], 0, h));
        quadProjs[i][0] = BoundBetween(quadProjs[i][0], 0, w);
        quadProjs[i][1] = BoundBetween(quadProjs[i][1], 0, h);
      }
      Imageub mask(pc.screenSize(), false);
      cv::fillConvexPoly(mask, quadProjs, true);

      for (auto it = sampledSegs.begin(); it != sampledSegs.end(); ++it) {
        if (!mask(it.pos())) {
          continue;
        }
        int seg = *it;
        double pixelDistToEyeSquared =
            Square(Distance(ecast<double>(it.pos()), pc.principlePoint())) +
            focal * focal;
        (onLeft ? line2leftSegsWithWeight
                : line2rightSegsWithWeight)[line][seg] +=
            1.0 * focal * focal / pixelDistToEyeSquared;
      }
    }
  }

  if (line2leftSegsWithWeightPtr) {
    *line2leftSegsWithWeightPtr = line2leftSegsWithWeight;
  }
  if (line2rightSegsWithWeightPtr) {
    *line2rightSegsWithWeightPtr = line2rightSegsWithWeight;
  }

  // collect lines' nearby tjunction legs
  // {line-lineRelation-line} -> linePiece -> bndPiece, occlusion hints
  // suggested by line's t-junction
  std::vector<std::pair<int, int>> tjunctionLines; // [hline, vline]
  std::vector<bool> tjunctionVLineLiesOnTheRightOfHLine;
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1 = -1, line2 = -1;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    // whether this forms a t-junction?
    if (mg.lines[line1].claz == mg.lines[line2].claz) {
      continue;
    }
    auto l1 = normalize(mg.lines[line1].component);
    Vec3 n1 = normalize(l1.first.cross(l1.second));
    auto l2 = normalize(mg.lines[line2].component);
    Vec3 n2 = normalize(l2.first.cross(l2.second));
    if (AngleBetweenUndirected(n1, n2) < DegreesToRadians(3)) {
      continue;
    }

    if (AngleBetweenDirected(l1.first, l1.second) <
            minAngleSizeOfLineInTJunction ||
        AngleBetweenDirected(l2.first, l2.second) <
            minAngleSizeOfLineInTJunction) {
      continue;
    }

    int hline = -1, vline = -1;
    double lambda1 = 0, lambda2 = 0;
    DistanceBetweenTwoLines(l1.ray(), l2.ray(), &lambda1, &lambda2);
    const double shrinkRatio = lambdaShrinkForHLineDetectionInTJunction,
                 shrinkRatio2 = lambdaShrinkForVLineDetectionInTJunction;
    if (lambda1 < (1.0 - shrinkRatio) && lambda1 > shrinkRatio &&
        (lambda2 < -shrinkRatio2 || lambda2 > (1.0 + shrinkRatio2))) {
      hline = line1;
      vline = line2;
    } else if (lambda2 < (1.0 - shrinkRatio) && lambda2 > shrinkRatio &&
               (lambda1 < -shrinkRatio2 || lambda1 > (1.0 + shrinkRatio2))) {
      hline = line2;
      vline = line1;
    } else {
      continue;
    }

    // Q: what if three lines form a T structure, and the center lies between
    // the two hlines?
    // A: if the two hlines are close and are colinear, then they should have
    // been merged after the line extraction step

    tjunctionLines.emplace_back(hline, vline);
    auto vlcenter = normalize(mg.lines[vline].component.center());
    auto &hl = mg.lines[hline].component;
    auto hlright = hl.first.cross(hl.second);
    bool vlIsOnRightOfHl = (vlcenter - hl.first).dot(hlright) > 0;
    tjunctionVLineLiesOnTheRightOfHLine.push_back(vlIsOnRightOfHl);
  }
  std::vector<std::array<int, 2>> tjunctionLeftRightNum(
      mg.nlines(), std::array<int, 2>{{0, 0}});
  for (int i = 0; i < tjunctionLines.size(); i++) {
    int hline = tjunctionLines[i].first;
    int vline = tjunctionLines[i].second;
    if (tjunctionVLineLiesOnTheRightOfHLine[i]) {
      tjunctionLeftRightNum[hline][1] += 1;
    } else {
      tjunctionLeftRightNum[hline][0] += 1;
    }
  }

  core::FactorGraph fg;

  std::cout << "building factor graph" << std::endl;

  std::vector<std::array<int, 2>> line2vhConnectLeftRight(mg.nlines(),
                                                          {{-1, -1}});
  static const int LineLabelDisconnect = 0;
  static const int LineLabelConnect = 1;

  std::map<int, int> vh2line;
  for (int line = 0; line < mg.nlines(); line++) {
    if (mg.lines[line].claz == -1) {
      continue;
    }
    auto &l = mg.lines[line].component;
    auto vhConnectLeft = fg.addVar(
        fg.addVarCategory(2, AngleBetweenDirected(l.first, l.second)));
    line2vhConnectLeftRight[line][0] = vhConnectLeft;
    vh2line[vhConnectLeft] = line;
    auto vhConnectRight = fg.addVar(
        fg.addVarCategory(2, AngleBetweenDirected(l.first, l.second)));
    line2vhConnectLeftRight[line][1] = vhConnectRight;
    vh2line[vhConnectRight] = line;
  }

  // factors preparation

  std::vector<std::array<double, 2>> line2wrongSegWeightSum(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = k == 0;

      if (vh == -1) {
        continue;
      }

      int claz = mg.lines[line].claz;
      auto &l = mg.lines[line].component;
      double lineAngleLen = AngleBetweenDirected(l.first, l.second);
      const auto &lps = mg.line2linePieces[line];

      // orientation consistency term between seg and line
      // consider nearby segs
      double wrongSegWeightSumThisSide = 0.0;
      for (auto &segAndWeight : (vhIsLeft ? line2leftSegsWithWeight
                                          : line2rightSegsWithWeight)[line]) {
        int seg = segAndWeight.first;
        double weight = segAndWeight.second;
        auto &segControl = mg.seg2control[seg];
        if (segControl.orientationClaz != -1 &&
            segControl.orientationClaz == claz) {
          wrongSegWeightSumThisSide += weight;
        }
      }
      line2wrongSegWeightSum[line][k] = wrongSegWeightSumThisSide;
    }
  }

  // the cost if is connected on this side [left, right]
  std::vector<std::array<double, 2>> line2labelConnectCostLeftRight(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});
  // the cost if is disconnected on this side [left, right]
  std::vector<std::array<double, 2>> line2labelDisconnectCostLeftRight(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});

  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = k == 0;

      if (vh == -1) {
        continue;
      }

      int claz = mg.lines[line].claz;
      auto &l = mg.lines[line].component;
      double lineAngleLen = AngleBetweenDirected(l.first, l.second);
      const auto &lps = mg.line2linePieces[line];

      // orientation consistency term between seg and line
      // consider nearby segs
      double wrongSegWeightSumThisSide = line2wrongSegWeightSum[line][k];
      double wrongSegWeightSumThatSide = line2wrongSegWeightSum[line][1 - k];

      // consider lines
      int tjunctionNumThisSide = tjunctionLeftRightNum[line][k];

      // now compute the cost
      double &labelConnectCost = line2labelConnectCostLeftRight[line][k];
      double &labelDisconnectCost = line2labelDisconnectCostLeftRight[line][k];

      labelConnectCost = wrongSegWeightSumThisSide /
                         std::max(wrongSegWeightSumThatSide, 1e-5) * 100;
      labelDisconnectCost = Gaussian(tjunctionNumThisSide, 7.0) * 4.0 + 1.0;
    }
  }

  if (false) {
    auto pim = PrintPIGraph(mg, ConstantFunctor<gui::Color>(gui::White),
                            ConstantFunctor<gui::Color>(gui::Transparent),
                            ConstantFunctor<gui::Color>(gui::Gray), 2, 0);
    auto drawLine = [&mg, &pim](const Line3 &line, const std::string &text,
                                const gui::Color &color, bool withTeeth) {
      double angle = AngleBetweenDirected(line.first, line.second);
      std::vector<Pixel> ps;
      for (double a = 0.0; a <= angle; a += 0.01) {
        ps.push_back(ToPixel(mg.view.camera.toScreen(
            RotateDirection(line.first, line.second, a))));
      }
      for (int i = 1; i < ps.size(); i++) {
        auto p1 = ps[i - 1];
        auto p2 = ps[i];
        if (Distance(p1, p2) >= pim.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, pim.cols, pim.rows), p1, p2);
        cv::line(pim, p1, p2, (cv::Scalar)color / 255.0, 2);
        if (withTeeth && i % 3 == 0) {
          auto teethp =
              ToPixel(RightPerpendicularDirectiion(ecast<double>(p2 - p1))) *
                  2 +
              p1;
          auto tp1 = p1, tp2 = teethp;
          cv::clipLine(cv::Rect(0, 0, pim.cols, pim.rows), tp1, tp2);
          cv::line(pim, tp1, tp2, (cv::Scalar)color / 255.0, 1);
        }
      }
      // cv::circle(pim, ps.back(), 2.0, (cv::Scalar)color / 255.0, 2);
      if (!text.empty()) {
        cv::putText(pim, text, ps.back() + Pixel(5, 0), 1, 0.7, color);
      }
    };
    for (int line = 0; line < mg.nlines(); line++) {
      if (line2labelConnectCostLeftRight[line][0] == 0 &&
          line2labelConnectCostLeftRight[line][1] == 0) {
        continue;
      }
      auto l = mg.lines[line].component;
      std::stringstream ss;
      ss.precision(2);
      ss << line << "-left[" << line2labelConnectCostLeftRight[line][0] << "-"
         << line2labelDisconnectCostLeftRight[line][0] << "]"
         << "-right[" << line2labelConnectCostLeftRight[line][1] << "-"
         << line2labelDisconnectCostLeftRight[line][1] << "]";

      drawLine(l, ss.str(), gui::Black, false);
    }
    gui::AsCanvas(pim).show(0, "line costs");
  }

  // make factors
  // data term
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];
    static const bool vhIsLefts[] = {true, false};

    if (vhs[0] == -1 || vhs[1] == -1) {
      continue;
    }

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = vhIsLefts[k];

      if (vh == -1) {
        continue;
      }
      double labelConnectCost = line2labelConnectCostLeftRight[line][k];
      double labelDisconnectCost = line2labelDisconnectCostLeftRight[line][k];

      int fc = fg.addFactorCategory(
          [labelConnectCost, labelDisconnectCost, &mg,
           line](const std::vector<int> &varlabels, void *givenData) -> double {
            size_t nvar = varlabels.size();
            assert(nvar == 1);
            int label = varlabels[0];
            if (label == LineLabelConnect) {
              return labelConnectCost;
            }
            return labelDisconnectCost;
          },
          1.0);
      fg.addFactor(fc, {vh});
    }

    // punish on both disconnected case
    auto &l = mg.lines[line].component;
    double lineAngleLen = AngleBetweenDirected(l.first, l.second);
    int fc = fg.addFactorCategory(
        [lineAngleLen](const std::vector<int> &varlabels,
                       void *givenData) -> double {
          size_t nvar = varlabels.size();
          assert(nvar == 2);
          int label1 = varlabels[0], label2 = varlabels[1];
          if (label1 == LineLabelDisconnect && label2 == LineLabelDisconnect) {
            return 5.0;
          }
          return 0.0;
        },
        1.0);
    fg.addFactor(fc, {vhs[0], vhs[1]});
  }

  // smooth term
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (!mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1, line2;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    auto vhs1 = line2vhConnectLeftRight[line1],
         vhs2 = line2vhConnectLeftRight[line2];
    if (vhs1[0] == -1 || vhs1[1] == -1 || vhs2[0] == -1 || vhs2[1] == -1) {
      continue;
    }
    auto &l1 = mg.lines[line1].component;
    auto &l2 = mg.lines[line2].component;
    auto nearest = DistanceBetweenTwoLines(l1, l2);
    double angleDist = AngleBetweenDirected(nearest.second.first.position,
                                            nearest.second.second.position);
    auto n1 = l1.first.cross(l1.second);
    auto n2 = l2.first.cross(l2.second);
    double normalAngle = AngleBetweenUndirected(n1, n2);
    bool sameDirection =
        l1.first.cross(l1.second).dot(l2.first.cross(l2.second)) > 0;

    if (!sameDirection) {
      std::swap(vhs2[0], vhs2[1]);
    }

    for (int k = 0; k < 2; k++) {
      auto vh1 = vhs1[k];
      auto vh2 = vhs2[k];
      int fc = fg.addFactorCategory(
          [line1, line2, angleDist, normalAngle](
              const std::vector<int> &varlabels, void *givenData) -> double {
            size_t nvar = varlabels.size();
            assert(nvar == 2);
            int label1 = varlabels[0];
            int label2 = varlabels[1];
            if (label1 != label2) {
              return Gaussian(normalAngle, DegreesToRadians(0.5)) * 10.0;
            }
            return 0.0;
          },
          1.0);
      fg.addFactor(fc, {vh1, vh2});
    }
  }

  // solve
  std::cout << "solving factor graph" << std::endl;

  std::vector<int> bestLabels;
  double minEnergy = std::numeric_limits<double>::infinity();
  fg.solve(5, 10, [&bestLabels,
                   &minEnergy](int epoch, double energy, double denergy,
                               const std::vector<int> &results) -> bool {
    std::cout << "epoch: " << epoch << "\t energy: " << energy << std::endl;
    if (energy < minEnergy) {
      bestLabels = results;
      minEnergy = energy;
    }
    if (denergy / std::max(energy, 1.0) >= 1e-3) {
      return false;
    }
    return true;
  });

  std::vector<LineSidingWeight> lsw(mg.nlines(), LineSidingWeight{0.5, 0.5});
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];
    if (vhs[0] == -1 || vhs[1] == -1) {
      continue;
    }
    auto &lineSidingWeight = lsw[line];
    bool connectLeft = bestLabels[vhs[0]] == LineLabelConnect;
    bool connectRight = bestLabels[vhs[1]] == LineLabelConnect;
    double connectLeftCost = line2labelConnectCostLeftRight[line][0],
           disconnectLeftCost = line2labelDisconnectCostLeftRight[line][0];
    double connectRightCost = line2labelConnectCostLeftRight[line][1],
           disconnectRightCost = line2labelDisconnectCostLeftRight[line][1];
    if (connectLeft && connectRight) {
      lineSidingWeight.leftWeightRatio =
          disconnectLeftCost + connectRightCost + 1.0;
      lineSidingWeight.rightWeightRatio =
          disconnectRightCost + connectLeftCost + 1.0;
      double costSum =
          lineSidingWeight.leftWeightRatio + lineSidingWeight.rightWeightRatio;
      lineSidingWeight.leftWeightRatio /= costSum;
      lineSidingWeight.rightWeightRatio /= costSum;
      assert(!IsInfOrNaN(lineSidingWeight.leftWeightRatio) &&
             !IsInfOrNaN(lineSidingWeight.rightWeightRatio));
    } else if (connectLeft) {
      lineSidingWeight.leftWeightRatio = 1.0;
      lineSidingWeight.rightWeightRatio = 0.0;
    } else if (connectRight) {
      lineSidingWeight.rightWeightRatio = 1.0;
      lineSidingWeight.leftWeightRatio = 0.0;
    } else {
      lineSidingWeight.rightWeightRatio = 0.0;
      lineSidingWeight.leftWeightRatio = 0.0;
    }
  }

  return lsw;
}

std::vector<LineSidingWeight> ComputeLinesSidingWeights2(
    const PIGraph<PanoramicCamera> &mg,
    double minAngleSizeOfLineInTJunction /*= DegreesToRadians(3)*/,
    double lambdaShrinkForHLineDetectionInTJunction /*= 0.2*/,
    double lambdaShrinkForVLineDetectionInTJunction /*= 0.1*/,
    double angleSizeForPixelsNearLines /*= DegreesToRadians(2)*/,
    std::vector<std::map<int, double>> *line2leftSegsWithWeightPtr,
    std::vector<std::map<int, double>> *line2rightSegsWithWeightPtr) {

  std::vector<std::map<int, double>> line2leftSegsWithWeight(mg.nlines());
  std::vector<std::map<int, double>> line2rightSegsWithWeight(mg.nlines());

  for (int line = 0; line < mg.nlines(); line++) {
    auto &l = mg.lines[line].component;
    int claz = mg.lines[line].claz;
    if (claz == -1) {
      continue;
    }
    for (int vpid = 0; vpid < mg.vps.size(); vpid++) {
      if (vpid == claz) {
        continue;
      }
      Vec3 vp = mg.vps[vpid];
      if (vp.dot(normalize(l.center())) < 0) {
        vp = -vp;
      }

      Vec3 lineRight = l.first.cross(l.second);
      bool onLeft = (vp - l.first).dot(lineRight) < 0;

      double lineAngleToVP = std::min(AngleBetweenDirected(l.first, vp),
                                      AngleBetweenDirected(l.second, vp));
      double sweepAngle =
          std::min(angleSizeForPixelsNearLines, lineAngleToVP - 1e-4);
      std::vector<Vec3> sweepQuad = {normalize(l.first), normalize(l.second),
                                     RotateDirection(l.second, vp, sweepAngle),
                                     RotateDirection(l.first, vp, sweepAngle)};
      Vec3 z = normalize(l.center());
      Vec3 y = normalize(normalize(l).direction());
      Vec3 x = normalize(y.cross(z));
      const double focal = mg.view.camera.focal() * 1.2;
      int w = std::ceil(tan(sweepAngle + 0.01) * focal * 2 * 1.5);
      int h = std::ceil(
          (2 * tan(AngleBetweenDirected(l.first, l.second) / 2.0) + 0.01) *
          focal * 1.5);
      PerspectiveCamera pc(w, h, Point2(w / 2.0, h / 2.0), focal, Origin(), z,
                           y);

      Imagei sampledSegs = MakeCameraSampler(pc, mg.view.camera)(mg.segs);

      std::vector<Point2i> quadProjs(4);
      for (int i = 0; i < 4; i++) {
        quadProjs[i] = pc.toScreen(sweepQuad[i]);
        // assert(IsBetween(quadProjs[i][0], 0));
        // assert(IsBetween(quadProjs[i][1], 0));
        quadProjs[i][0] = BoundBetween(quadProjs[i][0], 0, w);
        quadProjs[i][1] = BoundBetween(quadProjs[i][1], 0, h);
      }
      Imageub mask(pc.screenSize(), false);
      cv::fillConvexPoly(mask, quadProjs, true);

      for (auto it = sampledSegs.begin(); it != sampledSegs.end(); ++it) {
        if (!mask(it.pos())) {
          continue;
        }
        int seg = *it;
        double pixelDistToEyeSquared =
            Square(Distance(ecast<double>(it.pos()), pc.principlePoint())) +
            focal * focal;
        (onLeft ? line2leftSegsWithWeight
                : line2rightSegsWithWeight)[line][seg] +=
            1.0 * focal * focal / pixelDistToEyeSquared;
      }
    }
  }

  if (line2leftSegsWithWeightPtr) {
    *line2leftSegsWithWeightPtr = line2leftSegsWithWeight;
  }
  if (line2rightSegsWithWeightPtr) {
    *line2rightSegsWithWeightPtr = line2rightSegsWithWeight;
  }

  // collect lines' nearby tjunction legs
  // {line-lineRelation-line} -> linePiece -> bndPiece, occlusion hints
  // suggested by line's t-junction
  std::vector<std::pair<int, int>> tjunctionLines; // [hline, vline]
  std::vector<bool> tjunctionVLineLiesOnTheRightOfHLine;
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1 = -1, line2 = -1;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    // whether this forms a t-junction?
    if (mg.lines[line1].claz == mg.lines[line2].claz) {
      continue;
    }
    auto l1 = normalize(mg.lines[line1].component);
    Vec3 n1 = normalize(l1.first.cross(l1.second));
    auto l2 = normalize(mg.lines[line2].component);
    Vec3 n2 = normalize(l2.first.cross(l2.second));
    if (AngleBetweenUndirected(n1, n2) < DegreesToRadians(3)) {
      continue;
    }

    if (AngleBetweenDirected(l1.first, l1.second) <
            minAngleSizeOfLineInTJunction ||
        AngleBetweenDirected(l2.first, l2.second) <
            minAngleSizeOfLineInTJunction) {
      continue;
    }

    int hline = -1, vline = -1;
    double lambda1 = 0, lambda2 = 0;
    DistanceBetweenTwoLines(l1.ray(), l2.ray(), &lambda1, &lambda2);
    const double shrinkRatio = lambdaShrinkForHLineDetectionInTJunction,
                 shrinkRatio2 = lambdaShrinkForVLineDetectionInTJunction;
    if (lambda1 < (1.0 - shrinkRatio) && lambda1 > shrinkRatio &&
        (lambda2 < -shrinkRatio2 || lambda2 > (1.0 + shrinkRatio2))) {
      hline = line1;
      vline = line2;
    } else if (lambda2 < (1.0 - shrinkRatio) && lambda2 > shrinkRatio &&
               (lambda1 < -shrinkRatio2 || lambda1 > (1.0 + shrinkRatio2))) {
      hline = line2;
      vline = line1;
    } else {
      continue;
    }

    // Q: what if three lines form a T structure, and the center lies between
    // the two hlines?
    // A: if the two hlines are close and are colinear, then they should have
    // been merged after the line extraction step

    tjunctionLines.emplace_back(hline, vline);
    auto vlcenter = normalize(mg.lines[vline].component.center());
    auto &hl = mg.lines[hline].component;
    auto hlright = hl.first.cross(hl.second);
    bool vlIsOnRightOfHl = (vlcenter - hl.first).dot(hlright) > 0;
    tjunctionVLineLiesOnTheRightOfHLine.push_back(vlIsOnRightOfHl);
  }
  std::vector<std::array<int, 2>> tjunctionLeftRightNum(
      mg.nlines(), std::array<int, 2>{{0, 0}});
  for (int i = 0; i < tjunctionLines.size(); i++) {
    int hline = tjunctionLines[i].first;
    int vline = tjunctionLines[i].second;
    if (tjunctionVLineLiesOnTheRightOfHLine[i]) {
      tjunctionLeftRightNum[hline][1] += 1;
    } else {
      tjunctionLeftRightNum[hline][0] += 1;
    }
  }

  core::FactorGraph fg;

  std::cout << "building factor graph" << std::endl;

  std::vector<std::array<int, 2>> line2vhConnectLeftRight(mg.nlines(),
                                                          {{-1, -1}});
  static const int LineLabelDisconnect = 0;
  static const int LineLabelConnect = 1;

  std::map<int, int> vh2line;
  for (int line = 0; line < mg.nlines(); line++) {
    if (mg.lines[line].claz == -1) {
      continue;
    }
    auto &l = mg.lines[line].component;
    auto vhConnectLeft = fg.addVar(
        fg.addVarCategory(2, AngleBetweenDirected(l.first, l.second)));
    line2vhConnectLeftRight[line][0] = vhConnectLeft;
    vh2line[vhConnectLeft] = line;
    auto vhConnectRight = fg.addVar(
        fg.addVarCategory(2, AngleBetweenDirected(l.first, l.second)));
    line2vhConnectLeftRight[line][1] = vhConnectRight;
    vh2line[vhConnectRight] = line;
  }

  // factors preparation

  std::vector<std::array<double, 2>> line2wrongSegWeightSum(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = k == 0;

      if (vh == -1) {
        continue;
      }

      int claz = mg.lines[line].claz;
      auto &l = mg.lines[line].component;
      double lineAngleLen = AngleBetweenDirected(l.first, l.second);
      const auto &lps = mg.line2linePieces[line];

      // orientation consistency term between seg and line
      // consider nearby segs
      double wrongSegWeightSumThisSide = 0.0;
      for (auto &segAndWeight : (vhIsLeft ? line2leftSegsWithWeight
                                          : line2rightSegsWithWeight)[line]) {
        int seg = segAndWeight.first;
        double weight = segAndWeight.second;
        auto &segControl = mg.seg2control[seg];
        if (segControl.orientationClaz != -1 &&
            segControl.orientationClaz == claz) {
          wrongSegWeightSumThisSide += weight;
        }
      }
      line2wrongSegWeightSum[line][k] = wrongSegWeightSumThisSide;
    }
  }

  // the cost if is connected on this side [left, right]
  std::vector<std::array<double, 2>> line2labelConnectCostLeftRight(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});
  // the cost if is disconnected on this side [left, right]
  std::vector<std::array<double, 2>> line2labelDisconnectCostLeftRight(
      mg.nlines(), std::array<double, 2>{{0.0, 0.0}});

  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = k == 0;

      if (vh == -1) {
        continue;
      }

      int claz = mg.lines[line].claz;
      auto &l = mg.lines[line].component;
      double lineAngleLen = AngleBetweenDirected(l.first, l.second);
      const auto &lps = mg.line2linePieces[line];

      // orientation consistency term between seg and line
      // consider nearby segs
      double wrongSegWeightSumThisSide = line2wrongSegWeightSum[line][k];
      double wrongSegWeightSumThatSide = line2wrongSegWeightSum[line][1 - k];

      // consider lines
      int tjunctionNumThisSide = tjunctionLeftRightNum[line][k];

      // now compute the cost
      double &labelConnectCost = line2labelConnectCostLeftRight[line][k];
      double &labelDisconnectCost = line2labelDisconnectCostLeftRight[line][k];

      labelConnectCost = Square(wrongSegWeightSumThisSide /
                                std::max(wrongSegWeightSumThatSide, 1.0)) *
                         100;
      labelDisconnectCost = Gaussian(tjunctionNumThisSide, 10.0);
    }
  }

  // make factors
  // data term
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];
    static const bool vhIsLefts[] = {true, false};

    if (vhs[0] == -1 || vhs[1] == -1) {
      continue;
    }

    // {left, right}
    for (int k = 0; k < 2; k++) {
      auto vh = vhs[k];
      bool vhIsLeft = vhIsLefts[k];

      if (vh == -1) {
        continue;
      }
      double labelConnectCost = line2labelConnectCostLeftRight[line][k];
      double labelDisconnectCost = line2labelDisconnectCostLeftRight[line][k];

      int fc = fg.addFactorCategory(
          [labelConnectCost, labelDisconnectCost, &mg,
           line](const std::vector<int> &varlabels, void *givenData) -> double {
            assert(varlabels.size() == 1);
            int label = varlabels[0];
            if (label == LineLabelConnect) {
              return labelConnectCost;
            }
            return labelDisconnectCost;
          },
          1.0);
      fg.addFactor(fc, {vh});
    }

    // punish on both disconnected case
    auto &l = mg.lines[line].component;
    double lineAngleLen = AngleBetweenDirected(l.first, l.second);
    int fc = fg.addFactorCategory(
        [lineAngleLen](const std::vector<int> &varlabels,
                       void *givenData) -> double {
          assert(varlabels.size() == 2);
          int label1 = varlabels[0], label2 = varlabels[1];
          if (label1 == LineLabelDisconnect && label2 == LineLabelDisconnect) {
            return 5.0;
          } else if (label1 == LineLabelDisconnect ||
                     label2 == LineLabelDisconnect) {
            return 2.0;
          }
          return 0.0;
        },
        1.0);
    fg.addFactor(fc, {vhs[0], vhs[1]});
  }

  // smooth term
  for (int lineRelation = 0; lineRelation < mg.nlineRelations();
       lineRelation++) {
    if (!mg.lineRelation2IsIncidence[lineRelation]) {
      continue;
    }
    int line1, line2;
    std::tie(line1, line2) = mg.lineRelation2lines[lineRelation];
    auto vhs1 = line2vhConnectLeftRight[line1],
         vhs2 = line2vhConnectLeftRight[line2];
    if (vhs1[0] == -1 || vhs1[1] == -1 || vhs2[0] == -1 || vhs2[1] == -1) {
      continue;
    }
    auto &l1 = mg.lines[line1].component;
    auto &l2 = mg.lines[line2].component;
    auto nearest = DistanceBetweenTwoLines(l1, l2);
    double angleDist = AngleBetweenDirected(nearest.second.first.position,
                                            nearest.second.second.position);
    auto n1 = l1.first.cross(l1.second);
    auto n2 = l2.first.cross(l2.second);
    double normalAngle = AngleBetweenUndirected(n1, n2);
    bool sameDirection =
        l1.first.cross(l1.second).dot(l2.first.cross(l2.second)) > 0;

    if (!sameDirection) {
      std::swap(vhs2[0], vhs2[1]);
    }

    for (int k = 0; k < 2; k++) {
      auto vh1 = vhs1[k];
      auto vh2 = vhs2[k];
      int fc = fg.addFactorCategory(
          [line1, line2, angleDist, normalAngle](
              const std::vector<int> &varlabels, void *givenData) -> double {
            assert(varlabels.size() == 2);
            int label1 = varlabels[0];
            int label2 = varlabels[1];
            if (label1 != label2) {
              return 5;
            }
            return 0.0;
          },
          1.0);
      fg.addFactor(fc, {vh1, vh2});
    }
  }

  // solve
  std::cout << "solving factor graph" << std::endl;

  std::vector<int> bestLabels;
  double minEnergy = std::numeric_limits<double>::infinity();
  fg.solve(5, 10, [&bestLabels,
                   &minEnergy](int epoch, double energy, double denergy,
                               const std::vector<int> &results) -> bool {
    std::cout << "epoch: " << epoch << "\t energy: " << energy << std::endl;
    if (energy < minEnergy) {
      bestLabels = results;
      minEnergy = energy;
    }
    if (denergy / std::max(energy, 1.0) >= 1e-3) {
      return false;
    }
    return true;
  });

  std::vector<LineSidingWeight> lsw(mg.nlines(), LineSidingWeight{0.5, 0.5});
  for (int line = 0; line < mg.nlines(); line++) {
    auto &vhs = line2vhConnectLeftRight[line];
    if (vhs[0] == -1 || vhs[1] == -1) {
      continue;
    }
    auto &lineSidingWeight = lsw[line];
    bool connectLeft = bestLabels[vhs[0]] == LineLabelConnect;
    bool connectRight = bestLabels[vhs[1]] == LineLabelConnect;
    double connectLeftCost = line2labelConnectCostLeftRight[line][0],
           disconnectLeftCost = line2labelDisconnectCostLeftRight[line][0];
    double connectRightCost = line2labelConnectCostLeftRight[line][1],
           disconnectRightCost = line2labelDisconnectCostLeftRight[line][1];
    if (connectLeft && connectRight) {
      lineSidingWeight.leftWeightRatio =
          disconnectLeftCost + connectRightCost + 1.0;
      lineSidingWeight.rightWeightRatio =
          disconnectRightCost + connectLeftCost + 1.0;
      double costSum =
          lineSidingWeight.leftWeightRatio + lineSidingWeight.rightWeightRatio;
      lineSidingWeight.leftWeightRatio /= costSum;
      lineSidingWeight.rightWeightRatio /= costSum;
      assert(!IsInfOrNaN(lineSidingWeight.leftWeightRatio) &&
             !IsInfOrNaN(lineSidingWeight.rightWeightRatio));
    } else if (connectLeft) {
      lineSidingWeight.leftWeightRatio = 1.0;
      lineSidingWeight.rightWeightRatio = 0.0;
    } else if (connectRight) {
      lineSidingWeight.rightWeightRatio = 1.0;
      lineSidingWeight.leftWeightRatio = 0.0;
    } else {
      lineSidingWeight.rightWeightRatio = 0.0;
      lineSidingWeight.leftWeightRatio = 0.0;
    }
  }

  return lsw;
}

std::vector<LineSidingWeight> ComputeLinesSidingWeightsFromAnnotation(
    const PIGraph<PanoramicCamera> &mg, const PILayoutAnnotation &anno,
    double sampleAngleStep, double angleThres, double ratioThres) {

  assert(anno.nfaces() > 0);

  std::vector<bool> occBorder2leftIsFront(anno.nborders(), false);
  RTreeMap<Box3, std::pair<Line3, int>> occBorderSamplesTree;
  for (int border = 0; border < anno.nborders(); border++) {
    if (anno.border2connected[border]) {
      continue;
    }
    auto &cs = anno.border2corners[border];
    auto p1 = anno.corners[cs.first];
    auto p2 = anno.corners[cs.second];
    double spanAngle = AngleBetweenDirected(p1, p2);
    for (double a = 0.0; a < spanAngle; a += sampleAngleStep) {
      Vec3 sample1 = normalize(RotateDirection(p1, p2, a));
      Vec3 sample2 = normalize(RotateDirection(p1, p2, a + sampleAngleStep));
      Line3 piece(sample1, sample2);
      occBorderSamplesTree.emplace(BoundingBox(piece),
                                   std::make_pair(piece, border));
    }

    // get the occluding side of this border
    Line3 borderLine(anno.corners[cs.first], anno.corners[cs.second]);
    std::vector<int> adjFaces;
    for (int face = 0; face < anno.nfaces(); face++) {
      auto &faceCs = anno.face2corners[face];
      for (int k = 0; k < faceCs.size(); k++) {
        int c1 = faceCs[k];
        int c2 = faceCs[(k + 1) % faceCs.size()];
        if (c1 == cs.first && c2 == cs.second ||
            c1 == cs.second && c2 == cs.first) {
          adjFaces.push_back(face);
          break;
        }
      }
    }
    assert(adjFaces.size() == 2);

    Vec3 rightOfBorderLine =
        normalize(borderLine.first.cross(borderLine.second));
    std::vector<Scored<double>> rightDegree2depth;
    for (int face : adjFaces) {
      auto &faceCs = anno.face2corners[face];
      Vec3 normal = anno.face2plane[face].normal;
      assert(normal != Origin());
      Vec3 x, y;
      std::tie(x, y) = ProposeXYDirectionsFromZDirection(normal);
      Vec3 dirInFace;
      TriangulatePolygon(
          faceCs.begin(), faceCs.end(),
          [normal, x, y, &anno](int c) -> Point2 {
            Vec3 corner = anno.corners[c];
            return Point2(corner.dot(x), corner.dot(y));
          },
          [&anno, &cs, &dirInFace](int c1, int c2, int c3) {
            if (Contains({cs.first, cs.second}, c1) +
                    Contains({cs.first, cs.second}, c2) +
                    Contains({cs.first, cs.second}, c3) >=
                2) {
              dirInFace =
                  normalize(anno.corners[c1 + c2 + c3 - cs.first - cs.second]);
            }
          });
      assert(dirInFace != Origin());
      double rightDegree = dirInFace.dot(rightOfBorderLine);
      double depthOnBorderOfFace = norm(Intersection(
          Ray3(Origin(), borderLine.center()), anno.face2plane[face]));
      assert(depthOnBorderOfFace > 0.0);
      rightDegree2depth.push_back(ScoreAs(depthOnBorderOfFace, rightDegree));
    }
    assert(rightDegree2depth.size() == 2);
    std::sort(rightDegree2depth.begin(), rightDegree2depth.end());
    double leftDepth = rightDegree2depth.front().component;
    double rightDepth = rightDegree2depth.back().component;
    occBorder2leftIsFront[border] = leftDepth < rightDepth;
  }

  std::vector<LineSidingWeight> lsw(mg.nlines(), LineSidingWeight{0.5, 0.5});

  for (int line = 0; line < mg.nlines(); line++) {
    auto &l = mg.lines[line].component;
    if (mg.lines[line].claz == -1) {
      continue;
    }
    auto p1 = l.first;
    auto p2 = l.second;
    double spanAngle = AngleBetweenDirected(l.first, l.second);
    auto lineN = normalize(l.first.cross(l.second));

    std::map<int, int> nearbyBorder2count;
    int count = 0;
    for (double a = 0.0; a < spanAngle; a += sampleAngleStep) {
      Vec3 sample1 = normalize(RotateDirection(p1, p2, a));
      Vec3 sample2 = normalize(RotateDirection(p1, p2, a + sampleAngleStep));
      Line3 piece(sample1, sample2);
      occBorderSamplesTree.search(
          BoundingBox(piece).expand(sampleAngleStep * 2),
          [&lineN, &piece, angleThres, &nearbyBorder2count](
              const std::pair<Box3, std::pair<Line3, int>> &bdp) {
            auto &borderPiece = bdp.second.first;
            int border = bdp.second.second;
            auto borderN =
                normalize(borderPiece.first.cross(borderPiece.second));
            auto nearest = DistanceBetweenTwoLines(piece, borderPiece).second;
            double angleDist = AngleBetweenDirected(nearest.first.position,
                                                    nearest.second.position);
            if (angleDist < angleThres) {
              nearbyBorder2count[border]++;
            }
            return true;
          });
      count++;
    }

    int attachedBorder = -1;
    int maxCount = count * ratioThres;
    for (auto &pp : nearbyBorder2count) {
      if (pp.second > maxCount) {
        attachedBorder = pp.first;
      }
    }

    if (attachedBorder == -1) {
      continue;
    }

    Line3 borderLine(anno.corners[anno.border2corners[attachedBorder].first],
                     anno.corners[anno.border2corners[attachedBorder].second]);
    bool borderDirSameWithLine =
        lineN.dot(borderLine.first.cross(borderLine.second)) > 0;
    bool leftIsFront = borderDirSameWithLine
                           ? occBorder2leftIsFront[attachedBorder]
                           : !occBorder2leftIsFront[attachedBorder];

    if (leftIsFront) {
      lsw[line].leftWeightRatio = 1.0;
      lsw[line].rightWeightRatio = 0.0;
    } else {
      lsw[line].leftWeightRatio = 0.0;
      lsw[line].rightWeightRatio = 1.0;
    }
  }

  return lsw;
}

std::vector<std::array<std::set<int>, 2>>
CollectSegsNearLines(const PIGraph<PanoramicCamera> &mg,
                     double angleSizeForPixelsNearLines) {

  int width = mg.segs.cols;
  int height = mg.segs.rows;

  RTreeMap<Vec3, std::pair<Line3, int>> lineSamplesTree;
  for (int i = 0; i < mg.nlines(); i++) {
    auto &line = mg.lines[i].component;
    double spanAngle = AngleBetweenDirected(line.first, line.second);
    for (double a = 0.0; a < spanAngle;
         a += angleSizeForPixelsNearLines / 3.0) {
      Vec3 sample1 = normalize(RotateDirection(line.first, line.second, a));
      Vec3 sample2 = normalize(RotateDirection(
          line.first, line.second, a + angleSizeForPixelsNearLines / 3.0));
      lineSamplesTree.emplace(normalize(sample1 + sample2),
                              std::make_pair(Line3(sample1, sample2), i));
    }
  }

  // collect lines' nearby pixels and segs
  std::vector<std::set<Pixel>> line2nearbyPixels(mg.nlines());
  std::vector<std::map<int, Vec3>> line2nearbySegsWithLocalCenterDir(
      mg.nlines());
  std::vector<std::map<int, bool>> line2nearbySegsWithOnLeftFlag(mg.nlines());
  std::vector<std::map<int, double>> line2nearbySegsWithWeight(mg.nlines());

  for (auto it = mg.segs.begin(); it != mg.segs.end(); ++it) {
    Pixel p = it.pos();
    double weight = cos((p.y - (height - 1) / 2.0) / (height - 1) * M_PI);
    Vec3 dir = normalize(mg.view.camera.toSpace(p));
    int seg = *it;
    lineSamplesTree.search(
        BoundingBox(dir).expand(angleSizeForPixelsNearLines * 3),
        [&mg, &dir, &line2nearbyPixels, &line2nearbySegsWithLocalCenterDir,
         &line2nearbySegsWithWeight, angleSizeForPixelsNearLines, p, seg,
         weight](const std::pair<Vec3, std::pair<Line3, int>> &lineSample) {
          auto line = normalize(mg.lines[lineSample.second.second].component);
          // d(dir, line) < angleSizeForPixelsNearLines && lambda(dir, line) \in
          // [0, 1]
          auto dirOnLine = DistanceFromPointToLine(dir, lineSample.second.first)
                               .second.position;
          double angleDist = AngleBetweenDirected(dir, dirOnLine);
          double lambda = ProjectionOfPointOnLine(dir, line)
                              .ratio; // the projected position on line
          if (angleDist < angleSizeForPixelsNearLines &&
              IsBetween(lambda, 0.0, 1.0)) {
            line2nearbyPixels[lineSample.second.second].insert(p);
            auto &localCenterDir =
                line2nearbySegsWithLocalCenterDir[lineSample.second.second]
                                                 [seg];
            localCenterDir += dir * weight;
            double &segWeight =
                line2nearbySegsWithWeight[lineSample.second.second][seg];
            segWeight +=
                weight *
                Gaussian(
                    lambda - 0.5,
                    0.1); // the closer to the center, the more important it is!
          }
          return true;
        });
  }
  for (int i = 0; i < mg.nlines(); i++) {
    auto &nearbySegsWithLocalCenterDir = line2nearbySegsWithLocalCenterDir[i];
    auto &line = mg.lines[i].component;
    Vec3 lineRight = line.first.cross(line.second);
    for (auto &segWithDir : nearbySegsWithLocalCenterDir) {
      Vec3 centerDir = normalize(segWithDir.second);
      bool onLeft = (centerDir - line.first).dot(lineRight) < 0;
      line2nearbySegsWithOnLeftFlag[i][segWithDir.first] = onLeft;
    }
  }

  std::vector<std::array<std::set<int>, 2>> line2leftRightSegs;
  line2leftRightSegs.resize(mg.nlines());
  for (int line = 0; line < mg.nlines(); line++) {
    for (auto &pp : line2nearbySegsWithOnLeftFlag[line]) {
      int seg = pp.first;
      bool onLeft = pp.second;
      line2leftRightSegs[line][onLeft ? 0 : 1].insert(seg);
    }
  }

  return line2leftRightSegs;
}

void ApplyLinesSidingWeights(
    PIGraph<PanoramicCamera> &mg, const std::vector<LineSidingWeight> &lsw,
    const std::vector<std::array<std::set<int>, 2>> &line2leftRightSegs,
    bool connectSegsOnDanglingLine) {

  // fill back labels
  // initialize all seg-seg, seg-line and line-line relations
  for (int bp = 0; bp < mg.nbndPieces(); bp++) {
    mg.bndPiece2segRelation[bp] = SegRelation::Connected;
  }
  for (int lp = 0; lp < mg.nlinePieces(); lp++) {
    mg.linePiece2segLineRelation[lp] = SegLineRelation::Attached;
  }
  for (int lr = 0; lr < mg.nlineRelations(); lr++) {
    mg.lineRelations[lr] = LineRelation::Attached;
  }

  // set relations according to the occluding lines
  std::map<int, LineLabel> cutline2label;
  for (int line = 0; line < mg.nlines(); line++) {
    auto &lineSidingWeight = lsw[line];
    if (lineSidingWeight.isLineDetached()) {
      for (int lp : mg.line2linePieces[line]) {
        mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
      }
      if (connectSegsOnDanglingLine) {
        continue;
      }
    }
    if (!lineSidingWeight.isOcclusion()) {
      continue;
    }

    cutline2label.emplace(line,
                          LineLabel{lineSidingWeight.leftWeightRatio > 0,
                                    lineSidingWeight.rightWeightRatio > 0});
    const auto &leftSegs = line2leftRightSegs[line][0];
    const auto &rightSegs = line2leftRightSegs[line][1];

    // disconnect line-seg
    if (!lineSidingWeight.connectRight()) {
      for (int lp : mg.line2linePieces[line]) {
        int seg = mg.linePiece2seg[lp];
        if (seg != -1 && Contains(rightSegs, seg)) {
          mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
        }
      }
    } else if (!lineSidingWeight.connectLeft()) {
      for (int lp : mg.line2linePieces[line]) {
        int seg = mg.linePiece2seg[lp];
        if (seg != -1 && Contains(leftSegs, seg)) {
          mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
        }
      }
    }

    // disconnect seg-seg
    static const double affectSegRangeRatio = 1.0;
    if (lineSidingWeight.onlyConnectLeft()) { // connect left side only
      for (int frontSeg : leftSegs) {
        for (int bnd : mg.seg2bnds[frontSeg]) {
          int anotherSeg = mg.bnd2segs[bnd].first;
          if (anotherSeg == frontSeg) {
            anotherSeg = mg.bnd2segs[bnd].second;
          }
          if (Contains(rightSegs, anotherSeg)) {
            bool frontSegIsOnBndLeft = frontSeg == mg.bnd2segs[bnd].first;
            if (frontSegIsOnBndLeft) {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::LeftIsFront;
                }
              }
            } else {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::RightIsFront;
                }
              }
            }
          }
        }
      }
    } else if (lineSidingWeight.onlyConnectRight()) { // connect right side only
      for (int frontSeg : rightSegs) {
        for (int bnd : mg.seg2bnds[frontSeg]) {
          int anotherSeg = mg.bnd2segs[bnd].first;
          if (anotherSeg == frontSeg) {
            anotherSeg = mg.bnd2segs[bnd].second;
          }
          if (Contains(leftSegs, anotherSeg)) {
            bool frontSegIsOnBndLeft = frontSeg == mg.bnd2segs[bnd].first;
            if (frontSegIsOnBndLeft) {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::LeftIsFront;
                }
              }
            } else {
              for (int bndPiece : mg.bnd2bndPieces[bnd]) {
                auto &d1 = mg.bndPiece2dirs[bndPiece].front();
                auto &d2 = mg.bndPiece2dirs[bndPiece].back();
                auto &l = mg.lines[line].component;
                double angleRadius =
                    (AngleBetweenDirected(l.first, l.second) / 2.0) *
                    affectSegRangeRatio;
                if (AngleBetweenDirected(d1, l.center()) <= angleRadius &&
                    AngleBetweenDirected(d2, l.center()) <= angleRadius) {
                  mg.bndPiece2segRelation[bndPiece] = SegRelation::RightIsFront;
                }
              }
            }
          }
        }
      }
    }

    // disconnect those escaped...
    for (int lp : mg.line2linePieces[line]) {
      int bp = mg.linePiece2bndPiece[lp];
      if (bp != -1) {
        int seg1, seg2;
        std::tie(seg1, seg2) = mg.bnd2segs[mg.bndPiece2bnd[bp]];
        bool seg1left = Contains(leftSegs, seg1);
        bool seg1right = Contains(rightSegs, seg1);
        bool seg2left = Contains(leftSegs, seg2);
        bool seg2right = Contains(rightSegs, seg2);
        if (mg.bndPiece2segRelation[bp] != SegRelation::Connected) {
          continue;
        }
        if (lineSidingWeight.onlyConnectLeft()) {
          if (seg1left && seg2right) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else if (seg1right && seg2left) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        } else if (lineSidingWeight.onlyConnectRight()) {
          if (seg1left && seg2right) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else if (seg1right && seg2left) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        }
      }
    }

    // disconnect line-line
    for (int lr : mg.line2lineRelations[line]) {
      int anotherLine = mg.lineRelation2lines[lr].first;
      if (anotherLine == line) {
        anotherLine = mg.lineRelation2lines[lr].second;
      }
      auto anotherLineCenter =
          normalize(mg.lines[anotherLine].component.center());
      auto &thisLine = mg.lines[line].component;
      auto thisLineRightSide = thisLine.first.cross(thisLine.second);
      bool anotherLineIsOnTheRightSide =
          (anotherLineCenter - thisLine.first).dot(thisLineRightSide) > 0;
      if (lineSidingWeight.leftWeightRatio == 0 &&
              !anotherLineIsOnTheRightSide ||
          lineSidingWeight.rightWeightRatio == 0 &&
              anotherLineIsOnTheRightSide) {
        mg.lineRelations[lr] = LineRelation::Detached;
      }
    }
  }

  // cut all other relations using cutlines
  if (true) {
    RTreeMap<Box3, int> cutLinePiecesTree;
    static const double cutLinePieceAngle = DegreesToRadians(3);
    for (auto &lineLabelPair : cutline2label) {
      int line = lineLabelPair.first;
      auto &l = mg.lines[line].component;
      double spanAngle = AngleBetweenDirected(l.first, l.second);
      Vec3 lastDir = l.first;
      for (double a = cutLinePieceAngle; a <= spanAngle;
           a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(l.first, l.second, a);
        Line3 linePieceInst(normalize(lastDir), normalize(dir));
        cutLinePiecesTree.emplace(
            BoundingBox(linePieceInst).expand(cutLinePieceAngle), line);
        lastDir = dir;
      }
    }
    const auto isCut = [&cutLinePiecesTree,
                        &mg](const Line3 &connection) -> bool {
      bool isCutBySomeLine = false;
      double spanAngle =
          AngleBetweenDirected(connection.first, connection.second);
      for (double a = 0.0; a <= spanAngle; a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(connection.first, connection.second, a);
        cutLinePiecesTree.search(
            BoundingBox(normalize(dir)).expand(3 * cutLinePieceAngle),
            [&connection, &isCutBySomeLine,
             &mg](const std::pair<Box3, int> &cutLinePieceInst) {
              auto &cutLine = mg.lines[cutLinePieceInst.second].component;
              Vec3 vertDir = normalize(cutLine.first.cross(cutLine.second));
              if (connection.first.dot(vertDir) *
                      connection.second.dot(vertDir) >
                  1e-5) {
                return true;
              }
              auto interp = normalize(
                  Intersection(connection.ray(), Plane3(Origin(), vertDir)));
              if (cutLine.first.cross(interp).dot(
                      cutLine.second.cross(interp)) > 1e-5) {
                return true;
              }
              isCutBySomeLine = true;
              return false;
            });
        if (isCutBySomeLine) {
          break;
        }
      }
      return isCutBySomeLine;
    };

    // line-line
    for (int line = 0; line < mg.nlines(); line++) {
      if (Contains(cutline2label, line)) {
        continue;
      }
      // seg-bnd-bndpiece-line
      // seg-line
      for (int lp : mg.line2linePieces[line]) {
        if (mg.linePiece2segLineRelation[lp] == SegLineRelation::Detached) {
          continue;
        }
        int bp = mg.linePiece2bndPiece[lp];
        if (bp != -1) {
          if (mg.bndPiece2segRelation[bp] != SegRelation::Connected &&
              mg.bndPiece2segRelation[bp] != SegRelation::Unknown) {
            continue;
          }
          int seg1, seg2;
          std::tie(seg1, seg2) = mg.bnd2segs[mg.bndPiece2bnd[bp]];
          bool cutRelationToLeft = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg1])));
          bool cutRelationToRight = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg2])));
          if (cutRelationToLeft && !cutRelationToRight) {
            mg.bndPiece2segRelation[bp] = SegRelation::RightIsFront;
          } else if (!cutRelationToLeft && cutRelationToRight) {
            mg.bndPiece2segRelation[bp] = SegRelation::LeftIsFront;
          } else if (cutRelationToLeft && cutRelationToRight) {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        } else {
          int seg = mg.linePiece2seg[lp];
          assert(seg != -1);
          bool cutRelation = isCut(normalize(
              Line3(mg.lines[line].component.center(), mg.seg2center[seg])));
          if (cutRelation) {
            mg.linePiece2segLineRelation[lp] = SegLineRelation::Detached;
          }
        }
      }
      // line-line
      for (int lr : mg.line2lineRelations[line]) {
        int anotherLine = mg.lineRelation2lines[lr].first;
        if (anotherLine == line) {
          anotherLine = mg.lineRelation2lines[lr].second;
        }
        auto &line1 = mg.lines[line].component;
        auto &line2 = mg.lines[anotherLine].component;
        auto nearest = DistanceBetweenTwoLines(line1, line2).second;
        bool cutRelation = isCut(
            normalize(Line3(nearest.first.position, nearest.second.position)));
        if (cutRelation) {
          mg.lineRelations[lr] = LineRelation::Detached;
        }
      }
    }
  }
}
}
}