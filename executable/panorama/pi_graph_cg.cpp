#include "pch.hpp"

#include "containers.hpp"
#include "geo_context.hpp"
#include "line_detection.hpp"
#include "segmentation.hpp"

#include "pi_graph_annotation.hpp"
#include "pi_graph_cg.hpp"

namespace pano {
namespace experimental {

namespace {
template <class K, class T, class Pr, class Alloc>
const T &AtOr(const std::map<K, T, Pr, Alloc> &m, const K &key,
              const T &defaultVal) {
  if (Contains(m, key)) {
    return m.at(key);
  } else {
    return defaultVal;
  }
}
inline double AngleDistanceBetweenPointAndRay(const Vec3 &p, const Ray3 &ray) {
  auto p1 = ray.anchor;
  auto p2 = ray.direction;
  auto normal = p1.cross(p2);
  double angle = M_PI_2 - AngleBetweenUndirected(p, normal);
  double angleToAnchor = AngleBetweenDirected(p, ray.anchor);
  return std::min(angle, angleToAnchor);
}
template <class T, int N>
bool IsFuzzyNormalized(const Vec<T, N> &v, T epsilon = 1e-3) {
  return abs(norm(v) - 1.0) < epsilon;
}
int SwappedComponent(const Vec3 &orientation) {
  int maxComp =
      std::max_element(orientation.val, orientation.val + 3,
                       [](double a, double b) { return abs(a) < abs(b); }) -
      orientation.val;
  assert(abs(orientation[maxComp]) > 1e-8);
  return maxComp;
}
}

DenseMatd
PIConstraintGraph::Entity::SupportingPlane::matFromVarsToPlaneCoeffs() const {
  assert(dof == 1 || dof == 2 || dof == 3);
  if (dof == 1) {
    assert(IsFuzzyNormalized(toward));
    auto planeNormal = normalize(toward);
    // variable is 1.0/centerDepth
    //  let the normalized planeNormal = [nx, ny, nz], let normalized center =
    //  [cx, cy, cz]
    //  then the equation should be k nx x + k ny y + k nz z = 1
    //  if the depth of center is Dc, then
    //      k cx Dc nx + k cy Dc ny + k cz Dc nz = 1
    //  and then k = 1.0 / (Dc*dot(n, c))
    //  the equation coefficients thus are
    //      k n* = n* / (Dc*dot(n, c))
    //  therefore the equation coefficients corresponding to the variable
    //  (inverse planeDistanceToOrigin, 1.0/Dc) are
    //      n*/dot(n, c)
    DenseMatd coeffs(3, 1, 0.0);
    for (int i = 0; i < 3; i++) {
      coeffs(i, 0) = planeNormal[i] / planeNormal.dot(center);
    }
    return coeffs;
  } else if (dof == 2) {
    assert(IsFuzzyNormalized(along));
    auto m = normalize(along);
    int sc = SwappedComponent(m); // find a non zero component
    assert(sc == 0 || sc == 1 || sc == 2);

    // original along orientation = [mx, my, mz]
    // let the plane equation be ax + by + cz = 1
    // there should be a * mx + b * my + c * mz = 0

    // if sc = 0, meaning that mx != 0
    // then we can write as: a = b * (- my / mx) + c * (- mz / mx)
    // then [a]   [-my/mx -mz/mx]
    //      [b] = [   1      0  ]*[b]
    //      [c]   [   0      1  ] [c]
    if (sc == 0) {
      DenseMatd coeffs(3, 2, 0.0);
      coeffs(0, 0) = -m[1] / m[0];
      coeffs(0, 1) = -m[2] / m[0];
      coeffs(1, 0) = 1;
      coeffs(1, 1) = 0;
      coeffs(2, 0) = 0;
      coeffs(2, 1) = 1;
      return coeffs;
    }

    // if sc = 1, meaning that my != 0
    // then we can write as: b = a * (- mx / my) + c * (- mz / my)
    // then [a]   [    1     0  ] [a]
    //      [b] = [-mx/my -mz/my]*
    //      [c]   [    0     1  ] [c]
    if (sc == 1) {
      DenseMatd coeffs(3, 2, 0.0);
      coeffs(0, 0) = 1;
      coeffs(0, 1) = 0;
      coeffs(1, 0) = -m[0] / m[1];
      coeffs(1, 1) = -m[2] / m[1];
      coeffs(2, 0) = 0;
      coeffs(2, 1) = 1;
      return coeffs;
    }

    // if sc = 2, meaning that mz != 0
    // then we can write as: c = a * (- mx / mz) + b * (- my / mz)
    // then [a]   [    1     0  ] [a]
    //      [b] = [    0     1  ]*[b]
    //      [c]   [-mx/mz -my/mz]
    /*if (sc == 2)*/ {
      DenseMatd coeffs(3, 2, 0.0);
      coeffs(0, 0) = 1;
      coeffs(0, 1) = 0;
      coeffs(1, 0) = 0;
      coeffs(1, 1) = 1;
      coeffs(2, 0) = -m[0] / m[2];
      coeffs(2, 1) = -m[1] / m[2];
      return coeffs;
    }
  } else /*if (dof == 3)*/ {
    return DenseMatd::eye(3, 3);
  }
}

PIConstraintGraph::Entity::SupportingPlane::SupportingPlane() : dof(0) {}

PIConstraintGraph::Entity::SupportingPlane::SupportingPlane(
    const SegControl &control, const Vec3 &c, const std::vector<Vec3> &vps) {
  dof = control.dof();
  center = normalize(c);
  if (dof == 1) {
    toward = normalize(vps[control.orientationClaz]);
    if (toward.dot(center) < 0) {
      toward = -toward;
    }
  } else if (dof == 2) {
    along = normalize(vps[control.orientationNotClaz]);
  }
}

PIConstraintGraph::Entity::SupportingPlane::SupportingPlane(
    const Classified<Line3> &line, const std::vector<Vec3> &vps) {
  dof = line.claz != -1 ? 1 : 2;
  center = normalize(line.component.center());
  if (dof == 1) {
    const Vec3 &lineDirection = vps[line.claz];
    auto linePerp = normalize(line.component.center().cross(lineDirection));
    assert(abs(norm(linePerp) - 1.0) < 1e-2);
    auto linePlaneNormal = lineDirection.cross(linePerp);
    assert(IsFuzzyPerpendicular(linePlaneNormal, vps[line.claz]));
    toward = normalize(linePlaneNormal);
  } else {
    const Vec3 &maybeLineDirection = line.component.direction();
    auto linePerp = line.component.center().cross(maybeLineDirection);
    assert(IsFuzzyPerpendicular(linePerp, maybeLineDirection));
    along = normalize(linePerp);
  }
}

PIConstraintGraph
BuildPIConstraintGraph(const PIGraph<PanoramicCamera> &mg, double minAngleThresForAWideEdge,
                       double weightRatioForCoplanarityWithLines) {

  PIConstraintGraph cg;
  cg.seg2ent.resize(mg.nsegs, -1);
  cg.line2ent.resize(mg.nlines(), -1);

  auto &seg2ent = cg.seg2ent;
  auto &line2ent = cg.line2ent;
  auto &entities = cg.entities;
  auto &constraints = cg.constraints;

  using Entity = PIConstraintGraph::Entity;
  using Constraint = PIConstraintGraph::Constraint;

  // add entities
  // add segs
  for (int i = 0; i < mg.nsegs; i++) {
    if (!mg.seg2control[i].used) {
      continue;
    }

    Entity e;
    e.type = Entity::IsSeg;
    e.id = i;
    e.size = mg.seg2areaRatio[i] * 100;

    auto &control = mg.seg2control[i];
    e.supportingPlane = PIConstraintGraph::Entity::SupportingPlane(
        control, mg.seg2center[i], mg.vps);
    entities.push_back(e);

    int ent = entities.size() - 1;
    seg2ent[i] = ent;
  }
  // add lines
  for (int i = 0; i < mg.nlines(); i++) {
    if (!mg.line2used.empty() && !mg.line2used[i]) {
      continue;
    }

    Entity e;
    e.type = Entity::IsLine;
    e.id = i;
    e.size = AngleBetweenDirected(mg.lines[i].component.first,
                                    mg.lines[i].component.second);

    e.supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.lines[i], mg.vps);
    entities.push_back(e);

    int ent = entities.size() - 1;
    line2ent[i] = ent;
  }

  auto &ent2cons = cg.ent2cons;
  ent2cons.resize(entities.size()); // enttity -> constraints

  // add constraints
  // bndpieces, seg-seg
  for (int i = 0; i < mg.nbndPieces(); i++) {
    if (mg.bndPiece2segRelation[i] != SegRelation::Connected) {
      continue;
    }
    Constraint connect;
    connect.type = Constraint::Connection;
    auto &segPair = mg.bnd2segs[mg.bndPiece2bnd[i]];
    connect.ent1 = seg2ent[segPair.first];
    connect.ent2 = seg2ent[segPair.second];
    double len = mg.bndPiece2length[i];
    connect.weight = len;
    if (len >= minAngleThresForAWideEdge) {
      connect.anchors = {mg.bndPiece2dirs[i].front(),
                         mg.bndPiece2dirs[i].back()};
    } else {
      connect.anchors = {
          normalize(mg.bndPiece2dirs[i].front() + mg.bndPiece2dirs[i].back())};
    }
    constraints.push_back(connect);
    int connectCon = constraints.size() - 1;
    if (connect.ent1 == -1 || connect.ent2 == -1) {
      continue;
    }
    ent2cons[connect.ent1].push_back(connectCon);
    ent2cons[connect.ent2].push_back(connectCon);

    // check whether this bp lies on a line
    if (!mg.bndPiece2linePieces[i].empty() || len < minAngleThresForAWideEdge) {
      // if so, no planarity will be assigned
      continue;
    }

    // the coplanarity constraint if there are no lines on the bndpiece
    // if (mg.bndPiece2linePieces[i].empty())
    {
      Constraint coplanar;
      coplanar.type = Constraint::Coplanarity;
      coplanar.ent1 = connect.ent1;
      coplanar.ent2 = connect.ent2;
      coplanar.weight = len * (mg.bndPiece2linePieces[i].empty()
                                   ? 1.0
                                   : weightRatioForCoplanarityWithLines);
      // coplanar.anchorOrientation = -1;
      constraints.push_back(coplanar);
      int coplanarCon = constraints.size() - 1;
      ent2cons[coplanar.ent1].push_back(coplanarCon);
      ent2cons[coplanar.ent2].push_back(coplanarCon);
    }
  }

  // linepieces, seg-line
  for (int i = 0; i < mg.nlinePieces(); i++) {
    if (mg.linePiece2segLineRelation[i] == SegLineRelation::Detached) {
      continue;
    }
    int bndPiece = mg.linePiece2bndPiece[i];
    int line = mg.linePiece2line[i];
    if (bndPiece == -1) {
      int seg = mg.linePiece2seg[i];
      assert(seg != -1);
      Constraint c;
      c.type = Constraint::Connection;
      c.ent1 = seg2ent[seg];
      c.ent2 = line2ent[line];
      if (c.ent1 == -1 || c.ent2 == -1) {
        continue;
      }
      double len = mg.linePiece2length[i];
      c.weight = len;
      if (len >= minAngleThresForAWideEdge) {
        c.anchors = {mg.linePiece2samples[i].front(),
                     mg.linePiece2samples[i].back()};
      } else {
        c.anchors = {normalize(mg.linePiece2samples[i].front() +
                               mg.linePiece2samples[i].back())};
      }
      constraints.push_back(c);
      int con = constraints.size() - 1;
      ent2cons[c.ent1].push_back(con);
      ent2cons[c.ent2].push_back(con);
    } else {
      int segPair[] = {-1, -1};
      std::tie(segPair[0], segPair[1]) = mg.bnd2segs[mg.bndPiece2bnd[bndPiece]];
      auto segRelation = mg.bndPiece2segRelation[bndPiece];
      bool connected[] = {segRelation == SegRelation::Connected ||
                              segRelation == SegRelation::LeftIsFront,
                          segRelation == SegRelation::Connected ||
                              segRelation == SegRelation::RightIsFront};
      for (int k = 0; k < 2; k++) {
        int seg = segPair[k];
        if (!connected[k]) {
          continue;
        }
        Constraint c;
        c.type = Constraint::Connection;
        c.ent1 = seg2ent[seg];
        c.ent2 = line2ent[line];
        if (c.ent1 == -1 || c.ent2 == -1) {
          continue;
        }
        double len = mg.linePiece2length[i];
        c.weight = len;
        if (len >= minAngleThresForAWideEdge) {
          c.anchors = {mg.linePiece2samples[i].front(),
                       mg.linePiece2samples[i].back()};
        } else {
          c.anchors = {normalize(mg.linePiece2samples[i].front() +
                                 mg.linePiece2samples[i].back())};
        }
        constraints.push_back(c);
        int con = constraints.size() - 1;
        ent2cons[c.ent1].push_back(con);
        ent2cons[c.ent2].push_back(con);
      }
    }
  }

  // linerelations, line-line
  for (int i = 0; i < mg.nlineRelations(); i++) {
    if (mg.lineRelations[i] == LineRelation::Detached) {
      continue;
    }
    Constraint c;
    c.type = Constraint::Connection;
    auto &linePair = mg.lineRelation2lines[i];
    c.ent1 = line2ent[linePair.first];
    c.ent2 = line2ent[linePair.second];
    if (c.ent1 == -1 || c.ent2 == -1) {
      continue;
    }
    c.weight = mg.lineRelation2weight[i];
    c.anchors = {mg.lineRelation2anchor[i]};
    // c.anchorOrientation = -1;
    constraints.push_back(c);
    int con = constraints.size() - 1;
    ent2cons[c.ent1].push_back(con);
    ent2cons[c.ent2].push_back(con);
  }

  cg.cons2enabled.resize(constraints.size(), true);

  return cg;
}

PIConstraintGraph BuildPIConstraintGraph(
    const PIGraph<PanoramicCamera> &mg, const std::vector<LineSidingWeight> &lsw,
    const std::vector<std::array<std::set<int>, 2>> &line2leftRightSegs,
    double minAngleThresForAWideEdge) {

  PIConstraintGraph cg;
  cg.seg2ent.resize(mg.nsegs, -1);
  cg.line2ent.resize(mg.nlines(), -1);

  auto &seg2ent = cg.seg2ent;
  auto &line2ent = cg.line2ent;
  auto &entities = cg.entities;
  auto &constraints = cg.constraints;

  using Entity = PIConstraintGraph::Entity;
  using Constraint = PIConstraintGraph::Constraint;

  // add entities
  // add segs
  for (int i = 0; i < mg.nsegs; i++) {
    Entity e;
    e.type = Entity::IsSeg;
    e.id = i;
    e.size = mg.seg2areaRatio[i] * 100;

    auto &control = mg.seg2control[i];
    e.supportingPlane.dof = mg.seg2control[i].dof();
    e.supportingPlane.center = normalize(mg.seg2center[i]);
    if (e.supportingPlane.dof == 1) {
      e.supportingPlane.toward = normalize(mg.vps[control.orientationClaz]);
      if (e.supportingPlane.toward.dot(e.supportingPlane.center) < 0) {
        e.supportingPlane.toward = -e.supportingPlane.toward;
      }
    } else if (e.supportingPlane.dof == 2) {
      e.supportingPlane.along = normalize(mg.vps[control.orientationNotClaz]);
    } else {
    }

    entities.push_back(e);

    int ent = entities.size() - 1;
    seg2ent[i] = ent;
  }
  // add lines
  for (int i = 0; i < mg.nlines(); i++) {
    Entity e;
    e.type = Entity::IsLine;
    e.id = i;
    e.size = AngleBetweenDirected(mg.lines[i].component.first,
                                    mg.lines[i].component.second);

    e.supportingPlane.dof = mg.lines[i].claz != -1 ? 1 : 2;
    e.supportingPlane.center = normalize(mg.lines[i].component.center());
    if (e.supportingPlane.dof == 1) {
      const Vec3 &lineDirection = mg.vps[mg.lines[i].claz];
      auto linePerp =
          normalize(mg.lines[i].component.center().cross(lineDirection));
      assert(abs(norm(linePerp) - 1.0) < 1e-2);
      auto linePlaneNormal = lineDirection.cross(linePerp);
      assert(IsFuzzyPerpendicular(linePlaneNormal, mg.vps[mg.lines[i].claz]));
      e.supportingPlane.toward = normalize(linePlaneNormal);
    } else {
      const Vec3 &maybeLineDirection = mg.lines[i].component.direction();
      auto linePerp = mg.lines[i].component.center().cross(maybeLineDirection);
      assert(IsFuzzyPerpendicular(linePerp, maybeLineDirection));
      e.supportingPlane.along = normalize(linePerp);
    }

    entities.push_back(e);

    int ent = entities.size() - 1;
    line2ent[i] = ent;
  }

  auto &ent2cons = cg.ent2cons;
  ent2cons.resize(entities.size()); // enttity -> constraints

  std::map<std::pair<int, int>, double> segSeg2weightRatio;
  std::map<std::pair<int, int>, double> lineLine2weightRatio;
  std::map<std::pair<int, int>, double> segLine2weightRatio;

  std::vector<double> bndPiece2weightRatio(mg.nbndPieces(), 0.5);
  std::vector<double> linePiece2weightRatio(mg.nlinePieces(), 0.5);
  std::vector<double> lineRelation2weightRatio(mg.nlineRelations(), 0.5);

  {
    for (int line = 0; line < mg.nlines(); line++) {
      auto &lineSidingWeight = lsw[line];
      double leftSideWR = lineSidingWeight.leftWeightRatio;
      double rightSideWR = lineSidingWeight.rightWeightRatio;
      assert(!IsInfOrNaN(leftSideWR) && !IsInfOrNaN(rightSideWR));
      double crossWR = lineSidingWeight.minWeightRatio();

      const auto &leftSegs = line2leftRightSegs[line][0];
      const auto &rightSegs = line2leftRightSegs[line][1];

      // set seg-line weight ratio
      for (int seg : leftSegs) {
        segLine2weightRatio[std::make_pair(seg, line)] = leftSideWR;
      }
      for (int seg : rightSegs) {
        segLine2weightRatio[std::make_pair(seg, line)] = rightSideWR;
      }

      // set seg-seg weight ratio
      for (int seg1 : leftSegs) {
        for (int seg2 : rightSegs) {
          assert(seg1 != seg2);
          auto segPair = MakeOrderedPair(seg1, seg2);
          segSeg2weightRatio[segPair] =
              std::min(AtOr(segSeg2weightRatio, segPair, 1.0), crossWR);
        }
      }

      // set line-line weight ratio
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

        double wr = anotherLineIsOnTheRightSide ? rightSideWR : leftSideWR;
        auto linePair = MakeOrderedPair(line, anotherLine);
        lineLine2weightRatio[linePair] =
            std::min(AtOr(lineLine2weightRatio, linePair, 1.0), wr);
      }
    }

    // cut other crossing constraints
    RTreeMap<Box3, int> linePiecesTree;
    static const double cutLinePieceAngle = DegreesToRadians(3);
    for (int line = 0; line < mg.nlines(); line++) {
      auto &l = mg.lines[line].component;
      double spanAngle = AngleBetweenDirected(l.first, l.second);
      Vec3 lastDir = l.first;
      for (double a = cutLinePieceAngle; a <= spanAngle;
           a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(l.first, l.second, a);
        Line3 linePieceInst(normalize(lastDir), normalize(dir));
        linePiecesTree.emplace(
            BoundingBox(linePieceInst).expand(cutLinePieceAngle), line);
        lastDir = dir;
      }
    }

    const auto cutWR = [&linePiecesTree, &mg, &lsw](
        const Line3 &connection,
        const std::vector<int> &withdrawedLines) -> double {
      double wr = 1.0;
      double spanAngle =
          AngleBetweenDirected(connection.first, connection.second);
      for (double a = 0.0; a <= spanAngle; a += cutLinePieceAngle) {
        Vec3 dir = RotateDirection(connection.first, connection.second, a);
        linePiecesTree.search(
            BoundingBox(normalize(dir)).expand(3 * cutLinePieceAngle),
            [&connection, &wr, &mg, &lsw,
             &withdrawedLines](const std::pair<Box3, int> &linePieceInst) {
              if (Contains(
                      withdrawedLines,
                      linePieceInst.second)) { // the line should be withdrawed
                return true;
              }
              auto &cutLine = mg.lines[linePieceInst.second].component;
              Vec3 vertDir = normalize(cutLine.first.cross(cutLine.second));
              if (connection.first.dot(vertDir) *
                      connection.second.dot(vertDir) >
                  1e-5) { // not crossed
                return true;
              }
              auto interp = normalize(
                  Intersection(connection.ray(), Plane3(Origin(), vertDir)));
              if (cutLine.first.cross(interp).dot(
                      cutLine.second.cross(interp)) > 1e-5) { // not crossed
                return true;
              }
              auto &lineSidingWeight = lsw[linePieceInst.second];
              wr = std::min(wr, lineSidingWeight.minWeightRatio());
              return wr > 0.0;
            });
        if (wr <= 0.0) {
          break;
        }
      }
      return wr;
    };

    for (int line = 0; line < mg.nlines(); line++) {
      // seg-line
      for (int lp : mg.line2linePieces[line]) {
        int bp = mg.linePiece2bndPiece[lp];
        if (bp != -1) {
          int seg1, seg2;
          std::tie(seg1, seg2) = mg.bnd2segs[mg.bndPiece2bnd[bp]];
          double wrToLeftSeg =
              cutWR(normalize(Line3(mg.lines[line].component.center(),
                                    mg.seg2center[seg1])),
                    {line});
          segLine2weightRatio[std::make_pair(seg1, line)] = std::min(
              AtOr(segLine2weightRatio, std::make_pair(seg1, line), 1.0),
              wrToLeftSeg);
          double wrToRightSeg =
              cutWR(normalize(Line3(mg.lines[line].component.center(),
                                    mg.seg2center[seg2])),
                    {line});
          segLine2weightRatio[std::make_pair(seg2, line)] = std::min(
              AtOr(segLine2weightRatio, std::make_pair(seg2, line), 1.0),
              wrToRightSeg);
        } else {
          int seg = mg.linePiece2seg[lp];
          assert(seg != -1);
          double wrToSeg =
              cutWR(normalize(Line3(mg.lines[line].component.center(),
                                    mg.seg2center[seg])),
                    {line});
          segLine2weightRatio[std::make_pair(seg, line)] = std::min(
              AtOr(segLine2weightRatio, std::make_pair(seg, line), 1.0),
              wrToSeg);
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
        double wr = cutWR(
            normalize(Line3(nearest.first.position, nearest.second.position)),
            {line, anotherLine});
        auto linePair = MakeOrderedPair(line, anotherLine);
        lineLine2weightRatio[linePair] =
            std::min(AtOr(lineLine2weightRatio, linePair, 1.0), wr);
      }
    }
  }

  // add constraints
  // bndpieces, seg-seg
  for (int i = 0; i < mg.nbndPieces(); i++) {
    /*if (mg.bndPiece2segRelation[i] != SegRelation::Connected) {
        continue;
    }*/
    Constraint connect;
    connect.type = Constraint::Connection;
    auto &segPair = mg.bnd2segs[mg.bndPiece2bnd[i]];
    connect.ent1 = seg2ent[segPair.first];
    connect.ent2 = seg2ent[segPair.second];
    double len = mg.bndPiece2length[i];
    connect.weight =
        len * AtOr(segSeg2weightRatio,
                   MakeOrderedPair(segPair.first, segPair.second), 0.5);
    if (len >= minAngleThresForAWideEdge) {
      connect.anchors = {mg.bndPiece2dirs[i].front(),
                         mg.bndPiece2dirs[i].back()};
    } else {
      connect.anchors = {
          normalize(mg.bndPiece2dirs[i].front() + mg.bndPiece2dirs[i].back())};
    }
    constraints.push_back(connect);
    int connectCon = constraints.size() - 1;
    ent2cons[connect.ent1].push_back(connectCon);
    ent2cons[connect.ent2].push_back(connectCon);

    // check whether this bp lies on a line
    if (!mg.bndPiece2linePieces[i].empty() || len < minAngleThresForAWideEdge) {
      // if so, no planarity will be assigned
      continue;
    }

    // the coplanarity constraint if there are no lines on the bndpiece
    if (mg.bndPiece2linePieces[i].empty()) {
      Constraint coplanar;
      coplanar.type = Constraint::Coplanarity;
      coplanar.ent1 = connect.ent1;
      coplanar.ent2 = connect.ent2;
      coplanar.weight =
          len * AtOr(segSeg2weightRatio,
                     MakeOrderedPair(segPair.first, segPair.second), 0.5);
      // coplanar.anchorOrientation = -1;
      constraints.push_back(coplanar);
      int coplanarCon = constraints.size() - 1;
      ent2cons[coplanar.ent1].push_back(coplanarCon);
      ent2cons[coplanar.ent2].push_back(coplanarCon);
    }
  }

  // linepieces, seg-line
  for (int i = 0; i < mg.nlinePieces(); i++) {
    /*if (mg.linePiece2segLineRelation[i] == SegLineRelation::Detached) {
        continue;
    }*/
    int bndPiece = mg.linePiece2bndPiece[i];
    int line = mg.linePiece2line[i];
    if (bndPiece == -1) {
      int seg = mg.linePiece2seg[i];
      assert(seg != -1);
      Constraint c;
      c.type = Constraint::Connection;
      c.ent1 = seg2ent[seg];
      c.ent2 = line2ent[line];
      double len = mg.linePiece2length[i];
      c.weight =
          len * AtOr(segLine2weightRatio, std::make_pair(seg, line), 0.5);
      if (len >= minAngleThresForAWideEdge) {
        c.anchors = {mg.linePiece2samples[i].front(),
                     mg.linePiece2samples[i].back()};
      } else {
        c.anchors = {normalize(mg.linePiece2samples[i].front() +
                               mg.linePiece2samples[i].back())};
      }
      constraints.push_back(c);
      int con = constraints.size() - 1;
      ent2cons[c.ent1].push_back(con);
      ent2cons[c.ent2].push_back(con);
    } else {
      int segPair[] = {-1, -1};
      std::tie(segPair[0], segPair[1]) = mg.bnd2segs[mg.bndPiece2bnd[bndPiece]];
      auto segRelation = mg.bndPiece2segRelation[bndPiece];
      /* bool connected[] = {
           segRelation == SegRelation::Connected || segRelation ==
       SegRelation::LeftIsFront,
           segRelation == SegRelation::Connected || segRelation ==
       SegRelation::RightIsFront
       };*/
      for (int k = 0; k < 2; k++) {
        int seg = segPair[k];
        /*if (!connected[k]) {
            continue;
        }*/
        Constraint c;
        c.type = Constraint::Connection;
        c.ent1 = seg2ent[seg];
        c.ent2 = line2ent[line];
        double len = mg.linePiece2length[i];
        c.weight =
            len * AtOr(segLine2weightRatio, std::make_pair(seg, line), 0.5);
        if (len >= minAngleThresForAWideEdge) {
          c.anchors = {mg.linePiece2samples[i].front(),
                       mg.linePiece2samples[i].back()};
        } else {
          c.anchors = {normalize(mg.linePiece2samples[i].front() +
                                 mg.linePiece2samples[i].back())};
        }
        constraints.push_back(c);
        int con = constraints.size() - 1;
        ent2cons[c.ent1].push_back(con);
        ent2cons[c.ent2].push_back(con);
      }
    }
  }

  // linerelations, line-line
  for (int i = 0; i < mg.nlineRelations(); i++) {
    /* if (mg.lineRelations[i] == LineRelation::Detached) {
         continue;
     }*/
    Constraint c;
    c.type = Constraint::Connection;
    auto &linePair = mg.lineRelation2lines[i];
    c.ent1 = line2ent[linePair.first];
    c.ent2 = line2ent[linePair.second];
    c.weight = mg.lineRelation2weight[i] *
               AtOr(lineLine2weightRatio,
                    MakeOrderedPair(linePair.first, linePair.second), 0.5);
    c.anchors = {mg.lineRelation2anchor[i]};
    // c.anchorOrientation = -1;
    constraints.push_back(c);
    int con = constraints.size() - 1;
    ent2cons[c.ent1].push_back(con);
    ent2cons[c.ent2].push_back(con);
  }

  cg.cons2enabled.resize(constraints.size(), true);

  return cg;
}

// the staibility info
struct Stability {
  virtual int dof() const = 0;
  virtual void addAnchor(const Vec3 &anchor, double angleThres) = 0;
  virtual Stability *clone() const = 0;
  virtual void stablize() = 0;
};
bool supportersAreValid(const std::vector<Vec3> &supporters,
                        double angleThres) {
  for (int i = 0; i < supporters.size(); i++) {
    for (int j = i + 1; j < supporters.size(); j++) {
      if (AngleBetweenDirected(supporters[i], supporters[j]) <= angleThres) {
        return false;
      }
    }
  }
  return true;
}
struct Dof3Stability : Stability {
  std::vector<Vec3> supporters;
  Dof3Stability() {}
  Dof3Stability(const Dof3Stability &s) = default;
  virtual int dof() const override { return 3 - supporters.size(); }
  virtual void addAnchor(const Vec3 &anchor, double angleThres) override {
    assert(supporters.size() <= 3);
    if (supporters.size() == 3) {
      return;
    }
    if (supporters.empty()) {
      supporters.push_back(anchor);
    } else if (supporters.size() == 1) {
      if (AngleBetweenDirected(supporters[0], anchor) > angleThres) {
        supporters.push_back(anchor);
      }
    } else if (supporters.size() == 2) {
      if (AngleDistanceBetweenPointAndRay(
              anchor, Line3(supporters[0], supporters[1]).ray()) > angleThres) {
        supporters.push_back(anchor);
      }
    }
    assert(supportersAreValid(supporters, angleThres));
  }
  virtual Stability *clone() const override { return new Dof3Stability(*this); }
  virtual void stablize() override { supporters.resize(3); }
};
struct Dof2Stability : Stability {
  Vec3 axis;
  std::vector<Vec3> supporters;
  explicit Dof2Stability(const Vec3 &a) : axis(a) {}
  Dof2Stability(const Dof2Stability &s) = default;
  virtual int dof() const override { return 2 - supporters.size(); }
  virtual void addAnchor(const Vec3 &anchor, double angleThres) override {
    assert(supporters.size() <= 2);
    if (supporters.size() == 2) {
      return;
    }
    if (supporters.empty()) {
      supporters.push_back(anchor);
    } else if (supporters.size() == 1) {
      if (AngleDistanceBetweenPointAndRay(anchor, Ray3(supporters[0], axis)) >
          angleThres) {
        supporters.push_back(anchor);
      }
    }
    assert(supportersAreValid(supporters, angleThres));
  }
  virtual Stability *clone() const override { return new Dof2Stability(*this); }
  virtual void stablize() override { supporters.resize(2); }
};
struct Dof1Stability : Stability {
  bool stable;
  Dof1Stability() : stable(false) {}
  Dof1Stability(const Dof1Stability &s) = default;
  virtual int dof() const override { return stable ? 0 : 1; }
  virtual void addAnchor(const Vec3 &anchor, double angleThres) override {
    stable = true;
  }
  virtual Stability *clone() const override { return new Dof1Stability(*this); }
  virtual void stablize() override { stable = true; }
};

PICGDeterminablePart LocateDeterminablePart(const PIConstraintGraph &cg,
                                            double angleThres,
                                            bool connectAll) {
  PICGDeterminablePart dp;

  // collect ent stabilities
  std::vector<std::unique_ptr<Stability>> ent2stab(cg.entities.size());
  for (int ent = 0; ent < cg.entities.size(); ent++) {
    auto &e = cg.entities[ent];
    if (e.supportingPlane.dof == 1) {
      ent2stab[ent] = std::make_unique<Dof1Stability>();
    } else if (e.supportingPlane.dof == 2) {
      ent2stab[ent] = std::make_unique<Dof2Stability>(e.supportingPlane.along);
    } else if (e.supportingPlane.dof == 3) {
      ent2stab[ent] = std::make_unique<Dof3Stability>();
    }
  }

  // select the largest dof1 ent as the root!
  std::vector<int> rootCands;
  for (int i = 0; i < cg.entities.size(); i++) {
    int sz = cg.entities[i].size;
    if (ent2stab[i]->dof() != 1) {
      continue;
    }
    rootCands.push_back(i);
  }
  std::sort(rootCands.begin(), rootCands.end(), [&cg](int a, int b) {
    return cg.entities[a].size > cg.entities[b].size;
  });
  if (rootCands.empty()) {
    std::cout << "we can't find any entity whose dof is 1 !!!!!" << std::endl;
    return dp;
  }

  for (int root : rootCands) {
    std::cout << "root: " << root << std::endl;
    std::set<int> entsCollected;

    // initialize stabilities of entities
    std::vector<std::unique_ptr<Stability>> ent2stabHere(cg.entities.size());
    for (int ent = 0; ent < cg.entities.size(); ent++) {
      ent2stabHere[ent] = std::unique_ptr<Stability>(ent2stab[ent]->clone());
    }
    ent2stabHere[root]->stablize();
    assert(ent2stabHere[root]->dof() == 0);

    MaxHeap<int, int, std::greater<int>> Q; // a min heap recording dofs
    Q.set(root, ent2stabHere[root]->dof());
    while (!Q.empty()) {
      int curEnt = Q.top();
      int curEntDoF = ent2stabHere[curEnt]->dof();
      if (curEntDoF != 0) { // all remaining adjacent entities are not stable,
                            // stop the search
        break;
      }
      assert(std::all_of(Q.begin(), Q.end(),
                         [curEntDoF](const Scored<int, int> &ent) {
                           return ent.score >= curEntDoF;
                         }));
      Q.pop();
      entsCollected.insert(curEnt);
      for (int con : cg.ent2cons[curEnt]) {
        if (!cg.cons2enabled[con]) {
          continue;
        }
        auto &c = cg.constraints[con];
        if (c.weight == 0.0) {
          continue;
        }
        int adjEnt = c.ent1 == curEnt ? c.ent2 : c.ent1;
        if (Contains(entsCollected, adjEnt)) {
          continue;
        }

        if (connectAll) {
          ent2stabHere[adjEnt]->stablize(); ///////// !!!!!
        } else {
          if (c.isConnection()) {
            for (auto &anchor : c.anchors) {
              ent2stabHere[adjEnt]->addAnchor(anchor, angleThres);
            }
          } else if (c.isCoplanarity()) {
            assert(cg.entities[adjEnt].isSeg());
            ent2stabHere[adjEnt]->stablize();
          }
        }
        Q.set(adjEnt, ent2stabHere[adjEnt]->dof());
      }
    }
    if (entsCollected.size() > cg.entities.size() / 2) {
      dp.rootEnt = root;
      dp.determinableEnts = std::move(entsCollected);
      break;
    }
  }

  dp.consBetweenDeterminableEnts.clear();
  for (int i = 0; i < cg.constraints.size(); i++) {
    auto &c = cg.constraints[i];
    if (Contains(dp.determinableEnts, c.ent1) &&
        Contains(dp.determinableEnts, c.ent2)) {
      dp.consBetweenDeterminableEnts.insert(i);
    }
  }

  return dp;
}

PIConstraintGraph BuildPIConstraintGraph(const PILayoutAnnotation &anno,
                                         double minAngleThresForAWideEdge) {

  PIConstraintGraph cg;
  cg.seg2ent.resize(anno.nfaces(), -1);

  auto &seg2ent = cg.seg2ent;
  auto &line2ent = cg.line2ent;
  auto &entities = cg.entities;
  auto &constraints = cg.constraints;

  using Entity = PIConstraintGraph::Entity;
  using Constraint = PIConstraintGraph::Constraint;

  // add entities
  // add segs
  for (int i = 0; i < anno.nfaces(); i++) {
    Entity e;
    e.type = Entity::IsSeg;
    e.id = i;
    e.size = 1;

    auto &control = anno.face2control[i];
    e.supportingPlane.dof = control.dof();
    Vec3 center = Origin();
    for (int c : anno.face2corners[i]) {
      center += anno.corners[c];
    }
    e.supportingPlane.center = normalize(center);
    if (e.supportingPlane.dof == 1) {
      e.supportingPlane.toward = normalize(anno.vps[control.orientationClaz]);
      if (e.supportingPlane.toward.dot(e.supportingPlane.center) < 0) {
        e.supportingPlane.toward = -e.supportingPlane.toward;
      }
    } else if (e.supportingPlane.dof == 2) {
      e.supportingPlane.along = normalize(anno.vps[control.orientationNotClaz]);
    } else {
    }

    entities.push_back(e);

    int ent = entities.size() - 1;
    seg2ent[i] = ent;
  }

  auto &ent2cons = cg.ent2cons;
  ent2cons.resize(entities.size()); // enttity -> constraints

  // add constraints
  // face connect face
  // get corners2border and border2face
  std::map<std::pair<int, int>, int> corners2border;
  for (int i = 0; i < anno.nborders(); i++) {
    auto &cs = anno.border2corners[i];
    corners2border[cs] = i;
  }
  std::vector<std::pair<int, int>> border2face(anno.nborders(),
                                               std::make_pair(-1, -1));
  for (int i = 0; i < anno.nfaces(); i++) {
    auto &cs = anno.face2corners[i];
    for (int j = 0; j < cs.size(); j++) {
      int c1 = cs[j];
      int c2 = cs[(j + 1) % cs.size()];
      if (Contains(corners2border, std::make_pair(c1, c2))) {
        int b = corners2border.at(std::make_pair(c1, c2));
        border2face[b].first = i;
      } else if (Contains(corners2border, std::make_pair(c2, c1))) {
        int b = corners2border.at(std::make_pair(c2, c1));
        border2face[b].second = i;
      } else {
        SHOULD_NEVER_BE_CALLED();
      }
    }
  }

  // connect
  for (int i = 0; i < anno.nborders(); i++) {
    if (!anno.border2connected[i]) {
      continue;
    }
    Constraint connect;
    connect.type = Constraint::Connection;

    auto &segPair = border2face[i];
    connect.ent1 = seg2ent[segPair.first];
    connect.ent2 = seg2ent[segPair.second];
    double len =
        AngleBetweenDirected(anno.corners[anno.border2corners[i].first],
                               anno.corners[anno.border2corners[i].second]);
    connect.weight = len;
    if (len >= minAngleThresForAWideEdge) {
      connect.anchors = {anno.corners[anno.border2corners[i].first],
                         anno.corners[anno.border2corners[i].second]};
    } else {
      connect.anchors = {
          normalize(anno.corners[anno.border2corners[i].first] +
                    anno.corners[anno.border2corners[i].second])};
    }
    constraints.push_back(connect);
    int connectCon = constraints.size() - 1;
    ent2cons[connect.ent1].push_back(connectCon);
    ent2cons[connect.ent2].push_back(connectCon);
  }

  // coplanar
  for (auto &fp : anno.coplanarFacePairs) {
    int f1 = fp.first;
    int f2 = fp.second;
    Constraint coplanar;
    coplanar.type = Constraint::Coplanarity;
    coplanar.ent1 = f1;
    coplanar.ent2 = f2;
    coplanar.weight = 1.0;
    constraints.push_back(coplanar);
    int coplanarCon = constraints.size() - 1;
    ent2cons[coplanar.ent1].push_back(coplanarCon);
    ent2cons[coplanar.ent2].push_back(coplanarCon);
  }

  cg.cons2enabled.resize(constraints.size(), true);

  return cg;
}

PIConstraintGraph
BuildPIConstraintGraphWithLines(const PILayoutAnnotation &anno,
                                double minAngleThresForAWideEdge) {
  PIConstraintGraph cg;
  cg.seg2ent.resize(anno.nfaces(), -1);
  cg.line2ent.resize(anno.nborders(), -1);

  auto &seg2ent = cg.seg2ent;
  auto &line2ent = cg.line2ent;
  auto &entities = cg.entities;
  auto &constraints = cg.constraints;

  using Entity = PIConstraintGraph::Entity;
  using Constraint = PIConstraintGraph::Constraint;

  // add entities
  // add segs
  for (int i = 0; i < anno.nfaces(); i++) {
    Entity e;
    e.type = Entity::IsSeg;
    e.id = i;
    e.size = 1;

    auto &control = anno.face2control[i];
    e.supportingPlane.dof = control.dof();
    Vec3 center = Origin();
    for (int c : anno.face2corners[i]) {
      center += anno.corners[c];
    }
    e.supportingPlane.center = normalize(center);
    if (e.supportingPlane.dof == 1) {
      e.supportingPlane.toward = normalize(anno.vps[control.orientationClaz]);
      if (e.supportingPlane.toward.dot(e.supportingPlane.center) < 0) {
        e.supportingPlane.toward = -e.supportingPlane.toward;
      }
    } else if (e.supportingPlane.dof == 2) {
      e.supportingPlane.along = normalize(anno.vps[control.orientationNotClaz]);
    } else {
    }

    entities.push_back(e);

    int ent = entities.size() - 1;
    seg2ent[i] = ent;
  }

  // add lines
  for (int i = 0; i < anno.nborders(); i++) {
    if (!anno.border2connected[i]) {
      continue;
    }

    Entity e;
    e.type = Entity::IsLine;
    e.id = i;
    e.size = 1;

    Line3 line(normalize(anno.corners[anno.border2corners[i].first]),
               normalize(anno.corners[anno.border2corners[i].second]));
    e.supportingPlane.dof = 2;
    e.supportingPlane.center = normalize(line.center());
    if (e.supportingPlane.dof == 1) {
    } else {
      const Vec3 &maybeLineDirection = line.direction();
      auto linePerp = line.center().cross(maybeLineDirection);
      assert(IsFuzzyPerpendicular(linePerp, maybeLineDirection));
      e.supportingPlane.along = normalize(linePerp);
    }

    entities.push_back(e);

    int ent = entities.size() - 1;
    line2ent[i] = ent;
  }

  auto &ent2cons = cg.ent2cons;
  ent2cons.resize(entities.size()); // enttity -> constraints

  // add constraints
  // face connect face
  // get corners2border and border2face
  std::map<std::pair<int, int>, int> corners2border;
  for (int i = 0; i < anno.nborders(); i++) {
    auto &cs = anno.border2corners[i];
    corners2border[cs] = i;
  }
  std::vector<std::pair<int, int>> border2face(anno.nborders(),
                                               std::make_pair(-1, -1));
  for (int i = 0; i < anno.nfaces(); i++) {
    auto &cs = anno.face2corners[i];
    for (int j = 0; j < cs.size(); j++) {
      int c1 = cs[j];
      int c2 = cs[(j + 1) % cs.size()];
      if (Contains(corners2border, std::make_pair(c1, c2))) {
        int b = corners2border.at(std::make_pair(c1, c2));
        border2face[b].first = i;
      } else if (Contains(corners2border, std::make_pair(c2, c1))) {
        int b = corners2border.at(std::make_pair(c2, c1));
        border2face[b].second = i;
      } else {
        throw std::runtime_error(
            "neither (c1, c2) nor (c2, c1) is stored in corners2border!");
      }
    }
  }

  // border-face connect
  for (int i = 0; i < anno.nborders(); i++) {
    if (!anno.border2connected[i]) {
      continue;
    }
    {
      int segPair[] = {-1, -1};
      std::tie(segPair[0], segPair[1]) = border2face[i];
      for (int k = 0; k < 2; k++) {
        int seg = segPair[k];
        Constraint c;
        c.type = Constraint::Connection;
        c.ent1 = seg2ent[seg];
        c.ent2 = line2ent[i];
        double len = 1.0;
        c.weight = len;
        if (len >= minAngleThresForAWideEdge) {
          c.anchors = {anno.corners[anno.border2corners[i].first],
                       anno.corners[anno.border2corners[i].second]};
        } else {
          c.anchors = {anno.corners[anno.border2corners[i].first] +
                       anno.corners[anno.border2corners[i].second]};
        }
        constraints.push_back(c);
        int con = constraints.size() - 1;
        ent2cons[c.ent1].push_back(con);
        ent2cons[c.ent2].push_back(con);
      }
    }
  }

  // coplanar
  for (auto &fp : anno.coplanarFacePairs) {
    int f1 = fp.first;
    int f2 = fp.second;
    Constraint coplanar;
    coplanar.type = Constraint::Coplanarity;
    coplanar.ent1 = f1;
    coplanar.ent2 = f2;
    coplanar.weight = 1.0;
    constraints.push_back(coplanar);
    int coplanarCon = constraints.size() - 1;
    ent2cons[coplanar.ent1].push_back(coplanarCon);
    ent2cons[coplanar.ent2].push_back(coplanarCon);
  }

  cg.cons2enabled.resize(constraints.size(), true);

  return cg;
}
}
}