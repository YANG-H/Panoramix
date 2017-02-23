#include "pch.hpp"

#include "algorithms.hpp"
#include "containers.hpp"

#include "canvas.hpp"
#include "scene.hpp"

#include "pi_graph_solve.hpp"
#include "pi_graph_vis.hpp"

namespace pano {
namespace experimental {

namespace {
int SwappedComponent(const Vec3 &orientation) {
  for (int i = 0; i < 2; i++) {
    if (abs(orientation[i]) >= 1e-8) {
      return i;
    }
  }
  return 2;
}
}

std::vector<double> InverseDepthCoefficientsOfSegAtDirection(
    const std::vector<Vec3> &vps, const SegControl &control, const Vec3 &center,
    const Vec3 &direction) {
  if (control.dof() == 1) {
    Plane3 plane(center, vps[control.orientationClaz]);
    // variable is 1.0/centerDepth
    // corresponding coeff is 1.0/depthRatio
    // so that 1.0/depth = 1.0/centerDepth * 1.0/depthRatio -> depth =
    // centerDepth * depthRatio
    double depthRatio =
        norm(Intersection(Ray3(Point3(0, 0, 0), direction), plane));
    return std::vector<double>{1.0 / depthRatio};
  } else if (control.dof() == 2) {
    // depth = 1.0 / (ax + by + cz) where (x, y, z) = direction, (a, b, c) =
    // variables
    // -> 1.0/depth = ax + by + cz
    // -> 1.0/depth = ax + by + (- o1a - o2b)/o3 * z = a(x-o1*z/o3) +
    // b(y-o2*z/o3)
    auto orientation = normalize(vps[control.orientationNotClaz]);
    int sc = SwappedComponent(orientation);
    Vec3 forientation = orientation;
    std::swap(forientation[sc], forientation[2]);
    Vec3 fdirection = direction;
    std::swap(fdirection[sc], fdirection[2]);
    return std::vector<double>{
        fdirection[0] - forientation[0] * fdirection[2] / forientation[2],
        fdirection[1] - forientation[1] * fdirection[2] / forientation[2]};
  } else {
    // depth = 1.0 / (ax + by + cz) where (x, y, z) = direction, (a, b, c) =
    // variables
    // -> 1.0/depth = ax + by + cz
    return std::vector<double>{direction[0], direction[1], direction[2]};
  }
}

std::vector<double>
InverseDepthCoefficientsOfLineAtDirection(const std::vector<Vec3> &vps,
                                          const Classified<Line3> &line,
                                          const Vec3 &direction) {
  int claz = line.claz;
  auto &l = line.component;
  if (claz >= 0 && claz < vps.size()) {
    Ray3 infLine(normalize(l.center()), vps[claz]);
    // variable is 1.0/centerDepth
    // corresponding coeff is 1.0/depthRatio
    // so that 1.0/depth = 1.0/centerDepth * 1.0/depthRatio -> depth =
    // centerDepth * depthRatio
    double depthRatio =
        norm(DistanceBetweenTwoLines(Ray3(Point3(0, 0, 0), direction), infLine)
                 .second.first);
    return std::vector<double>{1.0 / depthRatio};
  } else {
    double theta =
        AngleBetweenDirected(normalize(l.first), normalize(l.second));
    double phi = AngleBetweenDirected(normalize(l.first), direction);
    /*           | sin(theta) | | p | | q |
    len:---------------------------------   => 1/len: [(1/q)sin(phi) -
    (1/p)sin(phi-theta)] / sin(theta)
    | p sin(phi) - q sin(phi - theta) |
    */
    // variables[0] -> 1/p
    // variables[1] -> 1/q
    double coeffFor1_p = -sin(phi - theta) / sin(theta);
    double coeffFor1_q = sin(phi) / sin(theta);
    assert(!IsInfOrNaN(coeffFor1_p) && !IsInfOrNaN(coeffFor1_q));
    return std::vector<double>{coeffFor1_p, coeffFor1_q};
  }
}

std::vector<double>
InverseDepthCoefficientsAtDirection(const PIGraph<PanoramicCamera> &mg,
                                    const PIConstraintGraph::Entity &e,
                                    const Vec3 &direction) {
  assert(IsFuzzyZero(norm(direction) - 1.0, 1e-2));
  if (e.isSeg()) {
    int seg = e.id;
    return InverseDepthCoefficientsOfSegAtDirection(
        mg.vps, mg.seg2control[seg], mg.seg2center[seg], direction);
  } else {
    int line = e.id;
    return InverseDepthCoefficientsOfLineAtDirection(mg.vps, mg.lines[line],
                                                     direction);
  }
}

DenseMatd SegPlaneEquationCoefficients(const std::vector<Vec3> &vps,
                                       const SegControl &control,
                                       const Vec3 &center) {
  if (control.dof() == 1) {
    auto &vp = vps[control.orientationClaz];
    assert(IsFuzzyZero(norm(center) - 1.0, 1e-2));
    assert(IsFuzzyZero(norm(vp) - 1.0, 1e-2));
    // variable is 1.0/centerDepth
    //  let center = [cx, cy, cz], vp = [vx, vy, vz]
    //  then the equation should be k vx x + k vy y + k vz z = 1
    //  if the depth of center is Dc, then
    //      k vx Dc cx + k vy Dc cy + k vz Dc cz = 1
    //  and then k = 1.0 / (Dc*dot(v, c))
    //  the equation coefficients thus are
    //      k v* = v* / (Dc*dot(v, c))
    //  therefore the equation coefficients corresponding to the variable
    //  (inverse center depth, 1.0/Dc) are
    //      v* / dot(v, c)
    DenseMatd coeffs(3, 1, 0.0);
    for (int i = 0; i < 3; i++) {
      coeffs(i, 0) = vp[i] / vp.dot(center);
    }
    return coeffs;
  } else if (control.dof() == 2) {
    auto orientation = normalize(vps[control.orientationNotClaz]);
    int sc = SwappedComponent(orientation); // find a non zero component
    Vec3 forientation = orientation;
    std::swap(forientation[sc], forientation[2]);
    // (a, b, c) \perp o => o1a + o2b + o3c = 0 => c =  (- o1a - o2b)/o3
    DenseMatd coeffs(3, 2, 0.0);
    coeffs(0, 0) = coeffs(1, 1) = 1;
    coeffs(2, 0) = -forientation[0] / forientation[2];
    coeffs(2, 1) = -forientation[1] / forientation[2];
    for (int k = 0; k < 2; k++) {
      std::swap(coeffs(sc, k), coeffs(2, k));
    }
    return coeffs;
  } else /*if (control.dof() == 3)*/ {
    return DenseMatd::eye(3, 3);
  }
}

Plane3 SegInstance(const PIGraph<PanoramicCamera> &mg, const double *variables, int nvar,
                   int seg) {
  DenseMatd k = SegPlaneEquationCoefficients(mg.vps, mg.seg2control[seg],
                                             mg.seg2center[seg]);
  assert(k.cols == nvar);
  std::vector<double> x(variables, variables + nvar);
  DenseMatd p = k * DenseMatd(x);
  return Plane3FromEquation(p(0, 0), p(1, 0), p(2, 0));
}

Line3 LineInstance(const PIGraph<PanoramicCamera> &mg, const double *variables, int nvar,
                   int line) {
  auto &l = mg.lines[line];
  if (l.claz >= 0 && l.claz < mg.vps.size()) {
    assert(nvar == 1);
    Ray3 infLine(normalize(l.component.center()) / NonZeroize(variables[0]),
                 mg.vps[l.claz]);
    return Line3(
        DistanceBetweenTwoLines(
            Ray3(Point3(0, 0, 0), normalize(l.component.first)), infLine)
            .second.second,
        DistanceBetweenTwoLines(
            Ray3(Point3(0, 0, 0), normalize(l.component.second)), infLine)
            .second.second);
  } else /*if (line.type == MGUnary::LineFree)*/ {
    /*           | sin(theta) | | p | | q |
    len:---------------------------------   => 1/len: [(1/q)sin(phi) -
    (1/p)sin(phi-theta)] / sin(theta)
    | p sin(phi) - q sin(phi - theta) |
    */
    // variables[0] -> 1/p
    // variables[1] -> 1/q
    assert(nvar == 2);
    return Line3(normalize(l.component.first) / NonZeroize(variables[0]),
                 normalize(l.component.second) / NonZeroize(variables[1]));
  }
}

Plane3 SegInstance(const std::vector<Vec3> &vps, const SegControl &control,
                   const Vec3 &center, const double *variables) {
  DenseMatd k = SegPlaneEquationCoefficients(vps, control, center);
  std::vector<double> x(variables, variables + k.cols);
  DenseMatd p = k * DenseMatd(x);
  return Plane3FromEquation(p(0, 0), p(1, 0), p(2, 0));
}

void ReconstructLayoutAnnotation(PILayoutAnnotation &anno,
                                 misc::Matlab &matlab) {

  if (anno.nfaces() == 0) {
    return;
  }

  // group faces as planes
  std::vector<int> face2plane(anno.nfaces());
  std::map<int, std::set<int>> plane2faces;

  std::vector<int> faces(anno.nfaces());
  std::iota(faces.begin(), faces.end(), 0);
  int nplanes = core::ConnectedComponents(
      faces.begin(), faces.end(),
      [&anno](int face) {
        std::vector<int> coplanarFaces;
        for (auto &p : anno.coplanarFacePairs) {
          if (p.first == face) {
            coplanarFaces.push_back(p.second);
          } else if (p.second == face) {
            coplanarFaces.push_back(p.first);
          }
        }
        return coplanarFaces;
      },
      [&face2plane, &plane2faces](int face, int ccid) {
        face2plane[face] = ccid;
        plane2faces[ccid].insert(face);
      });

  // get plane properties
  std::vector<SegControl> plane2control(nplanes);
  std::vector<Vec3> plane2center(nplanes);
  std::vector<int> plane2varPosition(nplanes);
  std::vector<int> plane2nvar(nplanes);

  int nvars = 0;
  for (int i = 0; i < nplanes; i++) {
    SegControl control = {-1, -1, true};
    Vec3 center;
    for (int face : plane2faces[i]) {
      auto &ctrl = anno.face2control[face];
      if (ctrl.dof() < control.dof()) {
        control = ctrl;
      }
      for (int c : anno.face2corners[face]) {
        center += normalize(anno.corners[c]);
      }
    }
    center /= norm(center);
    plane2control[i] = control;
    plane2center[i] = center;
    int nvar = control.dof();
    plane2nvar[i] = nvar;
    plane2varPosition[i] = nvars;
    nvars += nvar;
  }

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

  // the elements of sparse matrix A
  // inverse depths of vert1s on each anchor A1 * X
  // inverse depths of vert2s on each anchor A2 * X
  // weight on each anchor W
  std::vector<SparseMatElementd> A1triplets, A2triplets;
  int eid = 0;
  for (int i = 0; i < anno.nborders(); i++) {
    int plane1 = face2plane[border2face[i].first];
    int plane2 = face2plane[border2face[i].second];
    assert(plane1 != -1 && plane2 != -1);
    if (!anno.border2connected[i]) {
      continue;
    }
    int varpos1 = plane2varPosition[plane1];
    int varpos2 = plane2varPosition[plane2];

    Vec3 corners[] = {anno.corners[anno.border2corners[i].first],
                      anno.corners[anno.border2corners[i].second]};
    for (auto &anchor : corners) {
      auto coeffs1 = InverseDepthCoefficientsOfSegAtDirection(
          anno.vps, plane2control[plane1], plane2center[plane1], anchor);
      for (int k = 0; k < coeffs1.size(); k++) {
        A1triplets.emplace_back(eid, varpos1 + k, coeffs1[k]);
      }
      auto coeffs2 = InverseDepthCoefficientsOfSegAtDirection(
          anno.vps, plane2control[plane2], plane2center[plane2], anchor);
      for (int k = 0; k < coeffs2.size(); k++) {
        A2triplets.emplace_back(eid, varpos2 + k, coeffs2[k]);
      }
      eid++;
    }
  }

  int neqs = eid;

  auto A1 = MakeSparseMatFromElements(neqs, nvars, A1triplets.begin(),
                                      A1triplets.end());
  auto A2 = MakeSparseMatFromElements(neqs, nvars, A2triplets.begin(),
                                      A2triplets.end());

  matlab << "clear;";
  matlab.setVar("A1", A1);
  matlab.setVar("A2", A2);

  matlab << "m = size(A1, 1);"; // number of equations
  matlab << "n = size(A1, 2);"; // number of variables

  double minE = std::numeric_limits<double>::infinity();

  std::vector<double> X;
  for (int i = 0; i < 100; i++) {
    matlab << "D1D2 = ones(m, 1);"; //  current depths of anchors
    while (true) {
      matlab << "K = (A1 - A2) .* repmat(D1D2, [1, n]);";
      matlab << "cvx_begin"
             << "variable X(n);"
             << "minimize sum_square(K * X)"
             << "subject to"
             << "    ones(m, 1) <= A1 * X;"
             << "    ones(m, 1) <= A2 * X;"
             << "cvx_end";
      matlab << "e = norm(K * X);";
      double curE = matlab.var("e").scalar();
      std::cout << "e = " << curE << std::endl;
      if (IsInfOrNaN(curE)) {
        break;
      }
      if (curE < minE) {
        minE = curE;
        matlab << "X = 2 * X ./ median((A1 + A2) * X);";
        X = matlab.var("X").toCVMat();
      } else {
        break;
      }
      matlab << "D1D2 = 1./ (A1 * X) ./ (A2 * X);";
      matlab << "D1D2 = abs(D1D2) / norm(D1D2);";
    }
    if (IsInfOrNaN(minE)) {
      matlab << "scale = scale * 2;";
    } else {
      break;
    }
  }

  // install back as planes
  for (int i = 0; i < nplanes; i++) {
    Plane3 inst = SegInstance(anno.vps, plane2control[i], plane2center[i],
                              X.data() + plane2varPosition[i]);
    for (int face : plane2faces[i]) {
      anno.face2plane[face] = inst;
    }
  }
}

void ReconstructLayoutAnnotation2(PILayoutAnnotation &anno,
                                  misc::Matlab &matlab) {
  auto cg = BuildPIConstraintGraph(anno, 0);
  auto dp = LocateDeterminablePart(cg, DegreesToRadians(3), false);
  Solve(dp, cg, matlab);
  // install back as planes
  for (auto &e : cg.entities) {
    Plane3 inst = e.supportingPlane.reconstructed;
    if (e.isSeg()) {
      anno.face2plane[e.id] = inst;
    }
  }
}

void ReconstructLayoutAnnotation3(PILayoutAnnotation &anno,
                                  misc::Matlab &matlab) {
  auto cg = BuildPIConstraintGraphWithLines(anno, 0);
  auto dp = LocateDeterminablePart(cg, DegreesToRadians(3), false);
  Solve(dp, cg, matlab);
  // install back as planes
  for (auto &e : cg.entities) {
    Plane3 inst = e.supportingPlane.reconstructed;
    if (e.isSeg()) {
      anno.face2plane[e.id] = inst;
    }
  }
}

double Solve(const PICGDeterminablePart &dp, PIConstraintGraph &cg,
             misc::Matlab &matlab, int maxIter,
             double connectionWeightRatioOverCoplanarity, bool useCoplanarity) {

  auto &determinableEnts = dp.determinableEnts;

  /// build varriables and equations

  // vert start position in variable vector
  std::vector<int> ent2varPosition(cg.entities.size(), -1);
  std::vector<int> ent2nvar(cg.entities.size(), -1);
  std::vector<DenseMatd> ent2matFromVarToPlaneCoeffs(cg.entities.size());
  std::vector<int> var2ent;
  int rootVarPos = -1;
  int nvars = 0;
  for (int ent : determinableEnts) {
    auto &e = cg.entities[ent];
    int nvar = e.supportingPlane.dof;
    ent2nvar[ent] = nvar;
    ent2varPosition[ent] = nvars;
    if (ent == dp.rootEnt) {
      assert(nvar == 1);
      rootVarPos = nvars;
    }
    ent2matFromVarToPlaneCoeffs[ent] =
        e.supportingPlane.matFromVarsToPlaneCoeffs();
    var2ent.insert(var2ent.end(), (size_t)nvar, ent);
    nvars += nvar;
  }

  // connectivity term
  // the elements of sparse matrix A
  // inverse depths of vert1s on each anchor A1 * X
  // inverse depths of vert2s on each anchor A2 * X
  // weight on each anchor W
  std::vector<SparseMatElementd> A1triplets, A2triplets;
  std::vector<double> WA;
  int eidA = 0;

  // coplanarity adjacencies
  std::vector<SparseMatElementd> C1triplets, C2triplets;
  std::vector<double> WC;
  int eidC = 0;

  for (int cons : dp.consBetweenDeterminableEnts) {
    auto &constraint = cg.constraints[cons];
    auto &entity1 = cg.entities[constraint.ent1];
    auto &entity2 = cg.entities[constraint.ent2];
    int varpos1 = ent2varPosition[constraint.ent1];
    int varpos2 = ent2varPosition[constraint.ent2];

    assert(constraint.weight >= 0);
    if (constraint.isConnection()) { // connections
      double eachWeight = constraint.weight / sqrt(constraint.anchors.size());
      // [3 x k1]
      auto &matFromVarsToPlaneCoeffs1 =
          ent2matFromVarToPlaneCoeffs[constraint.ent1];
      // [3 x k2]
      auto &matFromVarsToPlaneCoeffs2 =
          ent2matFromVarToPlaneCoeffs[constraint.ent2];

      for (auto &anchor : constraint.anchors) {
        auto nanchor = normalize(anchor);
        // [1 x k1]
        {
          DenseMatd matFromVarsToInverseDepth1 =
              DenseMatd(nanchor).t() * matFromVarsToPlaneCoeffs1;
          assert(matFromVarsToInverseDepth1.rows == 1);
          assert(matFromVarsToInverseDepth1.cols ==
                 entity1.supportingPlane.dof);
          for (int k = 0; k < matFromVarsToInverseDepth1.cols; k++) {
            A1triplets.emplace_back(eidA, varpos1 + k,
                                    matFromVarsToInverseDepth1(0, k));
          }
        }
        // [1 x k2]
        {
          DenseMatd matFromVarsToInverseDepth2 =
              DenseMatd(nanchor).t() * matFromVarsToPlaneCoeffs2;
          assert(matFromVarsToInverseDepth2.rows == 1);
          assert(matFromVarsToInverseDepth2.cols ==
                 entity2.supportingPlane.dof);
          for (int k = 0; k < matFromVarsToInverseDepth2.cols; k++) {
            A2triplets.emplace_back(eidA, varpos2 + k,
                                    matFromVarsToInverseDepth2(0, k));
          }
        }
        WA.push_back(eachWeight);
        eidA++;
      }
    } else if (constraint.isCoplanarity()) { // coplanarities
      assert(entity1.isSeg() && entity2.isSeg());
      // [3 x k1]
      auto &matFromVarsToPlaneCoeffs1 =
          ent2matFromVarToPlaneCoeffs[constraint.ent1];
      // [3 x k2]
      auto &matFromVarsToPlaneCoeffs2 =
          ent2matFromVarToPlaneCoeffs[constraint.ent2];

      for (int k = 0; k < 3; k++) {
        for (int s = 0; s < matFromVarsToPlaneCoeffs1.cols; s++) {
          C1triplets.emplace_back(eidC, varpos1 + s,
                                  matFromVarsToPlaneCoeffs1(k, s));
        }
        for (int s = 0; s < matFromVarsToPlaneCoeffs2.cols; s++) {
          C2triplets.emplace_back(eidC, varpos2 + s,
                                  matFromVarsToPlaneCoeffs2(k, s));
        }
        WC.push_back(constraint.weight);
        eidC++;
      }
    }
  }

  matlab << "clear;";

  int neqsA = eidA;
  matlab.setVar("A1", MakeSparseMatFromElements(
                          neqsA, nvars, A1triplets.begin(), A1triplets.end()));
  matlab.setVar("A2", MakeSparseMatFromElements(
                          neqsA, nvars, A2triplets.begin(), A2triplets.end()));
  matlab << "A1(isnan(A1)) = 0;";
  matlab << "A2(isnan(A2)) = 0;";

  int neqsC = eidC;
  if (neqsC != 0) {
    matlab.setVar("C1",
                  MakeSparseMatFromElements(neqsC, nvars, C1triplets.begin(),
                                            C1triplets.end()));
    matlab.setVar("C2",
                  MakeSparseMatFromElements(neqsC, nvars, C2triplets.begin(),
                                            C2triplets.end()));
  } else {
    matlab << "C1 = zeros(0, size(A1, 2));";
    matlab << "C2 = zeros(0, size(A2, 2));";
  }
  matlab << "C1(isnan(C1)) = 0;";
  matlab << "C2(isnan(C2)) = 0;";

  matlab.setVar("WA", cv::Mat(WA));
  matlab.setVar("WC", cv::Mat(WC));

  matlab << "m = size(A1, 1);"; // number of connection equations
  matlab << "n = size(A1, 2);"; // number of variables
  matlab << "p = size(C1, 1);"; // number of coplanarity equations

  matlab.setVar("s", connectionWeightRatioOverCoplanarity);

  double minE = std::numeric_limits<double>::infinity();

  std::vector<double> X;

  {
    matlab << "D1D2 = ones(m, 1);"; //  current depths of anchors

    for (int t = 0; t < maxIter; t++) {

      matlab << "K = (A1 - A2) .* repmat(D1D2 .* WA, [1, n]);";
      matlab << "R = (C1 - C2) .* repmat(WC, [1, n]);";

      const std::string objectiveStr =
          useCoplanarity
              ? "sum_square(K * X) * 1e6 + sum_square(R * X) * (1e6 / s)"
              : "sum_square(K * X) * 1e6";

      matlab << "cvx_begin"
             << "variable X(n);" << ("minimize " + objectiveStr + ";")
             << "subject to"
             << "    ones(m, 1) <= A1 * X;"
             << "    ones(m, 1) <= A2 * X;"
             << "cvx_end";
      matlab << "D1D2 = 1./ (A1 * X) ./ (A2 * X);";
      matlab << "D1D2 = abs(D1D2) / norm(D1D2);";
      matlab << "K = (A1 - A2) .* repmat(D1D2 .* WA, [1, n]);";

      matlab << ("e = " + objectiveStr + ";");
      double curE = matlab.var("e").scalar();
      std::cout << "e = " << curE << std::endl;
      if (IsInfOrNaN(curE)) {
        break;
      }
      if (curE < minE) {
        minE = curE;
        matlab << "X = 2 * X ./ median((A1 + A2) * X);";
        X = matlab.var("X").toCVMat();
      } else {
        break;
      }
    }
  }

  if (IsInfOrNaN(minE)) {
    return minE;
  }

  // install results
  for (int ent : determinableEnts) {
    int varpos = ent2varPosition[ent];
    auto &e = cg.entities[ent];
    int nvar = ent2nvar[ent];
    for (int k = varpos; k < varpos + nvar; k++) {
      assert(var2ent[k] == ent);
    }

    auto &matFromVarToPlaneCoeffs = ent2matFromVarToPlaneCoeffs[ent];
    // [3 x 1] = [3 x nvar] * [nvar x 1]
    DenseMatd planeCoeffs =
        matFromVarToPlaneCoeffs * DenseMatd(nvar, 1, X.data() + varpos);
    assert(planeCoeffs.rows == 3 && planeCoeffs.cols == 1);

    e.supportingPlane.reconstructed = Plane3FromEquation(
        planeCoeffs(0, 0), planeCoeffs(1, 0), planeCoeffs(2, 0));
  }

  return minE;
}

int DisableUnsatisfiedConstraints(
    const PICGDeterminablePart &dp, PIConstraintGraph &cg,
    const std::function<bool(double distRankRatio, double avgDist,
                             double maxDist)> &whichToDisable) {

  std::vector<int> connections;
  std::vector<double> cons2avgDist(cg.constraints.size(), -1.0);
  std::vector<double> cons2maxDist(cg.constraints.size(), -1.0);
  for (int cons : dp.consBetweenDeterminableEnts) {
    auto &c = cg.constraints[cons];
    if (!c.isConnection()) {
      continue;
    }
    connections.push_back(cons);
    double avgDist = 0.0;
    double maxDist = 0.0;
    auto &plane1 = cg.entities[c.ent1].supportingPlane.reconstructed;
    auto &plane2 = cg.entities[c.ent2].supportingPlane.reconstructed;
    for (auto &anchor : c.anchors) {
      double d1 = norm(Intersection(Ray3(Origin(), anchor), plane1));
      double d2 = norm(Intersection(Ray3(Origin(), anchor), plane2));
      avgDist += abs(d1 - d2);
      maxDist = std::max(maxDist, abs(d1 - d2));
    }
    avgDist /= c.anchors.size();
    cons2avgDist[cons] = avgDist;
    cons2maxDist[cons] = maxDist;
  }

  std::sort(connections.begin(), connections.end(),
            [&cg, &cons2avgDist](int cons1, int cons2) {
              assert(cons2avgDist[cons1] > 0 && cons2avgDist[cons2] > 0);
              return cons2avgDist[cons1] > cons2avgDist[cons2];
            });

  int disabledNum = 0;
  for (int i = 0; i < connections.size(); i++) {
    double distRankRatio = i / double(connections.size());
    if (!cg.cons2enabled[connections[i]]) {
      continue;
    }
    if (whichToDisable(distRankRatio, cons2avgDist[connections[i]],
                       cons2maxDist[connections[i]])) {
      cg.cons2enabled[connections[i]] = false;
      disabledNum++;
    }
  }
  return disabledNum;
}

void DisorientDanglingLines(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg, double ratio) {
  std::vector<double> line2meanConsDist(mg.nlines(), 0.0);
  for (int line = 0; line < mg.nlines(); line++) {
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    if (cg.entities[ent].supportingPlane.dof == 2) {
      continue;
    }
    double distSum = 0.0;
    int distCount = 0;
    for (int cons : cg.ent2cons[ent]) {
      for (auto &anchor : cg.constraints[cons].anchors) {
        int ent1 = cg.constraints[cons].ent1;
        int ent2 = cg.constraints[cons].ent2;
        auto &plane1 = cg.entities[ent1].supportingPlane.reconstructed;
        auto &plane2 = cg.entities[ent2].supportingPlane.reconstructed;
        auto p1 = Intersection(Ray3(Origin(), anchor), plane1);
        auto p2 = Intersection(Ray3(Origin(), anchor), plane2);
        double dist = Distance(p1, p2);
        distSum += dist;
        distCount++;
      }
    }
    line2meanConsDist[line] = distSum / distCount;
  }

  std::vector<int> orderedLineIds(mg.nlines());
  std::iota(orderedLineIds.begin(), orderedLineIds.end(), 0);
  std::sort(orderedLineIds.begin(), orderedLineIds.end(),
            [&line2meanConsDist](int a, int b) {
              return line2meanConsDist[a] > line2meanConsDist[b];
            });

  for (int i = 0; i < orderedLineIds.size() * ratio; i++) {
    int line = orderedLineIds[i];
    mg.lines[line].claz = -1;
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    cg.entities[ent].supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.lines[line], mg.vps);
  }
}

void DisorientDanglingLines2(const PICGDeterminablePart &dp,
                             PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                             double thresRatio) {
  std::vector<double> line2meanSegDistRatio(mg.nlines(), 0.0);
  for (int line = 0; line < mg.nlines(); line++) {
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    if (cg.entities[ent].supportingPlane.dof == 2) {
      continue;
    }
    double distSum = 0.0;
    int distCount = 0;
    for (int cons : cg.ent2cons[ent]) {
      for (auto &anchor : cg.constraints[cons].anchors) {
        int ent1 = cg.constraints[cons].ent1;
        int ent2 = cg.constraints[cons].ent2;
        if (cg.entities[ent1].isLine() && cg.entities[ent2].isLine()) {
          continue;
        }
        auto &plane1 = cg.entities[ent1].supportingPlane.reconstructed;
        auto &plane2 = cg.entities[ent2].supportingPlane.reconstructed;
        auto p1 = Intersection(Ray3(Origin(), anchor), plane1);
        auto p2 = Intersection(Ray3(Origin(), anchor), plane2);
        double dist = Distance(p1, p2);
        distSum += dist;
        distCount++;
      }
    }
    line2meanSegDistRatio[line] =
        distSum / std::max(distCount, 1) /
        AngleBetweenDirected(mg.lines[line].component.first,
                               mg.lines[line].component.second);
  }

  for (int line = 0; line < mg.nlines(); line++) {
    if (line2meanSegDistRatio[line] < thresRatio) {
      continue;
    }
    mg.lines[line].claz = -1;
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    cg.entities[ent].supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.lines[line], mg.vps);
  }
}

void DisorientDanglingLines3(const PICGDeterminablePart &dp,
                             PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                             double disorientRatio, double thresRatio) {
  if (disorientRatio == 0.0)
    return;
  std::vector<double> line2maxEndDistToNearestSeg(mg.nlines(), 0.0);
  for (int line = 0; line < mg.nlines(); line++) {
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    auto &e = cg.entities[ent];
    auto &sp = e.supportingPlane.reconstructed;

    double &ndist = line2maxEndDistToNearestSeg[line];
    for (Vec3 dir :
         {mg.lines[line].component.first, mg.lines[line].component.second}) {
      double distToThis = std::numeric_limits<double>::max();

      auto &linePlane = cg.entities[ent].supportingPlane.reconstructed;
      auto pOnLine = Intersection(Ray3(Origin(), dir), linePlane);
      for (int cons : cg.ent2cons[ent]) {
        int segEnt = cg.constraints[cons].ent1;
        if (segEnt == ent) {
          segEnt = cg.constraints[cons].ent2;
        }
        if (cg.entities[segEnt].isLine()) {
          continue;
        }
        for (auto &anchor : cg.constraints[cons].anchors) {
          auto &segPlane = cg.entities[segEnt].supportingPlane.reconstructed;
          auto pOnSegPlane = Intersection(Ray3(Origin(), anchor), segPlane);
          double distance = Distance(pOnLine, pOnSegPlane);
          if (distance < distToThis) {
            distToThis = distance;
          }
        }
      }

      if (distToThis > ndist) {
        ndist = distToThis;
      }
    }
  }

  std::vector<int> lineids(mg.nlines());
  std::iota(lineids.begin(), lineids.end(), 0);
  std::sort(lineids.begin(), lineids.end(), [line2maxEndDistToNearestSeg](
                                                int a, int b) {
    return line2maxEndDistToNearestSeg[a] > line2maxEndDistToNearestSeg[b];
  });

  for (int i = 0; i < lineids.size() * disorientRatio; i++) {
    int line = lineids[i];
    double maxEndDist = line2maxEndDistToNearestSeg[line];
    if (maxEndDist < thresRatio) {
      continue;
    }
    if (mg.lines[line].claz == -1) {
      mg.line2used[line] = false;
      continue;
    }
    mg.lines[line].claz = -1;
    int ent = cg.line2ent[line];
    if (ent == -1) {
      continue;
    }
    cg.entities[ent].supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.lines[line], mg.vps);
  }
}

void DisorientDanglingSegs(const PICGDeterminablePart &dp,
                           PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                           double thresRatio) {
  std::vector<double> seg2meanDistRatio(mg.nsegs, 0.0);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    if (cg.entities[ent].supportingPlane.dof > 1) {
      continue;
    }
    double distSum = 0.0;
    int distCount = 0;
    for (int cons : cg.ent2cons[ent]) {
      for (auto &anchor : cg.constraints[cons].anchors) {
        int ent1 = cg.constraints[cons].ent1;
        int ent2 = cg.constraints[cons].ent2;
        auto &plane1 = cg.entities[ent1].supportingPlane.reconstructed;
        auto &plane2 = cg.entities[ent2].supportingPlane.reconstructed;
        auto p1 = Intersection(Ray3(Origin(), anchor), plane1);
        auto p2 = Intersection(Ray3(Origin(), anchor), plane2);
        double dist = Distance(p1, p2);
        distSum += dist;
        distCount++;
      }
    }
    seg2meanDistRatio[seg] = distSum / std::max(distCount, 1);
  }
  for (int seg = 0; seg < mg.nsegs; seg++) {
    if (seg2meanDistRatio[seg] < thresRatio) {
      continue;
    }
    mg.seg2control[seg].orientationClaz = -1;
    mg.seg2control[seg].orientationNotClaz = -1;
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    cg.entities[ent].supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.seg2control[seg],
                                                   mg.seg2center[seg], mg.vps);
  }
}

void DisorientDanglingSegs2(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                            double thresRatio) {

  std::vector<bool> seg2determined(mg.nsegs, false);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }

    double distSum = 0.0;
    int distCount = 0;
    for (int cons : cg.ent2cons[ent]) {
      int ent1 = cg.constraints[cons].ent1;
      int ent2 = cg.constraints[cons].ent2;
      auto &e1 = cg.entities[ent1];
      auto &e2 = cg.entities[ent2];
      if (e1.isLine() || e2.isLine()) {
        continue;
      }
      for (auto &anchor : cg.constraints[cons].anchors) {
        auto &plane1 = e1.supportingPlane.reconstructed;
        auto &plane2 = e2.supportingPlane.reconstructed;
        auto p1 = Intersection(Ray3(Origin(), anchor), plane1);
        auto p2 = Intersection(Ray3(Origin(), anchor), plane2);
        double dist = Distance(p1, p2);
        distSum += dist;
        distCount++;
      }
    }
    double distMean = distSum / distCount;
    if (distMean <= thresRatio) {
      seg2determined[seg] = true;
    }
  }

  if (true) {
    auto pim =
        PrintPIGraph(mg,
                     [&seg2determined](int seg) {
                       return seg2determined[seg] ? gui::Red : gui::White;
                     },
                     ConstantFunctor<gui::Color>(gui::Transparent),
                     ConstantFunctor<gui::Color>(gui::Gray), 2, 0);
    gui::AsCanvas(pim).show(0, "determined segs");
  }
}

void DisorientDanglingSegs3(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                            double disorientRatio, double thresRatio) {

  if (disorientRatio == 0.0) {
    return;
  }
  std::vector<double> seg2maxCornerDistToNearestSeg(mg.nsegs, 0.0);
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    auto &e = cg.entities[ent];
    auto &sp = e.supportingPlane.reconstructed;

    double &ndist = seg2maxCornerDistToNearestSeg[seg];
    for (int cons : cg.ent2cons[ent]) {
      for (auto &anchor : cg.constraints[cons].anchors) {
        int ent1 = cg.constraints[cons].ent1;
        int ent2 = cg.constraints[cons].ent2;
        auto &plane1 = cg.entities[ent1].supportingPlane.reconstructed;
        auto &plane2 = cg.entities[ent2].supportingPlane.reconstructed;
        auto p1 = Intersection(Ray3(Origin(), anchor), plane1);
        auto p2 = Intersection(Ray3(Origin(), anchor), plane2);
        double dist = Distance(p1, p2);
        if (dist > ndist) {
          ndist = dist;
        }
      }
    }

    std::vector<int> segids(mg.nsegs);
    std::iota(segids.begin(), segids.end(), 0);
    std::sort(segids.begin(), segids.end(),
              [&seg2maxCornerDistToNearestSeg](int a, int b) {
                return seg2maxCornerDistToNearestSeg[a] >
                       seg2maxCornerDistToNearestSeg[b];
              });

    int count = 0;
    for (int i = 0; i < mg.nsegs; i++) {
      int seg = segids[i];
      if (seg2maxCornerDistToNearestSeg[seg] < thresRatio) {
        continue;
      }
      count++;
    }

    for (int i = 0; i < count * disorientRatio; i++) {
      int seg = segids[i];
      if (seg2maxCornerDistToNearestSeg[seg] < thresRatio) {
        continue;
      }
      auto &control = mg.seg2control[seg];
      if (!control.used) {
        continue;
      }
      if (control.orientationClaz == -1 && control.orientationNotClaz == -1) {
        control.used = false;
        continue;
      }
      mg.seg2control[seg].orientationClaz = -1;
      mg.seg2control[seg].orientationNotClaz = -1;
      int ent = cg.seg2ent[seg];
      if (ent == -1) {
        continue;
      }
      cg.entities[ent].supportingPlane =
          PIConstraintGraph::Entity::SupportingPlane(
              mg.seg2control[seg], mg.seg2center[seg], mg.vps);
    }
  }
}

void OverorientSkewSegs(const PICGDeterminablePart &dp, PIConstraintGraph &cg,
                        PIGraph<PanoramicCamera> &mg, double angleThres,
                        double positionAngleThres, double oriRatio) {
  std::vector<Scored<int>> seg2oriHint(mg.nsegs, ScoreAs(-1, 0.0));

  std::vector<int> segids;
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    if (!Contains(dp.determinableEnts, ent)) {
      continue;
    }
    auto center = mg.seg2center[seg];
    auto &plane = cg.entities[ent].supportingPlane.reconstructed;

    auto &hint = seg2oriHint[seg];
    for (int k = 0; k < mg.vps.size(); k++) {
      auto &vp = mg.vps[k];
      double posAngle = AngleBetweenUndirected(center, vp);
      double normalAngle = AngleBetweenUndirected(plane.normal, vp);
      if (posAngle < positionAngleThres && normalAngle < angleThres) {
        double score = Gaussian(posAngle, positionAngleThres) +
                       Gaussian(normalAngle, angleThres) * 10;
        if (score > hint.score) {
          hint.component = k;
          hint.score = score;
        }
      }
    }

    if (hint.score > 0) {
      segids.push_back(seg);
    }
  }

  std::sort(segids.begin(), segids.end(), [&seg2oriHint](int a, int b) {
    return seg2oriHint[a].score > seg2oriHint[b].score;
  });

  for (int i = 0; i < segids.size() * oriRatio; i++) {
    int seg = segids[i];
    mg.seg2control[seg].orientationClaz = seg2oriHint[seg].component;
    mg.seg2control[seg].orientationNotClaz = -1;
    int ent = cg.seg2ent[seg];
    if (ent == -1) {
      continue;
    }
    cg.entities[ent].supportingPlane =
        PIConstraintGraph::Entity::SupportingPlane(mg.seg2control[seg],
                                                   mg.seg2center[seg], mg.vps);
  }
}
}
}