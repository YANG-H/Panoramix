#pragma once

#include "pi_graph_occlusion.hpp"

namespace pano {
namespace experimental {
// PIConstraintGraph
struct PIConstraintGraph {
  struct Entity {
    enum Type { IsSeg, IsLine } type;
    bool isSeg() const { return type == IsSeg; }
    bool isLine() const { return type == IsLine; }
    int id;
    // supporting plane
    struct SupportingPlane {
      int dof;
      Vec3 center;
      Vec3 toward;
      Vec3 along;
      Plane3 reconstructed;
      explicit SupportingPlane();
      explicit SupportingPlane(const SegControl &control, const Vec3 &center,
                               const std::vector<Vec3> &vps);
      explicit SupportingPlane(const Classified<Line3> &line,
                               const std::vector<Vec3> &vps);

      DenseMatd matFromVarsToPlaneCoeffs() const;
      template <class Archiver> void serialize(Archiver &ar) {
        ar(dof, center, toward, along, reconstructed);
      }
    } supportingPlane;
    double size;
    template <class Archiver> void serialize(Archiver &ar) {
      ar(type, id, supportingPlane, size);
    }
  };
  // constraint
  struct Constraint {
    enum Type { Coplanarity, Connection } type;
    bool isCoplanarity() const { return type == Coplanarity; }
    bool isConnection() const { return type == Connection; }
    int ent1, ent2;
    double weight;
    std::vector<Vec3> anchors;
    template <class Archiver> void serialize(Archiver &ar) {
      ar(type, ent1, ent2, weight, anchors);
    }
  };

  std::vector<Entity> entities;
  std::vector<Constraint> constraints;
  std::vector<std::vector<int>> ent2cons;
  std::vector<bool> cons2enabled;

  std::vector<int> seg2ent;
  std::vector<int> line2ent;

  template <class Archiver> inline void serialize(Archiver &ar) {
    ar(entities, constraints, ent2cons, cons2enabled, seg2ent, line2ent);
  }
};

// PICGDeterminablePart
// the determinable subgraph of PIConstraintGraph
struct PICGDeterminablePart {
  int rootEnt;
  std::set<int> determinableEnts;
  std::set<int> consBetweenDeterminableEnts;
  bool empty() const { return rootEnt == -1; }
  template <class Archiver> inline void serialize(Archiver &ar) {
    ar(rootEnt, determinableEnts, consBetweenDeterminableEnts);
  }
};

PIConstraintGraph
BuildPIConstraintGraph(const PIGraph<PanoramicCamera> &mg,
                       double minAngleThresForAWideEdge,
                       double weightRatioForCoplanarityWithLines = 0.0);

PIConstraintGraph BuildPIConstraintGraph(
    const PIGraph<PanoramicCamera> &mg,
    const std::vector<LineSidingWeight> &lsw,
    const std::vector<std::array<std::set<int>, 2>> &line2leftRightSegs,
    double minAngleThresForAWideEdge);

PICGDeterminablePart LocateDeterminablePart(const PIConstraintGraph &cg,
                                            double angleThres, bool connectAll);

// from annotation
class PILayoutAnnotation;
PIConstraintGraph BuildPIConstraintGraph(const PILayoutAnnotation &anno,
                                         double minAngleThresForAWideEdge);

PIConstraintGraph
BuildPIConstraintGraphWithLines(const PILayoutAnnotation &anno,
                                double minAngleThresForAWideEdge);
}
}