#pragma once

#include <Eigen/Core>

#include "mesh.hpp"

namespace pano {
namespace core {

// Transform
template <class VertDataT, class HalfDataT, class FaceDataT,
          class TransformVertFunT,
          class TransformHalfFunT = std::identity<HalfDataT>,
          class TransformFaceFunT = std::identity<FaceDataT>>
auto Transform(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
               TransformVertFunT transVert,
               TransformHalfFunT transHalf = TransformHalfFunT(),
               TransformFaceFunT transFace = TransformFaceFunT());

// DepthFirstSearchOneTree
template <class VertDataT, class HalfDataT, class FaceDataT,
          class ConstVertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearchOneTree(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle root,
    ConstVertexCallbackT vCallBack);

// DepthFirstSearch
template <class VertDataT, class HalfDataT, class FaceDataT,
          class ConstVertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearch(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                      ConstVertexCallbackT vCallBack);

// DepthFirstSearch
template <class VertDataT, class HalfDataT, class FaceDataT,
          class VertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearch(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                      VertexCallbackT vCallBack);

// ConnectedComponents
template <
    class VertDataT, class HalfDataT, class FaceDataT,
    class ConstVertexTypeRecorderT // record(const MeshT & m, VertH v, int ccid)
    >
int ConnectedComponents(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        ConstVertexTypeRecorderT vtr);

// RemoveDanglingComponents
template <class VertDataT, class HalfDataT, class FaceDataT>
void RemoveDanglingComponents(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// SearchAndAddFaces (liqi's method)
template <class VertDataT, class HalfDataT, class FaceDataT,
          class EdgeParallelScorerT, // (const MeshT & mesh, HalfH h1, HalfH h2)
                                     // -> double
          class EdgeMaskerT          // (const MeshT & mesh, HalfH h) -> bool
          >
void SearchAndAddFaces(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                       EdgeParallelScorerT epj, EdgeMaskerT emsk);

// SearchAndAddFaces (liqi's method)
template <class VertDataT, class HalfDataT, class FaceDataT>
void SearchAndAddFaces(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// ConstructInternalLoopFrom (liqi)
// IntersectFunT: (HalfHandle, HalfHandle) -> bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeIntersectFunT>
auto ConstructInternalLoopFrom(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle initial,
    HalfEdgeIntersectFunT intersectFun);

// DecomposeOnInternalLoop (liqi)
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfHandleIterT, class FD1 = FaceDataT, class FD2 = FaceDataT>
std::pair<typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle,
          typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>
DecomposeOnInternalLoop(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        HalfHandleIterT loopBegin, HalfHandleIterT loopEnd,
                        FD1 &&faceData = FD1(), FD2 &&oppoFaceData = FD2());

// AssertEdgesAreStiched
template <class VertDataT, class HalfDataT, class FaceDataT>
inline void
AssertEdgesAreStiched(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

namespace {
struct CompareLoopDefault {
  template <class LoopT1, class LoopT2>
  inline bool operator()(const LoopT1 &l1, const LoopT2 &l2) const {
    return l1.size() < l2.size();
  }
};
}

// DecomposeAll
// IntersectFunT: (HalfHandle, HalfHandle) -> bool
// CompareLoopT: (Container of HalfHandle, Container of HalfHandle) -> bool, if
// first loop is better, return true, otherwise false
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeIntersectFunT, class CompareLoopT = CompareLoopDefault>
std::vector<
    std::pair<typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle,
              typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>>
DecomposeAll(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
             HalfEdgeIntersectFunT intersectFun,
             CompareLoopT compareLoop = CompareLoopT());

// FindUpperBoundOfDRF
// - HalfEdgeColinearFunT: (HalfEdgeIterT hhsBegin, HalfEdgeIterT hhsEnd) ->
// bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeColinearFunT, class FaceHandleIterT>
int FindUpperBoundOfDRF(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        FaceHandleIterT fhsBegin, FaceHandleIterT fhsEnd,
                        HalfEdgeColinearFunT colinearFun);

// SortFacesWithPlanarityDependency
// - VertColinearFunT: (VertHandleIterT vhs_begin, VertHandleIterT vhs_end) ->
// bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class VertColinearFunT, class FaceHandleIterT,
          class FaceDependencyCallbackFunT = Dummy>
std::unordered_set<typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle>
SortFacesWithPlanarityDependency(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    FaceHandleIterT fhs_begin, FaceHandleIterT fhs_end,
    VertColinearFunT vhs_colinear_fun,
    FaceDependencyCallbackFunT face_dependency_callback_fun =
        FaceDependencyCallbackFunT());

// MakeTetrahedron
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeTetrahedron(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// MakeQuadFacedCube
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeQuadFacedCube(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// MakeTriFacedCube
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeTriFacedCube(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// MakeIcosahedron
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeIcosahedron(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// MakePrism
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakePrism(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
               double height);

// MakeCone
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeCone(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
              double height);

// MakeStarPrism
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeStarPrism(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
                   double innerRadius, double outerRadius, double height);

// MakeMeshProxy
template <class VertDataT, class HalfDataT, class FaceDataT>
Mesh<typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle,
     typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle,
     typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>
MakeMeshProxy(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh);

// LoadFromObjFile
Mesh<Point3> LoadMeshFromObjFile(const std::string &fname);
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace core {

// Transform
template <class VertDataT, class HalfDataT, class FaceDataT,
          class TransformVertFunT, class TransformHalfFunT,
          class TransformFaceFunT>
auto Transform(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
               TransformVertFunT transVert, TransformHalfFunT transHalf,
               TransformFaceFunT transFace) {
  using ToVertDataT =
      std::decay_t<typename FunctionTraits<TransformVertFunT>::ResultType>;
  using ToHalfDataT =
      std::decay_t<typename FunctionTraits<TransformHalfFunT>::ResultType>;
  using ToFaceDataT =
      std::decay_t<typename FunctionTraits<TransformFaceFunT>::ResultType>;
  Mesh<ToVertDataT, ToHalfDataT, ToFaceDataT> toMesh;
  toMesh.internalVertices().reserve(mesh.internalVertices().size());
  toMesh.internalHalfEdges().reserve(mesh.internalHalfEdges().size());
  toMesh.internalFaces().reserve(mesh.internalFaces().size());
  std::transform(
      mesh.internalVertices().begin(), mesh.internalVertices().end(),
      std::back_inserter(toMesh.internalVertices()), [transVert](auto &&from) {
        return Triplet<Mesh<VertDataT, HalfDataT, FaceDataT>::VertTopo,
                       ToVertDataT>(from.topo, transVert(from.data),
                                    from.exists);
      });
  std::transform(
      mesh.internalHalfEdges().begin(), mesh.internalHalfEdges().end(),
      std::back_inserter(toMesh.internalHalfEdges()), [transHalf](auto &&from) {
        return Triplet<Mesh<VertDataT, HalfDataT, FaceDataT>::HalfTopo,
                       ToHalfDataT>(from.topo, transHalf(from.data),
                                    from.exists);
      });
  std::transform(
      mesh.internalFaces().begin(), mesh.internalFaces().end(),
      std::back_inserter(toMesh.internalFaces()), [transFace](auto &&from) {
        return Triplet<Mesh<VertDataT, HalfDataT, FaceDataT>::FaceTopo,
                       ToFaceDataT>(from.topo, transFace(from.data),
                                    from.exists);
      });
  return toMesh;
}

// DepthFirstSearchOneTree
template <class VertDataT, class HalfDataT, class FaceDataT,
          class ConstVertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearchOneTree(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle root,
    ConstVertexCallbackT vCallBack) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  assert(root.valid() && !mesh.removed(root));
  if (!vCallBack(mesh, root))
    return false;
  auto &halves = mesh.topo(root).halfedges;
  for (auto hh : halves) {
    if (mesh.removed(hh))
      continue;
    auto vh = mesh.topo(hh).to();
    if (!DepthFirstSearchOneTree(mesh, vh, vCallBack))
      return false;
  }
  return true;
}

// DepthFirstSearch
template <class VertDataT, class HalfDataT, class FaceDataT,
          class ConstVertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearch(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                      ConstVertexCallbackT vCallBack) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  struct {
    bool operator()(const MeshType &mesh, VertH root,
                    std::vector<bool> &vertVisited,
                    ConstVertexCallbackT vCallBack) const {

      assert(root.valid() && !mesh.removed(root));
      if (vertVisited[root.id])
        return true;
      if (!vCallBack(mesh, root))
        return false;
      vertVisited[root.id] = true;
      auto &halves = mesh.topo(root).halfedges;
      for (auto hh : halves) {
        if (mesh.removed(hh))
          continue;
        auto vh = mesh.topo(hh).to();
        if (!(*this)(mesh, vh, vertVisited, vCallBack))
          return false;
      }
      return true;
    };
  } depthFirstSearchOneTree;

  std::vector<bool> visited(mesh.internalVertices().size(), false);
  while (true) {
    VertH root;
    for (auto &h : mesh.vertices()) {
      if (!visited[h.topo.hd.id]) {
        root = h.topo.hd;
        break;
      }
    }
    if (root.invalid())
      break;
    if (!depthFirstSearchOneTree(mesh, root, vCallBack, visited))
      break;
  }
}

// DepthFirstSearch
template <class VertDataT, class HalfDataT, class FaceDataT,
          class VertexCallbackT // bool callback(const Mesh & m, VertH v)
          >
bool DepthFirstSearch(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                      VertexCallbackT vCallBack) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = VertHandle;
  using HalfH = HalfHandle;
  using FaceH = FaceHandle;

  struct {
    bool operator()(MeshType &mesh, VertH root, std::vector<bool> &vertVisited,
                    VertexCallbackT vCallBack) const {

      assert(root.valid() && !mesh.removed(root));
      if (vertVisited[root.id])
        return true;
      if (!vCallBack(mesh, root))
        return false;
      vertVisited[root.id] = true;
      auto &halves = mesh.topo(root).halfedges;
      for (auto hh : halves) {
        if (mesh.removed(hh))
          continue;
        auto vh = mesh.topo(hh).to();
        if (!(*this)(mesh, vh, vertVisited, vCallBack))
          return false;
      }
      return true;
    };
  } depthFirstSearchOneTree;

  std::vector<bool> visited(mesh.internalVertices().size(), false);
  while (true) {
    VertH root;
    for (auto &h : mesh.vertices()) {
      if (!visited[h.topo.hd.id]) {
        root = h.topo.hd;
        break;
      }
    }
    if (root.invalid())
      break;
    if (!depthFirstSearchOneTree(mesh, root, visited, vCallBack))
      break;
  }
}

// ConnectedComponents
template <
    class VertDataT, class HalfDataT, class FaceDataT,
    class ConstVertexTypeRecorderT // record(const MeshT & m, VertH v, int ccid)
    >
int ConnectedComponents(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        ConstVertexTypeRecorderT vtr) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  struct {
    bool operator()(const MeshType &mesh, VertH root,
                    std::vector<bool> &vertVisited,
                    ConstVertexTypeRecorderT vtr, int cid) const {

      assert(root.valid() && !mesh.removed(root));
      if (vertVisited[root.id])
        return true;
      vtr(mesh, root, cid);
      vertVisited[root.id] = true;
      auto &halves = mesh.topo(root).halfedges;
      for (auto hh : halves) {
        if (mesh.removed(hh))
          continue;
        auto vh = mesh.topo(hh).to();
        if (!(*this)(mesh, vh, vertVisited, vtr, cid))
          return false;
      }
      return true;
    };
  } depthFirstSearchOneTree;

  std::vector<bool> visited(mesh.internalVertices().size(), false);
  int cid = 0;
  while (true) {
    VertH root;
    for (auto &h : mesh.vertices()) {
      if (!visited[h.topo.hd.id]) {
        root = h.topo.hd;
        break;
      }
    }
    if (root.invalid())
      break;

    depthFirstSearchOneTree(mesh, root, visited, vtr, cid);
    cid++;
  }
  return cid;
}

// RemoveDanglingComponents
template <class VertDataT, class HalfDataT, class FaceDataT>
void RemoveDanglingComponents(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  std::vector<int> cdegrees(mesh.internalVertices().size(), 0);
  std::vector<int> danglcs;
  while (true) {
    std::fill(cdegrees.begin(), cdegrees.end(), 0);
    for (auto &he : mesh.halfedges()) {
      cdegrees[he.topo.from().id]++;
    }
    danglcs.clear();
    for (size_t i = 0; i < cdegrees.size(); i++) {
      if (cdegrees[i] < 2) {
        danglcs.push_back(i);
      }
    }
    if (danglcs.empty()) {
      break;
    }
    for (int danglc : danglcs) {
      mesh.remove(VertH(danglc));
    }
  }
}

// SearchAndAddFaces (liqi's method)
template <class VertDataT, class HalfDataT, class FaceDataT,
          class EdgeParallelScorerT, // (const MeshT & mesh, HalfH h1, HalfH h2)
                                     // -> double
          class EdgeMaskerT          // (const MeshT & mesh, HalfH h) -> bool
          >
void SearchAndAddFaces(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                       EdgeParallelScorerT epj, EdgeMaskerT emsk) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  // the loop struct for each half edge
  struct Loop {
    std::vector<HalfH> halfArr;
    int priority;
  };
  using LoopPtr = std::shared_ptr<Loop>;

  static const int PRIORITY_INIT = 100;
  static const int PRIORITY_TRI_LOOP = 110;
  static const int PRIORITY_QUA_LOOP = 150;
  static const int PRIORITY_2QUA_LOOP = 200;

  // loops for each half edge
  std::vector<LoopPtr> half2loop(mesh.internalHalfEdges().size());
  for (size_t i = 0; i < half2loop.size(); i++) {
    if (mesh.removed(HalfH(i)) || !emsk(mesh, HalfH(i)))
      continue;
    half2loop[i] = std::make_shared<Loop>();
    half2loop[i]->halfArr.push_back(mesh.internalHalfEdges()[i].topo.hd);
    half2loop[i]->priority = PRIORITY_INIT;
  }

  // search one triloop
  bool triLoopFound = false;
  std::array<HalfH, 3> triLoop;
  for (auto &v : mesh.vertices()) {
    // if(triLoopFound)
    //    break;
    for (size_t i = 0; i < v.topo.halfedges.size(); i++) {
      // if(triLoopFound)
      //    break;
      HalfH h1 = v.topo.halfedges[i]; // h1
      if (mesh.removed(h1) || !emsk(mesh, h1))
        continue;

      VertH h1to = mesh.topo(h1).to();           // -h1->h1to
      auto &h3cands = mesh.topo(h1to).halfedges; // -h1->h1to-h3->
      if (half2loop[v.topo.halfedges[i].id]->priority >
          PRIORITY_INIT) // to avoid same loops
        continue;

      for (size_t j = 0; j < v.topo.halfedges.size(); j++) {
        // if(triLoopFound)
        //    break;
        if (i == j)
          continue;
        HalfH h2 = v.topo.halfedges[j]; // h2
        if (mesh.removed(h2) || !emsk(mesh, h2))
          continue;

        VertH h2to = mesh.topo(h2).to(); // -h2->h2to
        for (auto h3cand : h3cands) {
          // if(triLoopFound)
          //    break;
          if (mesh.removed(h3cand) || !emsk(mesh, h3cand))
            continue;
          VertH h3candto = mesh.topo(h3cand).to(); // -h1->h1to-h3->h3to
          if (h2to == h3candto) { // v ->h2-> h2to/h3to <-h3- h1to <-h1- v
            HalfH h3 = h3cand;

            HalfH h1op = mesh.topo(h1).opposite;
            HalfH h2op = mesh.topo(h2).opposite;
            HalfH h3op = mesh.topo(h3).opposite;

            triLoop = {{h1, h3, h2op}};

            half2loop[h1.id]->priority = PRIORITY_TRI_LOOP;
            half2loop[h2.id]->priority = PRIORITY_TRI_LOOP;
            half2loop[h3.id]->priority = PRIORITY_TRI_LOOP;
            half2loop[h1op.id]->priority = PRIORITY_TRI_LOOP;
            half2loop[h2op.id]->priority = PRIORITY_TRI_LOOP;
            half2loop[h3op.id]->priority = PRIORITY_TRI_LOOP;

            triLoopFound = true;
            // break;
          }
        }
      }
    }
  }

  // search quad loops
  std::vector<std::array<HalfH, 4>> quadLoops;
  std::vector<std::unordered_set<int>> half2quadloopids(
      mesh.internalHalfEdges().size());

  for (size_t i = 0; i < mesh.internalHalfEdges().size(); i++) {
    HalfH h1(i);
    if (mesh.removed(h1) || !emsk(mesh, h1))
      continue;
    for (size_t j = i + 1; j < mesh.internalHalfEdges().size(); j++) {
      HalfH h2(j);
      if (mesh.removed(h2) || !emsk(mesh, h2))
        continue;

      // check parallelism
      if (epj(mesh, h1, h2) < 0.9)
        continue;

      // check whether share same quadloops
      bool shareQuadLoops = false;
      auto &qlh1 = half2quadloopids[h1.id];
      auto &qlh2 = half2quadloopids[h2.id];
      for (int ql1 : qlh1) {
        if (shareQuadLoops)
          break;
        for (int ql2 : qlh2) {
          if (ql1 == ql2) {
            shareQuadLoops = true;
            break;
          }
        }
      }
      if (shareQuadLoops)
        continue;

      VertH h1start = mesh.topo(h1).from();
      VertH h1end = mesh.topo(h1).to();
      VertH endVert;
      for (HalfH h1starth : mesh.topo(h1start).halfedges) {
        if (mesh.removed(h1starth) || !emsk(mesh, h1starth))
          continue;
        if (h1starth == h1)
          continue;

        if (mesh.topo(h1starth).to() == mesh.topo(h2).from())
          endVert = mesh.topo(h2).to();
        else if (mesh.topo(h1starth).to() == mesh.topo(h2).to())
          endVert = mesh.topo(h2).from();
        else
          continue;

        for (HalfH h1endh : mesh.topo(h1end).halfedges) {
          if (mesh.removed(h1endh) || !emsk(mesh, h1endh))
            continue;
          if (h1endh == h1)
            continue;

          if (mesh.topo(h1endh).to() == endVert) {
            if (!epj(mesh, h1endh, h1starth))
              continue;

            quadLoops.push_back({{h1, h1starth, h2, h1endh}});
            int quadid = quadLoops.size() - 1;
            for (HalfH hinLoop : quadLoops.back()) {
              HalfH hinLoopOp = mesh.topo(hinLoop).opposite;
              half2quadloopids[hinLoop.id].insert(quadid);
              half2quadloopids[hinLoopOp.id].insert(quadid);
            }
          }
        }
      }
    }
  }

  // set priorities for half edges
  for (auto h : mesh.halfedges()) {
    // std::cout << "quad face num: " << half2quadloopids[h.topo.hd.id].size()
    // << std::endl;
    if (h.topo.hd.id % 2 == 1)
      continue; // ignore opposite half edges
    if (half2quadloopids[h.topo.hd.id].size() > 1) {
      half2loop[h.topo.hd.id]->priority = PRIORITY_2QUA_LOOP;
      half2loop[h.topo.opposite.id]->priority = PRIORITY_2QUA_LOOP;
    } else if (half2quadloopids[h.topo.hd.id].size() == 1) {
      half2loop[h.topo.hd.id]->priority = PRIORITY_QUA_LOOP;
      half2loop[h.topo.opposite.id]->priority = PRIORITY_QUA_LOOP;
    }
  }

  std::list<LoopPtr> masterList(
      half2loop.begin(), half2loop.end()); // make master list from loop table
  masterList.remove(0);
  std::list<LoopPtr> workingCopy;
  std::list<LoopPtr> faceLoopArr; // collected faces

  // sort master list in priority's descending order
  masterList.sort([](const LoopPtr &l1, const LoopPtr &l2) -> bool {
    return l1->priority > l2->priority;
  });

  auto printMasterList = [&mesh,
                          &faceLoopArr](const std::list<LoopPtr> &masterList) {
    /*std::cout << "masterList/workingCopy:" << std::endl;
    for (LoopPtr l : masterList) {
      std::cout << "[" << l->priority << "(";
      for (HalfH h : l->halfArr) {
        std::cout << mesh.topo(h).from().id << "-";
      }
      std::cout << mesh.topo(l->halfArr.back()).to().id;
      std::cout << ")]";
    }
    std::cout << std::endl;
    std::cout << "identified faces:" << std::endl;
    for (LoopPtr l : faceLoopArr) {
      std::cout << "[" << l->priority << "(";
      for (HalfH h : l->halfArr) {
        std::cout << mesh.topo(h).from().id << "-";
      }
      std::cout << mesh.topo(l->halfArr.back()).to().id;
      std::cout << ")]";
    }
    std::cout << std::endl;*/
  };

  // insert into master list in priority's descending order
  const auto insertIntoMasterList = [](std::list<LoopPtr> &masterList,
                                       LoopPtr nLoop) {
    bool inserted = false;
    for (auto i = masterList.begin(); i != masterList.end(); ++i) {
      if ((*i)->priority < nLoop->priority) {
        masterList.insert(i, nLoop);
        inserted = true;
        break;
      }
    }
    if (!inserted) {
      masterList.push_back(nLoop);
    }
  };

  // compute matting values
  const auto mattingValue = [epj, &mesh](LoopPtr loop0, LoopPtr loop1) {
    if (loop0->halfArr.size() == 1 && loop1->halfArr.size() == 1)
      return 1.0;
    std::vector<double> values;
    values.reserve(loop0->halfArr.size() * loop1->halfArr.size());
    for (HalfH h0 : loop0->halfArr) {
      for (HalfH h1 : loop1->halfArr) {
        double v = epj(mesh, h0, h1);
        values.push_back(std::pow(v, 10.0));
      }
    }
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double avg = sum / values.size();
    auto nend = std::remove_if(values.begin(), values.end(),
                               [avg](double v) { return v < avg; });
    double result = std::accumulate(values.begin(), nend, 0.0) /
                    std::distance(values.begin(), nend);
    result = result == 0 ? 0.1 : result;
    // std::cout << "matting value: " << result << std::endl;
    return result;
  };

  // connect loops
  const auto connectLoop = [&half2loop, mattingValue,
                            &mesh](LoopPtr loop0, LoopPtr loop1) -> LoopPtr {
    assert(mesh.topo(loop0->halfArr.back()).to() ==
           mesh.topo(loop1->halfArr.front()).from());
    double mab = mattingValue(loop0, loop1);
    loop0->priority = (int)(0.4 * mab * (loop0->priority + loop1->priority));
    // insert all loop1 halfedges into loop0
    for (HalfH h : loop1->halfArr) {
      loop0->halfArr.push_back(h);
      half2loop[h.id] = loop0; // set loops of half edges
    }
    return loop0;
  };

  printMasterList(masterList);

  // first move
  auto firstMove = [&mesh, &triLoop, triLoopFound, &masterList, &half2loop,
                    &faceLoopArr, connectLoop, insertIntoMasterList, emsk]() {
    // find 3-deg vertex
    VertH v3;
    for (auto &v : mesh.vertices()) {
      size_t c = std::count_if(
          v.topo.halfedges.begin(), v.topo.halfedges.end(),
          [&mesh, emsk](HalfH h) { return !mesh.removed(h) && emsk(mesh, h); });
      if (c == 3) {
        v3 = v.topo.hd;
        break;
      }
    }
    if (v3.valid()) { // if found
      auto &halfs = mesh.topo(v3).halfedges;
      std::vector<HalfH> halfsarr;
      halfsarr.reserve(3);
      for (auto h : halfs) {
        if (!mesh.removed(h) && emsk(mesh, h)) {
          halfsarr.push_back(h);
        }
      }
      assert(halfsarr.size() == 3);
      for (size_t j = 0; j < 3; j++) {
        size_t jnext = (j + 1) % 3;
        HalfH hpos = mesh.topo(halfsarr[j]).opposite;
        HalfH hnext = mesh.topo(halfsarr[jnext]).hd;
        LoopPtr loop0 = half2loop[hpos.id];
        LoopPtr loop1 = half2loop[hnext.id];
        masterList.remove(loop0);
        masterList.remove(loop1);

        // make a new loop
        LoopPtr nLoop = connectLoop(loop0, loop1);
        if (mesh.topo(nLoop->halfArr.front()).from() ==
            mesh.topo(nLoop->halfArr.back()).to()) { // is closed!
          nLoop->priority = 0;
          faceLoopArr.push_back(nLoop);
          // TODO
          // detectAndPerformForceConnection();
          return 2;
        } else {
          // insert nLoop into masterList
          insertIntoMasterList(masterList, nLoop);
        }
      }
    } else if (triLoopFound) { // not found, remove a triloop
      LoopPtr loops[3];
      for (size_t i = 0; i < 3; i++) {
        loops[i] = half2loop[triLoop[i].id];
        masterList.remove(loops[i]);
      }
      LoopPtr nLoop = std::make_shared<Loop>();
      nLoop->halfArr = {triLoop[0], triLoop[1], triLoop[2]};
      nLoop->priority = 0;
      for (size_t i = 0; i < 3; i++) {
        half2loop[triLoop[i].id] = nLoop;
      }
      faceLoopArr.push_back(nLoop);
    } else { // no 3-deg vertex nor triloop
      std::cout << "no 3-deg vertex, no triloops!" << std::endl;
    }
    return 0;
  };

  // detect And Perform Force Connection Once
  // returns: 1 -> performed
  //          0 -> not found
  //          2 -> new face found
  const auto detectAndPerformForceConnectionOnce =
      [&half2loop, &mesh, &faceLoopArr, connectLoop, insertIntoMasterList,
       emsk](std::list<LoopPtr> &masterList) {

        // std::cout << "fouce connection ..." << std::endl;

        if (masterList.empty())
          return 0;
        for (LoopPtr loop : masterList) {
          HalfH endh = loop->halfArr.back(); // the end halfedge of loop
          VertH endv = mesh.topo(endh).to();
          LoopPtr nextLoop;
          int connectNum = 0;
          for (HalfH endvh : mesh.topo(endv).halfedges) {
            if (mesh.removed(endvh) || !emsk(mesh, endvh))
              continue;
            if (mesh.topo(endvh).opposite == endh) // ignore same edge
              continue;
            if (half2loop[endvh.id]->priority ==
                0) // face already recognized on its loop
              continue;
            if (half2loop[endvh.id]->halfArr.front() !=
                endvh) // endvh must be the start halfedge in its loop
              continue;
            if (connectNum == 0)
              nextLoop = half2loop[endvh.id];
            connectNum++;
            if (connectNum > 1)
              break;
          }
          if (connectNum != 1)
            continue;

          // force connection needed!
          masterList.remove(loop);
          masterList.remove(nextLoop);
          LoopPtr nLoop = connectLoop(loop, nextLoop);
          if (mesh.topo(nLoop->halfArr.front()).from() ==
              mesh.topo(nLoop->halfArr.back()).to()) {
            nLoop->priority = 0;
            faceLoopArr.push_back(nLoop);
            return 2;
          } else {
            insertIntoMasterList(masterList, nLoop);
            return 1;
          }
        }
        return 0;
      };

  // detect And Perform Merge Once
  // returns: 1 -> performed
  //          0 -> not found
  //          2 -> new face found
  auto detectAndPerformMergeOnce = [&half2loop, &mesh, &faceLoopArr,
                                    connectLoop,
                                    emsk](std::list<LoopPtr> &masterList) {

    // std::cout << "merge ..." << std::endl;

    if (masterList.empty())
      return 0;
    for (LoopPtr loop : masterList) {
      if (mesh.topo(loop->halfArr.front()).from() ==
          mesh.topo(loop->halfArr.back()).to()) { // this loop is closed
        masterList.remove(loop);
        loop->priority = 0;
        faceLoopArr.push_back(loop);
        return 2;
      }
      HalfH endh = loop->halfArr.back();
      VertH endv = mesh.topo(endh).to();              // end vertex of this loop
      for (HalfH endvh : mesh.topo(endv).halfedges) { // attached edges
        if (mesh.removed(endvh) || !emsk(mesh, endvh))
          continue;
        if (mesh.topo(endvh).opposite == endh)
          continue;
        if (half2loop[endvh.id]->priority ==
            0) // face already recognized on its loop
          continue;
        if (half2loop[endvh.id]->halfArr.front() !=
            endvh) // endvh must be the start halfedge in its loop
          continue;
        LoopPtr loop1 = half2loop[endvh.id];
        if (mesh.topo(loop->halfArr.front()).from() !=
            mesh.topo(loop1->halfArr.back())
                .to()) // loop and loop1 form a circle!
          continue;

        // opposite loops of loop and loop1 should not be the same
        if (half2loop[mesh.topo(loop->halfArr.back()).opposite.id] ==
            half2loop[mesh.topo(loop1->halfArr.front()).opposite.id])
          continue;

        masterList.remove(loop);
        masterList.remove(loop1);
        LoopPtr nLoop = connectLoop(loop, loop1);
        assert(mesh.topo(nLoop->halfArr.front()).from() ==
               mesh.topo(nLoop->halfArr.back()).to());
        nLoop->priority = 0;
        faceLoopArr.push_back(nLoop);

        return 2;
      }
    }

    return 0;
  };

  // perform best merge once
  auto performBestMergeOnce = [&half2loop, &mesh, &faceLoopArr, connectLoop,
                               insertIntoMasterList, mattingValue,
                               emsk](std::list<LoopPtr> &masterList) {

    // std::cout << "try best merge ..." << std::endl;
    if (masterList.empty()) {
      std::cout << "current list is empty!" << std::endl;
      return 0;
    }

    LoopPtr loop = masterList.front();
    // find the non-minimun loop with the highest priority
    for (LoopPtr l : masterList) {
      loop = l;
      if (loop->halfArr.size() > 1)
        break;
    }
    if (loop->halfArr.size() ==
        1) // if all loops are minimum, then choose the first
      loop = masterList.front();

    if (mesh.topo(loop->halfArr.front()).from() ==
        mesh.topo(loop->halfArr.back()).to()) { // this loop is closed
      masterList.remove(loop);
      loop->priority = 0;
      faceLoopArr.push_back(loop);
      return 2;
    }

    HalfH endh = loop->halfArr.back();
    VertH endv = mesh.topo(endh).to(); // end vertex of this loop
    HalfH bestEndvh;
    double bestMattingValue = -1;
    LoopPtr bestLoop1;
    for (HalfH endvh : mesh.topo(endv).halfedges) { // attached edges
      if (mesh.removed(endvh) || !emsk(mesh, endvh))
        continue;
      if (mesh.topo(endvh).opposite == endh)
        continue;
      if (half2loop[endvh.id]->priority ==
          0) // face already recognized on its loop
        continue;
      if (half2loop[endvh.id]->halfArr.front() !=
          endvh) // endvh must be the start halfedge in its loop
        continue;
      LoopPtr loop1 = half2loop[endvh.id];
      // if(mesh.topo(loop->halfArr.front()).from() !=
      // mesh.topo(loop1->halfArr.back()).to()) // loop and loop1 form a circle!
      //    continue;

      // opposite loops of loop and loop1 should not be the same
      if (half2loop[mesh.topo(loop->halfArr.back()).opposite.id] ==
          half2loop[mesh.topo(loop1->halfArr.front()).opposite.id])
        continue;

      double mab = mattingValue(loop, loop1);
      if (mab > bestMattingValue) {
        bestMattingValue = mab;
        bestEndvh = endvh;
        bestLoop1 = loop1;
      }
    }

    if (bestEndvh.invalid())
      return 0;

    masterList.remove(loop);
    masterList.remove(bestLoop1);
    LoopPtr nLoop = connectLoop(loop, bestLoop1);
    if (mesh.topo(nLoop->halfArr.front()).from() ==
        mesh.topo(nLoop->halfArr.back()).to()) {
      nLoop->priority = 0;
      faceLoopArr.push_back(nLoop);
      return 2;
    } else {
      insertIntoMasterList(masterList, nLoop);
      return 1;
    }
  };

  // workflow
  int result = -1;
  result = firstMove();
  printMasterList(masterList);
  switch (result) {
  case 0:
  case 1:
    goto Label_TopForce;
    break;
  case 2:
    goto Label_TopMerge;
    break;
  }

Label_TopForce:
  if (masterList.empty())
    goto Label_End;
  result = detectAndPerformForceConnectionOnce(masterList);
  printMasterList(masterList);
  switch (result) {
  case 0:
    goto Label_TopMerge;
    break;
  case 1:
    goto Label_TopForce;
    break;
  case 2:
    goto Label_TopForce;
    break;
  }

Label_TopMerge:
  if (masterList.empty())
    goto Label_End;
  result = detectAndPerformMergeOnce(masterList);
  printMasterList(masterList);
  switch (result) {
  case 0:
    workingCopy = masterList;
    goto Label_SubMatting;
    break;
  case 1:
  case 2:
    goto Label_TopForce;
    break;
  }

Label_SubMatting:
  if (masterList.empty())
    goto Label_End;
  result = performBestMergeOnce(workingCopy);
  printMasterList(workingCopy);
  switch (result) {
  case 0:
  case 1:
    goto Label_SubForce;
    break;
  case 2:
    masterList = workingCopy;
    goto Label_TopForce;
    break;
  }

Label_SubForce:
  if (masterList.empty())
    goto Label_End;
  result = detectAndPerformForceConnectionOnce(workingCopy);
  printMasterList(workingCopy);
  switch (result) {
  case 0:
    goto Label_SubMerge;
    break;
  case 1:
    goto Label_SubForce;
    break;
  case 2:
    masterList = workingCopy;
    goto Label_TopForce;
    break;
  }

Label_SubMerge:
  if (masterList.empty())
    goto Label_End;
  result = detectAndPerformMergeOnce(workingCopy);
  printMasterList(workingCopy);
  switch (result) {
  case 0:
  case 1:
    goto Label_SubMatting;
    break;
  case 2:
    masterList = workingCopy;
    goto Label_TopForce;
    break;
  }

Label_End:

  std::cout << "face num: " << faceLoopArr.size() << std::endl;
  for (LoopPtr loop : faceLoopArr) {
    mesh.addFace(loop->halfArr);
  }
}

// SearchAndAddFaces (liqi's method)
template <class VertDataT, class HalfDataT, class FaceDataT>
void SearchAndAddFaces(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  using MeshType = Mesh<VertDataT, HalfDataT, FaceDataT>;
  using VertH = typename MeshType::VertHandle;
  using HalfH = typename MeshType::HalfHandle;
  using FaceH = typename MeshType::FaceHandle;

  auto eParallelJudger = [](const MeshType &mesh, HalfH h1, HalfH h2) {
    VertH h1v1 = mesh.topo(h1).from();
    VertH h1v2 = mesh.topo(h1).to();
    VertH h2v1 = mesh.topo(h2).from();
    VertH h2v2 = mesh.topo(h2).to();

    VertDataT h1p1 = mesh.data(h1v1);
    VertDataT h1p2 = mesh.data(h1v2);
    VertDataT h2p1 = mesh.data(h2v1);
    VertDataT h2p2 = mesh.data(h2v2);
    VertDataT h1v = h1p1 - h1p2;
    VertDataT h2v = h2p1 - h2p2;

    return std::abs(double(normalize(h1v).dot(normalize(h2v))));
  };

  std::vector<int> vccids(mesh.internalVertices().size(), -1);
  int ccnum =
      ConnectedComponents(mesh, [&vccids](const MeshType &, VertH vh, int cid) {
        vccids[vh.id] = cid;
      });

  for (int i = 0; i < ccnum; i++) {
    auto eMasker = [&i, &vccids](const MeshType &m, HalfH h) {
      return vccids[m.topo(h).from().id] == i;
    };
    SearchAndAddFaces(mesh, eParallelJudger, eMasker);
  }
}

// ConstructInternalLoopFrom (liqi)
// IntersectFunT: (HalfHandle, HalfHandle) -> bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeIntersectFunT>
auto ConstructInternalLoopFrom(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle initial,
    HalfEdgeIntersectFunT intersectFun) {

  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  if (mesh.degree(mesh.topo(initial).from()) < 4 ||
      mesh.degree(mesh.topo(initial).to()) < 4) {
    return std::list<HalfHandle>();
  }

  VertHandle startV = mesh.topo(initial).from();
  std::queue<std::list<HalfHandle>> Q;
  Q.push({initial});

  while (!Q.empty()) {
    auto curPath = std::move(Q.front());
    Q.pop();

    // if curPath is closed?
    if (mesh.topo(curPath.back()).to() == startV) {
      return curPath;
    }

    VertHandle endV = mesh.topo(curPath.back()).to();
    std::vector<HalfHandle> validNextHs;
    for (HalfHandle nexth : mesh.topo(endV).halfedges) {
      // next edge should not coincide with current path
      if (Contains(curPath, nexth) ||
          Contains(curPath, mesh.topo(nexth).opposite)) {
        continue;
      }

      // next end vertex's degree should >= 4
      VertHandle vh = mesh.topo(nexth).to();
      if (mesh.degree(vh) < 4) {
        continue;
      }

      // next end vertex should not lie on current path (except the startV)
      bool hasSharedVert = false;
      for (HalfHandle h : curPath) {
        VertHandle v = mesh.topo(h).to();
        assert(v != startV);
        if (v == vh) {
          hasSharedVert = true;
          break;
        }
      }
      if (hasSharedVert) {
        continue;
      }

      // next edge should not share any face with current path
      FaceHandle fh = mesh.topo(nexth).face;
      FaceHandle fhoppo = mesh.topo(mesh.topo(nexth).opposite).face;
      bool hasSharedFace = false;
      for (HalfHandle h : curPath) {
        FaceHandle fh2 = mesh.topo(h).face;
        FaceHandle fhoppo2 = mesh.topo(mesh.topo(h).opposite).face;
        if (fh == fh2 || fh == fhoppo2 || fhoppo == fh2 || fhoppo == fhoppo2) {
          hasSharedFace = true;
          break;
        }
      }
      if (hasSharedFace) {
        continue;
      }

      // next edge should not intersect current path
      bool hasIntersection = false;
      for (HalfHandle h : curPath) {
        if (intersectFun(h, nexth)) {
          hasIntersection = true;
          break;
        }
      }
      if (hasIntersection) {
        continue;
      }

      validNextHs.push_back(nexth);
    }

    if (validNextHs.size() == 1) {
      curPath.push_back(validNextHs.front());
      Q.push(std::move(curPath));
    } else {
      for (HalfHandle nexth : validNextHs) {
        auto nextPath = curPath;
        nextPath.push_back(nexth);
        Q.push(std::move(nextPath));
      }
    }
  }

  return std::list<HalfHandle>();
}

// DecomposeOnInternalLoop (liqi)
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfHandleIterT, class FD1, class FD2>
std::pair<typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle,
          typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>
DecomposeOnInternalLoop(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        HalfHandleIterT loopBegin, HalfHandleIterT loopEnd,
                        FD1 &&faceData, FD2 &&oppoFaceData) {

  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  if (loopBegin == loopEnd) {
    return std::pair<Handle<FaceTopo>, Handle<FaceTopo>>();
  }

  std::vector<HalfHandle> loop(loopBegin, loopEnd);
  assert(mesh.topo(loop.front()).from() == mesh.topo(loop.back()).to());

  std::vector<VertHandle> thisVhs;
  std::vector<VertHandle> anotherVhs;
  thisVhs.reserve(loop.size());
  anotherVhs.reserve(loop.size());

  for (int i = 0; i < loop.size(); i++) {
    HalfHandle hh = loop[i];
    HalfHandle nexth = loop[(i + 1) % loop.size()];

    VertHandle vh = mesh.topo(hh).to();
    thisVhs.push_back(vh);
    assert(mesh.degree(vh) >= 4);
    auto d = mesh.data(vh);
    VertHandle vh2 = mesh.addVertex(d);
    anotherVhs.push_back(vh2);

    // find all related half edges that DO NOT lie on the loop side
    std::set<HalfHandle> relatedHsNotOnLoopSide = {mesh.topo(hh).opposite};
    while (true) {
      bool newHFound = false;
      for (HalfHandle relatedH : mesh.topo(vh).halfedges) {
        // whether relatedH DOES NOT lie on the loop side?
        if (Contains(relatedHsNotOnLoopSide, relatedH)) {
          continue;
        }
        if (Contains(loop, relatedH)) {
          continue;
        }
        for (const HalfHandle &hNotOnLoopSide : relatedHsNotOnLoopSide) {
          if (mesh.topo(mesh.topo(relatedH).opposite).face ==
              mesh.topo(hNotOnLoopSide).face) {
            relatedHsNotOnLoopSide.insert(relatedH);
            newHFound = true;
            break;
          }
        }
      }
      if (!newHFound) {
        break;
      }
    }

    // redirect the relatedHsNotOnLoopSide to the newly added vh2
    for (const HalfHandle &hNotOnLoopSide : relatedHsNotOnLoopSide) {
      assert(mesh.topo(hNotOnLoopSide).from() == vh);
      mesh.topo(hNotOnLoopSide).from() = vh2;
      HalfHandle oppoH = mesh.topo(hNotOnLoopSide).opposite;
      assert(mesh.topo(oppoH).to() == vh);
      if (oppoH != hh) { // not on the loop
        mesh.topo(oppoH).to() = vh2;
      }
    }
    mesh.topo(mesh.topo(nexth).opposite).to() = vh2;

    mesh.topo(vh).halfedges.erase(
        std::remove_if(mesh.topo(vh).halfedges.begin(),
                       mesh.topo(vh).halfedges.end(),
                       [&relatedHsNotOnLoopSide](HalfHandle h) {
                         return Contains(relatedHsNotOnLoopSide, h);
                       }),
        mesh.topo(vh).halfedges.end());
    mesh.topo(vh2).halfedges.insert(mesh.topo(vh2).halfedges.begin(),
                                    relatedHsNotOnLoopSide.begin(),
                                    relatedHsNotOnLoopSide.end());
  }

  for (auto &hh : loop) {
    // detach opposites
    HalfHandle oppoh = mesh.topo(hh).opposite;
    mesh.topo(oppoh).opposite.reset();
    mesh.topo(hh).opposite.reset();
  }

  // add the cut faces
  FaceHandle fh1 = mesh.addFace(thisVhs.begin(), thisVhs.end(), true,
                                std::forward<FD1>(faceData));
  FaceHandle fh2 = mesh.addFace(anotherVhs.begin(), anotherVhs.end(), true,
                                std::forward<FD2>(oppoFaceData));
  // copy halfedge data
  assert(thisVhs.size() == anotherVhs.size());
  for (int i = 0; i < anotherVhs.size(); i++) {
    VertHandle anotherVh1 = anotherVhs[i];
    VertHandle anotherVh2 = anotherVhs[(i + 1) % anotherVhs.size()];
    HalfHandle anotherHH;
    for (HalfHandle hh : mesh.topo(anotherVh1).halfedges) {
      if (mesh.topo(hh).to() == anotherVh2) {
        anotherHH = hh;
        break;
      }
    }
    if (mesh.topo(anotherHH).face != fh1 && mesh.topo(anotherHH).face != fh2) {
      anotherHH = mesh.topo(anotherHH).opposite;
    }
    assert(mesh.topo(anotherHH).face == fh1 ||
           mesh.topo(anotherHH).face == fh2);
    VertHandle thisVh1 = thisVhs[i];
    VertHandle thisVh2 = thisVhs[(i + 1) % thisVhs.size()];
    HalfHandle thisHH;
    for (HalfHandle hh : mesh.topo(thisVh1).halfedges) {
      if (mesh.topo(hh).to() == thisVh2) {
        thisHH = hh;
        break;
      }
    }
    if (mesh.topo(thisHH).face != fh1 && mesh.topo(thisHH).face != fh2) {
      thisHH = mesh.topo(thisHH).opposite;
    }
    assert(mesh.topo(thisHH).face == fh1 || mesh.topo(thisHH).face == fh2);
    assert(anotherHH.valid() && thisHH.valid());
    mesh.data(anotherHH) = mesh.data(mesh.topo(thisHH).opposite);
    mesh.data(thisHH) = mesh.data(mesh.topo(anotherHH).opposite);
  }
  return std::make_pair(fh1, fh2);
}

// AssertEdgesAreStiched
template <class VertDataT, class HalfDataT, class FaceDataT>
inline void
AssertEdgesAreStiched(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  for (auto &ht : mesh.halfedges()) {
    assert(ht.topo.opposite.valid() && ht.topo.face.valid() &&
           ht.topo.from().valid() && ht.topo.to().valid() &&
           Contains(mesh.topo(ht.topo.from()).halfedges, ht.topo.hd) &&
           Contains(mesh.topo(ht.topo.face).halfedges, ht.topo.hd) &&
           mesh.topo(ht.topo.opposite).opposite == ht.topo.hd);
  }
}

// DecomposeAll
// IntersectFunT: (HalfHandle, HalfHandle) -> bool
// CompareLoopT: (Container of HalfHandle, Container of HalfHandle) -> bool, if
// first loop is better, return true, otherwise false
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeIntersectFunT, class CompareLoopT>
std::vector<
    std::pair<typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle,
              typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>>
DecomposeAll(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
             HalfEdgeIntersectFunT intersectFun, CompareLoopT compareLoop) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  AssertEdgesAreStiched(mesh);
  std::vector<std::pair<FaceHandle, FaceHandle>> cutFaces;
  while (true) {
    std::list<HalfHandle> minLoop;
    for (int i = 0; i < mesh.internalHalfEdges().size(); i++) {
      HalfHandle hh(i);
      if (mesh.removed(hh)) {
        continue;
      }
      auto loop = ConstructInternalLoopFrom(mesh, hh, intersectFun);
      if (!loop.empty()) {
        if (minLoop.empty() || compareLoop(loop, minLoop)) {
          minLoop = std::move(loop);
        }
      }
    }
    if (!minLoop.empty()) {
      auto newFaces =
          DecomposeOnInternalLoop(mesh, std::begin(minLoop), std::end(minLoop));
      if (newFaces.first.valid() && newFaces.second.valid()) {
        cutFaces.push_back(newFaces);
      }
    } else {
      break;
    }
  }
  return cutFaces;
}

// FindUpperBoundOfDRF
// - HalfEdgeColinearFunT: (HalfEdgeIterT hhsBegin, HalfEdgeIterT hhsEnd) ->
// bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class HalfEdgeColinearFunT, class FaceHandleIterT>
int FindUpperBoundOfDRF(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
                        FaceHandleIterT fhsBegin, FaceHandleIterT fhsEnd,
                        HalfEdgeColinearFunT colinearFun) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  if (fhsBegin == fhsEnd) {
    return 0;
  }

  assert(std::all_of(
      fhsBegin, fhsEnd, [&mesh, fhsBegin, fhsEnd](FaceHandle fh) -> bool {
        return std::all_of(
            mesh.topo(fh).halfedges.begin(), mesh.topo(fh).halfedges.end(),
            [&mesh, fhsBegin, fhsEnd](HalfHandle hh) -> bool {
              return std::any_of(
                  fhsBegin, fhsEnd, [&mesh, hh](FaceHandle fh) -> bool {
                    return fh == mesh.topo(mesh.topo(hh).opposite).face;
                  });
            });
      }));

  int ub = 3;
  std::set<FaceHandle> notInserted(std::next(fhsBegin), fhsEnd);
  while (!notInserted.empty()) {
    FaceHandle curfh;
    std::vector<HalfHandle> curhhs;
    for (auto it = std::next(fhsBegin); it != fhsEnd; ++it) {
      // check each face ...
      FaceHandle fh = *it;
      // ... that is not inserted yet
      if (!Contains(notInserted, fh)) {
        continue;
      }
      // collect its connection halfedges with inserted faces
      std::vector<HalfHandle> hhs;
      for (HalfHandle hh : mesh.topo(fh).halfedges) {
        HalfHandle oppohh = mesh.topo(hh).opposite;
        FaceHandle oppoface = mesh.topo(oppohh).face;
        if (!Contains(notInserted, oppoface)) {
          hhs.push_back(hh);
        }
      }
      // record the face that connect with inserted faces on the most halfedges
      if (!hhs.empty() && hhs.size() > curhhs.size()) {
        curfh = fh;
        curhhs = std::move(hhs);
      }
    }
    assert(curfh.valid() && "the faces are not all connected!");
    if (colinearFun(curhhs.begin(), curhhs.end())) {
      ub++;
    }
    notInserted.erase(curfh);
  }
  return ub;
}

// SortFacesWithPlanarityDependency
// - VertColinearFunT: (VertHandleIterT vhs_begin, VertHandleIterT vhs_end) ->
// bool
template <class VertDataT, class HalfDataT, class FaceDataT,
          class VertColinearFunT, class FaceHandleIterT,
          class FaceDependencyCallbackFunT>
std::unordered_set<typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle>
SortFacesWithPlanarityDependency(
    const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh,
    FaceHandleIterT fhs_begin, FaceHandleIterT fhs_end,
    VertColinearFunT vhs_colinear_fun,
    FaceDependencyCallbackFunT face_dependency_callback_fun) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  std::unordered_set<VertHandle> fundamental_anchors;
  std::unordered_set<VertHandle> vhs_determined;
  std::unordered_set<FaceHandle> fhs_not_determined(fhs_begin, fhs_end);
  size_t nfhs = fhs_not_determined.size();

  std::vector<FaceHandle> ordered_fhs;
  ordered_fhs.reserve(fhs_not_determined.size());

  while (!fhs_not_determined.empty()) {
    FaceHandle next_fh = *fhs_not_determined.begin();
    std::unordered_set<VertHandle> next_vhs;
    for (FaceHandle fh : fhs_not_determined) {
      // locate all the determined vhs
      std::unordered_set<VertHandle> vhs;
      for (HalfHandle hh : mesh.topo(fh).halfedges) {
        VertHandle vh = mesh.topo(hh).from();
        if (Contains(vhs_determined, vh)) {
          vhs.insert(vh);
        }
      }
      // locate only the face that connect with determined faces on the most
      // verts
      if (vhs.size() > next_vhs.size()) {
        next_fh = fh;
        next_vhs = std::move(vhs);
      }
    }

    std::unordered_set<VertHandle> vhs_tobe_anchored;
    if (vhs_colinear_fun(next_vhs.begin(), next_vhs.end())) {
      // we need other vhs to support current face
      // find other vhs
      auto &next_hhs = mesh.topo(next_fh).halfedges;
      if (next_vhs.empty()) {
        // no vhs determined at all
        bool found = false;
        for (int i1 = 0; !found && i1 < next_hhs.size(); i1++) {
          VertHandle vh1 = mesh.topo(next_hhs[i1]).from();
          for (int i2 = i1 + 1; !found && i2 < next_hhs.size(); i2++) {
            VertHandle vh2 = mesh.topo(next_hhs[i2]).from();
            if (vh1 == vh2) {
              continue;
            }
            for (int i3 = i2 + 1; !found && i3 < next_hhs.size(); i3++) {
              VertHandle vh3 = mesh.topo(next_hhs[i3]).from();
              if (vh1 == vh3 || vh2 == vh3) {
                continue;
              }
              VertHandle vhs[] = {vh1, vh2, vh3};
              if (!vhs_colinear_fun(vhs, vhs + 3)) {
                found = true;
                vhs_tobe_anchored.insert({vh1, vh2, vh3});
              }
            }
          }
        }
      } else if (next_vhs.size() == 1) {
        bool found = false;
        VertHandle vh1 = *next_vhs.begin();
        for (int i2 = 0; !found && i2 < next_hhs.size(); i2++) {
          VertHandle vh2 = mesh.topo(next_hhs[i2]).from();
          if (vh1 == vh2) {
            continue;
          }
          for (int i3 = i2 + 1; !found && i3 < next_hhs.size(); i3++) {
            VertHandle vh3 = mesh.topo(next_hhs[i3]).from();
            if (vh1 == vh3 || vh2 == vh3) {
              continue;
            }
            VertHandle vhs[] = {vh1, vh2, vh3};
            if (!vhs_colinear_fun(vhs, vhs + 3)) {
              found = true;
              vhs_tobe_anchored.insert({vh2, vh3});
            }
          }
        }
      } else {
        bool found = false;
        VertHandle vh1 = *next_vhs.begin();
        VertHandle vh2 = *std::next(next_vhs.begin());
        for (int i3 = 0; !found && i3 < next_hhs.size(); i3++) {
          VertHandle vh3 = mesh.topo(next_hhs[i3]).from();
          if (vh1 == vh3 || vh2 == vh3) {
            continue;
          }
          VertHandle vhs[] = {vh1, vh2, vh3};
          if (!vhs_colinear_fun(vhs, vhs + 3)) {
            found = true;
            vhs_tobe_anchored.insert({vh3});
          }
        }
      }

      assert(!vhs_tobe_anchored.empty() && "degenerated faces/edges occurs");
    }

    next_vhs.insert(vhs_tobe_anchored.begin(), vhs_tobe_anchored.end());
    face_dependency_callback_fun(next_fh, next_vhs.begin(), next_vhs.end());

    fundamental_anchors.insert(vhs_tobe_anchored.begin(),
                               vhs_tobe_anchored.end());
    fhs_not_determined.erase(next_fh);
    ordered_fhs.push_back(next_fh);

    // update vhs_determined
    for (HalfHandle hh : mesh.topo(next_fh).halfedges) {
      VertHandle vh = mesh.topo(hh).from();
      vhs_determined.insert(vh);
    }
  }

  assert(nfhs == ordered_fhs.size());
  std::copy(ordered_fhs.begin(), ordered_fhs.end(), fhs_begin);

  return fundamental_anchors;
}

// MakeTetrahedron
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeTetrahedron(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  mesh.clear();
  auto v1 = mesh.addVertex(VertDataT(0, 0, 0));
  auto v2 = mesh.addVertex(VertDataT(0, 0, 1));
  auto v3 = mesh.addVertex(VertDataT(0, 1, 0));
  auto v4 = mesh.addVertex(VertDataT(1, 0, 0));

  mesh.addFace({v1, v2, v3});
  mesh.addFace({v1, v4, v2});
  mesh.addFace({v1, v3, v4});
  mesh.addFace({v2, v4, v3});
}

// MakeQuadFacedCube
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeQuadFacedCube(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  /*
   *       4 ----- 5
          /		  /|
         0 ----- 1 |
         |	     | |
         | 7	 | 6  -- x
         |	     |/
         3 ----- 2
        /
       y
  */
  mesh.clear();
  auto v1 = mesh.addVertex(VertDataT(0, 1, 1));
  auto v2 = mesh.addVertex(VertDataT(1, 1, 1));
  auto v3 = mesh.addVertex(VertDataT(1, 1, 0));
  auto v4 = mesh.addVertex(VertDataT(0, 1, 0));

  auto v5 = mesh.addVertex(VertDataT(0, 0, 1));
  auto v6 = mesh.addVertex(VertDataT(1, 0, 1));
  auto v7 = mesh.addVertex(VertDataT(1, 0, 0));
  auto v8 = mesh.addVertex(VertDataT(0, 0, 0));

  mesh.addFace({v1, v2, v3, v4});
  mesh.addFace({v2, v6, v7, v3});
  mesh.addFace({v6, v5, v8, v7});
  mesh.addFace({v5, v1, v4, v8});
  mesh.addFace({v5, v6, v2, v1});
  mesh.addFace({v4, v3, v7, v8});
}

// MakeTriFacedCube
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeTriFacedCube(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  /*
   *       4 ----- 5
          /		/|
         0 ----- 1 |
         |	   | |
         | 7	   | 6  -- x
         |	   |/
         3 ----- 2
        /
       y
  */
  mesh.clear();
  auto v1 = mesh.addVertex(VertDataT(0, 1, 1));
  auto v2 = mesh.addVertex(VertDataT(1, 1, 1));
  auto v3 = mesh.addVertex(VertDataT(1, 1, 0));
  auto v4 = mesh.addVertex(VertDataT(0, 1, 0));

  auto v5 = mesh.addVertex(VertDataT(0, 0, 1));
  auto v6 = mesh.addVertex(VertDataT(1, 0, 1));
  auto v7 = mesh.addVertex(VertDataT(1, 0, 0));
  auto v8 = mesh.addVertex(VertDataT(0, 0, 0));

  static auto addTriFace =
      [&](typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle a,
          typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle b,
          typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle c,
          typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle d) {
        mesh.addFace({a, b, c});
        mesh.addFace({a, c, d});
      };

  addTriFace(v1, v2, v3, v4);
  addTriFace(v2, v6, v7, v3);
  addTriFace(v6, v5, v8, v7);
  addTriFace(v5, v1, v4, v8);
  addTriFace(v5, v6, v2, v1);
  addTriFace(v4, v3, v7, v8);
}

// MakeIcosahedron
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeIcosahedron(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {

  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  mesh.clear();
  double vertices[12][3]; /* 12 vertices with x, y, z coordinates */
  static const double Pi = 3.141592653589793238462643383279502884197;

  static const double phiaa = 26.56505; /* phi needed for generation */
  static const double r =
      1.0; /* any radius in which the polyhedron is inscribed */
  static const double phia = Pi * phiaa / 180.0; /* 2 sets of four points */
  static const double theb =
      Pi * 36.0 / 180.0; /* offset second set 36 degrees */
  static const double the72 = Pi * 72.0 / 180; /* step 72 degrees */
  vertices[0][0] = 0.0;
  vertices[0][1] = 0.0;
  vertices[0][2] = r;
  vertices[11][0] = 0.0;
  vertices[11][1] = 0.0;
  vertices[11][2] = -r;
  double the = 0.0;
  for (int i = 1; i < 6; i++) {
    vertices[i][0] = r * cos(the) * cos(phia);
    vertices[i][1] = r * sin(the) * cos(phia);
    vertices[i][2] = r * sin(phia);
    the = the + the72;
  }
  the = theb;
  for (int i = 6; i < 11; i++) {
    vertices[i][0] = r * cos(the) * cos(-phia);
    vertices[i][1] = r * sin(the) * cos(-phia);
    vertices[i][2] = r * sin(-phia);
    the = the + the72;
  }

  VertHandle vs[12];
  for (int i = 0; i < 12; i++)
    vs[i] = mesh.addVertex(
        VertDataT(vertices[i][0], vertices[i][1], vertices[i][2]));

  static auto polygon = [&](int a, int b, int c) {
    VertHandle vhs[] = {vs[a], vs[b], vs[c]};
    mesh.addFace(vhs, vhs + 3, true);
  };

  /* map vertices to 20 faces */
  polygon(0, 1, 2);
  polygon(0, 2, 3);
  polygon(0, 3, 4);
  polygon(0, 4, 5);
  polygon(0, 5, 1);
  polygon(1, 2, 6);
  polygon(2, 3, 7);
  polygon(3, 4, 8);
  polygon(4, 5, 9);
  polygon(5, 1, 10);
  polygon(6, 7, 2);
  polygon(7, 8, 3);
  polygon(8, 9, 4);
  polygon(9, 10, 5);
  polygon(10, 6, 1);
  polygon(11, 6, 7);
  polygon(11, 7, 8);
  polygon(11, 8, 9);
  polygon(11, 9, 10);
  polygon(11, 10, 6);
}

// MakePrism
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakePrism(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
               double height) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  mesh.clear();
  double angleStep = M_PI * 2.0 / nsides;
  std::vector<VertHandle> vhs1(nsides), vhs2(nsides);
  for (int i = 0; i < nsides; i++) {
    double angle = angleStep * i;
    double x = cos(angle), y = sin(angle);
    vhs1[i] = mesh.addVertex(VertDataT(x, y, 0));
    vhs2[i] = mesh.addVertex(VertDataT(x, y, height));
  }
  for (int i = 0; i < nsides; i++) {
    mesh.addFace(
        {vhs1[i], vhs1[(i + 1) % nsides], vhs2[(i + 1) % nsides], vhs2[i]});
  }
  mesh.addFace(vhs1.begin(), vhs1.end(), true);
  mesh.addFace(vhs2.begin(), vhs2.end(), true);
}

// MakeCone
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeCone(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
              double height) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  mesh.clear();
  double angleStep = M_PI * 2.0 / nsides;
  VertHandle topVh = mesh.addVertex(VertDataT(0, 0, height));
  std::vector<VertHandle> vhs(nsides);
  for (int i = 0; i < nsides; i++) {
    double angle = angleStep * i;
    double x = cos(angle), y = sin(angle);
    vhs[i] = mesh.addVertex(VertDataT(x, y, 0));
  }
  for (int i = 0; i < nsides; i++) {
    mesh.addFace({vhs[i], vhs[(i + 1) % nsides], topVh});
  }
  mesh.addFace(vhs.begin(), vhs.end(), true);
}

// MakeStarPrism
template <class VertDataT, class HalfDataT, class FaceDataT>
void MakeStarPrism(Mesh<VertDataT, HalfDataT, FaceDataT> &mesh, int nsides,
                   double innerRadius, double outerRadius, double height) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  mesh.clear();
  double angleStep = M_PI / nsides;
  std::vector<VertHandle> vhs1(nsides * 2), vhs2(nsides * 2);
  for (int i = 0; i < nsides * 2; i++) {
    double angle = angleStep * i;
    double r = i % 2 == 0 ? innerRadius : outerRadius;
    double x = cos(angle) * r, y = sin(angle) * r;
    vhs1[i] = mesh.addVertex(VertDataT(x, y, 0));
    vhs2[i] = mesh.addVertex(VertDataT(x, y, height));
  }
  for (int i = 0; i < nsides * 2; i++) {
    mesh.addFace({vhs1[i], vhs1[(i + 1) % (nsides * 2)],
                  vhs2[(i + 1) % (nsides * 2)], vhs2[i]});
  }
  mesh.addFace(vhs1.begin(), vhs1.end(), true);
  mesh.addFace(vhs2.begin(), vhs2.end(), true);
}

// MakeMeshProxy
template <class VertDataT, class HalfDataT, class FaceDataT>
Mesh<typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle,
     typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle,
     typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle>
MakeMeshProxy(const Mesh<VertDataT, HalfDataT, FaceDataT> &mesh) {
  using VertHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::VertHandle;
  using HalfHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::HalfHandle;
  using FaceHandle = typename Mesh<VertDataT, HalfDataT, FaceDataT>::FaceHandle;

  Mesh<VertHandle, HalfHandle, FaceHandle> proxy;
  proxy.internalVertices().reserve(mesh.internalVertices().size());
  proxy.internalHalfEdges().reserve(mesh.internalHalfEdges().size());
  proxy.internalFaces().reserve(mesh.internalFaces().size());
  for (int i = 0; i < mesh.internalVertices().size(); i++) {
    auto &from = mesh.internalVertices()[i];
    proxy.internalVertices().emplace_back(from.topo, from.topo.hd, from.exists);
  }
  for (int i = 0; i < mesh.internalHalfEdges().size(); i++) {
    auto &from = mesh.internalHalfEdges()[i];
    proxy.internalHalfEdges().emplace_back(from.topo, from.topo.hd,
                                           from.exists);
  }
  for (int i = 0; i < mesh.internalFaces().size(); i++) {
    auto &from = mesh.internalFaces()[i];
    proxy.internalFaces().emplace_back(from.topo, from.topo.hd, from.exists);
  }
  return proxy;
}
}
}
