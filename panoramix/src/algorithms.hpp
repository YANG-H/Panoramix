#pragma once

#include "basic_types.hpp"
#include "utility.hpp"

namespace pano {
namespace core {
// generic algorithms
template <class IterT, class OutIterT, class IsCompatibleFunT>
void FilterBy(IterT begin, IterT end, OutIterT out,
              IsCompatibleFunT &&is_comp_with_last);

// merge, rearrange the input array
// DistanceFunctorT(a, b) -> DistanceT : compute the distance from a to b
// returns the begin iterators of merged groups
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT = DefaultDistanceFunctor>
IterOutIterT MergeNearNaive(IterT begin, IterT end, IterOutIterT iters_out,
                            std::true_type, DistanceT thres,
                            DistanceFunctorT &&dist_fun = DistanceFunctorT());

// merge, without rearrangement
// DistanceFunctorT(a, b) -> DistanceT : compute the distance from a to b
// returns the iterators pointing to group leaders
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT = DefaultDistanceFunctor>
IterOutIterT MergeNearNaive(IterT begin, IterT end, IterOutIterT iters_out,
                            std::false_type, DistanceT thres,
                            DistanceFunctorT &&dist_fun = DistanceFunctorT());

// merge using RTree, without rearrangement
// DistanceFunctorT(a, b) -> ? : compute the distance from a to b
// BoundingBoxFunctorT(a) -> Box<?,?> : compute the bounding box of a
// returns the iterators pointing to group leaders
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT = DefaultDistanceFunctor,
          class BoundingBoxFunctorT = DefaultBoundingBoxFunctor>
IterOutIterT
MergeNearRTree(IterT begin, IterT end, IterOutIterT iters_out, std::false_type,
               DistanceT thres,
               DistanceFunctorT &&dist_fun = DistanceFunctorT(),
               BoundingBoxFunctorT &&get_bounding_box = BoundingBoxFunctorT());

// Minimum Spanning Tree
// EdgeVertsGetterT(Edge e)->std::pair<Vert,Vert>
// EdgeCompareOnWeightT(Edge e1, Edge e2)->bool
//     determins whether weight of e1 is lower than weight of e2
// VertCompareT(Vert v1, Vert v2)->bool
//     used in std::map to register set id of vertices
template <class VertIterT, class EdgeIterT, class EdgeVertsGetterT,
          class EdgeOutputIterT, class EdgeCompareOnWeightT,
          class VertCompareT =
              std::less<typename std::iterator_traits<VertIterT>::value_type>>
void MinimumSpanningTree(VertIterT verts_begin, VertIterT verts_end,
                         EdgeIterT edges_begin, EdgeIterT edges_end,
                         EdgeOutputIterT mst_edges,
                         EdgeVertsGetterT &&get_verts,
                         EdgeCompareOnWeightT &&compare_edge_on_weight,
                         VertCompareT &&compare_vert = VertCompareT());

// DepthFirstSearch
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertCallbackT,
          class VertCompareT =
              std::less<typename std::iterator_traits<VertIterT>::value_type>>
void DepthFirstSearch(VertIterT verts_begin, VertIterT verts_end,
                      NeighborVertsContainerGetterT &&get_neighbor_verts,
                      VertCallbackT &&vert_callback,
                      VertCompareT &&compare_vert = VertCompareT());

// BreadthFirstSearch
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertCallbackT,
          class VertCompareT =
              std::less<typename std::iterator_traits<VertIterT>::value_type>>
void BreadthFirstSearch(VertIterT verts_begin, VertIterT verts_end,
                        NeighborVertsContainerGetterT get_neighbor_verts,
                        VertCallbackT vert_callback,
                        VertCompareT compare_vert = VertCompareT());

// Topological Sort (using Depth First Search)
template <class VertIterT, class VertOutIterT,
          class PredecessorVertsContainerGetterT,
          class VertCompareT =
              std::less<typename std::iterator_traits<VertIterT>::value_type>>
void TopologicalSort(VertIterT verts_begin, VertIterT verts_end,
                     VertOutIterT sorted_verts_begin,
                     PredecessorVertsContainerGetterT get_predecessor_verts,
                     VertCompareT compare_vert = VertCompareT());

// Connected Components
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertexTypeRecorderT,
          class VertCompareT =
              std::less<typename std::iterator_traits<VertIterT>::value_type>>
int ConnectedComponents(VertIterT verts_begin, VertIterT verts_end,
                        NeighborVertsContainerGetterT get_neighbor_verts,
                        VertexTypeRecorderT record_vert_type,
                        VertCompareT compare_vert = VertCompareT());
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace core {

// generic algorithms
template <class IterT, class OutIterT, class IsCompatibleFunT>
void FilterBy(IterT begin, IterT end, OutIterT out,
              IsCompatibleFunT &&is_comp_with_last) {
  if (begin == end)
    return;
  *out = *begin;
  ++out;
  IterT lastIter = begin;
  ++begin;
  while (begin != end) {
    if (is_comp_with_last(*lastIter, *begin)) {
      *out = *begin;
      ++out;
      lastIter = begin;
    }
    ++begin;
  }
}

// merge, rearrange the input array
// DistanceFunctorT(a, b) -> DistanceT : compute the distance from a to b
// returns the begin iterators of merged groups
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT>
IterOutIterT MergeNearNaive(IterT begin, IterT end, IterOutIterT iters_out,
                            std::true_type, DistanceT thres,
                            DistanceFunctorT &&dist_fun) {
  if (begin == end)
    return iters_out;

  std::vector<IterT> gBegins(1, begin);
  for (auto i = std::next(begin); i != end; ++i) {
    DistanceT minDist = std::numeric_limits<DistanceT>::max();
    auto nearestGBeginIter = gBegins.end();
    for (auto giter = gBegins.begin(); giter != gBegins.end(); ++giter) {
      auto gBegin = *giter;
      DistanceT dist = dist_fun(*gBegin, *i);
      if (dist <= thres && dist < minDist) {
        minDist = dist;
        nearestGBeginIter = giter;
      }
    }
    if (nearestGBeginIter != gBegins.end()) { // found group
      if (std::next(nearestGBeginIter) != gBegins.end()) {
        auto nextGBegin = *std::next(nearestGBeginIter);
        std::rotate(nextGBegin, i, std::next(i));
        for (auto j = std::next(nearestGBeginIter); j != gBegins.end(); ++j)
          ++(*j);
      }
    } else { // add new group
      gBegins.push_back(i);
    }
  }
  return std::copy(gBegins.begin(), gBegins.end(), iters_out);
}

// merge, without rearrangement
// DistanceFunctorT(a, b) -> DistanceT : compute the distance from a to b
// returns the iterators pointing to group leaders
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT>
IterOutIterT MergeNearNaive(IterT begin, IterT end, IterOutIterT iters_out,
                            std::false_type, DistanceT thres,
                            DistanceFunctorT &&dist_fun) {
  if (begin == end)
    return iters_out;

  *(iters_out++) = begin;
  std::vector<IterT> gBegins(1, begin);
  for (auto i = std::next(begin); i != end; ++i) {
    auto giter = gBegins.begin();
    for (; giter != gBegins.end(); ++giter) {
      auto gBegin = *giter;
      auto dist = dist_fun(*gBegin, *i);
      if (dist <= thres) {
        break;
      }
    }
    if (giter == gBegins.end()) { // add new group
      gBegins.push_back(i);
      *(iters_out++) = i;
    }
  }

  return iters_out;
}

// merge using RTree, without rearrangement
// DistanceFunctorT(a, b) -> ? : compute the distance from a to b
// BoundingBoxFunctorT(a) -> Box<?,?> : compute the bounding box of a
// returns the iterators pointing to group leaders
template <class IterT, class IterOutIterT, class DistanceT,
          class DistanceFunctorT, class BoundingBoxFunctorT>
IterOutIterT MergeNearRTree(IterT begin, IterT end, IterOutIterT iters_out,
                            std::false_type, DistanceT thres,
                            DistanceFunctorT &&dist_fun,
                            BoundingBoxFunctorT &&get_bounding_box) {

  if (begin == end)
    return iters_out;

  using BoxType = decltype(get_bounding_box(*begin));
  using T = typename BoxType::Type;
  static const int N = BoxType::Dimension;

  third_party::RTree<IterT, T, N> rtree;
  for (auto i = begin; i != end; ++i) {
    Box<T, N> box = get_bounding_box(*i);
    for (int k = 0; k < N; k++) { // extend the box
      box.minCorner[k] -= thres * 2;
      box.maxCorner[k] += thres * 2;
    }
    // search in RTree
    int foundCount = 0;
    rtree.Search(box.minCorner.val, box.maxCorner.val,
                 [dist_fun, i, thres, &foundCount](IterT it) {
                   if (dist_fun(*i, *it) <= thres) {
                     foundCount++;
                     return false;
                   }
                   return true;
                 });
    if (foundCount == 0) {
      rtree.Insert(box.minCorner.val, box.maxCorner.val, i);
      *(iters_out)++ = i;
    }
  }

  return iters_out;
}

// Minimum Spanning Tree
// EdgeVertsGetterT(Edge e)->std::pair<Vert,Vert>
// EdgeCompareOnWeightT(Edge e1, Edge e2)->bool
//     determins whether weight of e1 is lower than weight of e2
// VertCompareT(Vert v1, Vert v2)->bool
//     used in std::map to register set id of vertices
template <class VertIterT, class EdgeIterT, class EdgeVertsGetterT,
          class EdgeOutputIterT, class EdgeCompareOnWeightT, class VertCompareT>
void MinimumSpanningTree(VertIterT verts_begin, VertIterT verts_end,
                         EdgeIterT edges_begin, EdgeIterT edges_end,
                         EdgeOutputIterT mst_edges,
                         EdgeVertsGetterT &&get_verts,
                         EdgeCompareOnWeightT &&compare_edge_on_weight,
                         VertCompareT &&compare_vert) {

  using Edge = typename std::iterator_traits<typename EdgeIterT>::value_type;
  using Vert = typename std::iterator_traits<typename VertIterT>::value_type;
  static_assert(
      std::is_same<std::decay_t<decltype(std::get<0>(get_verts(*edges_begin)))>,
                   Vert>::value &&
          std::is_same<
              std::decay_t<decltype(std::get<1>(get_verts(*edges_begin)))>,
              Vert>::value,
      "result of EdgeVertsGetterT must be convertiable to std::tuple<Vert, "
      "Vert>!");

  std::vector<Edge> edges(edges_begin, edges_end);
  std::sort(edges.begin(), edges.end(), compare_edge_on_weight);

  std::map<Vert, int, VertCompareT> vertSetIds(compare_vert);
  int idx = 0;
  for (auto i = verts_begin; i != verts_end; ++i)
    vertSetIds.insert(std::make_pair((*i), idx++));

  auto remainedEdgesBegin = edges.begin();
  while (remainedEdgesBegin != edges.end()) {
    Edge e = *remainedEdgesBegin;
    auto verts = get_verts(e);
    int fromid = vertSetIds[std::get<0>(verts)];
    int toid = vertSetIds[std::get<1>(verts)];
    if (fromid != toid) {
      *mst_edges++ = e;
      for (auto &vtoid : vertSetIds) {
        if (vtoid.second == toid) {
          vtoid.second = fromid;
        }
      }
    }
    ++remainedEdgesBegin;
  }
}

// DepthFirstSearch
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertCallbackT, class VertCompareT>
void DepthFirstSearch(VertIterT verts_begin, VertIterT verts_end,
                      NeighborVertsContainerGetterT &&get_neighbor_verts,
                      VertCallbackT &&vert_callback,
                      VertCompareT &&compare_vert) {

  using Vert = typename std::iterator_traits<typename VertIterT>::value_type;
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::begin(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::end(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");

  struct {
    bool operator()(Vert root, std::map<Vert, bool, VertCompareT> &vVisited,
                    NeighborVertsContainerGetterT vNeighborsGetter,
                    VertCallbackT vCallback) const {
      if (vVisited[root])
        return true;
      if (!vCallback(root))
        return false;

      vVisited[root] = true;
      auto vNeighborsContainer = vNeighborsGetter(root);
      for (const auto &v : vNeighborsContainer) {
        if (!(*this)(v, vVisited, vNeighborsGetter, vCallback))
          return false;
      }
      return true;
    }
  } depthFirstSearchOneTree;

  std::map<Vert, bool, VertCompareT> visited(compare_vert);
  for (auto i = verts_begin; i != verts_end; ++i)
    visited[*i] = false;
  while (true) {
    auto rootIter = verts_begin;
    while (rootIter != verts_end && visited[*rootIter]) {
      ++rootIter;
    }
    if (rootIter == verts_end)
      break;
    if (!depthFirstSearchOneTree(*rootIter, visited, get_neighbor_verts,
                                 vert_callback))
      break;
  }
}

// BreadthFirstSearch
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertCallbackT, class VertCompareT>
void BreadthFirstSearch(VertIterT verts_begin, VertIterT verts_end,
                        NeighborVertsContainerGetterT get_neighbor_verts,
                        VertCallbackT vert_callback,
                        VertCompareT compare_vert) {

  using Vert = typename std::iterator_traits<typename VertIterT>::value_type;
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::begin(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::end(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");

  struct {
    bool operator()(const Vert &root,
                    std::map<Vert, bool, VertCompareT> &vVisited,
                    const NeighborVertsContainerGetterT &vNeighborsGetter,
                    VertCallbackT vCallback) {
      std::queue<Vert> Q;
      Q.push(root);
      vVisited[root] = true;
      while (!Q.empty()) {
        Vert v = Q.front();
        Q.pop();
        if (!vCallback(v))
          return false;
        auto vNeighborsContainer = vNeighborsGetter(v);
        for (const auto &vv : vNeighborsContainer) {
          if (vVisited.at(vv))
            continue;
          Q.push(vv);
          vVisited[vv] = true;
        }
      }
      return true;
    }
  } breadthFirstSearchOneTree;

  std::map<Vert, bool, VertCompareT> visited(compare_vert);
  for (auto i = verts_begin; i != verts_end; ++i)
    visited[*i] = false;
  while (true) {
    auto rootIter = verts_begin;
    while (rootIter != verts_end && visited[*rootIter]) {
      ++rootIter;
    }
    if (rootIter == verts_end)
      break;
    if (!breadthFirstSearchOneTree(*rootIter, visited, get_neighbor_verts,
                                   vert_callback))
      break;
  }
}

// Topological Sort (using Depth First Search)
template <class VertIterT, class VertOutIterT,
          class PredecessorVertsContainerGetterT, class VertCompareT>
void TopologicalSort(VertIterT verts_begin, VertIterT verts_end,
                     VertOutIterT sorted_verts_begin,
                     PredecessorVertsContainerGetterT get_predecessor_verts,
                     VertCompareT compare_vert) {

  using Vert = typename std::iterator_traits<typename VertIterT>::value_type;
  static_assert(
      std::is_same<Vert,
                   std::decay_t<decltype(*std::begin(
                       get_predecessor_verts(std::declval<Vert>())))>>::value,
      "PredecessorVertsContainerGetterT should returns a container of Vert");
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::end(get_predecessor_verts(
                             std::declval<Vert>())))>>::value,
      "PredecessorVertsContainerGetterT should returns a container of Vert");

  struct {
    void operator()(Vert root, std::map<Vert, bool, VertCompareT> &vVisited,
                    VertOutIterT sortedVertsOut,
                    PredecessorVertsContainerGetterT get_predecessor_verts) {
      if (vVisited[root])
        return;

      vVisited[root] = true;
      for (const auto &v : get_predecessor_verts(root)) {
        (*this)(v, vVisited, sortedVertsOut, get_predecessor_verts);
      }

      *sortedVertsOut = root;
      ++sortedVertsOut;
    }
  } depthFirstSearchOneTree;

  std::map<Vert, bool, VertCompareT> visited(compare_vert);
  for (auto i = verts_begin; i != verts_end; ++i)
    visited[*i] = false;
  while (true) {
    auto rootIter = verts_begin;
    while (rootIter != verts_end && visited[*rootIter]) {
      ++rootIter;
    }
    if (rootIter == verts_end)
      break;
    depthFirstSearchOneTree(*rootIter, visited, sorted_verts_begin,
                            get_predecessor_verts);
  }
}

// Connected Components
template <class VertIterT, class NeighborVertsContainerGetterT,
          class VertexTypeRecorderT, class VertCompareT>
int ConnectedComponents(VertIterT verts_begin, VertIterT verts_end,
                        NeighborVertsContainerGetterT get_neighbor_verts,
                        VertexTypeRecorderT record_vert_type,
                        VertCompareT compare_vert) {

  using Vert = typename std::iterator_traits<typename VertIterT>::value_type;
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::begin(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");
  static_assert(
      std::is_same<Vert, std::decay_t<decltype(*std::end(get_neighbor_verts(
                             std::declval<Vert>())))>>::value,
      "NeighborVertsContainerGetterT should returns a container of Vert");

  struct {
    void operator()(const Vert &root,
                    std::map<Vert, bool, VertCompareT> &vVisited,
                    const NeighborVertsContainerGetterT &vNeighborsGetter,
                    const VertexTypeRecorderT &vTypeRecorder, int cid) {
      std::queue<Vert> Q;
      Q.push(root);
      vVisited[root] = true;
      while (!Q.empty()) {
        Vert v = Q.front();
        Q.pop();
        vTypeRecorder(v, cid);
        for (const auto &vv : vNeighborsGetter(v)) {
          if (vVisited.at(vv))
            continue;
          Q.push(vv);
          vVisited[vv] = true;
        }
      }
    }
  } breadthFirstSearchOneTree;

  std::map<Vert, bool, VertCompareT> visited(compare_vert);
  for (auto i = verts_begin; i != verts_end; ++i)
    visited[*i] = false;

  int cid = 0;
  while (true) {
    auto rootIter = verts_begin;
    while (rootIter != verts_end && visited[*rootIter]) {
      ++rootIter;
    }
    if (rootIter == verts_end)
      break;
    breadthFirstSearchOneTree(*rootIter, visited, get_neighbor_verts,
                              record_vert_type, cid);
    cid++;
  }

  return cid;
}
}
}
