#pragma once

#include "algorithms.hpp"
#include "containers.hpp"
#include "handle.hpp"

namespace pano {
namespace core {
// the mesh class
// the half edge mesh representing a manifold
template <class VertDataT, class HalfDataT = Dummy, class FaceDataT = Dummy>
class Mesh {
public:
  using VertData = VertDataT;
  using HalfData = HalfDataT;
  using FaceData = FaceDataT;

  struct VertTopo;
  struct HalfTopo;
  struct FaceTopo;
  using VertHandle = Handle<VertTopo>;
  using HalfHandle = Handle<HalfTopo>;
  using FaceHandle = Handle<FaceTopo>;

  struct VertTopo {
    Handle<VertTopo> hd;
    std::vector<HalfHandle> halfedges;
    template <class Archive> void serialize(Archive &ar) { ar(hd, halfedges); }
  };
  struct HalfTopo {
    Handle<HalfTopo> hd;
    Handle<VertTopo> endVertices[2];
    Handle<VertTopo> &from() { return endVertices[0]; }
    Handle<VertTopo> &to() { return endVertices[1]; }
    const Handle<VertTopo> &from() const { return endVertices[0]; }
    const Handle<VertTopo> &to() const { return endVertices[1]; }
    Handle<HalfTopo> opposite;
    Handle<FaceTopo> face;
    template <class Archive> void serialize(Archive &ar) {
      ar(hd, endVertices, opposite, face);
    }
  };
  struct FaceTopo {
    Handle<FaceTopo> hd;
    std::vector<HalfHandle> halfedges;
    template <class Archive> void serialize(Archive &ar) { ar(hd, halfedges); }
  };

  decltype(auto) internalVertices() { return _verts; }
  decltype(auto) internalHalfEdges() { return _halfs; }
  decltype(auto) internalFaces() { return _faces; }
  decltype(auto) internalVertices() const { return _verts; }
  decltype(auto) internalHalfEdges() const { return _halfs; }
  decltype(auto) internalFaces() const { return _faces; }

  auto vertices() {
    return MakeConditionalRange(_verts,
                                TripletExistsPred<VertTopo, VertDataT>());
  }
  auto halfedges() {
    return MakeConditionalRange(_halfs,
                                TripletExistsPred<HalfTopo, HalfDataT>());
  }
  auto faces() {
    return MakeConditionalRange(_faces,
                                TripletExistsPred<FaceTopo, FaceDataT>());
  }
  auto vertices() const {
    return MakeConditionalRange(_verts,
                                TripletExistsPred<VertTopo, VertDataT>());
  }
  auto halfedges() const {
    return MakeConditionalRange(_halfs,
                                TripletExistsPred<HalfTopo, HalfDataT>());
  }
  auto faces() const {
    return MakeConditionalRange(_faces,
                                TripletExistsPred<FaceTopo, FaceDataT>());
  }

  VertTopo &topo(VertHandle v) { return _verts[v.id].topo; }
  HalfTopo &topo(HalfHandle h) { return _halfs[h.id].topo; }
  FaceTopo &topo(FaceHandle f) { return _faces[f.id].topo; }
  const VertTopo &topo(VertHandle v) const { return _verts[v.id].topo; }
  const HalfTopo &topo(HalfHandle h) const { return _halfs[h.id].topo; }
  const FaceTopo &topo(FaceHandle f) const { return _faces[f.id].topo; }

  VertDataT &data(VertHandle v) { return _verts[v.id].data; }
  HalfDataT &data(HalfHandle h) { return _halfs[h.id].data; }
  FaceDataT &data(FaceHandle f) { return _faces[f.id].data; }
  const VertDataT &data(VertHandle v) const { return _verts[v.id].data; }
  const HalfDataT &data(HalfHandle h) const { return _halfs[h.id].data; }
  const FaceDataT &data(FaceHandle f) const { return _faces[f.id].data; }

  template <class VDT = VertDataT> VertHandle addVertex(VDT &&vd = VDT()) {
    VertTopo topo;
    topo.hd.id = _verts.size();
    _verts.emplace_back(std::move(topo), std::forward<VDT>(vd), true);
    return _verts.back().topo.hd;
  }

  template <class HDT = HalfDataT, class HDRevT = HDT>
  HalfHandle addEdge(VertHandle from, VertHandle to, HDT &&hd = HDT(),
                     HDRevT &&hdrev = HDRevT(),
                     bool mergeDuplicateEdge = true) {
    if (from == to) {
      return HalfHandle();
    }

    HalfHandle hh1;
    HalfHandle hh2;
    if (mergeDuplicateEdge) {
      hh1 = findEdge(from, to);
      hh2 = findEdge(to, from);
    }

    if (hh1.invalid()) {
      hh1 = HalfHandle(_halfs.size());
      Triplet<HalfTopo, HalfDataT> ht;
      ht.topo.hd.id = _halfs.size();
      ht.topo.from() = from;
      ht.topo.to() = to;
      ht.exists = true;
      ht.data = std::forward<HDT>(hd);
      _halfs.push_back(ht);
      _verts[from.id].topo.halfedges.push_back(hh1);
    }
    if (hh2.invalid()) {
      hh2 = HalfHandle(_halfs.size());
      Triplet<HalfTopo, HalfDataT> ht;
      ht.topo.hd.id = _halfs.size();
      ht.topo.from() = to;
      ht.topo.to() = from;
      ht.exists = true;
      ht.data = std::forward<HDRevT>(hdrev);
      _halfs.push_back(ht);
      _verts[to.id].topo.halfedges.push_back(hh2);
    }

    _halfs[hh1.id].topo.opposite = hh2;
    _halfs[hh2.id].topo.opposite = hh1;
    return hh1;
  }

  template <class FDT = FaceDataT>
  FaceHandle addFace(const std::vector<HalfHandle> &halfedges,
                     FDT &&fd = FDT()) {
    Triplet<FaceTopo, FaceDataT> ft;
    ft.topo.hd.id = _faces.size();
    ft.topo.halfedges = halfedges;
    ft.exists = true;
    ft.data = std::forward<FDT>(fd);
    //_faces.push_back({ { { _faces.size() }, halfedges }, true, fd });
    _faces.push_back(ft);
    for (HalfHandle hh : halfedges) {
      _halfs[hh.id].topo.face = _faces.back().topo.hd;
    }
    return _faces.back().topo.hd;
  }

  template <class FDT = FaceDataT>
  FaceHandle addFace(const std::vector<VertHandle> &vertices,
                     bool autoflip = true, FDT &&fd = FDT()) {
    std::vector<HalfHandle> halfs;
    assert(vertices.size() >= 3);
    auto verts = vertices;

    if (autoflip) {
      for (size_t i = 0; i < verts.size(); i++) {
        size_t inext = (i + 1) % verts.size();
        HalfHandle hh = findEdge(verts[i], verts[inext]);
        if (hh.valid() && _halfs[hh.id].topo.face.valid()) {
          std::reverse(verts.begin(), verts.end());
          break;
        }
      }
    }

    for (size_t i = 0; i < verts.size(); i++) {
      size_t inext = (i + 1) % verts.size();
      halfs.push_back(addEdge(verts[i], verts[inext]));
    }
    return addFace(halfs, std::forward<FDT>(fd));
  }
  template <class VertHandleIterT, class FDT = FaceDataT,
            class = std::enable_if_t<
                std::is_same<std::iterator_traits<VertHandleIterT>::value_type,
                             VertHandle>::value>>
  FaceHandle addFace(VertHandleIterT vhBegin, VertHandleIterT vhEnd,
                     bool autoflip = true, FDT &&fd = FDT()) {
    std::vector<HalfHandle> halfs;
    std::vector<VertHandle> verts(vhBegin, vhEnd);
    assert(verts.size() >= 3);

    if (autoflip) {
      for (size_t i = 0; i < verts.size(); i++) {
        size_t inext = (i + 1) % verts.size();
        HalfHandle hh = findEdge(verts[i], verts[inext]);
        if (hh.valid() && _halfs[hh.id].topo.face.valid()) {
          std::reverse(verts.begin(), verts.end());
          break;
        }
      }
    }
    for (size_t i = 0; i < verts.size(); i++) {
      size_t inext = (i + 1) % verts.size();
      halfs.push_back(addEdge(verts[i], verts[inext]));
    }
    return addFace(halfs, std::forward<FDT>(fd));
  }

  template <class FDT = FaceDataT>
  FaceHandle addFace(VertHandle v1, VertHandle v2, VertHandle v3,
                     bool autoflip = true, FDT &&fd = FDT()) {
    HalfHandle hh = findEdge(v3, v1);
    if (hh.valid() && _halfs[hh.id].topo.face.valid() && autoflip) {
      std::swap(v1, v3);
    }
    return addFace({addEdge(v1, v2), addEdge(v2, v3), addEdge(v3, v1)},
                   std::forward<FDT>(fd));
  }

  template <class FDT = FaceDataT>
  FaceHandle addFace(VertHandle v1, VertHandle v2, VertHandle v3, VertHandle v4,
                     bool autoflip = true, FDT &&fd = FDT()) {
    HalfHandle hh = findEdge(v4, v1);
    if (hh.valid() && _halfs[hh.id].topo.face.valid() && autoflip) {
      std::swap(v1, v4);
    }
    return addFace(
        {addEdge(v1, v2), addEdge(v2, v3), addEdge(v3, v4), addEdge(v4, v1)},
        std::forward<FDT>(fd));
  }

  HalfHandle findEdge(VertHandle from, VertHandle to) const {
    for (HalfHandle hh : _verts[from.id].topo.halfedges) {
      assert(_halfs[hh.id].topo.endVertices[0] == from);
      if (_halfs[hh.id].topo.endVertices[1] == to) {
        return hh;
      }
    }
    return HalfHandle();
  }

  size_t degree(VertHandle v) const {
    return _verts[v.id].topo.halfedges.size();
  }
  size_t degree(FaceHandle f) const {
    return _faces[f.id].topo.halfedges.size();
  }

  HalfHandle firstHalf(HalfHandle hh) const {
    return topo(hh).opposite < hh ? topo(hh).opposite : hh;
  }

  bool removed(FaceHandle f) const { return !_faces[f.id].exists; }
  bool removed(HalfHandle e) const { return !_halfs[e.id].exists; }
  bool removed(VertHandle v) const { return !_verts[v.id].exists; }

  void remove(FaceHandle f) {
    if (f.invalid() || removed(f))
      return;
    _faces[f.id].exists = false;
    for (auto &hh : _faces[f.id].topo.halfedges) {
      hh.reset();
    }
  }
  void remove(HalfHandle e) {
    if (h.invalid() || removed(h))
      return;
    HalfHandle hop = _halfs[h.id].topo.opposite;
    _halfs[h.id].exists = false;
    _halfs[hop.id].exists = false;

    remove(_halfs[h.id].topo.face);
    remove(_halfs[hop.id].topo.face);

    _halfs[h.id].topo.from().reset();
    _halfs[hop.id].topo.to().reset();
    _halfs[h.id].topo.face.reset();
    _halfs[hop.id].topo.face.reset();
  }
  void remove(VertHandle v) {
    if (v.invalid() || removed(v))
      return;
    _verts[v.id].exists = false;
    for (HalfHandle hh : _verts[v.id].topo.halfedges)
      remove(hh);
    _verts[v.id].topo.halfedges.clear();
  }

  Mesh &unite(const Mesh &m) {
    std::vector<VertHandle> vtable(m.Vertices().size());
    std::vector<HalfHandle> htable(m.HalfEdges().size());
    std::vector<FaceHandle> ftable(m.Faces().size());

    for (auto v : m.vertices()) {
      vtable[v.topo.hd.id] = addVertex(v.data);
    }
    for (auto h : m.halfedges()) {
      VertHandle oldfrom = h.topo.from();
      VertHandle oldto = h.topo.to();
      VertHandle newfrom = vtable[oldfrom.id];
      VertHandle newto = vtable[oldto.id];
      htable[h.topo.hd.id] =
          addEdge(newfrom, newto, h.data, m.data(h.topo.opposite));
    }
    for (auto f : m.faces()) {
      std::vector<HalfHandle> hs;
      hs.reserve(f.topo.halfedges.size());
      for (auto hh : f.topo.halfedges) {
        hs.push_back(htable[hh.id]);
      }
      ftable[f.topo.hd.id] = addFace(hs, f.data);
    }

    return *this;
  }

  // garbage collection
  template <class VertHandlePtrContainerT = std::vector<VertHandle *>,
            class HalfHandlePtrContainerT = std::vector<HalfHandle *>,
            class FaceHandlePtrContainerT = std::vector<FaceHandle *>>
  void gc(const VertHandlePtrContainerT &vps = VertHandlePtrContainerT(),
          const HalfHandlePtrContainerT &hps = HalfHandlePtrContainerT(),
          const FaceHandlePtrContainerT &fps = FaceHandlePtrContainerT()) {
    std::vector<VertHandle> vnlocs;
    std::vector<HalfHandle> hnlocs;
    std::vector<FaceHandle> fnlocs;
    RemoveAndMap(_verts, vnlocs);
    RemoveAndMap(_halfs, hnlocs);
    RemoveAndMap(_faces, fnlocs);

    for (size_t i = 0; i < _verts.size(); i++) {
      UpdateOldHandle(vnlocs, _verts[i].topo.hd);
      UpdateOldHandleContainer(hnlocs, _verts[i].topo.halfedges);
      RemoveInValidHandleFromContainer(_verts[i].topo.halfedges);
    }
    for (size_t i = 0; i < _halfs.size(); i++) {
      UpdateOldHandle(hnlocs, _halfs[i].topo.hd);
      UpdateOldHandleContainer(vnlocs, _halfs[i].topo.endVertices);
      UpdateOldHandle(hnlocs, _halfs[i].topo.opposite);
      UpdateOldHandle(fnlocs, _halfs[i].topo.face);
    }
    for (size_t i = 0; i < _faces.size(); i++) {
      UpdateOldHandle(fnlocs, _faces[i].topo.hd);
      UpdateOldHandleContainer(hnlocs, _faces[i].topo.halfedges);
      RemoveInValidHandleFromContainer(_faces[i].topo.halfedges);
    }
    for (auto vp : vps) {
      UpdateOldHandle(vnlocs, *vp);
    }
    for (auto hp : hps) {
      UpdateOldHandle(hnlocs, *hp);
    }
    for (auto fp : fps) {
      UpdateOldHandle(fnlocs, *fp);
    }
  }

  void clear() {
    _verts.clear();
    _halfs.clear();
    _faces.clear();
  }

  template <class T>
  HandledTable<VertHandle, T> createVertexTable(const T &v) const {
    return HandledTable<VertHandle, T>(_verts.size(), v);
  }
  template <class T>
  HandledTable<HalfHandle, T> createHalfEdgeTable(const T &v) const {
    return HandledTable<HalfHandle, T>(_halfs.size(), v);
  }
  template <class T>
  HandledTable<FaceHandle, T> createFaceTable(const T &v) const {
    return HandledTable<FaceHandle, T>(_faces.size(), v);
  }

  template <class Archive> void serialize(Archive &ar) {
    ar(_verts, _halfs, _faces);
  }

private:
  TripletArray<VertTopo, VertDataT> _verts;
  TripletArray<HalfTopo, HalfDataT> _halfs;
  TripletArray<FaceTopo, FaceDataT> _faces;
};

using Mesh2 = Mesh<Point2>;
using Mesh3 = Mesh<Point3>;
}
}