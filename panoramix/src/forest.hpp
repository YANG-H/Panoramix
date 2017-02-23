#pragma once

#include "handle.hpp"

namespace pano {
namespace core {

// ForestTopo
struct ForestTopo {
  Handle<ForestTopo> hd;
  std::set<Handle<ForestTopo>> children;
  Handle<ForestTopo> parent;

  template <class Archive> void serialize(Archive &ar) {
    ar(hd, children, parent);
  }
};

// Forest
template <class T> class Forest {
public:
  Forest() {}
  Forest(const Forest &) = default;
  Forest &operator=(const Forest &) = default;
  Forest(Forest &&f) { _nodes = std::move(f._nodes); }
  Forest &operator=(Forest &&f) {
    _nodes = std::move(f._nodes);
    return *this;
  }

  using NodeHandle = Handle<ForestTopo>;
  using NodeExistsPred = TripletExistsPred<ForestTopo, T>;

  const T &data(NodeHandle h) const { return _nodes[h.id].data; }
  T &data(NodeHandle h) { return _nodes[h.id].data; }
  const ForestTopo &topo(NodeHandle h) const { return _nodes[h.id].topo; }
  NodeHandle parent(NodeHandle h) const { return _nodes[h.id].topo.parent; }

  auto nodes() const { return MakeConditionalRange(_nodes, NodeExistsPred()); }
  auto nodes() { return MakeConditionalRange(_nodes, NodeExistsPred()); }
  const TripletArray<ForestTopo, T> &internalNodes() const { return _nodes; }
  NodeHandle firstRoot() const {
    for (auto &n : _nodes) {
      if (n.topo.parent.invalid())
        return n.topo.hd;
    }
    return NodeHandle();
  }

  NodeHandle add(NodeHandle parent, const T &data) {
    ForestTopo topo;
    topo.hd = NodeHandle(_nodes.size());
    topo.parent = parent;
    _nodes.emplace_back(std::move(topo), data);
    if (parent.valid()) {
      _nodes[parent.id].topo.children.insert(topo.hd);
    }
    return topo.hd;
  }

  NodeHandle add(NodeHandle parent, T &&data) {
    ForestTopo topo;
    topo.hd = NodeHandle(_nodes.size());
    topo.parent = parent;
    _nodes.emplace_back(std::move(topo), std::move(data));
    if (parent.valid()) {
      _nodes[parent.id].topo.children.insert(topo.hd);
    }
    return topo.hd;
  }

  NodeHandle addRoot(const T &data) { return add(NodeHandle(), data); }
  NodeHandle addRoot(T &&data) { return add(NodeHandle(), std::move(data)); }
  bool isRoot(NodeHandle nh) const {
    return _nodes[nh.id].topo.parent.invalid();
  }
  bool isLeaf(NodeHandle nh) const {
    auto &children = _nodes[nh.id].topo.children;
    for (auto &ch : children) {
      if (ch.valid())
        return false;
    }
    return true;
  }

  void remove(NodeHandle h) {
    _nodes[h.id].exists = false;
    for (auto &ch : _nodes[h.id].topo.children) {
      remove(ch);
    }
  }

  void clear() { _nodes.clear(); }

  template <class NodeHandlePtrContainerT = HandlePtrArray<ForestTopo>>
  void gc(const NodeHandlePtrContainerT &nh_ptrs = NodeHandlePtrContainerT()) {
    std::vector<NodeHandle> nnlocs;
    RemoveAndMap(_nodes, nnlocs);
    for (auto &node : _nodes) {
      UpdateOldHandle(nnlocs, node.topo.hd);
      UpdateOldHandle(nnlocs, node.topo.parent);
      UpdateOldHandleContainer(nnlocs, node.topo.children);
      RemoveInValidHandleFromContainer(node.topo.children);
    }
    for (auto &nhPtr : nh_ptrs) {
      UpdateOldHandle(nnlocs, *nhPtr);
    }
  }

  template <class NodeHandleCallbackFunT>
  bool depthFirstSearch(NodeHandle as_root,
                        const NodeHandleCallbackFunT &callback) const {
    assert(_nodes[as_root.id].exists);
    if (!callback(as_root))
      return false;
    for (auto &ch : _nodes[as_root.id].topo.children) {
      if (_nodes[ch.id].exists) {
        if (!depthFirstSearch(ch, callback))
          return false;
      }
    }
    return true;
  }

  template <class NodeHandleCallbackFunT>
  bool breadthFirstSearch(NodeHandle as_root,
                          const NodeHandleCallbackFunT &callback) const {
    assert(_nodes[as_root.id].exists);
    std::queue<NodeHandle> nhs;
    nhs.push(as_root);
    while (!nhs.empty()) {
      NodeHandle nh = nhs.front();
      nhs.pop();
      if (!callback(nh))
        return false;
      for (auto &ch : _nodes[nh.id].topo.children) {
        if (_nodes[ch.id].exists)
          nhs.push(ch);
      }
    }
    return true;
  }

  template <class Archive> void serialize(Archive &ar) { ar(_nodes); }

private:
  TripletArray<ForestTopo, T> _nodes;
};
}
}
