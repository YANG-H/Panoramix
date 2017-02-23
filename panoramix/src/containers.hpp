#pragma once

#include <memory>

#include "basic_types.hpp"
#include "handle.hpp"
#include "iterators.hpp"
#include "meta.hpp"
#include "utility.hpp"

namespace pano {
namespace core {

// ArrayView
template <class T> class ArrayView {
public:
  using ValueType = T;

  explicit ArrayView(T &d)
      : _data(static_cast<const uint8_t *>(&d)), _size(1),
        _offset_bytes(sizeof(T)) {}
  ArrayView(T *d, size_t s)
      : _data(static_cast<const uint8_t *>(d)), _size(s),
        _offset_bytes(sizeof(T)) {}
  ArrayView(T *d, size_t s, ptrdiff_t ofs)
      : _data(static_cast<const uint8_t *>(d)), _size(s), _offset_bytes(ofs) {}

  const T &at(size_t i) const {
    assert(i < _size);
    return static_cast<const T &>(_data[i * _offset_bytes]);
  }
  const T &operator[](size_t i) const {
    assert(i < _size);
    return static_cast<const T &>(_data[i * _offset_bytes]);
  }
  T &operator[](size_t i) {
    assert(i < _size);
    return static_cast<T &>(_data[i * _offset_bytes]);
  }

  const T &front() const { return at(0); }
  T &front() { return (*this)[0]; }
  const T &back() const { return at(_size - 1); }
  T &back() { return (*this)[_size - 1]; }

  size_t size() const { return _size; }

  auto begin() const {
    return MakeTransformIterator(MakeIotaIterator<ptrdiff_t>(0),
                                 _index_functor());
  }
  auto end() const {
    return MakeTransformIterator(MakeIotaIterator<ptrdiff_t>(_size),
                                 _index_functor());
  }
  auto begin() {
    return MakeTransformIterator(MakeIotaIterator<ptrdiff_t>(0),
                                 _index_functor());
  }
  auto end() {
    return MakeTransformIterator(MakeIotaIterator<ptrdiff_t>(_size),
                                 _index_functor());
  }

private:
  constexpr auto _index_functor() const {
    return [this](ptrdiff_t i) { return this->at(i); };
  }
  auto _index_functor() {
    return [this](ptrdiff_t i) { return (*this)[i]; };
  }

private:
  const uint8_t *_data;
  size_t _size;
  ptrdiff_t _offset_bytes;
};

// ElementBinaryRelations
template <class T> class ElementBinaryRelations {
public:
  ElementBinaryRelations() : _nelements(0) {}
  explicit ElementBinaryRelations(size_t n)
      : _nelements(n), _relations(n * (n - 1) / 2) {}
  ElementBinaryRelations(size_t n, const T &v)
      : _nelements(n), _relations(n * (n - 1) / 2, v) {}

public:
  size_t size() const { return _relations.size(); }
  size_t nelements() const { return _nelements; }
  bool empty() const { return _relations.empty(); }
  decltype(auto) operator()(int i, int j) const {
    if (i == j) {
      return T();
    }
    int offset = i < j ? (j * (j - 1) / 2 + i) : (i * (i - 1) / 2 + j);
    return _relations[offset];
  }
  decltype(auto) operator()(int i, int j) {
    if (i == j) {
      throw std::invalid_argument("i and j should not be the same");
    }
    int offset = i < j ? (j * (j - 1) / 2 + i) : (i * (i - 1) / 2 + j);
    return _relations[offset];
  }
  constexpr auto nonZeroNeighbors(int i) const {
    return MakeConditionalRange(MakeIotaRange<int>(_nelements),
                                [this, i](int ind) { return (*this)(i, ind); });
  }

  template <class ArchiveT>
  void serialize(ArchiveT & ar) {
    ar(_relations, _nelements);
  }

private:
  std::vector<T> _relations;
  size_t _nelements;
};


// RTreeSet
template <class T, class BoundingBoxFunctorT = DefaultBoundingBoxFunctor>
class RTreeSet {
public:
  using BoxType =
      decltype(std::declval<BoundingBoxFunctorT>()(std::declval<T>()));
  using ValueType = typename BoxType::Type;
  static const int Dimension = BoxType::Dimension;

  explicit RTreeSet(const BoundingBoxFunctorT &bboxFun = BoundingBoxFunctorT())
      : _rtree(std::make_unique<third_party::RTree<T, ValueType, Dimension>>()),
        _bbox(bboxFun) {}

  template <class IterT>
  RTreeSet(IterT begin, IterT end,
           const BoundingBoxFunctorT &bboxFun = BoundingBoxFunctorT())
      : _rtree(std::make_unique<third_party::RTree<T, ValueType, Dimension>>()),
        _bbox(bboxFun) {
    insert(begin, end);
  }

  RTreeSet(RTreeSet &&r)
      : _rtree(std::move(r._rtree)), _bbox(std::move(r._bbox)) {}
  RTreeSet &operator=(RTreeSet &&r) {
    _rtree = std::move(r._rtree);
    _bbox = std::move(r._bbox);
    return *this;
  }

  RTreeSet(const RTreeSet &) = delete;
  RTreeSet &operator=(const RTreeSet &) = delete;

public:
  size_t size() const { return _rtree->Count(); }
  bool empty() const { return size() == 0; }

  void clear() { return _rtree->RemoveAll(); }

  void insert(const T &t) {
    auto box = bbox(t);
    _rtree->Insert(box.minCorner.val, box.maxCorner.val, t);
  }

  template <class IterT> void insert(IterT begin, IterT end) {
    while (begin != end) {
      insert(*begin);
      ++begin;
    }
  }

  template <class CallbackFunctorT>
  int search(const BoxType &b, CallbackFunctorT &&callback) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val, callback);
  }
  int count(const BoxType &b) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val,
                          StaticConstantFunctor<bool, true>());
  }

private:
  std::unique_ptr<third_party::RTree<T, ValueType, Dimension>> _rtree;
  BoundingBoxFunctorT _bbox;
};

// RTreeMap
template <class T, class ValT,
          class BoundingBoxFunctorT = DefaultBoundingBoxFunctor>
class RTreeMap {
public:
  using BoxType =
      decltype(std::declval<BoundingBoxFunctorT>()(std::declval<T>()));
  using ValueType = typename BoxType::Type;
  static const int Dimension = BoxType::Dimension;

  explicit RTreeMap(const BoundingBoxFunctorT &bboxFun = BoundingBoxFunctorT())
      : _rtree(std::make_unique<
               third_party::RTree<std::pair<T, ValT>, ValueType, Dimension>>()),
        _bbox(bboxFun) {}

  template <class IterT>
  RTreeMap(IterT begin, IterT end,
           const BoundingBoxFunctorT &bboxFun = BoundingBoxFunctorT())
      : _rtree(std::make_unique<
               third_party::RTree<std::pair<T, ValT>, ValueType, Dimension>>()),
        _bbox(bboxFun) {
    insert(begin, end);
  }

  RTreeMap(RTreeMap &&r)
      : _rtree(std::move(r._rtree)), _bbox(std::move(r._bbox)) {}
  RTreeMap &operator=(RTreeMap &&r) {
    _rtree = std::move(r._rtree);
    _bbox = std::move(r._bbox);
    return *this;
  }

  RTreeMap(const RTreeMap &) = delete;
  RTreeMap &operator=(const RTreeMap &) = delete;

public:
  size_t size() const { return _rtree->Count(); }
  bool empty() const { return size() == 0; }

  void clear() { return _rtree->RemoveAll(); }

  void insert(const std::pair<T, ValT> &p) {
    auto box = _bbox(p.first);
    _rtree->Insert(box.minCorner.val, box.maxCorner.val, p);
  }
  void emplace(const T &key, const ValT &val) {
    auto box = _bbox(key);
    _rtree->Insert(box.minCorner.val, box.maxCorner.val,
                   std::make_pair(key, val));
  }

  template <class IterT> void insert(IterT begin, IterT end) {
    while (begin != end) {
      insert(*begin);
      ++begin;
    }
  }

  template <class CallbackFunctorT>
  int search(const BoxType &b, CallbackFunctorT &&callback) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val, callback);
  }
  int count(const BoxType &b) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val,
                          StaticConstantFunctor<bool, true>());
  }

private:
  std::unique_ptr<third_party::RTree<std::pair<T, ValT>, ValueType, Dimension>>
      _rtree;
  BoundingBoxFunctorT _bbox;
};

// simple RTree
template <class BoxT, class T> class RTree {
public:
  using BoxType = BoxT;
  using ValueType = typename BoxType::Type;
  static const int Dimension = BoxType::Dimension;

  explicit RTree()
      : _rtree(
            std::make_unique<third_party::RTree<T, ValueType, Dimension>>()) {}

  template <class IterT>
  RTree(IterT begin, IterT end)
      : _rtree(
            std::make_unique<third_party::RTree<T, ValueType, Dimension>>()) {
    insert(begin, end);
  }

  RTree(RTree &&r) : _rtree(std::move(r._rtree)) {}
  RTree &operator=(RTree &&r) {
    _rtree = std::move(r._rtree);
    return *this;
  }

  RTree(const RTree &) = delete;
  RTree &operator=(const RTree &) = delete;

public:
  size_t size() const { return _rtree->Count(); }
  bool empty() const { return size() == 0; }

  void clear() { return _rtree->RemoveAll(); }

  void insert(const BoxType &box, const T &t) {
    _rtree->Insert(box.minCorner.val, box.maxCorner.val, t);
  }

  void insert(const std::pair<BoxType, T> &p) {
    _rtree->Insert(p.first.minCorner.val, p.first.maxCorner.val, p.second);
  }

  template <class IterT> void insert(IterT begin, IterT end) {
    while (begin != end) {
      insert(*begin);
      ++begin;
    }
  }

  template <class CallbackFunctorT>
  int search(const BoxType &b, CallbackFunctorT &&callback) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val, callback);
  }

  int count(const BoxType &b) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val,
                          StaticConstantFunctor<bool, true>());
  }

private:
  std::unique_ptr<third_party::RTree<T, ValueType, Dimension>> _rtree;
};

// RTree Wrapper
template <class T, class BoundingBoxFunctorT = DefaultBoundingBoxFunctor>
class RTreeWrapper {
public:
  using BoxType =
      decltype(std::declval<BoundingBoxFunctorT>()(std::declval<T>()));
  using ValueType = typename BoxType::Type;
  static const int Dimension = BoxType::Dimension;

  explicit RTreeWrapper(BoundingBoxFunctorT getBB = BoundingBoxFunctorT())
      : _rtree(std::make_unique<third_party::RTree<T, ValueType, Dimension>>()),
        _getBoundingBox(getBB) {}

  template <class IterT>
  RTreeWrapper(IterT begin, IterT end,
               BoundingBoxFunctorT getBB = BoundingBoxFunctorT())
      : _rtree(std::make_unique<third_party::RTree<T, ValueType, Dimension>>()),
        _getBoundingBox(getBB) {
    insert(begin, end);
  }

  RTreeWrapper(RTreeWrapper &&r)
      : _rtree(std::move(r._rtree)),
        _getBoundingBox(std::move(r._getBoundingBox)) {}
  RTreeWrapper &operator=(RTreeWrapper &&r) {
    _rtree = std::move(r._rtree);
    _getBoundingBox = std::move(r._getBoundingBox);
    return *this;
  }

  RTreeWrapper(const RTreeWrapper &) = delete;
  RTreeWrapper &operator=(const RTreeWrapper &) = delete;

public:
  size_t size() const { return _rtree->Count(); }
  bool empty() const { return size() == 0; }

  void clear() { return _rtree->RemoveAll(); }
  const BoundingBoxFunctorT &getBoundingBox() const { return _getBoundingBox; }

  void insert(const BoxType &box, const T &t) {
    _rtree->Insert(box.minCorner.val, box.maxCorner.val, t);
  }

  void insert(const T &t) {
    BoxType box = _getBoundingBox(t);
    for (int i = 0; i < Dimension; i++) {
      if (isnan(box.minCorner[i]) || isnan(box.maxCorner[i])) {
#ifdef _DEBUG
        std::cout << "invalid box type (NaN value), ignore this element"
                  << std::endl;
#endif
        return;
      }
      if (!(box.minCorner[i] <= box.maxCorner[i])) {
#ifdef _DEBUG
        std::cout << "invalid box type (minCorner[i] > maxCorner[i]), ignore "
                     "this element"
                  << std::endl;
#endif
        return;
      }
    }
    _rtree->Insert(box.minCorner.val, box.maxCorner.val, t);
  }

  template <class IterT> void insert(IterT begin, IterT end) {
    while (begin != end) {
      insert(*begin);
      ++begin;
    }
  }

  template <class CallbackFunctorT>
  int search(const BoxType &b, CallbackFunctorT &&callback) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val, callback);
  }

  int count(const BoxType &b) const {
    return _rtree->Search(b.minCorner.val, b.maxCorner.val,
                          StaticConstantFunctor<bool, true>());
  }

  template <class CallbackFunctorT>
  int searchNear(const T &t, CallbackFunctorT &&callback) const {
    auto b = _getBoundingBox(t);
    return _rtree->Search(b.minCorner.val, b.maxCorner.val, callback);
  }

  template <class IsEqualT = std::equal_to<T>>
  bool contains(const T &t, IsEqualT &&cmp = IsEqualT()) const {
    bool exists = false;
    search(_getBoundingBox(t), [&exists, &t, &cmp](const T &ele) {
      if (cmp(ele, t)) {
        exists = true;
        return false;
      }
      return true;
    });
    return exists;
  }

private:
  std::unique_ptr<third_party::RTree<T, ValueType, Dimension>> _rtree;
  BoundingBoxFunctorT _getBoundingBox;
};

// max heap
template <class KeyT, class ScoreT = double,
          class ScoreCompareT = std::less<ScoreT>,
          class KeyToIdT = std::unordered_map<KeyT, int>>
class MaxHeap {
public:
  using iterator = typename std::vector<Scored<KeyT, ScoreT>>::iterator;
  using const_iterator =
      typename std::vector<Scored<KeyT, ScoreT>>::const_iterator;
  using value_type = Scored<KeyT, ScoreT>;

  MaxHeap(const ScoreCompareT &cmp = ScoreCompareT()) : _scoreCompare(cmp) {}

  template <class IterT, class = std::enable_if_t<std::is_same<
                             std::iterator_traits<IterT>::value_type,
                             core::Scored<KeyT, ScoreT>>::value>>
  MaxHeap(IterT begin, IterT end, const ScoreCompareT &cmp = ScoreCompareT())
      : _scoreCompare(cmp) {
    _data.reserve(std::distance(begin, end));
    while (begin != end) {
      _data.push_back(*begin);
      _keyToId[_data.back().component] = _data.size() - 1;
      *begin++;
    }
    makeMaxHeap();
  }

  template <class IterT,
            class = std::enable_if_t<std::is_same<
                std::iterator_traits<IterT>::value_type, KeyT>::value>>
  MaxHeap(IterT vbegin, IterT vend, const ScoreT &defaultScore = ScoreT(),
          const ScoreCompareT &cmp = ScoreCompareT())
      : _scoreCompare(cmp) {
    _data.reserve(std::distance(vbegin, vend));
    while (vbegin != vend) {
      _data.push_back(core::ScoreAs(*vbegin, defaultScore));
      _keyToId[_data.back().component] = _data.size() - 1;
      ++vbegin;
    }
    // makeMaxHeap(); no need to make heap since all scores are same
  }

  template <
      class IterT, class FuncT,
      class = std::enable_if_t<
          std::is_same<std::iterator_traits<IterT>::value_type, KeyT>::value &&
          std::is_same<decltype(std::declval<FuncT>()(*std::declval<IterT>())),
                       ScoreT>::value>>
  MaxHeap(IterT vbegin, IterT vend, FuncT &&fun) {
    _data.reserve(std::distance(vbegin, vend));
    while (vbegin != vend) {
      _data.push_back(core::ScoreAs(*vbegin, fun(*vbegin)));
      _keyToId[_data.back().component] = _data.size() - 1;
      ++vbegin;
    }
    makeMaxHeap(); // need to make heap
  }

  const_iterator begin() const { return _data.begin(); }
  const_iterator end() const { return _data.end(); }
  const_iterator cbegin() const { return _data.cbegin(); }
  const_iterator cend() const { return _data.cend(); }

  const KeyT &top() const { return _data.front().component; }
  const ScoreT &topScore() const { return _data.front().score; }
  const ScoreT &operator[](const KeyT &key) const {
    return _data[_keyToId.at(key)].score;
  }
  const ScoreT &at(const KeyT &key) const {
    return _data[_keyToId.at(key)].score;
  }
  size_t size() const { return _data.size(); }
  bool empty() const { return _data.empty(); }
  size_t height() const { return static_cast<size_t>(log2(_data.size())); }

  void pop() {
    if (_data.empty())
      return;
    swapKeys(0, _data.size() - 1);
    _keyToId.erase(_data.back().component);
    _data.erase(_data.end() - 1, _data.end());
    maxHeapify(0);
  }

  void set(const KeyT &key, const ScoreT &newScore) {
    if (!contains(key)) {
      _data.push_back(ScoreAs(key, newScore));
      _keyToId[key] = _data.size() - 1;
      int id = _data.size() - 1;
      while (id > 0 &&
             _scoreCompare(_data[parentId(id)].score, _data[id].score)) {
        swapKeys(id, parentId(id));
        id = parentId(id);
      }
    } else {
      auto &oldScore = at(key);
      if (oldScore == newScore)
        return;
      else if (_scoreCompare(oldScore, newScore)) { // increase key
        int id = _keyToId[key];
        _data[id].score = newScore;
        while (id > 0 &&
               _scoreCompare(_data[parentId(id)].score, _data[id].score)) {
          swapKeys(id, parentId(id));
          id = parentId(id);
        }
      } else { // decrease key
        int id = _keyToId[key];
        _data[id].score = newScore;
        maxHeapify(id);
      }
    }
  }

  void clear() {
    _data.clear();
    _keyToId.clear();
  }
  void swap(MaxHeap<KeyT, ScoreT> &h) {
    _data.swap(h._data);
    _keyToId.swap(h._keyToId);
  }

  bool contains(const KeyT &k) const {
    return _keyToId.find(k) != _keyToId.end();
  }

private:
  static int parentId(int id) { return (id - 1) / 2; }
  static int leftId(int id) { return id * 2 + 1; }
  static int rightId(int id) { return id * 2 + 2; }

  void swapKeys(int id1, int id2) {
    std::swap(_keyToId[_data[id1].component], _keyToId[_data[id2].component]);
    std::swap(_data[id1], _data[id2]);
  }

  void maxHeapify(int id) {
    auto l = leftId(id);
    auto r = rightId(id);
    int largest = id;
    if (l < _data.size() && _scoreCompare(_data[id].score, _data[l].score)) {
      largest = l;
    }
    if (r < _data.size() &&
        _scoreCompare(_data[largest].score, _data[r].score)) {
      largest = r;
    }
    if (largest != id) {
      swapKeys(id, largest);
      maxHeapify(largest);
    }
  }

  void makeMaxHeap() {
    for (int i = _data.size() / 2 - 1; i >= 0; --i) {
      maxHeapify(i);
    }
  }

private:
  std::vector<Scored<KeyT, ScoreT>> _data;
  KeyToIdT _keyToId;
  ScoreCompareT _scoreCompare;
};

template <class KeyT, class ScoreT, class ScoreCompareT, class KeyToIdT>
bool Contains(const MaxHeap<KeyT, ScoreT, ScoreCompareT, KeyToIdT> &h,
              const KeyT &k) {
  return h.contains(k);
}

// sparse dict
template <class T> class Dictionary {
  struct Node {
    std::vector<std::unique_ptr<Node>> children;
    std::unique_ptr<T> val;
    bool isLeaf() const { return children.empty(); }
    Node() : val(nullptr) {}
    explicit Node(size_t nc) : children(nc), val(nullptr) {}
    explicit Node(std::unique_ptr<T> v) : val(std::move(v)) {}
    Node(Node &&) = default;
    Node(const Node &) = delete;
    Node &operator=(Node &&) = default;
    Node &operator=(const Node &) = delete;
    template <class Archive> void serialize(Archive &ar) { ar(children, val); }
  };

public:
  Dictionary() : _root(nullptr), _size(0) {}
  explicit Dictionary(const std::vector<size_t> &ncs)
      : _nchildren(ncs), _root(nullptr), _size(0) {}
  explicit Dictionary(std::vector<size_t> &&ncs)
      : _nchildren(std::move(ncs)), _root(nullptr), _size(0) {}
  Dictionary(Dictionary &&) = default;
  Dictionary(const Dictionary &) = delete;

  Dictionary &operator=(Dictionary &&) = default;
  Dictionary &operator=(const Dictionary &) = delete;

public:
  template <class IndexT, class TT,
            class = std::enable_if_t<std::is_integral<IndexT>::value>>
  void insert(const IndexT *inds, TT &&val) {
    Node *leaf = create(inds);
    if (!leaf->val) {
      leaf->val = std::make_unique<T>(std::forward<TT>(val));
      _size++;
    } else {
      leaf->val = std::make_unique<T>(std::forward<TT>(val));
    }
  }
  template <class IndexT, class TT>
  void insert(std::initializer_list<IndexT> inds, TT &&val) {
    assert(inds.size() == _nchildren.size());
    return insert(inds.begin(), std::forward<TT>(val));
  }
  template <class IndexT,
            class = std::enable_if_t<std::is_integral<IndexT>::value>>
  bool contains(const IndexT *inds) const {
    return locate(inds) != nullptr;
  }
  template <class IndexT>
  bool contains(std::initializer_list<IndexT> inds) const {
    assert(inds.size() == _nchildren.size());
    return contains(inds.begin());
  }
  template <class IndexT,
            class = std::enable_if_t<std::is_integral<IndexT>::value>>
  const T &at(const IndexT *inds) const {
    auto n = locate(inds);
    assert(n->isLeaf());
    return *(n->val);
  }
  template <class IndexT,
            class = std::enable_if_t<std::is_integral<IndexT>::value>>
  T &at(const IndexT *inds) {
    auto n = locate(inds);
    assert(n->isLeaf());
    return *(n->val);
  }
  template <class IndexT>
  const T &at(std::initializer_list<IndexT> inds) const {
    assert(inds.size() == _nchildren.size());
    return at(inds.begin());
  }
  template <class IndexT> T &at(std::initializer_list<IndexT> inds) {
    assert(inds.size() == _nchildren.size());
    return at(inds.begin());
  }

  size_t size() const { return _size; }

  template <class Archive> void serialize(Archive &ar) {
    ar(_nchildren, _size, _root);
  }

private:
  template <class IndexT = size_t> Node *create(const IndexT *inds) {
    if (_nchildren.empty()) {
      return nullptr;
    }
    if (!_root) {
      _root = std::make_unique<Node>(_nchildren[0]);
    }
    Node *parent = _root.get();
    int lastInd = inds[0];
    for (int i = 1; i < _nchildren.size(); i++) {
      auto ind = inds[i];
      size_t nc = _nchildren[i];
      if (!parent->children[lastInd]) {
        parent->children[lastInd] = std::make_unique<Node>(nc);
      }
      parent = parent->children[lastInd].get();
      lastInd = ind;
    }
    if (!parent->children[lastInd]) {
      parent->children[lastInd] = std::make_unique<Node>(nullptr);
    }
    return parent->children[lastInd].get();
  }

  template <class IndexT = size_t> Node *locate(const IndexT *inds) const {
    Node *curNode = _root.get();
    for (int i = 0; i < _nchildren.size(); i++) {
      auto ind = inds[i];
      size_t nc = _nchildren[i];
      if (!curNode) {
        return nullptr;
      }
      assert(ind < nc);
      curNode = curNode->children[ind].get();
    }
    return curNode;
  }

private:
  std::vector<size_t> _nchildren;
  size_t _size;
  std::unique_ptr<Node> _root;
};

// MergeFindSet
template <class T> class MergeFindSet {
  struct Element {
    int rank;
    int parent;
    T data;
  };

public:
  MergeFindSet() {}
  template <class IterT> MergeFindSet(IterT b, IterT e) { init(b, e); }

  int setsCount() const { return _nsets; }
  const T &data(int id) const { return _elements[id].data; }

  int find(int x) {
    int y = x;
    while (y != _elements[y].parent)
      y = _elements[y].parent;
    _elements[x].parent = y;
    return y;
  }
  template <class MergeFunT = std::plus<T>>
  void join(int x, int y, MergeFunT &&merge = MergeFunT()) {
    if (_elements[x].rank > _elements[y].rank) {
      _elements[y].parent = x;
      _elements[x].data = merge(_elements[x].data, _elements[y].data);
    } else {
      _elements[x].parent = y;
      _elements[y].data = merge(_elements[x].data, _elements[y].data);
      if (_elements[x].rank == _elements[y].rank)
        _elements[y].rank++;
    }
    _nsets--;
  }

private:
  template <class IterT> void init(IterT b, IterT e) {
    _elements.reserve(std::distance(b, e));
    int i = 0;
    _nsets = 0;
    while (b != e) {
      _elements.push_back(Element{0, i, *b});
      ++b;
      ++i;
      ++_nsets;
    }
  }

private:
  int _nsets;
  std::vector<Element> _elements;
};
}
}
