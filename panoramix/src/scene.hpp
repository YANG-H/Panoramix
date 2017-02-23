#pragma once

#include "cameras.hpp"
#include "forest.hpp"
#include "meta.hpp"
#include "utility.hpp"

#include "color.hpp"
#include "shader.hpp"
#include "discretization.hpp"
#include "resource.hpp"

class QWidget;

namespace pano {
namespace gui {

// render mode
using RenderModeFlags = int8_t;
enum RenderModeFlag : RenderModeFlags {
  None = 0,
  Points = 1,
  Lines = 1 << 1,
  Triangles = 1 << 2,
  All = (1 << 3) - 1
};
inline RenderModeFlags operator|(RenderModeFlag f1, RenderModeFlag f2) {
  return static_cast<RenderModeFlags>(f1) | static_cast<RenderModeFlags>(f2);
}

enum InteractionID { ClickLeftButton, PressSpace, Unknown };

// building visual objects
class SceneObject;
using SceneObjectPtr = std::shared_ptr<SceneObject>;
using SceneObjectTree = Forest<SceneObjectPtr>;

using SceneObjectHandle = SceneObjectTree::NodeHandle;
using SceneObjectMeshTriangle =
    std::pair<SceneObjectHandle, TriMesh::TriangleHandle>;
using SceneObjectMeshLine = std::pair<SceneObjectHandle, TriMesh::LineHandle>;
using SceneObjectMeshPoint = std::pair<SceneObjectHandle, TriMesh::PointHandle>;

template <class T>
using SceneObjectCallbackFunction = void(InteractionID, T &data);

struct RenderOptions {
#define DECL_PROPERTY(type, name)                                              \
  \
private:                                                                       \
  type _##name;                                                                \
  \
public:                                                                        \
  inline const type &name() const { return _##name; }                          \
  inline type &name() { return _##name; }                                      \
  inline RenderOptions &name(const type &v) {                                  \
    _##name = v;                                                               \
    return *this;                                                              \
  }
  DECL_PROPERTY(std::string, winName)
  DECL_PROPERTY(Color, backgroundColor)
  DECL_PROPERTY(RenderModeFlags, renderMode)
  DECL_PROPERTY(PerspectiveCamera, camera)
  DECL_PROPERTY(bool, fixUpDirectionInCameraMove)
  DECL_PROPERTY(float, bwColor)
  DECL_PROPERTY(float, bwTexColor)
  DECL_PROPERTY(bool, cullFrontFace)
  DECL_PROPERTY(bool, cullBackFace)
  DECL_PROPERTY(float, panoramaHoriCenterRatio)
  DECL_PROPERTY(float, panoramaAspectRatio)
  DECL_PROPERTY(Point3, panoramaProjectionCenter)
#undef DECL_PROPERTY
  RenderOptions();
};

class SceneObjectInternal;
class SceneObject {
public:
  explicit SceneObject();
  explicit SceneObject(const OpenGLShaderSource &shaderSource);
  virtual ~SceneObject();

  // install shaders
  // before initialize()
  void setShaderSource(const OpenGLShaderSource &shaderSource);

  // initialize rendering
  void initialize() const;

  // render with given camera
  void render(const RenderOptions &options,
              const Mat4f &thisModelMatrix) const;

  // model matrix
  Mat4f &modelMatrix() { return _modelMat; }
  const Mat4f &modelMatrix() const { return _modelMat; }

  // mesh
  TriMesh &mesh() { return _mesh; }
  const TriMesh &mesh() const { return _mesh; }

  void setEntitySelection(EntityPtr ent, bool selected);
  void switchEntitySelection(EntityPtr ent);
  void clearSelection();
  inline const std::set<EntityPtr> &selectedEntities() const {
    return _selectedEntities;
  }
  inline bool entityIsSelected(EntityPtr ent) const {
    return Contains(_selectedEntities, ent);
  }

  // resource
  std::vector<ResourcePtr> &resources() { return _resources; }
  const std::vector<ResourcePtr> &resources() const { return _resources; }

  // data binding
  template <class FunT> inline void bindCallbackFunction(FunT &&fun) {
    _callback = std::forward<FunT>(fun);
  }

  // invoke call back
  inline bool invokeCallbackFunction(InteractionID iid, EntityPtr ent) const {
    if (_callback)
      return _callback(iid, ent);
    return false;
  }
  inline bool
  invokeCallbackFunctionOnTriangle(InteractionID iid,
                                   TriMesh::TriangleHandle t) const {
    if (_callback)
      return _callback(iid, _mesh.entTriangles.at(t));
    return false;
  }
  inline bool invokeCallbackFunctionOnLine(InteractionID iid,
                                           TriMesh::LineHandle t) const {
    if (_callback)
      return _callback(iid, _mesh.entLines.at(t));
    return false;
  }
  inline bool invokeCallbackFunctionOnPoint(InteractionID iid,
                                            TriMesh::PointHandle t) const {
    if (_callback)
      return _callback(iid, _mesh.entPoints.at(t));
    return false;
  }

  inline float lineWidth() const { return _lineWidth; }
  inline float pointSize() const { return _pointSize; }
  inline void setLineWidth(float lw) { _lineWidth = lw; }
  inline void setPointSize(float ps) { _pointSize = ps; }

  inline bool selectable() const { return _selectable; }
  inline void setSelectable(bool s) { _selectable = s; }

protected:
  Mat4f _modelMat;

  TriMesh _mesh;

  std::vector<ResourcePtr> _resources;
  OpenGLShaderSource _shaderSource;

  float _lineWidth;
  float _pointSize;

  bool _selectable;
  SceneObjectInternal *_internal;

  std::function<bool(InteractionID, EntityPtr)> _callback;

  std::set<EntityPtr> _selectedEntities;
};

struct SceneObjectInstallingOptions {
  OpenGLShaderSource defaultShaderSource;
  float lineWidth;
  float pointSize;
  DiscretizeOptions discretizeOptions;
};

template <class T>
std::shared_ptr<SceneObject> Visualize(const T &data,
                                       const SceneObjectInstallingOptions &o,
                                       std::false_type isContainer) {
  std::shared_ptr<SceneObject> vo = std::make_shared<SceneObject>();

  auto dopt = o.discretizeOptions;
  dopt.entity(&data);
  Discretize(vo->mesh(), data, dopt);

  vo->setShaderSource(o.defaultShaderSource);
  vo->setLineWidth(o.lineWidth);
  vo->setPointSize(o.pointSize);
  vo->setSelectable(false);
  return vo;
}

template <class T>
std::shared_ptr<SceneObject> Visualize(const T &data,
                                       const SceneObjectInstallingOptions &o,
                                       std::true_type isContainer) {
  std::shared_ptr<SceneObject> vo = std::make_shared<SceneObject>();

  auto dopt = o.discretizeOptions;
  for (auto &e : data) {
    dopt.entity(&e);
    Discretize(vo->mesh(), e, dopt);
  }

  vo->setShaderSource(o.defaultShaderSource);
  vo->setLineWidth(o.lineWidth);
  vo->setPointSize(o.pointSize);
  vo->setSelectable(false);
  return vo;
}

template <class T, class FunT>
std::shared_ptr<SceneObject> Visualize(T &data, FunT &&fun,
                                       const SceneObjectInstallingOptions &o,
                                       std::false_type isContainer) {

  std::shared_ptr<SceneObject> vo = std::make_shared<SceneObject>();

  auto dopt = o.discretizeOptions;
  dopt.entity(nullptr);
  Discretize(vo->mesh(), data, dopt);

  vo->setShaderSource(o.defaultShaderSource);
  vo->setLineWidth(o.lineWidth);
  vo->setPointSize(o.pointSize);
  vo->setSelectable(true);

  struct Wrapper {
    inline Wrapper(FunT &&f) : originalFun(std::forward<FunT>(f)) {}
    inline bool operator()(InteractionID iid, EntityPtr entPtr) const {
      originalFun(iid, entPtr.ref<T>());
      return true;
    }
    std::decay_t<FunT> originalFun;
  };

  vo->bindCallbackFunction(Wrapper(std::forward<FunT>(fun)));
  return vo;
}

template <class T, class FunT>
std::shared_ptr<SceneObject> Visualize(T &data, FunT &&fun,
                                       const SceneObjectInstallingOptions &o,
                                       std::true_type isContainer) {

  using ValueType = std::decay_t<decltype(*std::begin(data))>;

  std::shared_ptr<SceneObject> vo = std::make_shared<SceneObject>();

  auto dopt = o.discretizeOptions;
  for (auto &e : data) {
    dopt.entity(&e);
    Discretize(vo->mesh(), e, dopt);
  }

  vo->setShaderSource(o.defaultShaderSource);
  vo->setLineWidth(o.lineWidth);
  vo->setPointSize(o.pointSize);
  vo->setSelectable(true);

  struct Wrapper {
    inline Wrapper(FunT &&f) : originalFun(std::forward<FunT>(f)) {}
    inline bool operator()(InteractionID iid, EntityPtr entPtr) const {
      originalFun(iid, entPtr.ref<ValueType>());
      return true;
    }
    std::decay_t<FunT> originalFun;
  };

  vo->bindCallbackFunction(Wrapper(std::forward<FunT>(fun)));
  return vo;
}

using SceneObjectEntity = std::pair<SceneObjectHandle, EntityPtr>;

class SceneBuilder;
class SceneInternal;
class Scene {
public:
  Scene();
  explicit Scene(SceneObjectTree &&tree);
  explicit Scene(const SceneObjectTree &tree);
  Scene(Scene &&s);
  Scene &operator=(Scene &&s);
  Scene(const Scene &s) = delete;
  Scene &operator=(const Scene &s) = delete;
  ~Scene();

public:
  inline bool null() const { return _internal == nullptr; }
  inline const SceneObjectTree &tree() const { return _tree; }

  inline void select(SceneObjectEntity ent) {
    _tree.data(ent.first)->setEntitySelection(ent.second, true);
  }
  inline void switchSelect(SceneObjectEntity ent) {
    _tree.data(ent.first)->switchEntitySelection(ent.second);
  }
  inline void clearSelection() {
    for (auto &n : _tree.nodes())
      n.data->clearSelection();
  }

  const Box3 &boundingBox() const;
  Box3 boundingBoxOfObject(SceneObjectHandle h) const;

  Box3
  boundingBoxOfTriangleInObjectMesh(const SceneObjectMeshTriangle &omt) const;
  Box3
  boundingBoxOfLineInObjectMesh(const SceneObjectMeshLine &oml) const;
  Box3
  boundingBoxOfPointInObjectMesh(const SceneObjectMeshPoint &omp) const;

  void initialize() const;
  void render(const RenderOptions &options) const;

  SceneObjectMeshTriangle
  pickTriangleOnScreen(const RenderOptions &options,
                       const Point2 &pOnScreen) const;
  SceneObjectMeshLine pickLineOnScreen(const RenderOptions &options,
                                       const Point2 &pOnScreen) const;
  SceneObjectMeshPoint pickPointOnScreen(const RenderOptions &options,
                                         const Point2 &pOnScreen) const;

  inline SceneObjectEntity entityOfTriangle(SceneObjectMeshTriangle t) const {
    if (t.first.invalid())
      return SceneObjectEntity();
    return {t.first, _tree.data(t.first)->mesh().entTriangles.at(t.second)};
  }
  inline SceneObjectEntity entityOfLine(SceneObjectMeshLine t) const {
    if (t.first.invalid())
      return SceneObjectEntity();
    return {t.first, _tree.data(t.first)->mesh().entLines.at(t.second)};
  }
  inline SceneObjectEntity entityOfPoint(SceneObjectMeshPoint t) const {
    if (t.first.invalid())
      return SceneObjectEntity();
    return {t.first, _tree.data(t.first)->mesh().entPoints.at(t.second)};
  }

  void pickOnScreen(const RenderOptions &options, const Point2 &pOnScreen,
                    SceneObjectMeshTriangle &t, SceneObjectMeshLine &l,
                    SceneObjectMeshPoint &p) const;
  std::set<SceneObjectEntity> pickOnScreen(const RenderOptions &options,
                                           const Point2 &pOnScreen) const;

  void invokeCallbackFunctions(InteractionID iid,
                               const std::set<SceneObjectEntity> &ents,
                               bool selectedOnly = true) const;
  void invokeCallbackFunctionsOnAllSelected(InteractionID iid) const;

  PerspectiveCamera
  perfectView(int width, int height,
              const Vec3 &up = Vec3(0, 0, 1)) const;

private:
  void update();

private:
  std::unique_ptr<SceneInternal> _internal;
  SceneObjectTree _tree;
};

void PopUpGui(RenderOptions &options, QWidget *widget = nullptr);

class SceneWidget;
class SceneBuilder {
public:
  SceneBuilder();
  SceneBuilder(const SceneObjectInstallingOptions &defaultO);

public:
  const SceneObjectTree &tree() const { return _tree; }
  SceneObjectInstallingOptions &installingOptions() {
    return _installingOptions;
  }
  const SceneObjectInstallingOptions &installingOptions() const {
    return _installingOptions;
  }
  Scene scene() { return Scene(_tree); }

  SceneObjectHandle activeObjectHandle() const { return _activeOH; }
  SceneObject &activeObject() const { return *_tree.data(_activeOH); }

  const TriMesh &activeMesh() const { return _tree.data(_activeOH)->mesh(); }
  TriMesh &activeMesh() { return _tree.data(_activeOH)->mesh(); }

  template <class T> inline SceneBuilder &add(const T &data) {
    _tree.add(_activeOH,
              Visualize(data, _installingOptions, IsContainer<T>()));
    return *this;
  }
  template <class T, class FunT> inline SceneBuilder &add(T &data, FunT &&fun) {
    _tree.add(
        _activeOH,
        Visualize(data, std::forward<FunT>(fun), _installingOptions,
                  std::integral_constant<bool, IsContainer<T>::value>()));
    return *this;
  }

  template <class T> inline SceneBuilder &begin(const T &data) {
    _activeOH = _tree.add(
        _activeOH,
        Visualize(data, _installingOptions,
                  std::integral_constant<bool, IsContainer<T>::value>()));
    return *this;
  }

  template <class T, class FunT>
  inline SceneBuilder &begin(T &data, FunT &&fun) {
    _activeOH = _tree.add(
        _activeOH,
        Visualize(data, std::forward<FunT>(fun), _installingOptions,
                  std::integral_constant<bool, IsContainer<T>::value>()));
    return *this;
  }

  inline SceneBuilder &shaderSource(const OpenGLShaderSource &ss) {
    activeObject().setShaderSource(ss);
    return *this;
  }

  inline SceneBuilder &resource(const std::string resourceName) {
    activeObject().resources().push_back(ResourceStore::get(resourceName));
    return *this;
  }

  inline SceneBuilder &lineWidth(float w) {
    activeObject().setLineWidth(w);
    return *this;
  }

  inline SceneBuilder &pointSize(float s) {
    activeObject().setPointSize(s);
    return *this;
  }

  inline SceneBuilder &rotate(const Vec3 &axis, double angle) {
    auto &mat = activeObject().modelMatrix();
    // rotate
    auto a = normalize(axis);
    double l = a[0], m = a[1], n = a[2];
    double cosv = cos(angle), sinv = sin(angle);
    Mat4f rot(
        l * l * (1 - cosv) + cosv, m * l * (1 - cosv) - n * sinv,
        n * l * (1 - cosv) + m * sinv, 0, l * m * (1 - cosv) + n * sinv,
        m * m * (1 - cosv) + cosv, n * m * (1 - cosv) - l * sinv, 0,
        l * n * (1 - cosv) - m * sinv, m * n * (1 - cosv) + l * sinv,
        n * n * (1 - cosv) + cosv, 0, 0, 0, 0, 1);
    mat = rot * mat;
    return *this;
  }

  inline SceneBuilder &end() {
    if (!_tree.isRoot(_activeOH)) {
      _activeOH = _tree.parent(_activeOH);
    }
    return *this;
  }

  void clear();

  SceneWidget *createWidget(const RenderOptions &options,
                            QWidget *parent = nullptr);
  RenderOptions show(bool doModal = true, bool autoSetCamera = true,
                     const RenderOptions &options = RenderOptions());

private:
  SceneObjectInstallingOptions _installingOptions;
  SceneObjectTree _tree;
  SceneObjectHandle _activeOH;
};
}
}
