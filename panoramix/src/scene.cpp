#include "pch.hpp"

#include "containers.hpp"
#include "macros.hpp"
#include "qttools.hpp"
#include "scene.hpp"
#include "ui.hpp"

namespace pano {
namespace gui {

using namespace core;

namespace {

template <class T> struct OpenGLDataTraits {};
template <> struct OpenGLDataTraits<int8_t> {
  enum { GLType = GL_BYTE };
};
template <> struct OpenGLDataTraits<uint8_t> {
  enum { GLType = GL_UNSIGNED_BYTE };
};
template <> struct OpenGLDataTraits<int16_t> {
  enum { GLType = GL_SHORT };
};
template <> struct OpenGLDataTraits<uint16_t> {
  enum { GLType = GL_UNSIGNED_SHORT };
};
template <> struct OpenGLDataTraits<int32_t> {
  enum { GLType = GL_INT };
};
template <> struct OpenGLDataTraits<uint32_t> {
  enum { GLType = GL_UNSIGNED_INT };
};
template <> struct OpenGLDataTraits<float> {
  enum { GLType = GL_FLOAT };
};
template <> struct OpenGLDataTraits<double> {
  enum { GLType = GL_DOUBLE };
};

template <class StructT, class T,
          class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline void SetAttributeArray(QOpenGLShaderProgram *program,
                              const char *attributeName, const T &firstVector) {
  program->setAttributeArray(attributeName, OpenGLDataTraits<T>::GLType,
                             &firstVector, 1, sizeof(StructT));
}

template <class StructT, class T, int N,
          class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline void SetAttributeArray(QOpenGLShaderProgram *program,
                              const char *attributeName,
                              const Vec<T, N> &firstVector) {
  program->setAttributeArray(attributeName, OpenGLDataTraits<T>::GLType,
                             &firstVector, N, sizeof(StructT));
}

template <class T>
inline void DrawElements(GLenum mode, const std::vector<T> &indices) {
  glDrawElements(mode, indices.size(), OpenGLDataTraits<T>::GLType,
                 indices.data());
}
}

RenderOptions::RenderOptions()
    : _winName("Scene"), _backgroundColor(ColorTag::White),
      _renderMode(RenderModeFlag::All),
      _camera(PerspectiveCamera(500, 500, Point2(250, 250), 250,
                                {1.0, 1.0, 1.0}, {0.0, 0.0, 0.0},
                                {0.0, 0.0, 1.0})),
      _fixUpDirectionInCameraMove(true), _bwColor(.3), _bwTexColor(0.7),
      _cullFrontFace(true), _cullBackFace(false),
      _panoramaProjectionCenter(0, 0, 0), _panoramaHoriCenterRatio(0.5f),
      _panoramaAspectRatio(0.5f) {}

class SceneObjectInternal {
public:
  QOpenGLShaderProgram *program;
  std::vector<uint32_t> meshVerticesSelected;
  inline SceneObjectInternal() : program(nullptr) {}
  inline void initialize() {
    if (!program)
      program = new QOpenGLShaderProgram;
  }
  inline bool isLinked() const { return program ? program->isLinked() : false; }
  inline QString log() const {
    return program ? program->log() : "QOpenGLShaderProgram not initialized!";
  }
  inline ~SceneObjectInternal() { delete program; }
};

SceneObject::SceneObject()
    : _shaderSource(OpenGLShaderSourceDescriptor::XLines),
      _modelMat(Mat4::eye()) {
  _internal = new SceneObjectInternal;
  _lineWidth = 2.0;
  _pointSize = 5.0;
  _selectable = false;
}

SceneObject::SceneObject(const OpenGLShaderSource &shaderSource)
    : _shaderSource(shaderSource), _modelMat(Mat4::eye()) {
  _internal = new SceneObjectInternal;
  _lineWidth = 2.0;
  _pointSize = 5.0;
  _selectable = false;
}

SceneObject::~SceneObject() {
  for (auto &res : _resources) {
    if (!res->isInitialized())
      continue;
    res->destroy();
    if (res->isInitialized())
      qDebug() << (_internal->log());
  }

  delete _internal;
}

void SceneObject::setShaderSource(const OpenGLShaderSource &shaderSource) {
  if (_internal->isLinked()) {
    qDebug() << "program is already linked! setting shaders failed!";
    return;
  }
  _shaderSource = shaderSource;
}

void SceneObject::initialize() const {
  if (_internal && _internal->program && _internal->program->isLinked()) {
    return;
  }
  _internal->meshVerticesSelected.resize(_mesh.vertices.size(), 0);

  auto vo = _internal;
  vo->initialize();
  auto program = vo->program;

  // setup shaders
  if (!program->addShaderFromSourceCode(
          QOpenGLShader::Vertex,
          QByteArray::fromRawData(_shaderSource.vertexShaderSource().data(),
                                  _shaderSource.vertexShaderSource().size())))
    qDebug() << (program->log());
  if (!program->addShaderFromSourceCode(
          QOpenGLShader::Fragment,
          QByteArray::fromRawData(_shaderSource.fragmentShaderSource().data(),
                                  _shaderSource.fragmentShaderSource().size())))
    qDebug() << (program->log());
  if (!program->link() || !program->bind()) {
    qDebug() << (program->log());
    return;
  }
  Q_ASSERT(program->isLinked());

  // initialize resources
  for (auto &res : _resources) {
    if (res->isInitialized())
      continue;
    res->initialize();
    if (!res->isInitialized())
      qDebug() << (program->log());
  }

  program->release();
}

void SceneObject::render(const RenderOptions &options,
                         const Mat4f &thisModelMatrix) const {
  if (_mesh.vertices.empty())
    return;

  auto vo = _internal;
  vo->initialize();
  auto program = vo->program;

  Q_ASSERT(program->isLinked());
  program->bind();

  // bind resources
  for (auto &res : _resources) {
    if (!res->bind())
      qDebug() << (program->log());
  }

  glLineWidth(_lineWidth);
  glPointSize(_pointSize);

  // assert(thisModelMatrix == Mat4f::eye());

  program->setUniformValue("panoramaCenter",
                           MakeQVec(options.panoramaProjectionCenter()));
  program->setUniformValue("panoramaHoriCenterRatio",
                           options.panoramaHoriCenterRatio());
  program->setUniformValue("panoramaAspectRatio",
                           options.panoramaAspectRatio());

  program->setUniformValue("projectionMatrix",
                           MakeQMatrix(options.camera().projectionMatrix()));
  program->setUniformValue("modelMatrix", MakeQMatrix(thisModelMatrix));
  program->setUniformValue("viewMatrix",
                           MakeQMatrix(options.camera().viewMatrix()));
  program->setUniformValue("tex", 0);
  program->setUniformValue("bwColor", options.bwColor());
  program->setUniformValue("bwTexColor", options.bwTexColor());

  SetAttributeArray<TriMesh::Vertex>(program, "position",
                                     _mesh.vertices.front().position);
  SetAttributeArray<TriMesh::Vertex>(program, "normal",
                                     _mesh.vertices.front().normal);
  SetAttributeArray<TriMesh::Vertex>(program, "color",
                                     _mesh.vertices.front().color);
  SetAttributeArray<TriMesh::Vertex>(program, "texCoord",
                                     _mesh.vertices.front().texCoord);

  using IsSelectedType =
      std::decay_t<decltype(_internal->meshVerticesSelected.front())>;
  SetAttributeArray<IsSelectedType>(program, "isSelected",
                                    _internal->meshVerticesSelected.front());
  assert(_mesh.vertices.size() == _internal->meshVerticesSelected.size());

  program->enableAttributeArray("position");
  program->enableAttributeArray("normal");
  program->enableAttributeArray("color");
  program->enableAttributeArray("texCoord");
  program->enableAttributeArray("isSelected");

  if (options.renderMode() & RenderModeFlag::Triangles) {
    DrawElements(GL_TRIANGLES, _mesh.iTriangles);
  }
  if (options.renderMode() & RenderModeFlag::Lines) {
    DrawElements(GL_LINES, _mesh.iLines);
  }
  if (options.renderMode() & RenderModeFlag::Points) {
    DrawElements(GL_POINTS, _mesh.iPoints);
  }

  program->disableAttributeArray("position");
  program->disableAttributeArray("normal");
  program->disableAttributeArray("color");
  program->disableAttributeArray("texCoord");
  program->disableAttributeArray("isSelected");

  program->release();
}

void SceneObject::setEntitySelection(EntityPtr ent, bool selected) {
  if (selected)
    _selectedEntities.insert(ent);
  else
    _selectedEntities.erase(ent);
  for (int i = 0; i < _mesh.numerOfPoints(); i++) {
    if (_mesh.entPoints[i] == ent) {
      _internal->meshVerticesSelected[i] = selected;
    }
  }
  for (int i = 0; i < _mesh.numberOfLines(); i++) {
    TriMesh::VertHandle v1, v2;
    _mesh.fetchLineVerts(i, v1, v2);
    if (_mesh.entLines[i] == ent) {
      _internal->meshVerticesSelected[v1] = selected;
      _internal->meshVerticesSelected[v2] = selected;
    }
  }
  for (int i = 0; i < _mesh.numberOfTriangles(); i++) {
    TriMesh::VertHandle v1, v2, v3;
    _mesh.fetchTriangleVerts(i, v1, v2, v3);
    if (_mesh.entTriangles[i] == ent) {
      _internal->meshVerticesSelected[v1] = selected;
      _internal->meshVerticesSelected[v2] = selected;
      _internal->meshVerticesSelected[v3] = selected;
    }
  }
}

void SceneObject::switchEntitySelection(EntityPtr ent) {
  if (Contains(_selectedEntities, ent)) {
    setEntitySelection(ent, false);
  } else {
    setEntitySelection(ent, true);
  }
}

void SceneObject::clearSelection() {
  _selectedEntities.clear();
  for (auto &s : _internal->meshVerticesSelected) {
    s = false;
  }
}

class SceneInternal {
public:
  template <class EntityT> class SceneObjectMeshEntityIndexer {
  public:
    inline SceneObjectMeshEntityIndexer() {}
    inline SceneObjectMeshEntityIndexer(std::map<EntityT, Box3> &&bbox)
        : calculatedBoundingBox(std::move(bbox)) {
      rtree = std::make_unique<third_party::RTree<EntityT, double, 3>>();
      // insert bboxes
      for (auto &b : calculatedBoundingBox) {
        if (HasValue(b.second, IsInfOrNaN<double>)) {
          continue;
        }
        rtree->Insert(b.second.minCorner.val, b.second.maxCorner.val, b.first);
      }
      assert(rtree->Count() <= calculatedBoundingBox.size());
    }
    inline SceneObjectMeshEntityIndexer(SceneObjectMeshEntityIndexer &&idx)
        : calculatedBoundingBox(std::move(idx.calculatedBoundingBox)),
          rtree(std::move(idx.rtree)) {}
    inline SceneObjectMeshEntityIndexer &
    operator=(SceneObjectMeshEntityIndexer &&idx) {
      calculatedBoundingBox = std::move(idx.calculatedBoundingBox);
      rtree = std::move(idx.rtree);
      return *this;
    }
    SceneObjectMeshEntityIndexer(const SceneObjectMeshEntityIndexer &) = delete;
    SceneObjectMeshEntityIndexer &
    operator=(const SceneObjectMeshEntityIndexer &) = delete;

    inline Box3 operator()(const EntityT &mti) const {
      return calculatedBoundingBox.at(mti);
    }
    template <class CallbackFunctorT>
    inline int search(const Box3 &b, CallbackFunctorT &&callback) const {
      return rtree->Search(b.minCorner.val, b.maxCorner.val,
                           std::forward<CallbackFunctorT>(callback));
    }
    inline bool empty() const { return !rtree || rtree->Count() == 0; }

  private:
    std::map<EntityT, Box3> calculatedBoundingBox;
    std::unique_ptr<third_party::RTree<EntityT, double, 3>> rtree;
  };

  std::map<SceneObjectHandle, Mat4f> calculatedModelMatrices;
  std::map<SceneObjectHandle, Box3> calculatedBoundingBoxes;
  std::map<SceneObjectHandle, std::vector<Point3>> transformedVerticesPositions;

  SceneObjectMeshEntityIndexer<SceneObjectMeshTriangle> triangles;
  SceneObjectMeshEntityIndexer<SceneObjectMeshLine> lines;
  SceneObjectMeshEntityIndexer<SceneObjectMeshPoint> points;

  Box3 boundingBox;
};

Scene::Scene() {}

Scene::Scene(SceneObjectTree &&tree) {
  _tree = std::move(tree);
  update();
}

Scene::Scene(const SceneObjectTree &tree) {
  _tree = tree;
  update();
}

Scene::Scene(Scene &&s) {
  _internal = std::move(s._internal);
  _tree = std::move(s._tree);
}

Scene &Scene::operator=(Scene &&s) {
  _internal = std::move(s._internal);
  _tree = std::move(s._tree);
  return *this;
}

Scene::~Scene() {}

void Scene::update() {
  _internal = std::make_unique<SceneInternal>();
  std::unordered_set<SceneObjectHandle> visited;

  std::map<SceneObjectMeshTriangle, Box3> triangleBoxes;
  std::map<SceneObjectMeshLine, Box3> lineBoxes;
  std::map<SceneObjectMeshPoint, Box3> pointBoxes;

  while (true) {
    SceneObjectHandle notVisited;
    for (auto &n : _tree.nodes()) {
      if (!Contains(visited, n.topo.hd)) {
        notVisited = n.topo.hd;
        break;
      }
    }
    if (notVisited.invalid())
      break;

    _tree.depthFirstSearch(notVisited, [this, &visited, &triangleBoxes,
                                        &lineBoxes, &pointBoxes](
                                           SceneObjectHandle h) -> bool {
      visited.insert(h);
      SceneObject *ro = _tree.data(h).get();
      // update model matrix
      if (_tree.isRoot(h)) { // is root
        assert(!Contains(_internal->calculatedModelMatrices, h));
        _internal->calculatedModelMatrices[h] = ro->modelMatrix();
      } else {
        assert(Contains(_internal->calculatedModelMatrices, _tree.parent(h)));
        _internal->calculatedModelMatrices[h] =
            _internal->calculatedModelMatrices.at(_tree.parent(h)) *
            ro->modelMatrix();
      }
      // update bounding box in world space
      auto &mesh = ro->mesh();
      std::vector<Point3> transformedVertexPositions;
      transformedVertexPositions.reserve(mesh.vertices.size());
      for (auto &vert : mesh.vertices) {
        Point4 transformedHCorner =
            _internal->calculatedModelMatrices.at(h) * vert.position;
        transformedVertexPositions.push_back(Point3(transformedHCorner[0],
                                                    transformedHCorner[1],
                                                    transformedHCorner[2]) /
                                             transformedHCorner[3]);
      }
      for (TriMesh::TriangleHandle i = 0; i < mesh.numberOfTriangles(); i++) {
        TriMesh::VertHandle v1, v2, v3;
        mesh.fetchTriangleVerts(i, v1, v2, v3);
        triangleBoxes[std::make_pair(h, i)] = BoundingBoxOfContainer(
            std::vector<Point3>{transformedVertexPositions.at(v1),
                                transformedVertexPositions.at(v2),
                                transformedVertexPositions.at(v3)});
      }

      /*  THERE_ARE_BUGS_HERE("We can't determine how much the bounding box of
         lines/point "
            "should be with a fixed line width/point size and a dynamic
         camera!!!!");*/

      for (TriMesh::LineHandle i = 0; i < mesh.numberOfLines(); i++) {
        TriMesh::VertHandle v1, v2;
        mesh.fetchLineVerts(i, v1, v2);
        lineBoxes[std::make_pair(h, i)] =
            BoundingBoxOfContainer(
                std::vector<Point3>{transformedVertexPositions[v1],
                                    transformedVertexPositions[v2]})
                .expand(1.0);
      }
      for (TriMesh::PointHandle i = 0; i < mesh.numerOfPoints(); i++) {
        TriMesh::VertHandle v;
        mesh.fetchPointVerts(i, v);
        pointBoxes[std::make_pair(h, i)] =
            BoundingBoxOfContainer(
                std::vector<Point3>{transformedVertexPositions[v]})
                .expand(1.0);
      }

      _internal->calculatedBoundingBoxes[h] =
          BoundingBoxOfContainer(transformedVertexPositions);
      _internal->transformedVerticesPositions[h] =
          std::move(transformedVertexPositions);
      return true;
    });
  }

  _internal->triangles = std::move(triangleBoxes);
  _internal->lines = std::move(lineBoxes);
  _internal->points = std::move(pointBoxes);

  _internal->boundingBox =
      BoundingBoxOfPairRange(_internal->calculatedBoundingBoxes.begin(),
                             _internal->calculatedBoundingBoxes.end());
}

const Box3 &Scene::boundingBox() const { return _internal->boundingBox; }

Box3 Scene::boundingBoxOfObject(SceneObjectHandle h) const {
  return _internal->calculatedBoundingBoxes.at(h);
}

Box3 Scene::boundingBoxOfTriangleInObjectMesh(
    const SceneObjectMeshTriangle &omt) const {
  return _internal->triangles(omt);
}

Box3 Scene::boundingBoxOfLineInObjectMesh(
    const SceneObjectMeshLine &oml) const {
  return _internal->lines(oml);
}

Box3 Scene::boundingBoxOfPointInObjectMesh(
    const SceneObjectMeshPoint &omp) const {
  return _internal->points(omp);
}

void Scene::initialize() const {
  for (auto &n : _tree.nodes()) {
    if (n.exists) {
      _tree.data(n.topo.hd)->initialize();
    }
  }
}

void Scene::render(const RenderOptions &options) const {
  glFrontFace(GL_CCW); // face direction set to clockwise
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_STENCIL_TEST);

  glEnable(GL_ALPHA_TEST);

  if (options.cullFrontFace() || options.cullBackFace()) {
    glEnable(GL_CULL_FACE);
    if (options.cullFrontFace() && !options.cullBackFace())
      glCullFace(GL_FRONT);
    else if (!options.cullFrontFace() && options.cullBackFace())
      glCullFace(GL_BACK);
    else
      glCullFace(GL_FRONT_AND_BACK);
  } else {
    glDisable(GL_CULL_FACE);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_POINT_SPRITE);

  for (auto &n : _tree.nodes()) {
    if (n.exists) {
      _tree.data(n.topo.hd)->render(
          options, _internal->calculatedModelMatrices.at(n.topo.hd));
    }
  }

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
}

inline Point3 ToAffinePoint(const Vec4f &p) {
  return Point3(p[0] / p[3], p[1] / p[3], p[2] / p[3]);
}

template <class TT>
bool TriangleIntersection(const Vec<TT, 3> &V1, // Triangle vertices
                          const Vec<TT, 3> &V2, const Vec<TT, 3> &V3,
                          const Vec<TT, 3> &O, // Ray origin
                          const Vec<TT, 3> &D, // Ray direction

                          TT *out, TT epsilon) {

  Vec<TT, 3> e1, e2; // Edge1, Edge2
  Vec<TT, 3> P, Q, T;
  TT det, inv_det, u, v;
  TT t;

  // Find vectors for two edges sharing V1
  e1 = V2 - V1;
  e2 = V3 - V1;
  // Begin calculating determinant - also used to calculate u parameter
  P = D.cross(e2);
  // if determinant is near zero, ray lies in plane of triangle
  det = e1.dot(P);
  // NOT CULLING
  if (det > -epsilon && det < epsilon)
    return false;
  inv_det = 1.f / det;

  // calculate distance from V1 to ray origin
  T = O - V1;

  // Calculate u parameter and test bound
  u = T.dot(P) * inv_det;
  // The intersection lies outside of the triangle
  if (u < 0.f || u > 1.f)
    return false;

  // Prepare to test v parameter
  Q = T.cross(e1);

  // Calculate V parameter and test bound
  v = D.dot(Q) * inv_det;
  // The intersection lies outside of the triangle
  if (v < 0.f || u + v > 1.f)
    return false;

  t = e2.dot(Q) * inv_det;

  if (t > epsilon) { // ray intersection
    *out = t;
    return true;
  }

  // No hit, no win
  return false;
}

SceneObjectMeshTriangle
Scene::pickTriangleOnScreen(const RenderOptions &options,
                            const Point2 &pOnScreen) const {

  if (null() || _internal->triangles.empty()) {
    return SceneObjectMeshTriangle();
  }

  Ray3 centerRay(options.camera().eye(),
                 normalize(options.camera().toSpace(pOnScreen)));

  Box3 bboxAll = boundingBox();
  auto bballAll = bboxAll.outerSphere();

  // discretize the ray
  double startLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) - bballAll.radius);
  double stopLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) + bballAll.radius);
  double stepNum = 1000.0;
  double stepLen = (stopLen - startLen) / stepNum;

  double epsilon = 1e-20;

  std::map<SceneObjectMeshTriangle, double> triangles;

  for (int i = 0; i <= stepNum; i++) {
    double centerLen = startLen + stepLen * i;
    double nextCenterLen = centerLen + stepLen;

    Point3 centerP = centerRay.anchor + centerLen * centerRay.direction;
    Point3 nextCenterP = centerRay.anchor + nextCenterLen * centerRay.direction;
    Line3 lineSegment(centerP, nextCenterP);

    Box3 detectionBox = BoundingBox(centerP) | BoundingBox(nextCenterP);
    detectionBox.expand(stepLen);

    _internal->triangles.search(
        detectionBox, [&triangles, this, &centerRay,
                       epsilon](const SceneObjectMeshTriangle &m) {
          auto &transformedVertPositions =
              _internal->transformedVerticesPositions.at(m.first);
          auto &vo = _tree.data(m.first);
          if (!vo->selectable()) {
            return true;
          }
          TriMesh::VertHandle v1, v2, v3;
          vo->mesh().fetchTriangleVerts(m.second, v1, v2, v3);
          double out = 0.0;
          bool intersected = TriangleIntersection(
              transformedVertPositions[v1], transformedVertPositions[v2],
              transformedVertPositions[v3], centerRay.anchor,
              centerRay.direction, &out, epsilon);
          if (intersected) {
            triangles[m] = out;
          }
          return true;
        });
  }

  double minDist = std::numeric_limits<double>::max();

  SceneObjectMeshTriangle nearest;

  for (auto &r : triangles) {
    double d = r.second;
    if (d < minDist) {
      minDist = d;
      nearest = r.first;
    }
  }

  return nearest;
}

SceneObjectMeshLine Scene::pickLineOnScreen(const RenderOptions &options,
                                            const Point2 &pOnScreen) const {

  if (null() || _internal->lines.empty()) {
    return SceneObjectMeshLine();
  }

  Ray3 centerRay(options.camera().eye(),
                 normalize(options.camera().toSpace(pOnScreen)));

  Box3 bboxAll = boundingBox();
  auto bballAll = bboxAll.outerSphere();

  // discretize the ray
  double startLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) - bballAll.radius);
  double stopLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) + bballAll.radius);
  double stepNum = 1000.0;
  double stepLen = (stopLen - startLen) / stepNum;

  std::map<SceneObjectMeshLine, double> lines;

  for (int i = 0; i <= stepNum; i++) {
    double centerLen = startLen + stepLen * i;
    double nextCenterLen = centerLen + stepLen;

    Point3 centerP = centerRay.anchor + centerLen * centerRay.direction;
    Point3 nextCenterP = centerRay.anchor + nextCenterLen * centerRay.direction;
    Line3 lineSegment(centerP, nextCenterP);

    Box3 detectionBox = BoundingBox(centerP) | BoundingBox(nextCenterP);
    detectionBox.expand(stepLen);

    _internal->lines.search(detectionBox, [&lines, this, &centerRay, &options,
                                           &pOnScreen, &lineSegment](
                                              const SceneObjectMeshLine &m) {
      auto &transformedVertPositions =
          _internal->transformedVerticesPositions.at(m.first);
      auto &vo = _tree.data(m.first);
      if (!vo->selectable()) {
        return true;
      }
      TriMesh::VertHandle v1, v2;
      vo->mesh().fetchLineVerts(m.second, v1, v2);
      Line3 lineInst(transformedVertPositions[v1],
                     transformedVertPositions[v2]);
      auto np =
          DistanceBetweenTwoLines(lineSegment, lineInst).second.second.position;
      auto npOnScreen = options.camera().toScreen(np); ///// test
      if (!options.camera().isVisibleOnScreen(np))
        return true;
      double distance = Distance(npOnScreen, pOnScreen);
      if (distance <= vo->lineWidth() / 2.0) {
        lines[m] = distance;
      }
      return true;
    });
  }

  double minDist = std::numeric_limits<double>::max();

  SceneObjectMeshLine nearest;

  for (auto &r : lines) {
    double d = r.second;
    if (d < minDist) {
      minDist = d;
      nearest = r.first;
    }
  }

  return nearest;
}

SceneObjectMeshPoint Scene::pickPointOnScreen(const RenderOptions &options,
                                              const Point2 &pOnScreen) const {

  if (null() || _internal->points.empty()) {
    return SceneObjectMeshPoint();
  }

  Ray3 centerRay(options.camera().eye(),
                 normalize(options.camera().toSpace(pOnScreen)));

  Box3 bboxAll = boundingBox();
  auto bballAll = bboxAll.outerSphere();

  // discretize the ray
  double startLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) - bballAll.radius);
  double stopLen = std::max(
      0.0, Distance(bballAll.center, options.camera().eye()) + bballAll.radius);
  double stepNum = 1000.0;
  double stepLen = (stopLen - startLen) / stepNum;

  std::map<SceneObjectMeshPoint, double> points;

  for (int i = 0; i <= stepNum; i++) {
    double centerLen = startLen + stepLen * i;
    double nextCenterLen = centerLen + stepLen;

    Point3 centerP = centerRay.anchor + centerLen * centerRay.direction;
    Point3 nextCenterP = centerRay.anchor + nextCenterLen * centerRay.direction;

    Box3 detectionBox = BoundingBox(centerP) | BoundingBox(nextCenterP);
    detectionBox.expand(stepLen);

    _internal->points.search(
        detectionBox, [&points, this, &centerRay, &options,
                       &pOnScreen](const SceneObjectMeshPoint &m) {
          auto &transformedVertPositions =
              _internal->transformedVerticesPositions.at(m.first);
          auto &vo = _tree.data(m.first);
          if (!vo->selectable()) {
            return true;
          }
          TriMesh::VertHandle v = m.second;
          Point3 point = transformedVertPositions.at(v);
          auto npOnScreen = options.camera().toScreen(point); ///// test
          if (!options.camera().isVisibleOnScreen(point))
            return true;
          double distance = Distance(npOnScreen, pOnScreen);
          if (distance <= vo->pointSize() / 2.0) {
            points[m] = distance;
          }
          return true;
        });
  }

  double minDist = std::numeric_limits<double>::max();

  SceneObjectMeshPoint nearest;

  for (auto &r : points) {
    double d = r.second;
    if (d < minDist) {
      minDist = d;
      nearest = r.first;
    }
  }

  return nearest;
}

void Scene::pickOnScreen(const RenderOptions &options, const Point2 &pOnScreen,
                         SceneObjectMeshTriangle &t, SceneObjectMeshLine &l,
                         SceneObjectMeshPoint &p) const {
  t.first.reset();
  l.first.reset();
  p.first.reset();
  if (options.renderMode() & RenderModeFlag::Points)
    p = pickPointOnScreen(options, pOnScreen);
  if (p.first.valid())
    return;
  if (options.renderMode() & RenderModeFlag::Lines)
    l = pickLineOnScreen(options, pOnScreen);
  if (l.first.valid())
    return;
  if (options.renderMode() & RenderModeFlag::Triangles)
    t = pickTriangleOnScreen(options, pOnScreen);
}

std::set<SceneObjectEntity> Scene::pickOnScreen(const RenderOptions &options,
                                                const Point2 &pOnScreen) const {
  SceneObjectMeshPoint p;
  SceneObjectMeshLine l;
  SceneObjectMeshTriangle t;
  pickOnScreen(options, pOnScreen, t, l, p);
  std::set<SceneObjectEntity> s;
  auto e = entityOfPoint(p);
  if (e.first.valid())
    s.insert(e);
  e = entityOfLine(l);
  if (e.first.valid())
    s.insert(e);
  e = entityOfTriangle(t);
  if (e.first.valid())
    s.insert(e);
  return s;
}

void Scene::invokeCallbackFunctions(InteractionID iid,
                                    const std::set<SceneObjectEntity> &ents,
                                    bool selectedOnly) const {
  for (auto &ent : ents) {
    if (!selectedOnly || _tree.data(ent.first)->entityIsSelected(ent.second))
      _tree.data(ent.first)->invokeCallbackFunction(iid, ent.second);
  }
}

void Scene::invokeCallbackFunctionsOnAllSelected(InteractionID iid) const {
  for (auto &n : _tree.nodes()) {
    if (n.exists) {
      for (auto ent : n.data->selectedEntities()) {
        n.data->invokeCallbackFunction(iid, ent);
      }
    }
  }
}

PerspectiveCamera Scene::perfectView(int width, int height,
                                     const Vec3 &up) const {
  PerspectiveCamera camera;
  auto sphere = boundingBox().outerSphere();
  camera.setUp(up, false);
  camera.resizeScreen(Size(width, height), false);
  camera.focusOn(sphere, true);
  return camera;
}

// SceneWidget
class SceneWidget : public QGLWidget {
public:
  std::shared_ptr<RenderOptions> options;
  Scene scene;

  SceneWidget(Scene &&v, std::shared_ptr<RenderOptions> ro,
              QWidget *parent = nullptr)
      : QGLWidget(parent), options(ro), scene(std::move(v)) {
    setMouseTracking(true);
    setAutoBufferSwap(false);
    grabKeyboard();
  }

protected:
  void initializeGL() {
    makeCurrent();
    glEnable(GL_MULTISAMPLE);
    GLint bufs;
    GLint samples;
    glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
    glGetIntegerv(GL_SAMPLES, &samples);
    qDebug("Have %d buffers and %d samples", bufs, samples);
    qglClearColor(MakeQColor(options->backgroundColor()));
    scene.initialize();
  }

  void paintGL() {
    QPainter painter;
    painter.begin(this);
    painter.beginNativePainting();
    qglClearColor(MakeQColor(options->backgroundColor()));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    PerspectiveCamera &camera = options->camera();
    camera.resizeScreen(Size(width(), height()));

    scene.render(*options);

    painter.endNativePainting();
    swapBuffers();
  }

  void resizeGL(int w, int h) {
    PerspectiveCamera &camera = options->camera();
    camera.resizeScreen(Size(w, h));
    camera.setFocal(std::max(width(), height()));
    glViewport(0, 0, w, h);
  }

public:
  void autoSetCamera() {
    auto sphere = scene.boundingBox().outerSphere();
    options->camera().resizeScreen(Size(width(), height()), false);
    options->camera().setFocal(std::max(width(), height()));
    options->camera().focusOn(sphere, true);
    update();
  }

protected:
  virtual void mousePressEvent(QMouseEvent *e) override {
    _lastPos = e->pos();
    if (e->buttons() & Qt::RightButton)
      setCursor(Qt::OpenHandCursor);
    else if (e->buttons() & Qt::MidButton)
      setCursor(Qt::SizeAllCursor);
    else if (e->buttons() & Qt::LeftButton) {
      auto ents =
          scene.pickOnScreen(*options, Point2(e->pos().x(), e->pos().y()));
      if (e->modifiers() & Qt::ControlModifier) {
        for (auto &ent : ents) {
          scene.switchSelect(ent);
        }
      } else {
        scene.clearSelection();
        for (auto &ent : ents) {
          scene.select(ent);
        }
      }
      scene.invokeCallbackFunctions(InteractionID::ClickLeftButton, ents, true);
    }
    update();
  }

  virtual void mouseMoveEvent(QMouseEvent *e) override {
    QVector3D t(e->pos() - _lastPos);
    t.setX(-t.x());
    auto sphere = scene.boundingBox().outerSphere();
    if ((e->buttons() & Qt::RightButton) &&
        !(e->modifiers() & Qt::ShiftModifier)) {
      Vec3 trans = t.x() * options->camera().rightward() +
                   t.y() * options->camera().upward();
      trans *=
          0.02 * Distance(options->camera().eye(), options->camera().center());
      options->camera().moveEyeWithCenterFixed(
          trans, sphere, true, true, options->fixUpDirectionInCameraMove());
      setCursor(Qt::ClosedHandCursor);
      update();
    } else if ((e->buttons() & Qt::MidButton) ||
               ((e->buttons() & Qt::RightButton) &&
                (e->modifiers() & Qt::ShiftModifier))) {
      Vec3 trans = t.x() * options->camera().rightward() +
                   t.y() * options->camera().upward();
      trans *=
          0.02 * Distance(options->camera().eye(), options->camera().center());
      options->camera().translate(trans, sphere, true);
      update();
    }
    _lastPos = e->pos();
  }

  virtual void wheelEvent(QWheelEvent *e) override {
    auto sphere = scene.boundingBox().outerSphere();
    double d = e->delta() / 10;
    double dist = Distance(options->camera().eye(), options->camera().center());
    Vec3 trans = d * dist / 1000.0 * options->camera().forward();
    options->camera().moveEyeWithCenterFixed(trans, sphere, false, true);
    update();
  }

  virtual void mouseReleaseEvent(QMouseEvent *e) override { unsetCursor(); }

  virtual void keyPressEvent(QKeyEvent *e) override {
    if (e->key() == Qt::Key_Space) {
      scene.invokeCallbackFunctionsOnAllSelected(InteractionID::PressSpace);
    }
  }

private:
  QPointF _lastPos;
};

namespace {
template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>>
QWidget *MakeGuiAgent(Noted<T> &value, QWidget *parent = nullptr) {
  QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
  spinBox->setValue(value.component);
  spinBox->setSingleStep(0.01);
  spinBox->setDecimals(3);
  spinBox->setRange(0.0, 1.0);
  auto signal = static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged);
  QObject::connect(spinBox, signal, [&value](double v) {
    // std::cout << "value of " << value.note << " is set to " << v << "!" <<
    // std::endl;
    value.component = v;
  });
  return spinBox;
}

QWidget *MakeGuiAgent(Noted<int> &value, QWidget *parent = nullptr) {
  QSpinBox *spinBox = new QSpinBox(parent);
  spinBox->setValue(value.component);
  spinBox->setSingleStep(1);
  spinBox->setRange(0, std::numeric_limits<int>::max());
  auto signal = static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged);
  QObject::connect(spinBox, signal, [&value](int v) {
    // std::cout << "value of " << value.note << " is set to " << v << "!" <<
    // std::endl;
    value.component = v;
  });
  return spinBox;
}

QWidget *MakeGuiAgent(Noted<bool> &value, QWidget *parent = nullptr) {
  QCheckBox *checkBox = new QCheckBox(parent);
  checkBox->setCheckable(true);
  checkBox->setChecked(value.component);
  QObject::connect(checkBox, &QCheckBox::clicked, [&value, checkBox]() {
    value.component = checkBox->isChecked();
  });
  return checkBox;
}

QWidget *MakeGuiAgent(Noted<Color> &value, QWidget *parent = nullptr) {
  NOT_IMPLEMENTED_YET();
}

template <class... Ts>
void PopUpDialog(QWidget *parent, Noted<Ts> &... values) {
  QString names[] = {QString::fromStdString(values.note)...};
  QWidget *agents[] = {MakeGuiAgent(values, nullptr)...};
  QDialog dialog;
  QFormLayout *layout = new QFormLayout;
  for (int i = 0; i < sizeof...(Ts); i++) {
    layout->addRow(names[i], agents[i]);
  }
  dialog.setLayout(layout);
  dialog.exec();
}
}

void PopUpGui(RenderOptions &options, QWidget *widget) {
  Noted<float> bwColor = NoteAs(options.bwColor(), "Blend Weight of Color");
  Noted<float> bwTexColor =
      NoteAs(options.bwTexColor(), "Blend Weight of Texture Color");
  Noted<bool> cullFrontFace =
      NoteAs(options.cullFrontFace(), "Cull Front Face");
  Noted<bool> cullBackFace = NoteAs(options.cullBackFace(), "Cull Back Face");
  Noted<bool> showPoints = NoteAs<bool>(
      options.renderMode() & RenderModeFlag::Points, "Show Points");
  Noted<bool> showLines =
      NoteAs<bool>(options.renderMode() & RenderModeFlag::Lines, "Show Lines");
  Noted<bool> showFaces = NoteAs<bool>(
      options.renderMode() & RenderModeFlag::Triangles, "Show Faces");
  Noted<float> panoramaHoriCenterRatio = NoteAs(
      options.panoramaHoriCenterRatio(), "Panorama Horizon Center Ratio");
  Noted<float> panoramaAspectRatio =
      NoteAs(options.panoramaAspectRatio(), "Panorama Aspect Ratio (h/w)");
  Noted<int> camFocal =
      NoteAs((int)options.camera().focal(), "Camera Focal Length");

  PopUpDialog(widget, bwColor, bwTexColor, cullFrontFace, cullBackFace,
              showPoints, showLines, showFaces, panoramaHoriCenterRatio,
              panoramaAspectRatio, camFocal);
  options.bwColor() = bwColor.component;
  options.bwTexColor() = bwTexColor.component;
  options.cullFrontFace() = cullFrontFace.component;
  options.cullBackFace() = cullBackFace.component;
  options.renderMode() =
      (showPoints.component ? RenderModeFlag::Points : RenderModeFlag::None) |
      (showLines.component ? RenderModeFlag::Lines : RenderModeFlag::None) |
      (showFaces.component ? RenderModeFlag::Triangles : RenderModeFlag::None);
  options.panoramaHoriCenterRatio() = panoramaHoriCenterRatio.component;
  options.panoramaAspectRatio() = panoramaAspectRatio.component;
  options.camera().setFocal(camFocal.component);
}

SceneBuilder::SceneBuilder() {
  _installingOptions.discretizeOptions.color(ColorTag::Black);
  _installingOptions.discretizeOptions.colorTable(
      ColorTableDescriptor::AllColors);
  _installingOptions.discretizeOptions.isolatedTriangles(false);
  _installingOptions.discretizeOptions.subdivisionNumU(32);
  _installingOptions.discretizeOptions.subdivisionNumV(64);
  _installingOptions.defaultShaderSource =
      OpenGLShaderSourceDescriptor::XTriangles;
  _installingOptions.pointSize = 10.0;
  _installingOptions.lineWidth = 5.0;
}

SceneBuilder::SceneBuilder(const SceneObjectInstallingOptions &defaultO)
    : _installingOptions(defaultO) {}

SceneWidget *SceneBuilder::createWidget(const RenderOptions &options,
                                        QWidget *parent) {
  return new SceneWidget(
      scene(), std::shared_ptr<RenderOptions>(new RenderOptions(options)),
      parent);
}

RenderOptions SceneBuilder::show(bool doModal, bool autoSetCamera,
                                 const RenderOptions &options) {
  auto app = UI::InitGui();

  QMainWindow *mwin = new QMainWindow;

	SceneWidget *w = nullptr;
  while (!w) {
    try {
      w = createWidget(options);
    } catch (...) {
			std::cout << "failed createWidget" << std::endl;
    }
  }

  mwin->setCentralWidget(w);
  mwin->setAttribute(Qt::WA_DeleteOnClose);
  if (!autoSetCamera) {
    mwin->resize(MakeQSize(options.camera().screenSize()));
  } else {
    mwin->resize(800, 800);
  }
  mwin->setWindowTitle(QString::fromStdString(options.winName()));
  mwin->setWindowIcon(UI::DefaultIcon());
  mwin->setStyleSheet(UI::DefaultCSS());

  auto menuView = mwin->menuBar()->addMenu(QObject::tr("View"));
  auto actionSettings = menuView->addAction(QObject::tr("Settings"));
  QObject::connect(actionSettings, &QAction::triggered, [w]() {
    PopUpGui(*w->options, w);
    w->update();
  });
  auto menuAbout = mwin->menuBar()->addMenu(QObject::tr("About"));
  auto actionAbout = menuAbout->addAction(QObject::tr("About"));
  QObject::connect(actionAbout, &QAction::triggered, [mwin]() {
    QMessageBox::about(
        mwin, QObject::tr("About this program"),
        QObject::tr("Panoramix.Vis is the visulization module of project "
                    "Panoramix developped by Yang Hao."));
  });
  mwin->statusBar()->show();

  auto palette = mwin->palette();
  palette.setColor(QPalette::Window, MakeQColor(options.backgroundColor()));
  mwin->setPalette(palette);
  // qDebug() << mwin->styleSheet();
  if (autoSetCamera) {
    w->autoSetCamera();
  }
  auto adjusted = w->options;
  mwin->show();
  if (doModal) {
    UI::ContinueGui();
  }
  return *adjusted;
}

void SceneBuilder::clear() {
  _tree.clear();
  _activeOH.reset();
}
}
}