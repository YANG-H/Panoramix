#pragma once

#include "any.hpp"
#include "basic_types.hpp"

#include "color.hpp"
#include "shader.hpp"
#include "transform.hpp"
#include "gui_util.hpp"

namespace pano {
namespace core {
template <class VertDataT, class HalfDataT, class FaceDataT> class Mesh;
}
namespace gui {

// discretize options
using EntityPtr = AnyPtr;
struct DiscretizeOptions {
#define DECL_PROPERTY(type, name)                                              \
  \
private:                                                                       \
  type _##name;                                                                \
  \
public:                                                                        \
  inline const type &name() const { return _##name; }                          \
  inline type &name() { return _##name; }                                      \
  inline DiscretizeOptions &name(const type &v) {                              \
    _##name = v;                                                               \
    return *this;                                                              \
  }
  DECL_PROPERTY(EntityPtr, entity)
  DECL_PROPERTY(Color, color)
  DECL_PROPERTY(ColorTable, colorTable)
  DECL_PROPERTY(bool, isolatedTriangles)
  DECL_PROPERTY(int, subdivisionNumU)
  DECL_PROPERTY(int, subdivisionNumV)
#undef DECL_PROPERTY
  DiscretizeOptions();
};

// triangular mesh
class TriMesh {
public:
  struct Vertex {
    Vertex();
    Vec4f position;
    Vec3f normal;
    Vec4f color; // the intrinsic color
    Vec2f texCoord;

    template <class Archive> inline void serialize(Archive &ar) {
      ar(position, normal, color, texCoord);
    }
  };

  using VertHandle = uint32_t;
  using PointHandle = uint32_t;
  using LineHandle = uint32_t;
  using TriangleHandle = uint32_t;

public:
  const Vertex &vertex(VertHandle vh) const { return vertices[vh]; }
  Vertex &vertex(VertHandle vh) { return vertices[vh]; }

  VertHandle addVertex(const Vertex &v, bool asPoint = false,
                       EntityPtr ent = nullptr);
  VertHandle addVertex(const Point3 &p, const DiscretizeOptions &o,
                       bool asPoint = false);
  VertHandle addVertex(const Point3 &p, const Vec3 &normal,
                       const DiscretizeOptions &o, bool asPoint = false);

  size_t numerOfPoints() const;
  void fetchPointVerts(PointHandle p, VertHandle &v) const;

  template <class T> T &entityAtPoint(PointHandle p) const {
    return *static_cast<T *>(entPoints[p]);
  }

  LineHandle addLine(VertHandle v1, VertHandle v2, EntityPtr ent = nullptr);
  LineHandle addIsolatedLine(const Vertex &v1, const Vertex &v2,
                             EntityPtr ent = nullptr);
  size_t numberOfLines() const;
  void fetchLineVerts(LineHandle l, VertHandle &v1, VertHandle &v2) const;

  template <class T> T &entityAtLine(LineHandle l) const {
    return *static_cast<T *>(entLines[l]);
  }

  TriangleHandle addTriangle(VertHandle v1, VertHandle v2, VertHandle v3,
                             EntityPtr entId = nullptr);
  TriangleHandle addIsolatedTriangle(const Vertex &v1, const Vertex &v2,
                                     const Vertex &v3, EntityPtr ent = nullptr);
  size_t numberOfTriangles() const;
  void fetchTriangleVerts(TriangleHandle t, VertHandle &v1, VertHandle &v2,
                          VertHandle &v3) const;

  template <class T> T &entityAtTriangle(TriangleHandle t) const {
    return *static_cast<T *>(entTriangles[t]);
  }

  void addQuad(VertHandle v1, VertHandle v2, VertHandle v3, VertHandle v4,
               EntityPtr ent = nullptr);
  void addPolygon(const std::vector<VertHandle> &vhs, EntityPtr ent = nullptr);

  void clear();

  Box3 boundingBox() const;

  template <class Archive> inline void serialize(Archive &ar) {
    ar(vertices, iverts, iPoints, iLines, iTriangles, entPoints, entLines,
       entTriangles);
  }

public:
  std::vector<Vertex> vertices;
  std::vector<VertHandle> iverts;

  std::vector<PointHandle> iPoints;
  std::vector<LineHandle> iLines;
  std::vector<TriangleHandle> iTriangles;

  std::vector<EntityPtr> entPoints;
  std::vector<EntityPtr> entLines;
  std::vector<EntityPtr> entTriangles;
};

// discretization

inline void Discretize(TriMesh &mesh, const Dummy &d,
                       const DiscretizeOptions &o) {}

template <class T>
inline void Discretize(TriMesh &mesh, const Point<T, 3> &p,
                       const DiscretizeOptions &o) {
  TriMesh::Vertex v;
  v.position = Vec4f(p[0], p[1], p[2], 1.0f);
  v.color = o.color();
  mesh.addVertex(v, true, o.entity());
}

template <class T>
inline void Discretize(TriMesh &mesh, const Point<T, 2> &p,
                       const DiscretizeOptions &o) {
  Discretize(mesh, cat(p, 0.0), o);
}

template <class T>
inline void Discretize(TriMesh &mesh, const Line<Point<T, 3>> &l,
                       const DiscretizeOptions &o) {
  TriMesh::Vertex v1, v2;
  v1.position = cat(ecast<float>(l.first), 1.0f);
  v1.color = o.color();
  v2.position = cat(ecast<float>(l.second), 1.0f);
  v2.color = o.color();
  mesh.addIsolatedLine(v1, v2, o.entity());
}

template <class T>
inline void Discretize(TriMesh &mesh, const Line<Point<T, 2>> &l,
                       const DiscretizeOptions &o) {
  Discretize(mesh, Line<Point<T, 3>>(cat(l.first, 0.0), cat(l.second, 0.0)), o);
}

template <class T>
inline void Discretize(TriMesh &mesh, const Chain<Point<T, 3>> &c,
                       const DiscretizeOptions &o) {
  if (c.size() == 0)
    return;
  std::vector<TriMesh::VertHandle> vhandles(c.size());
  for (int i = 0; i < c.size(); i++) {
    TriMesh::Vertex v;
    v.position = Vec4f(c.points[i][0], c.points[i][1], c.points[i][2], 1.0);
    v.color = o.color();
    vhandles[i] = mesh.addVertex(v, false, o.entity());
  }
  for (int i = 0; i + 1 < c.size(); i++) {
    mesh.addLine(vhandles[i], vhandles[i + 1], o.entity());
  }
  if (c.closed) {
    mesh.addLine(vhandles.back(), vhandles.front(), o.entity());
  }
}

void Discretize(TriMesh &mesh, const LayeredShape3 &m,
                const DiscretizeOptions &o);
void Discretize(TriMesh &mesh, const Sphere3 &s, const DiscretizeOptions &o);

template <class T>
void Discretize(TriMesh &mesh, const Sphere<T, 3> &s,
                const DiscretizeOptions &o) {
  Discretize(
      mesh, Sphere3{ecast<double>(s.center), static_cast<double>(s.radius)}, o);
}

template <class T>
void Discretize(TriMesh &mesh, const Box<T, 3> &b, const DiscretizeOptions &o) {
  std::vector<TriMesh::VertHandle> vhandles;
  vhandles.reserve(8);
  auto center = b.center();
  // add vertices
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        TriMesh::Vertex v;
        auto c = b.corner({i == 1, j == 1, k == 1});
        v.position = Vec4f(c[0], c[1], c[2], 1.0);
        v.normal = normalize(c - center);
        v.color = o.color();
        vhandles.push_back(mesh.addVertex(v, true, o.entity()));
      }
    }
  }
  // add edges
  mesh.addLine(vhandles[0], vhandles[4], o.entity());
  mesh.addLine(vhandles[1], vhandles[5], o.entity());
  mesh.addLine(vhandles[3], vhandles[7], o.entity());
  mesh.addLine(vhandles[2], vhandles[6], o.entity());

  mesh.addLine(vhandles[0], vhandles[2], o.entity());
  mesh.addLine(vhandles[1], vhandles[3], o.entity());
  mesh.addLine(vhandles[5], vhandles[7], o.entity());
  mesh.addLine(vhandles[4], vhandles[6], o.entity());

  mesh.addLine(vhandles[0], vhandles[1], o.entity());
  mesh.addLine(vhandles[2], vhandles[3], o.entity());
  mesh.addLine(vhandles[4], vhandles[5], o.entity());
  mesh.addLine(vhandles[6], vhandles[7], o.entity());

  // add faces
  mesh.addPolygon({vhandles[0], vhandles[4], vhandles[5], vhandles[1]},
                  o.entity());
  mesh.addPolygon({vhandles[4], vhandles[6], vhandles[7], vhandles[5]},
                  o.entity());
  mesh.addPolygon({vhandles[6], vhandles[2], vhandles[3], vhandles[7]},
                  o.entity());
  mesh.addPolygon({vhandles[2], vhandles[0], vhandles[1], vhandles[3]},
                  o.entity());
  mesh.addPolygon({vhandles[1], vhandles[5], vhandles[7], vhandles[3]},
                  o.entity());
  mesh.addPolygon({vhandles[2], vhandles[6], vhandles[4], vhandles[0]},
                  o.entity());
}

template <class T>
void Discretize(TriMesh &mesh, const Polygon<Point<T, 3>> &p,
                const DiscretizeOptions &o) {
  std::vector<TriMesh::VertHandle> vhandles(p.corners.size());
  for (int i = 0; i < p.corners.size(); i++) {
    TriMesh::Vertex v;
    v.position = Vec4f(p.corners[i][0], p.corners[i][1], p.corners[i][2], 1.0);
    v.normal = p.normal;
    v.color = o.color();
    vhandles[i] = mesh.addVertex(v, false, o.entity());
  }
  mesh.addPolygon(vhandles, o.entity());
}

void Discretize(TriMesh &mesh, const SingleViewPolygon3 &spp,
                const DiscretizeOptions &o);

template <class T, class H, class F>
void Discretize(TriMesh &mesh, const Mesh<Point<T, 3>, H, F> &m,
                const DiscretizeOptions &o) {
  using MeshType = Mesh<Point<T, 3>, H, F>;
  std::vector<TriMesh::VertHandle> vhandles(m.internalVertices().size());
  for (auto &vv : m.vertices()) {
    TriMesh::Vertex v;
    v.position = cat(vv.data, (T)1.0);
    v.color = o.color();
    vhandles[vv.topo.hd.id] = mesh.addVertex(v, true, o.entity());
  }
  for (auto &hh : m.halfedges()) {
    auto vh1 = hh.topo.from();
    auto vh2 = hh.topo.to();
    mesh.addLine(vhandles[vh1.id], vhandles[vh2.id], o.entity());
  }
  for (auto &ff : m.faces()) {
    std::vector<TriMesh::VertHandle> vhs;
    vhs.reserve(ff.topo.halfedges.size());
    for (auto &h : ff.topo.halfedges) {
      vhs.push_back(m.topo(h).to().id);
    }
    mesh.addPolygon(vhs, o.entity());
  }
}

template <class T, class E>
void Discretize(TriMesh &mesh, const TransformedIn3D<T, E> &t,
                const DiscretizeOptions &o) {
  int prevVertSize = mesh.iverts.size();
  Discretize(mesh, t.component, o);
  int curVertSize = mesh.iverts.size();
  Mat<float, 4, 4> mat4 = t.mat4;
  Mat<float, 3, 3> mat3(t.mat4(0, 0), t.mat4(1, 0), t.mat4(2, 0), t.mat4(0, 1),
                        t.mat4(1, 1), t.mat4(2, 1), t.mat4(0, 2), t.mat4(1, 2),
                        t.mat4(2, 2));
  for (int i = prevVertSize; i < curVertSize; i++) {
    auto &v = mesh.vertex(mesh.iverts[i]);
    v.position = mat4 * v.position;
    v.normal = mat3 * v.normal;
  }
}

template <class T>
inline void Discretize(TriMesh &mesh, const Classified<T> &c,
                       const DiscretizeOptions &o) {
  auto oo = o;
  oo.color(o.colorTable()[c.claz]);
  Discretize(mesh, c.component, oo);
}

template <class T>
inline void Discretize(TriMesh &mesh, const Colored<T> &c,
                       const DiscretizeOptions &o) {
  auto oo = o;
  oo.color(c.color);
  Discretize(mesh, c.component, oo);
}

template <class T, class D>
inline void Discretize(TriMesh &mesh, const Decorated<T, D> &d,
                       const DiscretizeOptions &o) {
  Discretize(mesh, d.component, o);
}

template <class IterT>
inline void DiscretizeRange(TriMesh &mesh, IterT begin, IterT end,
                            const DiscretizeOptions &o) {
  auto oo = o;
  while (begin != end) {
    oo.entity(&(*begin));
    Discretize(mesh, *begin, oo);
    ++begin;
  }
}

// Is discretizable ?
namespace {
template <class T> struct IsDiscretizableImp {
  template <class TT>
  static auto test(int)
      -> decltype(gui::Discretize(std::declval<TriMesh &>(), std::declval<TT>(),
                                  std::declval<DiscretizeOptions>()),
                  std::true_type()) {
    return std::true_type();
  }
  template <class> static std::false_type test(...) {
    return std::false_type();
  }
  static const bool value =
      std::is_same<decltype(test<T>(0)), std::true_type>::value;
};
}

template <class T>
struct IsDiscretizable
    : std::integral_constant<bool, IsDiscretizableImp<T>::value> {};
}

namespace core {
inline Box3 BoundingBox(const gui::TriMesh &m) { return m.boundingBox(); }
}
}
