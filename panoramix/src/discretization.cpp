#include "pch.hpp"


#include "algorithms.hpp"
#include "discretization.hpp"

namespace pano {
namespace gui {

using namespace core;

namespace {

inline Vec3 ToVec3Affine(const Vec4 &v4) {
  return Vec3(v4[0], v4[1], v4[2]) / v4[3];
}

inline Vec2 ToVec2(const Vec3 &v3) { return Vec2(v3[0], v3[1]); }
}

DiscretizeOptions::DiscretizeOptions()
    : _color(0, 0, 0, 1), _isolatedTriangles(false), _subdivisionNumU(32),
      _subdivisionNumV(64) {}

// tri mesh implementation
TriMesh::Vertex::Vertex()
    : position(0, 0, 0, 1), normal(0, 0, 0), color(0, 0, 0, 1), texCoord(0, 0) {
}

TriMesh::VertHandle TriMesh::addVertex(const TriMesh::Vertex &v, bool asPoint,
                                       EntityPtr ent) {
  vertices.push_back(v);
  iverts.push_back(static_cast<TriMesh::VertHandle>(vertices.size() - 1));
  if (asPoint) {
    iPoints.push_back(static_cast<TriMesh::VertHandle>(vertices.size() - 1));
    entPoints.push_back(ent);
  }
  return iverts.back();
}

TriMesh::VertHandle TriMesh::addVertex(const core::Point3 &p,
                                       const DiscretizeOptions &o,
                                       bool asPoint) {
  TriMesh::Vertex v;
  v.position = cat(p, 1.0);
  v.color = o.color();
  return addVertex(v, asPoint, o.entity());
}

TriMesh::VertHandle TriMesh::addVertex(const core::Point3 &p,
                                       const core::Vec3 &normal,
                                       const DiscretizeOptions &o,
                                       bool asPoint) {
  TriMesh::Vertex v;
  v.position = cat(p, 1.0);
  v.normal = normal;
  v.color = o.color();
  return addVertex(v, asPoint, o.entity());
}

size_t TriMesh::numerOfPoints() const { return iPoints.size(); }

void TriMesh::fetchPointVerts(PointHandle p, VertHandle &v) const {
  v = iPoints[p];
}

TriMesh::LineHandle TriMesh::addLine(TriMesh::VertHandle v1,
                                     TriMesh::VertHandle v2, EntityPtr ent) {
  iLines.push_back(v1);
  iLines.push_back(v2);
  entLines.push_back(ent);
  return iLines.size() / 2;
}

TriMesh::LineHandle TriMesh::addIsolatedLine(const Vertex &v1, const Vertex &v2,
                                             EntityPtr ent) {
  vertices.push_back(v1);
  iLines.push_back(vertices.size() - 1);
  vertices.push_back(v2);
  iLines.push_back(vertices.size() - 1);
  entLines.push_back(ent);
  return iLines.size() / 2;
}

size_t TriMesh::numberOfLines() const { return iLines.size() / 2; }

void TriMesh::fetchLineVerts(TriMesh::LineHandle l, TriMesh::VertHandle &v1,
                             TriMesh::VertHandle &v2) const {
  v1 = iLines[l * 2];
  v2 = iLines[l * 2 + 1];
}

TriMesh::TriangleHandle TriMesh::addTriangle(TriMesh::VertHandle v1,
                                             TriMesh::VertHandle v2,
                                             TriMesh::VertHandle v3,
                                             EntityPtr ent) {
  iTriangles.push_back(v1);
  iTriangles.push_back(v2);
  iTriangles.push_back(v3);
  entTriangles.push_back(ent);
  return iTriangles.size() / 3;
}

TriMesh::TriangleHandle TriMesh::addIsolatedTriangle(const Vertex &v1,
                                                     const Vertex &v2,
                                                     const Vertex &v3,
                                                     EntityPtr ent) {
  vertices.push_back(v1);
  iTriangles.push_back(vertices.size() - 1);
  vertices.push_back(v2);
  iTriangles.push_back(vertices.size() - 1);
  vertices.push_back(v3);
  iTriangles.push_back(vertices.size() - 1);
  entTriangles.push_back(ent);
  return iTriangles.size() / 3;
}

size_t TriMesh::numberOfTriangles() const { return iTriangles.size() / 3; }

void TriMesh::fetchTriangleVerts(TriMesh::TriangleHandle t,
                                 TriMesh::VertHandle &v1,
                                 TriMesh::VertHandle &v2,
                                 TriMesh::VertHandle &v3) const {
  v1 = iTriangles[t * 3];
  v2 = iTriangles[t * 3 + 1];
  v3 = iTriangles[t * 3 + 2];
}

void TriMesh::addQuad(TriMesh::VertHandle v1, TriMesh::VertHandle v2,
                      TriMesh::VertHandle v3, TriMesh::VertHandle v4,
                      EntityPtr ent) {
  addTriangle(v1, v2, v3, ent);
  addTriangle(v1, v3, v4, ent);
}

void TriMesh::addPolygon(const std::vector<TriMesh::VertHandle> &vhs,
                         EntityPtr ent) {
  assert(vhs.size() >= 3);
  // get normal direction
  Vec3 normal =
      normalize((ToVec3Affine(vertices[vhs[1]].position) -
                 ToVec3Affine(vertices[vhs[0]].position))
                    .cross((ToVec3Affine(vertices[vhs[2]].position) -
                            ToVec3Affine(vertices[vhs[1]].position))));

  TriangulatePolygon(vhs.begin(), vhs.end(),
                     [this, &normal](VertHandle vh) {
                       Vec3 v = ToVec3Affine(vertices[vh].position);
                       return ToVec2(v - v.dot(normal) * normal);
                     },
                     [this, ent](VertHandle a, VertHandle b, VertHandle c) {
                       addTriangle(a, b, c, ent);
                     });
}

void TriMesh::clear() {
  vertices.clear();
  iverts.clear();
  iPoints.clear();
  iLines.clear();
  iTriangles.clear();
  entPoints.clear();
  entLines.clear();
  entTriangles.clear();
}

Box3 TriMesh::boundingBox() const {
  if (vertices.empty())
    return Box3();
  Box3 box(ToVec3Affine(vertices.front().position),
           ToVec3Affine(vertices.front().position));
  for (auto &v : vertices) {
    auto p = ToVec3Affine(v.position);
    box = box | BoundingBox(p);
  }
  return box;
}

void Discretize(TriMesh &mesh, const core::LayeredShape3 &m,
                const DiscretizeOptions &o) {
  if (m.empty())
    return;
  Vec3 x, y, z, origin = Vec3(0, 0, 0);
  // set origin to the gravity center of all corners
  double cnum = 0;
  for (auto &cs : m.layers) {
    for (auto &c : cs) {
      origin += c;
      cnum++;
    }
  }
  origin /= cnum;

  std::tie(x, y) = ProposeXYDirectionsFromZDirection(m.normal);
  z = normalize(m.normal);

  // add verts
  std::vector<std::vector<TriMesh::VertHandle>> vhs(m.size());
  std::vector<std::vector<double>> projAngles(m.size());
  std::vector<std::vector<int>> ids(m.size());
  for (int i = 0; i < m.size(); i++) {
    for (auto &p : m.layers[i]) {
      auto n = normalize(p - origin);
      vhs[i].push_back(mesh.addVertex(p, n, o, false));
      Vec2 proj((p - origin).dot(x), (p - origin).dot(y));
      projAngles[i].push_back(SignedAngle(Vec2(1, 0), proj));
    }
    ids[i].resize(m.layers[i].size());
    std::iota(ids[i].begin(), ids[i].end(), 0);
    auto &angles = projAngles[i];
    std::sort(ids[i].begin(), ids[i].end(),
              [&angles](int a, int b) { return angles[a] < angles[b]; });
  }

  mesh.addPolygon(vhs.front(), o.entity());
  for (int i = 1; i < m.size(); i++) {
    // find nearest two vertices in last layer
    if (vhs[i].empty()) {
      continue;
    }
    auto &lastVhs = vhs[i - 1];
    auto &lastAngles = projAngles[i - 1];
    auto &lastids = ids[i - 1];
    auto &thisVhs = vhs[i];
    auto &thisAngles = projAngles[i];
    auto &thisids = ids[i];
    int a = 0, b = 0;
    double anglea = lastAngles[lastids[a]], angleb = thisAngles[thisids[b]];
    for (int k = 0; k < lastids.size() + thisids.size(); k++) {
      if (anglea < angleb) {
        int newa = (a + 1) % lastids.size();
        mesh.addTriangle(lastVhs[lastids[a]], thisVhs[thisids[b]],
                         lastVhs[lastids[newa]], o.entity());
        a = newa;
        double newanglea = lastAngles[lastids[newa]];
        if (newanglea < anglea) {
          newanglea += M_PI * 2;
        }
        anglea = newanglea;
      } else {
        int newb = (b + 1) % thisids.size();
        mesh.addTriangle(thisVhs[thisids[b]], thisVhs[thisids[newb]],
                         lastVhs[lastids[a]], o.entity());
        b = newb;
        double newangleb = thisAngles[thisids[newb]];
        if (newangleb < angleb) {
          newangleb += M_PI * 2;
        }
        angleb = newangleb;
      }
    }
  }

  std::reverse(vhs.back().begin(), vhs.back().end());
  mesh.addPolygon(vhs.back(), o.entity());
}

void Discretize(TriMesh &mesh, const core::Sphere3 &s,
                const DiscretizeOptions &o) {
  int m = o.subdivisionNumU();
  int n = o.subdivisionNumV();
  if (!o.isolatedTriangles()) {
    mesh.vertices.reserve(mesh.vertices.size() + m * n);
    std::vector<std::vector<TriMesh::VertHandle>> vhs(
        m, std::vector<TriMesh::VertHandle>(n));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        float xratio = 1.0f / n * j;
        float yratio = 1.0f / (m - 1) * i;
        float xangle = M_PI * 2 * xratio;
        float yangle = M_PI * yratio - M_PI_2;
        Vec4 point = {cos(xangle) * cos(yangle) * s.radius + s.center[0],
                      sin(xangle) * cos(yangle) * s.radius + s.center[1],
                      sin(yangle) * s.radius + s.center[2], 1};
        TriMesh::Vertex v;
        v.position = point;
        v.texCoord = {xratio, yratio};
        v.color = o.color();
        vhs[i][j] = mesh.addVertex(v, false, o.entity());
      }
    }
    for (int i = 1; i < m; i++) {
      int previ = i == 0 ? m - 1 : i - 1;
      for (int j = 0; j < n; j++) {
        int prevj = j == 0 ? n - 1 : j - 1;
        mesh.addTriangle(vhs[i][j], vhs[i][prevj], vhs[previ][prevj], o.entity());
        mesh.addTriangle(vhs[i][j], vhs[previ][prevj], vhs[previ][j], o.entity());
      }
    }
  } else {
    std::vector<std::vector<TriMesh::Vertex>> vs(
        m, std::vector<TriMesh::Vertex>(n));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        float xratio = 1.0f / n * j;
        float yratio = 1.0f / (m - 1) * i;
        float xangle = M_PI * 2 * xratio;
        float yangle = M_PI * yratio - M_PI_2;
        Vec4 point = {cos(xangle) * cos(yangle) * s.radius + s.center[0],
                      sin(xangle) * cos(yangle) * s.radius + s.center[1],
                      sin(yangle) * s.radius + s.center[2], 1};
        TriMesh::Vertex v;
        v.position = point;
        v.texCoord = {xratio, yratio};
        v.color = o.color();
        vs[i][j] = v;
      }
    }
    for (int i = 1; i < m; i++) {
      int previ = i == 0 ? m - 1 : i - 1;
      for (int j = 0; j < n; j++) {
        int prevj = j == 0 ? n - 1 : j - 1;
        mesh.addIsolatedTriangle(vs[i][j], vs[i][prevj], vs[previ][prevj],
                                 o.entity());
        mesh.addIsolatedTriangle(vs[i][j], vs[previ][prevj], vs[previ][j],
                                 o.entity());
      }
    }
  }
}

void Discretize(TriMesh &mesh, const SingleViewPolygon3 &spp,
                const DiscretizeOptions &o) {
  std::vector<Vec3> cs(spp.corners.size());
  for (int i = 0; i < spp.corners.size(); i++) {
    Ray3 line(spp.projection_center, spp.corners[i] - spp.projection_center);
    cs[i] = Intersection(line, spp.plane);
  }
  std::vector<TriMesh::VertHandle> vhandles(cs.size());
  for (int i = 0; i < cs.size(); i++) {
    TriMesh::Vertex v;
    v.position = cat(cs[i], 1.0);
    v.normal = spp.plane.normal;
    v.color = o.color();
    vhandles[i] = mesh.addVertex(v, false, o.entity());
  }
  mesh.addPolygon(vhandles, o.entity());
}
}
}