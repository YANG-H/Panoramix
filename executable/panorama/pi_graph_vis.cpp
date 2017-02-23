#include "pch.hpp"

#include "algorithms.hpp"

#include "pi_graph_postprocess.hpp"
#include "pi_graph_vis.hpp"

namespace pano {
namespace experimental {

inline double DepthAt(const Vec3 &direction, const Plane3 &plane,
                      const Point3 &eye = Point3(0, 0, 0)) {
  return norm(Intersection(Ray3(eye, direction), plane));
}
inline double DepthAt(const Vec3 &direction, const Line3 &line,
                      const Point3 &eye = Point3(0, 0, 0)) {
  return norm(
      DistanceBetweenTwoLines(Ray3(Point3(0, 0, 0), direction), line.ray())
          .second.first);
}

double DepthOfVertexAt(const PIConstraintGraph &cg,
                       const PIGraph<PanoramicCamera> &mg, int ent,
                       const Vec3 &direction, const Point3 &eye = Origin()) {
  auto &v = cg.entities[ent];
  auto &plane = v.supportingPlane.reconstructed;
  return DepthAt(direction, plane, eye);
}

void VisualizeReconstruction(
    const PICGDeterminablePart &dp, const PIConstraintGraph &cg,
    const PIGraph<PanoramicCamera> &mg, bool showConnectionLines,
    const std::function<gui::Color(int vert)> &vertColor,
    const std::function<void(int vert)> &vertClick, bool doModal) {

  gui::ResourceStore::set("texture", mg.view.image);

  gui::SceneBuilder viz;
  viz.installingOptions().discretizeOptions.colorTable(
      gui::ColorTableDescriptor::RGB);
  std::vector<core::Decorated<gui::Colored<core::SingleViewPolygon3>, int>>
      spps;
  std::vector<core::Decorated<gui::Colored<core::Line3>, int>> lines;

  for (int vert : dp.determinableEnts) {
    auto &v = cg.entities[vert];
    if (v.isSeg()) {
      int seg = v.id;
      if (!mg.seg2control[seg].used)
        continue;
      core::SingleViewPolygon3 spp;
      auto &contours = mg.seg2contours[seg];
      if (contours.empty() || contours.front().empty()) {
        continue;
      }
      // filter corners
      core::FilterBy(contours.front().begin(), contours.front().end(),
                     std::back_inserter(spp.corners),
                     [](const core::Vec3 &a, const core::Vec3 &b) -> bool {
                       return core::AngleBetweenDirected(a, b) > 0.0;
                     });
      if (spp.corners.size() < 3)
        continue;

      spp.projection_center = core::Point3(0, 0, 0);
      spp.plane = v.supportingPlane.reconstructed;
      if (HasValue(spp.plane, IsInfOrNaN<double>)) {
        WARNNING("inf plane");
        continue;
      }
      spps.push_back(core::DecorateAs(
          std::move(gui::ColorAs(spp, vertColor(vert))), vert));
    } else {
      int line = v.id;
      auto &plane = v.supportingPlane.reconstructed;
      auto &projectedLine = mg.lines[line].component;
      Line3 reconstructedLine(
          Intersection(Ray3(Origin(), projectedLine.first), plane),
          Intersection(Ray3(Origin(), projectedLine.second), plane));
      if (HasValue(reconstructedLine, IsInfOrNaN<double>)) {
        WARNNING("inf line");
        continue;
      }
      lines.push_back(core::DecorateAs(
          gui::ColorAs(reconstructedLine, vertColor(vert)), vert));
    }
  }

  auto ent2string = [&mg, &cg](int ent) -> std::string {
    auto &e = cg.entities[ent];
    std::stringstream ss;
    if (e.isSeg()) {
      ss << "seg " << e.id << " dof: " << mg.seg2control[e.id].dof();
    } else {
      ss << "line " << e.id << " claz: " << mg.lines[e.id].claz;
    }
    return ss.str();
  };
  viz.begin(
         spps,
         [&mg, &cg, &dp, &vertClick, ent2string](
             gui::InteractionID iid,
             const core::Decorated<gui::Colored<core::SingleViewPolygon3>,
                                   int> &spp) {
           int ent = spp.decoration;
           std::cout << ent2string(ent) << std::endl;
           for (int cons : cg.ent2cons[ent]) {
             if (Contains(dp.consBetweenDeterminableEnts, cons)) {
               if (cg.constraints[cons].isConnection()) {
                 std::cout << "connect with "
                           << ent2string(cg.constraints[cons].ent1 == ent
                                             ? cg.constraints[cons].ent2
                                             : cg.constraints[cons].ent1)
                           << " using " << cg.constraints[cons].anchors.size()
                           << " anchors, the weight is "
                           << cg.constraints[cons].weight << std::endl;
               } else {
                 std::cout << "coplanar with "
                           << ent2string(cg.constraints[cons].ent1 == ent
                                             ? cg.constraints[cons].ent2
                                             : cg.constraints[cons].ent1)
                           << ", the weight is " << cg.constraints[cons].weight
                           << std::endl;
               }
             }
           }
           if (vertClick) {
             vertClick(ent);
           }
         })
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .resource("texture")
      .end();
  viz.installingOptions().discretizeOptions.color(gui::ColorTag::Black);
  viz.installingOptions().lineWidth = 4.0;
  viz.begin(
         lines /*, [&mg, &cg, &dp, &vertClick, ent2string](gui::InteractionID iid,
                const core::Decorated<gui::Colored<Line3>, int> & line) {
                int ent = line.decoration;
                std::cout << ent2string(ent) << std::endl;
                for (int cons : cg.ent2cons[ent]) {
                    if (Contains(dp.consBetweenDeterminableEnts, cons)) {
                        if (cg.constraints[cons].isConnection()) {
                            std::cout << "connect with "
                                << ent2string(cg.constraints[cons].ent1 == ent ? cg.constraints[cons].ent2 : cg.constraints[cons].ent1)
                                << " using "
                                << cg.constraints[cons].anchors.size()
                                << " anchors, the weight is " << cg.constraints[cons].weight << std::endl;
                        } else {
                            std::cout << "coplanar with "
                                << ent2string(cg.constraints[cons].ent1 == ent ? cg.constraints[cons].ent2 : cg.constraints[cons].ent1)
                                << ", the weight is " << cg.constraints[cons].weight << std::endl;
                        }
                    }
                }
                vertClick(line.decoration);
            }*/)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XLines)
      .end();

  if (showConnectionLines) {
    std::vector<core::Decorated<gui::Colored<core::Line3>, std::pair<int, int>>>
        connectionLines;
    std::vector<core::Point3> connectionLineEnds;
    for (int cons : dp.consBetweenDeterminableEnts) {
      auto &e = cg.constraints[cons];
      if (!Contains(dp.determinableEnts, e.ent1) ||
          !Contains(dp.determinableEnts, e.ent2))
        continue;
      int snum = cg.entities[e.ent1].isSeg() + cg.entities[e.ent2].isSeg();
      auto &samples = e.anchors;
      for (auto &ss : samples) {
        double d1 = DepthOfVertexAt(cg, mg, e.ent1, ss);
        double d2 = DepthOfVertexAt(cg, mg, e.ent2, ss);
        Line3 line(normalize(ss) * d1, normalize(ss) * d2);
        connectionLines.push_back(DecorateAs(
            gui::ColorAs(line, snum == 0 ? gui::Black
                                         : snum == 1 ? gui::Blue : gui::Yellow),
            std::make_pair(e.ent1, e.ent2)));
        connectionLineEnds.push_back(
            connectionLines.back().component.component.first);
        connectionLineEnds.push_back(
            connectionLines.back().component.component.second);
      }
    }

    viz.installingOptions().discretizeOptions.color(gui::ColorTag::Black);
    viz.installingOptions().lineWidth = 3.0;
    viz.begin(connectionLines,
              [&mg, &cg,
               ent2string](gui::InteractionID iid,
                           const core::Decorated<gui::Colored<Line3>,
                                                 std::pair<int, int>> &line) {
                std::cout << "this is an anchor of edge connecting ";
                auto &v1 = cg.entities[line.decoration.first];
                auto &v2 = cg.entities[line.decoration.second];
                std::cout << "connecting " << ent2string(line.decoration.first)
                          << " and " << ent2string(line.decoration.second)
                          << std::endl;
              })
        .shaderSource(gui::OpenGLShaderSourceDescriptor::XLines)
        .end();
    viz.installingOptions().pointSize = 5.0;
    viz.begin(connectionLineEnds)
        .shaderSource(gui::OpenGLShaderSourceDescriptor::XPoints)
        .end();
  }

  viz.show(doModal, false,
           gui::RenderOptions()
               .cullFrontFace(false)
               .cullBackFace(true)
               .bwColor(1.0)
               .bwTexColor(0.0)
               .camera(PerspectiveCamera(600, 600, Point2(300, 300), 450,
                                         Point3(2, 2, -2), Point3(0, 0, 0))));
}

void VisualizeReconstructionCompact(const Image &im,
                                    const PICGDeterminablePart &dp,
                                    const PIConstraintGraph &cg,
                                    const PIGraph<PanoramicCamera> &mg,
                                    bool doModel, bool autoCam,
                                    bool fixCamUpDir) {
  Image3ub reversedIm = im.clone();
  ReverseRows(reversedIm);
  gui::ResourceStore::set("texture", reversedIm);

  auto compactPolygons = CompactModel(dp, cg, mg, 0.1);
  auto e = std::remove_if(
      compactPolygons.begin(), compactPolygons.end(), [](const Polygon3 &poly) {
        return poly.normal == Origin() || poly.corners.size() <= 2;
      });

  compactPolygons.erase(e, compactPolygons.end());

  gui::SceneBuilder viz;
  viz.installingOptions().discretizeOptions.colorTable(
      gui::ColorTableDescriptor::RGB);
  viz.begin(compactPolygons /*, sppCallbackFun*/)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .resource("texture")
      .end();

  std::vector<core::Decorated<gui::Colored<core::Line3>, int>> lines;
  for (int vert : dp.determinableEnts) {
    auto &v = cg.entities[vert];
    if (v.isLine()) {
      int line = v.id;
      auto &plane = v.supportingPlane.reconstructed;
      auto &projectedLine = mg.lines[line].component;
      Line3 reconstructedLine(
          Intersection(Ray3(Origin(), projectedLine.first), plane),
          Intersection(Ray3(Origin(), projectedLine.second), plane));
      if (HasValue(reconstructedLine, IsInfOrNaN<double>)) {
        WARNNING("inf line");
        continue;
      }
      lines.push_back(
          core::DecorateAs(gui::ColorAs(reconstructedLine, gui::Black), vert));
    }
  }
  viz.begin(lines)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XLines)
      .end();

  if (!autoCam) {
    viz.show(doModel, false,
             gui::RenderOptions()
                 .cullFrontFace(true)
                 .cullBackFace(false)
                 .bwColor(0.0)
                 .bwTexColor(1.0)
                 .camera(PerspectiveCamera(600, 600, Point2(300, 300), 450,
                                           Point3(2, 2, -2), Point3(0, 0, 0))));
  } else {
    viz.show(doModel, true, gui::RenderOptions()
                                .cullFrontFace(true)
                                .cullBackFace(false)
                                .bwColor(0.0)
                                .bwTexColor(1.0)
                                .fixUpDirectionInCameraMove(fixCamUpDir));
  }
}

void VisualizeLayoutAnnotation(const PILayoutAnnotation &anno,
                               double mergeDepthsThres) {

  if (anno.nfaces() == 0) {
    return;
  }

  std::vector<Polygon3> face2polygon(anno.nfaces());
  if (mergeDepthsThres > 0) {

    // get corner to faces
    std::vector<std::vector<int>> corner2faces(anno.ncorners());
    for (int face = 0; face < anno.nfaces(); face++) {
      for (int corner : anno.face2corners[face]) {
        corner2faces[corner].push_back(face);
      }
    }

    std::vector<std::map<int, double>> corner2faceDepths(anno.ncorners());
    for (int corner = 0; corner < anno.ncorners(); corner++) {
      auto &faceDepths = corner2faceDepths[corner];
      for (int face : corner2faces[corner]) {
        auto &plane = anno.face2plane[face];
        if (plane.normal != Origin()) {
          double depth =
              norm(Intersection(Ray3(Origin(), anno.corners[corner]), plane));
          faceDepths[face] = depth;
        }
      }

      if (faceDepths.empty()) {
        continue;
      }

      // cluster these depths
      std::vector<double> groupedDepths;
      std::map<int, int> face2group;

      for (auto &faceAndDepth : faceDepths) {
        int face = faceAndDepth.first;
        double depth = faceAndDepth.second;
        double minDepthDiff = mergeDepthsThres;
        for (int g = 0; g < groupedDepths.size();
             g++) { // find the group with min depth diff
          double depthDiff = abs(depth - groupedDepths[g]);
          if (depthDiff < minDepthDiff) {
            face2group[face] = g;
            minDepthDiff = depthDiff;
          }
        }
        if (!Contains(face2group,
                      face)) { // if not group is found, add a new group
          groupedDepths.push_back(depth);
          face2group[face] = groupedDepths.size() - 1;
        }
      }

      // update each face depth using group depth
      for (auto &faceAndDepth : faceDepths) {
        faceAndDepth.second = groupedDepths[face2group.at(faceAndDepth.first)];
      }
    }

    for (int face = 0; face < anno.nfaces(); face++) {
      auto &poly = face2polygon[face];
      poly.normal = anno.face2plane[face].normal;
      for (int corner : anno.face2corners[face]) {
        poly.corners.push_back(normalize(anno.corners[corner]) *
                               corner2faceDepths[corner].at(face));
      }
    }
  } else {
    for (int face = 0; face < anno.nfaces(); face++) {
      auto &poly = face2polygon[face];
      poly.normal = anno.face2plane[face].normal;
      for (int corner : anno.face2corners[face]) {
        poly.corners.push_back(Intersection(
            Ray3(Origin(), anno.corners[corner]), anno.face2plane[face]));
      }
    }
  }

  gui::SceneBuilder viz;
  viz.installingOptions().discretizeOptions.colorTable(
      gui::ColorTableDescriptor::RGB);
  std::vector<core::Decorated<gui::Colored<Polygon3>, int>> spps;
  // std::vector<core::Decorated<gui::Colored<Polygon3>, int>> pps;

  Image3ub reversedIm = anno.rectifiedImage.clone();
  ReverseRows(reversedIm);
  cv::cvtColor(reversedIm, reversedIm, CV_BGR2RGB);
  gui::ResourceStore::set("texture", reversedIm);
  for (int face = 0; face < anno.nfaces(); face++) {
    auto &polygon = face2polygon[face];
    if (polygon.normal == Origin() || polygon.corners.empty()) {
      continue;
    }

    static const gui::ColorTable rgbTable = gui::RGB;
    auto &control = anno.face2control[face];

    spps.push_back(core::DecorateAs(
        std::move(gui::ColorAs(polygon,
                               control.dof() == 1
                                   ? rgbTable[control.orientationClaz]
                                   : (control.dof() == 2
                                          ? rgbTable[control.orientationNotClaz]
                                          : gui::Color(gui::White)))),
        face));
  }

  viz.begin(spps /*, sppCallbackFun*/)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .resource("texture")
      .end();
  viz.show(true, false,
           gui::RenderOptions()
               .cullFrontFace(true)
               .cullBackFace(false)
               .bwColor(0.1)
               .bwTexColor(0.9)
               .camera(PerspectiveCamera(800, 800, Point2(250, 250), 500,
                                         Point3(1, 1, 1), Point3(0, 0, 0))));
}
}
}