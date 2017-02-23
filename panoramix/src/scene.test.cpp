#include "canvas.hpp"
#include "discretization.hpp"
#include "scene.hpp"
#include "gtest/gtest.h"

#include <iostream>
#include <random>
#include <string>

#include "../panoramix.unittest.hpp"

using namespace pano;


TEST(Canvas, Color) {
  int m = 600;
  core::Image_<core::Vec<double, 4>> hs(m, m);
  core::Image_<core::Vec<double, 4>> sv(m, m);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      hs(i, j) = gui::ColorFromHSV(i / (double)m, j / (double)m, 0.5);
      sv(i, j) = gui::ColorFromHSV(0.5, i / (double)m, j / (double)m);
    }
  }
  cv::imshow("[HS]V color", hs);
  cv::imshow("H[SV] color", sv);
  cv::waitKey();
}

static_assert(gui::IsDiscretizable<core::Line3>::value,
              "Line3 is not renderable!");
static_assert(gui::IsDiscretizable<core::Point3>::value,
              "Point3 is not renderable!");
static_assert(gui::IsDiscretizable<core::Classified<core::Line3>>::value,
              "Classified<Line3> is not renderable!");
static_assert(gui::IsDiscretizable<core::LayeredShape3>::value, "");
static_assert(!gui::IsDiscretizable<int>::value, "");

static auto a = [](gui::InteractionID iid, core::Sphere3 &s) {
  if (iid == gui::ClickLeftButton)
    std::cout << "clicked on the sphere" << std::endl;
};

static auto b = [](gui::InteractionID iid, const gui::SceneObjectTree &tree,
                   const gui::SceneObjectMeshTriangle &omt) {
  if (iid == gui::ClickLeftButton)
    std::cout << "clicked on the sphere" << std::endl;
};

void Print(const core::PerspectiveCamera &cam) {
  std::cout << std::setprecision(6)
            << "======================================" << std::endl;
  std::cout << std::setprecision(6) << "eye:\t" << cam.eye() << std::endl;
  std::cout << std::setprecision(6) << "center:\t" << cam.center() << std::endl;
  std::cout << std::setprecision(6) << "up:\t" << cam.up() << std::endl;
  std::cout << std::setprecision(6) << "size:\t" << cam.screenSize()
            << std::endl;
  std::cout << std::setprecision(6) << "focal:\t" << cam.focal() << std::endl;
  std::cout << std::setprecision(6) << "near:\t" << cam.nearPlane()
            << std::endl;
  std::cout << std::setprecision(6) << "far:\t" << cam.farPlane() << std::endl;
  std::cout << std::setprecision(6) << "viewMat:\t" << std::endl
            << cam.viewMatrix() << std::endl;
  std::cout << std::setprecision(6) << "projectMat:\t" << std::endl
            << cam.projectionMatrix() << std::endl;
  std::cout << std::setprecision(6) << "viewProjMat:\t" << std::endl
            << cam.viewProjectionMatrix() << std::endl;
  std::cout << std::setprecision(6)
            << "======================================" << std::endl;
}

TEST(Scene, LayeredShape3) {

  using namespace core;
  using namespace gui;

  core::LayeredShape3 ls;
  ls.layers = {
      {Point3(0, 0, 0), Point3(1, 0, 0), Point3(1, 1, 0), Point3(0, 1, 0)},
      {Point3(0, 0, 0.5), Point3(1, 0, 0.5), Point3(1, 1, 0.5),
       Point3(0, 1, 0.5)},
      {Point3(0, 0, 1) /*, Point3(1, 0, 1)*/, Point3(1, 1, 1),
       Point3(0, 1, 1)}};
  ls.normal = Vec3(0, 0, 1);

  auto tex = core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/indoor_pano1.jpg");
  if (tex.empty()) {
    return;
  }
  gui::ResourceStore::set("texture", tex);
  SceneBuilder()
      .begin(ls)
      .resource("texture")
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .end()
      .show(true, true,
            gui::RenderOptions().renderMode(gui::RenderModeFlag::Lines |
                                            gui::RenderModeFlag::Triangles));
}

TEST(Scene, Interaction) {

  using namespace gui;

  std::list<core::Sphere3> ss = {{core::Point3(1, 1, 1), 2.0},
                                 {core::Point3(-1, -1, -1), 2.0}};
  std::vector<gui::Colored<core::Line3>> xyz = {
      gui::ColorAs(core::Line3{core::Point3(), core::Point3(4, 0, 0)},
                   gui::Red),
      gui::ColorAs(core::Line3{core::Point3(), core::Point3(0, 4, 0)},
                   gui::Green),
      gui::ColorAs(core::Line3{core::Point3(), core::Point3(0, 0, 4)},
                   gui::Blue)};
  auto origin = gui::ColorAs(core::Point3(), gui::Black);

  auto tex = core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/indoor_pano1.jpg");
  if (tex.empty()) {
    return;
  }
  gui::ResourceStore::set("texture", tex);
  auto box = gui::MakeTransformableIn3D(core::BoundingBoxOfContainer(ss))
                 .rotate(core::Vec3(1, 0, 1), M_PI / 3);

  int clickedCount = 0;
  SceneBuilder sb;
  sb.begin(ss,
           [&clickedCount](gui::InteractionID iid, core::Sphere3 &s) {
             if (iid == gui::ClickLeftButton)
               std::cout << "clicked on the spheres, its center is at "
                         << s.center << std::endl;
             else
               std::cout << "pressed on the spheres, its center is at "
                         << s.center << std::endl;
           })
      .rotate(core::Vec3(0, 0, 1), M_PI_2)
      .resource("texture")
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .end();

  sb.begin(xyz)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XLines)
      .lineWidth(10)
      .end();

  sb.begin(origin)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPoints)
      .pointSize(40)
      .end();

  int n = 5;
  std::vector<gui::Colored<gui::TransformedIn3D<core::Box3>>> boxes;
  auto ctable = gui::CreateGreyColorTableWithSize(n + 1);
  for (int i = 0; i < n; i++) {
    boxes.push_back(gui::ColorAs(
        gui::MakeTransformableIn3D(core::BoundingBoxOfContainer(ss))
            .scale(0.1)
            .translate(core::Vec3(0, 3, 0))
            .rotate(core::Vec3(1, 0, 0), M_PI / n * i)
            .translate(core::Vec3(0, 0, -1)),
        ctable[i]));
    boxes.push_back(gui::ColorAs(
        gui::MakeTransformableIn3D(core::BoundingBoxOfContainer(ss))
            .scale(0.1)
            .translate(core::Vec3(0, 5, 0))
            .rotate(core::Vec3(1, 0, 0), M_PI / n * i)
            .translate(core::Vec3(0, 0, 1)),
        ctable[i]));
  }
  sb.begin(boxes).shaderSource(gui::OpenGLShaderSourceDescriptor::XLines).end();

  sb.show(true, true, gui::RenderOptions()
                          .renderMode(gui::RenderModeFlag::Lines |
                                      gui::RenderModeFlag::Triangles)
                          .cullBackFace(false)
                          .cullFrontFace(false));
}