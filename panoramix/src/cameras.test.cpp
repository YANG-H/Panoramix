#include "cameras.hpp"
#include "canvas.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

static_assert(core::IsCamera<core::PerspectiveCamera>::value, "");
static_assert(core::IsCamera<core::PanoramicCamera>::value, "");
static_assert(!core::IsCamera<core::Line3>::value, "");

TEST(Camera, PerspectiveCamera) {
  core::PerspectiveCamera cam(1000, 1000, core::Point2(500, 500), 500,
                              core::Vec3(5, 0, 0), core::Vec3(5, 5, 0));

  auto p = cam.toSpace(core::Point2(0, 500));
  auto d = core::normalize(p - cam.eye());
  ASSERT_TRUE(core::IsFuzzyParallel(d, core::Vec3(1, 1, 0)));
  p = cam.toSpace(core::Point2(500, 0));
  d = core::normalize(p - cam.eye());
  ASSERT_TRUE(core::IsFuzzyParallel(d, core::Vec3(0, 1, -1)));

  for (int i = 0; i < 100; i++) {
    core::Vec2 v(abs(rand()), abs(rand()));
    auto p = cam.toSpace(v);
    auto v2 = cam.toScreen(p);
    double dist = core::norm(v - v2);
    ASSERT_LT(dist, 0.01);
  }

  auto c = cam.toScreen(cam.center());
  double dist = core::norm(
      c - core::Vec2(cam.screenSize().width / 2, cam.screenSize().height / 2));
  ASSERT_LT(dist, 2);
}

TEST(Camera, PerspectiveCameraRandom) {
  for (int k = 0; k < 100; k++) {
    int w = abs(rand()) % 500;
    int h = abs(rand()) % 400;
    core::PerspectiveCamera cam(
        w, h, core::Point2(w, h) / 2.0, abs(rand()) % 600,
        core::Vec3(rand(), rand(), rand()), core::Vec3(rand(), rand(), rand()));
    for (int i = 0; i < 100; i++) {
      core::Vec2 v(rand(), rand());
      auto p = cam.toSpace(v);
      auto v2 = cam.toScreen(p);
      double dist = core::norm(v - v2);
      if (!std::isnan(dist) && !std::isinf(dist))
        ASSERT_LT(dist, 0.01);
    }
    auto c = cam.toScreen(cam.center());
    double dist = core::norm(c - core::Vec2(cam.screenSize().width / 2,
                                            cam.screenSize().height / 2));
    if (!std::isnan(dist) && !std::isinf(dist))
      ASSERT_LT(dist, 2);
  }
}

TEST(Camera, CameraSampler) {
  auto im = core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/indoor_pano1.jpg");
  if (im.empty()) {
    return;
  }

  EXPECT_EQ(2000, im.cols);
  EXPECT_EQ(1000, im.rows);
  cv::resize(im, im, cv::Size(1000, 500));
  auto c = gui::MakeCanvas(im);
  c.alpha(0.5);
  core::PanoramicCamera originCam(im.cols / M_PI / 2.0);
  core::PanoramicCamera newCam(im.cols / M_PI / 2.0, core::Vec3(0, 0, 0),
                               core::Vec3(0, 0, 1), core::Vec3(0, 1, 0));
  c.add(core::MakeCameraSampler(newCam, originCam)(im));
  c.show(false);

  core::PartialPanoramicCamera newCam2(newCam);
  gui::AsCanvas(core::MakeCameraSampler(newCam2, originCam)(im)).show();

  float camPositions[4][3] = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};

  auto panoView = core::CreatePanoramicView(im);
  std::vector<core::View<core::PerspectiveCamera, core::Image3ub>> perspViews;
  std::vector<core::View<core::PartialPanoramicCamera, core::Image3ub>>
      ppanoViews;

  for (int i = 0; i < 4; i++) {
    core::PerspectiveCamera cam(
        500, 600, core::Point2(250, 300), 150, core::Point3(0, 0, 0),
        core::Point3(camPositions[i][0], camPositions[i][1],
                     camPositions[i][2]),
        core::Vec3(0, 0, -1));
    auto v = panoView.sampled(cam);
    perspViews.push_back(std::move(v));
    gui::AsCanvas(perspViews.back().image).show();

    core::PartialPanoramicCamera cam2(500, 600, 250, core::Point3(0, 0, 0),
                                      core::Point3(camPositions[i][0],
                                                   camPositions[i][1],
                                                   camPositions[i][2]),
                                      panoView.camera.up());
    auto v2 = panoView.sampled(cam2);
    ppanoViews.push_back(std::move(v2));
    gui::AsCanvas(ppanoViews.back().image).show();
  }

  auto combined = core::Combine(panoView.camera, perspViews);
  gui::AsCanvas(combined.image).show();

  auto combined2 = core::Combine(panoView.camera, ppanoViews);
  gui::AsCanvas(combined2.image).show();
}