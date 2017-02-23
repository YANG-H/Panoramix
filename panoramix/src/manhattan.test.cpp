#include "cameras.hpp"
#include "canvas.hpp"
#include "gui_util.hpp"
#include "manhattan.hpp"
#include "utility.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

TEST(ManhattanTest, VanishingPointsDetector) {
  auto imNames = {PANORAMIX_TEST_DATA_DIR_STR "/indoor_persp1.jpg",
                  PANORAMIX_TEST_DATA_DIR_STR "/indoor_persp2.jpg"};

  core::LineSegmentExtractor::Params lsParams;
  lsParams.minLength = 20;
  lsParams.xBorderWidth = lsParams.yBorderWidth = 20;
  core::LineSegmentExtractor lineseg(lsParams);
  core::VanishingPointsDetector vpdetector;

  std::vector<std::string> failedFileNames;

  for (auto &imName : imNames) {
    core::Image3ub im = core::ImageRead(imName);
    core::ResizeToMakeWidthUnder(im, 400);

    std::vector<core::HPoint2> vps;
    double focalLength;

    std::vector<core::Classified<core::Line2>> classifiedLines =
        core::ClassifyEachAs(lineseg(im, 3), -1);
    vpdetector.params().algorithm =
        core::VanishingPointsDetector::TardifSimplified;
    auto result = vpdetector(classifiedLines, im.size());
    if (result.null()) {
      std::cout << "failed to find vanishing points!" << std::endl;
      continue;
    }
    std::tie(vps, focalLength) = result.unwrap();

    std::vector<core::Classified<core::Ray2>> vpRays;
    for (int i = 0; i < 3; i++) {
      std::cout << "vp[" << i << "] = " << vps[i].value() << std::endl;
      for (double a = 0; a <= M_PI * 2.0; a += 0.1) {
        core::Point2 p = core::Point2(im.cols / 2, im.rows / 2) +
                         core::Vec2(cos(a), sin(a)) * 1000.0;
        vpRays.push_back(core::ClassifyAs(
            core::Ray2(p, (vps[i] - core::HPoint2(p, 1.0)).numerator), i));
      }
    }
    gui::AsCanvas(im)
        .colorTable(gui::ColorTable(gui::ColorTableDescriptor::RGB)
                        .appendRandomizedGreyColors(vps.size() - 3))
        .thickness(1)
        .add(vpRays)
        .thickness(2)
        .add(classifiedLines)
        .show(0);
  }

  for (auto &filename : failedFileNames) {
    std::cout << "failed file: " << filename << std::endl;
  }
}

TEST(ManhattanTest, LocalManhattanVanishingPointDetector) {
  using namespace core;

  // forged experiment for panorama
  core::Image3ub im =
      core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/indoor_pano1.jpg");
  if (im.empty()) {
    return;
  }

  core::ResizeToMakeWidthUnder(im, 2000);

  core::PanoramicCamera ocam(im.cols / M_PI / 2.0);
  core::PerspectiveCamera cam(800, 800, core::Point2(400, 400), ocam.focal(),
                              {0, 0, 0}, {-2, 0, -0.5}, {0, 0, -1});

  auto pim = core::MakeCameraSampler(cam, ocam)(im);

  core::LineSegmentExtractor lineseg;
  lineseg.params().minLength = 5;
  lineseg.params().algorithm = core::LineSegmentExtractor::LSD;
  std::vector<Line2> line2s = lineseg(pim);

  Vec3 vp1 = {0, 0, 1};
  std::vector<Classified<Line3>> line3s(line2s.size());
  std::vector<Vec3> line3norms(line2s.size());
  for (int i = 0; i < line2s.size(); i++) {
    line3s[i].component.first = normalize(cam.toSpace(line2s[i].first));
    line3s[i].component.second = normalize(cam.toSpace(line2s[i].second));
    line3norms[i] = line3s[i].component.first.cross(line3s[i].component.second);
    line3s[i].claz = abs(line3norms[i].dot(vp1)) < 0.006 ? 0 : -1;
  }

  std::vector<std::pair<int, int>> pairs;
  for (int i = 0; i < line2s.size(); i++) {
    if (line3s[i].claz == 0)
      continue;
    if (abs(line3norms[i].dot(vp1)) < 0.01)
      continue;
    for (int j = i + 1; j < line2s.size(); j++) {
      if (line3s[i].claz == 0)
        continue;
      if (abs(line3norms[j].dot(vp1)) < 0.01)
        continue;
      double dist = Distance(line2s[i], line2s[j]);
      auto &n1 = line3norms[i];
      auto &n2 = line3norms[j];
      auto inter = n1.cross(n2);
      auto interp = cam.toScreen(inter);
      double dd = 40;
      if (dist < dd && Distance(interp, line2s[i]) < dd &&
          Distance(interp, line2s[j]) < dd) {
        pairs.emplace_back(i, j);
      }
    }
  }

  std::vector<std::pair<int, int>> orthoPairs;
  for (auto &p : pairs) {
    auto &n1 = line3norms[p.first];
    auto &n2 = line3norms[p.second];
    auto p1 = normalize(n1.cross(vp1));
    auto p2 = normalize(n2.cross(vp1));
    if (abs(p1.dot(p2)) < 0.02)
      orthoPairs.push_back(p);
  }

  auto viz = gui::AsCanvas(pim).thickness(2);
  for (int i = 0; i < line2s.size(); i++) {
    if (line3s[i].claz == 0) {
      viz.color(gui::ColorTag::Red).add(line2s[i]);
    }
  }
  for (auto &op : orthoPairs) {
    auto &n1 = line3norms[op.first];
    auto &n2 = line3norms[op.second];
    auto inter = n1.cross(n2);
    auto interp = cam.toScreen(inter);
    viz.color(gui::ColorTag::LightGray)
        .thickness(1)
        .add(Line2(line2s[op.first].center(), interp))
        .add(Line2(line2s[op.second].center(), interp));
    viz.color(gui::ColorTag::White)
        .thickness(2)
        .add(line2s[op.first])
        .add(line2s[op.second]);
  }

  viz.show();
}