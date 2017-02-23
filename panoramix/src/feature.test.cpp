#include "cameras.hpp"
#include "line_detection.hpp"
#include "segmentation.hpp"
#include "utility.hpp"
#include "canvas.hpp"
#include "gui_util.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;


TEST(Feature, LineSegmentExtractor) {
  core::LineSegmentExtractor lineseg;
  core::Image3ub im =
      core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/building.jpg");
  if (im.empty()) {
    return;
  }
  core::LineSegmentExtractor::Params params;
  params.algorithm = core::LineSegmentExtractor::LSD;
  core::LineSegmentExtractor lineseg2(params);
  gui::AsCanvas(im)
      .color(gui::ColorTag::Yellow)
      .thickness(2)
      .add(lineseg2(im))
      .show();
}

TEST(Feature, FeatureExtractor) {
  core::SegmentationExtractor segmenter;
  core::LineSegmentExtractor::Params params;
  params.algorithm = core::LineSegmentExtractor::LSD;
  core::LineSegmentExtractor lineSegmentExtractor(params);

  for (int i = 0; i < 4; i++) {
    std::string name =
        PANORAMIX_TEST_DATA_DIR_STR "/sampled_" + std::to_string(i) + ".png";
    core::Image3ub im = core::ImageRead(name);
    if (im.empty()) {
      continue;
    }
    auto segs = segmenter(im);
    gui::AsCanvas(im)
        .colorTable(gui::CreateRandomColorTableWithSize(segs.second))
        .add(segs.first)
        .add(lineSegmentExtractor(im))
        .show();
  }
}