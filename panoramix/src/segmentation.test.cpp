#include "cameras.hpp"
#include "segmentation.hpp"
#include "utility.hpp"
#include "canvas.hpp"
#include "gui_util.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;


TEST(SegmentationTest, SegmentationExtractor) {
  core::Image3ub im = gui::FileDialog::PickAnImage();
  if (im.empty())
    return;

  core::ResizeToMakeHeightUnder(im, 600);
  {
    core::SegmentationExtractor::Params p;
    p.c = 5;
    p.minSize = 400;
    p.sigma = 1;
    core::SegmentationExtractor seg(p);
    gui::AsCanvas(im).show();
    auto segs = seg(im);
    gui::AsCanvas(gui::CreateRandomColorTableWithSize(segs.second)(segs.first))
        .show();
  }
  {
    core::SegmentationExtractor::Params p;
    p.c = 5;
    p.minSize = 400;
    p.sigma = 1;
    core::SegmentationExtractor seg(p);
    auto segs = seg(
        im, {core::Line2({0.0, 0.0}, core::Point2(im.cols, im.rows)),
             core::Line2(core::Point2(im.cols, 0), core::Point2(0, im.rows))});
    gui::AsCanvas(gui::CreateRandomColorTableWithSize(segs.second)(segs.first))
        .show();
  }
  {
    core::SegmentationExtractor::Params p;
    p.algorithm = core::SegmentationExtractor::SLIC;
    p.superpixelSizeSuggestion = 3000;
    core::SegmentationExtractor seg(p);
    gui::AsCanvas(im).show();
    auto segs = seg(im);
    gui::AsCanvas(gui::CreateRandomColorTableWithSize(segs.second)(segs.first))
        .show();
  }
}

TEST(SegmentationTest, SegmentationBoundaryJunction) {
  core::Image im =
      core::ImageRead(PANORAMIX_TEST_DATA_DIR_STR "/indoor_pano2.jpg");
  if (im.empty()) {
    return;
  }
  core::ResizeToMakeHeightUnder(im, 800);
  core::SegmentationExtractor::Params p;
  auto segs = core::SegmentationExtractor(p)(im, true);
  auto junctions = core::ExtractBoundaryJunctions(segs.first);
  std::vector<core::Pixel> ps;
  for (auto &j : junctions) {
    ps.insert(ps.end(), j.second);
  }
  auto ctable = gui::CreateRandomColorTableWithSize(segs.second);
  auto image = ctable(segs.first);
  gui::AsCanvas(image).color(gui::Black).add(ps).show();
}

TEST(SegmentationTest, RemoveSmallRegionInSegmentation) {
  core::Image3ub im = gui::FileDialog::PickAnImage(PANORAMIX_TEST_DATA_DIR_STR);
  if (!core::MayBeAPanorama(im)) {
    std::cerr << "this is not a panorama!" << std::endl;
    return;
  }
  core::ResizeToMakeHeightUnder(im, 800);
  core::SegmentationExtractor segmenter;
  segmenter.params().algorithm = core::SegmentationExtractor::GraphCut;
  segmenter.params().sigma = 10.0;
  segmenter.params().c = 1.0;
  segmenter.params().superpixelSizeSuggestion = 2000;
  core::Imagei segs;
  int nsegs = 0;
  std::tie(segs, nsegs) = segmenter(im, true);
  gui::AsCanvas(gui::CreateRandomColorTableWithSize(nsegs)(segs)).show();

  EXPECT_TRUE(core::IsDenseSegmentation(segs));
  nsegs = core::RemoveSmallRegionInSegmentation(segs, 1000, true);
  EXPECT_TRUE(core::IsDenseSegmentation(segs));

  gui::AsCanvas(gui::CreateRandomColorTableWithSize(nsegs)(segs)).show();
}

TEST(SegmentationTest, RemoveThinRegionInSegmentation) {
  core::Image3ub im = gui::FileDialog::PickAnImage(PANORAMIX_TEST_DATA_DIR_STR);
  if (!core::MayBeAPanorama(im)) {
    std::cerr << "this is not a panorama!" << std::endl;
    return;
  }
  core::ResizeToMakeHeightUnder(im, 800);
  core::SegmentationExtractor segmenter;
  segmenter.params().algorithm = core::SegmentationExtractor::GraphCut;
  segmenter.params().sigma = 10.0;
  segmenter.params().c = 1.0;
  segmenter.params().superpixelSizeSuggestion = 2000;
  core::Imagei segs;
  int nsegs = 0;
  std::tie(segs, nsegs) = segmenter(im, true);
  core::Imagei segs2 = segs.clone();
  core::RemoveThinRegionInSegmentation(segs2, 1, true);
  int nsegs2 = core::DensifySegmentation(segs2);
  EXPECT_TRUE(core::IsDenseSegmentation(segs2));
  gui::AsCanvas(gui::CreateRandomColorTableWithSize(nsegs)(segs)).show();
  gui::AsCanvas(gui::CreateRandomColorTableWithSize(nsegs2)(segs2)).show();
}

TEST(SegmentationTest, ExtractSegmentationTopology) {
  using namespace core;

  Image3ub im = gui::FileDialog::PickAnImage(PANORAMIX_TEST_DATA_DIR_STR);
  if (im.empty())
    return;

  core::ResizeToMakeHeightUnder(im, 600);

  core::SegmentationExtractor::Params p;
  p.c = 5;
  p.minSize = 400;
  p.sigma = 1;
  core::SegmentationExtractor seg(p);
  gui::AsCanvas(im).show();
  auto segs = seg(im, true);
  gui::AsCanvas(gui::CreateRandomColorTableWithSize(segs.second)(segs.first))
      .show();

  std::vector<std::vector<Pixel>> bndpixels;
  std::vector<Pixel> juncpositions;
  std::vector<std::vector<int>> seg2bnds;
  std::vector<std::pair<int, int>> bnd2segs;
  std::vector<std::vector<int>> seg2juncs;
  std::vector<std::vector<int>> junc2segs;
  std::vector<std::pair<int, int>> bnd2juncs;
  std::vector<std::vector<int>> junc2bnds;
  core::ExtractSegmentationTopology(segs.first, bndpixels, juncpositions,
                                    seg2bnds, bnd2segs, seg2juncs, junc2segs,
                                    bnd2juncs, junc2bnds, true);

  gui::MakeCanvas(gui::CreateRandomColorTableWithSize(segs.second)(segs.first))
      .color(gui::Red)
      .thickness(2)
      .add(bndpixels)
      .show();
}