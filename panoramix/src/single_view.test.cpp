#include "cameras.hpp"
#include "segmentation.hpp"
#include "single_view.hpp"
#include "utility.hpp"
#include "canvas.hpp"
#include "gui_util.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;
using namespace core;


TEST(SingleView, ComputeSpatialRegionProperties) {
  Image3ub im = gui::FileDialog::PickAnImage();
  SegmentationExtractor segmenter;
  segmenter.params().algorithm = SegmentationExtractor::GraphCut;

  Imagei segs;
  int nsegs;
  std::tie(segs, nsegs) = segmenter(im);

  auto result = CreatePerspectiveView(im);
  if (result.failed()) {
    std::cout << "failed creating perspective view from this image"
              << std::endl;
    return;
  }

  auto view = result.unwrap();

  std::vector<std::vector<std::vector<Vec3>>> contours;
  ComputeSpatialRegionProperties(segs, view.camera, &contours);
}
