#include "panorama_reconstruction.hpp"

int main_run(int argc, char **argv) {
  gui::UI::InitGui(argc, argv);
  misc::SetCachePath(PANORAMIX_CACHE_DATA_DIR_STR "\\Panorama\\");
  pano::misc::MakeDir(pano::misc::CachePath());

  misc::Matlab matlab;

  std::vector<std::string> impaths;
  gui::FileDialog::PickImages(PANORAMIX_TEST_DATA_DIR_STR, &impaths);

  for (auto &&impath : impaths) {
    auto anno = LoadOrInitializeNewLayoutAnnotation(impath);
    anno.impath = impath;

    PanoramaReconstructionOptions options;
    options.useWallPrior = true;
    options.usePrincipleDirectionPrior = true;
    options.useGeometricContextPrior = true;

    options.useGTOcclusions = false;
    options.looseLinesSecondTime = false;
    options.looseSegsSecondTime = false;
    options.restrictSegsSecondTime = false;

    options.notUseOcclusions = false;
    options.notUseCoplanarity = false;

    options.refresh_preparation = false;
    options.refresh_mg_init = options.refresh_preparation || false;
    options.refresh_mg_oriented = options.refresh_mg_init || false;
    options.refresh_line2leftRightSegs = options.refresh_mg_init || false;
    options.refresh_lsw = options.refresh_mg_oriented || false;
    options.refresh_mg_occdetected =
        options.refresh_lsw || options.refresh_line2leftRightSegs || false;
    options.refresh_mg_reconstructed = options.refresh_mg_occdetected || false;

    RunPanoramaReconstruction(anno, options, matlab, true, false);

    SaveMatlabResultsOfPanoramaReconstruction(anno, options, matlab,
                                              impath + ".result.mat");
    SaveObjModelResultsOfPanoramaReconstruction(anno, options, matlab,
                                                impath + ".result.obj");
  }

  return 0;
}