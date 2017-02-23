#pragma once

#include "file.hpp"
#include "clock.hpp"
#include "parallel.hpp"

#include "canvas.hpp"
#include "gui_util.hpp"
#include "ui.hpp"

#include "pi_graph_annotation.hpp"
#include "pi_graph_cg.hpp"
#include "pi_graph_control.hpp"
#include "pi_graph_occlusion.hpp"
#include "pi_graph_postprocess.hpp"
#include "pi_graph_vis.hpp"

using namespace pano;
using namespace pano::core;
using namespace pano::experimental;

// options
struct PanoramaReconstructionOptions {
  // algorithm options
  static const int LayoutVersion = 0;
  bool useWallPrior;
  bool usePrincipleDirectionPrior;
  bool useGeometricContextPrior;
  bool useGTOcclusions;
  bool looseLinesSecondTime;
  bool looseSegsSecondTime;
  bool restrictSegsSecondTime;
  bool notUseOcclusions;

  bool notUseCoplanarity;

  static const std::string parseOption(bool b);
  std::string algorithmOptionsTag() const;
  std::string identityOfImage(const std::string &impath) const;

  // cache options
  bool refresh_preparation;
  bool refresh_mg_init;
  bool refresh_line2leftRightSegs;
  bool refresh_mg_oriented;
  bool refresh_lsw;
  bool refresh_mg_occdetected;
  bool refresh_mg_reconstructed;

  // print options out
  void print() const;

  template <class Archiver> void serialize(Archiver &ar) {
    ar(useWallPrior, usePrincipleDirectionPrior, useGeometricContextPrior,
       useGTOcclusions, looseLinesSecondTime, looseSegsSecondTime,
       restrictSegsSecondTime, notUseOcclusions, notUseCoplanarity);
    ar(refresh_preparation, refresh_mg_init, refresh_line2leftRightSegs,
       refresh_mg_oriented, refresh_lsw, refresh_mg_occdetected,
       refresh_mg_reconstructed);
  }
};

// report
struct PanoramaReconstructionReport {
  double time_preparation;
  double time_mg_init;
  double time_line2leftRightSegs;
  double time_mg_oriented;
  double time_lsw;
  double time_mg_occdetected;
  double time_mg_reconstructed;

  double time_solve_lp;

  bool succeeded;

  PanoramaReconstructionReport();

  void print() const;

  template <class Archiver> void serialize(Archiver &ar) {
    ar(time_preparation, time_mg_init, time_line2leftRightSegs,
       time_mg_oriented, time_lsw, time_mg_occdetected, time_mg_reconstructed,
       time_solve_lp, succeeded);
  }
};

// run the main algorithm
PanoramaReconstructionReport
RunPanoramaReconstruction(const PILayoutAnnotation &anno,
                          const PanoramaReconstructionOptions &options,
                          misc::Matlab &matlab, bool showGUI,
                          bool writeToFile = false);

// get result
bool GetPanoramaReconstructionResult(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, PIGraph<PanoramicCamera> &mg,
    PIConstraintGraph &cg, PICGDeterminablePart &dp);
std::vector<LineSidingWeight> GetPanoramaReconstructionOcclusionResult(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options);

// save matlab results
void SaveMatlabResultsOfPanoramaReconstruction(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab,
    const std::string &fileName);

// save .obj model files
void SaveObjModelResultsOfPanoramaReconstruction(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab,
    const std::string &fileName);

// get surface normal maps
template <class CameraT>
std::vector<Image3d> GetSurfaceNormalMapsOfPanoramaReconstruction(
    const std::vector<CameraT> &testCams, const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab) {

  PIGraph<PanoramicCamera> mg;
  PIConstraintGraph cg;
  PICGDeterminablePart dp;
  if (!GetPanoramaReconstructionResult(anno, options, mg, cg, dp)) {
    std::cout << "failed to load panoramix result, performing "
                 "RunPanoramaReconstruction ..."
              << std::endl;
    RunPanoramaReconstruction(anno, options, matlab, false);
    GetPanoramaReconstructionResult(anno, options, mg, cg, dp);
  }

  std::vector<Image3d> surfaceNormalMaps(testCams.size());
  ParallelRun(
      testCams.size(), std::thread::hardware_concurrency() - 1, [&](int i) {
        std::cout
            << "computing surface normal map for panoramix on testCamera - "
            << i << std::endl;
        auto &map = surfaceNormalMaps[i];
        auto &cam = testCams[i];
        map = SurfaceNormalMap(cam, dp, cg, mg, true);
      });
  return surfaceNormalMaps;
}

// get surface depth maps
template <class CameraT>
std::vector<Imaged> GetSurfaceDepthMapsOfPanoramaReconstruction(
    const std::vector<CameraT> &testCams, const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab) {

  PIGraph<PanoramicCamera> mg;
  PIConstraintGraph cg;
  PICGDeterminablePart dp;
  if (!GetPanoramaReconstructionResult(anno, options, mg, cg, dp)) {
    std::cout << "failed to load panoramix result, performing "
                 "RunPanoramaReconstruction ..."
              << std::endl;
    RunPanoramaReconstruction(anno, options, matlab, false);
    GetPanoramaReconstructionResult(anno, options, mg, cg, dp);
  }

  std::vector<Imaged> surfaceDepthMaps(testCams.size());
  ParallelRun(
      testCams.size(), std::thread::hardware_concurrency() - 1, [&](int i) {
        std::cout
            << "computing surface normal map for panoramix on testCamera - "
            << i << std::endl;
        auto &map = surfaceDepthMaps[i];
        auto &cam = testCams[i];
        map = SurfaceDepthMap(cam, dp, cg, mg, true);
      });
  return surfaceDepthMaps;
}
