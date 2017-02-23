#include <chrono>

#include "geo_context.hpp"
#include "line_detection.hpp"
#include "panorama_reconstruction.hpp"
#include "segmentation.hpp"

template <class T> double ElapsedInMS(const T &start) {
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
             std::chrono::system_clock::now() - start)
      .count();
}

const std::string PanoramaReconstructionOptions::parseOption(bool b) {
  return b ? "_on" : "_off";
}

std::string PanoramaReconstructionOptions::algorithmOptionsTag() const {
  std::stringstream ss;
  ss << "_LayoutVersion" << LayoutVersion << parseOption(useWallPrior)
     << parseOption(usePrincipleDirectionPrior)
     << parseOption(useGeometricContextPrior) << parseOption(useGTOcclusions)
     << parseOption(looseLinesSecondTime) << parseOption(looseSegsSecondTime)
     << parseOption(restrictSegsSecondTime);
  if (notUseOcclusions) {
    ss << "noocc";
  }
  if (notUseCoplanarity) {
    ss << "_nocop";
  }
  return ss.str();
}

std::string PanoramaReconstructionOptions::identityOfImage(
    const std::string &impath) const {
  return impath + algorithmOptionsTag();
}

void PanoramaReconstructionOptions::print() const {
  std::cout << "##############################" << std::endl;
  std::cout << " useWallPrior = " << useWallPrior << std::endl;
  std::cout << " usePrincipleDirectionPrior = " << usePrincipleDirectionPrior
            << std::endl;
  std::cout << " useGeometricContextPrior = " << useGeometricContextPrior
            << std::endl;
  std::cout << " useGTOcclusions = " << useGTOcclusions << std::endl;
  std::cout << " looseLinesSecondTime = " << looseLinesSecondTime << std::endl;
  std::cout << " looseSegsSecondTime = " << looseSegsSecondTime << std::endl;
  std::cout << " restrictSegsSecondTime = " << restrictSegsSecondTime
            << std::endl;
  std::cout << " notUseOcclusions = " << notUseOcclusions << std::endl;
  std::cout << " notUseCoplanarity = " << notUseCoplanarity << std::endl;
  std::cout << "------------------------------" << std::endl;
  std::cout << " refresh_preparation = " << refresh_preparation << std::endl;
  std::cout << " refresh_mg_init = " << refresh_mg_init << std::endl;
  std::cout << " refresh_line2leftRightSegs = " << refresh_line2leftRightSegs
            << std::endl;
  std::cout << " refresh_mg_oriented = " << refresh_mg_oriented << std::endl;
  std::cout << " refresh_lsw = " << refresh_lsw << std::endl;
  std::cout << " refresh_mg_occdetected = " << refresh_mg_occdetected
            << std::endl;
  std::cout << " refresh_mg_reconstructed = " << refresh_mg_reconstructed
            << std::endl;
  std::cout << "##############################" << std::endl;
}

PanoramaReconstructionReport::PanoramaReconstructionReport() {
  time_preparation = -1;
  time_mg_init = -1;
  time_line2leftRightSegs = -1;
  time_mg_oriented = -1;
  time_lsw = -1;
  time_mg_occdetected = -1;
  time_mg_reconstructed = -1;
  succeeded = false;
}

void PanoramaReconstructionReport::print() const {
  std::cout << "##############################" << std::endl;
  std::cout << " time_preparation = " << time_preparation << std::endl;
  std::cout << " time_mg_init = " << time_mg_init << std::endl;
  std::cout << " time_line2leftRightSegs = " << time_line2leftRightSegs
            << std::endl;
  std::cout << " time_mg_oriented = " << time_mg_oriented << std::endl;
  std::cout << " time_lsw = " << time_lsw << std::endl;
  std::cout << " time_mg_occdetected = " << time_mg_occdetected << std::endl;
  std::cout << " time_mg_reconstructed = " << time_mg_reconstructed
            << std::endl;
  std::cout << "##############################" << std::endl;
}

static const double thetaTiny = DegreesToRadians(2);
static const double thetaMid = DegreesToRadians(5);
static const double thetaLarge = DegreesToRadians(15);

PanoramaReconstructionReport
RunPanoramaReconstruction(const PILayoutAnnotation &anno,
                          const PanoramaReconstructionOptions &options,
                          misc::Matlab &matlab, bool showGUI,
                          bool writeToFile) {

  PanoramaReconstructionReport report;
#define START_TIME_RECORD(name)                                                \
  auto start_##name = std::chrono::system_clock::now()

#define STOP_TIME_RECORD(name)                                                 \
  report.time_##name = ElapsedInMS(start_##name);                              \
  std::cout << "refresh_" #name " time cost: " << report.time_##name << "ms"   \
            << std::endl

  options.print();
  const auto identity = options.identityOfImage(anno.impath);
  misc::SaveCache(identity, "options", options);
  misc::SaveCache(identity, "report", report);

  const std::string folder = misc::FolderOfFile(anno.impath) + "\\images\\" +
                             misc::Tagify(identity) + "\\";

  if (writeToFile) {
    misc::MakeDir(folder);
  }
  if (writeToFile) {
    cv::imwrite(folder + "im.png", anno.view.image);
  }

  auto image = anno.rectifiedImage.clone();
  ResizeToHeight(image, 700);

  /// prepare things!
  View<PanoramicCamera, Image3ub> view;
  std::vector<PerspectiveCamera> cams;
  std::vector<std::vector<Classified<Line2>>> rawLine2s;
  std::vector<Classified<Line3>> line3s;
  std::vector<Vec3> vps;
  int vertVPId;
  Imagei segs;
  int nsegs;

  if (options.refresh_preparation ||
      !misc::LoadCache(identity, "preparation", view, cams, rawLine2s, line3s,
                       vps, vertVPId, segs, nsegs)) {
    START_TIME_RECORD(preparation);

    view = CreatePanoramicView(image);

    // collect lines in each view
    cams = CreateCubicFacedCameras(view.camera, image.rows, image.rows,
                                   image.rows * 0.4);
    std::vector<Line3> rawLine3s;
    rawLine2s.resize(cams.size());
    for (int i = 0; i < cams.size(); i++) {
      auto pim = view.sampled(cams[i]).image;
      LineSegmentExtractor lineExtractor;
      lineExtractor.params().algorithm = LineSegmentExtractor::LSD;
      auto ls = lineExtractor(pim); // use pyramid
      rawLine2s[i] = ClassifyEachAs(ls, -1);
      for (auto &l : ls) {
        rawLine3s.emplace_back(normalize(cams[i].toSpace(l.first)),
                               normalize(cams[i].toSpace(l.second)));
      }
    }
    rawLine3s = MergeLines(rawLine3s, DegreesToRadians(3), DegreesToRadians(5));

    // estimate vp
    line3s = ClassifyEachAs(rawLine3s, -1);
    vps = EstimateVanishingPointsAndClassifyLines(line3s, nullptr, true);
    vertVPId = NearestDirectionId(vps, Vec3(0, 0, 1));

    if (showGUI) {
      gui::ColorTable ctable = gui::RGBGreys;
      for (int i = 0; i < cams.size(); i++) {
        auto &cam = cams[i];
        std::vector<Classified<Line2>> lines;
        for (auto &l3 : line3s) {
          if (!cam.isVisibleOnScreen(l3.component.first) ||
              !cam.isVisibleOnScreen(l3.component.second)) {
            continue;
          }
          auto p1 = cam.toScreen(l3.component.first);
          auto p2 = cam.toScreen(l3.component.second);
          lines.push_back(ClassifyAs(Line2(p1, p2), l3.claz));
        }
        auto pim = view.sampled(cams[i]).image;
        gui::AsCanvas(pim).thickness(3).colorTable(ctable).add(lines).show();
      }
    }

    // estimate segs
    nsegs = SegmentationForPIGraph(view, line3s, segs, DegreesToRadians(1));
    RemoveThinRegionInSegmentation(segs, 1, true);
    RemoveEmbededRegionsInSegmentation(segs, true);
    nsegs = DensifySegmentation(segs, true);
    assert(IsDenseSegmentation(segs));

    if (showGUI) {
      auto ctable = gui::CreateGreyColorTableWithSize(nsegs);
      ctable.randomize();
      gui::ColorTable rgb = gui::RGBGreys;
      auto canvas = gui::MakeCanvas(view.image).alpha(0.9).add(ctable(segs));
      for (auto &l : line3s) {
        static const double sampleAngle = M_PI / 100.0;
        auto &line = l.component;
        double spanAngle = AngleBetweenDirected(line.first, line.second);
        std::vector<Point2> ps;
        ps.reserve(spanAngle / sampleAngle);
        for (double angle = 0.0; angle <= spanAngle; angle += sampleAngle) {
          Vec3 dir = RotateDirection(line.first, line.second, angle);
          ps.push_back(view.camera.toScreen(dir));
        }
        for (int i = 1; i < ps.size(); i++) {
          auto &p1 = ps[i - 1];
          auto &p2 = ps[i];
          if (Distance(p1, p2) >= view.image.cols / 2) {
            continue;
          }
          canvas.thickness(2);
          canvas.colorTable(rgb).add(gui::ClassifyAs(Line2(p1, p2), l.claz));
        }
      }
      canvas.show();
    }

    STOP_TIME_RECORD(preparation);

    // save
    misc::SaveCache(identity, "preparation", view, cams, rawLine2s, line3s, vps,
                    vertVPId, segs, nsegs);
  }

  // gc !!!!
  std::vector<PerspectiveCamera> hcams;
  std::vector<Weighted<View<PerspectiveCamera, Image5d>>> gcs;
  Image5d gc;
  static const int hcamNum = 16;
  static const Sizei hcamScreenSize(500, 500);
  // static const Sizei hcamScreenSize(500, 700);
  static const int hcamFocal = 200;
  std::string hcamsgcsFileName;
  {
    std::stringstream ss;
    ss << "hcamsgcs_" << hcamNum << "_" << hcamScreenSize.width << "_"
       << hcamScreenSize.height << "_" << hcamFocal;
    hcamsgcsFileName = ss.str();
  }
  if (0 || !misc::LoadCache(anno.impath, hcamsgcsFileName, hcams, gcs)) {
    // extract gcs
    hcams = CreateHorizontalPerspectiveCameras(
        view.camera, hcamNum, hcamScreenSize.width, hcamScreenSize.height,
        hcamFocal);
    gcs.resize(hcams.size());
    for (int i = 0; i < hcams.size(); i++) {
      auto pim = view.sampled(hcams[i]);
      auto pgc = ComputeIndoorGeometricContextHedau(matlab, pim.image);
      gcs[i].component.camera = hcams[i];
      gcs[i].component.image = pgc;
      gcs[i].score = abs(
          1.0 - normalize(hcams[i].forward()).dot(normalize(view.camera.up())));
    }
    misc::SaveCache(anno.impath, hcamsgcsFileName, hcams, gcs);
  }
  std::string gcmergedFileName;
  {
    std::stringstream ss;
    ss << "gc_" << hcamNum << "_" << hcamScreenSize.width << "_"
       << hcamScreenSize.height << "_" << hcamFocal;
    gcmergedFileName = ss.str();
  }
  if (0 || !misc::LoadCache(anno.impath, gcmergedFileName, gc)) {
    gc = Combine(view.camera, gcs).image;
    misc::SaveCache(anno.impath, gcmergedFileName, gc);
  }

  // build pigraph!
  PIGraph<PanoramicCamera> mg;
  if (options.refresh_mg_init || !misc::LoadCache(identity, "mg_init", mg)) {
    std::cout << "########## refreshing mg init ###########" << std::endl;
    START_TIME_RECORD(mg_init);
    mg = BuildPIGraph(view, vps, vertVPId, segs, line3s, DegreesToRadians(1),
                      DegreesToRadians(1), DegreesToRadians(1), thetaTiny,
                      thetaLarge, thetaTiny);
    STOP_TIME_RECORD(mg_init);
    misc::SaveCache(identity, "mg_init", mg);
  }

  std::vector<std::array<std::set<int>, 2>> line2leftRightSegs;
  if (options.refresh_line2leftRightSegs ||
      !misc::LoadCache(identity, "line2leftRightSegs", line2leftRightSegs)) {
    std::cout << "########## refreshing line2leftRightSegs ###########"
              << std::endl;
    START_TIME_RECORD(line2leftRightSegs);
    line2leftRightSegs = CollectSegsNearLines(mg, thetaMid * 2);
    STOP_TIME_RECORD(line2leftRightSegs);
    misc::SaveCache(identity, "line2leftRightSegs", line2leftRightSegs);
  }

  // attach orientation constraints
  if (options.refresh_mg_oriented ||
      !misc::LoadCache(identity, "mg_oriented", mg)) {
    std::cout << "########## refreshing mg oriented ###########" << std::endl;
    START_TIME_RECORD(mg_oriented);
    if (options.usePrincipleDirectionPrior) {
      AttachPrincipleDirectionConstraints(mg);
    }
    if (options.useWallPrior) {
      AttachWallConstraints(mg, thetaTiny);
    }
    if (options.useGeometricContextPrior) {
      AttachGCConstraints(mg, gc, 0.7, 0.7, true);
    }
    STOP_TIME_RECORD(mg_oriented);
    misc::SaveCache(identity, "mg_oriented", mg);
  }

  // detect occlusions
  std::vector<LineSidingWeight> lsw;
  if (options.refresh_lsw || !misc::LoadCache(identity, "lsw", lsw)) {
    std::cout << "########## refreshing lsw ###########" << std::endl;
    START_TIME_RECORD(lsw);
    if (options.notUseOcclusions) {
      lsw.resize(mg.nlines(), LineSidingWeight{0.5, 0.5});
    } else if (!options.useGTOcclusions) {
      lsw = ComputeLinesSidingWeights2(mg, DegreesToRadians(3), 0.2, 0.1,
                                       thetaMid);
    } else {
      lsw = ComputeLinesSidingWeightsFromAnnotation(
          mg, anno, DegreesToRadians(0.5), DegreesToRadians(8), 0.6);
    }
    STOP_TIME_RECORD(lsw);
    misc::SaveCache(identity, "lsw", lsw);
  }

  if (options.refresh_mg_occdetected ||
      !misc::LoadCache(identity, "mg_occdetected", mg)) {
    std::cout << "########## refreshing mg occdetected ###########"
              << std::endl;
    START_TIME_RECORD(mg_occdetected);
    ApplyLinesSidingWeights(mg, lsw, line2leftRightSegs, true);
    if (anno.extendedOnTop && !anno.topIsPlane) {
      DisableTopSeg(mg);
    }
    if (anno.extendedOnBottom && !anno.bottomIsPlane) {
      DisableBottomSeg(mg);
    }
    STOP_TIME_RECORD(mg_occdetected);
    misc::SaveCache(identity, "mg_occdetected", mg);
  }

  PIConstraintGraph cg;
  PICGDeterminablePart dp;
  if (options.refresh_mg_reconstructed ||
      !misc::LoadCache(identity, "mg_reconstructed", mg, cg, dp)) {
    std::cout << "########## refreshing mg reconstructed ###########"
              << std::endl;
    START_TIME_RECORD(mg_reconstructed);
    cg = BuildPIConstraintGraph(mg, DegreesToRadians(1), 0.01);

    dp = LocateDeterminablePart(cg, DegreesToRadians(3), false);
    auto start = std::chrono::system_clock::now();
    double energy = Solve(dp, cg, matlab, 5, 1e6, !options.notUseCoplanarity);
    report.time_solve_lp = ElapsedInMS(start);
    if (IsInfOrNaN(energy)) {
      std::cout << "solve failed" << std::endl;
      return report;
    }
    STOP_TIME_RECORD(mg_reconstructed);
    misc::SaveCache(identity, "mg_reconstructed", mg, cg, dp);
  }

  if (showGUI) {
    VisualizeReconstruction(dp, cg, mg, false,
                            [&cg, &mg](int ent) -> gui::Color {
                              auto &e = cg.entities[ent];
                              if (e.isSeg()) {
                                auto nn = normalize(
                                    e.supportingPlane.reconstructed.normal);
                                Vec3 n;
                                for (int i = 0; i < 3; i++) {
                                  n[i] = abs(nn.dot(normalize(mg.vps[i])));
                                }
                                gui::Color color = normalize(n);
                                return color;
                              } else {
                                return gui::Black;
                              }
                            },
                            nullptr, true);

    VisualizeReconstructionCompact(anno.rectifiedImage, dp, cg, mg, true,
                                   false);
  }

  report.succeeded = true;
  misc::SaveCache(identity, "report", report);

  return report;
}

bool GetPanoramaReconstructionResult(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, PIGraph<PanoramicCamera> &mg,
    PIConstraintGraph &cg, PICGDeterminablePart &dp) {
  auto identity = options.identityOfImage(anno.impath);
  return misc::LoadCache(identity, "mg_reconstructed", mg, cg, dp);
}

std::vector<LineSidingWeight> GetPanoramaReconstructionOcclusionResult(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options) {
  std::vector<LineSidingWeight> lsw;
  auto identity = options.identityOfImage(anno.impath);
  misc::LoadCache(identity, "lsw", lsw);
  return lsw;
}

void SaveMatlabResultsOfPanoramaReconstruction(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab,
    const std::string &fileName) {
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

  misc::MAT matFile(fileName, misc::MAT::Write);

  // segs
  matFile.setVar("segs", cv::InputArray(mg.segs + 1));
  auto planes = misc::MXA::createStructMatrix(mg.nsegs, 1,
                                              {"reconstructed", "plane_coeff"});
  for (int seg = 0; seg < mg.nsegs; seg++) {
    int ent = cg.seg2ent[seg];
    bool reconstructed = Contains(dp.determinableEnts, ent);
    planes.setField("reconstructed", seg, reconstructed);
    if (reconstructed) {
      planes.setField("plane_coeff", seg,
                      misc::MXA(Plane3ToEquation(
                          cg.entities[ent].supportingPlane.reconstructed)));
    }
  }
  matFile.setVar("planes", planes);

  // lines
  std::vector<Line3> reconstructedLines;
  for (int line = 0; line < mg.nlines(); line++) {
    int ent = cg.line2ent[line];
    bool reconstructed = Contains(dp.determinableEnts, ent);
    if (reconstructed) {
      auto &plane = cg.entities[ent].supportingPlane.reconstructed;
      double d1 =
          norm(IntersectionOfLineAndPlane(
                   Ray3(Origin(), mg.lines[line].component.first), plane)
                   .position);
      double d2 =
          norm(IntersectionOfLineAndPlane(
                   Ray3(Origin(), mg.lines[line].component.second), plane)
                   .position);
      reconstructedLines.push_back(
          Line3(d1 * normalize(mg.lines[line].component.first),
                d2 * normalize(mg.lines[line].component.second)));
    }
  }

  auto lines = misc::MXA::createStructMatrix(reconstructedLines.size(), 1,
                                             {"line_p1", "line_p2"});
  for (int i = 0; i < reconstructedLines.size(); i++) {
    lines.setField("line_p1", i, misc::MXA(reconstructedLines[i].first));
    lines.setField("line_p2", i, misc::MXA(reconstructedLines[i].second));
  }
  matFile.setVar("lines", lines);


  // depthMap
  Imaged depthMap(mg.segs.size(), 0.0);
  for (auto it = depthMap.begin(); it != depthMap.end(); ++it) {
    int seg = mg.segs(it.pos());
    int ent = cg.seg2ent[seg];
    bool reconstructed = Contains(dp.determinableEnts, ent);
    if (reconstructed) {
      auto & plane = cg.entities[ent].supportingPlane.reconstructed;
      Vec3 dir = mg.view.camera.direction(it.pos());
      double depth = norm(Intersection(Ray3(Origin(), dir), plane));
      *it = depth;
    }
  }
  matFile.setVar("depths", depthMap);
}

void SaveObjModelResultsOfPanoramaReconstruction(
    const PILayoutAnnotation &anno,
    const PanoramaReconstructionOptions &options, misc::Matlab &matlab,
    const std::string &fileName) {

  std::ofstream ofs(fileName);
  if (!ofs) {
    return;
  }

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

  auto compactPolygons = CompactModel(dp, cg, mg, 0.1);
  
  // write vertex
  std::vector<Point3> vertices;
  std::vector<std::vector<int>> faceInds;
  for (auto & poly : compactPolygons) {
    if (poly.corners.empty()) {
      continue;
    }
    int prevVertNum = vertices.size();
    vertices.insert(vertices.end(), poly.corners.begin(), poly.corners.end());
    faceInds.push_back(
        MakeIotaRange<int>(prevVertNum, prevVertNum + poly.corners.size())
            .evalAsStdVector());
  }

  for (auto & v : vertices) {
    ofs << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
  }
  for (auto & f : faceInds) {
    if (f.size() < 3) {
      continue;
    }
    ofs << "f ";
    for (int vid : f) {
      ofs << (vid + 1) << "/" << 1 << "/" << 1 << " ";
    }
    ofs << std::endl;
  }
}
