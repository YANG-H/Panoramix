#pragma once

#include "basic_types.hpp"

namespace pano {
namespace core {

class PerspectiveCamera;
class PanoramicCamera;

// segmentation
class SegmentationExtractor {
public:
  using Feature = Imagei; // CV_32SC1
  enum Algorithm { GraphCut, SLIC, QuickShiftCPU, QuickShiftGPU };
  struct Params {
    inline Params()
        : sigma(0.8f), c(100.0f), minSize(200), algorithm(GraphCut),
          superpixelSizeSuggestion(1000), superpixelNumberSuggestion(100),
          useYUVColorSpace(false) {}
    float sigma; // for smoothing
    float c;     // threshold function
    int minSize; // min component size
    Algorithm algorithm;
    int superpixelSizeSuggestion;   // use superpixel size suggestion if
                                    // [superpixelSizeSuggestion > 0]
    int superpixelNumberSuggestion; // use superpixel number suggestion if
                                    // [superpixelSizeSuggestion < 0]
    bool useYUVColorSpace;
    template <class Archive> inline void serialize(Archive &ar) {
      ar(sigma, c, minSize, algorithm, superpixelSizeSuggestion,
         superpixelNumberSuggestion, useYUVColorSpace);
    }
  };

public:
  inline explicit SegmentationExtractor(const Params &params = Params())
      : _params(params) {}
  const Params &params() const { return _params; }
  Params &params() { return _params; }
  std::pair<Feature, int> operator()(const Image &im,
                                     bool isPanorama = false) const;
  std::pair<Feature, int> operator()(const Image &im,
                                     const std::vector<Line2> &lines,
                                     double extensionLength = 0.0) const;
  std::pair<Feature, int> operator()(const Image &im,
                                     const std::vector<Line3> &lines,
                                     const PanoramicCamera &cam,
                                     double extensionAngle = 0.0) const;
  template <class Archive> inline void serialize(Archive &ar) { ar(_params); }

private:
  Params _params;
};

// RemoveThinRegionInSegmentation
void RemoveThinRegionInSegmentation(Imagei &segs, int widthThres = 1.0,
                                    bool crossBorder = false);

// RemoveSmallRegionInSegmentation
int RemoveSmallRegionInSegmentation(Imagei &segs, double areaThres,
                                    bool panoWeights = false);

// RemoveDanglingPixelsInSegmentation
void RemoveDanglingPixelsInSegmentation(Imagei &segs, bool crossBorder = false);

// RemoveEmbededRegionsInSegmentation
void RemoveEmbededRegionsInSegmentation(Imagei &segs, bool crossBorder = false);

// DensifySegmentation
int DensifySegmentation(Imagei &segs, bool crossBorder = false);

// IsDenseSegmentation
bool IsDenseSegmentation(const Imagei &segRegions);

// FindRegionBoundaries
std::map<std::pair<int, int>, std::vector<std::vector<Pixel>>>
FindRegionBoundaries(const Imagei &segRegions, int connectionExtendSize,
                     bool simplifyStraightEdgePixels = true);

// ExtractBoundaryJunctions
std::vector<std::pair<std::vector<int>, Pixel>>
ExtractBoundaryJunctions(const Imagei &regions, bool crossBorder = false);

// ExtractSegmentationTopology
void ExtractSegmentationTopology(const Imagei &segs,
                                 std::vector<std::vector<Pixel>> &bndpixels,
                                 std::vector<Pixel> &juncpositions,
                                 std::vector<std::vector<int>> &seg2bnds,
                                 std::vector<std::pair<int, int>> &bnd2segs,
                                 std::vector<std::vector<int>> &seg2juncs,
                                 std::vector<std::vector<int>> &junc2segs,
                                 std::vector<std::pair<int, int>> &bnd2juncs,
                                 std::vector<std::vector<int>> &junc2bnds,
                                 bool crossBorder = false);

// SegmentationTopo
struct SegmentationTopo {
  std::vector<std::vector<Pixel>> bndpixels;
  std::vector<Pixel> juncpositions;
  std::vector<std::vector<int>> seg2bnds;
  std::vector<std::pair<int, int>> bnd2segs;
  std::vector<std::vector<int>> seg2juncs;
  std::vector<std::vector<int>> junc2segs;
  std::vector<std::pair<int, int>> bnd2juncs;
  std::vector<std::vector<int>> junc2bnds;

  SegmentationTopo() {}
  explicit SegmentationTopo(const Imagei &segs, bool corssBorder = false);

  size_t nboundaries() const { return bndpixels.size(); }
  size_t nsegs() const { return seg2bnds.size(); }
  size_t njunctions() const { return juncpositions.size(); }

  template <class Archiver> void serialize(Archiver &ar) {
    ar(bndpixels, juncpositions, seg2bnds, bnd2segs, seg2juncs, junc2segs,
       bnd2juncs, junc2bnds);
  }
};
}
}
