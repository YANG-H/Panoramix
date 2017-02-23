#include "pch.hpp"

#include <SLIC.h>

#include "cameras.hpp"
#include "containers.hpp"
#include "segmentation.hpp"
#include "utility.hpp"

namespace pano {
namespace core {

#pragma region SegmentationExtractor

namespace {

// the edge of graph
struct Edge {
  float w;
  int a, b;
};

// graph data
class Universe {
public:
  struct Element {
    int rank;
    int p; // parent
    float size;
  };
  explicit Universe(const std::vector<float> &eleSizes)
      : elements(eleSizes.size()), num(eleSizes.size()) {
    for (int i = 0; i < eleSizes.size(); i++) {
      elements[i].rank = 0;
      elements[i].size = eleSizes[i];
      elements[i].p = i;
    }
  }
  int find(int x) {
    int y = x;
    while (y != elements[y].p)
      y = elements[y].p;
    elements[x].p = y;
    return y;
  }
  void join(int x, int y) {
    if (elements[x].rank > elements[y].rank) {
      elements[y].p = x;
      elements[x].size += elements[y].size;
    } else {
      elements[x].p = y;
      elements[y].size += elements[x].size;
      if (elements[x].rank == elements[y].rank)
        elements[y].rank++;
    }
    num--;
  }
  float size(int x) const { return elements[x].size; }
  int numSets() const { return num; }

private:
  int num;
  std::vector<Element> elements;
};

inline float DefaultThreshold(float size, float c) {
  return size == 0.0 ? 1e8 : c / size;
}

// merge similar nodes in graph
template <class ThresholdFunT = decltype(DefaultThreshold)>
Universe SegmentGraph(const std::vector<float> &verticesSizes,
                      std::vector<Edge> &edges, float c,
                      ThresholdFunT &&thresholdFun = DefaultThreshold) {

  std::sort(edges.begin(), edges.end(),
            [](const Edge &e1, const Edge &e2) { return e1.w < e2.w; });

  int numVertices = verticesSizes.size();
  Universe u(verticesSizes);
  std::vector<float> threshold(numVertices);

  for (int i = 0; i < numVertices; i++)
    threshold[i] = thresholdFun(1, c);

  for (int i = 0; i < edges.size(); i++) {
    const Edge &edge = edges[i];

    // components conected by this edge
    int a = u.find(edge.a);
    int b = u.find(edge.b);
    if (a != b) {
      if ((edge.w <= threshold[a]) && (edge.w <= threshold[b])) {
        u.join(a, b);
        a = u.find(a);
        threshold[a] = edge.w + thresholdFun(u.size(a), c);
      }
    }
  }

  return u;
}

inline float ColorDistance(const Vec3 &a, const Vec3 &b, bool useYUV) {
  static const Mat3 RGB2YUV(0.299, 0.587, 0.114, -0.14713, -0.28886, 0.436,
                            0.615, -0.51499, -0.10001);
  static const Mat3 BGR2VUY = RGB2YUV.t();
  return useYUV ? norm(BGR2VUY * (a - b)) * 3.0 : norm(a - b);
}

// for edge weight computation
// measure pixel distance
inline float PixelDiff(const Image &im, const cv::Point &p1,
                       const cv::Point &p2, bool useYUV) {
  assert(im.depth() == CV_8U && im.channels() == 3);
  Vec3 c1 = im.at<cv::Vec<uint8_t, 3>>(p1);
  Vec3 c2 = im.at<cv::Vec<uint8_t, 3>>(p2);
  return ColorDistance(c1, c2, useYUV);
}

// measure pixel distance with 2d line cuttings
inline float PixelDiff(const Image &im, const Imagei &linesOccupation,
                       const Pixel &p1, const Pixel &p2,
                       const std::vector<Line2> &lines, bool useYUV) {

  assert(im.depth() == CV_8U && im.channels() == 3);
  for (int lineId : {linesOccupation(p1), linesOccupation(p2)}) {
    if (lineId >= 0) {
      auto &line = lines[lineId];
      double p1OnLeftFlag =
          (p1 - ToPixel(line.first)).cross(ToPixel(line.direction()));
      double p2OnLeftFlag =
          (p2 - ToPixel(line.first)).cross(ToPixel(line.direction()));
      if (p1OnLeftFlag * p2OnLeftFlag < 0) {
        return 1e5;
      }
    }
  }
  Vec3 c1 = im.at<cv::Vec<uint8_t, 3>>(p1);
  Vec3 c2 = im.at<cv::Vec<uint8_t, 3>>(p2);
  return ColorDistance(c1, c2, useYUV);
}

inline float PixelDiff(const Image &im, const Imagei &linesOccupation,
                       const Pixel &p1, const Pixel &p2,
                       const std::vector<Line3> &lines,
                       const PanoramicCamera &cam, bool useYUV) {

  assert(im.depth() == CV_8U && im.channels() == 3);
  auto direction1 = cam.toSpace(p1);
  auto direction2 = cam.toSpace(p2);
  for (int lineId : {linesOccupation(p1), linesOccupation(p2)}) {
    if (lineId >= 0) {
      auto &line = lines[lineId];
      auto lineNormal = line.first.cross(line.second);
      double p1OnLeftFlag = lineNormal.dot(direction1);
      double p2OnLeftFlag = lineNormal.dot(direction2);
      if (p1OnLeftFlag * p2OnLeftFlag < 0) {
        return 1e5;
      }
    }
  }
  Vec3 c1 = im.at<cv::Vec<uint8_t, 3>>(p1);
  Vec3 c2 = im.at<cv::Vec<uint8_t, 3>>(p2);
  return ColorDistance(c1, c2, useYUV);
}

template <class PixelDiffFuncT, class ImageT>
std::vector<Edge>
ComposeGraphEdges(int width, int height, bool isPanorama,
                  const ImageT &smoothed, const PixelDiffFuncT &pixelDiff,
                  bool useOnlyStraightConnectivity = false,
                  bool connectAllPolerPixelsIfIsPanorama = true) {

  std::vector<Edge> edges;
  edges.reserve(width * height * 4);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width - 1) {
        Edge edge;
        edge.a = y * width + x;
        edge.b = y * width + x + 1;
        edge.w = pixelDiff(smoothed, {x, y}, {(x + 1), y});
        edges.push_back(edge);
      }

      if (y < height - 1) {
        Edge edge;
        edge.a = y * width + x;
        edge.b = (y + 1) * width + x;
        edge.w = pixelDiff(smoothed, {x, y}, {x, (y + 1)});
        edges.push_back(edge);
      }

      if (!useOnlyStraightConnectivity) {
        if ((x < width - 1) && (y < height - 1)) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = (y + 1) * width + x + 1;
          edge.w = pixelDiff(smoothed, {x, y}, {(x + 1), (y + 1)});
          edges.push_back(edge);
        }

        if ((x < width - 1) && (y > 0)) {
          Edge edge;
          edge.a = y * width + x;
          edge.b = (y - 1) * width + (x + 1);
          edge.w = pixelDiff(smoothed, {x, y}, {(x + 1), (y - 1)});
          edges.push_back(edge);
        }
      }
    }
  }
  if (isPanorama) { // collect panorama borders
    for (int y = 0; y < height; y++) {
      Edge edge;
      edge.a = y * width + 0;
      edge.b = y * width + width - 1;
      edge.w = pixelDiff(smoothed, cv::Point{0, y}, cv::Point{width - 1, y});
      edges.push_back(edge);
      if (!useOnlyStraightConnectivity) {
        if (y < height - 1) {
          edge.b = (y + 1) * width + width - 1;
          edge.w =
              pixelDiff(smoothed, cv::Point{0, y}, cv::Point{width - 1, y + 1});
          edges.push_back(edge);
        }
        if (y > 0) {
          edge.b = (y - 1) * width + width - 1;
          edge.w =
              pixelDiff(smoothed, cv::Point{0, y}, cv::Point{width - 1, y - 1});
          edges.push_back(edge);
        }
      }
    }
    if (connectAllPolerPixelsIfIsPanorama) {
      for (int x1 = 0; x1 < width; x1++) {
        for (int x2 = x1; x2 < width; x2++) {
          Edge edge;
          edge.a = 0 * width + x1;
          edge.b = 0 * width + x2;
          edge.w = pixelDiff(smoothed, cv::Point{x1, 0}, cv::Point{x2, 0});
          edges.push_back(edge);
          edge.a = (height - 1) * width + x2;
          edge.b = (height - 1) * width + x1;
          edge.w = pixelDiff(smoothed, cv::Point{x2, height - 1},
                             cv::Point{x1, height - 1});
          edges.push_back(edge);
        }
      }
    }
  }
  return edges;
}

std::vector<float> ComposeGraphVerticesSizes(int width, int height,
                                             bool isPanorama) {
  if (!isPanorama) {
    return std::vector<float>(width * height, 1.0f);
  }
  std::vector<float> vsizes(width * height);
  float radius = width / 2.0 / M_PI;
  for (int y = 0; y < height; y++) {
    float longitude = (y - height / 2.0f) / radius;
    float scale = cos(longitude);
    if (scale < 0) {
      scale = 0.0;
    }
    std::fill_n(vsizes.data() + y * width, width, scale);
  }
  return vsizes;
}

std::pair<Imagei, Image>
PerformSegmentation(const std::vector<float> &verticesSizes,
                    std::vector<Edge> &edges, int width, int height,
                    float sigma, float c, int minSize, int &numCCs,
                    bool returnColoredResult = false) {

  int num = (int)edges.size();
  Universe u = SegmentGraph(verticesSizes, edges, c);

  // bool merged = true;
  // while (merged) {
  //    merged = false;
  for (int i = 0; i < num; i++) {
    int a = u.find(edges[i].a);
    int b = u.find(edges[i].b);
    if ((a != b) && ((u.size(a) < minSize) || (u.size(b) < minSize))) {
      u.join(a, b);
      // merged = true;
    }
  }
  //}

  numCCs = u.numSets();
  std::unordered_map<int, int> compIntSet;
  Imagei output(cv::Size2i(width, height));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u.find(y * width + x);
      if (compIntSet.find(comp) == compIntSet.end()) {
        compIntSet.insert(std::make_pair(comp, (int)compIntSet.size()));
      }
      output(cv::Point(x, y)) = compIntSet[comp];
    }
  }
  assert(compIntSet.size() == numCCs);

  if (!returnColoredResult) {
    return std::make_pair(output, Image());
  }

  Image coloredOutput(cv::Size2i(width, height), CV_8UC3);
  std::vector<cv::Vec<uint8_t, 3>> colors(numCCs);
  std::generate(colors.begin(), colors.end(), []() {
    return cv::Vec<uint8_t, 3>(uint8_t(std::rand() % 256),
                               uint8_t(std::rand() % 256),
                               uint8_t(std::rand() % 256));
  });
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      coloredOutput.at<cv::Vec<uint8_t, 3>>(cv::Point(x, y)) =
          colors[output(cv::Point(x, y))];
    }
  }
  return std::make_pair(output, coloredOutput);
}

// first return is CV_32SC1, the second is CV_8UC3 (for display)
std::pair<Imagei, Image> SegmentImage(const Image &im, float sigma, float c,
                                      int minSize, bool isPanorama, int &numCCs,
                                      bool returnColoredResult = false,
                                      bool useYUV = true) {

  assert(im.depth() == CV_8U && im.channels() == 3);

  int width = im.cols;
  int height = im.rows;
  Image smoothed;
  cv::GaussianBlur(im, smoothed, cv::Size(5, 5), sigma);

  // build pixel graph
  std::vector<float> vSizes =
      ComposeGraphVerticesSizes(width, height, isPanorama);
  // float(*pixelDiff)(const Image & im, const cv::Point & p1, const cv::Point &
  // p2, bool useYUV) = PixelDiff;
  std::vector<Edge> edges = ComposeGraphEdges(
      width, height, isPanorama, smoothed,
      [useYUV](const Image &im, const cv::Point &p1, const cv::Point &p2) {
        return PixelDiff(im, p1, p2, useYUV);
      });

  return PerformSegmentation(vSizes, edges, width, height, sigma, c, minSize,
                             numCCs, returnColoredResult);
}

// first return is CV_32SC1, the second is CV_8UC3 (for display)
std::pair<Imagei, Image>
SegmentImage(const Image &im, float sigma, float c, int minSize,
             const std::vector<Line2> &lines, int &numCCs,
             bool returnColoredResult = false, bool useYUV = true) {

  assert(im.depth() == CV_8U && im.channels() == 3);

  int width = im.cols;
  int height = im.rows;
  Image smoothed;
  cv::GaussianBlur(im, smoothed, cv::Size(5, 5), sigma);

  Imagei linesOccupation(im.size(), -1);
  for (int i = 0; i < lines.size(); i++) {
    auto &l = lines[i];
    cv::line(linesOccupation, ToPixel(l.first), ToPixel(l.second), i, 2);
  }

  // build pixel graph
  std::vector<float> vSizes = ComposeGraphVerticesSizes(width, height, false);
  std::vector<Edge> edges = ComposeGraphEdges(
      width, height, false, smoothed,
      [&linesOccupation, &lines, useYUV](const Image &im, const cv::Point &p1,
                                         const cv::Point &p2) {
        return PixelDiff(im, linesOccupation, p1, p2, lines, useYUV);
      });

  return PerformSegmentation(vSizes, edges, width, height, sigma, c, minSize,
                             numCCs, returnColoredResult);
}

// first return is CV_32SC1, the second is CV_8UC3 (for display)
std::pair<Imagei, Image> SegmentImage(const Image &im, float sigma, float c,
                                      int minSize,
                                      const std::vector<Line3> &lines,
                                      const PanoramicCamera &cam, int &numCCs,
                                      bool returnColoredResult = false,
                                      bool useYUV = true) {

  assert(im.depth() == CV_8U && im.channels() == 3);

  int width = im.cols;
  int height = im.rows;
  Image smoothed;
  cv::GaussianBlur(im, smoothed, cv::Size(5, 5), sigma);

  Imagei linesOccupation(im.size(), -1);
  for (int i = 0; i < lines.size(); i++) {
    auto &l = lines[i];
    double spanAngle = AngleBetweenDirected(l.first, l.second);
    std::vector<std::vector<Pixel>> pline(1);
    for (double a = 0.0; a <= spanAngle; a += 0.01) {
      auto direction = RotateDirection(l.first, l.second, a);
      pline.front().push_back(ToPixel(cam.toScreen(direction)));
    }
    cv::polylines(linesOccupation, pline, false, i, 2);
  }

  // build pixel graph
  std::vector<float> vSizes = ComposeGraphVerticesSizes(width, height, true);
  std::vector<Edge> edges = ComposeGraphEdges(
      width, height, true, smoothed,
      [&linesOccupation, &lines, &cam,
       useYUV](const Image &im, const cv::Point &p1, const cv::Point &p2) {
        return PixelDiff(im, linesOccupation, p1, p2, lines, cam, useYUV);
      });

  return PerformSegmentation(vSizes, edges, width, height, sigma, c, minSize,
                             numCCs, returnColoredResult);
}

std::pair<Imagei, int> SegmentImageUsingSLIC(const Image &im, int spsize,
                                             int spnum) {
  assert(im.depth() == CV_8U && im.channels() == 3);

  SLIC slic;
  unsigned int *ubuff = new unsigned int[im.cols * im.rows];
  // fill buffer
  for (int x = 0; x < im.cols; x++) {
    for (int y = 0; y < im.rows; y++) {
      auto &pixel = ubuff[y * im.cols + x];
      auto &color = im.at<Vec3ub>(Pixel(x, y));
      pixel = (color[2] << 16) + (color[1] << 8) + color[0];
    }
  }

  int *klabels = nullptr;
  int nlabels = 0;
  if (spsize > 0) {
    slic.DoSuperpixelSegmentation_ForGivenSuperpixelSize(
        ubuff, im.cols, im.rows, klabels, nlabels, spsize, 50);
  } else {
    slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(
        ubuff, im.cols, im.rows, klabels, nlabels, spnum, 50);
  }

  // fill labels back
  Imagei labels = Imagei::zeros(im.size());
  for (int x = 0; x < im.cols; x++) {
    for (int y = 0; y < im.rows; y++) {
      labels(Pixel(x, y)) = klabels[y * im.cols + x];
    }
  }

  delete[] ubuff;
  delete[] klabels;

  return std::make_pair(labels, nlabels);
}
}

#if 0
            //    inline void ToQuickShiftImage(const Image & IMG, image_t & im){
            //        im.N1 = IMG.rows;
            //        im.N2 = IMG.cols;
            //        im.K = IMG.channels();
            //        assert(im.K == 3);
            //        im.I = (float *)calloc(im.N1*im.N2*im.K, sizeof(float));
            //        for (int k = 0; k < im.K; k++)
            //        for (int col = 0; col < im.N2; col++)
            //        for (int row = 0; row < im.N1; row++)
            //        {
            //            auto & pt = IMG.at<cv::Vec<uint8_t, 3>>(/*im.N1 - 1 - row*/row, col);
            //            im.I[row + col*im.N1 + k*im.N1*im.N2] = 32. * pt[k] / 255.; // Scale 0-32
            //        }
            //    }

            //    namespace quickshift_details {

            //        int * map_to_flatmap(float * map, unsigned int size) {
            //            /********** Flatmap **********/
            //            int *flatmap = (int *)malloc(size*sizeof(int));
            //            for (unsigned int p = 0; p < size; p++)
            //            {
            //                flatmap[p] = map[p];
            //            }

            //            bool changed = true;
            //            while (changed)
            //            {
            //                changed = false;
            //                for (unsigned int p = 0; p < size; p++)
            //                {
            //                    changed = changed || (flatmap[p] != flatmap[flatmap[p]]);
            //                    flatmap[p] = flatmap[flatmap[p]];
            //                }
            //            }

            //            /* Consistency check */
            //            for (unsigned int p = 0; p < size; p++)
            //                assert(flatmap[p] == flatmap[flatmap[p]]);

            //            return flatmap;
            //        }

            //        image_t imseg(image_t im, int * flatmap) {
            //            /********** Mean Color **********/
            //            float * meancolor = (float *)calloc(im.N1*im.N2*im.K, sizeof(float));
            //            float * counts = (float *)calloc(im.N1*im.N2, sizeof(float));

            //            for (int p = 0; p < im.N1*im.N2; p++)
            //            {
            //                counts[flatmap[p]]++;
            //                for (int k = 0; k < im.K; k++)
            //                    meancolor[flatmap[p] + k*im.N1*im.N2] += im.I[p + k*im.N1*im.N2];
            //            }

            //            int roots = 0;
            //            for (int p = 0; p < im.N1*im.N2; p++)
            //            {
            //                if (flatmap[p] == p)
            //                    roots++;
            //            }
            //            printf("Roots: %d\n", roots);

            //            int nonzero = 0;
            //            for (int p = 0; p < im.N1*im.N2; p++)
            //            {
            //                if (counts[p] > 0)
            //                {
            //                    nonzero++;
            //                    for (int k = 0; k < im.K; k++)
            //                        meancolor[p + k*im.N1*im.N2] /= counts[p];
            //                }
            //            }
            //            if (roots != nonzero)
            //                printf("Nonzero: %d\n", nonzero);
            //            assert(roots == nonzero);


            //            /********** Create output image **********/
            //            image_t imout = im;
            //            imout.I = (float *)calloc(im.N1*im.N2*im.K, sizeof(float));
            //            for (int p = 0; p < im.N1*im.N2; p++)
            //            for (int k = 0; k < im.K; k++)
            //                imout.I[p + k*im.N1*im.N2] = meancolor[flatmap[p] + k*im.N1*im.N2];

            //            free(meancolor);
            //            free(counts);

            //            return imout;
            //        }

            //    }


            //    std::pair<Imagei, int> SegmentImageUsingQuickShiftCPU(const Image & originalIm, float sigma, float tau) {
            //        image_t im;
            //        ToQuickShiftImage(originalIm, im);
            //        float *map, *E, *gaps;
            //        int * flatmap;
            //        image_t imout;

            //        map = (float *)calloc(im.N1*im.N2, sizeof(float));
            //        gaps = (float *)calloc(im.N1*im.N2, sizeof(float));
            //        E = (float *)calloc(im.N1*im.N2, sizeof(float));

            //        quickshift(im, sigma, tau, map, gaps, E);

            //        /* Consistency check */
            //        for (int p = 0; p < im.N1*im.N2; p++)
            //        if (map[p] == p) assert(gaps[p] == INF);

            //        flatmap = quickshift_details::map_to_flatmap(map, im.N1*im.N2);
            //        imout = quickshift_details::imseg(im, flatmap);
            //        Imagei segmented(im.N1, im.N2);
            //        for (int col = 0; col < im.N2; col++)
            //        for (int row = 0; row < im.N1; row++)
            //        {
            //            segmented(row, col) = flatmap[row + col*im.N1];
            //        }
            //        int segnum = *std::max_element(flatmap, flatmap + im.N1 * im.N2) + 1;

            //        free(im.I);
            //        free(imout.I);
            //        free(map);
            //        free(gaps);
            //        free(E);
            //        free(flatmap);
            //        return std::make_pair(segmented, segnum);
            //    }

            //    std::pair<Imagei, int> SegmentImageUsingQuickShiftGPU(const Image & originalIm, float sigma, float tau) {
            //        image_t im;
            //        ToQuickShiftImage(originalIm, im);
            //        float *map, *E, *gaps;
            //        int * flatmap;
            //        image_t imout;

            //        map = (float *)calloc(im.N1*im.N2, sizeof(float));
            //        gaps = (float *)calloc(im.N1*im.N2, sizeof(float));
            //        E = (float *)calloc(im.N1*im.N2, sizeof(float));

            //        quickshift_gpu(im, sigma, tau, map, gaps, E);

            //        /* Consistency check */
            //        for (int p = 0; p < im.N1*im.N2; p++)
            //        if (map[p] == p) assert(gaps[p] == INF);

            //        flatmap = quickshift_details::map_to_flatmap(map, im.N1*im.N2);
            //        imout = quickshift_details::imseg(im, flatmap);
            //        Imagei segmented(im.N1, im.N2);
            //        for (int col = 0; col < im.N2; col++)
            //        for (int row = 0; row < im.N1; row++)
            //        {
            //            segmented(row, col) = flatmap[row + col*im.N1];
            //        }
            //        int segnum = *std::max_element(flatmap, flatmap + im.N1 * im.N2) + 1;

            //        free(im.I);
            //        free(imout.I);
            //        free(map);
            //        free(gaps);
            //        free(E);
            //        free(flatmap);
            //        return std::make_pair(segmented, segnum);
            //    }

            //}
#endif

std::pair<Imagei, int> SegmentationExtractor::
operator()(const Image &im, bool isPanorama) const {
  if (_params.algorithm == SLIC) {
    assert(!isPanorama);
    return SegmentImageUsingSLIC(im, _params.superpixelSizeSuggestion,
                                 _params.superpixelNumberSuggestion);
  } else if (_params.algorithm == GraphCut) {
    int numCCs;
    Imagei segim =
        SegmentImage(im, _params.sigma, _params.c, _params.minSize, isPanorama,
                     numCCs, false, _params.useYUVColorSpace)
            .first;
    return std::make_pair(segim, numCCs);
  } else if (_params.algorithm == QuickShiftCPU) {
    assert(!isPanorama);
    NOT_IMPLEMENTED_YET();
    // return SegmentImageUsingQuickShiftCPU(im, 6, 10);
  } else if (_params.algorithm == QuickShiftGPU) {
    assert(!isPanorama);
    NOT_IMPLEMENTED_YET();
    // return SegmentImageUsingQuickShiftGPU(im, 6, 10);
  } else {
    SHOULD_NEVER_BE_CALLED();
  }
}

std::pair<Imagei, int> SegmentationExtractor::
operator()(const Image &im, const std::vector<Line2> &lines,
           double extensionLength) const {
  assert(_params.algorithm == GraphCut);
  int numCCs;
  if (extensionLength == 0.0) {
    Imagei segim = SegmentImage(im, _params.sigma, _params.c, _params.minSize,
                                lines, numCCs, false, _params.useYUVColorSpace)
                       .first;
    return std::make_pair(segim, numCCs);
  } else {
    auto extLines = lines;
    for (auto &line : extLines) {
      auto d = normalize(line.direction());
      line.first -= (d * extensionLength);
      line.second += (d * extensionLength);
    }
    Imagei segim =
        SegmentImage(im, _params.sigma, _params.c, _params.minSize, extLines,
                     numCCs, false, _params.useYUVColorSpace)
            .first;
    return std::make_pair(segim, numCCs);
  }
}

std::pair<Imagei, int> SegmentationExtractor::
operator()(const Image &im, const std::vector<Line3> &lines,
           const PanoramicCamera &cam, double extensionAngle) const {
  assert(_params.algorithm == GraphCut);
  int numCCs;
  if (extensionAngle == 0.0) {
    Imagei segim =
        SegmentImage(im, _params.sigma, _params.c, _params.minSize, lines, cam,
                     numCCs, false, _params.useYUVColorSpace)
            .first;
    return std::make_pair(segim, numCCs);
  } else {
    auto extLines = lines;
    for (auto &line : extLines) {
      auto p1 = RotateDirection(line.first, line.second, -extensionAngle);
      auto p2 = RotateDirection(line.second, line.first, -extensionAngle);
      line.first = p1;
      line.second = p2;
    }
    Imagei segim =
        SegmentImage(im, _params.sigma, _params.c, _params.minSize, extLines,
                     cam, numCCs, false, _params.useYUVColorSpace)
            .first;
    return std::make_pair(segim, numCCs);
  }
}

#pragma endregion SegmentationExtractor

void RemoveThinRegionInSegmentation(Imagei &segs, int widthThres /*= 2.0*/,
                                    bool crossBorder /*= false*/) {
  /*if (crossBorder) {
      THERE_ARE_BUGS_HERE("CrossBorder should not be applied to top/bottom
  borders!");
  }*/

  int width = segs.cols, height = segs.rows;

  Imageb insiders(segs.size(), false);
  for (auto it = insiders.begin(); it != insiders.end(); ++it) {
    auto p = it.pos();
    int seg = segs(p);
    bool isInside = true;
    for (int x = -widthThres; x <= widthThres; x++) {
      if (!isInside) {
        break;
      }
      int xx = p.x + x;
      if ((xx < 0 || xx > width - 1) && !crossBorder) {
        continue;
      }
      xx = (xx + width) % width;
      for (int y = -widthThres; y <= widthThres; y++) {
        if (!isInside) {
          break;
        }
        int yy = p.y + y;
        if (yy < 0 || yy > height - 1) {
          continue;
        }
        if (segs(yy, xx) != seg) {
          isInside = false;
        }
      }
    }
    *it = isInside;
  }

  /*cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(2 * widthThres + 1, 2 * widthThres + 1),
      cv::Point(widthThres, widthThres));
  cv::erode(segregions, segregions, element);*/

  DenseMatd precompte(widthThres * 2 + 1, widthThres * 2 + 1, 0.0);
  for (int x = -widthThres; x <= widthThres; x++) {
    for (int y = -widthThres; y <= widthThres; y++) {
      precompte(x + widthThres, y + widthThres) = sqrt(x * x + y * y);
    }
  }

  std::vector<std::map<int, double>> distanceTable(width * height);
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();
    if (insiders(p)) // consider near-boundary pixels only
      continue;
    auto &dtable = distanceTable[p.x * height + p.y];
    for (int x = -widthThres; x <= widthThres; x++) {
      for (int y = -widthThres; y <= widthThres; y++) {
        int xx = p.x + x;
        if ((xx < 0 || xx > width - 1) && !crossBorder) {
          continue;
        }
        xx = (xx + width) % width;
        int yy = p.y + y;
        if (yy < 0 || yy > height - 1) {
          continue;
        }
        Pixel curp(xx, yy);
        double distance = precompte(x + widthThres, y + widthThres);
        if (insiders(curp)) { // curp is at certain regions
          int segid = segs(curp);
          if (!Contains(dtable, segid) || dtable.at(segid) > distance) {
            dtable[segid] = distance;
          }
        } else {
          auto &curdtable = distanceTable[curp.x * height + curp.y];
          // use dtable to update curdtable
          for (auto &dd : dtable) {
            if (!Contains(curdtable, dd.first) ||
                curdtable.at(dd.first) > dd.second + distance) {
              curdtable[dd.first] = dd.second + distance;
            }
          }
          // use curdtable to update dtable
          for (auto &dd : curdtable) {
            if (!Contains(dtable, dd.first) ||
                dtable.at(dd.first) > dd.second + distance) {
              dtable[dd.first] = dd.second + distance;
            }
          }
        }
      }
    }
  }

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int id = x * height + y;
      if (distanceTable[id].empty()) {
        continue;
      }
      int minDistSegId = -1;
      double minDist = std::numeric_limits<double>::infinity();
      for (auto &dd : distanceTable[id]) {
        if (dd.second < minDist) {
          minDistSegId = dd.first;
          minDist = dd.second;
        }
      }
      assert(minDistSegId != -1);
      segs(y, x) = minDistSegId;
    }
  }
}

int RemoveSmallRegionInSegmentation(Imagei &segs, double areaThres,
                                    bool panoWeights) {

  int nsegs = MinMaxValOfImage(segs).second + 1;

  std::vector<Scored<int>> segAreas(nsegs);
  for (int i = 0; i < nsegs; i++) {
    segAreas[i].component = i;
    segAreas[i].score = 0;
  }

  int height = segs.rows;
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    double weight = 1.0;
    if (panoWeights) {
      weight = cos((it.pos().y - height / 2.0) / height * M_PI);
    }
    segAreas[*it].score -= weight; // record negative areas
  }

  std::vector<std::map<int, double>> segAdjacents(nsegs);
  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();

    // seg ids related
    std::set<int> idset = {segs(p), segs(Pixel((p.x + 1) % segs.cols, p.y))};
    if (p.y <
        height - 1) { // note that the top/bottom borders cannot be crossed!
      idset.insert(segs(Pixel(p.x, p.y + 1)));
      idset.insert(segs(Pixel((p.x + 1) % segs.cols, p.y + 1)));
    }

    if (idset.size() <= 1) {
      continue;
    }

    double weight = cos((it.pos().y - height / 2.0) / height * M_PI);
    ;

    // register this pixel as a bnd candidate for bnd of related segids
    for (auto ii = idset.begin(); ii != idset.end(); ++ii) {
      for (auto jj = std::next(ii); jj != idset.end(); ++jj) {
        segAdjacents[*ii][*jj] += weight;
        segAdjacents[*jj][*ii] += weight;
      }
    }
  }

  std::vector<int> segParent(nsegs);
  std::iota(segParent.begin(), segParent.end(), 0);

  // record current root segs
  MaxHeap<int, double> rootSegs(segAreas.begin(), segAreas.end());

  while (-rootSegs.topScore() <= areaThres) {
    int smallestRootSeg = rootSegs.top();
    assert(segParent[smallestRootSeg] == smallestRootSeg);

    double smallestArea = -rootSegs.topScore();
    assert(smallestArea > 0);

    MaxHeap<int, double> adjacentRootSegs;
    for (auto &adj : segAdjacents[smallestRootSeg]) {
      int adjSeg = adj.first;
      // find its root seg
      while (adjSeg != segParent[adjSeg]) {
        adjSeg = segParent[adjSeg];
      }
      assert(rootSegs.contains(adjSeg));
      if (adjSeg == smallestRootSeg) {
        continue;
      }
      if (!adjacentRootSegs.contains(adjSeg)) {
        adjacentRootSegs.set(adjSeg, adj.second);
      } else {
        adjacentRootSegs.set(adjSeg, adjacentRootSegs.at(adjSeg) + adj.second);
      }
    }
    if (adjacentRootSegs.empty()) {
      rootSegs.set(smallestRootSeg, -areaThres - 1);
      continue;
    }

    int rootSegForMerging = adjacentRootSegs.top();
    assert(segParent[rootSegForMerging] == rootSegForMerging);

    // merge!
    double newNegativeArea = rootSegs.at(rootSegForMerging) - smallestArea;
    segParent[smallestRootSeg] = rootSegForMerging;
    rootSegs.pop();
    rootSegs.set(rootSegForMerging, newNegativeArea);
  }

  std::map<int, int> root2new;
  for (auto &root : rootSegs) {
    root2new[root.component] = root2new.size();
  }

  std::vector<int> old2new(nsegs);
  for (int i = 0; i < nsegs; i++) {
    int seg = i;
    while (segParent[seg] != seg) {
      seg = segParent[seg];
    }
    old2new[i] = root2new.at(seg);
  }

  for (int &seg : segs) {
    seg = old2new[seg];
  }

  return rootSegs.size();
}

void RemoveDanglingPixelsInSegmentation(Imagei &segs, bool crossBorder) {

  int width = segs.cols;
  int height = segs.rows;

  while (true) {
    bool hasDanglingPixels = false;
    for (auto it = segs.begin(); it != segs.end(); ++it) {
      int seg = *it;
      std::vector<int> neighbors;
      neighbors.reserve(4);
      auto p = it.pos();
      if (p.x > 0 || crossBorder) {
        neighbors.push_back(segs(Pixel((p.x + width - 1) % width, p.y)));
      }
      if (p.x < width - 1 || crossBorder) {
        neighbors.push_back(segs(Pixel((p.x + 1) % width, p.y)));
      }
      if (p.y > 0) {
        neighbors.push_back(segs(Pixel(p.x, p.y - 1)));
      }
      if (p.y < height - 1) {
        neighbors.push_back(segs(Pixel(p.x, p.y + 1)));
      }
      bool isDanglingPixel = true;
      for (int nb : neighbors) {
        if (nb == seg) {
          isDanglingPixel = false;
          break;
        }
      }
      if (isDanglingPixel) {
        int maxCountNeighbor = *std::max_element(
            neighbors.begin(), neighbors.end(), [&neighbors](int nb1, int nb2) {
              return std::count(neighbors.begin(), neighbors.end(), nb1) <
                     std::count(neighbors.begin(), neighbors.end(), nb2);
            });
        *it = maxCountNeighbor;
        hasDanglingPixels = true;
      }
    }
    if (!hasDanglingPixels) {
      break;
    }
  }
}

void RemoveEmbededRegionsInSegmentation(Imagei &segs, bool crossBorder) {
  int nsegs = MinMaxValOfImage(segs).second + 1;
  std::vector<std::set<int>> segNeighbors(nsegs);
  for (int x = 0; x < segs.cols; x++) {
    for (int y = 0; y < segs.rows; y++) {
      Pixel p(x, y);
      int seg1 = segs(p);
      Pixel nbs[] = {Pixel(x + 1, y), Pixel(x, y + 1), Pixel(x + 1, y + 1),
                     Pixel(x - 1, y + 1)};
      for (auto &nb : nbs) {
        if (crossBorder) {
          nb.x = (nb.x + segs.cols) % segs.cols;
        }
        if (Contains(segs.size(), nb)) {
          int seg2 = segs(nb);
          segNeighbors[seg1].insert(seg2);
          segNeighbors[seg2].insert(seg1);
        }
      }
    }
  }
  for (int &s : segs) {
    if (segNeighbors[s].size() == 1) {
      s = *segNeighbors[s].begin();
    }
  }
}

int DensifySegmentation(Imagei &segs, bool crossBorder) {

  int width = segs.cols;
  int height = segs.rows;

  // build pixel graph
  std::vector<float> vSizes =
      ComposeGraphVerticesSizes(width, height, crossBorder);
  std::vector<Edge> edges = ComposeGraphEdges(
      width, height, crossBorder, segs,
      [](const Imagei &im, const cv::Point &p1, const cv::Point &p2) {
        return im(p1) == im(p2) ? 0 : 1e5;
      },
      true, false);

  int num = (int)edges.size();
  Universe u = SegmentGraph(vSizes, edges, 0.1);

  for (int i = 0; i < num; i++) {
    int a = u.find(edges[i].a);
    int b = u.find(edges[i].b);
    if (a != b && edges[i].w == 0)
      u.join(a, b);
  }

  int numCCs = u.numSets();
  std::unordered_map<int, int> compIntSet;
  Imagei output(cv::Size2i(width, height));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u.find(y * width + x);
      if (compIntSet.find(comp) == compIntSet.end()) {
        compIntSet.insert(std::make_pair(comp, (int)compIntSet.size()));
      }
      output(cv::Point(x, y)) = compIntSet[comp];
    }
  }
  assert(compIntSet.size() == numCCs);

  segs = output;
  return numCCs;
}

bool IsDenseSegmentation(const Imagei &segRegions) {
  int minv, maxv;
  std::tie(minv, maxv) = MinMaxValOfImage(segRegions);
  if (minv != 0)
    return false;
  std::vector<bool> stored(maxv + 1, false);
  for (int i : segRegions) {
    stored[i] = true;
  }
  for (auto b : stored) {
    if (!b)
      return false;
  }
  return true;
}

std::map<std::pair<int, int>, std::vector<std::vector<Pixel>>>
FindRegionBoundaries(const Imagei &segRegions, int connectionExtendSize,
                     bool simplifyStraightEdgePixels) {

  std::map<std::pair<int, int>, std::vector<std::vector<Pixel>>> boundaryEdges;
  std::map<std::pair<int, int>, std::set<Pixel>> boundaryPixels;

  int width = segRegions.cols;
  int height = segRegions.rows;
  for (int y = 0; y < height - 1; y++) {
    for (int x = 0; x < width - 1; x++) {
      int originalRegionId = segRegions(Pixel(x, y));
      for (int xx = std::max(x - connectionExtendSize, 0);
           xx <= std::min(x + connectionExtendSize, width - 1); xx++) {
        for (int yy = std::max(y - connectionExtendSize, 0);
             yy <= std::min(y + connectionExtendSize, height - 1); yy++) {
          int regionIdHere = segRegions(Pixel(xx, yy));
          if (originalRegionId != regionIdHere) {
            boundaryPixels[MakeOrderedPair(originalRegionId, regionIdHere)]
                .insert(Pixel((x + xx) / 2, (y + yy) / 2));
          }
        }
      }
    }
  }

  for (auto &bpp : boundaryPixels) {
    int rid1 = bpp.first.first;
    int rid2 = bpp.first.second;
    auto &pixels = bpp.second;

    if (pixels.empty())
      continue;

    Pixel p = *pixels.begin();

    static const int xdirs[] = {1, 0, -1, 0, -1, 1, 1, -1, 0, 0, 2, -2};
    static const int ydirs[] = {0, 1, 0, -1, 1, -1, 1, -1, 2, -2, 0, 0};

    if (connectionExtendSize > 2) {
      IMPROVABLE_HERE("what if connectionExtendSize is too large? will it "
                      "cause bugs here searching edges?");
    }

    std::vector<std::vector<Pixel>> edges;
    edges.push_back({p});
    pixels.erase(p);

    while (true) {
      auto &curEdge = edges.back();
      auto &curTail = curEdge.back();

      bool foundMore = false;
      for (int i = 0; i < std::distance(std::begin(xdirs), std::end(xdirs));
           i++) {
        Pixel next = curTail;
        next.x += xdirs[i];
        next.y += ydirs[i];
        if (!IsBetween(next.x, 0, width) || !IsBetween(next.y, 0, height))
          continue;
        if (pixels.find(next) ==
            pixels.end()) // not a boundary pixel or already recorded
          continue;

        curEdge.push_back(next);
        pixels.erase(next);
        foundMore = true;
        break;
      }

      if (!foundMore) {
        // simplify current edge
        if (edges.back().size() <= 1) {
          edges.pop_back();
        } else {
          if (simplifyStraightEdgePixels) {
            bool closed =
                Distance(edges.back().front(), edges.back().back()) <= 1.5;
            cv::approxPolyDP(edges.back(), edges.back(), 2, closed);
          }
          if (edges.back().size() <= 1)
            edges.pop_back();
        }

        if (pixels.empty()) { // no more pixels
          break;
        } else { // more pixels
          Pixel p = *pixels.begin();
          edges.push_back({p});
          pixels.erase(p);
        }
      }
    }

    if (!edges.empty()) {
      boundaryEdges[MakeOrderedPair(rid1, rid2)] = edges;
    }
  }

  return boundaryEdges;
}

std::vector<std::pair<std::vector<int>, Pixel>>
ExtractBoundaryJunctions(const Imagei &regions, bool crossBorder) {
  std::vector<std::pair<std::vector<int>, Pixel>> junctions;
  for (auto it = regions.begin(); it != regions.end(); ++it) {
    auto p = it.pos();
    if (p.x == regions.cols - 1 && !crossBorder)
      continue;
    if (p.y == regions.rows - 1 && !crossBorder)
      continue;
    std::vector<int> regionIds = {
        regions(p), regions(Pixel((p.x + 1) % regions.cols, p.y)),
        regions(Pixel(p.x, (p.y + 1) % regions.rows)),
        regions(Pixel((p.x + 1) % regions.cols, (p.y + 1) % regions.rows))};
    std::set<int> idset(regionIds.begin(), regionIds.end());
    if (idset.size() == 3) {
      junctions.emplace_back(std::vector<int>(idset.begin(), idset.end()), p);
    } else if (idset.size() == 4) {
      junctions.emplace_back(std::move(regionIds), p);
    }
  }
  return junctions;
}

void ExtractSegmentationTopology(const Imagei &segs,
                                 std::vector<std::vector<Pixel>> &bndpixels,
                                 std::vector<Pixel> &juncpositions,
                                 std::vector<std::vector<int>> &seg2bnds,
                                 std::vector<std::pair<int, int>> &bnd2segs,
                                 std::vector<std::vector<int>> &seg2juncs,
                                 std::vector<std::vector<int>> &junc2segs,
                                 std::vector<std::pair<int, int>> &bnd2juncs,
                                 std::vector<std::vector<int>> &junc2bnds,
                                 bool crossBorder) {

  bndpixels.clear();
  juncpositions.clear();
  seg2bnds.clear();
  seg2juncs.clear();
  bnd2segs.clear();
  bnd2juncs.clear();
  junc2segs.clear();
  junc2bnds.clear();

  int width = segs.cols;
  int height = segs.rows;
  int nsegs = MinMaxValOfImage(segs).second + 1;

  seg2bnds.resize(nsegs);
  seg2juncs.resize(nsegs);

  std::map<std::set<int>, std::vector<int>> segs2juncs;
  std::map<std::pair<int, int>, std::set<Pixel>> segpair2pixels;
  std::map<Pixel, std::set<int>> pixel2segs;
  std::map<Pixel, int> pixel2junc;

  for (auto it = segs.begin(); it != segs.end(); ++it) {
    auto p = it.pos();
    if (p.x == segs.cols - 1 && !crossBorder)
      continue;
    if (p.y == segs.rows - 1 && !crossBorder)
      continue;

    // seg ids related
    std::set<int> idset = {
        segs(p), segs(Pixel((p.x + 1) % segs.cols, p.y)),
        segs(Pixel(p.x, (p.y + 1) % segs.rows)),
        segs(Pixel((p.x + 1) % segs.cols, (p.y + 1) % segs.rows))};

    if (idset.size() <= 1) {
      continue;
    }

    // meet a bnd or a junc (and bnd) pixel
    pixel2segs[p] = std::move(idset);
    auto &relatedsegs = pixel2segs[p];

    // register this pixel as a bnd candidate for bnd of related segids
    for (auto ii = relatedsegs.begin(); ii != relatedsegs.end(); ++ii) {
      for (auto jj = std::next(ii); jj != relatedsegs.end(); ++jj) {
        segpair2pixels[MakeOrderedPair(*ii, *jj)].insert(p);
      }
    }

    // meet a junc
    if (relatedsegs.size() >= 3) {
      // create a new junc
      juncpositions.push_back(p);
      int newjuncid = juncpositions.size() - 1;
      pixel2junc[p] = newjuncid;
      segs2juncs[relatedsegs].push_back(newjuncid);
      for (int segid : relatedsegs) {
        seg2juncs[segid].push_back(newjuncid);
      }
      junc2segs.emplace_back(relatedsegs.begin(), relatedsegs.end());
    }
  }

  junc2bnds.resize(juncpositions.size());

  // connect different junctions using allbndpixels
  // and thus generate seperated bnds
  for (int i = 0; i < juncpositions.size(); i++) {
    auto &juncpos = juncpositions[i];
    auto &relatedSegIds = junc2segs[i];

    for (int ii = 0; ii < relatedSegIds.size(); ii++) {
      for (int jj = ii + 1; jj < relatedSegIds.size(); jj++) {
        int segi = relatedSegIds[ii];
        int segj = relatedSegIds[jj];

        const auto &pixelsForThisSegPair =
            segpair2pixels.at(MakeOrderedPair(segi, segj));
        std::vector<Pixel> pixelsForThisBnd;

        std::set<Pixel> visitedPixels;

        // use BFS
        std::queue<Pixel> Q;
        Q.push(juncpos);
        visitedPixels.insert(juncpos);

        while (!Q.empty()) {
          auto curp = Q.front();
          pixelsForThisBnd.push_back(curp);

          // find another junc!
          if (Contains(pixel2junc, curp) && i < pixel2junc.at(curp) &&
              !pixelsForThisBnd.empty()) {
            int tojuncid = pixel2junc.at(curp);
            // make a new bnd!
            bndpixels.push_back(std::move(pixelsForThisBnd));
            int newbndid = bndpixels.size() - 1;
            bnd2juncs.emplace_back(i, tojuncid);
            bnd2segs.emplace_back(segi, segj);
            junc2bnds[i].push_back(newbndid);
            junc2bnds[tojuncid].push_back(newbndid);
            seg2bnds[segi].push_back(newbndid);
            seg2bnds[segj].push_back(newbndid);
            break;
          }

          Q.pop();
          for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
              auto nextp = curp + Pixel(x, y);
              if (!Contains(segs.size(), nextp) && !crossBorder) {
                continue;
              }
              if (!Contains(pixelsForThisSegPair, nextp)) {
                continue;
              }
              nextp.x = (nextp.x + width) % width;
              nextp.y = (nextp.y + height) % height;
              if (Contains(visitedPixels, nextp)) {
                continue;
              }
              Q.push(nextp);
              visitedPixels.insert(nextp);
            }
          }
        }
      }
    }
  }
}

SegmentationTopo::SegmentationTopo(const Imagei &segs, bool crossBorder) {
  core::ExtractSegmentationTopology(segs, bndpixels, juncpositions, seg2bnds,
                                    bnd2segs, seg2juncs, junc2segs, bnd2juncs,
                                    junc2bnds, crossBorder);
}
}
}