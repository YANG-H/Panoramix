#pragma once

#include "basic_types.hpp"
#include "utility.hpp"

#include "pi_graph.hpp"

namespace pano {
namespace experimental {

using namespace pano::core;

struct PILayoutAnnotation {
  std::string impath; // ver1

  Image originalImage;
  Image rectifiedImage;
  bool extendedOnTop;
  bool extendedOnBottom;

  bool topIsPlane;    // ver1
  bool bottomIsPlane; // ver1

  PanoramicView view;
  std::vector<Vec3> vps;
  int vertVPId;

  std::vector<Line3> lines; // ver1

  std::vector<Vec3> corners;
  std::vector<std::pair<int, int>> border2corners; // from, to
  std::vector<bool> border2connected;

  std::vector<std::vector<int>> face2corners;
  std::vector<SegControl> face2control;
  std::vector<Plane3> face2plane;
  std::vector<std::pair<int, int>> coplanarFacePairs;
  std::vector<Polygon3> clutters;

  PILayoutAnnotation() : vertVPId(-1) {}

  int ncorners() const { return corners.size(); }
  int nborders() const { return border2corners.size(); }
  int nfaces() const { return face2corners.size(); }

  int getBorder(int c1, int c2);
  int addBorder(int c1, int c2);
  int splitBorderBy(int b, int c);

  void regenerateFaces();
  int setCoplanar(int f1, int f2);

  template <class Archiver>
  inline void serialize(Archiver &ar, std::int32_t version) {
    if (version == 0) {
      ar(originalImage, rectifiedImage, extendedOnTop, extendedOnBottom);
      ar(view, vps, vertVPId);
      ar(corners, border2corners, border2connected);
      ar(face2corners, face2control, face2plane, coplanarFacePairs, clutters);
    } else if (version == 1) {
      ar(impath);
      ar(originalImage, rectifiedImage, extendedOnTop, extendedOnBottom);
      ar(topIsPlane, bottomIsPlane);
      ar(view, vps, vertVPId);
      ar(lines);
      ar(corners, border2corners, border2connected);
      ar(face2corners, face2control, face2plane, coplanarFacePairs, clutters);
    }
  }
};

std::string LayoutAnnotationFilePath(const std::string &imagePath);

PILayoutAnnotation
LoadOrInitializeNewLayoutAnnotation(const std::string &imagePath);

void EditLayoutAnnotation(const std::string &imagePath,
                          PILayoutAnnotation &anno);

void SaveLayoutAnnotation(const std::string &imagePath,
                          const PILayoutAnnotation &anno);

Imageb GuessMask(const PILayoutAnnotation &anno);
}
}

CEREAL_CLASS_VERSION(pano::experimental::PILayoutAnnotation, 1);