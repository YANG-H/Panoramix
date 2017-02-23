#include "pch.hpp"

#include "utility.hpp"

#include "single_view.hpp"

namespace pano {
namespace core {

namespace {
template <class T> inline size_t ElementsNum(const std::vector<T> &v) {
  return v.size();
}

template <class T>
inline size_t ElementsNum(const std::vector<std::vector<T>> &v) {
  size_t n = 0;
  for (auto &vv : v) {
    n += vv.size();
  }
  return n;
}
}

template <class CameraT>
std::vector<int> ComputeSpatialRegionPropertiesTemplated(
    const Imagei &segmentedRegions, const CameraT &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr,
    std::vector<Vec3> *ncentersPtr, std::vector<double> *areasPtr) {
  double minVal, maxVal;
  std::tie(minVal, maxVal) = MinMaxValOfImage(segmentedRegions);
  assert(minVal == 0.0);

  int regionNum = static_cast<int>(maxVal) + 1;

  if (ncontoursPtr) {
    ncontoursPtr->clear();
    ncontoursPtr->reserve(regionNum);
  }
  if (ncentersPtr) {
    ncentersPtr->clear();
    ncentersPtr->reserve(regionNum);
  }
  if (areasPtr) {
    areasPtr->clear();
    areasPtr->reserve(regionNum);
  }

  std::vector<int> idsOld2New(regionNum, -1);

  // calculate contours and tangential projected areas for each region data
  int newId = 0;
  for (int i = 0; i < regionNum; i++) {

    Image regionMask = (segmentedRegions == i);

    // find contour of the region
    std::vector<std::vector<Pixel>> contours;
    cv::findContours(regionMask, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                              // outer contours
    if (contours.empty()) {
      continue;
    }

    Vec3 centerDirection(0, 0, 0);
    std::vector<Vec3> directions;
    directions.reserve(ElementsNum(contours));
    for (auto &cs : contours) {
      for (auto &c : cs) {
        directions.push_back(normalize(cam.toSpace(c)));
        centerDirection += directions.back();
      }
    }
    centerDirection /= norm(centerDirection);
    // get max angle distance from center direction
    double radiusAngle = 0.0;
    for (auto &d : directions) {
      double a = AngleBetweenDirected(centerDirection, d);
      if (radiusAngle < a) {
        radiusAngle = a;
      }
    }

    // perform a more precise sample !
    int newSampleSize = cam.focal() * radiusAngle * 2 + 2;
    PartialPanoramicCamera sCam(
        newSampleSize, newSampleSize, cam.focal(), cam.eye(), centerDirection,
        ProposeXYDirectionsFromZDirection(centerDirection).second);
    Imagei sampledSegmentedRegions =
        MakeCameraSampler(sCam, cam)(segmentedRegions);

    // collect better contours
    contours.clear();
    regionMask = (sampledSegmentedRegions == i);
    cv::findContours(regionMask, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE); // CV_RETR_EXTERNAL: get only the
                                              // outer contours
    std::sort(contours.begin(), contours.end(),
              [](const std::vector<Pixel> &ca, const std::vector<Pixel> &cb) {
                return ca.size() > cb.size();
              });

    if (contours.size() >= 2) {
      std::cout << "multiple contours for one region in projection!";
    }

    contours.erase(std::remove_if(contours.begin(), contours.end(),
                                  [](const std::vector<Pixel> &c) {
                                    return c.size() <= 2;
                                  }),
                   contours.end());

    if (contours.empty() || contours.front().size() <= 2) {
      continue;
    }

    Vec3 center(0, 0, 0);
    std::vector<std::vector<Vec3>> normalizedContours(contours.size());
    double area = 0.0;
    for (int k = 0; k < contours.size(); k++) {
      normalizedContours[k].reserve(contours[k].size());
      for (auto &p : contours[k]) {
        normalizedContours[k].push_back(normalize(sCam.toSpace(p)));
        center += normalizedContours[k].back();
      }
      std::vector<Point2f> contourf(contours[k].size());
      for (int kk = 0; kk < contours[k].size(); kk++) {
        contourf[kk] = ecast<float>(contours[k][kk]);
      }
      area += cv::contourArea(contourf);
    }
    center = normalize(center);

    if (ncontoursPtr) {
      ncontoursPtr->push_back(std::move(normalizedContours));
    }
    if (ncentersPtr) {
      ncentersPtr->push_back(center);
    }
    if (areasPtr) {
      areasPtr->push_back(area);
    }

    idsOld2New[i] = newId;
    newId++;
  }

  return idsOld2New;
}

std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PerspectiveCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr,
    std::vector<Vec3> *ncentersPtr, std::vector<double> *areasPtr) {
  return ComputeSpatialRegionPropertiesTemplated(
      segmentedRegions, cam, ncontoursPtr, ncentersPtr, areasPtr);
}

std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PanoramicCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr,
    std::vector<Vec3> *ncentersPtr, std::vector<double> *areasPtr) {
  return ComputeSpatialRegionPropertiesTemplated(
      segmentedRegions, cam, ncontoursPtr, ncentersPtr, areasPtr);
}

std::vector<int> ComputeSpatialRegionProperties(
    const Imagei &segmentedRegions, const PartialPanoramicCamera &cam,
    std::vector<std::vector<std::vector<Vec3>>> *ncontoursPtr,
    std::vector<Vec3> *ncentersPtr, std::vector<double> *areasPtr) {
  return ComputeSpatialRegionPropertiesTemplated(
      segmentedRegions, cam, ncontoursPtr, ncentersPtr, areasPtr);
}

View<PartialPanoramicCamera, Imageub>
PerfectRegionMaskView(const std::vector<std::vector<Vec3>> &contours,
                      const Vec3 &center, double focal) {
  auto ncenter = normalize(center);
  double radiusAngle = 0.0;
  for (auto &cs : contours) {
    for (auto &c : cs) {
      double angle = AngleBetweenDirected(center, c);
      if (angle > radiusAngle) {
        radiusAngle = angle;
      }
    }
  }
  int ppcSize = std::ceil(2 * radiusAngle * focal);
  Vec3 x;
  std::tie(x, std::ignore) = ProposeXYDirectionsFromZDirection(ncenter);
  PartialPanoramicCamera ppc(ppcSize, ppcSize, focal, Point3(0, 0, 0), ncenter,
                             x);
  Imageub mask = Imageub::zeros(ppc.screenSize());

  // project contours to ppc
  std::vector<std::vector<Point2i>> contourProjs(contours.size());
  for (int k = 0; k < contours.size(); k++) {
    auto &contourProj = contourProjs[k];
    contourProj.reserve(contours[k].size());
    for (auto &d : contours[k]) {
      contourProj.push_back(ecast<int>(ppc.toScreen(d)));
    }
  }
  cv::fillPoly(mask, contourProjs, (uint8_t)1);
  return View<PartialPanoramicCamera, Imageub>{mask, ppc};
}
}
}