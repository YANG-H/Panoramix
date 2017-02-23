#pragma once

#include <fstream>

#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/bitset.hpp>
#include <cereal/types/chrono.hpp>
#include <cereal/types/common.hpp>
#include <cereal/types/complex.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/forward_list.hpp>
#include <cereal/types/list.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/queue.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/stack.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/unordered_set.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/archives/xml.hpp>

#include <opencv2/opencv.hpp>

namespace cv {

// MUST be defined in the namespace of the underlying type (cv::XXX),
//    definition of alias names in namespace pano::core won't work!
// see
// http://stackoverflow.com/questions/13192947/argument-dependent-name-lookup-and-typedef

// Serialization for cv::Mat
template <class Archive> void save(Archive &ar, Mat const &im) {
  ar(im.elemSize(), im.type(), im.cols, im.rows);
  ar(cereal::binary_data(im.data, im.cols * im.rows * im.elemSize()));
}

// Serialization for cv::Mat
template <class Archive> void load(Archive &ar, Mat &im) {
  size_t elemSize;
  int type, cols, rows;
  ar(elemSize, type, cols, rows);
  im.create(rows, cols, type);
  ar(cereal::binary_data(im.data, cols * rows * elemSize));
}

// Serialization for cv::Matx<T, M, N>
template <class Archive, class T, int M, int N>
inline void serialize(Archive &ar, Matx<T, M, N> &m) {
  ar(m.val);
}

// Serialization for cv::Point_<T>
template <class Archive, class T>
inline void serialize(Archive &ar, Point_<T> &p) {
  ar(p.x, p.y);
}

// Serialization for cv::Size_<T>
template <class Archive, class T>
inline void serialize(Archive &ar, Size_<T> &s) {
  ar(s.width, s.height);
}

// Serialization for cv::Rect_<T>
template <class Archive, class T>
inline void serialize(Archive &ar, Rect_<T> &r) {
  ar(r.x, r.y, r.width, r.height);
}

// Serialization for cv::KeyPoint
template <class Archive> inline void serialize(Archive &ar, KeyPoint &p) {
  ar(p.pt, p.size, p.angle, p.response, p.octave, p.class_id);
}

// Serialization for cv::Moments
template <class Archive> inline void serialize(Archive &ar, Moments &m) {
  ar(m.m00, m.m10, m.m01, m.m20, m.m11, m.m02, m.m30, m.m21, m.m12, m.m03);
  ar(m.mu20, m.mu11, m.mu02, m.mu30, m.mu21, m.mu12, m.mu03);
  ar(m.nu20, m.nu11, m.nu02, m.nu30, m.nu21, m.nu12, m.nu03);
}
}

namespace pano {

namespace core {

using BinaryOutputArchive = cereal::PortableBinaryOutputArchive;
using BinaryInputArchive = cereal::PortableBinaryInputArchive;

using JSONOutputArchive = cereal::JSONOutputArchive;
using JSONInputArchive = cereal::JSONInputArchive;

// serialization wrapper
template <class StringT, class... T>
inline bool SaveToDisk(StringT &&filename, const T &... data) {
  std::ofstream out(filename, std::ios::binary);
  if (!out.is_open()) {
    std::cout << "file \"" << filename << "\" cannot be saved!" << std::endl;
    return false;
  }
  try {
    BinaryOutputArchive archive(out);
    archive(data...);
    std::cout << "file \"" << filename << "\" saved" << std::endl;
    out.close();
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
  return true;
}

template <class StringT, class... T>
inline bool LoadFromDisk(StringT &&filename, T &... data) {
  std::ifstream in(filename, std::ios::binary);
  if (!in.is_open()) {
    std::cout << "file \"" << filename << "\" cannot be loaded!" << std::endl;
    return false;
  }
  try {
    BinaryInputArchive archive(in);
    archive(data...);
    std::cout << "file \"" << filename << "\" loaded" << std::endl;
    in.close();
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }
  return true;
}

using TimeStamp = uint64_t;

// last modified time
TimeStamp LastModifiedTimeOfFile(const char *filename);
TimeStamp LastModifiedTimeOfFile(const std::string &filename);

// get current time
TimeStamp CurrentTime();

// update b if b is older than a (b depends on a)
template <class StringT1, class StringT2, class FunT>
inline void UpdateIfFileIsTooOld(const StringT1 &a, const StringT2 &bNeedsA,
                                 const FunT &useAToUpdateBAndSaveB,
                                 bool forceUpdate = false) {
  if (forceUpdate) {
    std::cout << "updating \"" << bNeedsA << "\" (forced) ..." << std::endl;
    useAToUpdateBAndSaveB(a, bNeedsA);
    return;
  }
  TimeStamp aTime = LastModifiedTimeOfFile(a);
  TimeStamp bTime = LastModifiedTimeOfFile(bNeedsA);
  if (aTime > bTime) { // a is newer, then b must be updated
    std::cout << "updating \"" << bNeedsA << "\" ..." << std::endl;
    useAToUpdateBAndSaveB(a, bNeedsA);
  } else {
    std::cout << "\"" << bNeedsA << "\" doesn't need update" << std::endl;
  }
}
}
}
