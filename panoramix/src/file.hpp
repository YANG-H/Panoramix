#pragma once

#include "basic_types.hpp"

namespace pano {
namespace misc {
std::string Tagify(const std::string &path);

std::string CachePath();
void SetCachePath(const std::string &path);

template <class StringT, class... Ts>
inline bool SaveCache(const std::string &path, StringT &&what, Ts &&... ts) {
  return pano::core::SaveToDisk(
      CachePath() + Tagify(path) + "_" + what + ".cereal", ts...);
}

template <class StringT, class... Ts>
inline bool LoadCache(const std::string &path, StringT &&what, Ts &... ts) {
  return pano::core::LoadFromDisk(
      CachePath() + Tagify(path) + "_" + what + ".cereal", ts...);
}

std::string FolderOfFile(const std::string &filepath);
std::string NameOfFile(const std::string &filepath);
void MakeDir(const std::string dir);
}
}