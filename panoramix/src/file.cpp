#include "pch.hpp"

#include "file.hpp"
#include "qttools.hpp"

namespace pano {
namespace misc {
std::string Tagify(const std::string &path) {
  auto tag = path;
  std::replace(tag.begin(), tag.end(), '.', '_');
  std::replace(tag.begin(), tag.end(), '\\', '.');
  std::replace(tag.begin(), tag.end(), '/', '.');
  std::replace(tag.begin(), tag.end(), ':', '.');
  return tag;
}

static std::string _cachePath = PANORAMIX_CACHE_DATA_DIR_STR "/";
std::string CachePath() { return _cachePath; }
void SetCachePath(const std::string &path) { _cachePath = path; }

std::string FolderOfFile(const std::string &filepath) {
  QFileInfo finfo(QString::fromStdString(filepath));
  if (!finfo.exists())
    return std::string();
  return finfo.absolutePath().toStdString();
}

std::string NameOfFile(const std::string &filepath) {
  QFileInfo finfo(QString::fromStdString(filepath));
  if (!finfo.exists())
    return std::string();
  return finfo.fileName().toStdString();
}

void MakeDir(const std::string d) {
  QDir dir = QDir::root();
  if (!dir.mkpath(QString::fromStdString(d))) {
    std::cerr << ("Failed making directory of \"" + d + "\"") << std::endl;
  }
}
}
}