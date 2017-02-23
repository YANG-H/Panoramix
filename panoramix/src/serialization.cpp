#include "pch.hpp"

#include "serialization.hpp"

#ifdef _MSC_VER
#include <windows.h>
#endif

#include "macros.hpp"

namespace pano {
namespace core {

TimeStamp LastModifiedTimeOfFile(const char *filename) {
#ifdef _MSC_VER
  HANDLE hFile =
      ::CreateFile(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
                   FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
  if (hFile == INVALID_HANDLE_VALUE) {
    std::cerr << "file: " << filename << " doesn't exist!" << std::endl;
    ::CloseHandle(hFile);
    return 0;
  }
  ::FILETIME creationTime, lastAccessTime, lastWriteTime;
  ::GetFileTime(hFile, &creationTime, &lastAccessTime, &lastWriteTime);
  ::CloseHandle(hFile);
  return (uint64_t(lastWriteTime.dwHighDateTime) << 32u) +
         lastWriteTime.dwLowDateTime;
#else
  NOT_IMPLEMENTED_YET();
#endif
}

TimeStamp LastModifiedTimeOfFile(const std::string &filename) {
  return LastModifiedTimeOfFile(filename.data());
}

TimeStamp CurrentTime() {
#ifdef _MSC_VER
  ::FILETIME ft;
  ::GetSystemTimeAsFileTime(&ft);
  return (uint64_t(ft.dwHighDateTime) << 32u) + ft.dwLowDateTime;
#else
  NOT_IMPLEMENTED_YET();
#endif
}
}
}