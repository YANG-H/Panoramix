#pragma once

#include "basic_types.hpp"

namespace pano {
namespace gui {

// opengl shader source
enum class OpenGLShaderSourceDescriptor {
  XPoints,
  XLines,
  XTriangles,
  XPanorama,
  XPhong
};

class OpenGLShaderSource {
public:
  OpenGLShaderSource(OpenGLShaderSourceDescriptor d =
                         OpenGLShaderSourceDescriptor::XTriangles);

  template <class StringT1, class StringT2>
  OpenGLShaderSource(StringT1 &&vs, StringT2 &&fs)
      : _vshaderSrc(std::forward<StringT1>(vs)),
        _fshaderSrc(std::forward<StringT2>(fs)) {}

  const std::string &vertexShaderSource() const { return _vshaderSrc; }
  const std::string &fragmentShaderSource() const { return _fshaderSrc; }

  template <class Archive> inline void serialize(Archive &ar) {
    ar(_vshaderSrc, _fshaderSrc);
  }

private:
  std::string _vshaderSrc, _fshaderSrc;
};
}
}
