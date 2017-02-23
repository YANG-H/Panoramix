#pragma once

#include "basic_types.hpp"

namespace pano {
namespace gui {

using namespace ::pano::core;

// color
enum ColorTag {
  Transparent,

  White,
  Black,

  DimGray,
  Gray,
  DarkGray,
  Silver,
  LightGray,

  Red,
  Green,
  Blue,

  Yellow,
  Magenta,
  Cyan,
  Orange
};

class Color {
public:
  inline Color() : _rgba(255, 255, 255, 255) {}
  inline Color(int r, int g, int b, int a = 255) : _rgba(r, g, b, a) {}
  inline Color(double r, double g, double b, double a = 1.0)
      : _rgba(static_cast<int>(r * 255), static_cast<int>(g * 255),
              static_cast<int>(b * 255), static_cast<int>(a * 255)) {}

  // from vec4
  template <class T, class = std::enable_if_t<std::is_integral<T>::value>>
  inline Color(const Vec<T, 4> &v) : _rgba(v) {}

  template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>,
            class = void>
  inline Color(const Vec<T, 4> &v) : _rgba(v * 255) {}

  // from vec3
  template <class T, class = std::enable_if_t<std::is_integral<T>::value>>
  inline Color(const Vec<T, 3> &v, T a = 255)
      : _rgba(static_cast<int>(v[0]), static_cast<int>(v[1]),
              static_cast<int>(v[2]), a) {}

  template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>,
            class = void>
  inline Color(const Vec<T, 3> &v, T a = 1.0)
      : _rgba(static_cast<int>(v[0] * 255), static_cast<int>(v[1] * 255),
              static_cast<int>(v[2] * 255), a * 255) {}

  // from tag
  Color(ColorTag tag);
  // from raw data
  Color(const std::uint8_t *data, int cvType);

public:
  inline int red() const { return _rgba[0]; }
  inline int green() const { return _rgba[1]; }
  inline int blue() const { return _rgba[2]; }
  inline int alpha() const { return _rgba[3]; }

  inline float redf() const { return _rgba[0] / 255.0f; }
  inline float greenf() const { return _rgba[1] / 255.0f; }
  inline float bluef() const { return _rgba[2] / 255.0f; }
  inline float alphaf() const { return _rgba[3] / 255.0f; }

  inline bool isTransparent() const { return _rgba[3] == 0; }

  // to cv::Scalar (bgra)
  inline operator cv::Scalar() const {
    return cv::Scalar(_rgba[2], _rgba[1], _rgba[0], _rgba[3]);
  }

  // to vec4
  template <class T, class = std::enable_if_t<std::is_integral<T>::value>>
  inline operator Vec<T, 4>() const {
    return _rgba;
  }

  template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>,
            class = void>
  inline operator Vec<T, 4>() const {
    return Vec<T, 4>(_rgba) / 255.0;
  }

  // to vec3
  template <class T, class = std::enable_if_t<std::is_integral<T>::value>>
  inline operator Vec<T, 3>() const {
    return Vec<T, 3>(_rgba[0], _rgba[1], _rgba[2]);
  }

  template <class T, class = std::enable_if_t<std::is_floating_point<T>::value>,
            class = void>
  inline operator Vec<T, 3>() const {
    return Vec<T, 3>(_rgba[0] / 255.0, _rgba[1] / 255.0,
                           _rgba[2] / 255.0);
  }

  inline bool operator==(const Color &color) const {
    return _rgba == color._rgba;
  }

  template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
  inline Color &operator*=(T s) {
    _rgba *= s;
    return *this;
  }
  template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
  inline Color &operator/=(T s) {
    _rgba /= s;
    return *this;
  }

  inline Color blendWith(const Color &c, double alpha) const {
    return _rgba * (1.0 - alpha) + c._rgba * alpha;
  }

  template <class Archive> inline void serialize(Archive &ar) { ar(_rgba); }

private:
  Vec4i _rgba;
};

template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline Color operator*(const Color &c, T d) {
  Color cc = c;
  cc *= d;
  return cc;
}
template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline Color operator*(T d, const Color &c) {
  Color cc = c;
  cc *= d;
  return cc;
}
template <class T, class = std::enable_if_t<std::is_arithmetic<T>::value>>
inline Color operator/(const Color &c, T d) {
  Color cc = c;
  cc /= d;
  return cc;
}

std::ostream &operator<<(std::ostream &os, ColorTag ct);
Color ColorFromHSV(double h, double s, double v, double a = 1.0);
Color RandomColor();
inline Color ColorFromImage(const Image &im, Pixel p) {
  return Color(im.ptr(p.y, p.x), im.type());
}

// line style
// keep sync with Qt::PenStyle
enum PenStyle {
  NoPen,
  SolidLine,      // 1	A plain line.
  DashLine,       // 2	Dashes separated by a few pixels.
  DotLine,        // 3	Dots separated by a few pixels.
  DashDotLine,    // 4	Alternate dots and dashes.
  DashDotDotLine, // 5	One dash, two dots, one dash, two dots.
  CustomDashLine
};

struct PenConfig {
  std::string name;
  std::string description;
  double thickness;
  Color color;
  PenStyle style;
};

// color table
enum ColorTableDescriptor {
  RGB,
  AllColors,
  AllColorsExcludingWhite,
  AllColorsExcludingBlack,
  AllColorsExcludingWhiteAndBlack,
  RGBGreys
};

template <class T> struct Colored {
  T component;
  Color color;

  template <class Archive> inline void serialize(Archive &ar) {
    ar(component, color);
  }
};

template <class T>
inline Colored<std::decay_t<T>> ColorAs(T &&comp, const Color &color) {
  return Colored<std::decay_t<T>>{std::forward<T>(comp), color};
}

class ColorTable {
public:
  inline ColorTable() : _exceptionalColor(ColorTag::Transparent) {}
  inline ColorTable(const std::vector<Color> &ctable,
                    const Color &exceptColor = ColorTag::Transparent)
      : _colors(ctable), _exceptionalColor(exceptColor) {}
  inline ColorTable(std::vector<Color> &&ctable,
                    const Color &exceptColor = ColorTag::Transparent)
      : _colors(std::move(ctable)), _exceptionalColor(exceptColor) {}

  ColorTable(ColorTableDescriptor descriptor);

  inline ColorTable(std::initializer_list<Color> c,
                    const Color &exceptColor = ColorTag::Transparent)
      : _colors(c), _exceptionalColor(exceptColor) {}

  template <
      class ColorIterT,
      class = std::enable_if_t<std::is_same<
          std::iterator_traits<ColorIterT>::value_type, Color>::value>>
  inline ColorTable(ColorIterT begin, ColorIterT end,
                    const Color &exceptColor = ColorTag::Transparent)
      : _colors(begin, end), _exceptionalColor(exceptColor) {}

public:
  const std::vector<Color> &colors() const { return _colors; }
  size_t size() const { return _colors.size(); }
  const Color &exceptionalColor() const { return _exceptionalColor; }
  Color &exceptionalColor() { return _exceptionalColor; }
  const Color &operator[](int claz) const {
    return claz < 0 || claz >= _colors.size() ? _exceptionalColor
                                              : _colors[claz];
  }
  Color &operator[](int claz) {
    return claz < 0 || claz >= _colors.size() ? _exceptionalColor
                                              : _colors[claz];
  }
  template <class T, class ClassT>
  Colored<T> operator()(const Classified<T, ClassT> &c) const {
    return ColorAs(std::forward<T>(c.component), (*this)[c.claz]);
  }
  template <class T, class ClassT>
  Colored<T> operator()(Classified<T, ClassT> &&c) const {
    return ColorAs(std::move(c.component), (*this)[c.claz]);
  }

  bool empty() const { return _colors.empty(); }

  Image3ub operator()(const Imagei &indexIm) const;

  const Color &roundedAt(int claz) const {
    return claz < 0 ? _exceptionalColor : _colors[claz % _colors.size()];
  }

  ColorTable &randomize();
  ColorTable &appendRandomizedColors(size_t size);
  ColorTable &appendRandomizedGreyColors(size_t size);

  template <class Archive> inline void serialize(Archive &ar) {
    ar(_colors, _exceptionalColor);
  }

private:
  std::vector<Color> _colors;
  Color _exceptionalColor;
};

ColorTable
CreateGreyColorTableWithSize(int sz, const Color &exceptColor = ColorTag::Blue);
ColorTable CreateRandomColorTableWithSize(
    int sz, const Color &exceptColor = ColorTag::Transparent);
ColorTable
CreateJetColorTableWithSize(int sz,
                            const Color &exceptColor = ColorTag::Transparent);
}
}
