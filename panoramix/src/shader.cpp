#include "pch.hpp"

#include "shader.hpp"

#include "utility.hpp"

namespace pano {
namespace gui {

// opengl shader source
const std::pair<std::string, std::string> &
PredefinedShaderSource(OpenGLShaderSourceDescriptor name) {
  static const std::pair<std::string, std::string> xPointsShaderSource = {
      "#version 130\n"

      "attribute highp vec4 position;\n"
      "attribute highp vec3 normal;\n"
      "attribute lowp vec4 color;\n"
      "attribute lowp vec2 texCoord;\n"
      "attribute uint isSelected;\n"

      "uniform highp mat4 viewMatrix;\n"
      "uniform highp mat4 modelMatrix;\n"
      "uniform highp mat4 projectionMatrix;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    gl_Position = projectionMatrix * viewMatrix * modelMatrix * "
      "position;\n"
      "    pixelColor = color;\n"
      "    pixelTexCoord = texCoord;\n"
      "    pixelSelection = isSelected == 0u ? 0.0 : 1.0;\n"
      "}\n",

      "#version 130\n"

      "uniform sampler2D tex;\n"
      "uniform lowp vec4 globalColor;\n"

      "uniform lowp float bwColor;\n"
      "uniform lowp float bwTexColor;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "   lowp vec4 texColor = texture2D(tex, pixelTexCoord);\n"
      //"   gl_FragColor = (pixelColor * bwColor + texColor * bwTexColor)"
      //        "       / (bwColor + bwTexColor);\n"
      "   gl_FragColor = (pixelColor * 1.0 + texColor * 0.0);\n"
      "   gl_FragColor.a = 1.0 - pixelSelection * 0.5;\n"
      "   float distance = length(gl_PointCoord - vec2(0.5));\n"
      "   if(distance > 0.4 && distance <= 0.5)\n"
      "       gl_FragColor.a = (1.0 - (distance - 0.4) * 0.1) * "
      "gl_FragColor.a;\n"
      "   else if(distance > 0.5)\n"
      "       discard;\n"
      "}\n"};

  static const std::pair<std::string, std::string> xLinesShaderSource = {
      "#version 130\n"

      "attribute highp vec4 position;\n"
      "attribute highp vec3 normal;\n"
      "attribute lowp vec4 color;\n"
      "attribute lowp vec2 texCoord;\n"
      "attribute uint isSelected;\n"

      "uniform highp mat4 viewMatrix;\n"
      "uniform highp mat4 modelMatrix;\n"
      "uniform highp mat4 projectionMatrix;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    gl_Position = projectionMatrix * viewMatrix * modelMatrix * "
      "position;\n"
      "    pixelColor = color;\n"
      "    pixelTexCoord = texCoord;\n"
      "    pixelSelection = isSelected == 0u ? 0.0 : 1.0;\n"
      "}\n",

      "#version 130\n"

      "uniform sampler2D tex;\n"
      "uniform lowp vec4 globalColor;\n"

      "uniform lowp float bwColor;\n"
      "uniform lowp float bwTexColor;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "   lowp vec4 texColor = texture2D(tex, pixelTexCoord);\n"
      "   gl_FragColor = (pixelColor * 1.0 + texColor * 0.0)"
      "     ;\n"
      "   gl_FragColor.a = 1.0 - pixelSelection * 0.5;\n"
      "}\n"};

  static const std::pair<std::string, std::string> xTrianglesShaderSource = {
      "#version 130\n"

      "attribute highp vec4 position;\n"
      "attribute highp vec3 normal;\n"
      "attribute lowp vec4 color;\n"
      "attribute lowp vec2 texCoord;\n"
      "attribute uint isSelected;\n"

      "uniform highp mat4 viewMatrix;\n"
      "uniform highp mat4 modelMatrix;\n"
      "uniform highp mat4 projectionMatrix;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    gl_Position = projectionMatrix * viewMatrix * modelMatrix * "
      "position;\n"
      "    pixelColor = color;\n"
      "    pixelTexCoord = texCoord;\n"
      "    pixelSelection = isSelected == 0u ? 0.0 : 1.0;\n"
      "}\n",

      "#version 130\n"

      "uniform sampler2D tex;\n"
      "uniform lowp vec4 globalColor;\n"

      "uniform lowp float bwColor;\n"
      "uniform lowp float bwTexColor;\n"

      "varying lowp vec4 pixelColor;\n"
      "varying lowp vec2 pixelTexCoord;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    lowp vec4 texColor = texture2D(tex, pixelTexCoord);\n"
      "    gl_FragColor = (pixelColor * bwColor + texColor * bwTexColor)"
      "       / (bwColor + bwTexColor);\n"
      "    gl_FragColor.a = 1.0 - pixelSelection * 0.5;\n"
      "}\n"};

  static const std::pair<std::string, std::string> xPanoramaShaderSource = {
      "#version 130\n"

      "attribute highp vec3 position;\n"
      "attribute highp vec3 normal;\n"
      "attribute highp vec4 color;\n"
      "attribute lowp vec2 texCoord;\n"
      "attribute uint isSelected;\n"

      "uniform highp mat4 viewMatrix;\n"
      "uniform highp mat4 modelMatrix;\n"
      "uniform highp mat4 projectionMatrix;\n"

      "varying highp vec3 pixelPosition;\n"
      "varying highp vec3 pixelNormal;\n"
      "varying highp vec4 pixelColor;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    pixelPosition = position.xyz;\n"
      "    pixelNormal = normal;\n"
      "    pixelColor = color;\n"
      "    gl_Position = projectionMatrix * viewMatrix * modelMatrix * "
      "vec4(position, 1.0);\n"
      "    pixelSelection = isSelected == 0u ? 0.0 : 1.0;\n"
      "}\n",

      // 3.14159265358979323846264338327950288
      "#version 130\n"

      "uniform sampler2D tex;\n"

      "uniform lowp float bwColor;\n"
      "uniform lowp float bwTexColor;\n"
      "uniform highp vec3 panoramaCenter;\n"
      "uniform highp float panoramaHoriCenterRatio;\n"
      "uniform highp float panoramaAspectRatio;\n" // height/width

      "varying highp vec3 pixelPosition;\n"
      "varying highp vec3 pixelNormal;\n"
      "varying highp vec4 pixelColor;\n"
      "varying lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    highp vec3 direction = pixelPosition - panoramaCenter;\n"
      "    highp float longi = atan(direction.y, direction.x);\n"
      "    highp float lati = asin(direction.z / length(direction));\n"
      "    highp float globalx = longi / 3.1415926535897932 / 2.0 + 0.5;\n"
      "    highp float globaly = - lati / 3.1415926535897932 + 0.5;\n"
      //"    highp vec2 texCoord = vec2(longi / 3.1415926535897932 / 2.0 + 0.5,
      //- lati / 3.1415926535897932 + 0.5);\n"
      "    highp float texx = globalx;\n"
      "    highp float texy =  (globaly / 2.0 - (0.25 - "
      "panoramaHoriCenterRatio * panoramaAspectRatio)) / panoramaAspectRatio;\n"
      "    if(texy < 0.0 || texy > 1.0){ discard; } \n"
      "    highp vec2 texCoord = vec2(texx, texy);\n"
      "    lowp vec4 texColor = texture2D(tex, texCoord);\n"
      "    gl_FragColor = (pixelColor * bwColor + texColor * bwTexColor)"
      "       / (bwColor + bwTexColor);\n"
      "    gl_FragColor.a = 1.0 - pixelSelection * 0.5;\n"
      "}\n"};

  static const std::pair<std::string, std::string> xPhongShaderSource = {
      "#version 130\n"

      "attribute highp vec3 position;\n"
      "attribute highp vec3 normal;\n"
      "attribute highp vec4 color;\n"
      "attribute uint isSelected;\n"

      "uniform highp mat4 viewMatrix;\n"
      "uniform highp mat4 modelMatrix;\n"
      "uniform highp mat4 projectionMatrix;\n"

      "out highp vec3 pixelPositionTransformed;\n"
      "out highp vec3 pixelNormalTransformed;\n"
      "out highp vec4 pixelColor;\n"
      "out lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    vec4 posTrans = viewMatrix * modelMatrix * vec4(position, 1.0);\n"
      "    pixelPositionTransformed = posTrans.xyz / posTrans.w;\n"
      "    vec4 normalTrans = viewMatrix * modelMatrix * vec4(normal, 1.0);\n"
      "    pixelNormalTransformed = normalTrans.xyz / normalTrans.w;\n"
      "    pixelColor = color;\n"
      "    gl_Position = projectionMatrix * posTrans;\n"
      "    pixelSelection = isSelected == 0u ? 0.0 : 1.0;\n"
      "}\n",

      // 3.14159265358979323846264338327950288
      "#version 130\n"

      "in highp vec3 pixelPositionTransformed;\n"
      "in highp vec3 pixelNormalTransformed;\n"
      "in highp vec4 pixelColor;\n"
      "in lowp float pixelSelection;\n"

      "void main(void)\n"
      "{\n"
      "    vec3 lightDir = normalize(vec3(5.0, 5.0, 5.0) - pixelPositionTransformed);\n"
      "    vec4 ambient = vec4(1.0, 0.0, 0.0, 1.0);\n"
      "    vec4 diffuse = vec4(1.0, 1.0, 1.0, 1.0);\n"
      "    vec4 specular = vec4(1.0, 1.0, 1.0, 1.0);\n"
      "    gl_FragColor = 0.0 * ambient + "
      "       1.0 * pow(abs(dot(lightDir, normalize(pixelNormalTransformed))), 2) * diffuse + "
      "       .8 * pow(abs(dot(reflect(lightDir, normalize(pixelNormalTransformed)), - "
      "            normalize(pixelPositionTransformed))), 100) * specular;\n"
      "    gl_FragColor.a = 1.0 - pixelSelection * 0.5;\n"
      "}\n"};

  switch (name) {
  case OpenGLShaderSourceDescriptor::XPoints:
    return xPointsShaderSource;
  case OpenGLShaderSourceDescriptor::XLines:
    return xLinesShaderSource;
  case OpenGLShaderSourceDescriptor::XTriangles:
    return xTrianglesShaderSource;
  case OpenGLShaderSourceDescriptor::XPanorama:
    return xPanoramaShaderSource;
  case OpenGLShaderSourceDescriptor::XPhong:
    return xPhongShaderSource;
  default:
    return xTrianglesShaderSource;
  }
}

OpenGLShaderSource::OpenGLShaderSource(OpenGLShaderSourceDescriptor d) {
  std::tie(_vshaderSrc, _fshaderSrc) = PredefinedShaderSource(d);
}
}
}