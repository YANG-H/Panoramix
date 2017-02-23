#pragma once

#include "scene.hpp"
#include "utility.hpp"

#include "pi_graph.hpp"
#include "pi_graph_annotation.hpp"
#include "pi_graph_solve.hpp"

namespace pano {
namespace experimental {
// Print PIGraph<PanoramicCamera>
template <class SegColorerT = core::ConstantFunctor<gui::ColorTag>,
          class LinePieceColorerT = core::ConstantFunctor<gui::ColorTag>,
          class BndPieceColorerT = core::ConstantFunctor<gui::ColorTag>>
inline Image3f PrintPIGraph(const PIGraph<PanoramicCamera> &mg,
                            SegColorerT &&segColor = gui::Transparent,
                            LinePieceColorerT &&lpColor = gui::Transparent,
                            BndPieceColorerT &&bpColor = gui::Transparent,
                            int boundaryWidth = 1, int lineWidth = 2);

template <class SegColorerT = core::ConstantFunctor<gui::ColorTag>,
          class LinePieceColorerT = core::ConstantFunctor<gui::ColorTag>,
          class BndPieceColorerT = core::ConstantFunctor<gui::ColorTag>>
inline Image3f PrintPIGraph2(const PIGraph<PanoramicCamera> &mg,
                             SegColorerT &&segColor = gui::Transparent,
                             LinePieceColorerT &&lpColor = gui::Transparent,
                             BndPieceColorerT &&bpColor = gui::Transparent,
                             int boundaryWidth = 1, int lineWidth = 2);

// VisualizeReconstruction
void VisualizeReconstruction(
    const PICGDeterminablePart &dp, const PIConstraintGraph &cg,
    const PIGraph<PanoramicCamera> &mg, bool showConnectionLines = true,
    const std::function<gui::Color(int vert)> &vertColor =
        core::ConstantFunctor<gui::Color>(gui::Black),
    const std::function<void(int vert)> &vertClick =
        core::ConstantFunctor<void>(),
    bool doModal = true);

// the compact version
void VisualizeReconstructionCompact(const Image &im,
                                    const PICGDeterminablePart &dp,
                                    const PIConstraintGraph &cg,
                                    const PIGraph<PanoramicCamera> &mg,
                                    bool doModel, bool autoCam = false,
                                    bool fixCamUpDir = true);

// VisualizeLayoutAnnotation
void VisualizeLayoutAnnotation(const PILayoutAnnotation &anno,
                               double mergeDepthsThres = 0.0);
}
}

namespace pano {
namespace experimental {
// Print PIGraph<PanoramicCamera>
template <class SegColorerT, class LinePieceColorerT, class BndPieceColorerT>
inline Image3f PrintPIGraph(const PIGraph<PanoramicCamera> &mg,
                            SegColorerT &&segColor, LinePieceColorerT &&lpColor,
                            BndPieceColorerT &&bpColor, int boundaryWidth,
                            int lineWidth) {
  Image3f rendered = Image3f::zeros(mg.segs.size());
  // segs
  for (auto it = rendered.begin(); it != rendered.end(); ++it) {
    int seg = mg.segs(it.pos());
    gui::Color color = segColor(seg);
    *it = Vec3f(color.bluef(), color.greenf(), color.redf());
  }
  // lines
  if (lineWidth > 0) {
    for (int lp = 0; lp < mg.linePiece2line.size(); lp++) {
      gui::Color color = lpColor(lp);
      if (color.isTransparent())
        continue;
      auto &ps = mg.linePiece2samples[lp];
      for (int i = 1; i < ps.size(); i++) {
        auto p1 = ToPixel(mg.view.camera.toScreen(ps[i - 1]));
        auto p2 = ToPixel(mg.view.camera.toScreen(ps[i]));
        if (Distance(p1, p2) >= rendered.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, rendered.cols, rendered.rows), p1, p2);
        cv::line(rendered, p1, p2, (cv::Scalar)color / 255.0, lineWidth);
      }
    }
  }
  // region boundary
  if (boundaryWidth > 0) {
    for (int bp = 0; bp < mg.bndPiece2bnd.size(); bp++) {
      gui::Color color = bpColor(bp);
      if (color.isTransparent())
        continue;
      auto &e = mg.bndPiece2dirs[bp];
      for (int i = 1; i < e.size(); i++) {
        auto p1 = core::ToPixel(mg.view.camera.toScreen(e[i - 1]));
        auto p2 = core::ToPixel(mg.view.camera.toScreen(e[i]));
        if (Distance(p1, p2) >= rendered.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, rendered.cols, rendered.rows), p1, p2);
        cv::line(rendered, p1, p2, (cv::Scalar)color / 255.0, boundaryWidth);
      }
    }
  }
  return rendered;
}

// PrintPIGraph2 PIGraph<PanoramicCamera>
template <class SegColorerT, class LinePieceColorerT, class BndPieceColorerT>
inline Image3f
PrintPIGraph2(const PIGraph<PanoramicCamera> &mg, SegColorerT &&segColor,
              LinePieceColorerT &&lpColor, BndPieceColorerT &&bpColor,
              int boundaryWidth, int lineWidth) {
  Image3f rendered = Image3f::zeros(mg.segs.size());
  // segs
  for (auto it = rendered.begin(); it != rendered.end(); ++it) {
    int seg = mg.segs(it.pos());
    gui::Color color = segColor(seg, it.pos());
    *it = Vec3f(color.bluef(), color.greenf(), color.redf());
  }
  // lines
  if (lineWidth > 0) {
    for (int lp = 0; lp < mg.linePiece2line.size(); lp++) {
      gui::Color color = lpColor(lp);
      if (color.isTransparent())
        continue;
      auto &ps = mg.linePiece2samples[lp];
      for (int i = 1; i < ps.size(); i++) {
        auto p1 = ToPixel(mg.view.camera.toScreen(ps[i - 1]));
        auto p2 = ToPixel(mg.view.camera.toScreen(ps[i]));
        if (Distance(p1, p2) >= rendered.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, rendered.cols, rendered.rows), p1, p2);
        cv::line(rendered, p1, p2, (cv::Scalar)color / 255.0, lineWidth);
      }
    }
  }
  // region boundary
  if (boundaryWidth > 0) {
    for (int bp = 0; bp < mg.bndPiece2bnd.size(); bp++) {
      gui::Color color = bpColor(bp);
      if (color.isTransparent())
        continue;
      auto &e = mg.bndPiece2dirs[bp];
      for (int i = 1; i < e.size(); i++) {
        auto p1 = core::ToPixel(mg.view.camera.toScreen(e[i - 1]));
        auto p2 = core::ToPixel(mg.view.camera.toScreen(e[i]));
        if (Distance(p1, p2) >= rendered.cols / 2) {
          continue;
        }
        cv::clipLine(cv::Rect(0, 0, rendered.cols, rendered.rows), p1, p2);
        cv::line(rendered, p1, p2, (cv::Scalar)color / 255.0, boundaryWidth);
      }
    }
  }
  return rendered;
}
}
}