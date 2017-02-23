#pragma once

#include "pi_graph.hpp"

namespace pano {
namespace experimental {

void AttachPrincipleDirectionConstraints(PIGraph<core::PanoramicCamera> &mg);
void AttachWallConstraints(PIGraph<core::PanoramicCamera> &mg, double angle);

void DisableTopSeg(PIGraph<core::PanoramicCamera> &mg);
void DisableBottomSeg(PIGraph<core::PanoramicCamera> &mg);

void AttachGCConstraints(PIGraph<core::PanoramicCamera> &mg,
                         const View<core::PanoramicCamera, Image5d> &gc,
                         double clutterThres = 0.7, double wallThres = 0.5,
                         bool onlyConsiderBottomHalf = true);
void AttachGCConstraints(PIGraph<core::PanoramicCamera> &mg,
                         const View<core::PerspectiveCamera, Image5d> &gc,
                         double clutterThres = 0.7, double wallThres = 0.5,
                         bool onlyConsiderBottomHalf = true);

inline void AttachGCConstraints(PIGraph<core::PanoramicCamera> &mg, const Image5d &gc,
                         double clutterThres = 0.7, double wallThres = 0.5,
                         bool onlyConsiderBottomHalf = true) {
  assert(mg.view.image.size() == gc.size());
  AttachGCConstraints(mg, MakeView(gc, mg.view.camera), clutterThres, wallThres,
                      onlyConsiderBottomHalf);
}
}
}