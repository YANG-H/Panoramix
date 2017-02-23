#pragma once

#include "matlab_api.hpp"
#include "pi_graph.hpp"
#include "pi_graph_annotation.hpp"
#include "pi_graph_cg.hpp"

namespace pano {
namespace experimental {

void ReconstructLayoutAnnotation(PILayoutAnnotation &anno,
                                 misc::Matlab &matlab);

// use general plane representation method
void ReconstructLayoutAnnotation2(PILayoutAnnotation &anno,
                                  misc::Matlab &matlab);
void ReconstructLayoutAnnotation3(PILayoutAnnotation &anno,
                                  misc::Matlab &matlab);


double Solve(const PICGDeterminablePart &dp, PIConstraintGraph &cg,
             misc::Matlab &matlab,
             int maxIter = std::numeric_limits<int>::max(),
             double connectionWeightRatioOverCoplanarity = 1e7,
             bool useCoplanarity = true);

int DisableUnsatisfiedConstraints(
    const PICGDeterminablePart &dp, PIConstraintGraph &cg,
    const std::function<bool(double distRankRatio, double avgDist,
                             double maxDist)> &whichToDisable);

// disorient invalid entities according to current reconstuction
void DisorientDanglingLines(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg, double ratio);

void DisorientDanglingLines2(const PICGDeterminablePart &dp,
                             PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                             double thresRatio);

void DisorientDanglingLines3(const PICGDeterminablePart &dp,
                             PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                             double disorientRatio, double thresRatio);

void DisorientDanglingSegs(const PICGDeterminablePart &dp,
                           PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                           double thresRatio);

// disorient those that are not compatible with neighbor segs
// also disconnect their connections with lines
void DisorientDanglingSegs2(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                            double thresRatio);

void DisorientDanglingSegs3(const PICGDeterminablePart &dp,
                            PIConstraintGraph &cg, PIGraph<PanoramicCamera> &mg,
                            double disorientRatio, double thresRatio);

void OverorientSkewSegs(const PICGDeterminablePart &dp, PIConstraintGraph &cg,
                        PIGraph<PanoramicCamera> &mg, double angleThres,
                        double positionAngleThres, double oriRatio);
}
}