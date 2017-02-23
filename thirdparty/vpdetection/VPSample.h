/* 
 *  Copyright (c) 2011  Chen Feng (cforrest (at) umich.edu)
 *    and the University of Michigan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <vector>

namespace VPSample {

	std::vector<std::vector<float> *>* run(
		//// Input arguments
		// Arg 0, points
		std::vector<std::vector<float> *> *mDataPoints,
		// Arg 1, Number of desired samples
		unsigned int mNSample,
		// Arg 2, type of model: 0 - Planes 1 - 2dLines
		unsigned int /*mModelType*/,
		// ----- facultatives
		// Arg 3, Non uniform first sampling vector(NULL-empty if uniform sampling is choosen)
		double *mFirstSamplingVector = NULL,
		// Arg 4, Non first sampling type: 0 - Uniform(def) 1 - Exp 2 - Kd-Tree
		unsigned int mNFSamplingType = 1,
		// Arg 5, Sigma Exp(def = 1.0) or neigh search for Kd-Tree (def = 10)
		double mSigmaExp = 1.0, int mKdTreeRange = 10,
		// Arg 6, only for kd-tree non first sampling: close points probability (def = 0.8)
		double mKdTreeCloseProb = 0.8,
		// Arg 7, only for kd-tree non first sampling: far points probability (def = 0.2)
		double mKdTreeFarProb = 0.2
		);

}
