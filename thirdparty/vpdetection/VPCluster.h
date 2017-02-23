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

namespace VPCluster {
	//return num of clusters
	unsigned int run(
		//// OUTPUT
		std::vector<unsigned int>& Lables,
		std::vector<unsigned int>& LableCount,
		//// Input arguments
		// Arg 0, points
		std::vector<std::vector<float> *> *mDataPoints,
		// Arg 1, Models
		std::vector<std::vector<float> *> *mModels,
		// Arg 2, InliersThreshold
		float mInlierThreshold,
		// Arg 3, type of model: 0 - Planes 1 - 2dLines 2 Vanishing point
		unsigned int /*mModelType*/,
		// ----- facultatives
		// Arg 4, Select the KNN number of neighboards that can be merged. ( = 0 if all the points are used; n^2 complexity)
		int mKDTreeRange = -1,
		// Arg 5, Already existing clusters, Logical Matrix containing Pts X NCluster");
		std::vector<std::vector<unsigned int> > mExistingClusters = std::vector<std::vector<unsigned int> >()
		);
}
