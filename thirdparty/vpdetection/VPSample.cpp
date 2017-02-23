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

#include "RandomSampler.h"
#include "JLinkage.h"
#include "VPPrimitive.h"
#include "updator.h"
#include "VPSample.h"

#include <iostream>
#include <fstream>
#include <vector>

namespace VPSample {
	using namespace Updator;

	/*void PrintHelp(){
		printf("\n\n *** JLinkageRandomSamplerMex v.1.0 *** Part of the SamantHa Project");
		printf("\n     Author roberto.toldo@univr.it - Vips Lab - Department of Computer Science - University of Verona(Italy)");
		printf("\n ***********************************************************************");
		printf("\n Usage: [Labels] = JLnkRandomSampler(Points, NSamples, ModelType, (FirstSamplingVectorProb = []), (SamplingType = UNIFORM), (Par1), (Par2), (Par3))");
		printf("\n Input:");
		printf("\n        Points - Input dataset (Dimension x NumberOfPoints)");
		printf("\n        NSamples - Number of desired samples");
		printf("\n        ModelType - type of models extracted. Currently the model supported are: 0 - Planes 1 - 2dLines");
		printf("\n        FirstSamplingVectorProb(facultative) - Associate to each point a (non-uniform) probability to be randomly picked (leave [] to set a uniform probability for each point)");
		printf("\n        SamplingType(facultative) - Non first sampling strategy: 0 - Uniform(default) 1 - Exp 2 - Kd-Tree 3 - Memory Efficient Kd-Tree(Slower)");
		printf("\n        Par1(facultative) - Sigma Exp(default = 1.0) or Neighbor search for Kd-Tree (default = 10)");
		printf("\n        Par2(facultative) - only for kd-tree non first sampling: close points probability (default = 0.8)");
		printf("\n        Par3(facultative) - only for kd-tree non first sampling: far points probability (default = 0.2)");
		printf("\n Output:");
		printf("\n        Models - Generated hypotesis(Dimension x NSampler)");
		printf("\n");
	}*/

	unsigned int mMSS = 0;

	// Function pointers
	std::vector<float>  *(*mGetFunction)(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts);
	float (*mDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt);

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
		double *mFirstSamplingVector/* = NULL*/,
		// Arg 4, Non first sampling type: 0 - Uniform(def) 1 - Exp 2 - Kd-Tree
		unsigned int mNFSamplingType/* = NFST_EXP*/,
		// Arg 5, Sigma Exp(def = 1.0) or neighboards search for Kd-Tree (def = 10)
		double mSigmaExp/* = 1.0*/, int mKdTreeRange/* = 10*/,
		// Arg 6, only for kd-tree non first sampling: close points probability (def = 0.8)
		double mKdTreeCloseProb/* = 0.8*/,
		// Arg 7, only for kd-tree non first sampling: far points probability (def = 0.2)
		double mKdTreeFarProb/* = 0.2*/
		) 
	{ 
		// arg2 : NSample
		if(mNSample == 0) {
			printf("Invalid NSample");
			return 0;
		}

		// arg3 : modelType;
		/*switch(mModelType){
		case MT_PLANE: mMSS = 3; break;
		case MT_LINE: mMSS = 2; break;
		case MT_VP:*/ mMSS = 2; /*break;
		default: printf("Invalid model type"); return 0; break;
		}*/

		// Arg 0, data points	
		if(mDataPoints->at(0)->size() < mMSS) {
			printf("Invalid data points vector");
			return 0;
		}

		if(mNFSamplingType >=NFST_SIZE) {
			printf("Invalid Non first sampling type");
			return 0;
		}

		if(mKdTreeRange <= 0 || mSigmaExp < 0.0) {
			printf("Invalid Sigma exp or KdTreeRange");
			return 0;
		}

		mKdTreeCloseProb = 1.0;
		mKdTreeFarProb = 1.0;

		if(mKdTreeCloseProb < 0.0 ) {
			printf("Invalid mKdTreeCloseProb");
			return 0;
		}

		if(mKdTreeCloseProb < 0.0 ) {
			printf("Invalid mKdTreeFarProb");
			return 0;
		}

		// Set the distance and get functions
		/*switch(mModelType)
		{
		case MT_PLANE:
			mGetFunction = GetFunction_Plane;
			mDistanceFunction = DistanceFunction_Plane;
			mPtPairDistance = PtPairDistance_Euclidean;
			break;
		case MT_LINE:
			mGetFunction = GetFunction_Line;
			mDistanceFunction = DistanceFunction_Line;
			mPtPairDistance = PtPairDistance_Euclidean;
			break;
		case MT_VP:*/
			mGetFunction = GetFunction_VP;
			mDistanceFunction = DistanceFunction_VP;
			/*break;
		default: printf("Invalid model type"); return 0; break;
		}*/

		RandomSampler mRandomSampler(mGetFunction, mDistanceFunction, (int)(*(*mDataPoints)[0]).size()-1, mMSS, (int)mDataPoints->size(),true);

		printf("Inizialing Data \n");
		printf("\t Loading Points... \n");
		mRandomSampler.SetPoints(mDataPoints);

		printf("\t Inizialing Probabilities... \n");
		if(mFirstSamplingVector != NULL)
			for(unsigned int i=0; i < mDataPoints->size(); i++)
				mRandomSampler.SetFirstSamplingProb(i, (float)mFirstSamplingVector[i]);

		if(NFST_EXP == mNFSamplingType)
			mRandomSampler.SetNFSamplingTypeExp((float)mSigmaExp);

		if(NFST_NN == mNFSamplingType)
			mRandomSampler.SetNFSamplingTypeNN(mKdTreeRange, (float)mKdTreeCloseProb, (float)mKdTreeFarProb, false);
		if(NFST_NN_ME == mNFSamplingType)
			mRandomSampler.SetNFSamplingTypeNN(mKdTreeRange, (float)mKdTreeCloseProb, (float)mKdTreeFarProb, true);

		InitializeWaitbar("Generating Hypotesis");
		std::vector<std::vector<float> *> *mModels = 
			mRandomSampler.GetNSample(mNSample, 0, NULL, &UpdateWaitbar);
		CloseWaitbar();


		return mModels;

	}

}
