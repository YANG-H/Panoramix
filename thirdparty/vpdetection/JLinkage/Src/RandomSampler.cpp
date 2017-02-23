#include "RandomSampler.h"
#include "stdio.h" //FC

// Constructor - Set functions, MSS and points. Non uniform sampling type must be set later
RandomSampler::RandomSampler(
	std::vector<float> * (*nGetFunction)(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts), 
	float (*nDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt), 
	unsigned int nPtDimension,
	unsigned int nMSS, 
	unsigned int nMaxAllocablePoints,
	bool nCopyPtsCoords
	):
	mKDTree(nPtDimension),
	mDataPoints(),
	mNeighCustomVecF(),
	mNeighCustomVecFCumSum()
	{

	mMSS = nMSS; // Minimal sample set
	mGetFunction = nGetFunction; // Get function
	mDistanceFunction = nDistanceFunction; // Distance function

	if(nMaxAllocablePoints == 0){
		mGrowablePoints = true;
		mMaxAllocablePoints = 0;
	}
	else{
		mGrowablePoints = false;
		mMaxAllocablePoints = nMaxAllocablePoints;
		mFreeCells.resize(mMaxAllocablePoints);
		mDataPoints.resize(mMaxAllocablePoints);
		for(unsigned int i=0; i < mMaxAllocablePoints; i++){
			mFreeCells[i] = i;
			mDataPoints[i] = new sPt();
			mDataPoints[i]->mActive = false;
			mDataPoints[i]->mIndex = i;
			mDataPoints[i]->mCoord = NULL;
		}
		
		mNeighCustomVecF.resize(mMaxAllocablePoints, 0.0f);
		mNeighCustomVecFCumSum.resize(mMaxAllocablePoints, 0.0f);
		SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);

	}

	// Non-Uniform sampling parameters -- Set dummy values
	mSigmaExp = 1.0f;
	mDNeigh = 10;
	mNNPClose = 1.0f;
	mNNPFar = 2.0f;
	mNFSType = NFST_UNIFORM;
	mActivePoints = 0;
	mPtDimension = nPtDimension;
	mCopyPtsCoords = nCopyPtsCoords;

	#ifndef _DEBUG
		srand((unsigned int)time(NULL));
	#endif
}

// Destructor
RandomSampler::~RandomSampler(){	
	ClearDataPoints();
}

// Clear mDataPoints Vector
void RandomSampler::ClearDataPoints(){
	for(unsigned int i=0; i<mDataPoints.size(); i++){
		if(mDataPoints[i] != NULL){
			if(mCopyPtsCoords && mDataPoints[i]->mCoord != NULL)
				delete (mDataPoints[i]->mCoord);
			delete mDataPoints[i];
		}
	}
}

// Set the data Points
int RandomSampler::SetPoints(std::vector<std::vector<float> *> *nDataPoints){
	
	// Dummy input check
	assert(nDataPoints != NULL && nDataPoints->size() >= mMSS);
	if(nDataPoints == NULL || nDataPoints->size() <= mMSS)
		return -1;

	// Allocation infos
	if(mGrowablePoints)
		mMaxAllocablePoints = (unsigned int)(*nDataPoints).size();

	mFreeCells.clear();
	mFreeCells.reserve(mMaxAllocablePoints);

	// Clear already existing data points
	ClearDataPoints();

	unsigned int numOfPoints = (unsigned int)(*nDataPoints).size();
	unsigned int numOfPointsAllocable = std::min<int>(mMaxAllocablePoints, numOfPoints);

	mDataPoints.resize(numOfPointsAllocable);

	// Allocate sampling probability
	mNeighCustomVecF.clear();
	mNeighCustomVecFCumSum.clear();
	mNeighCustomVecF.resize(numOfPointsAllocable, 1.0f);
	mNeighCustomVecFCumSum.resize(numOfPointsAllocable, 0.0f);
	SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);

	// Allocate data
	for(unsigned int i=0; i < numOfPointsAllocable; i++){
		mDataPoints[i] = new sPt();
		mDataPoints[i]->mIndex = i;

		if(mCopyPtsCoords){
			mDataPoints[i]->mCoord = new std::vector<float>(*((*nDataPoints)[i]));
		}
		else{
			// Directly use the vector pointer
			mDataPoints[i]->mCoord = (*nDataPoints)[i];
		}
		if(i<numOfPoints){
			mDataPoints[i]->mActive = true;
		}
		else{
			mFreeCells.push_back(i);
			mDataPoints[i]->mActive = false;
		}

		// NFS infos
		mDataPoints[i]->mSecondSamplingProbability.clear();
		mDataPoints[i]->mSecondSamplingProbabilityCumSum.clear();

	}

	// Restore free cell indexes
	for(unsigned int i=numOfPointsAllocable; i < mMaxAllocablePoints; i++)
		mFreeCells[i-numOfPointsAllocable] = i;

	InitializeNFSampling();

	// Returns the number of points allocated
	mActivePoints = numOfPointsAllocable;

	return numOfPointsAllocable;
}	

// Add a new point to the class from an 3d-array. Works only if mCopyPtsCoords is set to true
int RandomSampler::AddPoint3d(double nCoordinate[]){

	if(!mCopyPtsCoords)
		return NULL;
	
	std::vector<float> *nCoordinatesVec = new std::vector<float>(3);
	(*nCoordinatesVec)[0] = (float) nCoordinate[0];
	(*nCoordinatesVec)[1] = (float) nCoordinate[1];
	(*nCoordinatesVec)[2] = (float) nCoordinate[2];
	mCopyPtsCoords = false;
	unsigned int toReturn = AddPoint(nCoordinatesVec);

	// No free space available
	if(toReturn == -1)
		delete nCoordinatesVec;

	mCopyPtsCoords = true;
	return toReturn;
}

// Add a new point to the class
int RandomSampler::AddPoint(std::vector<float> *nPointCoords){

	unsigned int nIndex;
	if(mGrowablePoints){
		if(mFreeCells.size()>0){
			// Found a free space
			nIndex = mFreeCells.back();
			mFreeCells.pop_back();
		}
		else{
			// Append new point at the end of the list and allocate new memory
			nIndex = (unsigned int)mDataPoints.size();
			if(mMaxAllocablePoints>0)
				mMaxAllocablePoints = mMaxAllocablePoints+(mMaxAllocablePoints/2);
			else
				mMaxAllocablePoints = 100;
			mDataPoints.resize(mMaxAllocablePoints);
			mFreeCells.reserve(mMaxAllocablePoints);
			
			// Allocate memory for new pts
			for (unsigned int i=nIndex; i<mMaxAllocablePoints; i++){
			
				mDataPoints[i] = new sPt();
				mDataPoints[i]->mIndex = i;
				mDataPoints[i]->mActive = false;
				mDataPoints[i]->mCoord = NULL;

				if(i != nIndex)
					mFreeCells.push_back(i);
			}

			// Allocate memory for nonfirst sampling pt
			mNeighCustomVecF.resize(mMaxAllocablePoints, 0.0f);
			mNeighCustomVecFCumSum.resize(mMaxAllocablePoints, 0.0f);
			SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);
		}


	}
	else{
		// No reallocation is necessary
		if(mFreeCells.size()>0){
			// Free space available
 			nIndex = mFreeCells[mFreeCells.size()-1];
			mFreeCells.pop_back();
		}
		else{
			return -1; // No free space available
		}
	}


	// Set the new point
	mDataPoints[nIndex]->mActive = true;
	// Set point coords
	if(mCopyPtsCoords){
		mDataPoints[nIndex]->mCoord = new std::vector<float>(*nPointCoords);
	}
	else{
		// Directly use the vector pointer
		mDataPoints[nIndex]->mCoord = nPointCoords;
	}

	mActivePoints++;
	AddPointUpdateProbability(nIndex);

	return nIndex;
}

// Remove a point from the cloud
bool RandomSampler::RemovePoint(unsigned int nIndex){


	if(nIndex >= mDataPoints.size()){
		return false;
	}

	//Get the point to delete and set inactive
	sPt *nPtStruct = mDataPoints[nIndex];
	if(nPtStruct->mActive)
		nPtStruct->mActive = false;
	else{
		return false;
	}
	// Update free cells info
	mActivePoints--;
	unsigned int i = 0;
	for(i = 0; i < mFreeCells.size(); i++){
		if(mFreeCells[i] > nIndex){
			mFreeCells.insert(mFreeCells.begin()+i, nIndex);
			break;
		}
	}
	if(i==mFreeCells.size())
		mFreeCells.push_back(nIndex);

	RemovePointUpdateProbability(nIndex);

	return true;
}

// Get point
const sPt *RandomSampler::GetPoint(unsigned int nIndex){

	return mDataPoints[nIndex];
}

// Manage probability updates if a new points is added
void RandomSampler::AddPointUpdateProbability(unsigned int nIndex){

	mNeighCustomVecF[nIndex] = 1.0f;
	SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);

	if((mNFSType == NFST_NN || mNFSType == NFST_EXP) && mMaxAllocablePoints != mDataPoints[0]->mSecondSamplingProbability.size() ){
		// Check for reallocation
		for(unsigned int i=0; i<(unsigned int)mMaxAllocablePoints; i++){

			if(i<mDataPoints.size()){
				// Reallocate pw distance
				if(mNFSType == NFST_EXP)
					mDataPoints[i]->mPairwiseEuclideanDistance.resize(mMaxAllocablePoints,0.0f);
			
				// Reallocate histograms
				mDataPoints[i]->mSecondSamplingProbability.resize(mMaxAllocablePoints,0.0f);			
				mDataPoints[i]->mSecondSamplingProbabilityCumSum.resize(mMaxAllocablePoints,0.0f); 
			}

		}	
	}

	// Case KD-Tree
	if(mNFSType == NFST_NN || mNFSType == NFST_NN_ME){
		sPtPointer ptPointer; ptPointer.mPt = (mDataPoints[nIndex]); ptPointer.mPt->mAlreadyFound = false;
		mKDTree.insert(ptPointer);
		mKDTree.optimise();
	
		if(mNFSType == NFST_NN){

			std::list<sPtPointer> kNeigh = FindKNearest(ptPointer,std::numeric_limits<float>::max(),mDNeigh, (int)mActivePoints);
			
			for(unsigned int i=0; i< mDataPoints[nIndex]->mSecondSamplingProbability.size(); i++){
				if(mDataPoints[i]->mActive){
					(mDataPoints[i]->mSecondSamplingProbability)[nIndex] = mNNPFar;
					(mDataPoints[nIndex]->mSecondSamplingProbability)[i] = mNNPFar;
				}
			}

			for(std::list<sPtPointer>::iterator it = kNeigh.begin(); it != kNeigh.end(); ++it){
				(mDataPoints[nIndex]->mSecondSamplingProbability)[(*it).mPt->mIndex] = mNNPClose;
				(mDataPoints[(*it).mPt->mIndex]->mSecondSamplingProbability)[nIndex] = mNNPClose;
				SetCumSumHist (&mDataPoints[(*it).mPt->mIndex]->mSecondSamplingProbabilityCumSum, mDataPoints[(*it).mPt->mIndex]->mSecondSamplingProbability);				
			}
			
			for(unsigned int i=0; i< mDataPoints.size(); i++){
				if(mDataPoints[i]->mActive){
					SetCumSumHist (&mDataPoints[i]->mSecondSamplingProbabilityCumSum, mDataPoints[i]->mSecondSamplingProbability);				
				}
			}			
		}
	}

	// Exponential case
	if(mNFSType == NFST_EXP){
		// update pairwise euclidean distance beetween points
		float eucDist = 0.0f;
		float expTemp = 0.0f;
		(mDataPoints[nIndex]->mPairwiseEuclideanDistance)[nIndex] = 0.0f;
		for(unsigned int j=0; j < mDataPoints.size(); j++){
			if(!mDataPoints[j]->mActive)
				continue;
			// Compute euclidean distance
			eucDist = VecEuclideanDist(*mDataPoints[nIndex]->mCoord, *mDataPoints[j]->mCoord);
			(mDataPoints[nIndex]->mPairwiseEuclideanDistance)[j] = eucDist;
			(mDataPoints[j]->mPairwiseEuclideanDistance)[nIndex] = eucDist;
			
			if(j == nIndex)
				expTemp = 0.0f;
			else
				expTemp = exp(-mSigmaExp/eucDist);

			(mDataPoints[nIndex]->mSecondSamplingProbability)[j] = expTemp;
			(mDataPoints[j]->mSecondSamplingProbability)[nIndex] = expTemp;

			// Reset cumsum pdf
			SetCumSumHist (&mDataPoints[j]->mSecondSamplingProbabilityCumSum, mDataPoints[j]->mSecondSamplingProbability);
		}
		SetCumSumHist (&mDataPoints[nIndex]->mSecondSamplingProbabilityCumSum, mDataPoints[nIndex]->mSecondSamplingProbability);
	}
}

void RandomSampler::RemovePointUpdateProbability(unsigned int nIndex){

	mNeighCustomVecF[nIndex] = 0.0f;
	SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);

	// Case KD-Tree
	if(mNFSType == NFST_NN || mNFSType == NFST_NN_ME){

		if(mNFSType == NFST_NN){
			for(unsigned int i=0; i< mDataPoints[nIndex]->mSecondSamplingProbability.size(); i++){
				(mDataPoints[i]->mSecondSamplingProbability)[nIndex] = 0.0f;
				(mDataPoints[nIndex]->mSecondSamplingProbability)[i] = 0.0f;
				SetCumSumHist (&mDataPoints[i]->mSecondSamplingProbabilityCumSum, mDataPoints[i]->mSecondSamplingProbability);
				
			}
			
			SetCumSumHist (&mDataPoints[nIndex]->mSecondSamplingProbabilityCumSum, mDataPoints[nIndex]->mSecondSamplingProbability);
		}
		sPtPointer ptPointer; ptPointer.mPt = (mDataPoints[nIndex]); ptPointer.mPt->mAlreadyFound = false;
		mKDTree.erase(ptPointer);
		mKDTree.optimise();
	}

	// Exponential case
	if(mNFSType == NFST_EXP){
		for(unsigned int j=0; j < mDataPoints.size(); j++){
			// Compute euclidean distance
			(mDataPoints[nIndex]->mSecondSamplingProbability)[j] = 0.0f;
			(mDataPoints[j]->mSecondSamplingProbability)[nIndex] = 0.0f;
			// Reset cumsum pdf
			SetCumSumHist (&mDataPoints[j]->mSecondSamplingProbabilityCumSum, mDataPoints[j]->mSecondSamplingProbability);
		}
	}

}


// Set probability for the non-first sampling as uniform
void RandomSampler::SetNFSamplingTypeUniform(){

	mNFSType = NFST_UNIFORM;
}

// Set Non first sampling type as Exp
void RandomSampler::SetNFSamplingTypeExp(float nSigmaExp){
	

	mSigmaExp = nSigmaExp;
	mNFSType = NFST_EXP;


	// Allocate necessary space
	for(unsigned int i=0; i < mMaxAllocablePoints; i++){
		
		mDataPoints[i]->mPairwiseEuclideanDistance.resize(mMaxAllocablePoints, 0.0f);

		mDataPoints[i]->mSecondSamplingProbability.resize(mMaxAllocablePoints, 0.0f);

		mDataPoints[i]->mSecondSamplingProbabilityCumSum.resize(mMaxAllocablePoints, 0.0f);
	}	

	// Some temp values
	float eucDist = 0.0f;
	float expTemp = 0.0f;

	// Compute pairwise euclidean distance beetween points
	for(unsigned int i=0; i<mMaxAllocablePoints; i++){
		if(!mDataPoints[i]->mActive)
			continue;
		(mDataPoints[i]->mPairwiseEuclideanDistance)[i] = 0.0f;
		for(unsigned int j=i+1; j < mMaxAllocablePoints; j++){
			if(!mDataPoints[j]->mActive)
				continue;
			// Compute euclidean distance
			eucDist = VecEuclideanDist(*mDataPoints[i]->mCoord, *mDataPoints[j]->mCoord);
			(mDataPoints[i]->mPairwiseEuclideanDistance)[j] = eucDist;
			(mDataPoints[j]->mPairwiseEuclideanDistance)[i] = eucDist;
			expTemp = exp(-nSigmaExp/eucDist);
			(mDataPoints[i]->mSecondSamplingProbability)[j] = expTemp;
			(mDataPoints[j]->mSecondSamplingProbability)[i] = expTemp;
		}
	}

	for(unsigned int i=0; i<mMaxAllocablePoints; i++){
		if(!mDataPoints[i]->mActive)
			continue;
		SetCumSumHist(&mDataPoints[i]->mSecondSamplingProbabilityCumSum, mDataPoints[i]->mSecondSamplingProbability);
	}

}

void RandomSampler::SetNFSamplingTypeNN(int nDNeigh, float nPClose, float nPFar, bool nMemoryEfficient ){

	if(nMemoryEfficient)
		mNFSType = NFST_NN_ME;
	else
		mNFSType = NFST_NN;
	
	mDNeigh = nDNeigh;
	mNNPClose = nPClose / (nPClose + nPFar);
	mNNPFar = nPFar / (nPClose + nPFar);

	
	mKDTree.clear();

	// Add points to the kd-tree
	for(unsigned int i=0; i < mMaxAllocablePoints; i++){
		if(mDataPoints[i]->mActive){
			sPtPointer ptPointer; ptPointer.mPt = (mDataPoints[i]); ptPointer.mPt->mAlreadyFound = false;
			mKDTree.insert(ptPointer);
		}
	}

	mKDTree.optimise();
	
	if(mNFSType == NFST_NN){
		std::vector<sPtPointer> nearPtsIdx;
		nearPtsIdx.reserve(mMaxAllocablePoints);
		for(unsigned int i=0; i < mMaxAllocablePoints; i++){
			if(mDataPoints[i]->mActive){
				nearPtsIdx.clear();
				
				mDataPoints[i]->mSecondSamplingProbability.resize(mMaxAllocablePoints, 0.0f);
				mDataPoints[i]->mSecondSamplingProbabilityCumSum.resize(mMaxAllocablePoints, 0.0f);
				
				sPtPointer ptPointer; ptPointer.mPt = (mDataPoints[i]); ptPointer.mPt->mAlreadyFound = false;

				std::list<sPtPointer> kNeigh = FindKNearest(ptPointer,std::numeric_limits<float>::max(),mDNeigh, (int)mActivePoints);
				
				for(unsigned int j=0; j< mDataPoints[i]->mSecondSamplingProbability.size(); j++){
					if(mDataPoints[j]->mActive)
						(mDataPoints[i]->mSecondSamplingProbability)[j] = mNNPFar ;
				}
		
				for(std::list<sPtPointer>::iterator it = kNeigh.begin(); it != kNeigh.end(); ++it){
					(mDataPoints[i]->mSecondSamplingProbability)[(*it).mPt->mIndex] = mNNPClose ;
				}
				
				SetCumSumHist (&mDataPoints[i]->mSecondSamplingProbabilityCumSum, mDataPoints[i]->mSecondSamplingProbability);
			}
		}	
	}
	

}

void RandomSampler::InitializeNFSampling(){

	switch(mNFSType){
		case NFST_UNIFORM:
				SetNFSamplingTypeUniform();
			break;
		case NFST_EXP:
				SetNFSamplingTypeExp(mSigmaExp);
			break;
		case NFST_NN:
				SetNFSamplingTypeNN(mDNeigh, mNNPClose, mNNPFar, false );
		case NFST_NN_ME:
			SetNFSamplingTypeNN(mDNeigh, mNNPClose, mNNPFar, true );
			break;
		default:
				assert(1 || printf("Invalid NFS"));
			break;
	}

}

	// Get(And Set) The vector of Custom sampling type for the first selection
double RandomSampler::GetFirstSamplingProb(unsigned int elIndex) const{
	assert(elIndex < mNeighCustomVecF.size());
	return mNeighCustomVecF[elIndex];
}
void RandomSampler::SetFirstSamplingProb(unsigned int elIndex, float newValue){
	assert(elIndex < mNeighCustomVecF.size());
	mNeighCustomVecF[elIndex] = newValue;
	SetCumSumHist (&mNeighCustomVecFCumSum, mNeighCustomVecF);
}
unsigned int RandomSampler::GetNumberOfLoadedDataPoints(){

	return mActivePoints;
}

unsigned int RandomSampler::GetActiveIndex(unsigned int nIndex) const{
	
	assert(nIndex < mActivePoints);

	unsigned int i = 0;
	unsigned int jIndex = 0;
	// mFreeCells are supposed to be sorted
	while(true){
		if(jIndex == nIndex && mDataPoints[i]->mActive)
			break;

		if(mDataPoints[i]->mActive)
			jIndex++;

		i++;
		
	}

	return i;

}

// Get preferences set from N samples
std::vector<bm::bvector<> *> *RandomSampler::GetNSampleAndPreferenceSet(float nInlierThreshold, unsigned int nSampleN, std::vector<bm::bvector<> *> *nPrevPointsPrefSet, unsigned int nModelSpanN, std::vector<std::vector<float> *> *nPrevModels, void (*OnProgress)(float)){

	// TODO: Add multithreading support to this function.

	// Allocate empty bit vector
	if(nPrevPointsPrefSet == NULL){
		nPrevPointsPrefSet = new std::vector<bm::bvector<> *>(mMaxAllocablePoints);
		for(unsigned int i=0; i < mMaxAllocablePoints; i++)
			(*nPrevPointsPrefSet)[i] = new bm::bvector<>;
	}

	assert(nPrevPointsPrefSet->size() == mMaxAllocablePoints);

	if(this->mActivePoints < this->mMSS){
		return nPrevPointsPrefSet;
	}

	// Get all the sample
	for(unsigned int n=0; n<nSampleN; n++){

		unsigned int currentModelIndex = nModelSpanN + n;

		std::vector<unsigned int> nSample(mMSS);

		if(OnProgress != NULL)
			OnProgress((float)n / (float)nSampleN);
		
		#ifdef _DEBUG
		// If we are in debug mode, generate hypotesis in a non random way
			static unsigned int nCounter = 0;
			nCounter = nCounter % this->mActivePoints;
			nSample[0] = (unsigned int)GetActiveIndex(nCounter);
			nCounter++;
		#else
			nSample[0] = (unsigned int)RandomSampleOnCumSumHist (mNeighCustomVecFCumSum);
		#endif

		GetNonFirstSamples(&nSample);
		
		std::vector<float> *nModelParams =	mGetFunction(mDataPoints, nSample);
		for(unsigned int i=0; i < mMaxAllocablePoints; i++) {
			if(mDataPoints[i]->mActive && (*mDistanceFunction)(*nModelParams, *mDataPoints[i]->mCoord) <nInlierThreshold ){
				// Fill bit vector
				assert(!(*(*nPrevPointsPrefSet)[i])[currentModelIndex]);
				(*(*nPrevPointsPrefSet)[i])[currentModelIndex] = true;
			}
		}
		if(nPrevModels == NULL)
			delete (nModelParams);
		else
			(*nPrevModels)[currentModelIndex] = nModelParams;

	}
	
	for(unsigned int i=0; i<mMaxAllocablePoints; i++)
		if(mDataPoints[i]->mActive)
			(*nPrevPointsPrefSet)[i]->optimize();

	return nPrevPointsPrefSet;
}

// Get preferences set from N samples
std::vector<std::vector<float> *> *RandomSampler::GetNSample(unsigned int nSampleN, unsigned int nModelSpanN, std::vector<std::vector<float> *> *nPrevModels, void (*OnProgress)(float)){

	if(this->mActivePoints < this->mMSS){
		return NULL;
	}

	// Allocate empty bit vector
	if(nPrevModels == NULL)
		nPrevModels = new std::vector<std::vector<float> *>(nSampleN + nModelSpanN);

	if(nPrevModels->size() < nSampleN + nModelSpanN)
		nPrevModels->resize(nSampleN + nModelSpanN);

#ifndef RS_NO_THREADS
	boost::thread_group *tg = new boost::thread_group();
#endif RS_NO_THREADS
		
	// Get all the samples
	for(unsigned int n=0; n<nSampleN; n++){

		unsigned int currentModelIndex = nModelSpanN + n;

		std::vector<unsigned int> nSample(mMSS);

		if(OnProgress != NULL)
			OnProgress((float)n / (float)nSampleN);

	#ifdef RS_NO_THREADS

		#ifdef _DEBUG
		// If we are in debug mode, generate hypotesis in a non random way
			static unsigned int nCounter = 0;
			nCounter = nCounter % this->mActivePoints;
			nSample[0] = (unsigned int)GetActiveIndex(nCounter);
			nCounter++;
		#else
			nSample[0] = (unsigned int)RandomSampleOnCumSumHist (mNeighCustomVecFCumSum);
		#endif
		GetNonFirstSamples(&nSample);

		std::vector<float> *nModelParams =	mGetFunction(mDataPoints, nSample);

		(*nPrevModels)[currentModelIndex] = nModelParams;
	#else

		tg->create_thread( boost::bind(&RandomSampler::GetSampleMultiThreadWrapper,this, nSample, currentModelIndex,nPrevModels, -1));

		if(tg->size() >= MAXTHREADS){
			tg->join_all();
			delete tg;
			tg = new boost::thread_group();
		}
	#endif

	}

#ifndef RS_NO_THREADS
	if(tg->size() > 0)
		tg->join_all();
	delete tg;
#endif RS_NO_THREADS

	return nPrevModels;

}

// Get preferences set from N samples
std::vector<std::vector<float> *> *RandomSampler::GetNSampleFromStartingPoint(unsigned int nSampleN, unsigned int nStartingPoint, unsigned int nModelSpanN, std::vector<std::vector<float> *> *nPrevModels, void (*OnProgress)(float) ){


	// Allocate empty bit vector
	if(nPrevModels == NULL)
		nPrevModels = new std::vector<std::vector<float> *>(nSampleN + nModelSpanN);

	if(nPrevModels->size() < nSampleN + nModelSpanN)
		nPrevModels->resize(nSampleN + nModelSpanN);

#ifndef RS_NO_THREADS
	boost::thread_group *tg = new boost::thread_group();
#endif RS_NO_THREADS
		
	// Get all the samples
	for(unsigned int n=0; n<nSampleN; n++){

		unsigned int currentModelIndex = nModelSpanN + n;

		std::vector<unsigned int> nSample(mMSS);

		if(OnProgress != NULL)
			OnProgress((float)n / (float)nSampleN);

	#ifdef RS_NO_THREADS
		
		nSample[0] = nStartingPoint;

		GetNonFirstSamples(&nSample);
		
		std::vector<float> *nModelParams =	mGetFunction(mDataPoints, nSample);

		(*nPrevModels)[currentModelIndex] = nModelParams;

	#else

		tg->create_thread( boost::bind(&RandomSampler::GetSampleMultiThreadWrapper,this, nSample, currentModelIndex,nPrevModels, nStartingPoint));

		if(tg->size() >= MAXTHREADS){
			tg->join_all();
			delete tg;
			tg = new boost::thread_group();
		}

	#endif

	}
	

#ifndef RS_NO_THREADS
	if(tg->size() > 0)
		tg->join_all();
	delete tg;
#endif RS_NO_THREADS

	return nPrevModels;
}

// Get non first samples - The first sample must be set in the nSample vector
void RandomSampler::GetNonFirstSamples(std::vector<unsigned int> *nSample) {

	assert(nSample->size() == mMSS);

	unsigned int nDegenerateCases = 0;
	for(unsigned int i=1; i<mMSS; i++){
		bool foundUnique = false;
		while(!foundUnique){
			// Check if degenerate max n has been reached
			if(nDegenerateCases > MAXDEGENERATESAMPLES){
				assert(1 || printf("Max number of degenerate samples reached"));
				for(unsigned int i=1; i<mMSS; i++){
					(*nSample)[i] = GetActiveIndex(GetActiveIndex((*nSample)[i-1] + 1) % mActivePoints);
				}
				return;
			}

			if(mNFSType == NFST_UNIFORM){	
					#ifdef _DEBUG
					// If we are in debug mode, generate hypotesis in a non random way
						static unsigned int nCounter = 0;
						nCounter = nCounter % this->mActivePoints;
						(*nSample)[i] = (unsigned int)GetActiveIndex(nCounter);
						nCounter++;
					#else
						(*nSample)[i] = GetActiveIndex((unsigned int) (((float)rand()/(float)RAND_MAX)*(float)(mActivePoints-1)));			
					#endif

			}
			else if (mNFSType == NFST_EXP || mNFSType == NFST_NN ){	
						#ifdef _DEBUG
						// If we are in debug mode, generate hypotesis in a non random way
							static unsigned int nCounter = 0;
							nCounter = nCounter % this->mActivePoints;
							(*nSample)[i] = (unsigned int)GetActiveIndex(nCounter);
							nCounter++;
						#else
							(*nSample)[i] = (unsigned int)RandomSampleOnCumSumHist (mDataPoints[(*nSample)[0]]->mSecondSamplingProbabilityCumSum);
						#endif
			}
			else if (mNFSType == NFST_NN_ME){
				#ifdef _DEBUG
				static unsigned int nCounterOut = 0;
				nCounterOut = nCounterOut % 100;
				nCounterOut++;
				if((float)nCounterOut/100.f < mNNPClose){
				#else
				if(((float)rand()/(float)RAND_MAX) < mNNPClose){
				#endif
					
					
					sPtPointer ptPointer; ptPointer.mPt = ((mDataPoints[(*nSample)[0]])); ptPointer.mPt->mAlreadyFound = false;

					std::list<sPtPointer> kneigh = FindKNearest(ptPointer,std::numeric_limits<float>::max(),mDNeigh, (int)mActivePoints);
					int size = (int)kneigh.size();
					if(size > (int)mMSS){
						#ifdef _DEBUG
						// If we are in debug mode, generate hypotesis in a non random way
							static unsigned int nCounter = 0;
							nCounter = nCounter % size;
							std::list<sPtPointer>::iterator kneighIt = kneigh.begin();
							unsigned h = 0;
							while(h < nCounter && kneighIt != kneigh.end()){
								++kneighIt;
								++h;
							}

							(*nSample)[i] = (*kneighIt).mPt->mIndex;		
							nCounter++;
						#else			
							unsigned int idx = 1+(unsigned int) (((float)rand()/(float)RAND_MAX)*(float)(size-2));
							std::list<sPtPointer>::iterator kneighIt = kneigh.begin();
							unsigned h = 0;
							while(h < idx && kneighIt != kneigh.end()){
								++kneighIt;
								++h;
							}

							(*nSample)[i] = (*kneighIt).mPt->mIndex;		
						#endif					
					}
					else{
						#ifdef _DEBUG
						// If we are in debug mode, generate hypotesis in a non random way
							static unsigned int nCounter = 0;
							nCounter = nCounter % this->mActivePoints;
							(*nSample)[i] = (unsigned int)GetActiveIndex(nCounter);
							nCounter++;
						#else
							(*nSample)[i] = GetActiveIndex((unsigned int) (((float)rand()/(float)RAND_MAX)*(float)(mActivePoints-1)));					
						#endif
					}
				}
				else{
					#ifdef _DEBUG
					// If we are in debug mode, generate hypotesis in a non random way
						static unsigned int nCounter = 0;
						nCounter = nCounter % this->mActivePoints;
						(*nSample)[i] = (unsigned int)GetActiveIndex(nCounter);
						nCounter++;
					#else
						(*nSample)[i] = GetActiveIndex((unsigned int) (((float)rand()/(float)RAND_MAX)*(float)(mActivePoints-1)));					
					#endif
			}
							
				
			}
			foundUnique = true;
			for(unsigned int j=0; j<i; j++){
				if((*nSample)[i] == (*nSample)[j]){
					foundUnique = false;
					nDegenerateCases++;
					break;
				}
			}
		}

	}
}

std::list<sPtPointer> RandomSampler::FindKNearest(sPtPointer __V, float const __Max_R, int k, int nMaxSize){
		int msSizePRAF = 0;
		std::list<sPtPointer> msAlreadyFoundPRAF;
		__V.mPt->mAlreadyFound = true;
		while(msSizePRAF < k && msSizePRAF < nMaxSize ){
			std::pair<KDTree::KDTree<sPtPointer>::iterator,float> nif = mKDTree.find_nearest_if(__V,__Max_R,sPredicateAlreadyFoundRS());
			msAlreadyFoundPRAF.push_back(*nif.first);
			msAlreadyFoundPRAF.back().mPt->mAlreadyFound = true;
			msSizePRAF++;
		}
		// Reset values
		for(std::list<sPtPointer>::iterator it = msAlreadyFoundPRAF.begin(); it != msAlreadyFoundPRAF.end(); ++it)
			(*it).mPt->mAlreadyFound = false;
		__V.mPt->mAlreadyFound = false;

	  return msAlreadyFoundPRAF;
}


void RandomSampler::GetSampleMultiThreadWrapper(RandomSampler *nRandomSampler, std::vector<unsigned int> &nSample, unsigned int nCurModelIdx, std::vector<std::vector<float> *> *nModelPt, int nStartingPoint){
	
	if(nStartingPoint < 0)
		nSample[0] = (unsigned int)RandomSampleOnCumSumHist (nRandomSampler->mNeighCustomVecFCumSum);
	else
		nSample[0] = nStartingPoint;

	nRandomSampler->GetNonFirstSamples(&nSample);

	std::vector<float> *nModelParams =	nRandomSampler->mGetFunction(nRandomSampler->mDataPoints, nSample);

	(*nModelPt)[nCurModelIdx] = nModelParams;

}
