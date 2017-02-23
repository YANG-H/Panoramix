#ifndef RANDOMSAMPLER_H
#define RANDOMSAMPLER_H

// Simply uncomment this line if you don't want to use threads
#define RS_NO_THREADS
// TODO WATCHOUT THREADS ARE BUGGED

#include <vector>
#include <assert.h>
#include <ctime>
#include <kdtree++/kdtree.hpp>
#include <bm/bm.h>
#include <limits>
#include "Utilities.h"

#ifndef RS_NO_THREADS
#define MAXTHREADS 8
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#endif

// Max degenerate samples before skipping
#define MAXDEGENERATESAMPLES 1000

// Type of sampling of the non-first element
// Currently there are three type of sampling implemented: Uniform, exponential, and kd-tree nearest neighbor
enum RS_NFSAMPLINGTYPE{
	NFST_UNIFORM,
	NFST_EXP,
	NFST_NN,
	NFST_NN_ME,
	NFST_SIZE
};

// This structure will store a single point
// Will store his coordinates, 
// - the active status(inactive points are not used for sampling)
// - non uniform sampling additional informations
typedef struct {
	
	std::vector<float> *mCoord;
	bool mActive;
	unsigned int mIndex;

	// The following vars are used only for the Exp NFST
	std::vector<float> mPairwiseEuclideanDistance;
	std::vector<float> mSecondSamplingProbability;
	std::vector<float> mSecondSamplingProbabilityCumSum;

	// Used for find k nearest ..
	bool mAlreadyFound;


}sPt;

typedef struct {
	sPt *mPt;
	
	// Used for the kd-tree
	typedef float value_type;
    inline value_type operator[](size_t const N) const { return (*(mPt->mCoord))[N]; }
		
}sPtPointer;

// Used for the kd-tree
inline bool operator==(sPtPointer const& A, sPtPointer const& B) {
  if(A.mPt == B.mPt)
		  return true;
  return false;
}

// Predicate to check if a node was already extracted in k-nn
struct sPredicateAlreadyFoundRS{

	bool operator()( const sPtPointer& A ) const{
		return !A.mPt->mAlreadyFound;
	}
};


/********************************************************
* Class RandomSampler.
* Define a RandomSampler Object. 
* A function that returns the model from a MSS(minimum sample set) of(float) coordinates 
* and a function that calculates the distance between a point and a model must be provided
* 
* Add and or remove point from the sampler any time you want. If you specify a number of 
* max allocable points equal to 0, this means that the structure grow automatically to store new points.
* Set the initial points with the -SetPoints- function and get the sample any time you want with the -Get*Sample*- functions
* Specify a non uniform first sampling by setting the probability vector with the function -SetFirstSamplingProb-
* Specify the non-first sampling probability(function of the first point extracted) using the methods -SetNFSamplingType*-
**********************************************************/
class RandomSampler{

public:
	// Constructor
	RandomSampler(std::vector<float> * (*nGetFunction)(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts), // Get function
		float (const std::vector<float>  &nModel, const std::vector<float>  &nDataPt), // Distance function
		unsigned int nPtDimension, // Dimension of the point space (used by the Kd-Tree) 
		unsigned int nMSS,  // number of Minimun sample set
		unsigned int nmMaxAllocablePoints = 0, // Number of max allocable pts - Set 0 to allowable a growable size
		bool nCopyPtsCoords = true // Use the pointer or copy the data when a new point is added? Please note that if the pointer is used no deallocation will occour on destructors
		);
	// Destructor
	~RandomSampler();
	
	// Data points Setters/Getters
    int SetPoints(std::vector<std::vector<float> *> *nDataPoints);
	int AddPoint(std::vector<float> *nPointCoords);
	int AddPoint3d(double nCoordinate[]);
	bool RemovePoint(unsigned int nIndex);
	const sPt *GetPoint(unsigned int nIndex);
	unsigned int GetNumberOfLoadedDataPoints();

	// Set sampling types
	void SetNFSamplingTypeUniform();	
	void SetNFSamplingTypeExp(float nSigmaExp);
	void SetNFSamplingTypeNN(int nDNeigh, float nPClose, float nPFar, bool nMemoryEfficient);

	// Get(And Set) The vector of Custom sampling type for the first selection
	double GetFirstSamplingProb(unsigned int elIndex) const;
	void SetFirstSamplingProb(unsigned int elIndex, float newValue);


	// Get N random samples
	// returns a vector of bit containing the Preference set of each point. All the needed memory are allocated inside this function, so you need to deallocate it manually later.
	// You can also specify an already existing vector of preference set. In this case no further allocation is done and the newModels are indexed starting from nModelSpanN value.
	// Additionally you can specify a vector to store the parameters of each model sample. The model parameter are allocate in nPrevModels[i + nPrevModels]. All the deallocation(also of the previous value of the vector) must be done outside this method.
	std::vector<bm::bvector<> *> *GetNSampleAndPreferenceSet(float nInlierThreshold, unsigned int nSampleN, std::vector<bm::bvector<> *> *nPrevPointsPrefSet = NULL, unsigned int nModelSpanN = 0, std::vector<std::vector<float> *> *nPrevModels = NULL, void (*OnProgress)(float) = NULL);

	// Get N random samples
	// returns the generated models
	// You can specify a previous vector to store the parameters of each model sample. The model parameter are allocate in nPrevModels[i + nPrevModels]. All the deallocation(also of the previous value of the vector) must be done outside this method.
	std::vector<std::vector<float> *> *GetNSample(unsigned int nSampleN, unsigned int nModelSpanN, std::vector<std::vector<float> *> *nPrevModels = NULL, void (*OnProgress)(float) = NULL);

	// Get N random samples from a specifiedStartingPoint
	// returns the generated models
	// You can specify a previous vector to store the parameters of each model sample. The model parameter are allocate in nPrevModels[i + nPrevModels]. All the deallocation(also of the previous value of the vector) must be done outside this method.
	std::vector<std::vector<float> *> *GetNSampleFromStartingPoint(unsigned int nSampleN, unsigned int nStartingPoint, unsigned int nModelSpanN = 0, std::vector<std::vector<float> *> *nPrevModels = NULL, void (*OnProgress)(float) = NULL);

	// Find k-nearest neighboard in kd-tree
	std::list<sPtPointer> RandomSampler::FindKNearest(sPtPointer __V, float const __Max_R, int k, int nMaxSize);

private:	
	// Clear DataPoints vector
	void ClearDataPoints();

	// Helper method for updating first sampling and non first sampling probability additional vars
	void AddPointUpdateProbability(unsigned int nIndex);
	void RemovePointUpdateProbability(unsigned int nIndex);
	void UpdateAllProbability();	
	void InitializeNFSampling();
	unsigned int GetActiveIndex(unsigned int nIndex) const;


	// Get non first samples
	void GetNonFirstSamples(std::vector<unsigned int> *nFirstFilled);

	// Data Points
	std::vector<sPt *> mDataPoints;

	// Check if use pointer or copy coordinates when a point is added
	bool mCopyPtsCoords;

	// Wrapper for multi-threading.
	void static GetSampleMultiThreadWrapper(RandomSampler *nRandomSampler, std::vector<unsigned int> &nSample, unsigned int nCurModelIdx, std::vector<std::vector<float> *> *nModelPt, int nStartingPoint);

	// Non-Uniform sampling parameters
	RS_NFSAMPLINGTYPE mNFSType;
	float mSigmaExp;
	int mDNeigh;
	float mNNPClose;
	float mNNPFar;
	std::vector<float> mNeighCustomVecF;
	std::vector<float> mNeighCustomVecFCumSum;
	unsigned int mMSS; // Minimal sample set
	KDTree::KDTree<sPtPointer> mKDTree;
	
	// Allocation infos
	bool mGrowablePoints;
	unsigned int mMaxAllocablePoints;
	std::vector<unsigned int> mFreeCells;
	unsigned int mActivePoints;
	unsigned int mPtDimension;

	// Function pointers to modelGetFunction and distance function
	std::vector<float>  *(*mGetFunction)(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts);
	float (*mDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt);

};

#endif