#ifndef JLINKAGE_H
#define JLINKAGE_H

// Simply uncomment this line if you don't want to use threads
#define JL_NO_THREADS

#include <vector>
#include <list>
#include <functional>
#include <algorithm>
#include <limits>
#include <bm/bm.h>
#include <kdtree++/kdtree.hpp>
#include "Utilities.h"

#ifndef JL_NO_THREADS
#define MAXTHREADS 8
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#endif

struct sClLnk;

// bVector in memory saving mode
typedef bm::bvector<bm::standard_allocator, bm::miniset<bm::block_allocator, bm::set_total_blocks> > bvect;
// bVector in standard mode (faster, but requires more memory)
//typedef bm::bvector<> bvect;

// Store data points information
struct sPtLnk{
	std::vector<float>  *mCoord;
	bvect mPreferenceSet;
	sClLnk *mBelongingCluster;
	bool mToBeUpdateKDTree;
	unsigned int mAddedIdx;
	// Used for find k nearest ..
	bool mAlreadyFound;
};

struct sPtLnkPointer{
	sPtLnk *mPtLnk;
	// Used for the kd-tree
	typedef float value_type;
    inline value_type operator[](size_t const N) const { return (*(mPtLnk->mCoord))[N]; }

};
// Used for the kd-tree
inline bool operator==(sPtLnkPointer const& A, sPtLnkPointer const& B) {
	if(A.mPtLnk == B.mPtLnk)
		return true;
	return false;
}

// Predicate to check if a node was already extracted in k-nn
struct sPredicateAlreadyFoundJL{

	bool operator()( const sPtLnkPointer& A ) const{
		return !A.mPtLnk->mAlreadyFound;
	}
};

struct sDist;

// Store cluster information
struct sClLnk{
	bvect mPreferenceSet;
	std::list<sDist *>  mPairwiseJaccardDistance;
	std::list<sPtLnk *> mBelongingPts;
	void *mOtherDataCl; // Pointer to other user-defined data. May be useful for user-defined cluster-cluster test
};

// Store the jaccard distance beetwen two cluster
struct sDist{
	sClLnk *mCluster1;
	sClLnk *mCluster2;
	float mPairwiseJaccardDistance;
	unsigned int mPairwiseUnion;
	unsigned int mPairwiseIntersection;
	bool mToBeUpdated;

    bool operator()(const sDist *left, const sDist *right) const{return left->mPairwiseJaccardDistance < right->mPairwiseJaccardDistance;}
	
};

// JLinkage class, 
class JLinkage{

public:

	// Costructor and destructor
	JLinkage(
		float (*nDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt), // The Distance function
		float nInliersThreshold, // Scale of the algorithm
		unsigned int nModelBufferSize = 0, // Model buffer size, set to 0 to allow growable size
		bool nCopyCoord = true, // Copy the coords value of the new points or use directly the pointer? (The second choice is faster but you need to deallocate memory outside the class)		
		unsigned int nPtDimension = 3, //Used by Kd-Tree
		int nKNeighboards = -1,  //K-neighboards for each points to be used for the clusterization (neg value means use all the dataset)
		bool (*nClusterClusterDiscardTest)(const sClLnk *, const sClLnk *) = NULL, // Function handler to perform a cluster-cluster test. Should return true if the cluster can be merged by some additional condition, false otherwise.
		void (*nInitializeAdditionalData)(sClLnk *) = NULL, // Function handler -  initialization routine when additional data are created
		void (*nClusterMergeAdditionalOperation)(const sClLnk *) = NULL, // Function handler - called when two clusters are merged
		void (*nDestroyAdditionalData)(sClLnk *) = NULL // Function handler -  called when additional data are destroyed
	);
	~JLinkage();
	
	// Data Points accessor
	sPtLnk *AddPoint(std::vector<float>  *nCoordinate);
	sPtLnk *AddPoint3d(double nCoordinate[]);
	void RemovePoint(sPtLnk *);
	unsigned int AddModel(std::vector<float>  *nModel);
    const std::vector<float>  *GetModel(unsigned int nModel);
	unsigned int GetClusterN() const{return (unsigned int)mDataClusters.size();}
	unsigned int GetPointsN() const{return (unsigned int)mDataPointsSize;}
	unsigned int GetModelsN() const{return (unsigned int)mModels.size();}
	
	// The following functions are fast but unsafe. Use them wisely
	std::list<sPtLnk *> *GetPoints(){return &mDataPoints;}
	std::list<sClLnk *> *GetClusters(){return &mDataClusters;}
	std::vector<std::vector<float> *> *GetModels(){return &mModels;}
	KDTree::KDTree<sPtLnkPointer> *GetKdTree(){return &mKDTree;}
	bool ManualClusterMerge(std::vector<unsigned int> nPtIndexOfCluster);

	// Do a clusterization, return the final clusters
	const std::list<sClLnk *> DoJLClusterization(void (*OnProgress)(float) = NULL);


	// Find k-nearest neighboard in kd-tree
	std::list<sPtLnkPointer> JLinkage::FindKNearest(sPtLnkPointer __V, float const __Max_R, int k, int nMaxSize);

private:	

	void UpdateJaccardDistances();

	// Helper threadable functions
	static void UpdateDistance(sDist * tIterDist, float *nCurrentInvalidatingDistance, bool (*nClusterClusterDiscardTest)(const sClLnk *, const sClLnk *));

	// Max models sizes, set to 0 to allow list
	unsigned int mModelBufferSize;
	unsigned int mCurrentModelPointer;

	void RemoveModel(unsigned int nModelN);

	bool mCopyPtsCoords;

	// Scale of the algorithm
	float mInliersThreshold;

	// Initial Data Points
	std::list<sPtLnk *> mDataPoints;
	unsigned int mDataPointsSize;

	// Current clusters
	std::list<sClLnk *> mDataClusters;

	// Current models
	std::vector<std::vector<float> *> mModels;

	// Store the distances that have to be updated
	std::list<sDist *> mDistancesToBeUpdated;
	int mDistancesToBeUpdatedSize;

	// Distance function
	float (*mDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt);

	// Mode k-nearest neighboard to find
	int mKNeighboards;
	
	// if the Number of neighboards is set(>0), we need a kd-tree to store the neighorhood information
	KDTree::KDTree<sPtLnkPointer> mKDTree;
	
	// Flag if a Kd-Tree have to be be used
	bool mUseKDTree;

	// Function handler to perform a cluster-cluster test. Should return true if the clusters can be merged togheter, false otherwise.
	// The test is additive to the one already performed inside J-Linkage. If function pointer is null, simply it's ignored
	bool (*mClusterClusterDiscardTest)(const sClLnk *, const sClLnk *);

	void (*mInitializeAdditionalData)(sClLnk *);// Function handler -  initialization routine when additional data are created
	void (*mClusterMergeAdditionalOperation)(const sClLnk *); // Function handler - called when two clusters are merged
	void (*mDestroyAdditionalData)(sClLnk *); // Function handler -  called when additional data are destroyed

};

#endif
