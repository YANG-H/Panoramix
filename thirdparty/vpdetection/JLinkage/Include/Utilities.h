#ifndef UTILITIESJL_H
#define UTILITIESJL_H

#include <vector>
#include "bm/bm.h"
#include "bm/bmalgo.h"

// given two Preference set Vectors compute the jaccard distance
float PSJaccardDist(const bm::bvector<> &nB1, const bm::bvector<> &nB2, unsigned int *nUnion = NULL, unsigned int *nIntersection = NULL);
// Memory saving variant
typedef bm::bvector<bm::standard_allocator, bm::miniset<bm::block_allocator, bm::set_total_blocks> > bvect;
float PSJaccardDist(const bvect &nB1, const bvect &nB2, unsigned int *nUnion = NULL, unsigned int *nIntersection = NULL);

// given two Preference set Vectors compute the jaccard distance
template<typename T>
T PSJaccardDist(const std::vector<unsigned int> *nVecI, const std::vector<unsigned int> *nVecJ){
	std::vector<unsigned int>::const_iterator iteratorI = nVecI->begin();
	std::vector<unsigned int>::const_iterator iteratorJ = nVecJ->begin();

	int nIntersection = 0;
	int nUnion = 0;
	while(iteratorJ != nVecJ->end() && iteratorI!= nVecI->end()){
		if(iteratorJ != nVecJ->end() && iteratorI!= nVecI->end() && (*iteratorI) == (*iteratorJ)){
			nIntersection++;
			nUnion++;
			iteratorI++;
			iteratorJ++;
		}
		else if(iteratorJ == nVecJ->end() || (*iteratorI) < (*iteratorJ)){
			nUnion++;
			iteratorI++;
		}
		else if(iteratorI == nVecI->end() || (*iteratorJ) < (*iteratorI)){
			nUnion++;
			iteratorJ++;
		}
	}
	if(nIntersection == 0)
		return (T)1.0;
	else
		return (T)1 - ((T)nIntersection/(T)nUnion);
}


// given two vector of the same size compute euclidean distance
template<typename T>
T VecEuclideanDist(const std::vector<T> &nVecI, const std::vector<T> &nVecJ){
	assert(nVecI.size() == nVecJ.size());
	T eucDist = 0.0f;
	std::vector<T> tempVec = std::vector<T>(nVecI.size());
	for(unsigned int i=0; i<nVecI.size(); i++)
		eucDist += (nVecI[i] - nVecJ[i]) * (nVecI[i] - nVecJ[i]);

	return sqrt(eucDist);
}

// given a vector of cumulative pdf sums, return the randomly choosen index according to the pdf
template<typename T>

unsigned int RandomSampleOnCumSumHist(const std::vector<T> &nHist){

	T randSample = (T)rand()/(T)RAND_MAX;
	for(unsigned int i=0; i<nHist.size(); i++){
		if(randSample < nHist[i])
			return i;
	}

	return (unsigned int)nHist.size()-1;
}

// Initialize pdf cumsum histogram from a probability histogram
template<typename T>
void SetCumSumHist(std::vector<T> *nHistCumSum, const std::vector<T> &nHist){
	assert(nHistCumSum->size() == nHist.size());

	if(nHistCumSum->size() != nHist.size())
		return;
	unsigned int nSize = (unsigned int)nHistCumSum->size();

    T mean = (T)0, sum = (T)0;

	for(unsigned int i=0; i<nSize; i++){
		mean += (nHist[i] - mean) / ((T) (i + 1));
	}

    for (unsigned int i = 0; i < nSize; i++){
        sum += (nHist[i] / mean) / nSize;
        (*nHistCumSum)[i] = sum;
    }

}



#endif