#include "Utilities.h"

float PSJaccardDist(const bm::bvector<> &nB1, const bm::bvector<> &nB2, unsigned int *nUnionPtr, unsigned int *nIntersectionPtr){
	
	unsigned int nUnion = bm::count_or(nB1, nB2);
	unsigned int nIntersection = bm::count_and(nB1, nB2);
	if(nUnionPtr != NULL)
		*nUnionPtr = nUnion;
	if(nIntersectionPtr != NULL)
		*nIntersectionPtr = nIntersection;
	if(nIntersection == 0)
		return 1.0f;
	else
		return 1.0f - ((float)nIntersection/(float)nUnion);
}

// Memory saving variant
float PSJaccardDist(const bvect &nB1, const bvect &nB2, unsigned int *nUnionPtr, unsigned int *nIntersectionPtr){
	
	unsigned int nUnion = bm::count_or(nB1, nB2);
	unsigned int nIntersection = bm::count_and(nB1, nB2);
	if(nUnionPtr != NULL)
		*nUnionPtr = nUnion;
	if(nIntersectionPtr != NULL)
		*nIntersectionPtr = nIntersection;
	if(nIntersection == 0)
		return 1.0f;
	else
		return 1.0f - ((float)nIntersection/(float)nUnion);
}