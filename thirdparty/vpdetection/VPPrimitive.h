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
#ifndef VPPRIMITIVE_H
#define VPPRIMITIVE_H

/*#####################Vanishing Point Related Functions####################
see reference:
Tardif J.-P., 
Non-iterative Approach for Fast and Accurate Vanishing Point Detection, 
12th IEEE International Conference on Computer Vision, 
Kyoto, Japan, September 27 - October 4, 2009.
############################################################################*/

inline void vec_cross(float a1, float b1, float c1,
							 float a2, float b2, float c2,
							 float& a3, float& b3, float& c3)
{
	a3 = b1*c2 - c1*b2;
	b3 = -(a1*c2 - c1*a2);
	c3 = a1*b2 - b1*a2;
}

inline void vec_norm(float& a, float& b, float& c)
{
	float len = sqrt(a*a+b*b+c*c);
	a/=len; b/=len; c/=len;
}

inline std::vector<float>  *GetFunction_VP(const std::vector<sPt *> &nDataPtXMss,
	const std::vector<unsigned int>  &nSelectedPts){

	std::vector<float>   *nReturningVector = new std::vector<float> (3,0.0f);

	float xs0 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[0];
	float ys0 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[1];
	float xe0 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[2];
	float ye0 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[3];
	float xs1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[0];
	float ys1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[1];
	float xe1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[2];
	float ye1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[3];

	float l0[3],l1[3],v[3];
	vec_cross(xs0,ys0,1,
		xe0,ye0,1,
		l0[0],l0[1],l0[2]);
	vec_cross(xs1,ys1,1,
		xe1,ye1,1,
		l1[0],l1[1],l1[2]);
	vec_cross(l0[0],l0[1],l0[2],
		l1[0],l1[1],l1[2],
		v[0],v[1],v[2]);
	vec_norm(v[0],v[1],v[2]);

	(*nReturningVector)[0] = v[0];
	(*nReturningVector)[1] = v[1];
	(*nReturningVector)[2] = v[2];

	return nReturningVector;
}

inline float DistanceFunction_VP(const std::vector<float> &nModel, 
	const std::vector<float>  &nDataPt){
	float l[3], mid[3] = {(nDataPt[0]+nDataPt[2])/2.0, (nDataPt[1]+nDataPt[3])/2.0, 1};
	vec_cross(mid[0],mid[1],mid[2],
		nModel[0],nModel[1],nModel[2],
		l[0],l[1],l[2]);
	float dist = fabs(l[0]*nDataPt[0]+l[1]*nDataPt[1]+l[2]) / sqrt(l[0]*l[0]+l[1]*l[1]);
	return  dist;
}

#endif