#ifndef PRIMITIVEFUNCTIONS_H
#define PRIMITIVEFUNCTIONS_H

std::vector<float>  *GetFunction_Plane(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts){

		std::vector<float>   *nReturningVector = new std::vector<float> (4,0.0f); // normal and d
		
		// normal found as cross product of X2 - X1, X3 - X1
		float x1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[0] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[0];
		float x2 = (*nDataPtXMss[nSelectedPts[2]]->mCoord)[0] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[0];
		float y1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[1] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[1];
		float y2 = (*nDataPtXMss[nSelectedPts[2]]->mCoord)[1] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[1];
		float z1 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[2] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[2];
		float z2 = (*nDataPtXMss[nSelectedPts[2]]->mCoord)[2] - (*nDataPtXMss[nSelectedPts[0]]->mCoord)[2];

		(*nReturningVector)[0] = y1 * z2 - z1 * y2;
		(*nReturningVector)[1] = z1 * x2 - x1 * z2;
		(*nReturningVector)[2] = x1 * y2 - y1 * x2;
		// compute d
		(*nReturningVector)[3] =  - ((*nReturningVector)[0] * (*nDataPtXMss[nSelectedPts[0]]->mCoord)[0]) 
									 - ((*nReturningVector)[1] * (*nDataPtXMss[nSelectedPts[0]]->mCoord)[1]) 
									 - ((*nReturningVector)[2] * (*nDataPtXMss[nSelectedPts[0]]->mCoord)[2]); 
		
		// Normalize
		float normN = sqrt((*nReturningVector)[0] * (*nReturningVector)[0] + (*nReturningVector)[1] * (*nReturningVector)[1] +  (*nReturningVector)[2] * (*nReturningVector)[2]);
		(*nReturningVector)[0]  = (*nReturningVector)[0] /normN;
		(*nReturningVector)[1]  = (*nReturningVector)[1] /normN;
		(*nReturningVector)[2]  = (*nReturningVector)[2] /normN;
		(*nReturningVector)[3]  = (*nReturningVector)[3] /normN;

		return nReturningVector;
	}
	
float DistanceFunction_Plane(const std::vector<float> &nModel, const std::vector<float>  &nDataPt){
	return fabs(nModel[0] * nDataPt[0] + nModel[1] * nDataPt[1] + nModel[2] * nDataPt[2] + nModel[3]);
}

std::vector<float>  *GetFunction_Line(const std::vector<sPt *> &nDataPtXMss, const std::vector<unsigned int>  &nSelectedPts){
	
	std::vector<float>   *nReturningVector = new std::vector<float> (3,0.0f); // normal and d
	
	float x1 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[0];
	float x2 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[0];
	float y1 = (*nDataPtXMss[nSelectedPts[0]]->mCoord)[1];
	float y2 = (*nDataPtXMss[nSelectedPts[1]]->mCoord)[1];

	// A
	(*nReturningVector)[0] = (y2 - y1) / (y2*x1 - x2*y1);

	// C
	(*nReturningVector)[2] = -1;

	// B
	(*nReturningVector)[1] = (-(*nReturningVector)[2] -  (*nReturningVector)[0] * x1)/y1;

	return nReturningVector;
}

float DistanceFunction_Line(const std::vector<float> &nModel, const std::vector<float>  &nDataPt){
	//dist(i) = sqrt( (P(1:2)'*X(:,i)+P(3) ).^2  / (P(1)^2+P(2)^2));
	float dist = fabs(nModel[0] * nDataPt[0] + nModel[1] * nDataPt[1] + nModel[2]) / sqrt(nModel[0] * nModel[0] + nModel[1] * nModel[1]);
	return  dist;
}


#endif