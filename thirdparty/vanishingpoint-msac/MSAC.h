#ifndef __MSAC_H__
#define __MSAC_H__

#include <opencv2/opencv.hpp>
#include "cv.h"      
#include "highgui.h" 
#include "cxcore.h"  

#include "errorNIETO.h"

#include <math.h>

#define MODE_LS		0
#define MODE_NIETO	1

class MSAC
{
public:
	MSAC(void);
	~MSAC(void);

private:
	// Error Mode
	int __mode;	// Error mode (MODE_LS or MODE_NIETO)

	// Image info
	int __width;
	int __height;

	// RANSAC Options 
	float __epsilon;
	float __P_inlier;
	float __T_noise_squared;
	int __min_iters;
	int __max_iters;
	bool __reestimate;
	bool __verbose;
	bool __update_T_iter;
	bool __notify;

	// Parameters (precalculated)
	int __minimal_sample_set_dimension;	// Dimension of the MSS (minimal sample set)
	int __N_I_best;				// Number of inliers of the best Consensus Set
	float __J_best;				// Cost of the best Consensus Set
	std::vector<int> __MSS;			// Minimal sample set

	// Auxiliar variables
	cv::Mat __a, __an, __b, __bn, __li, __c;	
	cv::Mat __vp, __vpAux;

	// Calibration
	cv::Mat __K;				// Approximated Camera calibration matrix

	// Data (Line Segments)	
	cv::Mat __Li;				// Matrix of appended line segments (3xN) for N line segments
	cv::Mat __Mi;				// Matrix of middle points (3xN)
	cv::Mat __Lengths;			// Matrix of lengths (1xN)

	// Consensus set
	std::vector<int> __CS_idx, __CS_best;	// Indexes of line segments: 1 -> belong to CS, 0 -> does not belong 
	std::vector<int> __ind_CS_best;		// Vector of indexes of the Consensus Set 
	double vp_length_ratio;			

public:
	
	/** Initialisation of MSAC procedure*/
	void init(int mode, cv::Size imSize, bool verbose=false);

	/** Main function which returns, if detected, several vanishing points and a vector of containers of line segments
		corresponding to each Consensus Set.*/
	void multipleVPEstimation(std::vector<std::vector<cv::Point> > &lineSegments, std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters, std::vector<int> &numInliers, std::vector<cv::Mat> &vps, int numVps);
		
	/** Draws vanishing points and line segments according to the vanishing point they belong to*/
	void drawCS(cv::Mat &im, std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters, std::vector<cv::Mat> &vps);

private:	
	/** This function returns a randomly selected MSS*/
	void GetMinimalSampleSet(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi, std::vector<int> &MSS, cv::Mat &vp);

	/** This function returns the Consensus Set for a given vanishing point and set of line segments*/
	float GetConsensusSet(int vpNum, cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi, cv::Mat &vEst, std::vector<float> &E, int *CS_counter);

	/** This is an auxiliar function that formats data into appropriate containers*/
	void fillDataContainers(std::vector<std::vector<cv::Point> > &lineSegments);
	
	// Estimation functions
	/** This function estimates the vanishing point for a given set of line segments using the Least-squares procedure*/
	void estimateLS(cv::Mat &Li, cv::Mat &Lengths, std::vector<int> &set, int set_length, cv::Mat &vEst);

	/** This function estimates the vanishing point for a given set of line segments using the Nieto's method*/
	void estimateNIETO(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi, std::vector<int> &set, int set_length, cv::Mat &vEst);
	
	// Error functions
	/** This function computes the residuals of the line segments given a vanishing point using the Least-squares method*/
	float errorLS(int vpNum, cv::Mat &Li, cv::Mat &vp, std::vector<float> &E, int *CS_counter);

	/** This function computes the residuals of the line segments given a vanishing point using the Nieto's method*/
	float errorNIETO(int vpNum, cv::Mat &Li, cv::Mat &lengthsLS, cv::Mat &Mi, cv::Mat &vp, std::vector<float> &E, int *CS_counter);
	
};

#endif // __MSAC_H__
