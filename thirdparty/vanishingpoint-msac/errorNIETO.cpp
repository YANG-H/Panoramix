#include "errorNIETO.h"

using namespace std;
using namespace cv;

float distanceNieto( cv::Mat &vanishingPoint, cv::Mat &lineSegment, float lengthLineSegment, cv::Mat &midPoint )
{
	// IMPORTANT: The vanishing point must arrive here uncalibrated and in Cartesian coordinates
	// Line segment normal (2D)
	float n0 = -lineSegment.at<float>(1,0);
	float n1 = lineSegment.at<float>(0,0);
	float nNorm = sqrt(n0*n0 + n1*n1);

	// Mid point
	float c0 = midPoint.at<float>(0,0);
	float c1 = midPoint.at<float>(1,0);
	float c2 = midPoint.at<float>(2,0);

	// Vanishing point (uncalibrated)
	float v0 = vanishingPoint.at<float>(0,0);
	float v1 = vanishingPoint.at<float>(1,0);
	float v2 = vanishingPoint.at<float>(2,0);

	float r0, r1;
	r0 = v1*c2 - v2*c1;
	r1 = v2*c0 - v0*c2;
	float rNorm = sqrt(r0*r0 + r1*r1);

	float num = (r0*n0 + r1*n1);
	if( num < 0 )
		num = -num;

	float d = 0;
	if(nNorm != 0 && rNorm != 0)
		d = num/(nNorm*rNorm);
	
	// d *= lengthLineSegment;

	return d;
}
void evaluateNieto( const double *param, int m_dat, const void *data,
					double *fvec, int *info)
{
	// Cast to correct types
	data_struct *mydata;
	mydata = (data_struct *) data;
	int ndat = 0;

	// IMPORTANT!!: the vanishing point has arrived here calibrated AND in spherical coordinates!
	// 1) Get Cartesian coordaintes
	double theta = param[0];
	double phi = param[1];
	double x = cos(phi)*sin(theta);
	double y = sin(phi)*sin(theta);
	double z = cos(theta);

	// 2) Uncalibrate it using the K matrix in the data	
	cv::Mat vn = Mat(3,1,CV_32F);
	vn.at<float>(0,0) = x;
	vn.at<float>(1,0) = y;
	vn.at<float>(2,0) = z;
	
	cv::Mat vanishingPoint = mydata->K*vn;
	if(vanishingPoint.at<float>(2,0) != 0)
	{
		vanishingPoint.at<float>(0,0) /= vanishingPoint.at<float>(2,0);
		vanishingPoint.at<float>(1,0) /= vanishingPoint.at<float>(2,0);
		vanishingPoint.at<float>(2,0) = 1;
	}

	cv::Mat lineSegment = Mat(3,1,CV_32F);
	float lengthLineSegment = 0;
	cv::Mat midPoint = Mat(3,1,CV_32F);

	// Fill fvec
	for(int p=0; p< mydata->LSS.cols ; p++)
	{
		// Compute error for this vanishing point (contained in param), and this line segment (LSS.at<double>(0,p);LSS.at<double>(1,p);LSS.at<double>(2,p)) and midPoint
		lineSegment.at<float>(0,0) = mydata->LSS.at<float>(0,p);
		lineSegment.at<float>(1,0) = mydata->LSS.at<float>(1,p);
		lineSegment.at<float>(2,0) = mydata->LSS.at<float>(2,p);

		lengthLineSegment = mydata->Lengths.at<float>(p,p);

		midPoint.at<float>(0,0) = mydata->midPoints.at<float>(0,p);
		midPoint.at<float>(1,0) = mydata->midPoints.at<float>(1,p);
		midPoint.at<float>(2,0) = mydata->midPoints.at<float>(2,p);
		
		fvec[p] = distanceNieto(vanishingPoint, lineSegment, lengthLineSegment, midPoint);
	}

	/* to prevent a 'unused variable' warning */
	*info = *info;

}