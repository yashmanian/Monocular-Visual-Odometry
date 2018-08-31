#include "FeatTrack.hpp"


// Constructor
FeatTrack::FeatTrack()
{
	winSize = cv::Size(21,21);
	termcriteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	fast_thresh = 20;
	nonMaxSuppression = true;
}



// Track Features
void FeatTrack::featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, std::vector<uchar> &status)
{
	std::vector<float> err;

	cv::calcOpticalFlowPyrLK(img_1, img_2, img_1_points, img_2_points, status, err, winSize, 3, termcriteria, 0, 0.001);

	int idxCorrect = 0;

	for(int i=0; i<status.size(); ++i)
	{
		cv::Point2f pt = img_2_points[i - idxCorrect];
		if(status[i] == 0 || pt.x < 0 || pt.y < 0)
		{
			if(pt.x<0 || pt.y<0)
			{
				status[i] = 0;
			}
			img_1_points.erase(img_1_points.begin() + (i - idxCorrect));
			img_2_points.erase(img_2_points.begin() + (i - idxCorrect));
			idxCorrect++;
		}
	}
}


// Detect Features
void FeatTrack::featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &img_points)
{
	std::vector<cv::KeyPoint> keypoints_1;
	cv::FAST(img_1, keypoints_1, fast_thresh, nonMaxSuppression);
	cv::KeyPoint::convert(keypoints_1, img_points, std::vector<int>());
}


// Compute Homography
void FeatTrack::computeHomography(std::vector<cv::Point2f> &x1, std::vector<cv::Point2f> &x2, cv::Mat &H)
{
	cv::Mat X = cv::Mat::zeros(x1.size()*2, 9, CV_64F);  // [8x9]
	cv::Mat U, W, Vt, Z, Xa, Xb;    // SVD Matrices
	size_t len = 0;

	for(size_t i = 0; i < x1.size() ; ++i)
	{
		Xa = (cv::Mat_<double>(1,9) << -x1[i].x, -x1[i].y, -1, 0, 0, 0, x1[i].x*x2[i].x, x1[i].y*x2[i].x, x2[i].x);
		Xb = (cv::Mat_<double>(1,9) << 0, 0, 0, -x1[i].x, -x1[i].y, -1, x1[i].x*x2[i].y, x1[i].y*x2[i].y, x2[i].y);
		Xa.copyTo(X.row(len));
		Xb.copyTo(X.row(len+1));
		len += 2;
	}

	// Compute SVD of A
	cv::SVD::compute(X, W, U, Vt, cv::SVD::FULL_UV);
	Z = Vt.row(8);  // [1x9]
	H = Z.reshape(0, 3);
}

// Run RANSAC on Homography
void FeatTrack::homographyRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points)
{

}