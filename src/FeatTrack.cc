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

// Get Homography
//void FeatTrack::homographyRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points)
//{
//
//}