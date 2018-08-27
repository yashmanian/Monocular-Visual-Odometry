#include "VisOdom.hpp"

// Constructor
VisOdom::VisOdom()
{
	winSize = cv::Size(21,21);
	termcriteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	fast_thresh = 20;
	nonMaxSuppression = true;
	focal = 718.8560;
	pp = cv::Point2d(607.1928, 185.2157);
	epipolarThresh = 0.999;
	probEssential = 1.0;
}

// Track Features
void VisOdom::featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, std::vector<uchar> &status)
{
	std::vector<float> err;

	cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcriteria, 0, 0.001);

	int idxCorrect = 0;

	for(int i=0; i<status.size(); ++i)
	{
		cv::Point2f pt = points2[i - idxCorrect];
		if(status[i] == 0 || pt.x < 0 || pt.y < 0)
		{
			if(pt.x<0 || pt.y<0)
			{
				status[i] = 0;
			}
			points1.erase(points1.begin() + (i - idxCorrect));
			points2.erase(points2.begin() + (i - idxCorrect));
			idxCorrect++;
		}
	}
}


// Detect Features
void VisOdom::featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &points1)
{
	std::vector<cv::KeyPoint> keypoints_1;

	cv::FAST(img_1, keypoints_1, fast_thresh, nonMaxSuppression);
	cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

// Compute absolute scale from Ground truth
double VisOdom::getAbsoluteScale(int frame_id, int sequence_id)
{
	std::string line;
	int i = 0;
	double x_prev, y_prev, z_prev;
	double x = 0, y = 0, z = 0;

	char filenamePoses[200];

	std::sprintf(filenamePoses, "/home/yashmanian/KITTI_VO/Datasets/poses/%02d.txt",sequence_id);
	std::ifstream poseFile(filenamePoses);

	if(poseFile.is_open())
	{
		while(getline(poseFile, line) && i<=frame_id)
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;

			std::istringstream in(line);

			for(int j = 0; j<12; ++j)
			{
				in >> z;
				if(j == 7) y = z;
				if(j == 3) x = z;
			}
			i++;
		}
	}

	poseFile.close();
}

// Compute Essential Matrix. Fill in with custom implementation later
void VisOdom::computeEssentialMatrix(cv::Mat &E, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask)
{
	E = findEssentialMat(points2, points1, this->focal, this->pp, cv::RANSAC, this->epipolarThresh, this->probEssential, mask);
}

void VisOdom::computePose(cv::Mat &E, cv::Mat &R, cv::Mat &t, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask)
{
	recoverPose(E, points2, points1, R, t, this->focal, this->pp, mask);
}