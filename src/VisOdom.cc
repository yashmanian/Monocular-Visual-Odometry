#include "VisOdom.hpp"


// Constructor
VisOdom::VisOdom()
{
	winSize = cv::Size(21,21);
	termcriteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	fast_thresh = 20;
	nonMaxSuppression = true;

	// Initialize Camera Parameters to default value, in case filre read goes wrong
	fx = 718.8560;
	fy = 718.8560;
	ppx = 607.1928;
	ppy = 185.2157;
	skew = 0;

	// pp = cv::Point2d(ppx, ppy);
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
double VisOdom::getAbsoluteScale(char filename_gt[200], int frame_id)
{
	std::string line;
	int i = 0;
	double x_prev, y_prev, z_prev;
	double x = 0, y = 0, z = 0;

	std::ifstream poseFile(filename_gt);

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
	else
	{
		std::cout << "Unable to open Pose file!" << std::endl;
		return 0;
	}

	poseFile.close();

	return sqrt((x-x_prev)*(x-x_prev)  + (y-y_prev)*(y-y_prev)  + (z-z_prev)*(z-z_prev));
}



// Compute Essential Matrix. Fill in with custom implementation later
void VisOdom::computeEssentialMatrix(cv::Mat &E, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask)
{
	cv::Mat F;
	F = cv::findFundamentalMat(points1, points2, CV_FM_RANSAC, this->epipolarThresh, this->probEssential, mask);
	E = cv::findEssentialMat(points2, points1, this->fx, this->pp, cv::RANSAC, this->epipolarThresh, this->probEssential, mask);
}



// Compute Pose from Essential Matrix
void VisOdom::computePose(cv::Mat &E, cv::Mat &R, cv::Mat &t, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask)
{
	cv::recoverPose(E, points2, points1, R, t, this->fx, this->pp, mask);
}



// Get Camera Calibration Matrix from calib.txt. Implementation not pretty. Will change later
Eigen::Matrix3d VisOdom::getCameraParams(char filename_calib[200])
{
	std::ifstream calibFile;
	calibFile.open(filename_calib);

	if(calibFile.fail())
	{
		std::cerr << "* Error opening file!" << std::endl;
		exit(1);
	}

	std::string lineA, x;
	double BS;
	if(std::getline(calibFile, lineA))
	{
		std::istringstream streamA(lineA);

		for(int n = 0; n < 13; n++)
		{
			if(n == 0)
			{
				streamA >> x;
			}
			if(n == 1)
			{
				streamA >> this->fx;
			}
			if(n == 2)
			{
				streamA >> this->skew;
			}
			if(n == 3)
			{
				streamA >> this->ppx;
			}
			if(n == 4)
			{
				streamA >> BS;
			}
			if(n == 5)
			{
				streamA >> BS;
			}
			if(n == 6)
			{
				streamA >> this->fy;
			}
			if(n == 7)
			{
				streamA >> this->ppy;
			}
		}

	}
	std::cout << K(0,0) << std::endl;
	this->pp = cv::Point2d(ppx, ppy);
	this->K << fx, skew, ppx,
		 	    0, fy, ppy,
		 		0, 0, 1;

	return K;
}