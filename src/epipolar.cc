#include "epipolar.hpp"
epipolar::epipolar()
{
	// Initialize Camera Parameters to default value, in case file read goes wrong
	fx = 718.8560;
	fy = 718.8560;
	cx = 607.1928;
	cy = 185.2157;
	skew = 0;
	MaxIter = 100;
}


// Compute absolute scale from Ground truth
double epipolar::getAbsoluteScale(char filename_gt[200], int frame_id)
{
	std::string line;
	int i = 0;
	double x_prev = 0, y_prev = 0, z_prev = 0;
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

// Get Camera Calibration Matrix from calib.txt. Implementation not pretty. Will change later
cv::Mat epipolar::getCameraParams(char filename_calib[200])
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
				streamA >> this->cx;
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
				streamA >> this->cy;
			}
		}

	}
	this->pp = cv::Point2d(cx, cy);
	this->K.at<double>(0,0) = fx;
	this->K.at<double>(0,1) = skew;
	this->K.at<double>(0,2) = cx;
	this->K.at<double>(1,1) = fy;
	this->K.at<double>(2,2) = 1;
	return K;
}

// Estimate Fundamental Matrix using the 8-point algorithm
void epipolar::estimateFundamentalMatrix(std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2, cv::Mat &F)
{
	size_t n = p1.size();
	cv::Mat A = cv::Mat::ones(8, 9, CV_64F);
	cv::Mat D = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat Xa, Xb, U, W, Vt, Z;
	for(size_t i = 0; i < n; ++i)
	{
		Xa = (cv::Mat_<double>(1,9) << p1[i].x*p2[i].x, p2[i].x*p1[i].y, p2[i].x, p2[i].y*p1[i].x, p2[i].y*p1[i].y, p2[i].y, p1[i].x, p1[i].y, 1);
		Xa.copyTo(A.row(i));
	}

	cv::SVD::compute(A, W, U, Vt, cv::SVD::FULL_UV);
	Z = Vt.col(8);  // [1x9]

	int count = 0;

	for (size_t i = 0; i < 3; i++)
	{
		for (size_t k = 0; k < 3; k++)
		{
			F.at<double>(i,k) = Z.at<double>(count);
			++count;
		}
	}

	// Enforce Rank 2
	cv::SVD::compute(F, W, U, Vt, cv::SVD::FULL_UV);
	D.at<double>(0,0) = W.at<double>(0,0);
	D.at<double>(1,1) = W.at<double>(1,1);
	F = U*D*Vt;
	if(F.at<double>(2,2) < 0) F *= -1;
}

// Refine Fundamental Matrix with RANSAC
void epipolar::fundamentalMatrixRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, cv::Mat &F)
{
	std::vector<int> randomIdx;
	std::vector<cv::Point2f> rndSet1, rndSet2;

	size_t size = img_1_points.size();
	int randomKey = rand()%int(size);
	cv::Mat F_t = cv::Mat::zeros(3, 3, CV_64F);
	int inliers = 0, maxInliers = 6;

	for(int i = 0; i < this->MaxIter; ++i)
	{
		// Select 8 random matched indices (change rand() to c++11 version later)
		while(randomIdx.size() < 8)
		{
			while (std::find(randomIdx.begin(), randomIdx.end(), randomKey) != randomIdx.end())
			{
				randomKey = rand() % int(size);
			}
			randomIdx.push_back(randomKey);
		}

		for(size_t j = 0; j < randomIdx.size(); ++j)
		{
			rndSet1.push_back(cv::Point2f(img_1_points[randomIdx[j]].x, img_1_points[randomIdx[j]].y));
			rndSet2.push_back(cv::Point2f(img_2_points[randomIdx[j]].x, img_2_points[randomIdx[j]].y));
		}

		this->estimateFundamentalMatrix(rndSet1, rndSet2, F_t);
		cv::Mat X1, X2, D;
		for(size_t j = 0; j < rndSet1.size(); ++j)
		{
			X1 = (cv::Mat_<double>(3,1) << rndSet1[j].x, rndSet1[j].y, 1);
			X2 = (cv::Mat_<double>(1,3) << rndSet2[j].x, rndSet2[j].y, 1);
			D = X2*F_t*X1;
			double det = cv::determinant(D);
			if(det < 0.5)
			{
				inliers++;
			}
		}

		if(inliers >= maxInliers)
		{
			F = F_t.clone();
			break;
		}

		randomIdx.clear();
		rndSet1.clear();
		rndSet2.clear();
		inliers = 0;
	}

	std::cout << F << std::endl;

}