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
	cv::Point2f p1c, p2c;
	int size = p1.size();
	double t = 0, scale1 = 0, scale2 = 0;
	// Normalize points to shift origin to image center
	for(int i = 0; i < size; ++i)
	{
		p1c += p1[i];
		p2c += p2[i];
	}

	t = 1./size;
	p1c *= t;
	p2c *= t;

	for( int i = 0; i < size; i++ )
	{
		scale1 += cv::norm(cv::Point2d(p1[i].x - p1c.x, p1[i].y - p1c.y));
		scale2 += cv::norm(cv::Point2d(p2[i].x - p2c.x, p2[i].y - p2c.y));
	}

	scale1 *= t;
	scale2 *= t;

	if(scale1 < FLT_EPSILON || scale2 < FLT_EPSILON)
	{
		std::cout << "Scale less than threshold!" << std::endl;
		return;
	}

	scale1 = sqrt(2.)/scale1;
	scale2 = sqrt(2.)/scale2;

	cv::Mat A = cv::Mat::zeros(9, 9, CV_64F);

	// Define A for linear system Ax = 0, for each pair of matched points. For computational efficiency, using (At*A)*x = 0
	double x1, y1, x2, y2;
	cv::Mat Xa;
	for(int i = 0; i < size; ++i)
	{
		// Convert points to image center origin
		x1 = (p1[i].x - p1c.x) * scale1;
		x2 = (p2[i].x - p2c.x) * scale2;
		y1 = (p1[i].y - p1c.y) * scale1;
		y2 = (p2[i].y - p2c.y) * scale2;
		Xa = (cv::Mat_<double>(9,1) << x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1);
		A += Xa*Xa.t();
	}

	// Find Eigen values of A
	cv::Mat W, V;
	cv::eigen(A, W, V);

	int count = 0;
	for(int i = 0; i < 9; ++i)
	{
		if(fabs(W.at<double>(i)) < DBL_EPSILON)
		{
			W.at<double>(i) = 0.0;
		}
	}

	cv::Mat Z, F_t;
	V.col(8).copyTo(Z);
	F_t = Z.reshape(0, 3);

	// Enforce Rank 2 on F
	cv::Mat W_svd, U, Vt;
	cv::Mat W_diag = cv::Mat::zeros(3,3, CV_64F);

	cv::SVD::compute(F_t, W_svd, U, Vt);
	W_svd.at<double>(2) = 0;
	W_diag.at<double>(0,0) = W_svd.at<double>(0);
	W_diag.at<double>(1,1) = W_svd.at<double>(1);
	W_diag.at<double>(2,2) = W_svd.at<double>(2);
	F_t = U*W_diag*Vt;

	// Transform fundamental matrix back to image origin points
	cv::Mat S1 = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat S2 = cv::Mat::zeros(3, 3, CV_64F);
	S1.at<double>(0,0) = scale1;
	S1.at<double>(0,2) = -scale1*p1c.x;
	S1.at<double>(1,1) = scale1;
	S1.at<double>(1,2) = -scale1*p1c.y;
	S1.at<double>(2,2) = 1;
	S2.at<double>(0,0) = scale2;
	S2.at<double>(0,2) = -scale2*p2c.x;
	S2.at<double>(1,1) = scale2;
	S2.at<double>(1,2) = -scale2*p2c.y;
	S2.at<double>(2,2) = 1;

	F_t = S2.t()*F_t*S1;

	//Force F_t(3,3) = 1
	F_t *= 1./F_t.at<double>(2,2);
	F = F_t.clone();
}

// Refine Fundamental Matrix with RANSAC
void epipolar::fundamentalMatrixRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, cv::Mat &F)
{
	std::vector<int> randomIdx;
	std::vector<cv::Point2f> rndSet1, rndSet2;

	size_t size = img_1_points.size();
	int randomKey = rand()%int(size);
	cv::Mat F_t = cv::Mat::zeros(3, 3, CV_64F);
	int inliers = 0, maxInliers = 5;

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
			if(fabs(det) < 0.5)
			{
				inliers++;
			}
		}

		if(inliers >= maxInliers)
		{
			F = F_t.clone();
			return;
		}

		randomIdx.clear();
		rndSet1.clear();
		rndSet2.clear();
		inliers = 0;
	}

}