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

	cv::SVD::compute(F_t, W_svd, U, Vt, cv::SVD::FULL_UV);
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

// Estimate Essential Matrix from Fundamental Matrix
void epipolar::estimateEssentialMatrix(cv::Mat &F, cv::Mat &E)
{
	E = this->K.t() *F* this->K;

	// Enforce rank 2 by making last singular value 0
	cv::Mat U, W, Vt;
	cv::SVD::compute(E, W, U, Vt, cv::SVD::FULL_UV);
	cv::Mat W_diag = cv::Mat::eye(3, 3, CV_64F);
	W_diag.at<double>(2,2) = 0;

	E = U*W_diag*Vt;
}

void epipolar::estimatePose(cv::Mat &E, cv::Mat &R, cv::Mat &t, std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2)
{
	cv::Mat U, D, Vt, R1, R2, C1r1, C1r2, C2r1, C2r2, P1, P2, P3, P4;
	cv::SVD::compute(E, D, U, Vt, cv::SVD::FULL_UV);
	cv::Mat W = cv::Mat::zeros(3, 3, CV_64F);
	W.at<double>(0,1) = -1;
	W.at<double>(1,0) = 1;
	W.at<double>(2,2) = 1;

	cv::Mat x1 = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat x2 = cv::Mat::zeros(3, 1, CV_64F);
	x1.at<double>(0) = p1[0].x;
	x1.at<double>(1) = p1[0].y;
	x1.at<double>(2) = 1;
	x2.at<double>(0) = p2[0].x;
	x2.at<double>(1) = p2[0].y;
	x2.at<double>(2) = 1;

	// Compute Possible Poses
	U.col(2).copyTo(C1r1);
	C1r2 = C1r1;
	C2r1 = C1r1*-1;
	C2r2 = C2r1;

	R1 = U*W*Vt;
	R2 = U*W.t()*Vt;

	if(cv::determinant(R1) == -1)
	{
		R1 *= -1;
		C1r1 *= -1;
		C2r1 *= -1;
	}
	if(cv::determinant(R2) == -1)
	{
		R2 *= -1;
		C1r2 *= -1;
		C2r2 *= -1;
	}

	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), -1*C1r1, C1r1);
	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), -1*C1r2, C1r2);
	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), -1*C2r1, C2r1);
	cv::hconcat(cv::Mat::eye(3, 3, CV_64F), -1*C2r2, C2r2);
	P1 = this->K * R1 * C1r1;
	P2 = this->K * R1 * C2r1;
	P3 = this->K * R2 * C1r2;
	P4 = this->K * R2 * C2r2;

	std::vector<cv::Mat> PXcam;
	PXcam.push_back(P1);
	PXcam.push_back(P2);
	PXcam.push_back(P3);
	PXcam.push_back(P4);

	int pose = this->cheiralityCheckedPose(PXcam, x1, x2);

	if(pose == 1)
	{
		R = R1;
		t = C1r1.col(3);
	}
	if(pose == 2)
	{
		R = R1;
		t = C2r1.col(3);
	}
	if(pose == 3)
	{
		R = R2;
		t = C1r2.col(3);
	}
	if(pose == 4)
	{
		R = R2;
		t = C2r2.col(3);
	}

}

int epipolar::cheiralityCheckedPose(std::vector<cv::Mat> &PXcam, cv::Mat &x1, cv::Mat &x2)
{
	cv::Mat Pcam = (cv::Mat_<double>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
	cv::Mat P = this->K*Pcam;
	cv::Mat x1hat = this->K.inv()*x1;
	cv::Mat x2hat, A1, A2, A3, A4, U, W, Vt, V, xi;
	cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat P_t = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	cv::Mat PX_t = cv::Mat::zeros(3, 3, CV_64F);
	double A1n, A2n, A3n, A4n, Z, T, m, sign;
	int pose;

	// For each pose, re-project point in 3D and find depth
	cv::Mat X3D = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat depth = cv::Mat::zeros(4, 2, CV_64F);

	for(int i = 0; i < 4; ++i)
	{
		x2hat = this->K.inv()*x2;
		// Build A matrix
		A1 = Pcam.row(2)*x1hat.at<double>(0) - Pcam.row(0);
		A2 = Pcam.row(2)*x1hat.at<double>(1) - Pcam.row(1);
		A3 = PXcam[i].row(2)*x2hat.at<double>(0) - PXcam[i].row(0);
		A4 = PXcam[i].row(2)*x2hat.at<double>(1) - PXcam[i].row(1);

		// Normalize A
		A1n = sqrt(cv::sum(A1)[0]*cv::sum(A1)[0]);
		A2n = sqrt(cv::sum(A2)[0]*cv::sum(A2)[0]);
		A3n = sqrt(cv::sum(A3)[0]*cv::sum(A3)[0]);
		A4n = sqrt(cv::sum(A4)[0]*cv::sum(A4)[0]);

		A1 /= A1n;
		A2 /= A2n;
		A3 /= A3n;
		A4 /= A4n;

		A1.copyTo(A.row(0));
		A2.copyTo(A.row(1));
		A3.copyTo(A.row(2));
		A4.copyTo(A.row(3));

		// Obtain 3D point
		cv::SVD::compute(A, W, U, Vt, cv::SVD::FULL_UV);
		V = Vt.t();
		X3D.at<double>(0,i) = V.at<double>(0,3);
		X3D.at<double>(1,i) = V.at<double>(1,3);
		X3D.at<double>(2,i) = V.at<double>(2,3);
		X3D.at<double>(3,i) = V.at<double>(3,3);

		// Get depth on second image (Replace with Rect)
		PX_t.at<double>(0,0) = PXcam[i].at<double>(0,0);
		PX_t.at<double>(0,1) = PXcam[i].at<double>(0,1);
		PX_t.at<double>(0,2) = PXcam[i].at<double>(0,2);
		PX_t.at<double>(1,0) = PXcam[i].at<double>(1,0);
		PX_t.at<double>(1,1) = PXcam[i].at<double>(1,1);
		PX_t.at<double>(1,2) = PXcam[i].at<double>(1,2);
		PX_t.at<double>(2,0) = PXcam[i].at<double>(2,0);
		PX_t.at<double>(2,1) = PXcam[i].at<double>(2,1);
		PX_t.at<double>(2,2) = PXcam[i].at<double>(2,2);
		if(cv::determinant(PX_t) < 0)
		{
			sign = -1;
		}
		else
		{
			sign = 1;
		}

		xi = PXcam[i]*X3D.col(i);
		Z = xi.at<double>(2);
		T = X3D.at<double>(3,i);
		m = sqrt(cv::sum(PX_t)[0]*cv::sum(PX_t)[0]);
		depth.at<double>(i, 0) = (sign*Z)/(T*m);

		// Check depth on first camera
		if(cv::determinant(P_t) < 0)
		{
			sign = -1;
		}
		else
		{
			sign = 1;
		}

		xi = Pcam*X3D.col(i);
		Z = xi.at<double>(2);
		T = X3D.at<double>(3,i);
		m = sqrt(cv::sum(P_t)[0] * cv::sum(P_t)[0]);
		depth.at<double>(i, 1) = (sign*Z)/(T*m);
	}
	if(depth.at<double>(0,0) > 0 && depth.at<double>(0,1))
	{
		pose = 1;
	}
	if(depth.at<double>(1,0) > 0 && depth.at<double>(1,1))
	{
		pose = 2;
	}
	if(depth.at<double>(2,0) > 0 && depth.at<double>(2,1))
	{
		pose = 3;
	}
	if(depth.at<double>(3,0) > 0 && depth.at<double>(3,1))
	{
		pose = 4;
	}
	//std::cout << pose << std::endl;
	return pose;
}