/** MIT License
Copyright (c) 2018 Yash Manian
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *@copyright Copyright 2018 Yash Manian
 *@file epipolar.cpp
 *@author Yash Manian
 *@brief Function definitions for methods declared in class epipolar
 *@brief The following is a collection of methods to compute various Epipolar constraints
 */


#include "epipolar.hpp"

epipolar::epipolar()
{
	// Initialize Camera Parameters to default value, in case file read goes wrong
	fx = 718.8560;
	fy = 718.8560;
	cx = 607.1928;
	cy = 185.2157;
	skew = 0;
	MaxIter = 30;
	inlierThresh = 0.80;
	RANSAC_THRESH = 0.005;
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
Eigen::Matrix3d epipolar::getCameraParams(char filename_calib[200])
{
	using namespace std;
	using namespace Eigen;

	ifstream calibFile;
	calibFile.open(filename_calib);

	if(calibFile.fail())
	{
		cerr << "* Error opening file!" << endl;
		exit(1);
	}

	string lineA, x;
	double BS;
	if(getline(calibFile, lineA))
	{
		istringstream streamA(lineA);

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

	this->K << this->fx, this->skew, this->cx,
			    0, this->fy, this->cy,
			    0, 0, 1;
	return K;
}



// Estimate Fundamental Matrix using the 8-point algorithm
void epipolar::estimateFundamentalMatrix(Eigen::MatrixXd &points1, Eigen::MatrixXd &points2, Eigen::Matrix3d &F)
{
	using namespace Eigen;
	using namespace std;

	if(points1.cols() != points2.cols())
	{
		cout << "Number of matched features should be equal!" << endl;
		return;
	}


	int npts = points1.cols();
	Matrix3d T1, T2, D;
	MatrixXd p1(3, npts), p2(3, npts), A(npts, 9), x1(npts, 1), x2(npts, 1), y1(npts, 1), y2(npts, 1), V(9, 1);

	// Normalize points to shift origin to centroid
	T1 = this->normalise2dpts(points1);
	T2 = this->normalise2dpts(points2);
	p1 = T1*points1;
	p2 = T2*points2;

	x1 = p1.row(0).transpose();
	y1 = p1.row(1).transpose();
	x2 = p2.row(0).transpose();
	y2 = p2.row(1).transpose();

	// Building constraint matrix
	A << x2.array()*x1.array(), x2.array()*y1.array(), x2, y2.array()*x1.array(), y2.array()*y1.array(), y2, x1, y1, MatrixXd::Ones(npts,1);

	// Run SVD and get last column
	BDCSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);

	V = svd.matrixV().col(8);

	// Reshape last column into Fundamental matrix
	Map<MatrixXd> F_t(V.data(), 3, 3);

	// Enforce Rank 2 for F_t
	svd.compute(F_t.transpose());
	D << svd.singularValues()(0), 0, 0,
		 0, svd.singularValues()(1), 0,
		 0, 0, 0;

	F_t = svd.matrixU() * D * svd.matrixV().transpose();

	// Denormalize
	F = T2.transpose() * F_t * T1;

	if(F(2,2) < 0)
	{
		F *= -1;
	}
	// Normalize F by last element
	F /= F(2,2);
}


// Refine Fundamental Matrix using RANSAC
void epipolar::FundamentalMatRANSAC(Eigen::Matrix3d &F, Eigen::MatrixXd &points1, Eigen::MatrixXd &points2)
{
	using namespace std;
	using namespace Eigen;

	// Check for matrix dimension constraints
	if(points1.cols() != points2.cols())
	{
		cout << "Number of matched features should be equal!" << endl;
		return;
	}

	if(points1.rows() != 3)
	{
		if(points1.rows() == 2)
		{
			points1.conservativeResize(points1.rows() + 1, points1.cols());
			points1.row(points1.rows()-1) = MatrixXd::Ones(points1.cols(),1);
			points2.conservativeResize(points2.rows() + 1, points2.cols());
			points2.row(points2.rows()-1) = MatrixXd::Ones(points2.cols(),1);
		}
		else
		{
			cout << "Matrix needs to be at least 2xN!" << endl;
			return;
		}
	}

	// Variable declaration
	int size = points1.cols();
	vector<int> randomIdx;
	MatrixXd batch1(3, 8), batch2(3, 8), distFunc(size, size), dist(size, 1);
	MatrixXd selectPts1 = points1, selectPts2 = points2;
	Matrix3d F_t, bestF;
	int randomKey = rand()%int(size);
	int maxInliers = 0;

	// RANSAC loop
	for(int i = 0; i < this->MaxIter; ++i)
	{

		// Select 8 random matched indices (change rand() to c++11 version later)
		while (randomIdx.size() < 8)
		{
			while (find(randomIdx.begin(), randomIdx.end(), randomKey) != randomIdx.end())
			{
				randomKey = rand() % int(size);
			}
			randomIdx.push_back(randomKey);
		}

		batch1 << points1.col(randomIdx[0]), points1.col(randomIdx[1]), points1.col(randomIdx[2]), points1.col(
				randomIdx[3]), points1.col(randomIdx[4]), points1.col(randomIdx[5]), points1.col(
				randomIdx[6]), points1.col(randomIdx[7]);
		batch2 << points2.col(randomIdx[0]), points2.col(randomIdx[1]), points2.col(randomIdx[2]), points2.col(
				randomIdx[3]), points2.col(randomIdx[4]), points2.col(randomIdx[5]), points2.col(
				randomIdx[6]), points2.col(randomIdx[7]);

		// Estimate fundamental matrix from batch of random points (fitting function)
		this->estimateFundamentalMatrix(batch1, batch2, F_t);

		// Exploit epipolar planar constraint as distance function
		distFunc = points2.transpose() * F_t * points1;
		dist = distFunc.diagonal();
		int inliers = (dist.array() < RANSAC_THRESH).count();

		// Compare current inliers to best estimate
		if (inliers > maxInliers)
		{
			double inlierCount = double(inliers)/double(size);

			// If a certain threshold of inliers is met
			if(inlierCount >= inlierThresh)
			{
				maxInliers = inliers;
				bestF = F_t;
				break;
			}

			maxInliers = inliers;
			bestF = F_t;

			// Replace using libigl later to slice matrix
			int inCount = 0;
			for(int k = 0; k < size; k++)
			{
				double ny = dist(k);
				if(dist(k) < RANSAC_THRESH)
				{

					if(inCount > selectPts1.cols())
					{
						selectPts1.conservativeResize(selectPts1.rows(), points1.cols() + 1);
						selectPts2.conservativeResize(selectPts2.rows(), points2.cols() + 1);
					}

					selectPts1.col(inCount) = points1.col(k);
					selectPts2.col(inCount) = points2.col(k);
					inCount++;
				}
			}

			if(inCount < selectPts1.cols())
			{
				selectPts1.conservativeResize(selectPts1.rows(), inCount - 1);
			}

		}
	}

	cout << selectPts1 << endl;
	// Estimate final Fundamental matrix using selected points
	this->FundamentalMatRANSAC(F, selectPts1, selectPts2);
	cout << F << endl;
}

// Compute epipoles
Eigen::MatrixXd epipolar::computeEpipole(Eigen::Matrix3d &F)
{
	using namespace std;
	using namespace Eigen;

	// Since both epipoles lie on the Epipolar plane, ite can be said e2T*F*e1 = 0
	// Therefore, by solving linear least squares
	BDCSVD<MatrixXd> SVD(F, ComputeThinU | ComputeThinV);
	MatrixXd E(3,2);
	E << SVD.matrixV().col(2), SVD.matrixU().col(2);

	// Enforce Normalization
	E.array().rowwise() /= E.row(2).array();

	return E;
}


