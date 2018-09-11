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
 *@file FeatTrack.cpp
 *@author Yash Manian
 *@brief Function definitions for methods declared in class FeatTrack.
 *@brief The following is a collection of methods to detect, track and compute homography
 */


#include "FeatTrack.hpp"


// Constructor
FeatTrack::FeatTrack()
{
	winSize = cv::Size(21,21);
	termcriteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	fast_thresh = 20;
	nonMaxSuppression = true;
	MaxIter = 1000;
	threshHomography = 10;
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
	cv::Mat X = cv::Mat::zeros(int(x1.size())*2, 9, CV_64F);  // [8x9]
	cv::Mat U, W, Vt, Z, Xa, Xb;    // SVD Matrices
	int len = 0;

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

// Run RANSAC on Homography for image stitching (Not for VO!)
void FeatTrack::homographyRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points)
{
	std::vector<int> randomIdx;
	std::vector<cv::Point2f> rndSet1, rndSet2;

	size_t size = img_1_points.size();
	int randomKey = rand()%int(size);
	cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
	int inliers = 0, maxInliers = 0;

	// RANSAC loop
	for(size_t i = 0; i < this->MaxIter; i++)
	{
		// Select 4 random matched indices (change rand() to c++11 version later)
		while(randomIdx.size() < 4)
		{
			while (std::find(randomIdx.begin(), randomIdx.end(), randomKey) != randomIdx.end())
			{
				randomKey = rand() % int(size);
			}
			randomIdx.push_back(randomKey);
		}

		// Form the homography pairs
		for(size_t j = 0; j < randomIdx.size(); ++j)
		{
			rndSet1.emplace_back(cv::Point2f(img_1_points[randomIdx[j]].x, img_1_points[randomIdx[j]].y));
			rndSet2.emplace_back(cv::Point2f(img_2_points[randomIdx[j]].x, img_2_points[randomIdx[j]].y));
		}

		this->computeHomography(rndSet1, rndSet2, H);

		for(int j = 0; j < size; ++j)
		{
			cv::Mat homogenousVec1 = (cv::Mat_<double>(3,1) << img_1_points[j].x, img_1_points[j].y, 1);
			homogenousVec1 = H*homogenousVec1;
			homogenousVec1 /= homogenousVec1.at<double>(2,0);
			cv::Mat homogenousVec2 = (cv::Mat_<double>(3,1) << img_2_points[j].x, img_2_points[j].y, 1);

			cv::Mat threshFunc;
			cv::pow((homogenousVec2 - homogenousVec1), 2, threshFunc);

			if(sum(threshFunc).val[0] > threshHomography)
			{
				inliers++;
			}
			if(inliers > maxInliers)
			{
				maxInliers = inliers;
			}

			randomIdx.clear();
			rndSet1.clear();
			rndSet2.clear();
		}

	}
}