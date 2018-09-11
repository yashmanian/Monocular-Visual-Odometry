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
 *@file epiLines.cpp
 *@author Yash Manian
 *@brief File to compute epipolar lines and verify position of epipole.
 */

#include "FeatTrack.hpp"
#include "epipolar.hpp"
#include "PointOps.hpp"
#include <ctime>

using namespace std;
using namespace cv;
using namespace Eigen;



int main()
{
	/*-----------------------Declarations-----------------------*/
	// Path variables
	int sequence_id = 0;
	int frame_id = 1;

	// Read Images
	Mat imR, imL, imLc, imRc;
	FeatTrack Features;
	epipolar epi;
	PointOps Pt;
	double scale = 1.00;
	char filename_imL[200], filename_imR[200], filename_gt[200], filename_loop[200], text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);

	sprintf(filename_imL, "/home/yashmanian/Datasets/stereo-pairs/cones/imL.png");
	sprintf(filename_imR, "/home/yashmanian/Datasets/stereo-pairs/cones/imR.png");


	/*-----------------------Test Script-----------------------*/
////////////////////////////////////////////////////////////////////////////////////////////
	/*-----------------------Read Images and Parameters-----------------------*/

	imLc = imread(filename_imL);
	imRc = imread(filename_imR);

	// Throw Exception
	if(!imLc.data || !imRc.data)
	{
		cout << "--(!) Error reading images" << endl;
		return -1;
	}

	/*-----------------------Operations-----------------------*/
	cvtColor(imLc, imL, COLOR_BGR2GRAY);
	cvtColor(imRc, imR, COLOR_BGR2GRAY);


	// Feature detection
	vector<Point2f> points1, points2;	// Vector to store coordinates of features in pixels
	Features.featureDetection(imL, points1);
	// Feature Tracking
	vector<uchar> status;
	Features.featureTracking(imL, imR, points1, points2, status);	// Track features to second image

	vector<Point2f> pt1, pt2;


	MatrixXd pts1 = epi.cv2EigenHomogenous(points1);
	MatrixXd pts2 = epi.cv2EigenHomogenous(points2);
	Matrix3d F, Fcv;
	MatrixXd E1(3,2), E2(3,2);

	epi.estimateFundamentalMatrix(pts1, pts2, F);
	E1 =  epi.computeEpipole(F);
	Mat F2 = Mat::zeros(3,3, CV_64F);
	F2.at<double>(0,0) = F(0,0);
	F2.at<double>(0,1) = F(0,1);
	F2.at<double>(0,2) = F(0,2);
	F2.at<double>(1,0) = F(1,0);
	F2.at<double>(1,1) = F(1,1);
	F2.at<double>(1,2) = F(1,2);
	F2.at<double>(2,0) = F(2,0);
	F2.at<double>(2,1) = F(2,1);
	F2.at<double>(2,2) = F(2,2);

	Mat mask;
	Mat F_t = findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99, mask);
	Fcv(0,0) = F_t.at<double>(0,0);
	Fcv(0,1) = F_t.at<double>(0,1);
	Fcv(0,2) = F_t.at<double>(0,2);
	Fcv(1,0) = F_t.at<double>(1,0);
	Fcv(1,1) = F_t.at<double>(1,1);
	Fcv(1,2) = F_t.at<double>(1,2);
	Fcv(2,0) = F_t.at<double>(2,0);
	Fcv(2,1) = F_t.at<double>(2,1);
	Fcv(2,2) = F_t.at<double>(2,2);
	E2 =  epi.computeEpipole(Fcv);

	vector<Vec3f> lines;
	computeCorrespondEpilines(points1, 1, F2, lines);

	// for all epipolar lines
	for (auto it = lines.begin(); it != lines.end(); ++it)
	{
		// draw the epipolar line between first and last column
		line(imLc, Point(0, -(*it)[2] / (*it)[1]), Point(imLc.cols, -((*it)[2] + (*it)[0] * imLc.cols) / (*it)[1]), Scalar(255), 1, 8, 0);
	}

	cout << F_t << "\n" << F << "\n"  << endl;
	cout << E1 << "\n" << E2  << endl;

////////////////////////////////////////////////////////////////////////////////////////////
	imshow("Left Image 1", imLc);
//	imshow("Right Image 1", imRc);


	waitKey(0);

	return 0;
}


