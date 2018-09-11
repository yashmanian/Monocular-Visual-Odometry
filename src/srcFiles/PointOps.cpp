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
 *@file PointOps.cpp
 *@author Yash Manian
 *@brief Function definitions for methods declared in class PointOps.
 *@brief The following is a collection of methods to manipulate point formats.
 */

#include "PointOps.hpp"

// Normalize points about centroid.
Eigen::MatrixXd PointOps::normalise2dpts(Eigen::MatrixXd &pts)
{
	Eigen::MatrixXd homogenous(3, pts.size());
	Eigen::RowVectorXd newX(pts.size()), newY(pts.size()), distance;
	Eigen::Matrix3d T;

	// Enforce nomalization
	pts.array().rowwise() /= pts.row(2).array();

	// Find mean
	double meanX = pts.row(0).mean();
	double meanY = pts.row(1).mean();

	newX = pts.row(0).array() - meanX;
	newY = pts.row(1).array() - meanY;

	distance = newX.array()*newX.array() + newY.array()*newY.array();
	distance = distance.array().sqrt();
	double meanDist = distance.mean();
	double scale = sqrt(2)/meanDist;

	T << scale, 0, -scale*meanX,
			0, scale, -scale*meanY,
			0, 0, 1;

	return T;
}


// Converts points from vector of openCV features to an Eigen matrix
Eigen::MatrixXd PointOps::cv2EigenHomogenous(std::vector<cv::Point2f> &pts)
{
	Eigen::MatrixXd homogenous(3, pts.size());
	for(int i = 0; i < pts.size(); ++i)
	{
		homogenous.col(i) << pts[i].x, pts[i].y, 1;
	}
	return homogenous;
}