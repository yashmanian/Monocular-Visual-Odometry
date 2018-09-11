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
 *@file epipolar.hpp
 *@author Yash Manian
 *@brief Header file with definitions for class epipolar.
 *@brief Declaration of methods to compute Epipolar constraints
 */


#ifndef EPIPOLAR_H
#define EPIPOLAR_H

// Opencv includes
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// Includes
#include <iostream>
#include <ctype.h>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "PointOps.hpp"


/**
 *@brief Definition of the epipolar class. It contains all the functions and variables required to compute epipolar geometry.
 *@brief The sole purpose of this class is to compute geometry based on matched points.
 *@brief Inherits the PointOps class and uses the mathematical conversion methods from it.
 */

class epipolar: public PointOps
{
private:
	double fx, fy, skew, cx, cy;
	Eigen::Matrix3d K;
	int MaxIter;
	double RANSAC_THRESH, inlierThresh;

public:

/**
 *@brief Constructor for the epipolar class.
 *@brief Initializes the camera matrix to default values in the event of file reading failure. Initializes RANSAC variables.
 */
	epipolar();

/**
 *@brief Gets camera parameters from the calib.txt files in the dataset. The folder structure is assumed to be similar to the KITTI dataset
 *@brief Reads the parameters from the text files and stores it in the K matrix parameter. If reading the file fails, default values are used.
 */
	Eigen::Matrix3d getCameraParams(char filename_calib[200]);

/**
 *@brief Gets scale from ground truth file depending on the image sequence, from the pose folder.
 *@brief Ground truth is used to get scale for this application. This can be switched out to get scale from another source, like a speedometer.
 */
	double getAbsoluteScale(char filename_gt[200], int frame_id);

/**
 *@brief Estimates fundamental matrix from matched points. Takes in the matrix F by reference which is updated.
 *@brief This version uses the 8 point algorithm and solves the Linear Least Squares problem Ax = 0 using SVD.
 *@brief The method can take more than 8 points and still compute the matrix.
 *@brief Takes in the matched points in the form of a 3xN matrix of homogenous coordinates.
 */
	void estimateFundamentalMatrix(Eigen::MatrixXd &points1, Eigen::MatrixXd &points2, Eigen::Matrix3d &F);

/**
 *@brief Uses RANSAC to refine the Fundamental matrix. This takes advantage of the fact that the features lie on the epipolar plane.
 *@brief Uses x2T*F*x1 = 0 as a distance function for the RANSAC. Once inliers are selected, a final fundamental matrix is computed using them.
 */
	void FundamentalMatRANSAC(Eigen::Matrix3d &F, Eigen::MatrixXd &points1, Eigen::MatrixXd &points2);

/**
 *@brief Computes epipole based on the idea that the epipoles lie on the epipolar plane. So Linear Least Squares to solve e2T*F*e1 = 0.
 *@brief Takes in the Fundamental matrix and returns a 3x2 matrix in homogenous form, with the coordinates for the epipole.
 */
	Eigen::MatrixXd computeEpipole(Eigen::Matrix3d &F);
};

#endif