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
 *@file PointOps.hpp
 *@author Yash Manian
 *@brief Header file with definitions for class PointOps.
 *@brief Declaration of methods to manipulate point formats.
 */

#ifndef POINTOPS_H
#define POINTOPS_H

// Includes
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "opencv2/features2d/features2d.hpp"


/**
 *@brief Definition of the PointOps class. It contains all the functions and variables required manipulate point formats.
 *@brief This class will contain methods to run mathematical operations on sets of matched points.
 */

class PointOps
{
private:

public:
/**
 *@brief Normalises the point set around its centroid.
 *@brief Takes in a 3xN matrix of points and returns a 3xN matrix of normalised homogenous coordinates.
 */
	Eigen::MatrixXd normalise2dpts(Eigen::MatrixXd &pts);

/**
 *@brief Converts a vector of 2D points to a 3xN matrix of homogenous points.
 *@brief Allows conversion from openCV Point2f features to Eigen Matrices.
 */
	Eigen::MatrixXd cv2EigenHomogenous(std::vector<cv::Point2f> &pts);
};


#endif //POINTOPS_H
