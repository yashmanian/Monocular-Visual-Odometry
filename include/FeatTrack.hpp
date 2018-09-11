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
 *@file FeatTrack.hpp
 *@author Yash Manian
 *@brief Header file with definitions for class FeatTrack.
 *@brief Declaration of methods to track and detect features in images.
 */


#ifndef FEATTRACK_H
#define FEATTRACK_H

// Opencv includes
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// Includes
#include <iostream>
#include <ctype.h>
#include <math.h>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


/**
 *@brief Definition of the FeatTrack class. It contains all the functions and variables for detection, tracking and computing homography.
 */
class FeatTrack
{
private:
	cv::Size winSize;
	cv::TermCriteria termcriteria;
	int fast_thresh;
	bool nonMaxSuppression;
	cv::Mat H  = cv::Mat::eye(3,3, CV_64F);
	int MaxIter, threshHomography;

public:
/**
 *@brief Constructor for the FeatTrack class.
 *@brief Initializes the flow calculation parameters and homography RANSAC parameters.
 */
	FeatTrack();

/**
 *@brief Calculates flow between two images based on detected features.
 *@brief Takes in the two images in the cv::Mat format, and a vector of features from the first image, and a vector of characters to emphasize match.
 */
	void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, std::vector<uchar> &status);

/**
 *@brief Detects FAST features on the input image.
 *@brief Takes in image and an empty vector to store points.
 */
	void featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &img_points);

/**
 *@brief Computes homography between two images.
 *@brief Takes in four matched points from each image to compute affine transform.
 */
	void computeHomography(std::vector<cv::Point2f> &x1, std::vector<cv::Point2f> &x2, cv::Mat &H);

/**
 *@brief Uses RANSAC to refine the Homography matrix.
 *@brief Uses computeHomography as a distance function for the RANSAC. Once inliers are selected, a final Homography matrix is computed using them.
 */
	void homographyRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points);
};

#endif