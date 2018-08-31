#ifndef EPIPOLAR_H
#define EPIPOLAR_H

// Opencv includes
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
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



class epipolar
{
private:
	cv::Size winSize;
	cv::TermCriteria termcriteria;
	double fx, fy, skew, cx, cy;
	cv::Point2d pp;
	cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
	int MaxIter;

public:
	epipolar();
	cv::Mat getCameraParams(char filename_calib[200]);
	double getAbsoluteScale(char filename_gt[200], int frame_id);
	void estimateFundamentalMatrix(std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &F);
	void fundamentalMatrixRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, cv::Mat &F);
};

#endif