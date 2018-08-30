#ifndef FEATTRACK_H
#define FEATTRACK_H

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

class FeatTrack
{
private:
	cv::Size winSize;
	cv::TermCriteria termcriteria;
	int fast_thresh;
	bool nonMaxSuppression;
	cv::Mat H  = cv::Mat::eye(3,3, CV_64F);

public:
	FeatTrack();
	void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points, std::vector<uchar> &status);
	void featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &img_points);
	//void homographyRANSAC(std::vector<cv::Point2f> &img_1_points, std::vector<cv::Point2f> &img_2_points);
};

#endif