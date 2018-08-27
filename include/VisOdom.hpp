#ifndef VISODOM_H
#define VISODOM_H

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

class VisOdom
{
private:
	cv::Size winSize;
	cv::TermCriteria termcriteria;
	int fast_thresh;
	bool nonMaxSuppression;
	double focal, epipolarThresh, probEssential;
	cv::Point2d pp;
public:
	VisOdom();
	void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, std::vector<uchar> &status);
	void featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &points1);
	double getAbsoluteScale(int frame_id, int sequence_id);
	void computeEssentialMatrix(cv::Mat &E, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask);
	void computePose(cv::Mat &E, cv::Mat &R, cv::Mat &t, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &mask);
};

#endif