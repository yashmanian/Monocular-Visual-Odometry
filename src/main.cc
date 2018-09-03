#include "FeatTrack.hpp"
#include "epipolar.hpp"
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
	Mat img_1, img_2, img_1_c, img_2_c;
	FeatTrack Features;
	epipolar epi;
	double scale = 1.00;
	char filename_img_1[200], filename_img_2[200], filename_gt[200], filename_calib[200];
	
	sprintf(filename_img_1, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/image_2/%06d.png", sequence_id, 0);
	sprintf(filename_img_2, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/image_2/%06d.png", sequence_id, 1);
	sprintf(filename_gt, "/home/yashmanian/Datasets/KITTI_VO/poses/%02d.txt",sequence_id);
	sprintf(filename_calib, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/calib.txt", sequence_id);



	/*-----------------------Read Parameters-----------------------*/
	img_1_c = imread(filename_img_1);
	img_2_c = imread(filename_img_2);
	Mat K = epi.getCameraParams(filename_calib);

	// Throw Exception
	if(!img_1_c.data || !img_2_c.data)
	{
		cout << "--(!) Error reading images" << endl;
		return -1;
	}

	/*-----------------------Operations-----------------------*/
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);


	// Feature detection
	vector<Point2f> points1, points2;	// Vector to store coordinates of features in pixels
	Features.featureDetection(img_1, points1);
	// Feature Tracking
	vector<uchar> status;
	Features.featureTracking(img_1, img_2, points1, points2, status);	// Track features to second image

// Test Script
////////////////////////////////////////////////////////////////////////////////////////////
	clock_t begin = clock();
	Mat F_t = Mat::zeros(3, 3, CV_64F);
	Mat E_t = Mat::zeros(3, 3, CV_64F);
	Mat R, t;

	epi.fundamentalMatrixRANSAC(points1, points2, F_t);
	epi.estimateEssentialMatrix(F_t, E_t);
	epi.estimatePose(E_t, R, t);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

///////////////////////////////////////////////////////////////////////////////////////////

	for(size_t i = 0; i < points1.size(); ++i) // Convert to range based later
	{
		circle(img_2_c, Point(points1[i].x, points1[i].y), 1, CV_RGB(255,0,0), 2);
		circle(img_2_c, Point(points2[i].x, points2[i].y), 1, CV_RGB(255,255,0), 2);
	}	

	imshow("Image 2 color with flow", img_2_c);
	waitKey(0);
	return 0;
}