#include "FeatTrack.hpp"
#include "epipolar.hpp"
#include <ctime>

using namespace std;
using namespace cv;
using namespace Eigen;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

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
	char filename_img_1[200], filename_img_2[200], filename_gt[200], filename_calib[200], filename_loop[200], text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);
	
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
	Mat R, t, R_f, t_f, prevImg, currImg, currImg_c;

	epi.fundamentalMatrixRANSAC(points1, points2, F_t);
	epi.estimateEssentialMatrix(F_t, E_t);
	epi.estimatePose(E_t, R, t, points1, points2);

	// Display setup
	namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
	//namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
	Mat traj = Mat::zeros(600, 600, CV_8UC3);

	R_f = R.clone();
	t_f = t.clone();

	prevImg = img_2.clone();

	// Loop
	for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)
	{

		// Read image from location
		sprintf(filename_loop, "/home/yashmanian/Datasets/KITTI_VO/sequences/00/image_2/%06d.png", numFrame);
		currImg_c = imread(filename_loop);

		if(!currImg_c.data)
		{
			cout << "--(!) Error reading images" << endl;
			return -1;
		}

		// Convert to grayscale
		cvtColor(currImg_c, currImg, COLOR_BGR2GRAY);
		// Track features
		Features.featureTracking(prevImg, currImg, points1, points2, status);

		// Estimate fundamental matrix
		epi.fundamentalMatrixRANSAC(points1, points2, F_t);

		// If features drop below a threshold, trigger redetection
		if (points1.size() < MIN_NUM_FEAT)
		{
			Features.featureDetection(prevImg, points1);
			Features.featureTracking(prevImg, currImg, points1, points2, status);
		}

		prevImg = currImg.clone();
		points1 = points2;

		imshow("Road facing camera", currImg_c);

		waitKey(1);
	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

///////////////////////////////////////////////////////////////////////////////////////////

	return 0;
}