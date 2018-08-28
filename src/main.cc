#include "VisOdom.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

int main()
{
	// Path variables
	int sequence_id = 0;
	int frame_id = 1;
	// Read Images
	Mat img_1, img_2, img_1_c, img_2_c;
	VisOdom VO;
	double scale = 1.00;
	char filename_img_1[200], filename_img_2[200], filename_gt[200], filename_calib[200];
	
	sprintf(filename_img_1, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/image_2/%06d.png", sequence_id, 0);
	sprintf(filename_img_2, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/image_2/%06d.png", sequence_id, 1);
	sprintf(filename_gt, "/home/yashmanian/Datasets/KITTI_VO/poses/%02d.txt",sequence_id);
	sprintf(filename_calib, "/home/yashmanian/Datasets/KITTI_VO/sequences/%02d/calib.txt", sequence_id);

	Matrix3d K = VO.getCameraParams(filename_calib);

	cout << "Camera Params = " << endl << K << endl;

	img_1_c = imread(filename_img_1);
	img_2_c = imread(filename_img_2);

	// Throw Exception
	
	if(!img_1_c.data || !img_2_c.data)
	{
		cout << "--(!) Error reading images" << endl;
		return -1;
	}

	// Convert to grayscale
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

	// Feature detection
	vector<Point2f> points1, points2;	// Vector to store coordinates of features in pixels
	VO.featureDetection(img_1, points1);

	// Feature Tracking
	vector<uchar> status;
	VO.featureTracking(img_1, img_2, points1, points2, status);	// Track features to second image

	// Compute Essential Matrix
	Mat E, mask;
	VO.computeEssentialMatrix(E, points1, points2, mask);

	// Pose Recovery
	Mat R, t;
	VO.computePose(E, R, t, points1, points2, mask);

	// Scale
	scale = VO.getAbsoluteScale(filename_gt, frame_id);

	// cout << "Rotation\n" << R << endl;
	// cout << "Translation\n" << t << endl;
	// cout << "Scale: " << scale << endl;

	for(int i = 0; i < points1.size(); ++i)
	{
		circle(img_2_c, Point(points1[i].x, points1[i].y), 0.5, CV_RGB(255,0,0), 2);
		circle(img_2_c, Point(points2[i].x, points2[i].y), 0.5, CV_RGB(255,255,0), 2);
	}	

	imshow("Image 2 color with flow", img_2_c);
	waitKey(0);
	return 0;
}