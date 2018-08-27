#include "VisOdom.hpp"

using namespace std;
using namespace cv;


int main()
{
	// Read Images
	Mat img_1, img_2, img_1_c, img_2_c;
	VisOdom VO;
	double scale = 1.00;
	char filename1[200], filename2[200];
	sprintf(filename1, "/home/yashmanian/Datasets/sequences/00/image_2/%06d.png",0);
	sprintf(filename2, "/home/yashmanian/Datasets/sequences/00/image_2/%06d.png",1);
	
	img_1_c = imread(filename1);
	img_2_c = imread(filename2);

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

	cout << "Rotation\n" << R << endl;
	cout << "Translation\n" << t << endl;

	for(int i = 0; i < points1.size(); ++i)
	{
		circle(img_2_c, Point(points1[i].x, points1[i].y), 0.5, CV_RGB(255,0,0), 2);
		circle(img_2_c, Point(points2[i].x, points2[i].y), 0.5, CV_RGB(255,255,0), 2);
	}	

	imshow("Image 2 color with flow", img_2_c);
	waitKey(0);
	return 0;
}