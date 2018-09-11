//
// Created by yashmanian on 9/9/18.
//

#include "PointOps.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;


int main()
{
	// Create PointOps object
	PointOps Pt;

	// Create test points
	vector<Point2f> pts;
	Point2f pt1, pt2, pt3, pt4;

	pt1.x = 0, pt1.y = 0;
	pt2.x = 300, pt2.y = 0;
	pt3.x = 300, pt3.y = 400;
	pt4.x = 0, pt4.y = 400;

	pts.push_back(pt1);
	pts.push_back(pt2);
	pts.push_back(pt3);
	pts.push_back(pt4);

	MatrixXd A = Pt.cv2EigenHomogenous(pts);
	cout << A << endl;

	A = Pt.normalise2dpts(A);
	cout << A << endl;

}