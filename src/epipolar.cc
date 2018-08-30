#include "epipolar.hpp"
epipolar::epipolar()
{
	// Initialize Camera Parameters to default value, in case file read goes wrong
	fx = 718.8560;
	fy = 718.8560;
	cx = 607.1928;
	cy = 185.2157;
	skew = 0;
}


// Compute absolute scale from Ground truth
double epipolar::getAbsoluteScale(char filename_gt[200], int frame_id)
{
	std::string line;
	int i = 0;
	double x_prev = 0, y_prev = 0, z_prev = 0;
	double x = 0, y = 0, z = 0;

	std::ifstream poseFile(filename_gt);

	if(poseFile.is_open())
	{
		while(getline(poseFile, line) && i<=frame_id)
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;

			std::istringstream in(line);

			for(int j = 0; j<12; ++j)
			{
				in >> z;
				if(j == 7) y = z;
				if(j == 3) x = z;
			}
			i++;
		}
	}
	else
	{
		std::cout << "Unable to open Pose file!" << std::endl;
		return 0;
	}

	poseFile.close();

	return sqrt((x-x_prev)*(x-x_prev)  + (y-y_prev)*(y-y_prev)  + (z-z_prev)*(z-z_prev));
}

// Get Camera Calibration Matrix from calib.txt. Implementation not pretty. Will change later
cv::Mat epipolar::getCameraParams(char filename_calib[200])
{
	std::ifstream calibFile;
	calibFile.open(filename_calib);

	if(calibFile.fail())
	{
		std::cerr << "* Error opening file!" << std::endl;
		exit(1);
	}

	std::string lineA, x;
	double BS;
	if(std::getline(calibFile, lineA))
	{
		std::istringstream streamA(lineA);

		for(int n = 0; n < 13; n++)
		{
			if(n == 0)
			{
				streamA >> x;
			}
			if(n == 1)
			{
				streamA >> this->fx;
			}
			if(n == 2)
			{
				streamA >> this->skew;
			}
			if(n == 3)
			{
				streamA >> this->cx;
			}
			if(n == 4)
			{
				streamA >> BS;
			}
			if(n == 5)
			{
				streamA >> BS;
			}
			if(n == 6)
			{
				streamA >> this->fy;
			}
			if(n == 7)
			{
				streamA >> this->cy;
			}
		}

	}
	this->pp = cv::Point2d(cx, cy);
	this->K.at<double>(0,0) = fx;
	this->K.at<double>(0,1) = skew;
	this->K.at<double>(0,2) = cx;
	this->K.at<double>(1,1) = fy;
	this->K.at<double>(2,2) = 1;
	return K;
}