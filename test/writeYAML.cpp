#include<bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main(){
	cv::FileStorage fs("./output/test.yml", FileStorage::WRITE);
	cv::Mat Tcl_real = (Mat_<double>(4,4) << -0.00710709,   -0.999765,  -0.0204772,   -0.474639,
				0.00616826,   0.0204335,   -0.999772,  -0.0725839,
				0.999956, -0.00723177,  0.00602159,   -0.266694,
				0,           0,           0,           1);
	fs << "Tcl_real" << Tcl_real ;
	fs.release();    	//close the file opened
	return 0;
}