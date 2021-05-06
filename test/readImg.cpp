#include<bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;

// string imgPath = "/home/dfz/dfz/master/code note/CamVox/src/CamVox/isee-camvox/camvox/calibration/calibration.bmp";
string imgPath = "./data/0000000000.png";
int gaussianSize=5;
int cannyThres=30;
int contoursLen=50;

int main(){
	cv::Mat src = cv::imread(imgPath,0);

	cv::Mat gray;
	cv::equalizeHist(src, gray);
	GaussianBlur(gray, gray, cv::Size(gaussianSize, gaussianSize), 0, 0);

	cv::Mat canny;
	Canny(gray, canny, cannyThres, cannyThres * 3, 3, true);

	vector<vector<cv::Point>> contoursPts;
	vector<cv::Vec4i> hierarchy;
	cv::findContours(canny, contoursPts, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
    cv::Mat contoursEdge=cv::Mat::zeros(src.size(),CV_8UC1);
    for(int i=0;i<contoursPts.size();i++) { 
		if(contoursPts[i].size()<contoursLen)
			continue;
        cv::drawContours(contoursEdge,contoursPts,i,cv::Scalar((uchar)contoursPts[i].size()),1,8,hierarchy);
    } 
	cv::Mat contoursEdgeRGB;
	cv::applyColorMap(contoursEdge, contoursEdgeRGB, cv::COLORMAP_JET);

	cv::imshow("src", src);
	cv::imshow("equalizeHist+GaussianBlur", gray);
	cv::imshow("Canny", canny);
    cv::imshow("contoursEdgeRGB",contoursEdgeRGB); 
	cv::imwrite("/home/dfz/dfz/master/Code/calibCameraLiDAR/output/cannyEdge.png",contoursEdgeRGB);
    cv::waitKey();
    cv::destroyAllWindows();
	return 0;
}