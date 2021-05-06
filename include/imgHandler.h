#ifndef IMG_HANDLER_H
#define IMG_HANDLER_H

#include<bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;

class imgHandler{
public:
	imgHandler(string imgPath,string configPath){
		readParameter(configPath);
		src = cv::imread(imgPath,0);
		run();
	}
	void readParameter(string& configPath);
	void run();
	void imgPreprocess();//标定前的图像预处理
	void extractEdge();//提取图像边缘
public:
	int gaussianSize,cannyThres,contoursThres;
	cv::Mat src,gray;
	cv::Mat canny,edgeRGB,edgeGray;//边缘图片
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
};

#endif