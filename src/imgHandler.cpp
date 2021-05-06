#include "imgHandler.h"
using namespace std;

void imgHandler::readParameter(string& configPath){
	cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
	gaussianSize=fsSettings["gaussian_size"];
	cannyThres=fsSettings["canny_threshold"];
	contoursThres=fsSettings["contours_threshold"];
}

void imgHandler::run(){
	imgPreprocess();
	extractEdge();
}

void imgHandler::imgPreprocess(){
	cv::equalizeHist(src,gray);
	cv::GaussianBlur(gray,gray,cv::Size(gaussianSize, gaussianSize), 0, 0);
}

void imgHandler::extractEdge(){
	cv::Canny(gray, canny, cannyThres, cannyThres * 3, 3, true);
	cv::findContours(canny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
	edgeGray=cv::Mat::zeros(src.size(),CV_8UC1);
    for(int i=0;i<contours.size();i++) { 
		if(contours[i].size()<contoursThres)
			continue;
        cv::drawContours(edgeGray,contours,i,cv::Scalar((uchar)contours[i].size()),1,8,hierarchy);
    } 
	cv::applyColorMap(edgeGray, edgeRGB, cv::COLORMAP_JET);
}