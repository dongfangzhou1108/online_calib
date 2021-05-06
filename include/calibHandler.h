#ifndef CALIB_HANDLER_H
#define CALIB_HANDLER_H

#include<bits/stdc++.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include"opencv2/core/eigen.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include"imgHandler.h"
#include"pcHandler.h"
using namespace std;

class calibHandler{
public:
	calibHandler(string& configPath,imgHandler* imgH,pcHandler* pcH)
		:imgHandler_(imgH),pcHandler_(pcH){
		readParameter(configPath);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcTmp(new pcl::PointCloud<pcl::PointXYZI>);
		pcTmp->points=pcHandler_->pcEdge->points;
		edgePc=pcTmp;
		pcl::PointCloud<pcl::PointXYZI>::Ptr imgTmp(new pcl::PointCloud<pcl::PointXYZI>);
		for(int i=0;i<imgHandler_->edgeGray.rows;i++){
			for(int j=0;j<imgHandler_->edgeGray.cols;j++){
				pcl::PointXYZI p;
				p.x=((double)j-K(0,2))/K(0,0);
				p.y=((double)i-K(1,2))/K(1,1);
				p.z=1.0;
				p.intensity=(float)imgHandler_->edgeGray.at<uchar>(i,j);
				imgTmp->points.push_back(p);
			}
		}
		edgeImg=imgTmp;
		kdTree.setInputCloud(edgeImg);
	}
	void readParameter(string& configPath);
	//for Debug
	cv::Mat projPC2Img(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,cv::Mat& img,Eigen::Matrix4d& Tcl);
	// bool calib();
	void findCorrespondence();
	// void updateTcl();
	// void calcScore();
public:
	imgHandler* imgHandler_;
	pcHandler* pcHandler_;
	Eigen::Matrix3d K;
	Eigen::Matrix4d Tcl_real,Tcl_calib;
	map<int,pcl::PointXYZI> correspondence;
	pcl::PointCloud<pcl::PointXYZI>::Ptr edgePc,edgeImg;
	int findSize;
	double searchRadius;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdTree;
};

#endif