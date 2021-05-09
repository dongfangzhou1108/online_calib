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
#include <pcl/io/pcd_io.h>
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
				if(!imgHandler_->edgeGray.at<uchar>(i,j))
					continue;
				pcl::PointXYZI p;
				p.x=((double)j-K(0,2))/K(0,0);
				p.y=((double)i-K(1,2))/K(1,1);
				p.z=1.0;
				p.intensity=(float)imgHandler_->edgeGray.at<uchar>(i,j);
				imgTmp->points.push_back(p);
			}
		}
		edgeImg=imgTmp;
		edgeImg->height=1;
		edgeImg->width=edgeImg->points.size();
		pcl::io::savePCDFileASCII("./output/edgeImg.pcd", *edgeImg);
		kdTree.setInputCloud(edgeImg);
		calcEdgeImgUsed();
		edgeImgUsed->height=1;
		edgeImgUsed->width=edgeImgUsed->points.size();
		pcl::io::savePCDFileASCII("./output/edgeImgUsed.pcd", *edgeImgUsed);
		pcl::KdTreeFLANN<pcl::PointXYZI> kdTree_;
		kdTree_.setInputCloud(edgeImgUsed);
		kdTree=kdTree_;
	}
	void readParameter(string& configPath);
	void calcEdgeImgUsed();//计算与点云边缘真正对应的图像边缘
	//for Debug
	cv::Mat projPC2Img(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,cv::Mat& img,Eigen::Matrix4d& Tcl);
	// bool calib();
	void findCorrespondence();
	cv::Mat drawCorrespondence(cv::Mat proj);
	void updateTcl();
	// void calcScore();
public:
	imgHandler* imgHandler_;
	pcHandler* pcHandler_;
	Eigen::Matrix3d K;
	Eigen::Matrix4d Tcl_real,Tcl_calib;
	//归一化坐标
	map<int,pcl::PointXYZI> projPts;
	map<int,pcl::PointXYZI> correspondence;
	pcl::PointCloud<pcl::PointXYZI>::Ptr edgePc,edgeImg,edgeImgUsed;
	int findSize;
	double searchRadius,deltaTheta,deltaX;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdTree;
};

#endif