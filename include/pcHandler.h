#ifndef PC_HANDLER_H
#define PC_HANDLER_H

#include<bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

using namespace std;

class pcHandler{
public:
	pcHandler(string& pcPath,string& configPath);
	void readParameter(string& configPath);
	void run();
	void readLidarData(string& pcPath);
	void separateInScans();
	void extractEdge();
public:
	int scanNum;
	double depThres,distThres,edgeThres,intensityThres;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc,pcEdge;
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudScans;
};

#endif