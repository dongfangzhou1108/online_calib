#include"pcHandler.h"

pcHandler::pcHandler(string& pcPath,string& configPath){
	readParameter(configPath);
	readLidarData(pcPath);
	run();
}

void pcHandler::readParameter(string& configPath){
	cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
	depThres=fsSettings["depth_threshold"];
	scanNum=fsSettings["scan_num"];
	distThres=fsSettings["diatance_threshold"];
	edgeThres=fsSettings["point_cloud_edge_threshold"];
	intensityThres=fsSettings["intensity_threshold"];
}

void pcHandler::readLidarData(string& pcPath){
    ifstream binFile(pcPath, ifstream::in | ifstream::binary);
    binFile.seekg(0, ios::end);
    const size_t fileSize = binFile.tellg() / sizeof(float);
    binFile.seekg(0, ios::beg);
	vector<float> binBuf;
    binBuf.resize(fileSize);
    binFile.read(reinterpret_cast<char*>(&binBuf[0]), fileSize*sizeof(float));

	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointXYZI p;
	for(int i = 0; i < (int)binBuf.size(); i += 4) {
		p.x = binBuf[i];
		p.y = binBuf[i + 1];
		p.z = binBuf[i + 2];
		p.intensity = binBuf[i+3];
		bool isFinite = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);//移除NaN点
		if(!isFinite) continue;
		float dist = pow(p.x,2) + pow(p.y,2) + pow(p.z,2);
		bool inRange = dist > pow(depThres,2);//移除closed点
		if(!inRange) continue;
		tmp->points.push_back(p);
	}
	pc=tmp;
    return;
}

void pcHandler::run(){
	separateInScans();
	extractEdge();
}

void pcHandler::separateInScans(){
	cloudScans.clear();
	cloudScans.resize(scanNum);
	for(int i=0;i<cloudScans.size();i++){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
		cloudScans[i]=tmp;
	}
	pcl::PointXYZI p;
    int line=0,count=0;
    double preAngle = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
	for(int i=0;i<pc->points.size();i++){
		p=pc->points[i];
		double angle = atan2(p.y,p.x) * 180.0 / 3.1415926535898;
		if(i==0) 
			preAngle=angle;
		else{
			if(preAngle<0&&angle>0){
				cloudScans[line]->points=scan->points;
				scan->points.clear();
				line++;
			}
			scan->points.push_back(p);
			preAngle=angle;
		}
	}
	if(scan->points.size()!=0)
		cloudScans[line]->points=scan->points;
}

void pcHandler::extractEdge(){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
	for(int i = 0; i < scanNum; i++){
		if(cloudScans[i]->points.size()==0)
			continue;
		for(int j=1;j<cloudScans[i]->points.size()-1;j++){
			if(cloudScans[i]->points[j].x<0)
				continue;
			float dist=sqrt(cloudScans[i]->points[j].x*cloudScans[i]->points[j].x+
				cloudScans[i]->points[j].y*cloudScans[i]->points[j].y+
				cloudScans[i]->points[j].z*cloudScans[i]->points[j].z);
			if(dist>distThres)
				continue;
			if(max(abs(cloudScans[i]->points[j-1].intensity - cloudScans[i]->points[j].intensity),
					abs(cloudScans[i]->points[j+1].intensity - cloudScans[i]->points[j].intensity))>intensityThres
				&& max(cloudScans[i]->points[j-1].x - cloudScans[i]->points[j].x,
					cloudScans[i]->points[j+1].x - cloudScans[i]->points[j].x)>edgeThres)
				tmp->points.push_back(cloudScans[i]->points[j]);
		}
	}
	pcEdge=tmp;
}