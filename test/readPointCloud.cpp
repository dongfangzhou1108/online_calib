#include<bits/stdc++.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

string binPath = "./data/0000000000.bin";
string imgPath = "./data/0000000000.png";
double depThres = 0.1;//深度阈值
float distThres=20.0;
double edgeThres=1.0;
double intensityThres=0.15;
int N_SCANS = 64;//velodyne线束
float startOri = 0;
float endOri = 0;

void readLidarData(const string binPath, vector<float>& binBuf) {
    ifstream binFile(binPath, ifstream::in | ifstream::binary);
    binFile.seekg(0, ios::end);
    const size_t fileSize = binFile.tellg() / sizeof(float);
    binFile.seekg(0, ios::beg);

	binBuf.clear();
    binBuf.resize(fileSize);
    binFile.read(reinterpret_cast<char*>(&binBuf[0]), fileSize*sizeof(float));
    return;
}

void separateInScans(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc, vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloudScans) {
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
				pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
				cloudScans[line]=tmp;
				cloudScans[line]->points=scan->points;
				scan->points.clear();
				line++;
			}
			scan->points.push_back(p);
			preAngle=angle;
		}
	}
	if(scan->points.size()!=0)
		cloudScans[line]=scan;
	return;
}

void extratPcEdge(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,pcl::PointCloud<pcl::PointXYZI>::Ptr pcEdge){
	for(int i=1;i<pc->points.size()-1;i++){
		if(pc->points[i].x<0)
			continue;
		float dist=sqrt(pc->points[i].x*pc->points[i].x+pc->points[i].y*pc->points[i].y+pc->points[i].z*pc->points[i].z);
		if(dist>distThres)
			continue;
		if(max(abs(pc->points[i-1].intensity - pc->points[i].intensity),abs(pc->points[i+1].intensity - pc->points[i].intensity))>intensityThres
			&& max(pc->points[i-1].x - pc->points[i].x,pc->points[i+1].x - pc->points[i].x)>edgeThres)
			pcEdge->points.push_back(pc->points[i]);
	}
}

void projPc2Img(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,cv::Mat& src,vector<cv::Point2f>& projPts,Eigen::Matrix4d Tcl,Eigen::Matrix3d K){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
	cv::Mat proj=cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
	for(int i=0;i<pc->points.size();i++){
		Eigen::Vector3d ptsL(pc->points[i].x,pc->points[i].y,pc->points[i].z);
		Eigen::Vector3d ptsC = Tcl.block<3,3>(0,0) * ptsL + Tcl.block<3,1>(0,3);
		ptsC=1.0/ptsC.z()*ptsC;
		Eigen::Vector3d pixel= K * ptsC;
		cv::Point2f pixelCV(pixel.x(),pixel.y());
		if(cv::Rect(0, 0, src.cols, src.rows).contains(pixelCV)){
			proj.at<uchar>((int)pixel.y(),(int)pixel.x())=(uchar)(ptsL.x()/distThres*255.0);
			tmp->points.push_back(pc->points[i]);
			projPts.push_back(pixelCV);
		}
	}
	applyColorMap(proj, src, cv::COLORMAP_JET);
	pc=tmp;
}

int main(){
    Eigen::Matrix4d Tcl;
    Tcl << -0.00710709,   -0.999765,  -0.0204772,   -0.474639,
				0.00616826,   0.0204335,   -0.999772,  -0.0725839,
				0.999956, -0.00723177,  0.00602159,   -0.266694,
				0,           0,           0,           1;
    Eigen::Matrix3d K;
    K << 7.215377e+02 ,0.000000e+00 ,6.095593e+02 ,
			0.000000e+00 ,7.215377e+02 ,1.728540e+02 ,
			0.000000e+00 ,0.000000e+00 ,1.000000e+00;

	vector<float> binBuf;
	readLidarData(binPath, binBuf);

	pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
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
		pc->push_back(p);
	}

    int cloudSize = pc->points.size();
	startOri = -atan2(pc->points[0].y, pc->points[0].x);
	endOri = -atan2(pc->points[cloudSize - 1].y, pc->points[cloudSize - 1].x) + 2 * M_PI;
    if(endOri - startOri > 3 * M_PI) // 确保 PI < endOri - startOri < 3*PI
		endOri -= 2 * M_PI; 
    else if(endOri - startOri < M_PI) 
		endOri += 2 * M_PI;

	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudScans(N_SCANS, nullptr);
	separateInScans(pc, cloudScans);

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcEdge(new pcl::PointCloud<pcl::PointXYZI>);
	for(int i = 0; i < N_SCANS; i++){
		if(!cloudScans[i])
			continue;
		extratPcEdge(cloudScans[i],pcEdge);
	}

	cv::Mat src = cv::imread(imgPath,0);
	cv::Mat projColorMap=cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
	vector<cv::Point2f> projPts;
	projPc2Img(pcEdge,projColorMap,projPts,Tcl,K);

	cv::Mat output;
    cv::cvtColor(src, output, CV_GRAY2BGR);
	cv::Mat B(src.rows, src.cols, CV_8UC1);
	cv::Mat G(src.rows, src.cols, CV_8UC1);
	cv::Mat R(src.rows, src.cols, CV_8UC1);
	cv::Mat out[] = {B,G,R};
	cv::split(projColorMap, out);
	for(auto&pts:projPts){
		uchar b=B.at<uchar>((int)pts.y,(int)pts.x);
		uchar g=G.at<uchar>((int)pts.y,(int)pts.x);
		uchar r=R.at<uchar>((int)pts.y,(int)pts.x);
		cv::circle(output, pts, 2,cv::Scalar(b,g,r),2);
	}
    cv::imshow("output", output);
    cv::waitKey();
    cv::destroyAllWindows();
	cv::imwrite("/home/dfz/dfz/master/Code/calibCameraLiDAR/output/pointCloudEdge.png",output);

	return 0;
}