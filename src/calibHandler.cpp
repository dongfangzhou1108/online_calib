#include"calibHandler.h"

void calibHandler::readParameter(string& configPath){
	cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
	findSize=fsSettings["find_size"];
	searchRadius=fsSettings["search_radius"];
    cv::FileNode n = fsSettings["projection_parameters"];
	K=Eigen::Matrix3d::Identity();
    K(0,0) = static_cast<double>(n["fx"]);
    K(1,1) = static_cast<double>(n["fy"]);
    K(0,2) = static_cast<double>(n["cx"]);
    K(1,2) = static_cast<double>(n["cy"]);
	cv::Mat Tcl_real_cv;
	fsSettings["Tcl_real"]>>Tcl_real_cv;
	cv::cv2eigen(Tcl_real_cv,Tcl_real);
	Tcl_calib=Tcl_real;
	Eigen::Vector3d eulerAngle=Tcl_real.block<3,3>(0,0).eulerAngles(2,1,0);//zyx:roll+pitch+yaw
	cv::RNG rng((unsigned)time(NULL));
	for(int i=0;i<3;i++){
		eulerAngle(i,0)+=rng.uniform(-3.0/180.0*CV_PI,3.0/180.0*CV_PI);
		Tcl_calib(i,3)+=rng.uniform(-0.05,0.05);
	}
	Eigen::Matrix3d rot_calib;
	rot_calib=Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()) * 
						Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()) * 
						Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX());
	Tcl_calib.block<3,3>(0,0)=rot_calib;
}

cv::Mat calibHandler::projPC2Img(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,cv::Mat& img,Eigen::Matrix4d& Tcl){
	cv::Mat proj=cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	vector<cv::Point2f> projPts;
	for(int i=0;i<pc->points.size();i++){
		Eigen::Vector3d ptsL(pc->points[i].x,pc->points[i].y,pc->points[i].z);
		Eigen::Vector3d ptsC = Tcl.block<3,3>(0,0) * ptsL + Tcl.block<3,1>(0,3);
		ptsC=1.0/ptsC.z()*ptsC;//归一化坐标
		Eigen::Vector3d pixel= K * ptsC;
		cv::Point2f pixelCV(pixel.x(),pixel.y());
		if(cv::Rect(0, 0, img.cols, img.rows).contains(pixelCV)){
			proj.at<uchar>((int)pixel.y(),(int)pixel.x())=(uchar)(ptsL.norm()/pcHandler_->distThres*255.0);
			projPts.push_back(pixelCV);
		}
	}
	cv::Mat projRGB=cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
	cv::applyColorMap(proj, projRGB, cv::COLORMAP_JET);
	cv::Mat B(img.rows, img.cols, CV_8UC1);
	cv::Mat G(img.rows, img.cols, CV_8UC1);
	cv::Mat R(img.rows, img.cols, CV_8UC1);
	cv::Mat BGR[] = {B,G,R};
	cv::split(projRGB, BGR);
	cv::Mat res;
	cv::cvtColor(img, res, CV_GRAY2BGR);
	for(auto&pts:projPts){
		uchar b=B.at<uchar>((int)pts.y,(int)pts.x);
		uchar g=G.at<uchar>((int)pts.y,(int)pts.x);
		uchar r=R.at<uchar>((int)pts.y,(int)pts.x);
		cv::circle(res, pts, 2,cv::Scalar(b,g,r),2);
	}
	return res;
}

void calibHandler::findCorrespondence(){
	for(int i=0;i<edgePc->points.size();i++){
		Eigen::Vector3d ptsL(edgePc->points[i].x,edgePc->points[i].y,edgePc->points[i].z);
		Eigen::Vector3d ptsC = Tcl_calib.block<3,3>(0,0) * ptsL + Tcl_calib.block<3,1>(0,3);
		ptsC=1.0/ptsC.z()*ptsC;
		Eigen::Vector3d pixel=K*ptsC;
		cv::Point2f pixelCV(pixel.x(),pixel.y());
		if(cv::Rect(0, 0, imgHandler_->src.cols, imgHandler_->src.rows).contains(pixelCV)){
			pcl::PointXYZI p;
			p.x=(pixel.x()-K(0,2))/K(0,0);
			p.y=(pixel.y()-K(1,2))/K(1,1);
			p.z=1.0;
			p.intensity=1.0;
			vector<int> k_indices(findSize);
			vector<float> k_sqr_distances(findSize);
			if(kdTree.nearestKSearch(p,findSize,k_indices,k_sqr_distances)>0){
				correspondence[i]=edgeImg->points[k_indices[0]];
			}
		}
	}
}