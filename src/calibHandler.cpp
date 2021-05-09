#include"calibHandler.h"

void calibHandler::calcEdgeImgUsed(){
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
	for(int i=0;i<edgePc->points.size();i++){
		Eigen::Vector3d ptsL(edgePc->points[i].x,edgePc->points[i].y,edgePc->points[i].z);
		Eigen::Vector3d ptsC = Tcl_real.block<3,3>(0,0) * ptsL + Tcl_real.block<3,1>(0,3);
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
				tmp->points.push_back(p);
			}
		}
	}
	edgeImgUsed=tmp;
}

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
	deltaTheta=fsSettings["delta_theta"];
	deltaX=fsSettings["delta_dx"];
	for(int i=0;i<3;i++){
		eulerAngle(i,0)+=rng.uniform(-deltaTheta/180.0*CV_PI,deltaTheta/180.0*CV_PI);
		Tcl_calib(i,3)+=rng.uniform(-deltaX,deltaX);
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
		cv::circle(res, pts, 1,cv::Scalar(b,g,r),1);
	}
	return res;
}

void calibHandler::findCorrespondence(){
	projPts.clear();
	correspondence.clear();
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
				projPts[i]=p;
				// correspondence[i]=edgeImg->points[k_indices[0]];
				//下部分使用的是图像确定匹配的部分
				correspondence[i]=edgeImgUsed->points[k_indices[0]];
			}
		}
	}
}

cv::Mat calibHandler::drawCorrespondence(cv::Mat proj){
	cv::Mat B(proj.rows, proj.cols, CV_8UC1);
	cv::Mat G(proj.rows, proj.cols, CV_8UC1);
	cv::Mat R(proj.rows, proj.cols, CV_8UC1);
	cv::Mat BGR[] = {B,G,R};
	cv::split(proj, BGR);
	cv::Mat res=proj.clone();
	// cv::Mat res;
	// cv::cvtColor(imgHandler_->gray, res, CV_GRAY2BGR);
	for(auto&p:projPts){
		pcl::PointXYZI ptsL=p.second;
		pcl::PointXYZI ptsC=correspondence[p.first];
		Eigen::Vector3d ptsL_(ptsL.x,ptsL.y,ptsL.z);
		Eigen::Vector3d ptsC_(ptsC.x,ptsC.y,ptsC.z);
		Eigen::Vector3d pixelL_=K*ptsL_;
		Eigen::Vector3d pixelC_=K*ptsC_;
		cv::Point2f pixelL(pixelL_.x(),pixelL_.y());
		cv::Point2f pixelC(pixelC_.x(),pixelC_.y());
		uchar b=B.at<uchar>((int)pixelL.y,(int)pixelL.x);
		uchar g=G.at<uchar>((int)pixelL.y,(int)pixelL.x);
		uchar r=R.at<uchar>((int)pixelL.y,(int)pixelL.x);
		cv::circle(res, pixelC, 2,cv::Scalar(b,g,r),2);
		// cv::line(res, pixelL, pixelC,cv::Scalar(b,g,r) , 2);
	}
	return res;
}

void calibHandler::updateTcl(){
	vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> ptsVL;
	vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> ptsVC;
	Eigen::Vector3d weightL=Eigen::Vector3d::Zero();
	Eigen::Vector3d weightC=Eigen::Vector3d::Zero();
	for(auto&p:projPts){
		Eigen::Vector3d ptsL(edgePc->points[p.first].x,edgePc->points[p.first].y,edgePc->points[p.first].z);
		Eigen::Vector3d ptsProj = Tcl_calib.block<3,3>(0,0) * ptsL + Tcl_calib.block<3,1>(0,3);
		Eigen::Vector3d ptsC(correspondence[p.first].x,correspondence[p.first].y,correspondence[p.first].z);
		ptsC=ptsProj.z()*ptsC;
		ptsVL.push_back(ptsL);
		ptsVC.push_back(ptsC);
		weightL+=ptsL;
		weightC+=ptsC;
	}
	weightL=weightL/projPts.size();
	weightC=weightC/projPts.size();
	for(int i=0;i<ptsVL.size();i++){
		ptsVL[i]-=weightL;
		ptsVC[i]-=weightC;
	}
	Eigen::Matrix3d W=Eigen::Matrix3d::Zero();
	for(int i=0;i<ptsVL.size();i++)
		W+=ptsVC[i]*ptsVL[i].transpose();
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
	Eigen::Matrix3d U=svd.matrixU();
	Eigen::Matrix3d V=svd.matrixV();
	Eigen::Matrix3d R=U*(V.transpose());
	if(R.determinant()<0)
		R=-R;
	Eigen::Vector3d t=weightC-R*weightL;
	Tcl_calib.block<3,3>(0,0)=R;
	Tcl_calib.block<3,1>(0,3)=t;
}