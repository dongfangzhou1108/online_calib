#include"calibHandler.h"
int main(){
	string imgPath = "./data/0000000000.png";
	string binPath = "./data/0000000000.bin";
	string configPath="./config/config.yaml";

	imgHandler imgH(imgPath,configPath);
	pcHandler pcH(binPath,configPath);
	calibHandler calibH(configPath,&imgH,&pcH);
	while(1){
		cv::Mat proj=calibH.projPC2Img(pcH.pcEdge,imgH.edgeGray,calibH.Tcl_calib);
		calibH.findCorrespondence();
		cv::Mat corres=calibH.drawCorrespondence(proj);
		cv::imshow("proj", proj);
		cv::waitKey();
		cv::destroyAllWindows();
		Eigen::Vector3d eulerAngle=calibH.Tcl_real.block<3,3>(0,0).eulerAngles(2,1,0);//zyx:roll+pitch+yaw
		Eigen::Vector3d tcl=calibH.Tcl_real.block<3,1>(0,3);
		calibH.updateTcl();
		Eigen::Vector3d eulerAngleCalib=calibH.Tcl_calib.block<3,3>(0,0).eulerAngles(2,1,0);//zyx:roll+pitch+yaw
		Eigen::Vector3d tclCalib=calibH.Tcl_calib.block<3,1>(0,3);
		Eigen::Vector3d deltaRPY=(calibH.Tcl_real.block<3,3>(0,0).transpose()*calibH.Tcl_calib.block<3,3>(0,0)).eulerAngles(2,1,0);
		cout << calibH.correspondence.size() << " deltaRPY:" << deltaRPY.transpose() << " delta_t: " << (tcl-tclCalib).transpose() << endl;
	}

	return 0;
}