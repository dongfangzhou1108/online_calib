#include"calibHandler.h"
int main(){
	string imgPath = "./data/0000000000.png";
	string binPath = "./data/0000000000.bin";
	string configPath="./config/config.yaml";

	imgHandler imgH(imgPath,configPath);
	pcHandler pcH(binPath,configPath);
	calibHandler calibH(configPath,&imgH,&pcH);
	cv::Mat test=calibH.projPC2Img(pcH.pcEdge,imgH.edgeGray,calibH.Tcl_calib);
    cv::imshow("test", test);
    cv::waitKey();
    cv::destroyAllWindows();

	return 0;
}