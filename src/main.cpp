#include "yolo_server.h"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]){
    std::cerr<<"in while"<<std::endl;
	
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    cv::Mat colorimage;
    while(true)
    {
	std::cout<<"in while"<<std::endl;
        colorimage = yolov4->Get_RGBimage();
        cout<<colorimage.rows<<"   "<<colorimage.cols<<endl;
	cv::imshow("Color Image", colorimage);
        waitKey(30);
    }
    return 0;
}
