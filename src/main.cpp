#include "yolo_server.h"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]){
    std::cerr<<"in while"<<std::endl;
	
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    cv::Mat img_from_camera;
    while(true)
    {         
        if(yolov4->GetImageSubstate()) 
        {
            img_from_camera = yolov4->Get_RGBimage();
            cout<<img_from_camera.rows<<"  "<<img_from_camera.cols<<endl;
            cv::imshow("Color Image", img_from_camera);
            waitKey(30);
        }
    }
    return 0;
}
