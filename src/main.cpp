#include "yolo_server.h"

int main(){
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    cv::Mat colorimage;
    colorimage = yolov4->Get_RGBimage();
    cv::imshow("Color Image", colorimage)
    return 0;
}
