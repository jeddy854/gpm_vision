#include "yolo_server.h"

using namespace cv;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    cv::Mat colorimage;
    while(true)
    {
        colorimage = yolov4->Get_RGBimage();
        cv::imshow("Color Image", colorimage);
        wiatKey(30);
    }
    return 0;
}
