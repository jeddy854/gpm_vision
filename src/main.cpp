#include "yolo_server.h"
#include <iostream>

using namespace std;

void drawBoundingBox(cv::Mat image, bbox_t boundingBox);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();

    cv::Mat img_from_camera;
    while(true)
    {         
        std::vector<bbox_t> predict_result;
        if(yolov4->GetImageSubstate()) 
        {
            img_from_camera = yolov4->Get_RGBimage();
            cout<<img_from_camera.rows<<"  "<<img_from_camera.cols<<endl;
            predict_result = yolov4->detector->detect(img_from_camera, thresh = 0.5);
            for (auto p : predict_result) 
            {
                drawBoundingBox(img_from_camera, p);
            }
            cv::imshow("Color Image", img_from_camera);
            cv::waitKey(30);
        }
    }
    return 0;
}
/** Draw a bounding box onto a Mat, include drawing it's class name. */
void drawBoundingBox(cv::Mat image, bbox_t boundingBox) {
    cv::Rect rect(boundingBox.x, boundingBox.y, boundingBox.w, boundingBox.h);
    // Random select a color of bounding box
    int r = 50 + ((43 * (boundingBox.obj_id + 1)) % 150);
    int g = 50 + ((97 * (boundingBox.obj_id + 1)) % 150);
    int b = 50 + ((37 * (boundingBox.obj_id + 1)) % 150);
    cv::Scalar color(b, g, r);
    cv::rectangle(image, rect, color, 2);
    cv::putText(image, to_string(boundingBox.obj_id),
                rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
}