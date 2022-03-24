#include "yolo_server.h"
cv::Mat img_from_camera;
cv::Mat depth_from_camera;
Intrinsic_Matrix intrin;
void drawBoundingBox(cv::Mat& image, bbox_t boundingBox);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    while(true)
    {         
        std::vector<bbox_t> predict_result;
        if(yolov4->GetAllDataSubstate()) 
        {
            img_from_camera = yolov4->Get_RGBimage();
            depth_from_camera = yolov4->Get_Depthimage();
            yolov4->Get_CameraIntrin(intrin);
            predict_result = yolov4->detector->detect(img_from_camera, 0.3);

            cout<<predict_result.size()<<endl;
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
void drawBoundingBox(cv::Mat& image, bbox_t boundingBox) {
    cv::Rect rect(boundingBox.x, boundingBox.y, boundingBox.w, boundingBox.h);
    // Random select a color of bounding box
    int r = 50 + ((43 * (boundingBox.obj_id + 1)) % 150);
    int g = 50 + ((97 * (boundingBox.obj_id + 1)) % 150);
    int b = 50 + ((37 * (boundingBox.obj_id + 1)) % 150);
    cv::Scalar color(b, g, r);
    cv::rectangle(image, rect, color, 2);
    float depth = depth_from_camera.at<float>(boundingBox.y+boundingBox.h/2,boundingBox.x+boundingBox.w/2);
    if(depth>0)
    {
        boundingBox.z_3d = depth;
        boundingBox.x_3d = ((boundingBox.x+boundingBox.w/2-intrin.cx)/intrin.fx)*depth;
        boundingBox.y_3d = ((boundingBox.y+boundingBox.h/2-intrin.cy)/intrin.fy)*depth;
    }
    else 
    {
        boundingBox.z_3d = 0;
        boundingBox.x_3d = 0;
        boundingBox.y_3d = 0;
    }
    cv::putText(image, "( "+to_string(boundingBox.x_3d)+" "+to_string(boundingBox.y_3d)+" "+to_string(boundingBox.z_3d)+" )",
                rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
}