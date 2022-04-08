#include "yolo_server.h"
cv::Mat img_from_camera;
cv::Mat depth_from_camera;
Intrinsic_Matrix intrin;
process::Yolo_result result;
void drawBoundingBox(cv::Mat& image, bbox_t& boundingBox);

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
            if(predict_result.size()<1) continue;
            for (auto p : predict_result) 
            {   
                result.x.push_back(p.x);
                result.y.push_back(p.y);
                result.w.push_back(p.w);
                result.h.push_back(p.h);
                result.prob.push_back(p.prob);
                result.obj_id.push_back(p.obj_id);
                drawBoundingBox(img_from_camera, p);
            }
            yolov4->Result_Pub(result);
            result.x.clear();
            result.y.clear();
            result.w.clear();
            result.h.clear();
            result.prob.clear();
            result.obj_id.clear();
            #ifdef DRAWING
            cv::imshow("Color Image", img_from_camera);
            cv::waitKey(30);
            #endif
        }
    }
    return 0;
}
/** Draw a bounding box onto a Mat, include drawing it's class name. */
void drawBoundingBox(cv::Mat& image, bbox_t& boundingBox) {
    #ifdef DRAWING
    cv::Rect rect(boundingBox.x, boundingBox.y, boundingBox.w, boundingBox.h);
    // Random select a color of bounding box
    int r = 50 + ((43 * (boundingBox.obj_id + 1)) % 150);
    int g = 50 + ((97 * (boundingBox.obj_id + 1)) % 150);
    int b = 50 + ((37 * (boundingBox.obj_id + 1)) % 150);
    cv::Scalar color(b, g, r);
    cv::rectangle(image, rect, color, 2);
    #endif
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
    #ifdef DRAWING
    cv::putText(image, "( "+to_string(boundingBox.x_3d)+" "+to_string(boundingBox.y_3d)+" "+to_string(boundingBox.z_3d)+" )",
                rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
    #endif
}