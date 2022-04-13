#include "yolo_server.h"
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/ServerSocket.h>
#include <string>
cv::Mat img_from_camera;
cv::Mat depth_from_camera;
Intrinsic_Matrix intrin;
// process::Yolo_result result;
void drawBoundingBox(cv::Mat& image, bbox_t& boundingBox);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo1");
    Yolo *yolov4;
    yolov4 = Yolo::GetYolo();
    Poco::Net::ServerSocket srv(3000);
    while(true)
    {         
        std::vector<bbox_t> predict_result;
        if(yolov4->GetAllDataSubstate()) 
        {
            cout<<"Get Ready."<<endl;
            Poco::Net::StreamSocket socket = srv.acceptConnection();
            Poco::Net::SocketStream stream(socket);
            cout<<"Accept Connection."<<endl;
            img_from_camera = yolov4->Get_RGBimage();
            depth_from_camera = yolov4->Get_Depthimage();
            yolov4->Get_CameraIntrin(intrin);
            predict_result = yolov4->detector->detect(img_from_camera, 0.3);
            cout<<predict_result.size()<<endl;
            if(predict_result.size()<1) continue;
            for (auto p : predict_result) 
            {   
                drawBoundingBox(img_from_camera, p);
                stream << to_string(p.obj_id)+" "+to_string(p.x_3d)+" "+to_string(p.y_3d)+" "+to_string(p.z_3d)+" ";
                cout << p.obj_id<<" "<<p.x_3d<<" "<<p.y_3d<<" "<<p.z_3d<<endl;
            }
            stream.flush();
            socket.shutdownSend();
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
        boundingBox.z_3d = depth-4.1;
        boundingBox.x_3d = ((boundingBox.x+boundingBox.w/2-intrin.cx)/intrin.fx)*depth-32.5;
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