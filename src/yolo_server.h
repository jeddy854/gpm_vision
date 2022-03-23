#ifndef YOLO_SERVER
#define YOLO_SERVER
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <vector>
#include "yolo_v2_class.hpp"
#include <iostream>

using namespace std;

class Yolo
{
    public:
        static Yolo *GetYolo(void);
        ~Yolo(void);
        cv::Mat Get_RGBimage(void);
        bool GetImageSubstate(void);
        Detector *detector;
    /* Get realsense d435i rgb and depth image*/
    /* ROS */
    private:
        ros::NodeHandle ni_;
        image_transport::ImageTransport it_;
        ros::NodeHandle n_;
        void InitialRos(void);
        void Ros_spin(void);
        image_transport::Subscriber image_sub;
        bool image_sub_flag;
        ros::Subscriber calib_sub;
        void IntelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg);
        void IntelD435i_CalibCb(const sensor_msgs::CameraInfo &msg);
        std::thread *ros_thread;
    
    private:
        static Yolo *inst;
        Yolo(void);
        float fx;
        float fy;
        float cx;
        float cy;
        float k1;
        float k2;
        float k3;
        float p1;
        float p2;
        cv::Mat img_from_camera;
        cv::Mat depth_from_camera;
}; 
#endif