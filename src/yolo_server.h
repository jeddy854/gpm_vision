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
#define OPENCV
#include "yolo_v2_class.hpp"
#include <iostream>
struct Intrinsic_Matrix
{   
    float fx;
    float fy;
    float cx;
    float cy;
};
class Yolo
{
    public:
        static Yolo *GetYolo(void);
        ~Yolo(void);
        cv::Mat Get_RGBimage(void);
        cv::Mat Get_Depthimage(void);
        void Get_CameraIntrin(Intrinsic_Matrix& intrin);
        bool GetAllDataSubstate(void); 
        Detector *detector;
    /* Get realsense d435i rgb and depth image*/
    /* ROS */
    private:
        ros::NodeHandle ni_; // for image_transport
        image_transport::ImageTransport it_;
        ros::NodeHandle n_;
        void InitialRos(void);
        void Ros_spin(void);
        image_transport::Subscriber image_sub;
        image_transport::Subscriber depth_sub;
        ros::Subscriber calib_sub;
        bool image_sub_flag;
        bool depth_sub_flag;
        bool calib_sub_flag;
        void IntelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg);
        void IntelD435i_DepthCb(const sensor_msgs::ImageConstPtr &msg);
        void IntelD435i_CalibCb(const sensor_msgs::CameraInfo &msg);
        std::thread *ros_thread;
    
        static Yolo *inst;
        Yolo(void);
        Intrinsic_Matrix intrin;
        cv::Mat img_from_camera;
        cv::Mat depth_from_camera;
}; 
#endif
