#ifndef YOLO_SERVER
#define YOLO_SERVER
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// #include "yolo_v2_class.hpp"

class Yolo
{
    public:
        static Yolo *GetYolo();
        ~Yolo(){ inst_ = nullptr; };
        cv::Mat Get_RGBimage();

    /* Get realsense d435i rgb and depth image*/
    /* ROS */
    private:
        ros::NodeHandel n;
        void InitialRos(void);
        void Ros_spin(void);
        image_transport::Subscriber image_sub;
        image_transport::Subscriber depth_sub;
        ros::Subscriber calib_sub;
        void IntelD435i_DepthCb(const sensor_msgs::ImageConstPtr &msg);
        void IntelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg);
        void IntelD435i_CalibCb(const sensor_msgs::CameraInfo &msg);
        thread *ros_thread;
    
    private:
        static Yolo *inst_;
        yolo();
        cv::Mat img_from_camera;
        cv::Mat depth_from_camera;
        float fx;
        float fy;
        float cx;
        float cy;
        float k1;
        float k2;
        float k3;
        float p1;
        float p2;
}   
#endif