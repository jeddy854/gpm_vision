#ifndef YOLO_SERVER_H
#define YOLO_SERVER_H
#include <thread>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#define OPENCV
#include "yolo_v2_class.hpp"
#include <iostream>
#include <ros/ros.h>
class Yolo
{
    public:
        static Yolo *GetYolo(void);
        ~Yolo(void);
        cv::Mat Get_RGBimage(void);
        cv::Mat Get_Depthrgbimage(void);
        void Get_Depth(float x, float y, float& depth);
        void Get_CameraIntrin(rs2_intrinsics& align_intrinsics);
        Detector *detector;
    /* Get realsense d435i rgb and depth image*/
    /* realsense sdk */
        void Realsense_stream_update(void);
    
    private:
        static Yolo *inst;
        Yolo(void);
        cv::Mat img_from_camera;
        cv::Mat depth_from_camera;
        cv::Mat depth_rgb_from_camera;
        //realsense sdk
        rs2::pipeline pipe;
        rs2::pipeline_profile p_profile;
        rs2::config cfg;
        rs2_intrinsics align_intrinsics;
        rs2::frame color_frame_;
        rs2::frame depth_frame_;
        const int w = 640;
        const int h = 480;
}; 
#endif