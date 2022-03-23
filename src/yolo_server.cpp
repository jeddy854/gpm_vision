#include "yolo_server.h"
#include <iostream>

Yolo *Yolo::inst = nullptr;
Yolo *Yolo::GetYolo()
{
    if (inst == nullptr)
        inst = new Yolo();
    return inst;
}
Yolo::Yolo() : it_(ni_)
{
    InitialRos();
}

Yolo::~Yolo()
{
    inst = nullptr;
    ros_thread->join();
    delete ros_thread;
}

void Yolo::InitialRos()
{
    this->image_sub = this->it_.subscribe("/camera/color/image_raw", 10, &Yolo::IntelD435i_ImageCb, this);
    this->depth_sub = this->it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Yolo::IntelD435i_DepthCb, this);
    this->calib_sub = this->n_.subscribe("/camera/color/camera_info", 1, &Yolo::IntelD435i_CalibCb, this);
    ros_thread = new std::thread(&Yolo::Ros_spin,this);
}

void Yolo::Ros_spin()
{
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(int(100)));
}

void Yolo::IntelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        this->img_from_camera = cv_ptr->image.clone();
	std::cout<< this->img_from_camera.rows<<std::endl;
	
    }
    catch (cv_bridge::Exception &e)
    {
	std::cout<<"error"<<std::endl;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void Yolo::IntelD435i_DepthCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        this->depth_from_camera = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void Yolo::IntelD435i_CalibCb(const sensor_msgs::CameraInfo &msg)
{
    try
    {
        this->fx = msg.K[0];
        this->fy = msg.K[4];
        this->cx = msg.K[2];
        this->cy = msg.K[5];
        this->k1 = msg.D[0];
        this->k2 = msg.D[1];
        this->p1 = msg.D[2];
        this->p2 = msg.D[3];
        this->k3 = msg.D[4];
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Cameral Calibration Parameter exception: %s", e.what());
        return;
    }
}

cv::Mat Yolo::Get_RGBimage()
{
    return this->img_from_camera;
}
