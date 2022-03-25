#include "yolo_server.h"

Yolo *Yolo::inst = nullptr;
Yolo *Yolo::GetYolo()
{
    if (inst == nullptr)
        inst = new Yolo();
    return inst;
}
Yolo::Yolo() 
: it_(ni_),
  image_sub_flag(false),
  depth_sub_flag(false),
  calib_sub_flag(false)
{
    detector = new Detector("/home/vision1/model/yolo_v4/yolov4-tiny.cfg", "/home/vision1/model/yolo_v4/yolov4-tiny.weights");
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
    this->image_sub = this->it_.subscribe("/camera/color/image_raw", 1, &Yolo::IntelD435i_ImageCb, this);
    this->depth_sub = this->it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Yolo::IntelD435i_DepthCb, this);
    this->calib_sub = this->n_.subscribe("/camera/color/camera_info", 1, &Yolo::IntelD435i_CalibCb, this);
    ros_thread = new std::thread(&Yolo::Ros_spin,this);
}

void Yolo::Ros_spin()
{   
    while(true)
    {   
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(int(100)));
    }
}

void Yolo::IntelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(this->img_from_camera);
        if(this->img_from_camera.rows > 0) image_sub_flag = true;
        else image_sub_flag = false;
    }
    catch (cv_bridge::Exception &e)
    {
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
        if(this->img_from_camera.rows > 0) depth_sub_flag = true;
        else depth_sub_flag = false;
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
        this->intrin.fx = msg.K[0];
        this->intrin.fy = msg.K[4];
        this->intrin.cx = msg.K[2];
        this->intrin.cy = msg.K[5];
        if(intrin.fx>0 && intrin.fy>0 && intrin.cx>0 && intrin.cy>0) calib_sub_flag = true;
        else calib_sub_flag = false;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Cameral Calibration Parameter exception: %s", e.what());
        return;
    }
}

cv::Mat Yolo::Get_RGBimage(void)
{
    return this->img_from_camera;
}

cv::Mat Yolo::Get_Depthimage(void)
{
    return this->depth_from_camera;
}

void Yolo::Get_CameraIntrin(Intrinsic_Matrix& intrin)
{
    intrin = this->intrin;
}

bool Yolo::GetAllDataSubstate(void)
{
    return this->image_sub_flag * this->depth_sub_flag * this->calib_sub_flag;
}
