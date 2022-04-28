#include "yolo_server.h"
using namespace cv;
using namespace std;
Yolo *Yolo::inst = nullptr;
Yolo *Yolo::GetYolo()
{
    if (inst == nullptr)
        inst = new Yolo();
    return inst;
}
Yolo::Yolo() 
{
    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 30);
    p_profile = pipe.start(cfg); 
    // detector = new Detector("/home/gpm-server/gpm/gpm_jssp/setting/yolov4-tiny.cfg", "/home/gpm-server/gpm/gpm_jssp/model/yolov4-tiny_last.weights");
    // detector = new Detector("/home/vision1/gpm/gpm_jssp/setting/yolov4-tiny.cfg", "/home/vision1//gpm/gpm_jssp/model/yolov4-tiny_last_04_28.weights");
    detector = new Detector("/home/vision2/gpm/gpm_jssp/setting/yolov4-tiny.cfg", "/home/vision2//gpm/gpm_jssp/model/yolov4-tiny_last_04_28.weights");
}

Yolo::~Yolo()
{
    inst = nullptr;
}

void Yolo::Realsense_stream_update(void)
{
    rs2::frameset frames;
    rs2::align align_to_color(RS2_STREAM_COLOR);
    align_intrinsics = p_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    rs2::spatial_filter spat;
    spat.set_option(RS2_OPTION_HOLES_FILL, 3);
    // rs2::temporal_filter temp;
    rs2::hole_filling_filter hole_filling_filter;
    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 3.f);
    frames = pipe.wait_for_frames();
    // Align and get the frame
    frames = spat.process(frames);
    // frames = temp.process(frames);
    frames = hole_filling_filter.process(frames);
    // auto aligned_frames = align_to_color.process(frames);
    // color_frame_ = aligned_frames.get_color_frame();
    // depth_frame_ = aligned_frames.get_depth_frame();
    color_frame_ = frames.get_color_frame();
    depth_frame_ = frames.get_depth_frame();
    // Transform the realsense data to cv mat   
    cv::Mat(cv::Size(w, h), CV_8UC3, (void *)color_frame_.get_data(), cv::Mat::AUTO_STEP).copyTo(img_from_camera);
    cv::Mat(cv::Size(w, h), CV_8UC1, (void *)depth_frame_.get_data(), cv::Mat::AUTO_STEP).copyTo(depth_from_camera);
    rs2::frame color_aligned_depth_frame = color_map.colorize(depth_frame_);
    cv::Mat(cv::Size(w, h), CV_8UC3, (void *)color_aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP).copyTo(depth_rgb_from_camera);
}

cv::Mat Yolo::Get_RGBimage(void)
{
    return this->img_from_camera;
}

cv::Mat Yolo::Get_Depthrgbimage(void)
{
    return this->depth_rgb_from_camera;
}

void Yolo::Get_Depth(float x, float y, float& depth)
{
    depth = depth_frame_.as<rs2::depth_frame>().get_distance(x, y);
}

void Yolo::Get_CameraIntrin(rs2_intrinsics& align_intrinsics)
{
    align_intrinsics = this->align_intrinsics;
}