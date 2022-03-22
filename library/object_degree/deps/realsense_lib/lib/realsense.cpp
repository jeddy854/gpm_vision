#include "realsense.h"
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;
using namespace rs2;
using namespace realsense;

StreamConfig::StreamConfig() {
    width = 0;
    height = 0;
    format = RS2_FORMAT_ANY;
    framerate = 0;
}

Config::Config() {
    device_type = DeviceType::CAMERA_ID;
    camera_id = 0;
}

RealSense::RealSense() {
    fps_ = 0;

    // device
    dev_list_ = ctx_.query_devices();
    device_number_ = dev_list_.size();

    // filter
    align_stream_ = RS2_STREAM_COLOR;
    align_ = new rs2::align(RS2_STREAM_COLOR);
    depth_to_disparity_ = new disparity_transform(true);
    disparity_to_depth_ = new disparity_transform(false);
    enable_align_ = 0;
    enable_decimation_filter_ = 0;
    enable_hole_filling_filter_ = 0;
    enable_spatial_filter_ = 0;
    enable_temporal_filter_ = 0;
    enable_threshold_filter_ = 0;
    enable_units_transform_ = 0;
}

RealSense::RealSense(Config cfg) : RealSense() { connect(cfg); }

RealSense::~RealSense() {
    // filter
    delete align_;
    delete depth_to_disparity_;
    delete disparity_to_depth_;
}

void RealSense::connect(Config cfg) {
    // setting connection configuration
    if (cfg.device_type == DeviceType::CAMERA_ID) {
        dev_ = dev_list_[cfg.camera_id];
        cfg_.enable_device(dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    } else {
        cfg_.enable_device(cfg.serial_id);
    }

    if (cfg.stream_configs.size() == 0) {
        cfg_.enable_all_streams();
    } else {
        for (StreamConfig& s : cfg.stream_configs) {
            cfg_.enable_stream(s.stream, s.width, s.height, s.format,
                               s.framerate);
        }
    }
    //    this->dev_.hardware_reset();

    pipe_ = pipeline(ctx_);
    pipe_profile_ = pipe_.start(cfg_);

    // show information
    dev_ = pipe_profile_.get_device();
    cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " ";
    cout << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;

    // find color sensor
    vector<sensor> sensors = dev_.query_sensors();
    for (sensor& s : sensors) {
        string sensor_info = s.get_info(RS2_CAMERA_INFO_NAME);
        if (sensor_info == "RGB Camera") {
            color_sensor_ = s;
        } else if (sensor_info == "L500 Depth Sensor" ||
                   sensor_info == "Stereo Module") {
            depth_sensor_ = s;
        } else if (sensor_info == "Motion Module") {
            motion_sensor_ = s;
        }
    }
}

void RealSense::disconnect() { pipe_.stop(); }

void RealSense::update() {
    // time count
    auto start = chrono::high_resolution_clock::now();

    pipe_profile_ = pipe_.get_active_profile();

    // wait for next set of frames from the camera
    frames_ = pipe_.wait_for_frames();

    // filter
    if (enable_decimation_filter_)
        frames_ = decimation_filter_.process(frames_);
    if (enable_threshold_filter_) frames_ = threshold_filter_.process(frames_);
    if (enable_spatial_filter_ || enable_temporal_filter_)
        frames_ = depth_to_disparity_->process(frames_);
    if (enable_spatial_filter_) frames_ = spatial_filter_.process(frames_);
    if (enable_temporal_filter_) frames_ = temporal_filter_.process(frames_);
    if (enable_spatial_filter_ || enable_temporal_filter_)
        frames_ = disparity_to_depth_->process(frames_);
    if (enable_hole_filling_filter_)
        frames_ = hole_filling_filter_.process(frames_);
    if (enable_units_transform_) frames_ = units_transform_.process(frames_);
    if (enable_align_) frames_ = align_->process(frames_);

    // get each frame
    color_frame_ = frames_.get_color_frame();
    depth_frame_ = frames_.get_depth_frame();
    infrared_frame_ = frames_.get_infrared_frame();
    gyro_frame_ = frames_.first_or_default(RS2_STREAM_GYRO);
    accel_frame_ = frames_.first_or_default(RS2_STREAM_ACCEL);

    // intrinsics
    depth_intrinsics_ = depth_frame_.get_profile()
                            .as<rs2::video_stream_profile>()
                            .get_intrinsics();

    // time count
    auto end = chrono::high_resolution_clock::now();
    auto duration = end - start;
    double time = chrono::duration_cast<chrono::milliseconds>(duration).count();
    fps_ = (int)(1000.0 / time);
}

int RealSense::fps() { return fps_; }

rs2_format RealSense::format(rs2_stream stream) {
    stream_profile_ = pipe_profile_.get_stream(stream);
    return stream_profile_.format();
}

int RealSense::fps(rs2_stream stream) {
    stream_profile_ = pipe_profile_.get_stream(stream);
    return stream_profile_.fps();
}
