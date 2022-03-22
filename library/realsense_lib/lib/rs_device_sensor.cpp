#include "realsense.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;
using namespace rs2;
using namespace realsense;

int RealSense::device_numbers() {
    return device_number_;
  }

std::string RealSense::device_name() {
    return string(dev_.get_info(RS2_CAMERA_INFO_NAME));
  }

std::string RealSense::device_serial_id() {
    return string(dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  }

bool RealSense::color_supports(rs2_option option) {
    return color_sensor_.supports(option);
  }

bool RealSense::depth_supports(rs2_option option) {
    return depth_sensor_.supports(option);
  }

bool RealSense::motion_supports(rs2_option option) {
    return motion_sensor_.supports(option);
  }

std::vector<rs2_option> RealSense::color_supported_options() {
    return color_sensor_.get_supported_options();
  }

std::vector<rs2_option> RealSense::depth_supported_options() {
    return depth_sensor_.get_supported_options();
  }

std::vector<rs2_option> RealSense::motion_supported_options() {
    return motion_sensor_.get_supported_options();
  }

option_range RealSense::color_option_range(rs2_option option) {
    return color_sensor_.get_option_range(option);
  }

option_range RealSense::depth_option_range(rs2_option option) {
    return depth_sensor_.get_option_range(option);
  }

option_range RealSense::motion_option_range(rs2_option option) {
    return motion_sensor_.get_option_range(option);
  }

float RealSense::color_option(rs2_option option) {
    return color_sensor_.get_option(option);
  }

float RealSense::depth_option(rs2_option option) {
    return depth_sensor_.get_option(option);
  }

float RealSense::motion_option(rs2_option option) {
    return motion_sensor_.get_option(option);
  }

void RealSense::set_color_option(rs2_option option, float value) {
    color_sensor_.set_option(option, value);
  }

void RealSense::set_depth_option(rs2_option option, float value) {
    depth_sensor_.set_option(option, value);
  }

void RealSense::set_motion_option(rs2_option option, float value) {
    motion_sensor_.set_option(option, value);
  }
