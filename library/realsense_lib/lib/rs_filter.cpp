#include "realsense.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;
using namespace rs2;
using namespace realsense;

void RealSense::enable_align(bool value) {
    enable_align_ = value;
  }

void RealSense::enable_decimation_filter(bool value) {
    enable_decimation_filter_ = value;
  }

void RealSense::enable_hole_filling_filter(bool value) {
    enable_hole_filling_filter_ = value;
  }

void RealSense::enable_spatial_filter(bool value) {
    enable_spatial_filter_ = value;
  }

void RealSense::enable_temporal_filter(bool value) {
    enable_temporal_filter_ = value;
  }

void RealSense::enable_threshold_filter(bool value) {
    enable_threshold_filter_ = value;
  }

void RealSense::enable_units_transform(bool value) {
    enable_units_transform_ = value;
  }

void RealSense::set_align_stream(rs2_stream stream) {
    align_stream_ = stream;
    enable_align_ = true;
    delete align_;
    align_ = new rs2::align(stream);
  }

rs2_stream RealSense::align_stream() {
    return align_stream_;
  }

bool RealSense::align_supports(rs2_option option) {
    return align_->supports(option);
  }

bool RealSense::colorizer_supports(rs2_option option) {
    return colorizer_.supports(option);
  }

bool RealSense::decimation_filter_supports(rs2_option option) {
    return decimation_filter_.supports(option);
  }

bool RealSense::hole_filling_filter_supports(rs2_option option) {
    return hole_filling_filter_.supports(option);
  }

bool RealSense::spatial_filter_supports(rs2_option option) {
    return spatial_filter_.supports(option);
  }

bool RealSense::temporal_filter_supports(rs2_option option) {
    return temporal_filter_.supports(option);
  }

bool RealSense::threshold_filter_supports(rs2_option option) {
    return threshold_filter_.supports(option);
  }

bool RealSense::units_transform_supports(rs2_option option) {
    return units_transform_.supports(option);
  }

std::vector<rs2_option> RealSense::align_supported_options() {
    return align_->get_supported_options();
  }

std::vector<rs2_option> RealSense::colorizer_supported_options() {
    return colorizer_.get_supported_options();
  }

std::vector<rs2_option> RealSense::decimation_filter_supported_options() {
    return decimation_filter_.get_supported_options();
  }

std::vector<rs2_option> RealSense::hole_filling_filter_supported_options() {
    return hole_filling_filter_.get_supported_options();
  }

std::vector<rs2_option> RealSense::spatial_filter_supported_options() {
    return spatial_filter_.get_supported_options();
  }

std::vector<rs2_option> RealSense::temporal_filter_supported_options() {
    return temporal_filter_.get_supported_options();
  }

std::vector<rs2_option> RealSense::threshold_filter_supported_options() {
    return threshold_filter_.get_supported_options();
  }

std::vector<rs2_option> RealSense::units_transform_supported_options() {
    return units_transform_.get_supported_options();
  }

rs2::option_range RealSense::align_option_range(rs2_option option) {
    return align_->get_option_range(option);
  }

rs2::option_range RealSense::colorizer_option_range(rs2_option option) {
    return colorizer_.get_option_range(option);
  }

rs2::option_range RealSense::decimation_filter_option_range(rs2_option option) {
    return decimation_filter_.get_option_range(option);
  }

rs2::option_range RealSense::hole_filling_filter_option_range(rs2_option
    option) {
        return hole_filling_filter_.get_option_range(option);
      }

rs2::option_range RealSense::spatial_filter_option_range(rs2_option option) {
    return spatial_filter_.get_option_range(option);
  }

rs2::option_range RealSense::temporal_filter_option_range(rs2_option option) {
    return temporal_filter_.get_option_range(option);
  }

rs2::option_range RealSense::threshold_filter_option_range(rs2_option option) {
    return threshold_filter_.get_option_range(option);
  }

rs2::option_range RealSense::units_transform_option_range(rs2_option option) {
    return units_transform_.get_option_range(option);
  }

float RealSense::align_option(rs2_option option) {
    return align_->get_option(option);
  }

float RealSense::colorizer_option(rs2_option option) {
    return colorizer_.get_option(option);
  }

float RealSense::decimation_filter_option(rs2_option option) {
    return decimation_filter_.get_option(option);
  }

float RealSense::hole_filling_filter_option(rs2_option option) {
    return hole_filling_filter_.get_option(option);
  }

float RealSense::spatial_filter_option(rs2_option option) {
    return spatial_filter_.get_option(option);
  }

float RealSense::temporal_filter_option(rs2_option option) {
    return temporal_filter_.get_option(option);
  }

float RealSense::threshold_filter_option(rs2_option option) {
    return threshold_filter_.get_option(option);
  }

float RealSense::units_transform_option(rs2_option option) {
    return units_transform_.get_option(option);
  }

void RealSense::set_align_option(rs2_option option, float value) {
    align_->set_option(option, value);
    enable_align_ = true;
  }

void RealSense::set_colorizer_option(rs2_option option, float value) {
    colorizer_.set_option(option, value);
  }

void RealSense::set_decimation_filter_option(rs2_option option, float value) {
    decimation_filter_.set_option(option, value);
    enable_decimation_filter_ = true;
  }

void RealSense::set_hole_filling_filter_option(rs2_option option, float value) {
    hole_filling_filter_.set_option(option, value);
    enable_hole_filling_filter_ = true;
  }

void RealSense::set_spatial_filter_option(rs2_option option, float value) {
    spatial_filter_.set_option(option, value);
    enable_spatial_filter_ = true;
  }

void RealSense::set_temporal_filter_option(rs2_option option, float value) {
    temporal_filter_.set_option(option, value);
    enable_temporal_filter_ = true;
  }

void RealSense::set_threshold_filter_option(rs2_option option, float value) {
    threshold_filter_.set_option(option, value);
    enable_threshold_filter_ = true;
  }

void RealSense::set_units_transform_option(rs2_option option, float value) {
    units_transform_.set_option(option, value);
    enable_units_transform_ = true;
  }

