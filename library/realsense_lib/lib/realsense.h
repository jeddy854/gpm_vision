#ifndef __REALSENSE_H__
#define __REALSENSE_H__

#include <librealsense2/rsutil.h>
#include <omp.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace realsense {
/*
 * 串流皆為預設，使用此資料型態來增加額外設定
 *
 * preset:
 *  width: 0
 *  height: 0
 *  format: auto
 *  framerate: auto
 *
 * discription:
 *  width: stream width
 *  height: stream height
 */
typedef struct StreamConfig {
    StreamConfig();
    rs2_stream stream;
    int width;
    int height;
    rs2_format format;
    int framerate;
} StreamConfig;

/*
 * 裝置開啟型態
 */
typedef enum class DeviceType { CAMERA_ID, SERIAL_ID } DeviceType;

/*
 * 裝置開啟設定
 *
 * preset:
 *  device_type: CAMERA_ID
 *  camera_id: 0
 *
 * discription:
 *  stream_configs: 預設全部啟用，增加成員進行指定啟用及設定
 */
typedef struct Config {
    Config();
    DeviceType device_type;
    int camera_id;
    std::string serial_id;
    std::vector<StreamConfig> stream_configs;
} Config;

/*
 * retrieve_image type
 */
typedef enum class ImageType {
    COLOR,    // CV8UC3
    DEPTH,    // CV8UC3
    INFRARED  // CV8U
} ImageType;

/*
 * retrieve_measure type
 */
typedef enum class MeasureType {
    X,      // CV32F
    Y,      // CV32F
    DEPTH,  // CV32F
    XYZ,    // CV32FC3
    GYRO,   // vector<double>(3)
    ACCEL   // vector<double>(3)
} MeasureType;

class RealSense {
   public:
    // construct
    RealSense();
    // construct and connect
    RealSense(Config cfg);
    ~RealSense();

    // connect
    void connect(Config cfg);
    void disconnect();

    // update
    void update();
    // fps of update
    int fps();

    // stream
    // format of stream
    rs2_format format(rs2_stream stream);
    // fps of stream
    int fps(rs2_stream stream);

    // frame
    void retrieve_color_image(cv::Mat &mat);
    void retrieve_depth_image(cv::Mat &mat);
    void retrieve_infrared_image(cv::Mat &mat);
    void retrieve_image(cv::Mat &mat, ImageType type);

    void retrieve_x_measure(cv::Mat &mat);
    void retrieve_y_measure(cv::Mat &mat);
    void retrieve_depth_measure(float &value, cv::Point point);
    // this function is based on above, please use less according to performance
    void retrieve_depth_measure(cv::Mat &mat);
    void retrieve_xyz_measure(std::vector<float> &values, cv::Point point);
    // this function is based on above, please use less according to performance
    void retrieve_xyz_measure(cv::Mat &mat);
    void retrieve_gyro_measure(std::vector<double> &vec);
    void retrieve_accel_measure(std::vector<double> &vec);
    // used for x, y, depth, xyz
    void retrieve_measure(cv::Mat &mat, MeasureType type);
    // used for gyro, accel
    void retrieve_measure(std::vector<double> &vec, MeasureType type);

    // device
    // current connecting devices on computer
    int device_numbers();
    // device name
    std::string device_name();
    // serial id of device
    std::string device_serial_id();

    // sensor
    bool color_supports(rs2_option option);
    bool depth_supports(rs2_option option);
    bool motion_supports(rs2_option option);

    std::vector<rs2_option> color_supported_options();
    std::vector<rs2_option> depth_supported_options();
    std::vector<rs2_option> motion_supported_options();

    rs2::option_range color_option_range(rs2_option option);
    rs2::option_range depth_option_range(rs2_option option);
    rs2::option_range motion_option_range(rs2_option option);

    float color_option(rs2_option option);
    float depth_option(rs2_option option);
    float motion_option(rs2_option option);

    void set_color_option(rs2_option option, float value);
    void set_depth_option(rs2_option option, float value);
    void set_motion_option(rs2_option option, float value);

    // filter
    void set_align_stream(rs2_stream stream);
    rs2_stream align_stream();
    void enable_align(bool value);
    void enable_decimation_filter(bool value);
    void enable_hole_filling_filter(bool value);
    void enable_spatial_filter(bool value);
    void enable_temporal_filter(bool value);
    void enable_threshold_filter(bool value);
    void enable_units_transform(bool value);

    bool align_supports(rs2_option option);
    bool colorizer_supports(rs2_option option);
    bool decimation_filter_supports(rs2_option option);
    bool hole_filling_filter_supports(rs2_option option);
    bool spatial_filter_supports(rs2_option option);
    bool temporal_filter_supports(rs2_option option);
    bool threshold_filter_supports(rs2_option option);
    bool units_transform_supports(rs2_option option);

    std::vector<rs2_option> align_supported_options();
    std::vector<rs2_option> colorizer_supported_options();
    std::vector<rs2_option> decimation_filter_supported_options();
    std::vector<rs2_option> hole_filling_filter_supported_options();
    std::vector<rs2_option> spatial_filter_supported_options();
    std::vector<rs2_option> temporal_filter_supported_options();
    std::vector<rs2_option> threshold_filter_supported_options();
    std::vector<rs2_option> units_transform_supported_options();

    rs2::option_range align_option_range(rs2_option option);
    rs2::option_range colorizer_option_range(rs2_option option);
    rs2::option_range decimation_filter_option_range(rs2_option option);
    rs2::option_range hole_filling_filter_option_range(rs2_option option);
    rs2::option_range spatial_filter_option_range(rs2_option option);
    rs2::option_range temporal_filter_option_range(rs2_option option);
    rs2::option_range threshold_filter_option_range(rs2_option option);
    rs2::option_range units_transform_option_range(rs2_option option);

    float align_option(rs2_option option);
    float colorizer_option(rs2_option option);
    float decimation_filter_option(rs2_option option);
    float hole_filling_filter_option(rs2_option option);
    float spatial_filter_option(rs2_option option);
    float temporal_filter_option(rs2_option option);
    float threshold_filter_option(rs2_option option);
    float units_transform_option(rs2_option option);

    void set_align_option(rs2_option option, float value);
    void set_colorizer_option(rs2_option option, float value);
    void set_decimation_filter_option(rs2_option option, float value);
    void set_hole_filling_filter_option(rs2_option option, float value);
    void set_spatial_filter_option(rs2_option option, float value);
    void set_temporal_filter_option(rs2_option option, float value);
    void set_threshold_filter_option(rs2_option option, float value);
    void set_units_transform_option(rs2_option option, float value);

   private:
    rs2::config cfg_;
    rs2::context ctx_;
    rs2::device dev_;
    rs2::device_list dev_list_;
    rs2::pipeline pipe_;
    rs2::pipeline_profile pipe_profile_;
    rs2::stream_profile stream_profile_;
    std::vector<rs2::stream_profile> stream_profiles_;
    rs2_intrinsics depth_intrinsics_;

    int fps_;

    // device
    int device_number_;

    // sensor
    rs2::sensor color_sensor_;
    rs2::sensor depth_sensor_;
    rs2::sensor motion_sensor_;

    // frame
    rs2::frameset frames_;
    rs2::frame color_frame_;
    rs2::frame depth_frame_;
    rs2::frame infrared_frame_;
    rs2::frame gyro_frame_;
    rs2::frame accel_frame_;

    // filter
    rs2::align *align_;
    rs2::colorizer colorizer_;
    rs2::decimation_filter decimation_filter_;
    rs2::disparity_transform *depth_to_disparity_;
    rs2::disparity_transform *disparity_to_depth_;
    rs2::hole_filling_filter hole_filling_filter_;
    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    rs2::threshold_filter threshold_filter_;
    rs2::units_transform units_transform_;

    rs2_stream align_stream_;
    bool enable_align_;
    bool enable_decimation_filter_;
    bool enable_hole_filling_filter_;
    bool enable_spatial_filter_;
    bool enable_temporal_filter_;
    bool enable_threshold_filter_;
    bool enable_units_transform_;
};

}  // namespace realsense

#ifdef REALSENSE_YAML_CPP
#include <yaml-cpp/yaml.h>

namespace YAML {
template <>
struct convert<realsense::StreamConfig> {
    static Node encode(const realsense::StreamConfig &cfg) {
        Node node;
        node.SetStyle(EmitterStyle::Flow);

        node["stream"] = static_cast<int>(cfg.stream);
        node["width"] = cfg.width;
        node["height"] = cfg.height;
        node["format"] = static_cast<int>(cfg.format);
        node["framerate"] = cfg.framerate;

        return node;
    }

    static bool decode(const Node &node, realsense::StreamConfig &cfg) {
        if (!node.IsMap() || !node["stream"]) {
            return false;
        }

        cfg.stream = static_cast<rs2_stream>(node["stream"].as<int>());

        if (node["width"]) cfg.width = node["width"].as<int>();
        if (node["height"]) cfg.height = node["height"].as<int>();

        if (node["format"])
            cfg.format = static_cast<rs2_format>(node["format"].as<int>());
        if (node["framerate"]) cfg.framerate = node["framerate"].as<int>();

        return true;
    }
};

template <>
struct convert<realsense::Config> {
    static Node encode(const realsense::Config &cfg) {
        Node node;

        node["device_type"] = static_cast<int>(cfg.device_type);
        node["camera_id"] = cfg.camera_id;
        node["serial_id"] = cfg.serial_id;
        node["stream_configs"] = cfg.stream_configs;

        return node;
    }

    static bool decode(const Node &node, realsense::Config &cfg) {
        if (!node.IsMap()) {
            return false;
        }

        if (node["device_type"])
            cfg.device_type = static_cast<realsense::DeviceType>(
                node["device_type"].as<int>());

        if (node["camera_id"]) cfg.camera_id = node["camera_id"].as<int>();
        if (node["serial_id"])
            cfg.serial_id = node["serial_id"].as<std::string>();

        if (node["stream_configs"])
            cfg.stream_configs =
                node["stream_configs"]
                    .as<std::vector<realsense::StreamConfig>>();

        return true;
    }
};
}  // namespace YAML

#endif

#endif
