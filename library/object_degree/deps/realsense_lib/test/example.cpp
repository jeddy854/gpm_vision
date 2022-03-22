#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs2;
using namespace realsense;

int main(int argc, char **argv) {
    Config config;
    StreamConfig color_config;
//     color_config.stream = RS2_STREAM_COLOR;
//     color_config.width = 1280;
//     color_config.height = 720;
//     StreamConfig depth_config;
//     depth_config.stream = RS2_STREAM_DEPTH;
//     depth_config.width = 1024;
//     depth_config.height = 768;
//     StreamConfig infrared_config;
//     infrared_config.stream = RS2_STREAM_INFRARED;
//     infrared_config.width = 1024;
//     infrared_config.height = 768;
//     StreamConfig gyro_config;
//     gyro_config.stream = RS2_STREAM_GYRO;
//     StreamConfig accel_config;
//     accel_config.stream = RS2_STREAM_ACCEL;
//     config.stream_configs.push_back(color_config);
//     config.stream_configs.push_back(depth_config);
//     config.stream_configs.push_back(infrared_config);
//     config.stream_configs.push_back(gyro_config);
//     config.stream_configs.push_back(accel_config);

    RealSense rs(config);
//     rs.connect(config);
    if (rs.depth_supports(RS2_OPTION_MIN_DISTANCE))
    rs.set_depth_option(RS2_OPTION_MIN_DISTANCE, 200);
    rs.set_align_stream(RS2_STREAM_COLOR);
    rs.set_colorizer_option(RS2_OPTION_COLOR_SCHEME, 3);
    rs.enable_hole_filling_filter(true);
    rs.set_hole_filling_filter_option(RS2_OPTION_HOLES_FILL, 1);

    //     vector<rs2_option> options = rs.depth_supported_options();
    //     cout << "depth option: " << "\n";
    //     for (rs2_option &option: options) {
    //         cout << option << "\n";
    //     }
    //     cout << endl;

    Mat color;
    Mat depth_image;
    Mat infrared;
    Mat depth;
    Mat xyz;
    vector<double> data;

    while(true) {
        rs.update();
        rs.retrieve_color_image(color);
        rs.retrieve_infrared_image(infrared);
        rs.retrieve_depth_image(depth_image);

        auto start = chrono::high_resolution_clock::now();
        //         rs.retrieve_depth_measure(depth);
        rs.retrieve_xyz_measure(xyz);
        auto end = chrono::high_resolution_clock::now();
        auto duration = end - start;
        double fps = 1000.0 /
        chrono::duration_cast<chrono::milliseconds>(duration).count();
        cout << "fps: " << fps << endl;

//         rs.retrieve_gyro_measure(data);
//         cout << "gyro: ";
//         for (auto &d: data) {
//             cout << d << " ";
//           }
//         cout << endl;

        imshow("color", color);
        imshow("infrared", infrared);
        imshow("depth", depth_image);

        if (waitKey(1) == 27) break;
      }

    return 0;
  }
