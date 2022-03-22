# include "realsense.h"
# include <librealsense2/rs.hpp>
# include <librealsense2/rsutil.h>
# include <omp.h>
# include <opencv2/opencv.hpp>
# include <vector>

using namespace cv;
using namespace std;
using namespace rs2;
using namespace realsense;

void RealSense::retrieve_color_image(cv::Mat &mat) {
    int w = color_frame_.as<video_frame>().get_width();
    int h = color_frame_.as<video_frame>().get_height();

    Mat image(Size(w, h), CV_8UC3, (void*)color_frame_.get_data(),
        Mat::AUTO_STEP);
    cvtColor(image, image, COLOR_BGR2RGB);

    image.copyTo(mat);
  }

void RealSense::retrieve_depth_image(cv::Mat &mat) {
    video_frame depth_image_frame = colorizer_.colorize(depth_frame_);

    int w = depth_image_frame.get_width();
    int h = depth_image_frame.get_height();

    Mat image(Size(w, h), CV_8UC3, (void*)depth_image_frame.get_data(),
        Mat::AUTO_STEP);

    image.copyTo(mat);
  }

void RealSense::retrieve_infrared_image(cv::Mat &mat) {
    int w = infrared_frame_.as<video_frame>().get_width();
    int h = infrared_frame_.as<video_frame>().get_height();

    Mat image(Size(w, h), CV_8U, (void*)infrared_frame_.get_data(),
        Mat::AUTO_STEP);

    image.copyTo(mat);
  }

void RealSense::retrieve_image(cv::Mat &mat, ImageType type) {
    if (type == ImageType::COLOR) {
        retrieve_color_image(mat);
      } else if (type == ImageType::DEPTH) {
          retrieve_depth_image(mat);
        } else if (type == ImageType::INFRARED) {
            retrieve_infrared_image(mat);
          }
  }

void RealSense::retrieve_x_measure(cv::Mat &mat) {
    int w = depth_frame_.as<video_frame>().get_width();
    int h = depth_frame_.as<video_frame>().get_height();

    Mat measure(Size(w, h), CV_32F);

#pragma omp parallel for
    for (int i = 0; i < measure.rows; i++) {
        for (int j = 0; j < measure.cols; j++) {
            vector<float> v;
            retrieve_xyz_measure(v, Point(j, i));
            measure.at<float>(i, j) = v[0];
          }
      }

    measure.copyTo(mat);
  }

void RealSense::retrieve_y_measure(cv::Mat &mat) {
    int w = depth_frame_.as<video_frame>().get_width();
    int h = depth_frame_.as<video_frame>().get_height();

    Mat measure(Size(w, h), CV_32F);

#pragma omp parallel for
    for (int i = 0; i < measure.rows; i++) {
        for (int j = 0; j < measure.cols; j++) {
            vector<float> v;
            retrieve_xyz_measure(v, Point(j, i));
            measure.at<float>(i, j) = v[1];
          }
      }

    measure.copyTo(mat);
  }

void RealSense::retrieve_depth_measure(float &value, cv::Point point) {
    value = depth_frame_.as<depth_frame>().get_distance(point.x, point.y);
  }

void RealSense::retrieve_depth_measure(cv::Mat &mat) {
    int w = depth_frame_.as<video_frame>().get_width();
    int h = depth_frame_.as<video_frame>().get_height();

    Mat measure(Size(w, h), CV_32F);

#pragma omp parallel for
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            retrieve_depth_measure(measure.at<float>(y, x), Point(x, y));
          }
      }

    measure.copyTo(mat);
  }

void RealSense::retrieve_xyz_measure(std::vector<float> &values, cv::Point
    point) {
        float pixel[2] = {(float)point.x, (float)point.y};
        float p[3];
        float depth;
        retrieve_depth_measure(depth, point);

        rs2_deproject_pixel_to_point(p, &depth_intrinsics_, pixel, depth);
        vector<float> v = {p[0],  p[1], p[2]};

        values.assign(v.begin(), v.end());
      }

void RealSense::retrieve_xyz_measure(cv::Mat &mat) {
    int w = depth_frame_.as<video_frame>().get_width();
    int h = depth_frame_.as<video_frame>().get_height();

    Mat measure(Size(w, h), CV_32FC3);

#pragma omp parallel for
    for (int i = 0; i < measure.rows; i++) {
        for (int j = 0; j < measure.cols; j++) {
            vector<float> v;
            retrieve_xyz_measure(v, Point(j, i));

            for (int k = 0; k < 3; k++) {
                measure.at<Vec3f>(i, j)[k] = v[k];
              }
          }
      }

    measure.copyTo(mat);
  }

void RealSense::retrieve_gyro_measure(std::vector<double> &vec) {
    rs2_vector gyro_data = gyro_frame_.as<motion_frame>().get_motion_data();
    std::vector<double> data = {gyro_data.x, gyro_data.y, gyro_data.z};
    vec.assign(data.begin(), data.end());
  }

void RealSense::retrieve_accel_measure(std::vector<double> &vec) {
    rs2_vector accel_data = accel_frame_.as<motion_frame>().get_motion_data();
    std::vector<double> data = {accel_data.x, accel_data.y, accel_data.z};
    vec.assign(data.begin(), data.end());
  }

void RealSense::retrieve_measure(cv::Mat &mat, MeasureType type) {
    if (type == MeasureType::X) {
        retrieve_x_measure(mat);
      } else if (type == MeasureType::Y) {
          retrieve_y_measure(mat);
        } else if (type == MeasureType::DEPTH) {
            retrieve_depth_measure(mat);
          } else if (type == MeasureType::XYZ) {
              retrieve_xyz_measure(mat);
            }
  }

void RealSense::retrieve_measure(std::vector<double> &vec, MeasureType type) {
    if (type == MeasureType::GYRO) {
        retrieve_gyro_measure(vec);
      } else if (type == MeasureType::ACCEL) {
          retrieve_accel_measure(vec);
        }
  }
