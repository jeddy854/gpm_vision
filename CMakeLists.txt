cmake_minimum_required(VERSION 3.0.2)

project(process)
add_compile_options(-std=c++14)

set(OpenCV_STATIC OFF CACHE BOOL "Using OpenCV static linking library")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport 
  cv_bridge 
  nodelet
)
include_directories(${catkin_INCLUDE_DIRS})

find_path(DARKNET_INCLUDE_DIRS yolo_v2_class.hpp
    "/usr/include"
    "/usr/local/include"
    "/home/vision2/api/darknet/include"
    )

find_library(DARKNET_LIBRARIES libdarknet.so
    "/usr/lib"
    "/usr/local/lib"
    "/home/vision2/api/darknet"
    )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${DARKNET_INCLUDE_DIRS}
  )

include_directories(src)

add_executable(yolo_server src/yolo_server.cpp src/main.cpp)

target_link_libraries(yolo_server ${DARKNET_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
