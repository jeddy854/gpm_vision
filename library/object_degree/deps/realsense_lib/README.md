# realsense_lib

**to see is not to believe**

## supported hardware
- [D435i](https://www.intelrealsense.com/zh-hans/depth-camera-d435i/)
- [L515](https://www.intelrealsense.com/zh-hans/lidar-camera-l515/)

## dependency
**library**

  - openmp
    
    ```sh
    sudo apt install libomp-dev
    ```
 
 - [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

**CMake**

```sh
find_package(realsense2 REQUIRED)
include_directories(${REALSENSE_INCLUDE_DIR})
find_package(OpenMP)
target_link_libraries(${YOUR_TARGET} realsense_lib)
```

## api
**namespace**

  - rs2
  - realsense

**enumeration**

  - [rs2_stream](https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93)
  - [rs2_format](https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#ae04b7887ce35d16dbd9d2d295d23aac7)
  - [rs2_option](https://intelrealsense.github.io/librealsense/doxygen/rs__option_8h.html#a8b9c011f705cfab20c7eaaa7a26040e2)

**data type**

  - [option_range](https://intelrealsense.github.io/librealsense/doxygen/structrs2_1_1option__range.html)

see [details](lib/realsense.h)


based on [Inter RealSense Cross Platform API](https://intelrealsense.github.io/librealsense/doxygen/index.html)
