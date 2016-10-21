cmake_minimum_required(VERSION 2.8.9)
project(smartek_camera)

find_package(catkin REQUIRED cv_bridge genmsg image_transport camera_info_manager sensor_msgs)
set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package()

find_package(OpenCV)

INCLUDE_DIRECTORIES($ENV{CAMERA_SDK_PATH}/include ${CMAKE_CURRENT_BINARY_DIR})

set(smartek_camera_node_SRCS
        src/smartek_camera_node.cpp
        $ENV{CAMERA_SDK_PATH}/include/gige_cpp/GigEVisionSDK.cpp
        include/smartek_camera_node.h)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(smartek_camera_node ${smartek_camera_node_SRCS})
target_link_libraries(smartek_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} GigEVisionSDK)
