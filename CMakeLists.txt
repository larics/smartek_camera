cmake_minimum_required(VERSION 2.8.9)
project(smartek_camera)

find_package(catkin REQUIRED COMPONENTS cv_bridge genmsg image_transport camera_info_manager sensor_msgs dynamic_reconfigure timesync)
find_package(OpenCV)

set(CMAKE_CXX_FLAGS "-std=c++14")

generate_dynamic_reconfigure_options(cfg/SmartekCamera.cfg)

set(smartek_camera_node_SRCS
        src/smartek_camera_node.cpp
        $ENV{CAMERA_SDK_PATH}/include/gige_cpp/GigEVisionSDK.cpp)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge image_transport camera_info_manager sensor_msgs dynamic_reconfigure timesync
  DEPENDS OpenCV)

include_directories(
  include 
  $ENV{CAMERA_SDK_PATH}/include 
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_executable(smartek_camera_node ${smartek_camera_node_SRCS})
target_link_libraries(smartek_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} GigEVisionSDK)

add_dependencies(smartek_camera_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
