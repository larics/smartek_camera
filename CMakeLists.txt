cmake_minimum_required(VERSION 3.9)
project(smartek_camera)

list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")

find_package(catkin REQUIRED COMPONENTS
        image_transport
        timesync
        sensor_msgs
        camera_info_manager
        cv_bridge
        )

find_package(CameraSuite REQUIRED)
find_package(cv_bridge REQUIRED)
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "-std=c++14")

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs timesync camera_info_manager
  DEPENDS OpenCV)

include_directories(
  include
  ${CAMERASUITE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_library(grabber STATIC src/Grabber.cpp ${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp)
target_link_libraries(grabber ${CAMERASUITE_LIBRARY})

add_executable(smartek_camera_node src/smartek_camera_node.cpp)
target_link_libraries(smartek_camera_node grabber ${OpenCV_LIBS} ${catkin_LIBRARIES})
