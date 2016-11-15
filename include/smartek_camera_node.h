#ifndef SMARTEK_CAMERA_NODE_H
#define SMARTEK_CAMERA_NODE_H

#include "gige_cpp/GigEVisionSDK.h"
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <smartek_camera/SmartekCameraConfig.h>

class SmartekCameraNode {

public:
    SmartekCameraNode();
    ~SmartekCameraNode();
    void processFrames();
    void run();

private:
    typedef smartek_camera::SmartekCameraConfig Config;
    void reconfigure_callback(Config& config, uint32_t level);

    Config config_;

    ros::NodeHandle *pn_;
    ros::NodeHandle *pnp_;
    image_transport::CameraPublisher cameraPublisher_;
    image_transport::ImageTransport *pimageTransport_;
    camera_info_manager::CameraInfoManager *pcameraInfoManager_;
    sensor_msgs::CameraInfo cameraInfo_;
    double nodeRate_;
    dynamic_reconfigure::Server<Config> 				reconfigureServer_;
    dynamic_reconfigure::Server<Config>::CallbackType 	reconfigureCallback_;


    bool cameraConnected_;
    std::string serialNumber_; // when using multiple cameras

    // GigEVisionSDK members
    gige::IDevice m_device_;
    gige::IImageProcAPI m_imageProcApi_;

    gige::IAlgorithm m_colorPipelineAlg_;
    gige::IParams m_colorPipelineParams_;
    gige::IResults m_colorPipelineResults_;
    gige::IImageBitmap m_colorPipelineBitmap_;
    gige::IImageInfo m_imageInfo_;

    void ros_publish_gige_image(const gige::IImageBitmapInterface& img );

    bool m_defaultGainNotSet_;
    double m_defaultGain_;

    bool memAllocated_;

    /* TIMESTAMP TUNING */

    // - previous state variables
    double p_cam;
    double p_ros;
    double p_out;
    double p_err;

    double i_err;


    ros::Time sync_timestamp(UINT64 c_cam_uint);
};


#endif //SMARTEK_CAMERA_NODE_H
