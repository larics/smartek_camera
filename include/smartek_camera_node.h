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




class SmartekCameraNode {

public:
    SmartekCameraNode();
    ~SmartekCameraNode();
    void processFrames();

private:
    ros::NodeHandle *pn_;
    ros::NodeHandle *pnp_;
    image_transport::CameraPublisher cameraPublisher_;
    image_transport::ImageTransport *pimageTransport_;
    camera_info_manager::CameraInfoManager *pcameraInfoManager_;
    sensor_msgs::CameraInfo cameraInfo_;


    // GigEVisionSDK members
    gige::IDevice m_device_;
    gige::IImageProcAPI m_imageProcApi_;

    gige::IAlgorithm m_colorPipelineAlg_;
    gige::IParams m_colorPipelineParams_;
    gige::IResults m_colorPipelineResults_;
    gige::IImageBitmap m_colorPipelineBitmap_;
    gige::IImageInfo m_imageInfo_;

    inline void ros_publish_gige_image(gige::IImageBitmap& img );

    bool m_defaultGainNotSet_;
    double m_defaultGain_;
};


#endif //SMARTEK_CAMERA_NODE_H
