#ifndef SMARTEK_CAMERA_NODE_H
#define SMARTEK_CAMERA_NODE_H

#include "gige_cpp/GigEVisionSDK.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp>




class SmartekCameraNode {

public:
    SmartekCameraNode ( ros::NodeHandle &n );
    ~SmartekCameraNode();
    void processFrames();

private:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;

    // GigEVisionSDK members
    gige::IDevice m_device;
    gige::IImageProcAPI m_imageProcApi;

    gige::IAlgorithm m_colorPipelineAlg;
    gige::IParams m_colorPipelineParams;
    gige::IResults m_colorPipelineResults;
    gige::IImageBitmap m_colorPipelineBitmap;

    bool m_defaultGainNotSet;
    double m_defaultGain;

};


#endif //SMARTEK_CAMERA_NODE_H
