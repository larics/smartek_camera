#include "smartek_camera_node.h"

int main ( int argc, char **argv ) {

    ros::init(argc, argv, "camera");
    SmartekCameraNode().run();

    return 0;
}

void SmartekCameraNode::ros_publish_gige_image(const gige::IImageBitmapInterface& img ) {

    UINT32 srcPixelType;
    UINT32 srcWidth, srcHeight;

    img.GetPixelType(srcPixelType);
    img.GetSize(srcWidth, srcHeight);

    UINT32 lineSize = img.GetLineSize();

    // construct an openCV image sharing memory with GigE
    cv::Mat opencv_image(srcHeight, srcWidth, config_.SmartekPipeline ? CV_8UC4 : CV_8UC1, (void *) img.GetRawData(), lineSize);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), config_.SmartekPipeline ? "bgra8" : "bayer_rggb8", opencv_image).toImageMsg();

    msg->header.frame_id=config_.frame_id;
    msg->header.seq = m_imageInfo_->GetImageID();


    //UINT64 timestamp_seconds = m_imageInfo_->GetTimestamp();
    //UINT64 timestamp_nanoseconds = m_imageInfo_->GetCameraTimestamp();
    msg->header.stamp = config_.EnableTuning ? sync_timestamp(m_imageInfo_->GetCameraTimestamp()) : ros::Time::now();

    cameraInfo_ = pcameraInfoManager_->getCameraInfo();
    cameraInfo_.header.stamp = msg->header.stamp;
    cameraInfo_.header.seq = msg->header.seq;
    cameraInfo_.header.frame_id = msg->header.frame_id;
    cameraInfo_.width = srcWidth;
    cameraInfo_.height = srcHeight;

    cameraPublisher_.publish(*msg, cameraInfo_);

}

ros::Time SmartekCameraNode::sync_timestamp(UINT64 c_cam_uint){
    double c_ros = ros::Time::now().toSec();
    double c_cam = (double) c_cam_uint / 1000000.0;
    if(m_imageInfo_->GetImageID() < 10){
        p_cam = c_cam;
        p_out = c_ros;
    }

    double c_err = p_out + (c_cam - p_cam) - c_ros; // difference between current and calculated time
    double d_err = c_err - p_err; // derivative of error
    i_err = i_err + c_err; // integral of error

    double pid_res = (config_.tune_kp*c_err + config_.tune_ki*i_err + config_.tune_kd*d_err + config_.TimeOffset) / 1000.0;

    double c_out = p_out + (c_cam - p_cam) + pid_res;

    p_cam = c_cam;
    p_ros = c_ros;
    p_out = c_out;
    p_err = c_err;

    //ROS_INFO("ROS_TIME: %f\tTIMESTAMP: %f\tERROR: %f\tPID_RES: %f", c_ros, c_out, c_err, pid_res);

    return ros::Time(c_out);
}

// IPv4 address conversion to string
static std::string IpAddrToString(UINT32 ipAddress) {
    std::stringstream stream;
    UINT32 temp1, temp2, temp3, temp4;

    temp1 = ((ipAddress >> 24) & 0xFF);
    temp2 = ((ipAddress >> 16) & 0xFF);
    temp3 = ((ipAddress >> 8) & 0xFF);
    temp4 = ((ipAddress) & 0xFF);

    stream << temp1 << "." << temp2 << "." << temp3 << "." << temp4;

    return stream.str();
}


void SmartekCameraNode::run() {
    ros::Rate rate(nodeRate_);
    while ( ros::ok() ) {
        processFrames();
        ros::spinOnce();
        rate.sleep();
    }
}


SmartekCameraNode::SmartekCameraNode() {
    pn_ = new ros::NodeHandle();
    pnp_ = new ros::NodeHandle(std::string("~"));

    serialNumber_ = pnp_->param<std::string>("SerialNumber", std::string());

    m_device_ = NULL;
    gige::InitGigEVisionAPI();
    gige::IGigEVisionAPI gigeVisionApi = gige::GetGigEVisionAPI();
    gige::InitImageProcAPI();
    m_imageProcApi_ = gige::GetImageProcAPI();
    cameraConnected_ = false;

    if (!gigeVisionApi->IsUsingKernelDriver()) {
        ROS_WARN("!!! Warning: Smartek Filter Driver not loaded.");
    }

    m_colorPipelineAlg_ = m_imageProcApi_->GetAlgorithmByName("ColorPipeline");
    m_colorPipelineAlg_->CreateParams(&m_colorPipelineParams_);
    m_colorPipelineAlg_->CreateResults(&m_colorPipelineResults_);
    m_imageProcApi_->CreateBitmap(&m_colorPipelineBitmap_);

    gigeVisionApi->FindAllDevices(3.0);
    gige::DevicesList devices = gigeVisionApi->GetAllDevices();

    if (devices.size() > 0) {

        if(serialNumber_.size() > 0) {
            for(int i = 0; i < devices.size(); i++)
                if(devices[i]->GetSerialNumber() == serialNumber_) {
                    m_device_ = devices[i];
                    break;
                }
            }
        else
            m_device_ = devices[0];

        // to change number of images in image buffer from default 10 images
        // call SetImageBufferFrameCount() method before Connect() method
        //m_device_->SetImageBufferFrameCount(20);

        if (m_device_ != NULL && m_device_->Connect()) {
            UINT32 address = m_device_->GetIpAddress();
            ROS_INFO_STREAM("IP address: " << IpAddrToString(address));

            // disable trigger mode
            m_device_->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            m_device_->SetStringNodeValue("AcquisitionMode", "Continuous");

            // start acquisition
            m_device_->SetIntegerNodeValue("TLParamsLocked", 1);
            m_device_->CommandNodeExecute("AcquisitionStart");

            double exposure, gain, acquisitionframerate;
            m_device_->GetFloatNodeValue("ExposureTime", exposure);
            ROS_INFO("Exposure: %.2lf", exposure);

            m_device_->GetFloatNodeValue("Gain", gain);
            ROS_INFO("Gain: %.2lf", gain);

            m_device_->GetFloatNodeValue("AcquisitionFramerate", acquisitionframerate);
            ROS_INFO("Acquisition framerate: %.2lf", acquisitionframerate);

            ROS_INFO("Timestamp tuning %s", config_.EnableTuning ? "ENABLED" : "DISABLED");
            ROS_INFO("Time offset: %f", config_.TimeOffset / 1000.0);
            ROS_INFO("Tune Kp: %f", config_.tune_kp / 1000.0);
            ROS_INFO("Tune Ki: %f", config_.tune_ki / 1000.0);
            ROS_INFO("Tune Kd: %f", config_.tune_kd / 1000.0);

            //m_defaultGainNotSet_ = true;
            //m_defaultGain_ = 0.0;
            cameraConnected_ = true;
        }
    }
    if (!cameraConnected_) {
        ROS_ERROR("No camera connected!");
        if(serialNumber_.size() > 0)
            ROS_ERROR_STREAM("Requested camera with serial number " << serialNumber_);
        ros::requestShutdown();
        memAllocated_ = false;
    }
    else {
        pnp_->param<double>("NodeRate", nodeRate_, 50);

        pimageTransport_ = new image_transport::ImageTransport(*pnp_);
        cameraPublisher_ = pimageTransport_->advertiseCamera("image_raw", 1);

        pcameraInfoManager_ = new camera_info_manager::CameraInfoManager(*pnp_, m_device_->GetSerialNumber());
        memAllocated_ = true;

        reconfigureCallback_ = boost::bind(&SmartekCameraNode::reconfigure_callback, this, _1, _2);
        reconfigureServer_.setCallback(reconfigureCallback_);
    }
}

void SmartekCameraNode::reconfigure_callback(Config &config, uint32_t level) {
    config_ = config;

    if(cameraConnected_) {
        ROS_INFO("Reconfiguring camera");

        m_device_->CommandNodeExecute("AcquisitionStop");
        m_device_->SetIntegerNodeValue("TLParamsLocked", 0);

        m_device_->SetFloatNodeValue("ExposureTime", config_.ExposureTime);
        m_device_->SetFloatNodeValue("Gain", config_.Gain);
        m_device_->SetFloatNodeValue("AcquisitionFrameRate", config_.AcquisitionFrameRate);

        ROS_INFO("New exposure: %.2lf", config_.ExposureTime);
        ROS_INFO("New gain: %.2lf", config_.Gain);
        ROS_INFO("New acquisition framerate: %.2lf", config_.AcquisitionFrameRate);
        ROS_INFO("Timestamp tuning %s", config_.EnableTuning ? "ENABLED" : "DISABLED");
        ROS_INFO("Time offset: %f", config_.TimeOffset / 1000.0);
        ROS_INFO("Tune Kp: %f", config_.tune_kp / 1000.0);
        ROS_INFO("Tune Ki: %f", config_.tune_ki / 1000.0);
        ROS_INFO("Tune Kd: %f", config_.tune_kd / 1000.0);

        m_device_->SetIntegerNodeValue("TLParamsLocked", 1);
        m_device_->CommandNodeExecute("AcquisitionStart");
    }

}

SmartekCameraNode::~SmartekCameraNode() {

    if (m_device_.IsValid() && m_device_->IsConnected()) {
        // stop acquisition
        bool status = m_device_->CommandNodeExecute("AcquisitionStop");
        status = m_device_->SetIntegerNodeValue("TLParamsLocked", 0);
        m_device_->Disconnect();
    }

    m_colorPipelineAlg_->DestroyParams(m_colorPipelineParams_);
    m_colorPipelineAlg_->DestroyResults(m_colorPipelineResults_);
    m_imageProcApi_->DestroyBitmap(m_colorPipelineBitmap_);

    gige::ExitImageProcAPI();
    gige::ExitGigEVisionAPI();

    if(memAllocated_) {
        delete pimageTransport_;
        delete pcameraInfoManager_;
    }

    delete pn_;
    delete pnp_;
}

void SmartekCameraNode::processFrames() {
    if (m_device_.IsValid() && m_device_->IsConnected()) {
        if (!m_device_->IsBufferEmpty()) {

            m_device_->GetImageInfo(&m_imageInfo_);

            if (m_imageInfo_ != NULL) {
                if(config_.SmartekPipeline) {
                    m_imageProcApi_->ExecuteAlgorithm(m_colorPipelineAlg_, m_imageInfo_, m_colorPipelineBitmap_,
                                                      m_colorPipelineParams_, m_colorPipelineResults_);
                    ros_publish_gige_image(gige::IImageBitmapInterface(m_colorPipelineBitmap_));
                } else
                    ros_publish_gige_image(gige::IImageBitmapInterface(m_imageInfo_));

            }

            // remove (pop) image from image buffer
            m_device_->PopImage(m_imageInfo_);
            // empty buffer
            m_device_->ClearImageBuffer();
        }
    }
}

