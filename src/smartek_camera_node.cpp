#include "smartek_camera_node.h"

/*! Set the default parameters of the TimestampSynchronizer class
 *  and construct an instance.
 */
void SmartekCameraNode::initTimestampSynchronizer() {
    defaultTimesyncOptions_.useMedianFilter = true;
    defaultTimesyncOptions_.medianFilterWindow = 2500;
    defaultTimesyncOptions_.useHoltWinters = true;
    defaultTimesyncOptions_.alfa_HoltWinters = 3e-3;
    defaultTimesyncOptions_.beta_HoltWinters = 2e-3;
    defaultTimesyncOptions_.alfa_HoltWinters_early = 1e-1;
    defaultTimesyncOptions_.beta_HoltWinters_early = 0.0;
    defaultTimesyncOptions_.earlyClamp = true;
    defaultTimesyncOptions_.earlyClampWindow = 500;
    defaultTimesyncOptions_.timeOffset = 0.0;
    defaultTimesyncOptions_.initialB_HoltWinters = -3e-7;
    defaultTimesyncOptions_.nameSuffix = std::string();
    ptimestampSynchronizer_ = std::make_unique<TimestampSynchronizer>(defaultTimesyncOptions_);
}

/*! Publish an image obtained from the GigEVision API
 *
 * \param img the image to be published
 * \param imgInfo the object from which the frame number and sensor timestamp are read
 *
 */
void SmartekCameraNode::publishGigeImage(const gige::IImageBitmapInterface &img, const gige::IImageInfo &imgInfo) {

    double currentRosTime = ros::Time::now().toSec();
    UINT64 currentCamTime_uint = imgInfo->GetCameraTimestamp();
    double currentCamTime = (double) currentCamTime_uint / 1000000.0;


    UINT32 srcPixelType;
    UINT32 srcWidth, srcHeight;

    img.GetPixelType(srcPixelType);
    img.GetSize(srcWidth, srcHeight);

    UINT32 lineSize = img.GetLineSize();

    // construct an openCV image which shares memory with the GigE image
    cv::Mat cvImage(srcHeight, srcWidth, config_.SmartekPipeline ? CV_8UC4 : CV_8UC1, (void *) img.GetRawData(), lineSize);
    // ...then, build an image sensor message from the openCV image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), config_.SmartekPipeline ? "bgra8" : "bayer_rggb8", cvImage).toImageMsg();

    UINT32 seq = imgInfo->GetImageID();

    msg->header.frame_id=config_.frame_id;
    msg->header.seq = seq;

    msg->header.stamp = ros::Time(config_.EnableTimesync ? ptimestampSynchronizer_->sync(currentCamTime, currentRosTime, seq)
                                                         : currentRosTime);

    cameraInfo_ = pcameraInfoManager_->getCameraInfo();
    cameraInfo_.header.stamp = msg->header.stamp;
    cameraInfo_.header.seq = msg->header.seq;
    cameraInfo_.header.frame_id = msg->header.frame_id;
    cameraInfo_.width = srcWidth;
    cameraInfo_.height = srcHeight;

    cameraPublisher_.publish(*msg, cameraInfo_);
}

/*! IPv4 address conversion to string
 *
 * \param ipAddress the IP address in integer form
 */
static std::string IpAddrToString(UINT32 ipAddress) {
    std::stringstream outStream;
    UINT32 part1, part2, part3, part4;

    part1 = ((ipAddress >> 24) & 0xFF);
    part2 = ((ipAddress >> 16) & 0xFF);
    part3 = ((ipAddress >> 8) & 0xFF);
    part4 = ((ipAddress) & 0xFF);

    outStream << part1 << "." << part2 << "." << part3 << "." << part4;

    return outStream.str();
}

void SmartekCameraNode::run() {
    while ( ros::ok() && isCameraConnected_ ) {
        processFrames();
        ros::spinOnce();
    }
}

SmartekCameraNode::SmartekCameraNode() {
    pn_ = std::make_unique<ros::NodeHandle>();
    pnp_ =std::make_unique<ros::NodeHandle>(std::string("~"));

    if(config_.EnableTimesync) {
        initTimestampSynchronizer();
    }

    serialNumber_ = pnp_->param<std::string>("SerialNumber", std::string());

    m_device_ = NULL;
    gige::InitGigEVisionAPI();
    gige::IGigEVisionAPI gigeVisionApi = gige::GetGigEVisionAPI();
    gige::InitImageProcAPI();
    m_imageProcApi_ = gige::GetImageProcAPI();
    isCameraConnected_ = false;

    if (!gigeVisionApi->IsUsingKernelDriver()) {
        ROS_WARN("!!! Warning: Smartek Filter Driver not loaded.");
    }

    m_colorPipelineAlg_ = m_imageProcApi_->GetAlgorithmByName("ColorPipeline");
    m_colorPipelineAlg_->CreateParams(&m_colorPipelineParams_);
    m_colorPipelineAlg_->CreateResults(&m_colorPipelineResults_);
    m_imageProcApi_->CreateBitmap(&m_colorPipelineBitmap_);

    ROS_INFO("Beginning device discovery...");
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
            ROS_INFO_STREAM("Device IP address: " << IpAddrToString(address));
            ROS_INFO_STREAM("Device serial number: " << m_device_->GetSerialNumber());

            // disable trigger mode
            m_device_->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            m_device_->SetStringNodeValue("AcquisitionMode", "Continuous");

            // start acquisition
            m_device_->SetIntegerNodeValue("TLParamsLocked", 1);
            m_device_->CommandNodeExecute("AcquisitionStart");

            double currentExposure, currentGain, currentAcquisitionFramerate;
            m_device_->GetFloatNodeValue("ExposureTime", currentExposure);
            ROS_INFO("Exposure: %.2lf", currentExposure);

            m_device_->GetFloatNodeValue("Gain", currentGain);
            ROS_INFO("Gain: %.2lf", currentGain);

            m_device_->GetFloatNodeValue("AcquisitionFrameRate", currentAcquisitionFramerate);
            ROS_INFO("Acquisition framerate: %.2lf", currentAcquisitionFramerate);

            ROS_INFO("Timesync %s", config_.EnableTimesync ? "enabled" : "disabled");

            //m_defaultGainNotSet_ = true;
            //m_defaultGain_ = 0.0;
            isCameraConnected_ = true;
        }
    }
    if (!isCameraConnected_) {
        ROS_ERROR("No camera connected!");
        if(serialNumber_.size() > 0)
            ROS_ERROR_STREAM("Requested camera with serial number " << serialNumber_);
    }
    else {
        pimageTransport_ = std::make_unique<image_transport::ImageTransport>(*pnp_);
        cameraPublisher_ = pimageTransport_->advertiseCamera("image_raw", 10);

        pcameraInfoManager_ = std::make_unique<camera_info_manager::CameraInfoManager>(*pnp_, m_device_->GetSerialNumber());

        reconfigureCallback_ = boost::bind(&SmartekCameraNode::reconfigure_callback, this, _1, _2);
        reconfigureServer_.setCallback(reconfigureCallback_);
    }
}

void SmartekCameraNode::reconfigure_callback(Config &config, uint32_t level) {
    if(isCameraConnected_) {
        ROS_INFO("Reconfiguring camera");

        m_device_->CommandNodeExecute("AcquisitionStop");
        m_device_->SetIntegerNodeValue("TLParamsLocked", 0);

        m_device_->SetFloatNodeValue("ExposureTime", config.ExposureTime);
        m_device_->SetFloatNodeValue("Gain", config.Gain);
        // the property in the GigEVision api is "AcqusitionFrameRate", with a capital R
        m_device_->SetFloatNodeValue("AcquisitionFrameRate", config.AcquisitionFramerate);

        ROS_INFO("New exposure: %.2lf", config.ExposureTime);
        ROS_INFO("New gain: %.2lf", config.Gain);
        ROS_INFO("New acquisition framerate: %.2lf", config.AcquisitionFramerate);
        ROS_INFO("Timesync %s", config.EnableTimesync ? "enabled" : "disabled");

        // reinitialize timesync if it has just been enabled
        if(config.EnableTimesync && !config_.EnableTimesync) {
            initTimestampSynchronizer();
        }

        m_device_->SetIntegerNodeValue("TLParamsLocked", 1);
        m_device_->CommandNodeExecute("AcquisitionStart");

        config_ = config;
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
}

void SmartekCameraNode::processFrames() {
    if (m_device_.IsValid() && m_device_->IsConnected()) {
        bool gotImage = false;

        // sleep until the next frame is received
        if(m_device_->WaitForImage(10.0)) {
            if (!m_device_->IsBufferEmpty()) {

                m_device_->GetImageInfo(&m_imageInfo_);

                if (m_imageInfo_ != NULL) {
                    gotImage = true;

                    if (config_.SmartekPipeline) {
                        m_imageProcApi_->ExecuteAlgorithm(m_colorPipelineAlg_, m_imageInfo_, m_colorPipelineBitmap_,
                                                          m_colorPipelineParams_, m_colorPipelineResults_);
                        publishGigeImage(gige::IImageBitmapInterface(m_colorPipelineBitmap_), m_imageInfo_);
                    } else
                        publishGigeImage(gige::IImageBitmapInterface(m_imageInfo_), m_imageInfo_);
                }

                // remove (pop) image from image buffer
                m_device_->PopImage(m_imageInfo_);
                // empty buffer
                m_device_->ClearImageBuffer();
            }
        }
        if(!gotImage){
            ROS_ERROR("Image not received!");
        }
    }
    else {
        isCameraConnected_ = false;
        ROS_ERROR("Camera disconnected!");
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera");
    ros::start();

    // if the smartek camera node quits for some reason, e.g. camera disconnected,
    // try to construct it again until the application receives an interrupt
    while(ros::ok()) {
        SmartekCameraNode().run();
    }

    ros::shutdown();
    return 0;
}
