#include "smartek_camera_node.h"


int main ( int argc, char **argv ) {

    ros::init(argc, argv, "smartek_camera_node");

    ros::NodeHandle n;
    ros::NodeHandle np(std::string("~"));

    SmartekCameraNode smartek_camera;
    ros::Rate rate ( 50 );

    while ( ros::ok() ) {
        smartek_camera.processFrames();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}




inline void SmartekCameraNode::ros_publish_gige_image(gige::IImageBitmap& img )

{
    UINT32 srcPixelType;
    UINT32 srcWidth, srcHeight;

    img->GetPixelType(srcPixelType);
    img->GetSize(srcWidth, srcHeight);

    UINT32 lineSize = img->GetLineSize();

    // construct an openCV image sharing memory with GigE
    cv::Mat opencv_image(srcHeight, srcWidth, CV_8UC4, img->GetRawData(), lineSize);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", opencv_image).toImageMsg();
    msg->header.frame_id="camera";
    msg->header.seq = m_imageInfo_->GetImageID();


    UINT64 timestamp_seconds = m_imageInfo_->GetTimestamp();
    UINT64 timestamp_nanoseconds = m_imageInfo_->GetCameraTimestamp();
    msg->header.stamp = ros::Time::now(); //ros::Time(timestamp_seconds, 0); timestamp_nanoseconds*1000); // fixme nanoseconds

    cameraInfo_ = pcameraInfoManager_->getCameraInfo();
    cameraInfo_.header.stamp = msg->header.stamp;
    cameraInfo_.header.seq = msg->header.seq;
    cameraInfo_.header.frame_id = msg->header.frame_id;
    cameraInfo_.width = srcWidth;
    cameraInfo_.height = srcHeight;

    cameraPublisher_.publish(*msg, cameraInfo_);
}

// IPv4 address conversion to string
static std::string IpAddrToString(UINT32 ipAddress)
{
    std::stringstream stream;
    UINT32 temp1, temp2, temp3, temp4;

    temp1 = ((ipAddress >> 24) & 0xFF);
    temp2 = ((ipAddress >> 16) & 0xFF);
    temp3 = ((ipAddress >> 8) & 0xFF);
    temp4 = ((ipAddress) & 0xFF);

    stream << temp1 << "." << temp2 << "." << temp3 << "." << temp4;

    return stream.str();
}



SmartekCameraNode::SmartekCameraNode() {

    m_device_ = NULL;
    gige::InitGigEVisionAPI();
    gige::IGigEVisionAPI gigeVisionApi = gige::GetGigEVisionAPI();
    gige::InitImageProcAPI();
    m_imageProcApi_ = gige::GetImageProcAPI();
    bool cameraConnected = false;

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
        // take first device in list
        m_device_ = devices[0];

        // use: -m GC651C
        // to connect to this model
//        QStringList arg = qApp->arguments();
//        if (arg.size() >= 3 && arg.at(1).compare("-m") == 0) {
//            std::string modelName = arg.at(2).toStdString();
//            for (int i = 0; i < devices.size(); i++) {
//                if (devices[i]->GetModelName().compare(modelName) == 0) {
//                    m_device_ = devices[i];
//                    break;
//                }
//            }
//        }

        // uncomment to use specific model
        /*for (int i = 0; i < devices.size(); i++) {
            if (devices[i]->GetModelName().compare("GC781C") == 0) {
                m_device_ = devices[i];
                break;
            }
        }*/

        // to change number of images in image buffer from default 10 images
        // call SetImageBufferFrameCount() method before Connect() method
        //m_device_->SetImageBufferFrameCount(20);

        if (m_device_ != NULL && m_device_->Connect()) {
            UINT32 address = m_device_->GetIpAddress();
            ROS_INFO_STREAM("IP address: " << IpAddrToString(address));

            // disable trigger mode
            bool status = m_device_->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            status = m_device_->SetStringNodeValue("AcquisitionMode", "Continuous");
            // start acquisition
            status = m_device_->SetIntegerNodeValue("TLParamsLocked", 1);
            status = m_device_->CommandNodeExecute("AcquisitionStart");

            double exposure, gain;
            m_device_->GetFloatNodeValue("ExposureTime", exposure);
            ROS_INFO("Exposure: %.2f", exposure);

            m_device_->GetFloatNodeValue("Gain", gain);

            ROS_INFO("Gain: %.2f", gain);

            m_defaultGainNotSet_ = true;
            m_defaultGain_ = 0.0;
            cameraConnected = true;
        }
    }
    if (!cameraConnected) {
        ROS_ERROR("No camera connected!");
        ros::shutdown();
    }

    pn_ = new ros::NodeHandle();
    pnp_ = new ros::NodeHandle(std::string("~"));

    pimageTransport_ = new image_transport::ImageTransport(*pnp_);
    cameraPublisher_ = pimageTransport_->advertiseCamera(ros::this_node::getName()+"/image_color", 1);

    pcameraInfoManager_ = new camera_info_manager::CameraInfoManager(ros::NodeHandle(ros::this_node::getName()), m_device_->GetModelName());

}

SmartekCameraNode::~SmartekCameraNode()
{

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

    delete pn_;
    delete pnp_;
    delete pimageTransport_;
    delete pcameraInfoManager_;

}

void SmartekCameraNode::processFrames() {
    if (m_device_.IsValid() && m_device_->IsConnected()) {
        if (!m_device_->IsBufferEmpty()) {

            m_device_->GetImageInfo(&m_imageInfo_);

            if (m_imageInfo_ != NULL) {
                m_imageProcApi_->ExecuteAlgorithm(m_colorPipelineAlg_, m_imageInfo_, m_colorPipelineBitmap_, m_colorPipelineParams_, m_colorPipelineResults_);

                //UINT32 a; m_colorPipelineBitmap_->GetPixelType(a);
                ros_publish_gige_image(m_colorPipelineBitmap_);


            }

            // remove (pop) image from image buffer
            m_device_->PopImage(m_imageInfo_);
            // empty buffer
            m_device_->ClearImageBuffer();
        }
    }
}

