#include "smartek_camera_node.h"


int main ( int argc, char **argv ) {

    ros::init(argc, argv, "smartek_camera_node");
    ros::NodeHandle n;

    SmartekCameraNode smartek_camera (n);
    ros::Rate rate ( 50 );

    while ( ros::ok() ) {
        smartek_camera.processFrames();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}




inline static cv::Mat gige_image_to_mat_ref(gige::IImageBitmap& img, int format)
{
    UINT32 srcPixelType;
    UINT32 srcWidth, srcHeight;

    img->GetPixelType(srcPixelType);
    img->GetSize(srcWidth, srcHeight);

    UINT32 lineSize = img->GetLineSize();

    return cv::Mat(srcHeight, srcWidth,
                   format, img->GetRawData(), lineSize);
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



SmartekCameraNode::SmartekCameraNode(ros::NodeHandle &n) :  n_(n),  it_(n)  {

    m_device = NULL;
    gige::InitGigEVisionAPI();
    gige::IGigEVisionAPI gigeVisionApi = gige::GetGigEVisionAPI();
    gige::InitImageProcAPI();
    m_imageProcApi = gige::GetImageProcAPI();
    bool cameraConnected = false;

    if (!gigeVisionApi->IsUsingKernelDriver()) {
        ROS_WARN("!!! Warning: Smartek Filter Driver not loaded.)");
    }

    m_colorPipelineAlg = m_imageProcApi->GetAlgorithmByName("ColorPipeline");
    m_colorPipelineAlg->CreateParams(&m_colorPipelineParams);
    m_colorPipelineAlg->CreateResults(&m_colorPipelineResults);
    m_imageProcApi->CreateBitmap(&m_colorPipelineBitmap);

    gigeVisionApi->FindAllDevices(3.0);
    gige::DevicesList devices = gigeVisionApi->GetAllDevices();

    if (devices.size() > 0) {
        // take first device in list
        m_device = devices[0];

        // use: -m GC651C
        // to connect to this model
//        QStringList arg = qApp->arguments();
//        if (arg.size() >= 3 && arg.at(1).compare("-m") == 0) {
//            std::string modelName = arg.at(2).toStdString();
//            for (int i = 0; i < devices.size(); i++) {
//                if (devices[i]->GetModelName().compare(modelName) == 0) {
//                    m_device = devices[i];
//                    break;
//                }
//            }
//        }

        // uncomment to use specific model
        /*for (int i = 0; i < devices.size(); i++) {
            if (devices[i]->GetModelName().compare("GC781C") == 0) {
                m_device = devices[i];
                break;
            }
        }*/

        // to change number of images in image buffer from default 10 images
        // call SetImageBufferFrameCount() method before Connect() method
        //m_device->SetImageBufferFrameCount(20);

        if (m_device != NULL && m_device->Connect()) {
            UINT32 address = m_device->GetIpAddress();

            ROS_INFO_STREAM("IP address: " << IpAddrToString(address));

            // disable trigger mode
            bool status = m_device->SetStringNodeValue("TriggerMode", "Off");
            // set continuous acquisition mode
            status = m_device->SetStringNodeValue("AcquisitionMode", "Continuous");
            // start acquisition
            status = m_device->SetIntegerNodeValue("TLParamsLocked", 1);
            status = m_device->CommandNodeExecute("AcquisitionStart");

            double exposure, gain;
            m_device->GetFloatNodeValue("ExposureTime", exposure);
            ROS_INFO("Exposure: %.2f", exposure);

            m_device->GetFloatNodeValue("Gain", gain);

            ROS_INFO("Gain: %.2f", gain);

            m_defaultGainNotSet = true;
            m_defaultGain = 0.0;
            cameraConnected = true;
        }
    }
    if (!cameraConnected) {
        ROS_ERROR("No camera connected!");
        ros::shutdown();
    }
    pub_ = it_.advertise("image", 1);

}

SmartekCameraNode::~SmartekCameraNode()
{

    if (m_device.IsValid() && m_device->IsConnected()) {
        // stop acquisition
        bool status = m_device->CommandNodeExecute("AcquisitionStop");
        status = m_device->SetIntegerNodeValue("TLParamsLocked", 0);
        m_device->Disconnect();
    }

    m_colorPipelineAlg->DestroyParams(m_colorPipelineParams);
    m_colorPipelineAlg->DestroyResults(m_colorPipelineResults);
    m_imageProcApi->DestroyBitmap(m_colorPipelineBitmap);

    gige::ExitImageProcAPI();
    gige::ExitGigEVisionAPI();
}

void SmartekCameraNode::processFrames()
{
    if (m_device.IsValid() && m_device->IsConnected()) {
        if (!m_device->IsBufferEmpty()) {
            gige::IImageInfo imageInfo;
            m_device->GetImageInfo(&imageInfo);

            if (imageInfo != NULL) {
                m_imageProcApi->ExecuteAlgorithm(m_colorPipelineAlg, imageInfo, m_colorPipelineBitmap, m_colorPipelineParams, m_colorPipelineResults);


                //UINT32 a; m_colorPipelineBitmap->GetPixelType(a);
                cv::Mat matImage = gige_image_to_mat_ref(m_colorPipelineBitmap, CV_8UC4);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", matImage).toImageMsg();
                pub_.publish(msg);
            }

            // remove (pop) image from image buffer
            m_device->PopImage(imageInfo);
            // empty buffer
            m_device->ClearImageBuffer();
        }
    }
}

