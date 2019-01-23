#include <cmath>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
    ptimestampSynchronizer_ = std::make_unique<TimestampSynchronizer>(defaultTimesyncOptions_);
}

/*! Publish an image obtained from the CameraSuite API
 *
 * \param img the image to be published
 * \param imgInfo the object from which the frame number and sensor timestamp are read
 *
 */
/*void SmartekCameraNode::publishGigeImage(const gige::IImageBitmapInterface &img, const gige::IImageInfo &imgInfo) {

    double currentRosTime = ros::Time::now().toSec();

    double currentCamTime = static_cast<double>(imgInfo->GetCameraTimestamp()) / 1000000.0;
    UINT32 seq = imgInfo->GetImageID();

    UINT32 srcPixelType;
    UINT32 srcWidth, srcHeight;

    img.GetPixelType(srcPixelType);
    img.GetSize(srcWidth, srcHeight);

    UINT32 lineSize = img.GetLineSize();

    // construct an openCV image which shares memory with the GigE image
    cv::Mat cvImage(srcHeight, srcWidth, config_.SmartekPipeline ? CV_8UC4 : CV_8UC1, (void *) img.GetRawData(), lineSize);
    // ...then, build an image sensor message from the openCV image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), config_.SmartekPipeline ? "bgra8" : "bayer_rggb8", cvImage).toImageMsg();

    msg->header.frame_id=frame_id_;
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
}*/

void SmartekCameraNode::run() {
    ros::Rate rate(100);

    cv::Mat im(2048, 2592, 16);
    cv::namedWindow("w", 1);

    while (ros::ok() && device_num_ != -1) {
        ros::spinOnce();
        data_ = grabber_->grab(device_num_, w_, h_, c_);

        if (data_ != NULL) {
            //cv::Mat cvImage(h_, w_, CV_8UC4, (void *) data_);
            publishImage(data_, w_, h_, c_);
            //free(data_);
        }
        cv::waitKey(1);

        rate.sleep();
    }
}

void SmartekCameraNode::publishImage(uint8_t *data, int w, int h, int c) {
    double currentRosTime = ros::Time::now().toSec();
    double currentCamTime = grabber_->getCameraTimestamp(device_num_) / 1000000.0;
    uint32_t seq = grabber_->getImageID(device_num_);

    /*cv_bridge::CvImage cv_image;
    cv_image.image = cvImage(w, h, CV_8UC4, (void*)data);
    cv_image.encoding = "rgb8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);*/

    cv::Mat cvImage(h, w, CV_8UC4, (void*)data);
    cv::imshow("w", cvImage);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", cvImage).toImageMsg();

    msg->header.frame_id = frame_id_;
    msg->header.seq = seq;
    msg->header.stamp = ros::Time(currentRosTime);//ros::Time(ptimestampSynchronizer_->sync(currentCamTime, currentRosTime, seq));

    cameraInfo_ = pcameraInfoManager_->getCameraInfo();
    cameraInfo_.header.stamp = msg->header.stamp;
    cameraInfo_.header.seq = msg->header.seq;
    cameraInfo_.header.frame_id = msg->header.frame_id;
    cameraInfo_.width = w;
    cameraInfo_.height = h;

    cameraPublisher_.publish(*msg, cameraInfo_);

    //printf("Timestamp: %f\n", currentCamTime);
    //printf("Seq: %d\n", seq);
    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    //cv::imshow( "Display window", cvImage );
    //cv::waitKey(1);
    //printf("Taj w: %d, h: %d, c: %d\n", w, h,c);
}

SmartekCameraNode::SmartekCameraNode():
        is_time_sync_enabled_(true),
        data_(NULL),
        w_(0),
        h_(0),
        c_(0),
        device_num_(-1) {
    n_ = ros::NodeHandle();
    np_ = ros::NodeHandle(std::string("~"));

    frame_id_ = np_.param<std::string>("frame_id", "camera");

    grabber_ = new Grabber();

    grabber_->findDevices();
    device_num_ = grabber_->getDeviceBySerialNumber("10130000");
    printf("device_num %d\n", device_num_);

    if (device_num_ != -1) grabber_->connect(device_num_);

    if(is_time_sync_enabled_) {
        initTimestampSynchronizer();
    }

    pimageTransport_ = std::make_unique<image_transport::ImageTransport>(np_);
    cameraPublisher_ = pimageTransport_->advertiseCamera("image_raw", 10);
    pcameraInfoManager_ = std::make_unique<camera_info_manager::CameraInfoManager>(np_, std::string("10130000"));
}

SmartekCameraNode::~SmartekCameraNode() {
    grabber_->disconect(device_num_);
    delete grabber_;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "smartek_camera_driver");
    ros::start();

    // if the smartek camera node quits for some reason, e.g. camera disconnected,
    // try to construct it again until the application receives an interrupt
    while(ros::ok()) {
        SmartekCameraNode().run();
    }

    ros::shutdown();
    return 0;
}
