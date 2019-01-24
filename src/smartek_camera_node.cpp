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

void SmartekCameraNode::run() {
    ros::Rate rate(100);

    volatile bool first_grab = false;

    while (ros::ok() && device_num_ != -1 && !first_grab) {
        data_ = grabber_->grab(device_num_, w_, h_, c_, pixel_type_);
        if (data_ != NULL && w_ != 0 && h_ != 0) {
            first_grab = true;
        }
        grabber_->popImage(device_num_);
        rate.sleep();
    }

    if (c_ == 1) {
        cv::Mat im(h_, w_, CV_8UC1);
    }
    else if (c_ == 2) {
        cv::Mat im(h_, w_, CV_8UC2);
    }
    else if (c_ == 3) {
        cv::Mat im(h_, w_, CV_8UC3);
    }
    else {
        cv::Mat im(h_, w_, CV_8UC4);
    }

    while (ros::ok() && device_num_ != -1) {
        ros::spinOnce();
        data_ = grabber_->grab(device_num_, w_, h_, c_, pixel_type_);

        if (data_ != NULL && w_ != 0 && h_ != 0) {
            publishImage(data_, w_, h_, c_, pixel_type_);
            grabber_->popImage(device_num_);
        }
        rate.sleep();
    }
}

void SmartekCameraNode::publishImage(uint8_t *data, int w, int h, int c, uint32_t pixel_type_) {
    double currentRosTime = ros::Time::now().toSec();
    double currentCamTime = grabber_->getCameraTimestamp(device_num_) / 1000000.0;
    std::string cv_bridge_type;
    uint32_t seq = grabber_->getImageID(device_num_);
    static int cv_type;

    if (c == 1) {
        cv_type = CV_8UC1;
    }
    else if (c == 2) {
        cv_type = CV_8UC2;
    }
    else if (c == 3) {
        cv_type = CV_8UC3;
    }
    else {
        cv_type = CV_8UC4;
    }

    cv::Mat cvImage(h, w, cv_type, (void*)data);

    cv_bridge_type = getCvBridgeType(pixel_type_);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), cv_bridge_type, cvImage).toImageMsg();

    msg->header.frame_id = frame_id_;
    msg->header.seq = seq;
    msg->header.stamp = ros::Time(enableTimesync_ ? ptimestampSynchronizer_->sync(currentCamTime, currentRosTime, seq)
                                                         : currentRosTime);

    cameraInfo_ = pcameraInfoManager_->getCameraInfo();
    cameraInfo_.header.stamp = msg->header.stamp;
    cameraInfo_.header.seq = msg->header.seq;
    cameraInfo_.header.frame_id = msg->header.frame_id;
    cameraInfo_.width = w;
    cameraInfo_.height = h;

    cameraPublisher_.publish(*msg, cameraInfo_);
}

std::string SmartekCameraNode::getCvBridgeType(uint32_t pixel_type) {
    std::string cv_bridge_type;

    if (pixel_type == GVSP_PIX_MONO8 || pixel_type == GVSP_PIX_MONO8_SIGNED) {
        cv_bridge_type = std::string("mono8");
    }
    else if (pixel_type == GVSP_PIX_MONO16) {
        cv_bridge_type = std::string("mono16");
    }
    else if (pixel_type == GVSP_PIX_RGB8_PACKED) {
        cv_bridge_type = std::string("rgb8");
    }
    else if (pixel_type == GVSP_PIX_BGR8_PACKED) {
        cv_bridge_type = std::string("bgr8");
    }
    else if (pixel_type == GVSP_PIX_RGBA8_PACKED) {
        cv_bridge_type = std::string("rgba8");
    }
    else if (pixel_type == GVSP_PIX_BGRA8_PACKED) {
        cv_bridge_type = std::string("bgra8");
    }
    else if (pixel_type == GVSP_PIX_BAYGR8) {
        cv_bridge_type = std::string("bayer_bggr8");
    }
    else if (pixel_type == GVSP_PIX_BAYRG8) {
        cv_bridge_type = std::string("bayer_gbrg8");
    }
    else if (pixel_type == GVSP_PIX_BAYGB8) {
        cv_bridge_type = std::string("bayer_rggb8");
    }
    else if (pixel_type == GVSP_PIX_BAYBG8) {
        cv_bridge_type = std::string("bayer_grbg8");
    }

    return cv_bridge_type;
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

    camera_ip_ = np_.param<std::string>("camera_ip", "10000085");
    frame_id_ = np_.param<std::string>("frame_id", "camera");
    enableTimesync_ = np_.param<bool>("time_sync", true);
    image_proc_type_ = np_.param<int>("image_proc_type", 1);

    grabber_ = new Grabber(image_proc_type_);

    grabber_->findDevices();
    device_num_ = grabber_->getDeviceBySerialNumber(camera_ip_.c_str());

    if (device_num_ != -1) grabber_->connect(device_num_);

    if(is_time_sync_enabled_) {
        initTimestampSynchronizer();
    }

    pimageTransport_ = std::make_unique<image_transport::ImageTransport>(np_);
    cameraPublisher_ = pimageTransport_->advertiseCamera("image_raw", 10);
    pcameraInfoManager_ = std::make_unique<camera_info_manager::CameraInfoManager>(np_, std::string(camera_ip_.c_str()));
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
