#ifndef SMARTEK_CAMERA_NODE_H
#define SMARTEK_CAMERA_NODE_H
/**
 * \file
 * \brief ROS driver for the Smartek camera
 *
 * \par Advertises
 *
 * - \b image_raw The image received from the camera, either in bayer_rg8 or bgra8 formats,
 *      depending whether the Smartek image processing pipeline is used.
 * - \b timestamp_synchronizer/debug_info Debug topic from the TimestampSynchronizer class. Can be used to
        restamp bags using different timestamp correction filter settings.
 *
 * \par Parameters
 *
 * - \b ~SerialNumber The identifier of the camera to be used (used in a multi-camera scenario)
 *
 * \par Dynamically reconfigurable parameters
 *
 * - \b ~ExposureTime The exposure time of the camera, in microseconds.
 * - \b ~AcquisitionFramerate The framerate of image acquisition.
 * - \b ~Gain The camera sensor gain.
 * - \b ~frame_id ROS TF frame of the camera
 * - \b ~SmartekPipeline Whether to use Smartek image processing pipeline.
 * - \b ~EnableTimesync True: use timestamp synchronization provided by the timesync package.
 *                      False: use system time (can be very noisy).
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <timesync/TimestampSynchronizer.h>
#include <Grabber.h>

class SmartekCameraNode {

public:
    /*! Constructor for the Smartek camera driver ROS node.
     */
    SmartekCameraNode();

    /*! Destructor for the Smartek camera driver ROS node.
     *  Stops image acquisition and communication with the camera.
     */
    ~SmartekCameraNode();
    /*! Processes frames until a ROS exit signal is received.
    */
    void run();

private:
    void initTimestampSynchronizer();
    void publishImage(uint8_t *data, int w, int h, int c);

    bool is_time_sync_enabled_;

    ros::NodeHandle n_;
    ros::NodeHandle np_;

    std::string frame_id_;

    Grabber *grabber_;
    uint8_t *data_;
    int w_, h_, c_, device_num_;

    TimestampSynchronizer::Options defaultTimesyncOptions_;
    std::unique_ptr<TimestampSynchronizer> ptimestampSynchronizer_;

    image_transport::CameraPublisher cameraPublisher_;
    std::unique_ptr<image_transport::ImageTransport> pimageTransport_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> pcameraInfoManager_;
    sensor_msgs::CameraInfo cameraInfo_;

};

#endif //SMARTEK_CAMERA_NODE_H
