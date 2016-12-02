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
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <timesync/TimestampSynchronizer.h>

#include <smartek_camera/SmartekCameraConfig.h>

#include <gige_cpp/GigEVisionSDK.h>

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
    typedef smartek_camera::SmartekCameraConfig Config;
    void reconfigure_callback(Config& config, uint32_t level);
    void publishGigeImage(const gige::IImageBitmapInterface &img, const gige::IImageInfo &imgInfo);
    void processFrames();
    void initTimestampSynchronizer();

    Config config_;
    std::string frame_id_; // ROS tf frame id

    ros::NodeHandle n_;
    ros::NodeHandle np_;

    image_transport::CameraPublisher cameraPublisher_;
    std::unique_ptr<image_transport::ImageTransport> pimageTransport_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> pcameraInfoManager_;
    sensor_msgs::CameraInfo cameraInfo_;

    dynamic_reconfigure::Server<Config> 				reconfigureServer_;
    dynamic_reconfigure::Server<Config>::CallbackType 	reconfigureCallback_;

    bool isCameraConnected_;
    std::string serialNumber_; // when using multiple cameras

    // GigEVisionSDK members
    gige::IDevice m_device_;
    gige::IImageProcAPI m_imageProcApi_;

    gige::IAlgorithm m_colorPipelineAlg_;
    gige::IParams m_colorPipelineParams_;
    gige::IResults m_colorPipelineResults_;
    gige::IImageBitmap m_colorPipelineBitmap_;
    gige::IImageInfo m_imageInfo_;

    TimestampSynchronizer::Options defaultTimesyncOptions_;
    std::unique_ptr<TimestampSynchronizer> ptimestampSynchronizer_;

    bool m_defaultGainNotSet_; // currently not used, maybe necessary for auto-gain control
    double m_defaultGain_;
};

#endif //SMARTEK_CAMERA_NODE_H
