/**
 * @class Grabber
 * @brief A Class that fetch an image from a camera.
 *
 * @details
 * Contact: carek.marko@gmail.com
 *
 * @author Marko Car
 * @version 0.1
 * @date July 2018
 *
 */

#ifndef OBJECT_DETECTION_GRABBER_H
#define OBJECT_DETECTION_GRABBER_H

#include "smcs_cpp/CameraSDK.h"
#include "smcs_cpp/IImageBitmap.h"
#include <string>

class Grabber {
    private:
        float *dataFromImageBitmap(const smcs::IImageBitmapInterface src, int &w, int &h, int &c);
        std::string IpAddrToString(uint32_t ipAddress);

        smcs::ICameraAPI smcs_api_;
        smcs::IImageProcAPI image_proc_api_;
        smcs::DevicesList devices_;
        smcs::IAlgorithm color_pipeline_alg_;
        smcs::IParams color_pipeline_params_;
        smcs::IResults color_pipeline_results_;
        smcs::IImageBitmap color_pipeline_bitmap_;

    public:
        /**
         * @brief A constructor.
         * It initialize CameraSuite and all the necessary variables.
         *
         */
        Grabber(void);

        /**
         * @brief A destructor.
         * Frees all used memory.
         */
        ~Grabber(void);

        /**
         * @brief Returns number of cameras detected on the network.
         *
         * @return number of cameras on the network.
         */
        int getNumberOfDevices(void);

        /**
         * @brief Search for all cameras on the network.
         *
         * @return number of cameras on the network.
         */
        int findDevices(void);

        /**
         * @brief Connects to the requested device.
         *
         * @return connection status. Returns 1 if connection is successfull.
         */
        int connect(int device_num);

        /**
         * @brief disconects the camera with given device number.
         *
         * @param device_num number of device which should be disconected
         */
        void disconect(int device_num);


        /**
         * @brief grabs an image.
         *
         * @param device_num number of device from which image should be grabbed.
         * @param w address to variable where width of the image should be stored.
         * @param h address to variable where height of the image should be stored.
         * @param c address to variable where number of channels of the image should be stored.
         * @return pointer to data array.
         */
        float *grab(int device_num, int &w, int &h, int &c);
};


#endif //OBJECT_DETECTION_GRABBER_H
