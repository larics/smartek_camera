//
// Created by suiauthon on 30.03.18..
//

#include "Grabber.h"
#include <sstream>

void SMCS_CALL smcs_ICallbackEvent_Handler(smcs_ICameraAPI_HANDLE hApi, smcs_IDevice_HANDLE hDevice,
                                           UINT32 eventType, const smcs_CallbackInfo* eventInfo)
{
    // Warning! Callback handler is called in context of API thread and for real GUI app need to be synchronised to GUI thread.
    if (eventType == smcs_GCT_CONNECT) {
        printf("Callback, Connected \n");
    }
    else if (eventType == smcs_GCT_DISCONNECT) {
        printf("Callback, Disconnected \n");
    }
    else if (eventType == smcs_GCT_LOG) {
        printf("Callback, Log: %s \n", eventInfo->messageString);
    }
}

Grabber::Grabber(void) {

    smcs::InitCameraAPI();
    smcs::InitImageProcAPI();

    smcs_api_ = smcs::GetCameraAPI();
    image_proc_api_ = smcs::GetImageProcAPI();

    smcs_api_->SetHeartbeatTime(3.0);

    if (!smcs_api_->IsUsingKernelDriver()) {
        printf("Warning: Smartek Filter Driver not loaded. \n");
    }

    smcs_api_->SetHeartbeatTime(3);
    smcs_ICameraAPI_RegisterCallback(smcs_api_, smcs_ICallbackEvent_Handler);

    color_pipeline_alg_ = image_proc_api_->GetAlgorithmByName("ColorPipeline");
    color_pipeline_alg_->CreateParams(&color_pipeline_params_);
    color_pipeline_alg_->CreateResults(&color_pipeline_results_);
    image_proc_api_->CreateBitmap(&color_pipeline_bitmap_);
}

Grabber::~Grabber(void) {
    color_pipeline_alg_->DestroyParams(color_pipeline_params_);
    color_pipeline_alg_->DestroyResults(color_pipeline_results_);
    image_proc_api_->DestroyBitmap(color_pipeline_bitmap_);

    smcs::ExitImageProcAPI();
    smcs::ExitCameraAPI();
}

int Grabber::findDevices(void) {
    // discover all devices on network
    smcs_api_->FindAllDevices(3.0);
    devices_ = smcs_api_->GetAllDevices();

    return devices_.size();
}

int Grabber::getNumberOfDevices(void) {
    return devices_.size();
}

int Grabber::getDeviceBySerialNumber(std::string serial_number) {
    int i, device_num = -1;
    std::string serial;

    for (i = 0; i < devices_.size(); i++) {
        serial = devices_[i]->GetSerialNumber();
        printf("Serial: %s\n", serial);
        if (!serial.compare(serial_number)) device_num = i;
    }

    return device_num;
}

int Grabber::connect(int device_num) {
    std::string text;
    INT64 int64Value;
    smcs::IDevice connected_device;
    int is_connected = 0;

    if (devices_.size() <= device_num) return is_connected;

    connected_device = devices_[device_num];

    if (connected_device != NULL && connected_device->Connect()) {
        is_connected = 1;
        printf("Connected to the camera: %s %d \n", IpAddrToString(connected_device->GetIpAddress()).c_str(),
               connected_device->GetIpAddress());
        text = connected_device->GetSerialNumber();
        printf("SerialNumber: %s \n", text.c_str());
        if (connected_device->GetStringNodeValue("DeviceVendorName", text)) {
            printf("Device Vendor: %s \n", text.c_str());
        }
        if (connected_device->GetStringNodeValue("DeviceModelName", text)) {
            printf("Device Model: %s \n", text.c_str());
        }
        if (connected_device->GetIntegerNodeValue("Width", int64Value)) {
            printf("Width: %d \n", (int)int64Value);
        }
        if (connected_device->GetIntegerNodeValue("Height", int64Value)) {
            printf("Height: %d \n", (int)int64Value);
        }

        // disable trigger mode
        connected_device->SetStringNodeValue("TriggerMode", "Off");
        // set continuous acquisition mode
        connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");
        connected_device->SetIntegerNodeValue("ImageBufferFrameCount", 1);
        // start acquisition
        connected_device->SetIntegerNodeValue("TLParamsLocked", 1);
        connected_device->CommandNodeExecute("AcquisitionStart");

    } else {
        is_connected = 0;
        printf("Cannot connect to the requested device: %d", device_num);
    }

    return is_connected;
}

void Grabber::disconect(int device_num) {
    if (devices_.size() <= device_num) return;

    devices_[device_num]->CommandNodeExecute("AcquisitionStop");
    devices_[device_num]->SetIntegerNodeValue("TLParamsLocked", 0);
    devices_[device_num]->Disconnect();
}

uint64_t Grabber::getCameraTimestamp(int device_num) {
    smcs::IDevice connected_device;
    smcs::IImageInfo image_info;
    uint64_t camera_timestamp = 0;


    if (devices_.size() <= device_num) return camera_timestamp;

    connected_device = devices_[device_num];

    if (connected_device.IsValid() && connected_device->IsConnected()) {
        if (!connected_device->IsBufferEmpty()) {
            connected_device->GetImageInfo(&image_info);
            camera_timestamp = image_info->GetCameraTimestamp();
        }
    }

    return camera_timestamp;
}

uint32_t Grabber::getImageID(int device_num) {
    smcs::IDevice connected_device;
    smcs::IImageInfo image_info;
    uint32_t image_id = 0;


    if (devices_.size() <= device_num) return image_id;

    connected_device = devices_[device_num];

    if (connected_device.IsValid() && connected_device->IsConnected()) {
        if (!connected_device->IsBufferEmpty()) {
            connected_device->GetImageInfo(&image_info);
            image_id = image_info->GetImageID();
        }
    }

    return image_id;
}

uint8_t *Grabber::grab(int device_num, int &w, int &h, int &c) {
    smcs::IDevice connected_device;
    smcs::IImageInfo image_info;
    UINT32 src_width, src_height;
    UINT32 src_pixel_type;
    uint8_t *im = NULL;

    if (devices_.size() <= device_num) return im;

    connected_device = devices_[device_num];

    if (connected_device.IsValid() && connected_device->IsConnected()) {
        if (!connected_device->IsBufferEmpty()) {
            connected_device->GetImageInfo(&image_info);
            if (image_info != NULL) {
                // white balance parameters
                //color_pipeline_params_->SetBooleanNodeValue("EnableWhiteBalance", true);

                //color_pipeline_params_->SetIntegerNodeValue("PixelDecimation", 8);

                // rgb gain parameters
                //color_pipeline_params_->SetFloatNodeValue("RedGain", 1.0);
                //color_pipeline_params_->SetFloatNodeValue("GreenGain", 1.0);
                //color_pipeline_params_->SetFloatNodeValue("BlueGain", 1.0);

                // rgb gamma parameters
                //color_pipeline_params_->SetFloatNodeValue("RedGamma", 1.0);
                //color_pipeline_params_->SetFloatNodeValue("GreenGamma", 1.0);
                //color_pipeline_params_->SetFloatNodeValue("BlueGamma", 1.0);

                // demosaic parameters
                // border type, default smcs_IPBT_BILINEAR_BORDER
                //color_pipeline_params_->SetIntegerNodeValue("BorderTypeDemosaic", smcs_IPBT_BILINEAR_BORDER);
                //color_pipeline_params_->SetIntegerNodeValue("BorderTypeDemosaic", smcs_IPBT_COLORIZED_BORDER);
                //color_pipeline_params_->SetIntegerNodeValue("BorderTypeDemosaic", smcs_IPBT_BLACK_BORDER);
                //color_pipeline_params_->SetIntegerNodeValue("BorderTypeDemosaic", smcs_IPBT_CROP_BORDER);
                //color_pipeline_params_->SetBooleanNodeValue("EnableDemosaic", true);
                // demosaic type, default Bilinear
                //color_pipeline_params_->SetStringNodeValue("DemosaicType", "Bilinear");
               // color_pipeline_params_->SetStringNodeValue("DemosaicType", "HQLinear");
                //color_pipeline_params_->SetStringNodeValue("DemosaicType", "PixelGroup");

                // color correction parameters
                // color correction is not enabled by default
                //color_pipeline_params_->SetBooleanNodeValue("EnableColorCorrection", false);
                // type of color correction, default Matrix3x3RGB
                //color_pipeline_params_->SetStringNodeValue("ColorCorrectionType", "Matrix3x3RGB");
                //color_pipeline_params_->SetStringNodeValue("ColorCorrectionType", "ColorGimp");

                // parameters for Matrix3x3RGB
                // example of Matrix3x3RGB
                //double matrix3x3RGB[3][3];
                //matrix3x3RGB[0][0] = 1.0;
                //matrix3x3RGB[0][1] = 0.0;
                //matrix3x3RGB[0][2] = 0.0;
                //matrix3x3RGB[1][0] = 0.0;
                //matrix3x3RGB[1][1] = 1.0;
                //matrix3x3RGB[1][2] = 0.0;
                //matrix3x3RGB[2][0] = 0.0;
                //matrix3x3RGB[2][1] = 0.0;
                //matrix3x3RGB[2][2] = 1.0;
                //for (int i = 0; i < 3; i++)
                //  for (int j = 0; j < 3; j++) {
                //      //matrix3x3RGB[i][j] = (i == j) ? 1.0 : 0.0;    // set matrix to do nothing
                //      std::stringstream matrix;
                //      matrix << "Matrix" << i << j;
                //      m_matrixParams->SetFloatNodeValue(matrix.str(), paramsImp->matrix3x3RGB[i][j]);
                //  }
                // parameters for color gimp
                // example of color gimp
                //std::string color[7] = {"All","Red","Yellow","Green","Cyan","Blue","Magenta"};
                //for (int j = 0; j < 7; j++) {
                //      std::stringstream hue;
                //      hue << "Hue" << color[j];
                //      m_colorPipelineParams->SetFloatNodeValue(hue.str(), 20);                // min -180 max 180     default 0
                //      std::stringstream saturation;
                //      saturation << "Saturation" << color[j];
                //      m_colorPipelineParams->SetFloatNodeValue(saturation.str(), 40);         // min -100 max 100     default 0
                //      std::stringstream lightness;
                //      lightness << "Lightness" << color[j];
                //      m_colorPipelineParams->SetFloatNodeValue(lightness.str(), 15);          // min -100 max 100     default 0
                //}
                //m_colorPipelineParams->SetFloatNodeValue("Overlay", 10);                      // min 0 max 100        default 0

                // sharpen parameters
                // sharpen is not enabled by default
                //m_colorPipelineParams->SetFloatNodeValue("SharpenFactor", 0.5);
                // border type, default smcs_IPBT_COPY_BORDER
                //m_colorPipelineParams->SetIntegerNodeValue("BorderTypeSharpen", smcs_IPBT_COPY_BORDER);
                //m_colorPipelineParams->SetIntegerNodeValue("BorderTypeSharpen", smcs_IPBT_BLACK_BORDER);
                //m_colorPipelineParams->SetIntegerNodeValue("BorderTypeSharpen", smcs_IPBT_CROP_BORDER);
                //m_colorPipelineParams->SetBooleanNodeValue("EnableSharpen", true);

                // paremeters for autoexposure
                // autoexposure is not enabled by default
                //double oldExposureTime, oldGain;
                //m_device->GetFloatNodeValue("ExposureTime", oldExposureTime);
                //m_colorPipelineParams->SetFloatNodeValue("OldExposure", oldExposureTime);
                //if (m_defaultGainNotSet) {
                //  m_device->GetFloatNodeValue("Gain", m_defaultGain);
                //  m_colorPipelineParams->SetFloatNodeValue("DefaultGain", m_defaultGain);
                //  m_colorPipelineParams->SetFloatNodeValue("OldGain", m_defaultGain);
                //  oldGain = m_defaultGain;
                //  m_defaultGainNotSet = false;
                //} else {
                //  m_device->GetFloatNodeValue("Gain", oldGain);
                //  m_colorPipelineParams->SetFloatNodeValue("OldGain", oldGain);
                //}
                //UINT32 pixelType;
                //imageInfo->GetPixelType(pixelType);
                //UINT32 bitsPerPixel = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)pixelType);
                //UINT32 targetPixelAverage = pow(2.0, (double)bitsPerPixel)/2;
                //m_colorPipelineParams->SetFloatNodeValue("TargetPixelAverage", targetPixelAverage);
                //m_colorPipelineParams->SetFloatNodeValue("MinExposure", 200);
                //m_colorPipelineParams->SetFloatNodeValue("MaxExposure", 200000);
                //m_colorPipelineParams->SetFloatNodeValue("ExposureTreshold", 12);
                //m_colorPipelineParams->SetFloatNodeValue("MaxGainOffset", 20);
                //m_colorPipelineParams->SetBooleanNodeValue("EnableAutoExposure", true);

                image_proc_api_->ExecuteAlgorithm(color_pipeline_alg_, image_info,
                                                  color_pipeline_bitmap_, color_pipeline_params_,
                                                  color_pipeline_results_);

                //rezultati su spremljeni u color_pipeline_results_
                // color_pipeline_results_->GetFloatNodeValue("RedGain", redGain);

                im = smcs::IImageBitmapInterface(color_pipeline_bitmap_).GetRawData();
                smcs::IImageBitmapInterface(color_pipeline_bitmap_).GetPixelType(src_pixel_type);
                smcs::IImageBitmapInterface(color_pipeline_bitmap_).GetSize(src_width, src_height);
                w = src_width;
                h = src_height;
                c = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)src_pixel_type) / 8;
                //im = dataFromImageBitmap(color_pipeline_bitmap_, w, h, c);
            }
            connected_device->PopImage(image_info);
            connected_device->ClearImageBuffer();
        }
    }

    return im;
}

float *Grabber::dataFromImageBitmap(const smcs::IImageBitmapInterface src, int &w, int &h, int &c) {
    UINT32 src_pixel_type;
    UINT32 src_width, src_height;
    const UINT8 *raw_data;
    float *data = NULL;
    int i, j, k, color_index = 0;
    int index = 0;

    src.GetPixelType(src_pixel_type);
    src.GetSize(src_width, src_height);

    if (src_pixel_type == GVSP_PIX_MONO8) {
        w = src_width;
        h = src_height;
        c = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)src_pixel_type) / 8;

        data = (float *)calloc(w*h*c, sizeof(float));
    }
    else if (src_pixel_type == GVSP_PIX_BGRA8_PACKED) {
        w = src_width;
        h = src_height;
        c = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES)src_pixel_type) / 8;

        data = (float *)calloc(w*h*c, sizeof(float));
        raw_data = src.GetRawData();

        for (k = 0; k < c; k++) {
            color_index = (c - k + 2) % c;
            for (j = 0; j < h; j++) {
                for (i = 0; i < w; i++) {
                    index = i + w*j + w*h*k;
                    data[index] = (float)raw_data[color_index + (i + w*j) * c] / 255.0;
                }
            }
        }
    }

    return data;
}

std::string Grabber::IpAddrToString(UINT32 ipAddress) {
    std::stringstream stream;

    UINT32 temp1 = ((ipAddress >> 24) & 0xFF);
    UINT32 temp2 = ((ipAddress >> 16) & 0xFF);
    UINT32 temp3 = ((ipAddress >> 8) & 0xFF);
    UINT32 temp4 = ((ipAddress) & 0xFF);

    stream << temp1 << "." << temp2 << "." << temp3 << "." << temp4;

    return stream.str();
}
