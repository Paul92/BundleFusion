#include "SLAMBenchSensor.h"

#include "../../../framework/shared/include/io/FrameBuffer.h"
#include "../../../framework/shared/include/io/SLAMFrame.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "SiftVisualization.h"
#include "ImageHelper.h"



SLAMBenchSensor::SLAMBenchSensor(slambench::io::DepthSensor *depth_sensor,
                                 slambench::io::CameraSensor *rgb_sensor) {
    this->depth_sensor = depth_sensor;
    this->rgb_sensor = rgb_sensor;

    this->depthFrame = NULL;
    this->rgbFrame = NULL;
}

SLAMBenchSensor::~SLAMBenchSensor() {
}



void SLAMBenchSensor::createFirstConnected() {

    RGBDSensor::init(depth_sensor->Width, depth_sensor->Height,
                     rgb_sensor->Width, rgb_sensor->Height); 

    initializeDepthIntrinsics(depth_sensor->Intrinsics[0] * depth_sensor->Width,
                              depth_sensor->Intrinsics[1] * depth_sensor->Height,
                              depth_sensor->Intrinsics[2] * depth_sensor->Width,
                              depth_sensor->Intrinsics[3] * depth_sensor->Height);

    initializeColorIntrinsics(rgb_sensor->Intrinsics[0] * rgb_sensor->Width,
                              rgb_sensor->Intrinsics[1] * rgb_sensor->Height,
                              rgb_sensor->Intrinsics[2] * rgb_sensor->Width,
                              rgb_sensor->Intrinsics[3] * rgb_sensor->Height);

}

void SLAMBenchSensor::pushRGBFrame(slambench::io::SLAMFrame *frame) {

    if (!rgbFrame)
        rgbFrame = new FrameData(frame->GetSize());

    memcpy(rgbFrame->data, frame->GetData(), rgbFrame->size);

}

void SLAMBenchSensor::pushDepthFrame(slambench::io::SLAMFrame *frame) {

    size_t noOfPixels = depth_sensor->Height * depth_sensor->Width;

    if (!depthFrame) 
        depthFrame = new FrameData(noOfPixels * sizeof(float));


    short int *data = (short int*)frame->GetData();
    float *depth = (float*)depthFrame->data;

    for (int i = 0; i < noOfPixels; i++) {
        if (data[i] != 0) {
            depth[i] = data[i] / 1000.;
        } else {
            depth[i] = -std::numeric_limits<float>::infinity();
        }
    }

}

bool SLAMBenchSensor::processDepth() {

    float* depth = getDepthFloat();
    size_t size = depthFrame->size;
    size_t pixels = depth_sensor->Height * depth_sensor->Width;

  //  short int *data = (short int*)(depthFrame->data);

    memcpy(depth, depthFrame->data, size);

//    cv::Mat mat(depth_sensor->Height, depth_sensor->Width, CV_8U);
//    for (int i = 0; i < pixels; i++) {
//        if (depth[i] == std::numeric_limits<float>::infinity()) {
//            mat.at<unsigned char>(i) = 255;
//        } else if (depth[i] == -std::numeric_limits<float>::infinity()) {
//            mat.at<unsigned char>(i) = 0;
//        } else {
//            mat.at<unsigned char>(i) = 127;
//        }
//    }
//    cv::namedWindow("cacamaca");
//    cv::imshow("cacamaca", mat);
//
//    auto cvDepth = cv::Mat(depth_sensor->Height, depth_sensor->Width, CV_32FC1, depthFrame->data);
//    
//    double min, max;
//    // TODO: can get rid of this after analysing the input format
//    cv::minMaxLoc(cvDepth, &min, &max);
//
//    cv::Mat charImage;
//    cvDepth -= min;
//    cvDepth.cv::Mat::convertTo(charImage, CV_8U, 255.0/(max-min));
//
//    cv::namedWindow("cacamacaca");
//    cv::imshow("cacamacaca", charImage);
//    cv::waitKey();

    return true;
}

bool SLAMBenchSensor::processColor() {

    size_t size = rgbFrame->size;

    vec4uc *colorData = getColorRGBX();

    unsigned char *data = (unsigned char *)rgbFrame->data;

    for (int i = 0; i < size; i += 3) {
            colorData[i / 3] = vec4uc(data[i], data[i + 1], data[i + 2], 255);
    }

    return true;
}

bool SLAMBenchSensor::depth_ready() {
    return this->depthFrame != NULL;
}

bool SLAMBenchSensor::rgb_ready() {
    return this->rgbFrame != NULL;
}


std::string SLAMBenchSensor::getSensorName() const {
    return "SLAMBENCH_SENSOR";
}

void *SLAMBenchSensor::getRGBData() const {
    return rgbFrame->data;
}

void *SLAMBenchSensor::getDepthData() const {
    return depthFrame->data;
}

