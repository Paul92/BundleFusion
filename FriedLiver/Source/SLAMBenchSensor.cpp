#include "SLAMBenchSensor.h"


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

    memcpy(depth, depthFrame->data, size);

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

