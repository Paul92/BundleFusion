#pragma once

/************************************************************************/
/* Reads sensor data files from .sens files                            */
/************************************************************************/

#include "GlobalAppState.h"

#include <SLAMBenchAPI.h>

#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>

#include "RGBDSensor.h"
#include "stdafx.h"


class SLAMBenchSensor : public RGBDSensor
{
public:

    SLAMBenchSensor(slambench::io::DepthSensor *depth_sensor,
                    slambench::io::CameraSensor *rgb_sensor);

    ~SLAMBenchSensor();

    void createFirstConnected();

    bool processDepth();

    bool processColor();

    std::string getSensorName() const;

    virtual void pushRGBFrame(slambench::io::SLAMFrame *);
    virtual void pushDepthFrame(slambench::io::SLAMFrame *);

    virtual bool depth_ready();
    virtual bool rgb_ready();

    virtual void *getRGBData() const;
    virtual void *getDepthData() const;

private:

    slambench::io::DepthSensor *depth_sensor;
    slambench::io::CameraSensor *rgb_sensor;

class FrameData {
    public:
        FrameData(size_t size) {
            data = new char[size];
            this->size = size;
        }

        ~FrameData() {
            delete[] data;
        }

    char *data;
    size_t size;
};

    FrameData *rgbFrame;
    FrameData *depthFrame;
};


