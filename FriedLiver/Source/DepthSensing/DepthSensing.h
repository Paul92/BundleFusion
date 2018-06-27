#pragma once

#include "RGBDSensor.h"
#include "TrajectoryManager.h"
#include "OnlineBundler.h"
#include "ConditionManager.h"

int startDepthSensing(OnlineBundler* bundler, RGBDSensor* sensor, CUDAImageManager* imageManager);

#ifndef _WIN32
void setup();
void destroy();
void frameRender();
void StopScanningAndExit(bool aborted);
#endif

RGBDSensor* getRunningSensor();
mat4f getLastRigidTransform();
bool isTrackingLost();
std::vector<mat4f> getTrajectory();
