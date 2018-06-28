#include <SLAMBenchAPI.h>

#include <io/SLAMFrame.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/CameraSensor.h>
#include <values/Value.h>

#include <string>
#include <algorithm>

#include "mLib.h"
#include "GlobalAppState.h"
#include "GlobalBundlingState.h"
#include "SLAMBenchSensor.h"
#include "DualGPU.h"
#include "ConditionManager.h"
#include "CUDAImageManager.h"
#include "DepthSensing/DepthSensing.h"

slambench::outputs::Output *pose_output = nullptr;
slambench::outputs::Output *rgb_frame_output = nullptr;
slambench::outputs::Output *depth_frame_output = nullptr;
slambench::outputs::Output *trajectory_output = nullptr;

const std::string defaultAlgorithmPath = "benchmarks/BundleFusion/src/original/FriedLiver/";
const std::string defaultAppConfigFile = defaultAlgorithmPath + "zParametersDefault.txt";
const std::string defaultBundlingConfigFile = defaultAlgorithmPath + "zParametersBundlingDefault.txt";

uchar *renderedDepthFrame = nullptr;
constexpr float MAX_DEPTH_VALUE = 6;

bool sb_new_slam_configuration(SLAMBenchLibraryHelper *slam_settings) {
    try {
        std::string appConfigFile;
        std::string bundlingConfigFile;

        slam_settings->addParameter(TypedParameter<std::string>("app", "app_config_file", "Application config file", &appConfigFile, &defaultAppConfigFile));
        slam_settings->addParameter(TypedParameter<std::string>("bund", "bundling_config_file", "Bundling config file", &bundlingConfigFile, &defaultBundlingConfigFile));

        ParameterFile parameterFileApp(appConfigFile);
        GlobalAppState::getInstance().readMembers(parameterFileApp);

        //Read the global camera tracking state
        ParameterFile parameterFileBundling(bundlingConfigFile);
        GlobalBundlingState::getInstance().readMembers(parameterFileBundling);
    } catch (const std::exception& e) {
        std::cout << "Caught exception: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    return true;
}

slambench::io::DepthSensor *depth_sensor;
slambench::io::CameraSensor *rgb_sensor;

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings) {

    // Intializing sensor
    std::cout << "Initializing sensor" << std::endl;

    for(const auto &sensor : slam_settings->get_sensors()) {
        if (sensor->GetType() == "Camera" && !rgb_sensor) {
            rgb_sensor = (slambench::io::CameraSensor*)sensor;	
        } else if(sensor->GetType() == "Depth") {
            depth_sensor = (slambench::io::DepthSensor*)sensor;
        }
        std::cout << "Got sensor of type " << sensor->GetType() << std::endl;
    }

    std::cout << "[OK] Initializing sensor" << std::endl;

    SLAMBenchSensor *slambench_sensor = new SLAMBenchSensor(depth_sensor, rgb_sensor);

    std::cout << "Performing sanity checks" << std::endl;

    std::cout << "SB sensor " << slambench_sensor << std::endl;
    std::cout << "rgb_sensor " << rgb_sensor << std::endl;

    std::cout << "  Rgb sensor  file format " << rgb_sensor->FrameFormat << " ... ";
    if(rgb_sensor->FrameFormat == slambench::io::frameformat::Raster) {
        std::cout << "[OK]" << std::endl;
    } else {
        std::cout << "[FAIL] - expected " << slambench::io::frameformat::Raster << "(raster)" << std::endl;
    }

    std::cout << "Depth sensor  file format " << depth_sensor->FrameFormat << " ... ";
    if(depth_sensor->FrameFormat == slambench::io::frameformat::Raster) {
        std::cout << "[OK]" << std::endl;
    } else {
        std::cout << "[FAIL] - expected " << slambench::io::frameformat::Raster << "(raster)" << std::endl;
    }

    std::cout << "  Rgb sensor pixel format " << rgb_sensor->PixelFormat << " ... ";
    if(rgb_sensor->PixelFormat == slambench::io::pixelformat::RGB_III_888) {
        std::cout << "[OK]" << std::endl;
    } else {
        std::cout << "[FAIL] - expected " << slambench::io::pixelformat::RGB_III_888 << "(RGB_III_888)" << std::endl;
    }

    std::cout << "Depth sensor pixel format " << depth_sensor->PixelFormat << " ... ";
    if(depth_sensor->PixelFormat == slambench::io::pixelformat::D_I_16) {
        std::cout << "[OK]" << std::endl;
    } else {
        std::cout << "[FAIL] - expected " << slambench::io::pixelformat::D_F_32 << "(D_I_16)" << std::endl;
    }

    std::cout << "Sensor size match ...           ";
    if(rgb_sensor->Width != depth_sensor->Width || rgb_sensor->Height != depth_sensor->Height) {
        std::cout << "[FAIL]" << std::endl;
    } else {
        std::cout << "[OK]" << std::endl;
    }

    std::cout << "Initializing slam system" << std::endl;

    DualGPU& dualGPU = DualGPU::get();	//needs to be called to initialize devices
    dualGPU.setDevice(DualGPU::DEVICE_RECONSTRUCTION);	//main gpu
    ConditionManager::init();

    std::cout << "Connecting to sensor..." << std::endl;

    slambench_sensor->createFirstConnected();

    std::cout << "creating image manager" << std::endl;

    CUDAImageManager* g_imageManager = new CUDAImageManager(GlobalAppState::get().s_integrationWidth,
                                          GlobalAppState::get().s_integrationHeight,
                                          GlobalBundlingState::get().s_widthSIFT,
                                          GlobalBundlingState::get().s_heightSIFT,
                                          slambench_sensor, false);



#ifdef RUN_MULTITHREADED
//    std::cout << "Running multithreaded" << std::endl;
//    bundlingThread = std::thread(bundlingThreadFunc);
//
//    while (!g_bundler)
//        sched_yield();
//
//
//    // TODO: set g_depthSensingBundler
    throw "Multithreading not supported";
#else
    std::cout << "Running singlethreaded" << std::endl;
    OnlineBundler* g_bundler = new OnlineBundler(slambench_sensor, g_imageManager);
#endif


    startDepthSensing(g_bundler, slambench_sensor, g_imageManager);


    dualGPU.setDevice(DualGPU::DEVICE_RECONSTRUCTION);	//main gpu

    setup();

    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    pose_output->SetActive(true);

    rgb_frame_output = new slambench::outputs::Output("RGB Frame", slambench::values::VT_FRAME);
    rgb_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(rgb_frame_output);
    rgb_frame_output->SetActive(true);

    depth_frame_output = new slambench::outputs::Output("Depth Frame", slambench::values::VT_FRAME);
    depth_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(depth_frame_output);
    depth_frame_output->SetActive(true);

    trajectory_output = new slambench::outputs::Output("Trajectory", slambench::values::VT_TRAJECTORY, true);
    trajectory_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(trajectory_output);
    trajectory_output->SetActive(true);

    renderedDepthFrame = new uchar[depth_sensor->Width * depth_sensor->Height * sizeof(uchar)];

    return true;
}

bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame* s) {
    assert(s != nullptr);

    SLAMBenchSensor *sensor = static_cast<SLAMBenchSensor*>(getRunningSensor());

    if(s->FrameSensor == depth_sensor) {
        sensor->pushDepthFrame(s);
    } else if(s->FrameSensor == rgb_sensor) {
        sensor->pushRGBFrame(s);
    } 

    return sensor->depth_ready() && sensor->rgb_ready();
}

bool sb_process_once(SLAMBenchLibraryHelper * slam_settings) {
    frameRender();
    return true;
}

inline Eigen::Matrix4f matToEigen(const mat4f &mat) {
    Eigen::Matrix4f target;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            (target)(i, j) = mat[i*4+j];
    return target;
}

inline Eigen::Matrix4f matToEigen2(const mat4f &mat) {
        Eigen::Matrix4f drift;
    static int v = 0;
    v++;
drift << 0, 0, 0, 0.00001*v,
     0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 0;

    Eigen::Matrix4f target;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            (target)(i, j) = mat[i*4+j];

    return target + drift;
}

void getLastTransform(Eigen::Matrix4f *mat) {
    auto lastRigidTransform = getLastRigidTransform();
    *mat = matToEigen(lastRigidTransform);
}

bool sb_get_pose(Eigen::Matrix4f *mat) {
    getLastTransform(mat);
    return true;
}

bool sb_get_tracked(bool* tracked) {
    *tracked = !isTrackingLost();
    return true;
}

bool sb_clean_slam_system() {
    StopScanningAndExit(false);

// TODO: Take care of the multithreading case

    destroy();

    delete pose_output;
    delete rgb_frame_output;
    delete depth_frame_output;

    delete renderedDepthFrame;

    return true;
}

void renderDepthFrame(uchar *renderedDepthFrame, const float *depthFrame) {
    for (int i = 0; i < depth_sensor->Width * depth_sensor->Height; i++) {
        float pxValue = std::max<float>(0, std::min<float>(depthFrame[i], MAX_DEPTH_VALUE));
        int renderedValue = 255 / MAX_DEPTH_VALUE * pxValue;
        renderedDepthFrame[i] = renderedValue;
    }
}

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *latest_output) {

    slambench::TimeStamp ts = *latest_output;

    if (pose_output->IsActive()) {
        Eigen::Matrix4f transform;
        getLastTransform(&transform);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pose_output->AddPoint(ts, new slambench::values::PoseValue(transform));
    }

    if (rgb_frame_output->IsActive()) {
        SLAMBenchSensor *sensor = static_cast<SLAMBenchSensor*>(getRunningSensor());

        void *colorFrame = sensor->getRGBData();

        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());

        rgb_frame_output->AddPoint(*latest_output,
                        new slambench::values::FrameValue(rgb_sensor->Width, rgb_sensor->Height,
                        slambench::io::pixelformat::EPixelFormat::RGB_III_888 , colorFrame));
    }

    if (depth_frame_output->IsActive()) {
        SLAMBenchSensor *sensor = static_cast<SLAMBenchSensor*>(getRunningSensor());

        void *depthFrame = sensor->getDepthData();

        renderDepthFrame(renderedDepthFrame, static_cast<float*>(depthFrame));

        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        depth_frame_output->AddPoint(*latest_output,
                        new slambench::values::FrameValue(depth_sensor->Width, depth_sensor->Height,
                        slambench::io::pixelformat::EPixelFormat::G_I_8, renderedDepthFrame));
    }

    if (trajectory_output->IsActive()) {
        std::vector<mat4f> initialTrajectory = getTrajectory();
        slambench::values::Trajectory trajectory;
        for (const auto &pose : initialTrajectory)
            trajectory.push_back(ts, matToEigen(pose));

        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());

        auto trajectoryValue = new slambench::values::TrajectoryValue(trajectory);
        trajectory_output->AddPoint(*latest_output, trajectoryValue);
    }

    return true;
}

