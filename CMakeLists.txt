cmake_minimum_required(VERSION 2.6.0)

SET(appname bundlefusion)

SET(CUDA_SEPARABLE_COMPILATION ON)

set(CUTIL_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Include/cutil/inc")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

find_package(FreeImage REQUIRED)
find_package(OpenCV    REQUIRED)
find_package(Eigen3    REQUIRED)
find_package(CUDA      REQUIRED)
find_package(Threads   REQUIRED)

include_directories("external/mLib/include"
                    ${SLAMBENCH_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${CUDA_INCLUDE_DIRS}
                    ${CUTIL_HEADER}
                    ${OpenCV_INCLUDE_DIRS}

                    "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source"
                    "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU"

                    ${FreeImage_INCLUDE_DIRS}
                    #${SRCS}
                    #${DEPTH_SENSING}
                    #${SIFT_GPU}
                    #${SOLVER}
                    )

MESSAGE("FREEIMAGE PATH " ${FreeImage_INCLUDE_DIRS})

add_definitions(
   -D_DEBUG 
   -DDEBUG 
   -DPROFILE 
   -D_CONSOLE 
#   -DD3DXFX_LARGEADDRESS_HANDLE 
#   -D_CRT_SECURE_NO_WARNINGS 
   -DNOMINMAX 
   -DUNICODE
   -D_NICODE
   #-DUSE_LIE_SPACE
   #-DPRINT_RESIDUALS_SPARSE
   #-DPRINT_MEM_STATS

   #-DRUN_MULTITHREADED
   -DUSE_GLOBAL_DENSE_AT_END
   #-DCUSTOM_FEATURES
   #-DSIFT_GPU
   #-DEVALUATE_SPARSE_CORRESPONDENCES
)

set(CMAKE_BUILD_TYPE "Debug")
set(CUDA_PROPAGATE_HOST_FLAGS "NO")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wno-terminate -Wno-deprecated-declarations -Wno-unused-parameter -Wno-unused-function -Wno-parentheses -Wno-unused-variable -Wno-unknown-pragmas -Wno-sign-compare -Wno-type-limits -Wno-reorder -Wno-sequence-point")

set(CUDA_VERBOSE_BUILD ON)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "--std=c++11;-Xcompiler;-fPIC;")           
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false;") 
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode;arch=compute_61,code=sm_61;") 
set(CUDA_FILES "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/CUDAImageUtil.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/CUDACache.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/OnlineBundler.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDAConstant.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDASceneRepHashSDF.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDAMarchingCubesSDF.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDASceneRepChunkGrid.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/ScanCS.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDARayCastSDF.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CameraUtil.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/CUDASiftConstant.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/SIFTImageManager.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/ProgramCU.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SBA.cu"
               "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/Solver/SolverBundling.cu"
    )

#CUDA_INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source"
#                         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU"
#                         "external/mLib/include"
#                         ${CUTIL_HEADER})
#
#
#CUDA_COMPILE(cuda_objs ${CUDA_FILES})

set(SRCS "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/ConditionManager.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/RGBDSensor.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SensorDataReader.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/CorrespondenceEvaluator.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/CUDAImageManager.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/CUDACache.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/TimingLog.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/DepthSensing.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDAScan.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDAMarchingCubesHashSDF.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDASceneRepChunkGrid.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDARayCastSDF.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/TrajectoryManager.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/OnlineBundler.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/Bundler.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftVisualization.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/OnlineBundler.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/CUDASceneRepHashSDF.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/DepthSensing/TimingLogDepthSensing.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/mLib.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/SiftGPU.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/GlobalUtil.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/SIFTImageManager.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/SiftPyramid.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/SiftMatch.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SBA.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/GlobalAppState.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SiftGPU/CuTexImage.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/Solver/CUDASolverBundling.cpp"
         "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/SLAMBenchSensor.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/FriedLiver/Source/FeatureFinder.cpp"
         )

cuda_add_library(bundlefusion-original SHARED
    "bundlefusion_library.cpp"
    ${SRCS}
    ${CUDA_FILES}
)

target_link_libraries(bundlefusion-original
                     ${CUDA_LIBRARIES}
                     ${FreeImage_LIBRARY}
)

if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(bundlefusion-original PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(bundlefusion-original "${CMAKE_THREAD_LIBS_INIT}")
endif()

SET_TARGET_PROPERTIES(bundlefusion-original PROPERTIES COMPILE_FLAGS "-fPIC")

generate_slam_library(bundlefusion-original "")


