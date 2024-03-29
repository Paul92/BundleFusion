#pragma once

//#define KINECT
//#define KINECT_ONE
//#define OPEN_NI
#define BINARY_DUMP_READER
//#define INTEL_SENSOR
//#define REAL_SENSE
#define STRUCTURE_SENSOR
#define SENSOR_DATA_READER

//#define RUN_MULTITHREADED

#include "stdafx.h"

#include <vector>
#include <string>
#include <list>


#define RENDERMODE_INTEGRATE 0 
#define RENDERMODE_VIEW 1

#define X_GLOBAL_APP_STATE_FIELDS \
	X(UINT, s_sensorIdx) \
	X(UINT, s_windowWidth) \
	X(UINT, s_windowHeight) \
	X(UINT, s_integrationWidth) \
	X(UINT, s_integrationHeight) \
	X(UINT, s_rayCastWidth) \
	X(UINT, s_rayCastHeight) \
	X(UINT, s_maxFrameFixes) \
	X(UINT, s_topNActive) \
	X(float, s_minPoseDistSqrt) \
	X(float, s_sensorDepthMax) \
	X(float, s_sensorDepthMin) \
	X(float, s_renderDepthMax) \
	X(float, s_renderDepthMin) \
	X(UINT, s_hashNumBuckets) \
	X(UINT, s_hashNumSDFBlocks) \
	X(UINT, s_hashMaxCollisionLinkedListSize) \
	X(float, s_SDFVoxelSize) \
	X(float, s_SDFMarchingCubeThreshFactor) \
	X(float, s_SDFTruncation) \
	X(float, s_SDFTruncationScale) \
	X(float, s_SDFMaxIntegrationDistance) \
	X(UINT, s_SDFIntegrationWeightSample) \
	X(UINT, s_SDFIntegrationWeightMax) \
	X(std::string, s_binaryDumpSensorFile) \
	X(bool, s_binaryDumpSensorUseTrajectory) \
	X(float, s_depthSigmaD) \
	X(float, s_depthSigmaR) \
	X(bool, s_depthFilter) \
	X(float, s_colorSigmaD) \
	X(float, s_colorSigmaR) \
	X(bool, s_colorFilter) \
	X(vec4f, s_materialAmbient) \
	X(vec4f, s_materialSpecular) \
	X(vec4f, s_materialDiffuse) \
	X(float, s_materialShininess) \
	X(vec4f, s_lightAmbient ) \
	X(vec4f, s_lightDiffuse) \
	X(vec4f, s_lightSpecular) \
	X(vec3f, s_lightDirection) \
	X(float, s_SDFRayIncrementFactor) \
	X(float, s_SDFRayThresSampleDistFactor) \
	X(float, s_SDFRayThresDistFactor) \
	X(bool, s_integrationEnabled) \
	X(bool, s_trackingEnabled) \
	X(bool, s_garbageCollectionEnabled) \
	X(UINT, s_garbageCollectionStarve) \
	X(bool, s_SDFUseGradients) \
	X(bool, s_timingsDetailledEnabled) \
	X(bool, s_timingsTotalEnabled) \
	X(UINT, s_RenderMode) \
	X(bool, s_playData) \
	X(float, s_renderingDepthDiscontinuityThresLin) \
	X(float, s_remappingDepthDiscontinuityThresLin) \
	X(float, s_remappingDepthDiscontinuityThresOffset) \
	X(float, s_renderingDepthDiscontinuityThresOffset) \
	X(bool, s_bUseCameraCalibration) \
	X(UINT, s_marchingCubesMaxNumTriangles) \
	X(bool, s_streamingEnabled) \
	X(vec3f, s_streamingVoxelExtents) \
	X(vec3i, s_streamingGridDimensions) \
	X(vec3i, s_streamingMinGridPos) \
	X(UINT, s_streamingInitialChunkListSize) \
	X(float, s_streamingRadius) \
	X(vec3f, s_streamingPos) \
	X(UINT, s_streamingOutParts) \
	X(UINT, s_recordDataWidth) \
	X(UINT, s_recordDataHeight) \
	X(bool, s_recordData) \
	X(bool, s_recordCompression) \
	X(std::string, s_recordDataFile) \
	X(bool, s_reconstructionEnabled) \
	X(bool, s_generateVideo) \
	X(std::string, s_generateVideoDir) \
	X(std::string, s_printTimingsDirectory) \
	X(std::string, s_printConvergenceFile) \
	X(mat4f, s_topVideoTransformWorld) \
	X(vec4f, s_topVideoCameraPose) \
	X(vec2f, s_topVideoMinMax) \
	X(UINT, s_numSolveFramesBeforeExit)


#ifndef VAR_NAME
#define VAR_NAME(x) #x
#endif

#define checkSizeArray(a, d)( (((sizeof a)/(sizeof a[0])) >= d))

class GlobalAppState
{
public:

#define X(type, name) type name;
	X_GLOBAL_APP_STATE_FIELDS
#undef X

		//! sets the parameter file and reads
	void readMembers(const ParameterFile& parameterFile) {
		m_ParameterFile = parameterFile;
		readMembers();
	}

	//! reads all the members from the given parameter file (could be called for reloading)
	void readMembers() {
#define X(type, name) \
	if (!m_ParameterFile.readParameter(std::string(#name), name)) {MLIB_WARNING(std::string(#name).append(" ").append("uninitialized"));	name = type();}
		X_GLOBAL_APP_STATE_FIELDS
#undef X
 

		m_bIsInitialized = true;
	}

	void print() const {
#define X(type, name) \
	std::cout << #name " = " << name << std::endl;
		X_GLOBAL_APP_STATE_FIELDS
#undef X
	}

	static GlobalAppState& getInstance() {
		static GlobalAppState s;
		return s;
	}
	static GlobalAppState& get() {
		return getInstance();
	}


	//! constructor
	GlobalAppState() {
		m_bIsInitialized = false;
#ifdef _WIN32
		m_pQuery = NULL;
#endif
	}

	//! destructor
	~GlobalAppState() {
	}


#ifdef _WIN32
	HRESULT OnD3D11CreateDevice(ID3D11Device* pd3dDevice);
	void OnD3D11DestroyDevice();
#endif

	void WaitForGPU();

	Timer	s_Timer;

private:
	bool			m_bIsInitialized;
	ParameterFile	m_ParameterFile;
#ifdef _WIN32
	ID3D11Query*	m_pQuery;
#endif
};
