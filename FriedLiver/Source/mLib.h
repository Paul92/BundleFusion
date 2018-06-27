
//
// mLib config options
//

#define MLIB_ERROR_CHECK
#define MLIB_BOUNDS_CHECK
#define MLIB_SOCKETS

//
// mLib includes
//


#include "mLibCore.h"
#include "mLibFreeImage.h"
#include "mLibDepthCamera.h"
//#include "mLibZlib.h"
//#include "mLibFreeImage.h"
#include "mLibLodePNG.h"

#ifdef _WIN32
#include "mLibD3D11.h"
#endif

#include "mLibEigen.h"

//move this to mlib (it's currently local)
#include "mLibCuda.h"

using namespace ml;


