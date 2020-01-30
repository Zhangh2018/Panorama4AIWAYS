/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2017 NXP
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
/*!*********************************************************************************
*  \file gftt_corners.cpp
*  \brief ApexCV Host wrapper for Good Feature To Track algorithm
***********************************************************************************/

#include <oal.h>
#include <apexcv_error_codes.hpp>
#include <apexcv_pro_gftt_corners.h>
#include <apexcv_pro_util.h>
#include <icp_feature.h>
#include <unistd.h>
#define MAX_CORNERS  4096

//#define ENABLE_PROFILE
#ifdef ENABLE_PROFILE
#include "common_time_measure.h"
#endif

// GENERATED ACF PROCESS
// box filter size 7, nms filter size 5
#include <GFTT_B7N5_COMPUTE.hpp>
#include <GFTT_B7N5_COMPUTE_EXTRACT.hpp>
#include <HARRIS_B7N5_COMPUTE.hpp>
#include <HARRIS_B7N5_COMPUTE_EXTRACT.hpp>
// box filter size 5, nms filter size 5
#include <GFTT_B5N5_COMPUTE.hpp>
#include <GFTT_B5N5_COMPUTE_EXTRACT.hpp>
#include <HARRIS_B5N5_COMPUTE.hpp>
#include <HARRIS_B5N5_COMPUTE_EXTRACT.hpp>
// box filter size 3, nms filter size 5
#include <GFTT_B3N5_COMPUTE.hpp>
#include <GFTT_B3N5_COMPUTE_EXTRACT.hpp>
#include <HARRIS_B3N5_COMPUTE.hpp>
#include <HARRIS_B3N5_COMPUTE_EXTRACT.hpp>




//#define DO_DEBUG_PRINT
#ifdef DO_DEBUG_PRINT
#define SHOW_ERROR(expr) if(!(expr)) \
 { \
   printf("%s : %s [%d]\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);\
 }
#else
#define SHOW_ERROR(expr)
#endif

#define LT(x, y)((x)>(y))
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
static int sort(short* vec, int* idx, int total);

namespace apexcv
{

GFTTCorners::GFTTCorners()
  : mState(init),
   mChunkWidth(0), mChunkHeight(0),
   mImageWidth(0), mImageHeight(0),
   mGFTTB7N5Compute(nullptr), mGFTTB5N5Compute(nullptr), mGFTTB3N5Compute(nullptr),
   mGFTTB7N5ComputeExtract(nullptr), mGFTTB5N5ComputeExtract(nullptr), mGFTTB3N5ComputeExtract(nullptr),
   mHARRISB7N5Compute(nullptr), mHARRISB5N5Compute(nullptr), mHARRISB3N5Compute(nullptr),
   mHARRISB7N5ComputeExtract(nullptr), mHARRISB5N5ComputeExtract(nullptr), mHARRISB3N5ComputeExtract(nullptr),
   mApuConfig(ACF_APU_CFG__DEFAULT),
   mApexId(0),
   mWeSelectedApuConfiguration(false),
   mBoxSize(BOX_SIZE),
   mNMSSize(NMS_SIZE),
   mSetChunkWidth(0),
   mSetACFSelectChunkSize(true)
{
}

GFTTCorners::~GFTTCorners()
{
  Release();
}

int GFTTCorners::Init_params(
          int aQualityLevel,
          int, // aMinDistance,
          int aBoxSize,
          int aNmsRadius,
          int aMaxCorners,
          int, // aUseHarrisDetector,
          int aHarrisK,
          int aHarrisThreshold)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  /* Harris parameter */
  if (mHarrisParams.empty()) {
    mHarrisParams = vsdk::SUMat(1, 7, VSDK_CV_32SC1);
  }
  if (!mHarrisParams.empty())
  {
    mBoxSize = aBoxSize;
    mNMSSize = aNmsRadius;
    vsdk::SMat lHarrisParamsMat = mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t* ptr = (int32_t*)lHarrisParamsMat.data;
    ptr[0] = aHarrisK; //Corner Coefficient
    ptr[1] = mBoxSize; // Box Size
    ptr[2] = mNMSSize; //nmsRadius
    //figure out shift values for the filters
    ptr[3] = DEFAULT_COVARIANCE;
    ptr[4] = (mBoxSize>>1) + (NMS_SIZE>>1);
    ptr[5] = aHarrisThreshold; // Threshold
    ptr[6] = aMaxCorners;
  }
  else
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  /* GFTT parameter */
  if (mGFTTParams.empty())
  {
    mGFTTParams = vsdk::SUMat(1, 6, VSDK_CV_32SC1);
  }
  if (!mGFTTParams.empty())
  {
    mBoxSize = aBoxSize;
    mNMSSize = aNmsRadius;
    vsdk::SMat lGFTTParamsMat = mGFTTParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t* ptr = (int32_t*)lGFTTParamsMat.data;
    ptr[0] = aQualityLevel; //quality Level
    ptr[1] = mBoxSize; // Box Size
    ptr[2] = mNMSSize; //nmsRadius
    //figure out shift values for the filters
    ptr[3] = DEFAULT_COVARIANCE;
    ptr[4] = (mBoxSize>>1) + (NMS_SIZE>>1);
    ptr[5] = aMaxCorners;
  }
  else
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  /* Final number of corners */
  if (mNumCorner.empty())
  {
    mNumCorner = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
  }
  if (mNumCorner.empty())
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  // everything pass, set return value to APEXCV_SUCCESS
  lRetVal = APEXCV_SUCCESS;

  EXIT:
  return lRetVal;
}

int GFTTCorners::Init_buffer(int aWidth, int aHeight)
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;

  switch (mState)
  {
    case HarrisAPEX :
      lRetVal = InitHarrisApex_buffer(aWidth, aHeight);
      break;
    case HarrisARM :
      lRetVal = InitHarrisArm_buffer(aWidth, aHeight);
      break;
    case GFTTARM :
      lRetVal = InitGfttArm_buffer(aWidth, aHeight);
      break;
    case GFTTAPEX :
      lRetVal = InitGfttApex_buffer(aWidth, aHeight);
      break;
    default:
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
  } /* switch (mState) */
  return lRetVal;
}

int GFTTCorners::InitHarrisArm_buffer(int aWidth, int aHeight)
{
  HARRIS_B7N5_COMPUTE *harrisB7N5Compute = (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
  HARRIS_B5N5_COMPUTE *harrisB5N5Compute = (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
  HARRIS_B3N5_COMPUTE *harrisB3N5Compute = (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;

  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
  /* Destination strength image buffer */
  mDstImg = vsdk::SUMat(aHeight, aWidth, VSDK_CV_16SC1);
  if (mDstImg.empty())
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  else
  {
    switch (mBoxSize)
    {
      case FLT_MASK_7:
      {
        lRetAcfVal = harrisB7N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_5:
      {
        lRetAcfVal = harrisB5N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_3:
      {
        lRetAcfVal = harrisB3N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    }
  } // else

  EXIT:
  return lRetVal;

}

int GFTTCorners::InitHarrisApex_buffer(int aWidth, int aHeight)
{
  HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
  HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
  HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;

  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
  /* Destination strength image buffer */
  mDstImg = vsdk::SUMat(aHeight, aWidth, VSDK_CV_16SC1);
  if (mDstImg.empty())
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  else
  {
    switch (mBoxSize)
    {
      case FLT_MASK_7:
      {
        lRetAcfVal = harrisB7N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_5:
      {
        lRetAcfVal = harrisB5N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_3:
      {
        lRetAcfVal = harrisB3N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } // else
  }

  EXIT:
  return lRetVal;

}

int GFTTCorners::InitGfttApex_buffer(int aWidth, int aHeight)
{
  GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
  GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
  GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;

  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
  /* Destination strength image buffer */
  mDstImg = vsdk::SUMat(aHeight, aWidth, VSDK_CV_16SC1);
  if (mDstImg.empty())
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  else
  {
    switch (mBoxSize)
    {
      case FLT_MASK_7:
      {
        lRetAcfVal =  gfttB7N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_5:
      { 
        lRetAcfVal =  gfttB5N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_3:
      {
        lRetAcfVal =  gfttB3N5ComputeExtract->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } // else
  }

  EXIT:
  return lRetVal;

}

int GFTTCorners::InitGfttArm_buffer(int aWidth, int aHeight)
{
  GFTT_B7N5_COMPUTE *gfttB7N5Compute = (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute; 
  GFTT_B5N5_COMPUTE *gfttB5N5Compute = (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute; 
  GFTT_B3N5_COMPUTE *gfttB3N5Compute = (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute; 

  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  /* Destination strength image buffer */
  mDstImg = vsdk::SUMat(aHeight, aWidth, VSDK_CV_16SC1);
  if (mDstImg.empty())
  {
    lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }
  else
  {
    switch (mBoxSize)
    {
      case FLT_MASK_7:
      {
        lRetAcfVal =  gfttB7N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_5:
      {
        lRetAcfVal =  gfttB5N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      case FLT_MASK_3:
      {
        lRetAcfVal =  gfttB3N5Compute->ConnectIO("OUTPUT_IMAGE", mDstImg);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
          goto EXIT;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } // else
  }

  EXIT:
  return lRetVal;

}

int GFTTCorners::Release()
{
  mHarrisParams = vsdk::SUMat();
  mGFTTParams = vsdk::SUMat();
  mSortFilterParams = vsdk::SUMat();
  mDstImg = vsdk::SUMat();
  mNumCorner = vsdk::SUMat();
  mState = init;

  if(nullptr != mGFTTB7N5Compute)
  {
     delete (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute;
     mGFTTB7N5Compute = nullptr;
  } 
  if(nullptr != mGFTTB5N5Compute)
  {
     delete (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute;
     mGFTTB5N5Compute = nullptr;
  }
  if(nullptr != mGFTTB3N5Compute)
  {
     delete (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute;
     mGFTTB3N5Compute = nullptr;
  }
  if(nullptr != mGFTTB7N5ComputeExtract)
  {
     delete (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
     mGFTTB7N5ComputeExtract = nullptr;
  }
  if(nullptr != mGFTTB5N5ComputeExtract)
  {
     delete (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
     mGFTTB5N5ComputeExtract = nullptr;
  }
  if(nullptr != mGFTTB3N5ComputeExtract)
  {
     delete (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;
     mGFTTB3N5ComputeExtract = nullptr;
  }
  if(nullptr != mHARRISB7N5Compute)
  {
     delete (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
     mHARRISB7N5Compute = nullptr;
  }
  if(nullptr != mHARRISB5N5Compute)
  {
     delete (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
     mHARRISB5N5Compute = nullptr;
  }
  if(nullptr != mHARRISB3N5Compute)
  {
     delete (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;
     mHARRISB3N5Compute = nullptr;
  }
  if(nullptr != mHARRISB7N5ComputeExtract)
  {
     delete (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
     mHARRISB7N5ComputeExtract = nullptr;
  }
  if(nullptr != mHARRISB5N5ComputeExtract)
  {
     delete (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
     mHARRISB5N5ComputeExtract = nullptr;
  }
  if(nullptr != mHARRISB3N5ComputeExtract)
  {
     delete (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;
     mHARRISB3N5ComputeExtract = nullptr;
  }

  return 0;
}
int GFTTCorners::IntializeSelectedGraph()
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;

  switch (mState)
  {
    case HarrisAPEX :
      lRetVal = IntializeHarrisApexSelectedGraph();
      break;
    case HarrisARM :
      lRetVal = IntializeHarrisArmSelectedGraph();
      break;
    case GFTTARM :
      lRetVal = IntializeGfttArmSelectedGraph();
      break;
    case GFTTAPEX :
      lRetVal = IntializeGfttApexSelectedGraph();
      break;
    default:
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
  } /* switch (mState) */
  return lRetVal;
}

int GFTTCorners::IntializeGfttApexSelectedGraph()
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
  GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
  GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      lRetAcfVal = gfttB7N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B7N5_COMPUTE_EXTRACT", gfttB7N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = gfttB7N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                    mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_5:
    {
      lRetAcfVal = gfttB5N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B5N5_COMPUTE_EXTRACT", gfttB5N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if((true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = gfttB5N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                    mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_3:
    {
      lRetAcfVal = gfttB3N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B3N5_COMPUTE_EXTRACT", gfttB3N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if((mWeSelectedApuConfiguration == true) &&
        (lRetVal == APEXCV_SUCCESS))
      {
        lRetAcfVal = gfttB3N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                    mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::IntializeGfttArmSelectedGraph()
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  GFTT_B7N5_COMPUTE *gfttB7N5Compute = (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute;
  GFTT_B5N5_COMPUTE *gfttB5N5Compute = (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute;
  GFTT_B3N5_COMPUTE *gfttB3N5Compute = (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      lRetAcfVal = gfttB7N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B7N5_COMPUTE", gfttB7N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = gfttB7N5Compute->SelectApuConfiguration(mApuConfig,
                                                             mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_5:
    {
      lRetAcfVal = gfttB5N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B5N5_COMPUTE", gfttB5N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = gfttB5N5Compute->SelectApuConfiguration(mApuConfig,
                                                             mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_3:
    {
      lRetAcfVal = gfttB3N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("GFTT_B3N5_COMPUTE", gfttB3N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = gfttB3N5Compute->SelectApuConfiguration(mApuConfig,
                                                             mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      } 
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::IntializeHarrisApexSelectedGraph()
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
  HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
  HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      lRetAcfVal = harrisB7N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B7N5_COMPUTE_EXTRACT", harrisB7N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB7N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                      mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_5:
    {
      lRetAcfVal = harrisB5N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B5N5_COMPUTE_EXTRACT", harrisB5N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB5N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                      mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_3:
    {
      lRetAcfVal = harrisB3N5ComputeExtract->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B3N5_COMPUTE_EXTRACT", harrisB3N5ComputeExtract);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB3N5ComputeExtract->SelectApuConfiguration(mApuConfig,
                                                                      mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::IntializeHarrisArmSelectedGraph()
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  HARRIS_B7N5_COMPUTE *harrisB7N5Compute = (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
  HARRIS_B5N5_COMPUTE *harrisB5N5Compute = (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
  HARRIS_B3N5_COMPUTE *harrisB3N5Compute = (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      lRetAcfVal = harrisB7N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B7N5_COMPUTE", harrisB7N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB7N5Compute->SelectApuConfiguration(mApuConfig,
                                                               mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_5:
    {
      lRetAcfVal = harrisB5N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B5N5_COMPUTE", harrisB5N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB5N5Compute->SelectApuConfiguration(mApuConfig,
                                                               mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    case FLT_MASK_3:
    {
      lRetAcfVal = harrisB3N5Compute->Initialize();
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto EXIT;
      }
      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("HARRIS_B3N5_COMPUTE", harrisB3N5Compute);
        lInfo.push_PortName("INPUT");
        mvInfoClusters.push_back(lInfo);
      }
      if(( true == mWeSelectedApuConfiguration) &&
        (APEXCV_SUCCESS == lRetVal))
      {
        lRetAcfVal = harrisB3N5Compute->SelectApuConfiguration(mApuConfig,
                                                               mApexId);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
          goto EXIT;
        }
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::ConnectIOSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;

  switch (mState)
  {
    case HarrisAPEX :
      lRetVal = ConnectIOHarrisApexSelectedGraph(aSrc);
      break;
    case HarrisARM :
      lRetVal = ConnectIOHarrisArmSelectedGraph(aSrc);
      break;
    case GFTTARM :
      lRetVal = ConnectIOGfttArmSelectedGraph(aSrc);
      break;
    case GFTTAPEX :
      lRetVal = ConnectIOGfttApexSelectedGraph(aSrc);
      break;
    default:
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
  } /* switch (mState) */
  return lRetVal;
}

int GFTTCorners::ConnectIOGfttApexSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
  GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
  GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      /* GFTT strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = gfttB7N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5ComputeExtract->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = gfttB7N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:
    {
      /* GFTT strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = gfttB5N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5ComputeExtract->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = gfttB5N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:
    {
      /* GFTT strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = gfttB3N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5ComputeExtract->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = gfttB3N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::ConnectIOGfttArmSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  GFTT_B7N5_COMPUTE *gfttB7N5Compute = (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute;
  GFTT_B5N5_COMPUTE *gfttB5N5Compute = (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute;
  GFTT_B3N5_COMPUTE *gfttB3N5Compute = (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      /* GFTT strength image computation graph */
      /* Input Port */
      lRetAcfVal = gfttB7N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5Compute->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:
    {
      /* GFTT strength image computation graph */
      /* Input Port */
      lRetAcfVal = gfttB5N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5Compute->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:
    {
      /* GFTT strength image computation graph */
      /* Input Port */
      lRetAcfVal = gfttB3N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5Compute->ConnectIO("PARAMS", mGFTTParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::ConnectIOHarrisApexSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
  HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
  HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      /* HARRIS strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = harrisB7N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5ComputeExtract->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = harrisB7N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:
    {
      /* HARRIS strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = harrisB5N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5ComputeExtract->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = harrisB5N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:
    {
      /* HARRIS strength image computation and corner extraction graph */
      /* Input Port */
      lRetAcfVal = harrisB3N5ComputeExtract->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5ComputeExtract->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      /* Output Port */
      lRetAcfVal = harrisB3N5ComputeExtract->ConnectIO("OUTPUT_CORNERLIST", mCornerList);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5ComputeExtract->ConnectIO("OUTPUT_COUNT", mNumCorner);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}

int GFTTCorners::ConnectIOHarrisArmSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  HARRIS_B7N5_COMPUTE *harrisB7N5Compute = (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
  HARRIS_B5N5_COMPUTE *harrisB5N5Compute = (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
  HARRIS_B3N5_COMPUTE *harrisB3N5Compute = (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;

  switch (mBoxSize)
  {
    case FLT_MASK_7:
    {
      /* HARRIS strength image computation graph */
      /* Input Port */
      lRetAcfVal = harrisB7N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5Compute->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:
    {
      /* HARRIS strength image computation graph */
      /* Input Port */
      lRetAcfVal = harrisB5N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5Compute->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:
    {
      /* HARRIS strength image computation graph */
      /* Input Port */
      lRetAcfVal = harrisB3N5Compute->ConnectIO("INPUT", aSrc);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5Compute->ConnectIO("PARAMS", mHarrisParams);
      if (0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  }

  EXIT:
  return lRetVal;
}
int GFTTCorners::ReconnectIOSelectedGraph(vsdk::SUMat& aSrc)
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;

  switch (mState)
  {
    case HarrisAPEX :
      lRetVal = ConnectIOHarrisApexSelectedGraph(aSrc);
      break;
    case HarrisARM :
      lRetVal = ConnectIOHarrisArmSelectedGraph(aSrc);
      break;
    case GFTTARM :
      lRetVal = ConnectIOGfttArmSelectedGraph(aSrc);
      break;
    case GFTTAPEX :
      lRetVal = ConnectIOGfttApexSelectedGraph(aSrc);
      break;
    default:
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
  } /* switch (mState) */
  return lRetVal;
}

int GFTTCorners::ProcessHarrisApexSelectedGraph()
{
  int lRetVal = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
 
  HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
  HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
  HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;
  
  switch (mBoxSize)
  {
    case FLT_MASK_7:  // box filter size 7
    {
      //Update corner list port size
      lRetAcfVal = harrisB7N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB7N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:  // box filter size 5
    {
      //Update corner list port size
      lRetAcfVal = harrisB5N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB5N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:  // box filter size 3
    {
      //Update corner list port size
      lRetAcfVal = harrisB3N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB3N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  } /* switch (mBoxSize) */

  EXIT:
  return lRetVal;
}
int GFTTCorners::ProcessHarrisArmSelectedGraph()
{
  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
 
  HARRIS_B7N5_COMPUTE *harrisB7N5Compute = (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
  HARRIS_B5N5_COMPUTE *harrisB5N5Compute = (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
  HARRIS_B3N5_COMPUTE *harrisB3N5Compute = (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;
   
  switch (mBoxSize)
  {
    case FLT_MASK_7:  // box filter size 7
    {
      //Update corner list port size
      lRetAcfVal = harrisB7N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB7N5Compute->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB7N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:  // box filter size 5
    {
      //Update corner list port size
      lRetAcfVal = harrisB5N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB5N5Compute->Start();  SHOW_ERROR(lRetAcfValVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB5N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
     break;
    }
    case FLT_MASK_3:  // box filter size 3
    {
      //Update corner list port size
      lRetAcfVal = harrisB3N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = harrisB3N5Compute->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = harrisB3N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  } /* switch (mBoxSize) */

  EXIT:
  return lRetVal;
}

int GFTTCorners::ProcessGfttApexSelectedGraph()
{
  int lRetVal   = APEXCV_SUCCESS;
  int lRetAcfVal = -1;

  GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
  GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
  GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;
  
  switch (mBoxSize)
  {
    case FLT_MASK_7:  // box filter size 7
    {
      //Update corner list port size
      lRetAcfVal = gfttB7N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB7N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:  // box filter size 5
    {
      //Update corner list port size
      lRetAcfVal = gfttB5N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB5N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfValal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:  // box filter size 3
    {
      //Update corner list port size
      lRetAcfVal = gfttB3N5ComputeExtract->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5ComputeExtract->OverridePortChunkSize("OUTPUT_CORNERLIST", mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u)*2, 1);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB3N5ComputeExtract->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5ComputeExtract->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  } /* switch (mBoxSize) */

  EXIT:
  return lRetVal;
}

int GFTTCorners::ProcessGfttArmSelectedGraph()
{
  int lRetVal    = APEXCV_SUCCESS;
  int lRetAcfVal = -1;
  
  GFTT_B7N5_COMPUTE *gfttB7N5Compute = (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute;
  GFTT_B5N5_COMPUTE *gfttB5N5Compute = (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute;
  GFTT_B3N5_COMPUTE *gfttB3N5Compute = (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute;
  
  switch (mBoxSize)
  {
    case FLT_MASK_7:  // box filter size 7
    {
      //Update corner list port size
      lRetAcfVal = gfttB7N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB7N5Compute->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB7N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_5:  // box filter size 5
    {
      //Update corner list port size
      lRetAcfVal = gfttB5N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB5N5Compute->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB5N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    case FLT_MASK_3:  // box filter size 3
    {
      //Update corner list port size
      lRetAcfVal = gfttB3N5Compute->SelectScenario("INPUT", mSetChunkWidth, 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        goto EXIT;
      }
      //run
      lRetAcfVal = gfttB3N5Compute->Start();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      lRetAcfVal = gfttB3N5Compute->Wait();  SHOW_ERROR(lRetAcfVal == 0);
      if ( 0 != lRetAcfVal)
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        goto EXIT;
      }
      break;
    }
    default:
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      break;
    }
  } /* switch (mBoxSize) */

  EXIT:
  return lRetVal;
}

int GFTTCorners::Initialize(
          vsdk::SUMat& aSrc            /*!< 8-bit grayscale source image. */,
          icp::FeatureDescriptor& aCorners  /*!< 16-bit signed integer destination corner list buffer. */,
          int aQualityLevel            /*!< Quality Level. */,
          int aMinDistance            /*!< Minimum Distance between 2 corners. */,
          int aMaxTotalCorners          /*!< Maximum number of total corners to detect. */,
          int aUseHarrisDetector        /*!< 0 - use GFTT; 1 - Use harris; .*/,
          int aHarrisK               /*!< Harris Corner coefficent */,
          int aHarrisThreshold          /*!< Harris Threshold */,
          int aBoxSize)
{
  int lRetVal = APEXCV_SUCCESS;

  if (mState != init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    goto EXIT;
  }
  if (aMinDistance > MAX_MIN_DISTANCE)
  {
    printf("Invalid configuration. minDistance cannot be higher than %d\n", MAX_MIN_DISTANCE);
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    goto EXIT;
  }
  else if (mState == init)
  {
    //Check if Harris or GFTT will be used
    if (aUseHarrisDetector)
    {
      if (aMaxTotalCorners > MAX_CORNERS)
        mState = HarrisARM;
      else
        mState = HarrisAPEX;
    }
    else
    {
      if (aMaxTotalCorners > MAX_CORNERS)
       mState = GFTTARM;
     else
       mState = GFTTAPEX;
    }

    switch(mState)
    {
      case GFTTAPEX:
      {
        GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = new GFTT_B7N5_COMPUTE_EXTRACT;
        GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = new GFTT_B5N5_COMPUTE_EXTRACT;
        GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = new GFTT_B3N5_COMPUTE_EXTRACT;

        if( nullptr == gfttB7N5ComputeExtract || nullptr == gfttB5N5ComputeExtract || nullptr == gfttB3N5ComputeExtract)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mGFTTB7N5ComputeExtract = gfttB7N5ComputeExtract;
        mGFTTB5N5ComputeExtract = gfttB5N5ComputeExtract;
        mGFTTB3N5ComputeExtract = gfttB3N5ComputeExtract;
        break;
      }
      case GFTTARM:
      {
        GFTT_B7N5_COMPUTE *gfttB7N5Compute = new GFTT_B7N5_COMPUTE;
        GFTT_B5N5_COMPUTE *gfttB5N5Compute = new GFTT_B5N5_COMPUTE;
        GFTT_B3N5_COMPUTE *gfttB3N5Compute = new GFTT_B3N5_COMPUTE;

        if( nullptr == gfttB7N5Compute || nullptr == gfttB5N5Compute || nullptr == gfttB3N5Compute)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mGFTTB7N5Compute = gfttB7N5Compute;
        mGFTTB5N5Compute = gfttB5N5Compute;
        mGFTTB3N5Compute = gfttB3N5Compute;
        break;
      }
      case HarrisAPEX:
      {
        HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = new HARRIS_B7N5_COMPUTE_EXTRACT;
        HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = new HARRIS_B5N5_COMPUTE_EXTRACT;
        HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = new HARRIS_B3N5_COMPUTE_EXTRACT;

        if( nullptr == harrisB7N5ComputeExtract || nullptr == harrisB5N5ComputeExtract || nullptr == harrisB3N5ComputeExtract)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mHARRISB7N5ComputeExtract = harrisB7N5ComputeExtract;
        mHARRISB5N5ComputeExtract = harrisB5N5ComputeExtract;
        mHARRISB3N5ComputeExtract = harrisB3N5ComputeExtract;
        break;
      }
      case HarrisARM:
      {
        HARRIS_B7N5_COMPUTE *harrisB7N5Compute = new HARRIS_B7N5_COMPUTE;
        HARRIS_B5N5_COMPUTE *harrisB5N5Compute = new HARRIS_B5N5_COMPUTE;
        HARRIS_B3N5_COMPUTE *harrisB3N5Compute = new HARRIS_B3N5_COMPUTE;

        if( nullptr == harrisB7N5Compute || nullptr == harrisB5N5Compute || nullptr == harrisB3N5Compute)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }

        mHARRISB7N5Compute = harrisB7N5Compute;
        mHARRISB5N5Compute = harrisB5N5Compute;
        mHARRISB3N5Compute = harrisB3N5Compute;

        break;
      }
      default:
        return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
        break;
    }

    mBoxSize = aBoxSize;
    lRetVal = IntializeSelectedGraph();
    if (APEXCV_SUCCESS == lRetVal)
    {
      // Keep the image size info
      mImageWidth  = aSrc.cols;
      mImageHeight = aSrc.rows;

      // set and check chunk size
      if (!mSetACFSelectChunkSize)
      {
        int32_t numCu = 64;
        RetNumCU(0, numCu);
        mSetChunkWidth = mImageWidth / numCu;

        if ((mSetChunkWidth <=32) && ((mSetChunkWidth%2) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
        {
          lRetVal = APEXCV_SUCCESS;
        }
        else if ((mSetChunkWidth <=64) && ((mSetChunkWidth%4) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
        {
          lRetVal = APEXCV_SUCCESS;
        }
        else if ((mSetChunkWidth <=128) && ((mSetChunkWidth%8) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
        {
          lRetVal = APEXCV_SUCCESS;
        }
        else
        {
          lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
          goto EXIT;
        }
      }
      // Set the corner coefficient
      lRetVal = Init_params(aQualityLevel, aMinDistance,
                             mBoxSize, mNMSSize,
                             aMaxTotalCorners,
                             aUseHarrisDetector, aHarrisK, aHarrisThreshold);
      if (APEXCV_SUCCESS != lRetVal)
      {
        goto EXIT;
      }
      lRetVal = Init_buffer(mImageWidth, mImageHeight);
      if (APEXCV_SUCCESS != lRetVal)
      {
        goto EXIT;
      }
      // Convert FeatureDescriptor into DataDescriptor
      int capacity = aCorners.GetSize();

      mCornerList = vsdk::CustomPointerSUMat(1, capacity*2, VSDK_CV_32SC1, (uint8_t *)aCorners.GetDataPtrPhys(), (uint8_t *)aCorners.GetDataPtr());
      // save pointer to the corner list
      mCorners = &aCorners;

      lRetVal = ConnectIOSelectedGraph(aSrc);

    } /* if (APEXCV_SUCCESS == lRetVal) */
  } /* else if (mState == init) */

  // Release resource if init unsuccessful
  EXIT:
  if ((APEXCV_SUCCESS != lRetVal) &&
      (APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED != lRetVal))
  {
    Release();
  }
  return lRetVal;
}

int GFTTCorners::ReconnectIO(
          vsdk::SUMat& aSrc,
          icp::FeatureDescriptor& aCorners)
{

  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;

  // if not initialized once
  if (mState == init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    // Keep the image size info
    mImageWidth  = aSrc.cols;
    mImageHeight = aSrc.rows;
    // Convert FeatureDescriptor into DataDescriptor
    int capacity = aCorners.GetSize();
    mCornerList = vsdk::CustomPointerSUMat(1, capacity*2, VSDK_CV_32SC1, (uint8_t *)aCorners.GetDataPtrPhys(), (uint8_t *)aCorners.GetDataPtr());
    // save the pointer to the corner list
    mCorners = &aCorners;

    // set and check chunk size
    if (!mSetACFSelectChunkSize)
    {
      int32_t numCu = 64;
      RetNumCU(0, numCu);
      mSetChunkWidth = mImageWidth / numCu;

      if ((mSetChunkWidth <=32) && ((mSetChunkWidth%2) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
      {
        lRetVal = APEXCV_SUCCESS;
      }
      else if ((mSetChunkWidth <=64) && ((mSetChunkWidth%4) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
      {
        lRetVal = APEXCV_SUCCESS;
      }
      else if ((mSetChunkWidth <=128) && ((mSetChunkWidth%8) == 0) && ((mSetChunkWidth * numCu) == mImageWidth))
      {
        lRetVal = APEXCV_SUCCESS;
      }
      else
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        goto EXIT;
      }
    }
    //reallocate buffer
    lRetVal = Init_buffer(mImageWidth, mImageHeight);
    if (APEXCV_SUCCESS != lRetVal)
    {
      goto EXIT;
    }
    lRetVal = ReconnectIOSelectedGraph(aSrc);
  } /* else */

  EXIT:
  // Release resource if init unsuccessful
  if (APEXCV_SUCCESS != lRetVal)
  {
    Release();
  }
  return lRetVal;
}

int GFTTCorners::SetParameters(
            int aQualityLevel        /*!< Quality Level*/,
            int aMinDistance        /*!< Minimum distance between 2 corners*/,
            int aMaxCorners         /*!< Maximum number of total corners to detect. */,
            int aUseHarrisDetector    /*!< 0 - use GFTT; 1 - Use harris; .*/,
            int aHarrisK           /*!< Harris Corner coefficent */,
            int aHarrisThreshold      /*!< Harris Threshold */)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (mState == init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    if (aMinDistance > MAX_MIN_DISTANCE)
    {
      printf("Invalid configuration. minDistance cannot be higher than %d\n", MAX_MIN_DISTANCE);
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    }
    else
    {
      lRetVal = Init_params(aQualityLevel, aMinDistance,
                            mBoxSize, mNMSSize,
                            aMaxCorners,
                            aUseHarrisDetector, aHarrisK, aHarrisThreshold);
    }
  }
  return lRetVal;
}
int GFTTCorners::SetMaxNumberCorners(int aMaxTotalCorners)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (mState == init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    if (aMaxTotalCorners > MAX_CORNERS)
    {
      switch (mState)
      {
        case HarrisAPEX:
        case HarrisARM:
          mState = HarrisARM;
          lRetVal = APEXCV_SUCCESS;
          break;
        case GFTTAPEX:
        case GFTTARM:
          mState = GFTTARM;
          lRetVal = APEXCV_SUCCESS;
          break;
        default:
          lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
          break;
      } /* switch (mState) */
    }
    else
    {
      switch (mState)
      {
        case HarrisARM:
        case HarrisAPEX:
          mState = HarrisAPEX;
          lRetVal = APEXCV_SUCCESS;
          break;
        case GFTTARM:
        case GFTTAPEX:
          mState = GFTTAPEX;
          lRetVal = APEXCV_SUCCESS;
          break;
        default:
          lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
          break;
      } /* switch (mState) */
    } /* else */
    if (APEXCV_SUCCESS == lRetVal)
    {
      mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(6u) = aMaxTotalCorners;
      mGFTTParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(5u) = aMaxTotalCorners;
    } /* if (maxTotalCorners > MAX_CORNERS)*/
  } /*   if (mState == init) */
  return lRetVal;
}
int GFTTCorners::Process()
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  int lRet = 0;
  //Trigger appropriate graph based on the state
  switch (mState)
  {
    case init:
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    case HarrisAPEX:
      lRetVal = ProcessHarrisApexSelectedGraph();
      if (APEXCV_SUCCESS == lRetVal)
      {
        lRet = mCorners->SetCount(mNumCorner.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u));
        if (lRet != 0)
        {
          lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
      break;
    case HarrisARM:
      lRetVal = ProcessHarrisArmSelectedGraph();
      if (APEXCV_SUCCESS == lRetVal)
      {
        lRetVal = ProcessCorners(*mCorners); SHOW_ERROR(lRetVal == APEXCV_SUCCESS);
      }
      break;
    case GFTTAPEX:
      lRetVal = ProcessGfttApexSelectedGraph();
      if (APEXCV_SUCCESS == lRetVal)
      {
        lRet = mCorners->SetCount(mNumCorner.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u));
        if  (lRet != 0)
        {
          lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
      break;
    case GFTTARM:
      lRetVal = ProcessGfttArmSelectedGraph();
      if (APEXCV_SUCCESS == lRetVal)
      {
        lRetVal = ProcessCorners(*mCorners); SHOW_ERROR(lRetVal == APEXCV_SUCCESS);
      }
      break;
    default:
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
  } /* switch (mState) */
  return lRetVal;
}
int GFTTCorners::GfttRetBlockWidthAndHeight(int32_t &aChunkWidth, int32_t &aChunkHeight)
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;
  int lRetAcfVal = -1;
  if (mState == GFTTAPEX)
  {
    GFTT_B7N5_COMPUTE_EXTRACT *gfttB7N5ComputeExtract = (GFTT_B7N5_COMPUTE_EXTRACT *)mGFTTB7N5ComputeExtract;
    GFTT_B5N5_COMPUTE_EXTRACT *gfttB5N5ComputeExtract = (GFTT_B5N5_COMPUTE_EXTRACT *)mGFTTB5N5ComputeExtract;
    GFTT_B3N5_COMPUTE_EXTRACT *gfttB3N5ComputeExtract = (GFTT_B3N5_COMPUTE_EXTRACT *)mGFTTB3N5ComputeExtract;
  
    switch (mBoxSize)
    {
      case FLT_MASK_7:  // box filter size 7
      {
        lRetAcfVal = gfttB7N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_5:  // box filter size 5
      {
        lRetAcfVal = gfttB5N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_3:  // box filter size 3
      {
        lRetAcfVal = gfttB3N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } /* switch (mState) */
  } /*  if (mState == GFTTAPEX) */
  else
  {
    GFTT_B7N5_COMPUTE *gfttB7N5Compute = (GFTT_B7N5_COMPUTE *)mGFTTB7N5Compute;
    GFTT_B5N5_COMPUTE *gfttB5N5Compute = (GFTT_B5N5_COMPUTE *)mGFTTB5N5Compute;
    GFTT_B3N5_COMPUTE *gfttB3N5Compute = (GFTT_B3N5_COMPUTE *)mGFTTB3N5Compute;

    switch (mBoxSize)
    {
      case FLT_MASK_7:  // box filter size 7
      {
        lRetAcfVal = gfttB7N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_5:  // box filter size 5
      {
        lRetAcfVal = gfttB5N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_3:  // box filter size 3
      {
        lRetAcfVal = gfttB3N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } /* switch (mState) */
  } /* if (mState == GFTTARM) */
  return lRetVal;
}

int GFTTCorners::HarrisRetBlockWidthAndHeight(int32_t &aChunkWidth, int32_t &aChunkHeight)
{
  int lRetVal    = APEXCV_ERROR_INTERNAL_ERROR;
  int lRetAcfVal = -1;
  if (mState == HarrisAPEX)
  {
    HARRIS_B7N5_COMPUTE_EXTRACT *harrisB7N5ComputeExtract = (HARRIS_B7N5_COMPUTE_EXTRACT *)mHARRISB7N5ComputeExtract;
    HARRIS_B5N5_COMPUTE_EXTRACT *harrisB5N5ComputeExtract = (HARRIS_B5N5_COMPUTE_EXTRACT *)mHARRISB5N5ComputeExtract;
    HARRIS_B3N5_COMPUTE_EXTRACT *harrisB3N5ComputeExtract = (HARRIS_B3N5_COMPUTE_EXTRACT *)mHARRISB3N5ComputeExtract;
    switch (mBoxSize)
    {
      case FLT_MASK_7:  // box filter size 7
      {
        lRetAcfVal = harrisB7N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_5:  // box filter size 5
      {
        lRetAcfVal = harrisB5N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_3:  // box filter size 3
      {
        lRetAcfVal = harrisB3N5ComputeExtract->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } /* switch (mState) */
  }  /* if (mState == GFTTAPEX) */
  else
  {
    HARRIS_B7N5_COMPUTE *harrisB7N5Compute = (HARRIS_B7N5_COMPUTE *)mHARRISB7N5Compute;
    HARRIS_B5N5_COMPUTE *harrisB5N5Compute = (HARRIS_B5N5_COMPUTE *)mHARRISB5N5Compute;
    HARRIS_B3N5_COMPUTE *harrisB3N5Compute = (HARRIS_B3N5_COMPUTE *)mHARRISB3N5Compute;
    switch (mBoxSize)
    {
      case FLT_MASK_7:  // box filter size 7
      {
        lRetAcfVal = harrisB7N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_5:  // box filter size 5
      {
        lRetAcfVal = harrisB5N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      case FLT_MASK_3:  // box filter size 3
      {
        lRetAcfVal = harrisB3N5Compute->QueryPortChunkSize("INPUT", aChunkWidth, aChunkHeight);
        if (0 != lRetAcfVal)
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_QUERYPORTCHUNKSIZE;
        }
        else
        {
          lRetVal = APEXCV_SUCCESS;
        }
        break;
      }
      default:
      {
        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        break;
      }
    } /* switch (mState) */
  } /* if (mState == GFTTARM) */
  return lRetVal;
}

int GFTTCorners::RetBlockWidth()
{
  int32_t chunkWidth  = 0;
  int32_t chunkHeight = 0;
  int lRetVal         = APEXCV_ERROR_INTERNAL_ERROR;

  switch (mState)
  {
    case init:
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    case HarrisAPEX:
    case HarrisARM:
      lRetVal = HarrisRetBlockWidthAndHeight(chunkWidth, chunkHeight);
      break;
    case GFTTAPEX:
    case GFTTARM:
      lRetVal = GfttRetBlockWidthAndHeight(chunkWidth, chunkHeight);
      break;
    default:

      break;
  }  /* switch (mState) */
  if (APEXCV_SUCCESS == lRetVal)
  {
    mChunkWidth = chunkWidth;
  }
  else
  {
    chunkWidth = 0;
  }
  return chunkWidth;
}

int GFTTCorners::RetBlockHeight()
{
  int32_t chunkWidth  = 0;
  int32_t chunkHeight = 0;
  int lRetVal         = APEXCV_ERROR_INTERNAL_ERROR;
  switch (mState)
  {
    case init:
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    case HarrisAPEX:
    case HarrisARM:
      lRetVal = HarrisRetBlockWidthAndHeight(chunkWidth, chunkHeight);
      break;
    case GFTTAPEX:
    case GFTTARM:
      lRetVal = GfttRetBlockWidthAndHeight(chunkWidth, chunkHeight);
      break;
    default:
      break;
  }  /* switch (mState) */
  if (APEXCV_SUCCESS == lRetVal)
  {
    mChunkHeight = chunkHeight;
  }
  else
  {
    chunkHeight = 0;
  }
    return chunkHeight;
}
int GFTTCorners::RetNumberCorners() {
  return mNumCorner.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u);
}
vsdk::SUMat GFTTCorners::RetCornerImage()
{
  return mDstImg;
}
int GFTTCorners::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                            int32_t aApexId)
{
  mApuConfig = aApuConfig;
  mApexId   = aApexId;

  mWeSelectedApuConfiguration = true;

  return APEXCV_SUCCESS;
}
int GFTTCorners::ProcessCorners(icp::FeatureDescriptor&  aCorners)
{
  int lRetVal = APEXCV_SUCCESS;
  //extract parameters
  vsdk::SMat lGFTTParamsMat = mGFTTParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat lHarrisParamsMat = mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat lDstImgMat = mDstImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

  int maxCorners = ((int32_t*) lGFTTParamsMat.data)[5];
  int qualityLevel =  ((int32_t*) lGFTTParamsMat.data)[0];
  int harrisThreshold = ((int32_t*) lHarrisParamsMat.data)[5];

#ifdef ENABLE_PROFILE
  uint64_t t[16] = {0};
  int tindx  = 0;
  t[tindx++] = FSL_Ticks();
#endif
  if (mDstImg.type() != VSDK_CV_16SC1 ||
      aCorners.GetSize() < maxCorners)
  {
    printf("Invalid parameters\n");
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
  }
  else
  {
    vsdk::SUMat lUmatTemp;
    vsdk::SMat lMatTemp;
    int sstr;
    int cnt;
    int* pTmpData;
    int* pTmp;
    short* pSrc;
    int width      = mDstImg.cols;
    int height     = mDstImg.rows;

    lUmatTemp = vsdk::SUMat(height, width, VSDK_CV_32SC1);
    if (lUmatTemp.empty())
    {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      goto EXIT;
    }
    {
      lMatTemp = lUmatTemp.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      pTmpData = (int*)lMatTemp.data;
    }

    pSrc       = (short*)lDstImgMat.data;
    sstr        = mDstImg.step[0]>>1;
    pTmp        = pTmpData;
    cnt          = 0;
    // Enumerate the corner list from the strength image
    for( int i = 0; i < height; ++i, pSrc += sstr )
    {
      for ( int j = 0; j < width; ++j)
      {
        //check threshold
        if (pSrc[j] > 0)
        {
          *pTmp++ = i*sstr+j;
          cnt++;
        }
      }
    }

    {
      // Sort the corner list
      pSrc = (short*)lDstImgMat.data;
      lRetVal = sort(pSrc, pTmpData, cnt);
      if(lRetVal != APEXCV_SUCCESS)
      {
        goto EXIT;
      }
      // Extract corners
      short threshold = harrisThreshold;
      short maxEigen = pSrc[pTmpData[0]];
      if (mState == GFTTARM) threshold = (((maxEigen*qualityLevel)>>7)+1)>>1;
      aCorners.SetCount(0);
      int ncorners = 0;
      for (int i = 0; i < cnt; ++i)
      {
        if (ncorners >= maxCorners ) break;
        int ofs = (int)(pTmpData[i]);
        int y = (int)(ofs / width);
        int x = (int)(ofs - y*width);
        short stren = pSrc[y*sstr + x];
        if (stren < threshold) continue;
        aCorners.Add(x, y, stren);
        ++ncorners;
      }
    }
  #ifdef ENABLE_PROFILE
    t[tindx++] = FSL_Ticks();
    printf("ARM PROCESS CORNERS: time = %8ld @ %4d corners\n", (t[tindx-1] - t[tindx-2]), ncorners);
  #endif
  }

  EXIT:
  return lRetVal;
}
/*
 *  covariance scalefactor is supported for default covariance value 6 and bigger than default covariance value 
 */
int GFTTCorners::SetShiftValue(  int aCovarianceScaleFactor, /*!< Rescale shift value for covariance filter */
                      int aBoxSize /*!< box filter size */)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  // check the box filter size and nms filter size
  if (mState == init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else if ((aBoxSize == FLT_MASK_7) || (aBoxSize == FLT_MASK_5) || (aBoxSize == FLT_MASK_3))
  {
    vsdk::SMat lHarrisParamsMat = mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    vsdk::SMat lGFTTParamsMat = mGFTTParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t* ptr0 = (int32_t*)lHarrisParamsMat.data;
    int32_t* ptr1 = (int32_t*)lGFTTParamsMat.data;
    /* Harris */
    ptr0[1] = aBoxSize;
    ptr0[3] = aCovarianceScaleFactor;
    ptr0[4] = (aBoxSize>>1) + (NMS_SIZE>>1);
    /* GFTT */
    ptr1[1] = aBoxSize;
    ptr1[3] = aCovarianceScaleFactor;
    ptr1[4] = (aBoxSize>>1) + (NMS_SIZE>>1);
    mBoxSize = aBoxSize;
    mNMSSize = NMS_SIZE;
    lRetVal = APEXCV_SUCCESS;
  }
  else
  {
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
  }
  return lRetVal;
}
int GFTTCorners::ResetShiftValue()
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (mState == init)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    vsdk::SMat lHarrisParamsMat = mHarrisParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    vsdk::SMat lGFTTParamsMat = mGFTTParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t* ptr0 = (int32_t*)lHarrisParamsMat.data;
    int32_t* ptr1 = (int32_t*)lGFTTParamsMat.data;

    /* Harris */
    ptr0[1] = BOX_SIZE;
    ptr0[3] = DEFAULT_COVARIANCE;
    ptr0[4] = (BOX_SIZE>>1) + (NMS_SIZE>>1);
    /* GFTT */
    ptr1[1] = BOX_SIZE;
    ptr1[3] = DEFAULT_COVARIANCE;
    ptr1[4] = (BOX_SIZE>>1) + (NMS_SIZE>>1);
    mBoxSize = BOX_SIZE;
    mNMSSize = NMS_SIZE;
    lRetVal = APEXCV_SUCCESS;
  }

  return lRetVal;
}

} /*namespace apexcv*/

static inline void swap(int *ptr1, int *ptr2)
{
  int tmp;
  tmp = *ptr1;
  *ptr1 = *ptr2;
  *ptr2 = tmp;
}

static int sort(short* vec, int* idx, int total)
{
  int lRetVal = APEXCV_SUCCESS;
  int isort_thresh = 7;
  int sp = 0;
  int* arr;

  struct
  {
    int *lb;
    int *ub;
  }stack[48];

  if( total <= 1 )
  {
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    goto EXIT;
  }

  arr = &idx[0];
  stack[0].lb = arr;
  stack[0].ub = arr + (total - 1);

  while(sp >= 0)
  {
    int* left = stack[sp].lb;
    int* right = stack[sp--].ub;

    for(;;)
    {
      int i, n = (int)(right - left) + 1, m;
      int* ptr;
      int* ptr2;

      if( n <= isort_thresh )
      {
        insert_sort:
        for( ptr = left + 1; ptr <= right; ptr++ )
        {
          for( ptr2 = ptr; ptr2 > left && LT(vec[ptr2[0]], vec[ptr2[-1]]); ptr2--)
          {
            swap(&ptr2[0], &ptr2[-1] );
          }
        }
        break;
      }
      else
      {
        int* left0;
        int* left1;
        int* right0;
        int* right1;
        int* pivot;
        int* a;
        int* b;
        int* c;
        int swap_cnt = 0;

        left0 = left;
        right0 = right;
        pivot = left + (n/2);

        if( n > 40 )
        {
          int d = n / 8;
          a = left, b = left + d, c = left + 2*d;
          vec[*a] = vec[*a];
          vec[*b] = vec[*b];
          vec[*c] = vec[*c];
          left = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                             : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

          a = pivot - d, b = pivot, c = pivot + d;
          pivot = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                              : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

          a = right - 2*d, b = right - d, c = right;
          right = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                              : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));
        }

        a = left, b = pivot, c = right;
        pivot = LT(vec[*a], vec[*b]) ? (LT(vec[*b], vec[*c]) ? b : (LT(vec[*a], vec[*c]) ? c : a))
                            : (LT(vec[*c], vec[*b]) ? b : (LT(vec[*a], vec[*c]) ? a : c));

        if( pivot != left0 )
        {
          swap(pivot, left0);
          pivot = left0;
        }

        left = left1 = left0 + 1;
        right = right1 = right0;

        for(;;)
        {
          while( left <= right && !LT(vec[*pivot], vec[*left]) )
          {
            if( !LT(vec[*left], vec[*pivot]) )
            {
              if( left > left1 )
              {
                swap(left1, left );
              }
              swap_cnt = 1;
              left1++;
            }
            left++;
          }

          while( left <= right && !LT(vec[*right], vec[*pivot]) )
          {
            if( !LT(vec[*pivot], vec[*right]) )
            {
              if( right < right1 )
              {
                swap(right1, right );
              }
              swap_cnt = 1;
              right1--;
            }
            right--;
          }

          if( left > right )
            break;

          swap(left, right );
          swap_cnt = 1;
          left++;
          right--;
        }

        if( swap_cnt == 0 )
        {
          left = left0, right = right0;
          goto insert_sort;
        }

        n = MIN( (left1 - left0), (left - left1) );
        for( i = 0; i < n; i++ )
        {
          swap(&left0[i], &left[i-n] );
        }

        n = MIN( (right0 - right1), (right1 - right) );
        for( i = 0; i < n; i++ )
        {
          swap(&left[i], &right0[i-n+1] );
        }

        n = (left - left1);
        m = (right1 - right);
        if( n > 1 )
        {
          if( m > 1 )
          {
            // check sp index out of range
            if(sp>=47)
            {
              printf("Error: Stack overflow\n");
              lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
              goto EXIT;
            }
            if( n > m )
            {
              stack[++sp].lb = left0;
              stack[sp].ub = left0 + n - 1;
              left = right0 - m + 1, right = right0;
            }
            else
            {
              stack[++sp].lb = right0 - m + 1;
              stack[sp].ub = right0;
              left = left0, right = left0 + n - 1;
            }
          }
          else
          {
            left = left0, right = left0 + n - 1;
          }
        }
        else if( m > 1 )
        {
          left = right0 - m + 1, right = right0;
        }
        else
          break;
      } /* else for if( n <= isort_thresh ) */
    }  /* for(;;) */
  }  /* while( sp >= 0 ) */
  EXIT:
  return lRetVal;
}
