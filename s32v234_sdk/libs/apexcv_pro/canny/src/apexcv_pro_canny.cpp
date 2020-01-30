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
*  \file apexcv_canny.cpp
*  \brief ApexCV Host wrapper for Canny Edge Detector algorithm
***********************************************************************************/

#include <stdint.h>

#include <icp_data.h>
#include <oal.h>
#include <iostream>
#include <apexcv_error_codes.hpp>

#include <apexcv_pro_canny.h>
#include <apexcv_pro_util.h>

// Generated Classes
#include <CANNY_NON_MAXIMA_SUPPRESS.hpp>
#include <CANNY_FULL.hpp>
#include <CANNY_FULL_ITERATION.hpp>
#include <CANNY_PROMOTE_EDGES_FULL.hpp>
#include <CANNY_PROMOTE_EDGES.hpp>
#include <CANNY_CREATE_IMAGE.hpp>

namespace apexcv{

/*!*********************************************************************************
*  \brief Default Constructor.
*
*  Allocate resources for the ACF processes.
***********************************************************************************/
Canny::Canny()
:
  mEdgeMapInput(0), mEdgeMapOutput(0),
  mInit(false),
  mWidth(0),mHeight(0),
  mBlockWidth(0),mBlockHeight(0),mIndex(0),
  mProcessed(false),
  mFullIterProcess(nullptr),
  mSuppressProcess(nullptr),
  mPromoteFullProcess(nullptr),
  mPromoteProcess(nullptr)
{
}

/*!*********************************************************************************
*  \brief Destructor.
*  Release resources for the ACF processes.
***********************************************************************************/
Canny::~Canny()
{
  Release();
}

/*!*********************************************************************************
*  \brief Release resources.
*  Release resources for the ACF processes and intermediate buffers.
***********************************************************************************/
void Canny::Release()
{
  if (nullptr != mFullIterProcess)
  {
    delete (CANNY_FULL_ITERATION*)mFullIterProcess;
    mFullIterProcess = nullptr;
  }
  if (nullptr != mSuppressProcess)
  {
    delete (CANNY_NON_MAXIMA_SUPPRESS*)mSuppressProcess;
    mSuppressProcess = nullptr;
  }
  if (nullptr != mPromoteFullProcess)
  {
    delete (CANNY_PROMOTE_EDGES_FULL*)mPromoteFullProcess;
    mPromoteFullProcess = nullptr;
  }
  if (nullptr != mPromoteProcess)
  {
    delete (CANNY_PROMOTE_EDGES*)mPromoteProcess;
    mPromoteProcess = nullptr;
  }

  mEdgeMapInput = 0;
  mEdgeMapOutput = 0;
  mIndex = 0;
  mConfig.mIters = 0;
  mBlockWidth = 0;
  mBlockHeight = 0;
  mWidth = 0;
  mHeight = 0;
  mInit = false;
  mConfig.mIsUseProcessCombined = false;
  mProcessed = false;
}

/*!*********************************************************************************
*  \brief Initialization
*
*  Initializes the intermediate buffers needed for the processes, initializes the
*  ACF processes and connect buffers to processes' IO. The number of iterations refers
*  to the number of times the block connection process is to be run.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::Initialize(
           vsdk::SUMat& aSrc,
           vsdk::SUMat& aDst,
           uint16_t aLow,
           uint16_t aHigh
)
{
  int lRetVal =  APEXCV_ERROR_INTERNAL_ERROR;

  if (mInit)
  {
    return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  if (aSrc.type()  == VSDK_CV_8UC1 &&
      aDst.type()  == VSDK_CV_8UC1 &&
      aSrc.cols    == aDst.cols &&
      aSrc.rows    == aDst.rows)
  {
    // Initialize Buffers
    mWidth  = aSrc.cols;
    mHeight = aSrc.rows;

    mEdgeMapImg_a  = vsdk::SUMat(mHeight, mWidth, VSDK_CV_8UC1);
    mEdgeMapImg_b  = vsdk::SUMat(mHeight, mWidth, VSDK_CV_8UC1);
    mLowThreshImg  = vsdk::SUMat(1, 1, VSDK_CV_16UC1);
    mHighThreshImg = vsdk::SUMat(1, 1, VSDK_CV_16UC1);

    if (!mEdgeMapImg_a.empty() && !mEdgeMapImg_b.empty() && !mLowThreshImg.empty() && !mHighThreshImg.empty())
    {
      mLowThreshImg.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED).at<uint16_t>(0u) = aLow;
      mHighThreshImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint16_t>(0u) = aHigh;
      mDestOutput = aDst;
      lRetVal = APEXCV_SUCCESS;
    }
    else
    {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
  }
  else
  {
    lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
  } /*  if (src.type()  == VSDK_CV_8UC1 &&
       dst.type()  == VSDK_CV_8UC1 &&
       src.cols    == dst.cols &&
       src.rows    == dst.rows)
     */

  if (APEXCV_SUCCESS == lRetVal)
  {
    if (false == mConfig.mIsUseProcessCombined)
    { // selected ProcessSeparated.
      CANNY_NON_MAXIMA_SUPPRESS* lSuppressProcess    = new CANNY_NON_MAXIMA_SUPPRESS;
      CANNY_PROMOTE_EDGES_FULL*  lPromoteFullProcess = new CANNY_PROMOTE_EDGES_FULL;
      CANNY_PROMOTE_EDGES*       lPromoteProcess     = new CANNY_PROMOTE_EDGES;
      if (nullptr == lSuppressProcess ||
          nullptr == lPromoteFullProcess ||
          nullptr == lPromoteProcess)
      {
        lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      else
      {
        mSuppressProcess    = lSuppressProcess;
        mPromoteFullProcess = lPromoteFullProcess;
        mPromoteProcess     = lPromoteProcess;

        lRetVal |= lSuppressProcess->Initialize();
        lRetVal |= lSuppressProcess->ConnectIO("INPUT",       aSrc);
        lRetVal |= lSuppressProcess->ConnectIO("OUTPUT",      mEdgeMapImg_a);
        lRetVal |= lSuppressProcess->ConnectIO("LOW_THRESH",  mLowThreshImg);
        lRetVal |= lSuppressProcess->ConnectIO("HIGH_THRESH", mHighThreshImg);
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("CANNY_NON_MAXIMA_SUPPRESS", lSuppressProcess);
          lInfo.push_PortName("INPUT");
          mvInfoClusters.push_back(lInfo);
        }
        // Connect the Edge Connection Process's
        // Used for first extra iteration
        lRetVal |= lPromoteFullProcess->Initialize();
        lRetVal |= lPromoteFullProcess->ConnectIO("INPUT",  mEdgeMapImg_a);
        lRetVal |= lPromoteFullProcess->ConnectIO("OUTPUT", aDst);
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("CANNY_PROMOTE_EDGES_FULL", lPromoteFullProcess);
          lInfo.push_PortName("INPUT");
          mvInfoClusters.push_back(lInfo);
        }
        // Used for subsequent extra iterations
        lRetVal |= lPromoteProcess->Initialize();
        if (mConfig.mIters > 0)
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("CANNY_PROMOTE_EDGES", lPromoteProcess);
          lInfo.push_PortName("INPUT");
          mvInfoClusters.push_back(lInfo);
        }

        mProcessed = false;
        if (lRetVal)
        {
          lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
        }
        else
        {
          mInit = true;
          lRetVal = APEXCV_SUCCESS;
        }
      } /* else */
    } /*  false == mConfig.mIsUseProcessCombined */
    else
    { // selected ProcessCombined
      CANNY_FULL_ITERATION* lFullIterProcess = new CANNY_FULL_ITERATION;
      CANNY_PROMOTE_EDGES*  lPromoteProcess  = new CANNY_PROMOTE_EDGES;
      if (nullptr == lFullIterProcess ||
          nullptr == lPromoteProcess)
      {
        lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      else
      {
        mFullIterProcess  = lFullIterProcess;
        mPromoteProcess   = lPromoteProcess;

        lRetVal |= lFullIterProcess->Initialize();
        lRetVal |= lFullIterProcess->ConnectIO("INPUT",  aSrc);
        lRetVal |= lFullIterProcess->ConnectIO("OUTPUT", aDst);
        lRetVal |= lFullIterProcess->ConnectIO("EDGE_OUTPUT", mEdgeMapImg_b);
        lRetVal |= lFullIterProcess->ConnectIO("LOW_THRESH",  mLowThreshImg);
        lRetVal |= lFullIterProcess->ConnectIO("HIGH_THRESH", mHighThreshImg);
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("CANNY_FULL_ITERATION", lFullIterProcess);
          lInfo.push_PortName("INPUT");
          mvInfoClusters.push_back(lInfo);
        }
        // Used for subsequent extra iterations
        lRetVal |= lPromoteProcess->Initialize();
        if (mConfig.mIters > 1)
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("CANNY_PROMOTE_EDGES", lPromoteProcess);
          lInfo.push_PortName("INPUT");
          mvInfoClusters.push_back(lInfo);
        }

        mProcessed = false;
        if (lRetVal)
        {
          lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
        }
        else
        {
          mInit = true;
          lRetVal = APEXCV_SUCCESS;
        }
      } /* else */
    } /*  true == mConfig.mIsUseProcessCombined */
  } /* if (APEXCV_SUCCESS == lRetVal) */

  if (APEXCV_SUCCESS != lRetVal)
  {
    Release();
  }
  return lRetVal;
}

/*!*********************************************************************************
*  \brief Reconnect IO
*
*  Use this to reconnect the input and output buffers. This only needs to
*  be done if the connected Input/Outputs are changed. If only the data within (no size,
*  data pointer, or type changes), then this does not need to be called.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::ReconnectIO(
          vsdk::SUMat& aSrc,
          vsdk::SUMat& aDst
)
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  if (mInit)
  {
    if ((aSrc.type()  == VSDK_CV_8UC1) &&
        (aDst.type()  == VSDK_CV_8UC1) &&
        (aSrc.cols    == aDst.cols) &&
        (aSrc.rows    == aDst.rows) &&
        (false       == aSrc.empty()) &&
        (false       == aDst.empty())
      )
    {
      mWidth = aSrc.cols;
      mHeight = aSrc.rows;
      mDestOutput = aDst;

      if (false == mConfig.mIsUseProcessCombined)
      { /* false:: selectProcessSeparated */
        mEdgeMapImg_a = vsdk::SUMat(mHeight, mWidth, VSDK_CV_8UC1);
        mEdgeMapImg_b = vsdk::SUMat(mHeight, mWidth, VSDK_CV_8UC1);

        if (mEdgeMapImg_a.empty() || mEdgeMapImg_b.empty())
        {
          lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        else
        {
          CANNY_NON_MAXIMA_SUPPRESS* lSuppressProcess    = (CANNY_NON_MAXIMA_SUPPRESS*) mSuppressProcess;
          CANNY_PROMOTE_EDGES_FULL*  lPromoteFullProcess = (CANNY_PROMOTE_EDGES_FULL*) mPromoteFullProcess;
          lRetVal  = lSuppressProcess->ConnectIO("INPUT", aSrc);
          lRetVal |= lSuppressProcess->ConnectIO("OUTPUT", mEdgeMapImg_a);
          lRetVal |= lSuppressProcess->ConnectIO("LOW_THRESH",  mLowThreshImg);
          lRetVal |= lSuppressProcess->ConnectIO("HIGH_THRESH", mHighThreshImg);
          lRetVal |= lPromoteFullProcess->ConnectIO("INPUT", mEdgeMapImg_a);
          lRetVal |= lPromoteFullProcess->ConnectIO("OUTPUT", aDst);
          if (lRetVal)
          {
            lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
          }
        }
      }/* if (false == mConfig.mIsUseProcessCombined) */
      else
      {  /* true:: selected ProcessCombined */
        CANNY_FULL_ITERATION* lFullIterProcess = (CANNY_FULL_ITERATION*) mFullIterProcess;
        lRetVal  = lFullIterProcess->ConnectIO("INPUT",       aSrc);
        lRetVal |= lFullIterProcess->ConnectIO("OUTPUT",      aDst);
        lRetVal |= lFullIterProcess->ConnectIO("EDGE_OUTPUT", mEdgeMapImg_b);
        lRetVal |= lFullIterProcess->ConnectIO("LOW_THRESH",  mLowThreshImg);
        lRetVal |= lFullIterProcess->ConnectIO("HIGH_THRESH", mHighThreshImg);
        if (lRetVal)
        {
          lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
        }
      } /* if (false == mConfig.mIsUseProcessCombined) */
    } /* if ((src.type() == VSDK_CV_8UC1) && (dst.type()  == VSDK_CV_8UC1)) ... */
    else
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    } /*  if (APEXCV_SUCCESS == lRetVal) */
  } /*   if (mInit) */
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  } /* if (mInit) */

  return lRetVal;
}

/*!*********************************************************************************
 *  \brief brief Set Edge Promotion Iteration and select separated kernel process or combined kernel process
 *
 *  Set give configure data to the configuration structure in Canny class
 ***********************************************************************************/
APEXCV_LIB_RESULT Canny::SetConfiguration(CannyConfig aConfig)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (mInit)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  else
  {
    mConfig = aConfig;
    lRetVal = APEXCV_SUCCESS;
  }
  return lRetVal;
}

/*!*********************************************************************************
*  \brief Return the configuration structure
*
*  Returns current configuration structure in Canny class.
***********************************************************************************/
void Canny::GetConfiguration(CannyConfig& aConfig)
{
  aConfig = mConfig;
}

/*!*********************************************************************************
*  \brief Set the Edge Hysteresis Thresholds
*
*  Sets the low and high thresholds for edge hysteresis. Only affects the _process()_
*  call. This does not affect the _promoteEdges()_ call.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::SetThresholds(uint16_t aLow, uint16_t aHigh)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (!mInit)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    mLowThreshImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint16_t>(0u) = aLow;
    mHighThreshImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint16_t>(0u) = aHigh;
    lRetVal = APEXCV_SUCCESS;
  }

  return lRetVal;
}

/*!*********************************************************************************
*  \brief Return the Edge Hysteresis Thresholds
*
*  Returns the low and high thresholds for edge hysteresis.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::GetThresholds(uint16_t& aLow, uint16_t& aHigh)
{
  int lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
  if (!mInit)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    aLow = mLowThreshImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint16_t>(0u);
    aHigh = mHighThreshImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint16_t>(0u);
    lRetVal = APEXCV_SUCCESS;
  }
  return lRetVal;

}

/*!*********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apex::Canny::initialize
*  function must be called with the appropriate parameters.
*
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::Process()
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  if (false == mConfig.mIsUseProcessCombined)
  {
    lRetVal = ProcessSeparated();
  }
  else
  {
    lRetVal = ProcessCombined();
  }
  return lRetVal;
}

/*!*********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apex::Canny::initialize
*  function must be called with the appropriate parameters.
*
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
*
*  This will run the version where the kernels are split into multiple graphs.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::ProcessSeparated()
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  if (mInit)
  {
    // call separated acf graphs
    lRetVal = APEXCV_SUCCESS;
    mBlockWidth  = 0;
    mBlockHeight = 0;
    CANNY_NON_MAXIMA_SUPPRESS*   lSuppressProcess    = (CANNY_NON_MAXIMA_SUPPRESS*) mSuppressProcess;
    CANNY_PROMOTE_EDGES_FULL*    lPromoteFullProcess = (CANNY_PROMOTE_EDGES_FULL*) mPromoteFullProcess;
    CANNY_PROMOTE_EDGES*         lPromoteProcess     = (CANNY_PROMOTE_EDGES*) mPromoteProcess;

    // Perform Non-Maxima Suppression
    lRetVal |= lSuppressProcess->Start();
    lRetVal |= lSuppressProcess->Wait();

    // Perform Edge Connection
    if (!lRetVal)
    {
      uint32_t iters = mConfig.mIters;
      // set right output buffer
      lRetVal |= lPromoteFullProcess->Start();
      lRetVal |= lPromoteFullProcess->Wait();
      mEdgeMapInput  = &mDestOutput;
      mEdgeMapOutput = &mEdgeMapImg_a;

      // Perform extra iterations of edge connection
      while (iters > 0 && !lRetVal)
      {
        if (iters == 1)
        {
          mEdgeMapOutput = &mDestOutput;
        }
        lPromoteProcess->ConnectIO("INPUT", *mEdgeMapInput);
        lPromoteProcess->ConnectIO("OUTPUT", *mEdgeMapOutput);
        lPromoteProcess->SelectScenario("INPUT", mBlockWidth, mBlockHeight);
        lRetVal |= lPromoteProcess->Start();
        lRetVal |= lPromoteProcess->Wait();
        if (iters > 0)
        {
          --iters;
        }
        vsdk::SUMat* temp = mEdgeMapInput;
        mEdgeMapInput = mEdgeMapOutput;
        mEdgeMapOutput = temp;
        if (iters == 1)
        {
          mEdgeMapOutput = &mDestOutput;
        }
      }
    } /* if (!lRetVal) */
    if (!lRetVal)
    {
      mProcessed = true;
    }
    else
    {
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      mProcessed = false;
    }
  } /* if (mInit) */
  return lRetVal;
}

/*!*********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apex::Canny::initialize
*  function must be called with the appropriate parameters.
*
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
*
*  This will run the version where every kernel is in one graph.
*  However, this version will detect less edges than the normal process (with zero
*  extra iterations) where the kernels are split into multiple graphs.
*
***********************************************************************************/
APEXCV_LIB_RESULT Canny::ProcessCombined()
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  if (mInit)
  {
    lRetVal = APEXCV_SUCCESS;
    uint32_t iter = mConfig.mIters;

    CANNY_FULL_ITERATION* lFullIterProcess = (CANNY_FULL_ITERATION*) mFullIterProcess;
    CANNY_PROMOTE_EDGES*  lPromoteProcess  = (CANNY_PROMOTE_EDGES*)  mPromoteProcess;

    lRetVal |= lFullIterProcess->Start();
    lRetVal |= lFullIterProcess->Wait();
    lFullIterProcess->QueryPortChunkSize("INPUT", mBlockWidth, mBlockHeight);
    if (iter > 0)
    {
      --iter;
    }
    // setting input output buffer
    mEdgeMapInput  = &mEdgeMapImg_b;
    mEdgeMapOutput = &mEdgeMapImg_a;
    if (iter == 1)
    {
      mEdgeMapOutput = &mDestOutput;
    }

    while (iter > 0 && !lRetVal)
    {
      if (iter > 0)
      {
        lPromoteProcess->ConnectIO("INPUT", *mEdgeMapInput);
        lPromoteProcess->ConnectIO("OUTPUT", *mEdgeMapOutput);
        lPromoteProcess->SelectScenario("INPUT", mBlockWidth, mBlockHeight);
        lRetVal |= lPromoteProcess->Start();
        lRetVal |= lPromoteProcess->Wait();
        --iter;
        vsdk::SUMat* temp = mEdgeMapInput;
        mEdgeMapInput = mEdgeMapOutput;
        mEdgeMapOutput = temp;
      }
      if (iter == 1)
      {
        mEdgeMapOutput = &mDestOutput;
      }
    }

    if (!lRetVal)
    {
      mProcessed = true;
    }
    else
    {
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      mProcessed = false;
    }
  } /*  if (mInit) */

  return lRetVal;
}

/*!*********************************************************************************
*  \brief Runs the Edge promotion extra times
*
*  This will run extra edge promotion iterations on a the previously obtained result
*  using either _promoteEdges()_ or _process()_ or _processCombined()_.
*  Only valid after running _process()_ or _processCombined()_.
***********************************************************************************/
APEXCV_LIB_RESULT Canny::PromoteEdges(int aIterations)
{
  int lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  int iter = aIterations;

  if (mInit)
  {
    if (mProcessed)
    {
      lRetVal = APEXCV_SUCCESS;
      CANNY_PROMOTE_EDGES* lPromoteProcess = (CANNY_PROMOTE_EDGES*) mPromoteProcess;

      // setting input output buffer
      if (iter <= 1)
      {
        mEdgeMapOutput = &mDestOutput;
      }

      while (iter > 0 && !lRetVal)
      {
        lPromoteProcess->ConnectIO("INPUT", *mEdgeMapInput);
        lPromoteProcess->ConnectIO("OUTPUT", *mEdgeMapOutput);
        lPromoteProcess->SelectScenario("INPUT", mBlockWidth, mBlockHeight);
        lRetVal |= lPromoteProcess->Start();
        lRetVal |= lPromoteProcess->Wait();
        --iter;
        if (iter > 0)
        {
          vsdk::SUMat* temp = mEdgeMapInput;
          mEdgeMapInput = mEdgeMapOutput;
          mEdgeMapOutput = temp;
        }
        if (iter == 1)
        {
          mEdgeMapOutput = &mDestOutput;
        }
      }
      if (lRetVal)
      {
        lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
      }
    } /* if (mProcessed) */
    else
    {
      lRetVal = APEXCV_ERROR_INTERNAL_ERROR;
    }
  } /*  if (mInit) */
  return lRetVal;
}

}  /* namespace apex */
