/******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2017 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/

//***********************************************
// TOGGLE THIS MACRO TO ENABLE/DISABLE PROFILING
//***********************************************
#define FAST_PERF_PROB 0

#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <string>

#include "apex_fast.h"
#include <oal.h>
#include <sumat.hpp>

#ifndef APEX2_EMULATE
#include <icp_data.h>
#include <APU_FAST.hpp>
#include <APU_FAST_NMS.hpp>
//#include< APU_FAST_SERIALIZED.hpp >
#else
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"

#include "apu_fast_apu_process_desc.hpp"
#include "apu_fast_nms_apu_process_desc.hpp"

REGISTER_PROCESS_TYPE(FAST9_GR_ID, apu_fast_apu_process_desc);
REGISTER_PROCESS_TYPE(FAST9_NMS_GR_ID, apu_fast_nms_apu_process_desc);
using namespace APEX2;

#endif // #ifndef APEX2_EMULATE

#include "apu_fast_graph_names.h"

#if FAST_PERF_PROB
#include <profiling_time_measure.h>
//#ifdef __STANDALONE__
// const uint32_t cTimeScale = 1; // In standalone values are already in
// microseconds #else  const uint32_t cTimeScale = 1000; // In linux values are
// in nanoseconds #endif
#endif

/***************************************************************************************/
namespace apex_fast9
{
/*************************************************************************************/
Fast9::Fast9()
    : mInit(0), mNonMaxSupression(false), mProcessType(FAST9_NONE),
      mImgWidth(0), mImgHeight(0), mpProcess(NULL)
{
}

/*************************************************************************************/
Fast9::~Fast9()
{
  if(mInit)
  {
    if(NULL != mpProcess)
    {
      delete((ACF_Process_APU*)mpProcess);
    }
    mInit = 0;
  }
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::Initialize(vsdk::SUMat& aDst,
                                    vsdk::SUMat& aSrc,
                                    const int   acThreshold,
                                    const bool  acNonMaxSuppression,
                                    vsdk::Rect& arRoi)
{
  int32_t          acfStatus = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess;

#if FAST_PERF_PROB
  int64_t time1, time2, time3;
  time1 = HostTicks();
#endif

  // Check if Initialize has already been called
  if(0 != mInit)
  {
    return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }

  //     -> Only 8bit unsigned images are supported
  if(VSDK_CV_8UC1 != aSrc.type())
  {
    return APEXCV_ERROR_INVALID_ARGUMENT;
  }

  // Start the initialization

  // Default is Normal Fast
  mProcessType = FAST9_PROCESS_IMG;

  // Fill in the class parameters
  mpOut             = &(aDst);
  mNonMaxSupression = (acNonMaxSuppression);
  mImgWidth         = (aSrc.cols);
  mImgHeight        = (aSrc.rows);
  mThreshold        = vsdk::SUMat(1, 1, VSDK_CV_8UC1);

  // Set default hw configuration
  // mApuConfig = ACF_APU_CFG__DEFAULT;
  mApexId = 0;

  // Update the values
  mThreshold.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at< uint8_t >(0u) =
      acThreshold;

  // Initialize process type
  if(acNonMaxSuppression == true)
  {
    mProcessType = FAST9_PROCESS_IMG_NMS;
  }

  mpProcess = NULL;

#if FAST_PERF_PROB
  time2 = HostTicks();
#endif

  // Create the process
  switch(mProcessType)
  {
    case FAST9_PROCESS_IMG:
    {
      mpProcess = new APU_FAST;
      printf("\nAPU_FAST graph instantiated\n");
    }
    break;

    case FAST9_PROCESS_IMG_NMS:
    {
      mpProcess = new APU_FAST_NMS;
      printf("\nAPU_FAST_NMS graph instantiated\n");
    }
    break;

    default:
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  pACFProcess = (ACF_Process_APU*)mpProcess;

  // Initialize process
  acfStatus = pACFProcess->Initialize();

  if(ACF_SUCCESS != acfStatus)
  {
#ifdef APEX2_EMULATE
    fprintf(stderr, "Error in graph %s: %s\n",
            pACFProcess->RetProcessIdentifier().c_str(),
            pACFProcess->ErrorMessage().c_str());
#endif
    delete((ACF_Process_APU*)mpProcess);
    return APEXCV_ERROR_ACF_PROC_INITIALIZE;
  }

  // Select APU config
  acfStatus = pACFProcess->SelectApuConfiguration(mApuConfig, mApexId);

  if(ACF_SUCCESS != acfStatus)
  {
#ifdef APEX2_EMULATE
    fprintf(stderr, "\nError in APEX-Graph %s: %s\n",
            pACFProcess->RetProcessIdentifier().c_str(),
            pACFProcess->ErrorMessage().c_str());
#endif
    delete((ACF_Process_APU*)mpProcess);
    return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
  }

  if(arRoi.x == 0 && arRoi.y == 0 && arRoi.width == aSrc.cols &&
     arRoi.height == aSrc.rows)
  {
    // Connect IO
    acfStatus = pACFProcess->ConnectIO(FAST9_GR_IN_IMG, aSrc);
    if(ACF_SUCCESS != acfStatus)
    {
#ifdef APEX2_EMULATE
      fprintf(stderr, "\nError in APEX-Graph %s: %s\n",
              pACFProcess->RetProcessIdentifier().c_str(),
              pACFProcess->ErrorMessage().c_str());
#else
      fprintf(stderr,
              "\nError in APEX-Graph: could not connect input port %s\n",
              (char*)FAST9_GR_IN_IMG);
#endif
      delete((ACF_Process_APU*)mpProcess);
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pACFProcess->ConnectIO(FAST9_GR_OUT, aDst);
    if(ACF_SUCCESS != acfStatus)
    {
#ifdef APEX2_EMULATE
      fprintf(stderr, "Error in APEX-Graph %s: %s\n",
              pACFProcess->RetProcessIdentifier().c_str(),
              pACFProcess->ErrorMessage().c_str());
#else
      fprintf(stderr,
              "\nError in APEX-Graph: could not connect output port %s\n",
              (char*)FAST9_GR_OUT);
#endif
      delete((ACF_Process_APU*)mpProcess);
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pACFProcess->ConnectIO(FAST9_GR_OUT, aDst);
    if(ACF_SUCCESS != acfStatus)
    {
#ifdef APEX2_EMULATE
      fprintf(stderr, "Error in APEX-Graph %s: %s\n",
              pACFProcess->RetProcessIdentifier().c_str(),
              pACFProcess->ErrorMessage().c_str());
#endif
      delete((ACF_Process_APU*)mpProcess);
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
  }
  else
  {
    // Connect IO with ROI
    acfStatus = pACFProcess->ConnectIO_ROI(FAST9_GR_IN_IMG, aSrc, arRoi.x,
                                           arRoi.y, arRoi.width, arRoi.height);

    if(ACF_SUCCESS != acfStatus)
    {
#ifdef APEX2_EMULATE
      fprintf(stderr, "\nError in APEX-Graph %s: %s\n",
              pACFProcess->RetProcessIdentifier().c_str(),
              pACFProcess->ErrorMessage().c_str());
#else
      fprintf(
          stderr,
          "\nError in APEX-Graph: could not connect input port %s with ROI\n",
          (char*)FAST9_GR_IN_IMG);

#endif

      delete((ACF_Process_APU*)mpProcess);
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pACFProcess->ConnectIO_ROI(FAST9_GR_OUT, aDst, arRoi.x, arRoi.y,
                                           arRoi.width, arRoi.height);
    if(ACF_SUCCESS != acfStatus)
    {
#ifdef APEX2_EMULATE
      fprintf(stderr, "Error in APEX-Graph %s: %s\n",
              pACFProcess->RetProcessIdentifier().c_str(),
              pACFProcess->ErrorMessage().c_str());
#else
      fprintf(
          stderr,
          "\nError in APEX-Graph: could not connect output port %s with ROI\n",
          (char*)FAST9_GR_OUT);

#endif
      delete((ACF_Process_APU*)mpProcess);
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
  }

  acfStatus = pACFProcess->ConnectIO(FAST9_GR_IN_THR, mThreshold);

  if(ACF_SUCCESS != acfStatus)
  {
#ifdef APEX2_EMULATE
    fprintf(stderr, "Error in APEX-Graph %s: %s\n",
            pACFProcess->RetProcessIdentifier().c_str(),
            pACFProcess->ErrorMessage().c_str());
#else
    fprintf(stderr,
            "\nError in APEX-Graph: could not connect input port %s with ROI\n",
            (char*)FAST9_GR_IN_THR);
#endif
    delete((ACF_Process_APU*)mpProcess);

    return APEXCV_ERROR_ACF_PROC_CONNECTIO;
  }

#if FAST_PERF_PROB
  time3 = HostTicks();
#endif

#if FAST_PERF_PROB
  printf(
      "\nAPEX-Fast9::Initialize for process type %d\n\t- initializations take "
      "%d "
      "usec\n\t- process creation and connect IO operations take %d usec\n",
      mProcessType, (int)HostTicksToSeconds(time2 - time1) * 1000000.0f,
      (int)HostTicksToSeconds(time3 - time2) * 1000000.0f);
#endif

#ifdef APEX2_EMULATE
  if(pACFProcess->WarningOccured())
  {
    fprintf(stderr, "Warning in APEX-Graph %s: %s\n",
            pACFProcess->RetProcessIdentifier().c_str(),
            pACFProcess->WarningMessages().c_str());
  }
#endif

  // Init is finished only here !
  mInit = 1;
  // Exit the function
  return APEXCV_SUCCESS;
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                                int32_t     aApexId)
{
  int32_t          acfStatus   = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess = (ACF_Process_APU*)mpProcess;

  // Apex APU Config can be set only after initialization is done
  if(0 == mInit)
  {
    return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }

  // Update the process & check for errors
  acfStatus = pACFProcess->SelectApuConfiguration(aApuConfig, aApexId);

  if(ACF_SUCCESS != acfStatus)
  {
    return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
  }

  // If here => APU Config updated successfully
  // => update mApuConfig and mApexId
  // mApuConfig = aApuConfig;
  mApexId = aApexId;

  return APEXCV_SUCCESS;
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::Process()
{
  int32_t          acfStatus   = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess = (ACF_Process_APU*)mpProcess;

  if(0 == mInit)
  {
    return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }

  acfStatus = pACFProcess->Start();

  if(ACF_SUCCESS != acfStatus)
  {
    return APEXCV_ERROR_ACF_PROC_EXEC;
  }

  acfStatus = pACFProcess->Wait();

  if(ACF_SUCCESS != acfStatus)
  {
    return APEXCV_ERROR_ACF_PROC_EXEC;
  }

#if FAST_PERF_PROB
  {
    printf("\nAPEX-Fast9::Process processType %d - mProcess profiling info\n",
           mProcessType);
    pACFProcess->DumpApuProfilingInfoToStdout();
    pACFProcess->DumpHostProfilingInfoToStdout();

    ACF_Process_PortInfo* lPortInfo = pACFProcess->RetPortInfo(FAST_GR_IN_IMG);
    if(lPortInfo != NULL)
    {
      printf("\nINPUT %d, %d\n",
             lPortInfo->mChunkWidth * lPortInfo->mE0.x *
                 RetDataTypeSizeInBytes(icp::DATATYPE(lPortInfo->mE0.dataType)),
             lPortInfo->mChunkHeight * lPortInfo->mE0.y);
    }
  }
#endif

  return APEXCV_SUCCESS;
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::ProcessNoPolling()
{
  int32_t          acfStatus   = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess = (ACF_Process_APU*)mpProcess;

  if(0 == mInit)
  {
    return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }

  acfStatus = pACFProcess->Start();

  if(ACF_SUCCESS != acfStatus)
  {
    return APEXCV_ERROR_ACF_PROC_EXEC;
  }

  return APEXCV_SUCCESS;
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::ProcessWait()
{
  int32_t          acfStatus   = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess = (ACF_Process_APU*)mpProcess;

  if(0 == mInit)
  {
    return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }

  acfStatus = pACFProcess->Wait();

  if(ACF_SUCCESS != acfStatus)
  {
    return APEXCV_ERROR_ACF_PROC_EXEC;
  }

  return APEXCV_SUCCESS;
}

/*************************************************************************************/
int Fast9::GetNrOfFeatures()
{
  int cnt = 0;

  if(mInit)
  {
    // Manually count the number of points in the fast out image that are > 0
    int      rowJump = mpOut->step[0];
    uint8_t* pFastImg =
        (uint8_t*)(mpOut->getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data);

    for(int y = 0; y < mImgHeight; y++)
    {
      uint8_t* pStartRow = &pFastImg[y * rowJump];

      for(int x = 0; x < mImgWidth; x++)
      {
        if(pStartRow[x] > 0)
        {
          cnt++;
        }
      }
    }
  }

  return cnt;
}

/*************************************************************************************/
APEXCV_LIB_RESULT Fast9::ReconnectIO(vsdk::SUMat& aDst,
                                     vsdk::SUMat& aSrc,
                                     const int   acThreshold,
                                     const bool  acNonMaxSuppression)
{
  int32_t          acfStatus = ACF_SUCCESS;
  ACF_Process_APU* pACFProcess;

#if FAST_PERF_PROB
  int64_t time1, time2;

  time1 = HostTicks();
#endif

  // Reset mInit flag
  mInit = 0;

  mThreshold.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at< uint8_t >(0u) =
      acThreshold;
  mNonMaxSupression = acNonMaxSuppression;
  mImgWidth         = aSrc.cols;
  mImgHeight        = aSrc.rows;
  mpOut             = &(aDst);
  bool reInit       = false;

  if(acNonMaxSuppression)
  {
    if(mProcessType != FAST9_PROCESS_IMG_NMS)
    {
      delete((ACF_Process_APU*)mpProcess);
      mpProcess    = new APU_FAST_NMS;
      mProcessType = FAST9_PROCESS_IMG_NMS;
      reInit       = true;
    }
  }
  else
  {
    if(mProcessType != FAST9_PROCESS_IMG)
    {
      delete((ACF_Process_APU*)mpProcess);
      mpProcess    = new APU_FAST;
      mProcessType = FAST9_PROCESS_IMG;
      reInit       = true;
    }
  }

  pACFProcess = (ACF_Process_APU*)mpProcess;

  // ReInitialize only if needed
  if(reInit)
  {
    acfStatus = pACFProcess->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      delete((ACF_Process_APU*)mpProcess);

      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
  }
  // These values are corelated with the Kernel Port Information
  acfStatus = pACFProcess->ConnectIO(FAST9_GR_IN_IMG, aSrc);

  if(ACF_SUCCESS != acfStatus)
  {
    delete((ACF_Process_APU*)mpProcess);

    return APEXCV_ERROR_ACF_PROC_CONNECTIO;
  }

  acfStatus = pACFProcess->ConnectIO(FAST9_GR_IN_THR, mThreshold);

  if(ACF_SUCCESS != acfStatus)
  {
    delete((ACF_Process_APU*)mpProcess);

    acfStatus = pACFProcess->ConnectIO(FAST9_GR_IN_THR, mThreshold);

    acfStatus = pACFProcess->ConnectIO(FAST9_GR_OUT, aDst);

    return APEXCV_ERROR_ACF_PROC_CONNECTIO;
  }

  acfStatus = pACFProcess->ConnectIO(FAST9_GR_OUT, aDst);

  if(ACF_SUCCESS != acfStatus)
  {
    delete((ACF_Process_APU*)mpProcess);

    return APEXCV_ERROR_ACF_PROC_CONNECTIO;
  }

#if FAST_PERF_PROB
  time2 = HostTicks();
#endif

#if FAST_PERF_PROB
  printf("\nAPEX-Fast9::ReconnectIO for process type %d %d usec\n",
         mProcessType, (int)HostTicksToSeconds(time2 - time1) * 1000000.0f);
#endif

  // Init is finished only here !
  mInit = 1;

  return APEXCV_SUCCESS;
}
} // namespace apex
