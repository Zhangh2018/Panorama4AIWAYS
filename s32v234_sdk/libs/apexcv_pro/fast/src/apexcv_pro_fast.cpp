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

#include <stdio.h>

#include <apexcv_pro_fast.h>

#ifndef APEX2_EMULATE
  #include <icp_data.h>
  #include <APU_FAST.hpp>
  #include <APU_FAST_NMS.hpp>
  #include <APU_FAST_SERIALIZED.hpp>
#else
  #include "apu_lib.hpp"
  #include "apu_extras.hpp"
  #include "acf_lib.hpp"

  #include "apu_fast_apu_process_desc.hpp"
  #include "apu_fast_nms_apu_process_desc.hpp"
  #include "apu_fast_serialized_apu_process_desc.hpp"

REGISTER_PROCESS_TYPE(APU_FAST, apu_fast_apu_process_desc);
REGISTER_PROCESS_TYPE(APU_FAST_NMS, apu_fast_nms_apu_process_desc);
REGISTER_PROCESS_TYPE(APU_FAST_SERIALIZED, apu_fast_serialized_apu_process_desc);

#endif // #ifndef APEX2_EMULATE

#include <oal.h>

#define FAST_PERF_PROB 0
#if FAST_PERF_PROB
#include <common_time_measure.h>
#ifdef __STANDALONE__
const uint32_t cTimeScale = 1; // In standalone values are already in microseconds
#else
const uint32_t cTimeScale = 1000; // In linux values are in nanoseconds
#endif
#endif

/***************************************************************************************/
namespace apexcv
{
  /*************************************************************************************/
  Fast::Fast()
    : mInit(0), mNonMaxSupression(false), mOutIsAList(false), mProcessType(FAST_NONE), mImgWidth(0), mImgHeight(0), mpProcess(nullptr)
  {
  }

  /*************************************************************************************/
  Fast::~Fast()
  {
    if(mInit)
    {
      if(nullptr != mpProcess)
      {
        delete ((ACF_Process_APU *)mpProcess);
      }
      mInit = 0;
    }
  }
  
  /*************************************************************************************/
  
  static APEXCV_LIB_RESULT ConnectExternalBuffers(ACF_Process_APU *apProcess,
                                       unsigned int aProcessType,
                                       vsdk::SUMat& aIn,
                                       vsdk::SUMat& aOut,
                                       vsdk::SUMat& aCircumference,
                                       vsdk::SUMat& aThreshold,
                                       vsdk::SUMat& aOutCounter,
                                       vsdk::SUMat& aOutMaxSize)
  {
    APEXCV_LIB_RESULT status;

    // Connect IO
    status = apProcess->ConnectIO("INPUT", aIn);

    if(ACF_SUCCESS != status)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    status = apProcess->ConnectIO("OUTPUT", aOut);

    if(ACF_SUCCESS != status)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = apProcess->ConnectIO("CIRCUMFERENCE", aCircumference);

    if(ACF_SUCCESS != status)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    status = apProcess->ConnectIO("THRESHOLD", aThreshold);

    if(ACF_SUCCESS != status)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    if(aProcessType == FAST_PROCESS_SERIALIZED)
    {
      // These values are corelated with the Kernel Port Information
      const int maxOutSize = FAST_MAX_LIST_ELEMENTS;

      // Fill in the parameters
      aOutMaxSize.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = maxOutSize;
      aOutCounter.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = 0;

      status = apProcess->ConnectIO("OUT_COUNTER", aOutCounter);

      if(ACF_SUCCESS != status)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      status = apProcess->ConnectIO("OUT_MAX_SIZE", aOutMaxSize);

      if(ACF_SUCCESS != status)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
    }

    return APEXCV_SUCCESS;
  }

  /*************************************************************************************/
  APEXCV_LIB_RESULT Fast::Initialize(vsdk::SUMat& aDst,
                                     vsdk::SUMat& aSrc,
                                     const int acThreshold,
                                     const bool acNonMaxSuppression,
                                     const int acCircumference,
                                     const bool acOutIsList)
  {
    int32_t acfStatus = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess;

#if FAST_PERF_PROB
    int64_t time1, time2, time3;

    time1 = FSL_Ticks();
#endif

    // Check if Initialize has already been called
    if(0 != mInit)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    // Check if arguments are valid
    //     -> acNonMaxSuppression and acOutIsList cannot both be set
    if(!(acNonMaxSuppression == false || acOutIsList == false))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    //-> Only 8bit unsigned images are supported
    if(VSDK_CV_8UC1 != aSrc.type())
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Start the initialization
    // Default is Normal Fast
    mProcessType = FAST_PROCESS_IMG;

    // Fill in the class parameters
    mpOut             = &(aDst);
    mNonMaxSupression = (acNonMaxSuppression);
    mImgWidth         = (aSrc.cols);
    mImgHeight        = (aSrc.rows);
    mOutIsAList       = (acOutIsList);
    mThreshold        = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    mCircumference    = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    mOutMaxSize       = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mCounter          = vsdk::SUMat(1, 1, VSDK_CV_32SC1);

    // Set default hw configuration
    mApuConfig = ACF_APU_CFG__DEFAULT;
    mApexId    = 0;

    // Update the values
    mThreshold.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint8_t>(0u)     = acThreshold;
    mCircumference.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint8_t>(0u) = acCircumference;
    mOutMaxSize.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = FAST_MAX_LIST_ELEMENTS;
    mCounter.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u)    = 0;

    // Initialize process type
    if(acNonMaxSuppression == true)
    {
      mProcessType = FAST_PROCESS_IMG_NMS;
    }
    else if(acOutIsList == true)
    {
      mProcessType = FAST_PROCESS_SERIALIZED;
    }
    mpProcess = NULL;

#if FAST_PERF_PROB
    time2 = FSL_Ticks();
#endif

    // Allocate process memory based on the user specifications
    switch(mProcessType)
    {
      case FAST_PROCESS_IMG:
      {
        mpProcess = new APU_FAST;
      }
      break;

      case FAST_PROCESS_SERIALIZED:
      {
        mpProcess = new APU_FAST_SERIALIZED;
      }
      break;

      case FAST_PROCESS_IMG_NMS:
      {
        mpProcess = new APU_FAST_NMS;
      }
      break;

      default:

        return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Point to the current process
    pACFProcess = (ACF_Process_APU *)mpProcess;

    // Initialize process
    acfStatus = pACFProcess->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("FAST_PROCESS", pACFProcess);
      lInfo.push_PortName("INPUT");
      mvInfoClusters.push_back(lInfo);
    }

    // Select APU config
    acfStatus = pACFProcess->SelectApuConfiguration(mApuConfig, mApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    // Connect the APEX graph with the exterior buffers 
    acfStatus = ConnectExternalBuffers(pACFProcess,
                                       mProcessType,
                                       aSrc,
                                       aDst,
                                       mCircumference,
                                       mThreshold,
                                       mCounter,
                                       mOutMaxSize);

    if(APEXCV_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return acfStatus;
    }

#if FAST_PERF_PROB
    time3 = FSL_Ticks();
#endif

#if FAST_PERF_PROB
    printf("\nLbp::Initialize for process type %d\n\t- initializations take %d usec\n\t- process creation and connect IO operations take %d usec\n",
           mProcessType,
           (int)(time2 - time1) / cTimeScale,
           (int)(time3 - time2) / cTimeScale);
#endif

    // Init is finished only here !
    mInit = 1;

    // Exit the function
    return APEXCV_SUCCESS;
  }

  /*************************************************************************************/
  APEXCV_LIB_RESULT Fast::SelectApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
  {
    int32_t acfStatus            = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess = (ACF_Process_APU *)mpProcess;

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
    mApuConfig = aApuConfig;
    mApexId    = aApexId;

    return APEXCV_SUCCESS;
  }

  /*************************************************************************************/
  APEXCV_LIB_RESULT Fast::Process()
  {
    // Loads the code inside APEX and starts the processing
    int32_t acfStatus            = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess = (ACF_Process_APU *)mpProcess;

    if(0 == mInit)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    // Process Go !
    acfStatus = pACFProcess->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    // Wait for the process to finish
    acfStatus = pACFProcess->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    return APEXCV_SUCCESS;
  }

  /*************************************************************************************/
  APEXCV_LIB_RESULT Fast::ProcessNoPolling()
  {
    // Loads the code inside APEX and starts the processing
    // Can be used to start the processing without waiting

    int32_t acfStatus            = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess = (ACF_Process_APU *)mpProcess;

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
  APEXCV_LIB_RESULT Fast::ProcessWait()
  {
    // Checks to see if the process has finished
    int32_t acfStatus            = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess = (ACF_Process_APU *)mpProcess;

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
  int Fast::GetNrOfFeatures()
  {
    // Helper function that will output the number of detected keypoints
    // In case of the OUTPUT being an image this method can be very slow
    int cnt = 0;

    if(mInit)
    {      
      if(mOutIsAList)
      {
        int32_t *pCnt = (int32_t *)mCounter.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
        cnt = *pCnt;
      }
      else
      {
        // Manually count the number of points in the fast out image that are > 0
        // Very slow !
        int rowJump       = mpOut->step[0];
        uint8_t *pFastImg = (uint8_t *)(mpOut->getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data);

        for(int y = 0; y < mImgHeight; y++)
        {
          uint8_t *pStartRow = &pFastImg[y * rowJump];

          for(int x = 0; x < mImgWidth; x++)
          {
            if(pStartRow[x] > 0)
            {
              cnt++;
            }
          }
        }
      }
    }

    return cnt;
  }

  /*************************************************************************************/
  APEXCV_LIB_RESULT Fast::ReconnectIO(vsdk::SUMat& aDst,
                                      vsdk::SUMat& aSrc,
                                      const int acThreshold,
                                      const bool acNonMaxSuppression,
                                      const int acCircumference,
                                      const int acOutIsList)
  {
    int32_t acfStatus = ACF_SUCCESS;
    ACF_Process_APU *pACFProcess;

#if FAST_PERF_PROB
    int64_t time1, time2;

    time1 = FSL_Ticks();
#endif

    // Reset mInit flag
    mInit = 0;
    
    // Reconnect all the necessary buffers for FAST to work
    mThreshold.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint8_t>(0u)     = acThreshold;
    mCircumference.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<uint8_t>(0u) = acCircumference;
    mNonMaxSupression                                                        = acNonMaxSuppression;
    mOutIsAList                                                              = acOutIsList;
    mImgWidth                                                                = aSrc.cols;
    mImgHeight                                                               = aSrc.rows;
    mpOut                                                                    = &(aDst);
    
    if(acNonMaxSuppression)
    {
      if(mProcessType != FAST_PROCESS_IMG_NMS)
      {
        // Deallocate old process memory
        delete ((ACF_Process_APU *)mpProcess);
        
        // Allocate a new chunk of memory
        mpProcess    = new APU_FAST_NMS;

        // Store the process type
        mProcessType = FAST_PROCESS_IMG_NMS;
      }
    }
    else
    {
      if(acOutIsList == true)
      {
        if(mProcessType != FAST_PROCESS_SERIALIZED)
        {
          // Deallocate old process memory
          delete ((ACF_Process_APU *)mpProcess);

          // Allocate a new chunk of memory
          mpProcess    = new APU_FAST_SERIALIZED;

          // Store the process type
          mProcessType = FAST_PROCESS_SERIALIZED;
        }
      }
      else
      {
        if(mProcessType != FAST_PROCESS_IMG)
        {
          // Deallocate old process memory
          delete ((ACF_Process_APU *)mpProcess);

          // Allocate a new chunk of memory
          mpProcess    = new APU_FAST;

          // Store the process type
          mProcessType = FAST_PROCESS_IMG;
        }
      }
    }

    // Cast to APU Process
    pACFProcess = (ACF_Process_APU *)mpProcess;

    // ReInitialize only if needed
    acfStatus = pACFProcess->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    // Connecting the graph to the external buffers
    acfStatus = ConnectExternalBuffers(pACFProcess,
                                       mProcessType,
                                       aSrc,
                                       aDst,
                                       mCircumference,
                                       mThreshold,
                                       mCounter,
                                       mOutMaxSize);
    if(APEXCV_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return acfStatus;
    }      

    // Select APU config
    acfStatus = pACFProcess->SelectApuConfiguration(mApuConfig, mApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      delete ((ACF_Process_APU *)mpProcess);

      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

#if FAST_PERF_PROB
    time2 = FSL_Ticks();
#endif

#if FAST_PERF_PROB
    printf("\nLbp::ReconnectIO for process type %d %d usec\n",
           mProcessType,
           (int)(time2 - time1) / cTimeScale);
#endif

    // Init is finished only here !
    mInit = 1;

    return APEXCV_SUCCESS;
  }
} // namespace apex
