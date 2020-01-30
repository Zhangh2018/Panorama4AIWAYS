/******************************************************************************
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015, Digital Media Professionals.
 * Copyright 2017 NXP
 * All Rights Reserved
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
 *****************************************************************************/

/*!****************************************************************************
 *  \file apexcv_lbp.cpp
 *  \brief ApexCV Host wrapper for LBP algorithm
 *****************************************************************************/

//***********************************************
// TOGGLE THIS MACRO TO ENABLE/DISABLE PROFILING
//***********************************************
#include <oal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "apexcv_pro_lbp.h"

// GENERATED ACF PROCESSES
#include <LBP_EXTRACT_DESCRIPTOR.hpp>
#include <LBP_COMPARE_DESCRIPTOR.hpp>

static const int acceptableChunkWidthIndirectInput[] =
{
  4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64
};

/*****************************************************************************/
namespace apexcv
{
  /*
    Default constructor
  */
  Lbp::Lbp():mInitStateTrain(0), 
             mInitStatePredict(0),
             mApuConfig(ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1), 
             mApexId(0),
             mpProcessExtDescTrain(nullptr),
             mpProcessExtDescPredict(nullptr),
             mpProcessCompareDesc(nullptr)
  {
  }

  /*
    Default destructor - the all the associated memory regions for the APEX
                         processes are deleted
  */
  Lbp::~Lbp()
  {
    if(mpProcessExtDescTrain)
    {
      delete (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescTrain;
    }

    if(mpProcessExtDescPredict)
    {
      delete (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescPredict;
    }

    if(mpProcessCompareDesc)
    {
      delete (LBP_COMPARE_DESCRIPTOR *) mpProcessCompareDesc;
    }    

  }

  /*
    The LBP helper library is organized in three main functions:
        1. Initialize
        2. ReConnectIO
        3. Process
    The api is very similar to the majority of the functions in APEXCV.
    LBP uses two processes: a. Trainer
                            b. Predictor                            
    LBP works by dividing the image in chunks, calculate the binary pattern
    and accumulate all the values in a chunk inside a historgram. This will form
    the feature vector.

    a. The training sequence is precalculated and read from a file
    b. The Predictor is doing the LBP + Histogram and the decision based on the
       training sequence
  */
  APEXCV_LIB_RESULT Lbp::InitializeTrain(vsdk::SUMat& aSrc,
                                         int aSrcWidth,
                                         int aSrcHeight,
                                         int aSrcNum,
                                         vsdk::SUMat& aDescriptor)
  {
    unsigned int i;
    int processWidth, processHeight;
    int chunkHeight, chunkWidth;
    int numCU;
    int32_t acfStatus;

    acfStatus = ACF_SUCCESS;

    // Check if Initialize has already been called
    if(0 != mInitStateTrain)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    // Check if arguments are valid
    // -> (1) Validate width and height
    if(aSrcWidth == 0 || aSrcHeight == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // -> (2) Validate data types
    if((aSrc.type() != VSDK_CV_8UC1) ||
       (aDescriptor.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescTrain = new LBP_EXTRACT_DESCRIPTOR;
    if(pProcessExtDescTrain == NULL)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mpProcessExtDescTrain = pProcessExtDescTrain; 

    processWidth  = (aSrcWidth - 2 * LBP_RADIUS) / LBP_GRID_X;
    processHeight = (aSrcHeight - 2 * LBP_RADIUS) / LBP_GRID_Y;
    chunkHeight   = processHeight + 2 * LBP_RADIUS;
    chunkWidth    = 0;

    // Find HW friendly chunk width
    for(i = 0; i < sizeof(acceptableChunkWidthIndirectInput) / sizeof(acceptableChunkWidthIndirectInput[0]); i++)
    {
      if((processWidth + 2 * LBP_RADIUS) <= acceptableChunkWidthIndirectInput[i])
      {
        chunkWidth = acceptableChunkWidthIndirectInput[i];
        break;
      }
    }

    // -> (3) Check if using a larger buffer is needed
    mTrainSrc = aSrc;

    if((processWidth * (LBP_GRID_X - 1) + chunkWidth) > aSrcWidth)
    {
      // Copy src image data to larger buffer to avoid memory leak
      int margin = (processWidth * (LBP_GRID_X - 1) + chunkWidth) - aSrcWidth;
      mTrainSrc =
        vsdk::SUMat(vsdk::SUMat(aSrc.rows + margin, aSrc.cols, VSDK_CV_8UC1),
                   vsdk::Rect(0, 0, aSrc.cols, aSrc.rows));

      if(!mTrainSrc.empty())
      {
        vsdk::SMat lTrainSrcMat = mTrainSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        vsdk::SMat srcMat       = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        uint8_t *pOriginal     = (uint8_t *)srcMat.data;
        uint8_t *pTrainSrc     = (uint8_t *)lTrainSrcMat.data;

        memcpy(pTrainSrc, pOriginal, sizeof(uint8_t) * aSrc.cols * aSrc.rows);
      }
      else
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }

    // Initialize process
    acfStatus = pProcessExtDescTrain->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    // Select APU configuration
    acfStatus = pProcessExtDescTrain->SelectApuConfiguration(mApuConfig, mApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    // -> (4) Check allocation for mTrainChunkOffset and mProcessSizeTrain
    numCU             = pProcessExtDescTrain->RetCuArrayWidth();
    mTrainChunkOffset = vsdk::SUMat(LBP_GRID_X * LBP_GRID_Y * aSrcNum / numCU, numCU, VSDK_CV_32SC1);
    mProcessSizeTrain = vsdk::SUMat(1, 1, VSDK_CV_16UC2);

    if(mTrainChunkOffset.empty() || mProcessSizeTrain.empty())
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    // Prepare to connect IC
    vsdk::SMat lChunkOffsetMat      = mTrainChunkOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    vsdk::SMat lProcessSizeTrainMat = mProcessSizeTrain.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

    int32_t *pChunkOffset  = (int32_t *)lChunkOffsetMat.data;
    uint16_t *pProcessSize = (uint16_t *)lProcessSizeTrainMat.data;

    for(int i = 0; i < aSrcNum; i++)
    {
      for(int h = 0; h < LBP_GRID_Y; h++)
      {
        for(int w = 0; w < LBP_GRID_X; w++)
        {
          pChunkOffset[i * LBP_GRID_Y * LBP_GRID_X + h * LBP_GRID_X + w] =
            i * aSrcWidth * aSrcHeight + h * processHeight * aSrcWidth + w * processWidth;
        }
      }
    }
    pProcessSize[0] = processWidth;
    pProcessSize[1] = processHeight;

    // Connect IO
    acfStatus = pProcessExtDescTrain->ConnectIndirectInput("INPUT", mTrainSrc, mTrainChunkOffset);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->ConnectIO("PROCESS_SIZE", mProcessSizeTrain);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->ConnectIO("OUTPUT", aDescriptor);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->SelectScenario("", 0, 0);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->OverridePortChunkSize("INPUT", chunkWidth, chunkHeight);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
    }
    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("LBP_EXTRACT_DESCRIPTOR", pProcessExtDescTrain);
      lInfo.push_PortName("INPUT");
      mvInfoClusters.push_back(lInfo);
    }
    // Init is finished only here!
    mInitStateTrain = 1;

    // Exit the function
    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::InitializePredict(vsdk::SUMat& aModel,
                                           int aModelNum,
                                           vsdk::SUMat& aSrc,
                                           int aSrcWidth,
                                           int aSrcHeight,
                                           vsdk::SUMat& aDescriptor,
                                           vsdk::SUMat& aClosestID,
                                           vsdk::SUMat& aDistance)
  {
    unsigned int i;
    int processWidth, processHeight;
    int chunkHeight, chunkWidth;
    int32_t acfStatus;
    int numCU;

    acfStatus = ACF_SUCCESS;

    // Check if Initialize has already been called
    if(0 != mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    // Check if arguments are valid
    // -> (1) Validate width and height
    if(aSrcWidth == 0 || aSrcHeight == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // -> (2) Validate data types
    if((aModel.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)) ||
       (aSrc.type() != VSDK_CV_8UC1) ||
       (aDescriptor.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)) ||
       (aClosestID.type() != VSDK_CV_16UC1) ||
       (aDistance.type() != VSDK_CV_32SC1))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescPredict = new LBP_EXTRACT_DESCRIPTOR;
    if(pProcessExtDescPredict == NULL)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }    
    mpProcessExtDescPredict = pProcessExtDescPredict; 

    processWidth  = (aSrcWidth - 2 * LBP_RADIUS) / LBP_GRID_X;
    processHeight = (aSrcHeight - 2 * LBP_RADIUS) / LBP_GRID_Y;
    chunkHeight   = processHeight + 2 * LBP_RADIUS;
    chunkWidth    = 0;

    // Find HW friendly chunk width
    for(i = 0; i < sizeof(acceptableChunkWidthIndirectInput) / sizeof(acceptableChunkWidthIndirectInput[0]); i++)
    {
      if((processWidth + 2 * LBP_RADIUS) <= acceptableChunkWidthIndirectInput[i])
      {
        chunkWidth = acceptableChunkWidthIndirectInput[i];
        break;
      }
    }

    // Update source buffer
    mPredictSrc = aSrc;

    // -> (3) Check if using a larger buffer is needed
    if((processWidth * (LBP_GRID_X - 1) + chunkWidth) > aSrcWidth)
    {
      // Copy aSrc image data to larger buffer to avoid memory leak
      int margin = (processWidth * (LBP_GRID_X - 1) + chunkWidth) - aSrcWidth;
      mPredictSrc =
        vsdk::SUMat(vsdk::SUMat(aSrc.rows + margin, aSrc.cols, VSDK_CV_8UC1),
                   vsdk::Rect(0, 0, aSrc.cols, aSrc.rows));

      if(mPredictSrc.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }

      vsdk::SMat lPredictSrcMat = mPredictSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      vsdk::SMat srcMat         = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      uint8_t *pOriginal       = (uint8_t *)srcMat.data;
      uint8_t *pPredictSrc     = (uint8_t *)lPredictSrcMat.data;

      memcpy(pPredictSrc, pOriginal, sizeof(uint8_t) * aSrc.cols * aSrc.rows);
    }

    /*----------------------- mProcessExtDescPredict -----------------------*/
    // Initialize process
    acfStatus = pProcessExtDescPredict->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    // Select APU configuration
    acfStatus = pProcessExtDescPredict->SelectApuConfiguration(mApuConfig, mApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    // -> (4) Check allocation for mChunkOffset and mProcessSizePredict
    numCU               = pProcessExtDescPredict->RetCuArrayWidth();
    mChunkOffset        = vsdk::SUMat(LBP_GRID_X * LBP_GRID_Y / numCU, numCU, VSDK_CV_32SC1);
    mProcessSizePredict = vsdk::SUMat(1, 1, VSDK_CV_16UC2);
    mTest               = vsdk::SUMat(((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU) * 2,
                                     numCU, VSDK_CV_MAKETYPE(VSDK_CV_8U, 128));

    if(mChunkOffset.empty() || mProcessSizePredict.empty())
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    // Prepare to connect IO
    vsdk::CustomPointerSUMat lModel(aModelNum * ((LBP_GRID_X *LBP_GRID_Y + numCU - 1) / numCU) * 2,
                                   numCU,
                                   VSDK_CV_MAKETYPE(VSDK_CV_8U, 128), (uint8_t*)aModel.u->handle, (uint8_t*)0);
    lModel.rows    = ((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU) * aModelNum;
    lModel.step[0] = numCU * 128;
    lModel.step[2] = 2;

    vsdk::CustomPointerSUMat lTest(((LBP_GRID_X *LBP_GRID_Y + numCU - 1) / numCU) * 2,
                                  numCU,
                                  VSDK_CV_MAKETYPE(VSDK_CV_8U, 128), (uint8_t*)aDescriptor.u->handle, (uint8_t*)0);
    lTest.rows    = ((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU);
    lTest.step[0] = numCU * 128;
    lTest.step[2] = 2;

    vsdk::SMat lChunkOffsetMat        = mChunkOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    vsdk::SMat lProcessSizePredictMat = mProcessSizePredict.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    int32_t *pChunkOffset            = (int32_t *)lChunkOffsetMat.data;
    uint16_t *pProcessSize           = (uint16_t *)lProcessSizePredictMat.data;

    for(int h = 0; h < LBP_GRID_Y; h++)
    {
      for(int w = 0; w < LBP_GRID_X; w++)
      {
        pChunkOffset[h * LBP_GRID_X + w] = h * processHeight * aSrcWidth + w * processWidth;
      }
    }
    pProcessSize[0] = processWidth;
    pProcessSize[1] = processHeight;

    // Connect IO
    acfStatus = pProcessExtDescPredict->SelectScenario("PROCESS_SIZE", 1, 1);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
    }

    acfStatus = pProcessExtDescPredict->OverridePortChunkSize("INPUT", chunkWidth, chunkHeight);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
    }

    acfStatus = pProcessExtDescPredict->ConnectIndirectInput("INPUT", mPredictSrc, mChunkOffset);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescPredict->ConnectIO("PROCESS_SIZE", mProcessSizePredict);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescPredict->ConnectIO("OUTPUT", aDescriptor);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    LBP_COMPARE_DESCRIPTOR * pProcessCompareDesc = new LBP_COMPARE_DESCRIPTOR;
    if(pProcessCompareDesc == NULL)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mpProcessCompareDesc = pProcessCompareDesc; 
    
    /*----------------------- mProcessCompareDesc -----------------------*/
    // Initialize process
    acfStatus = pProcessCompareDesc->Initialize();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }

    // Select APU configuration
    acfStatus = pProcessCompareDesc->SelectApuConfiguration(mApuConfig, mApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    // Connect IO
    acfStatus = pProcessCompareDesc->ConnectIO("INPUT_MODEL", lModel);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    if(numCU < 64)
    {
      acfStatus = pProcessCompareDesc->SelectScenario("INPUT_MODEL", 1, 2);
    }
    else
    {
      acfStatus = pProcessCompareDesc->SelectScenario("INPUT_MODEL", 1, 1);
    }

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("INPUT_TEST", lTest);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("CLOSEST_ID", aClosestID);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("DISTANCE", aDistance);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("LBP_COMPARE_DESCRIPTOR", pProcessCompareDesc);
      lInfo.push_PortName("INPUT_TEST");
      mvInfoClusters.push_back(lInfo);
    }

    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("LBP_EXTRACT_DESCRIPTOR", pProcessExtDescPredict);
      lInfo.push_PortName("INPUT");
      mvInfoClusters.push_back(lInfo);
    }

    
    // Init is finished only here!
    mInitStatePredict = 1;

    // Exit the function
    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ReconnectTrainIO(vsdk::SUMat& aSrc,
                                          int aSrcWidth,
                                          int aSrcHeight,
                                          int aSrcNum,
                                          vsdk::SUMat& aDescriptor)
  {
    int numCU;
    int processWidth, processHeight;
    int chunkHeight, chunkWidth;
    unsigned int i;
    int32_t acfStatus;

    // Check if initialized
    if(!mInitStateTrain)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    // Check if arguments are valid
    if(aSrcWidth == 0 || aSrcHeight == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Check input data types
    if((aSrc.type() != VSDK_CV_8UC1) ||
       (aDescriptor.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
            
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescTrain = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescTrain;

    processWidth  = (aSrcWidth - 2 * LBP_RADIUS) / LBP_GRID_X;
    processHeight = (aSrcHeight - 2 * LBP_RADIUS) / LBP_GRID_Y;
    chunkHeight   = processHeight + 2 * LBP_RADIUS;
    chunkWidth    = 0;

    // Find HW friendly chunk width
    for(i = 0; i < sizeof(acceptableChunkWidthIndirectInput) / sizeof(acceptableChunkWidthIndirectInput[0]); i++)
    {
      if((processWidth + 2 * LBP_RADIUS) <= acceptableChunkWidthIndirectInput[i])
      {
        chunkWidth = acceptableChunkWidthIndirectInput[i];
        break;
      }
    }
	
	// Update train source buffer
	mTrainSrc = aSrc;

    // Check if using a larger buffer is needed
    if((processWidth * (LBP_GRID_X - 1) + chunkWidth) > aSrcWidth)
    {
      // Copy src image data to larger buffer to avoid memory leak
      int margin = (processWidth * (LBP_GRID_X - 1) + chunkWidth) - aSrcWidth;
      mTrainSrc = vsdk::SUMat(vsdk::SUMat(aSrc.rows + margin, aSrc.cols, VSDK_CV_8UC1),
                             vsdk::Rect(0, 0, aSrc.cols, aSrc.rows));

      if(!mTrainSrc.empty())
      {
        vsdk::SMat lTrainSrcMat = mTrainSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        vsdk::SMat srcMat       = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        uint8_t *pOriginal     = (uint8_t *)srcMat.data;
        uint8_t *pTrainSrc     = (uint8_t *)lTrainSrcMat.data;

        memcpy(pTrainSrc, pOriginal, sizeof(uint8_t) * aSrc.cols * aSrc.rows);
      }
      else
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }

    // Check allocation for mTrainChunkOffset
    numCU             = pProcessExtDescTrain->RetCuArrayWidth();
    mTrainChunkOffset = vsdk::SUMat(LBP_GRID_X * LBP_GRID_Y * aSrcNum / numCU, numCU, VSDK_CV_32SC1);

    if(mTrainChunkOffset.empty())
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    // Prepare to connect IO
    {
      vsdk::SMat lChunkOffsetMat      = mTrainChunkOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      vsdk::SMat lProcessSizeTrainMat = mProcessSizeTrain.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

      int32_t *pChunkOffset  = (int32_t *)lChunkOffsetMat.data;
      uint16_t *pProcessSize = (uint16_t *)lProcessSizeTrainMat.data;

      for(int i = 0; i < aSrcNum; i++)
      {
        for(int h = 0; h < LBP_GRID_Y; h++)
        {
          for(int w = 0; w < LBP_GRID_X; w++)
          {
            pChunkOffset[i * LBP_GRID_Y * LBP_GRID_X + h * LBP_GRID_X + w] =
              i * aSrcWidth * aSrcHeight + h * processHeight * aSrcWidth + w * processWidth;
          }
        }
      }
      pProcessSize[0] = processWidth;
      pProcessSize[1] = processHeight;
    }

    // Connect IO
    acfStatus = pProcessExtDescTrain->OverridePortChunkSize("INPUT", chunkWidth, chunkHeight);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
    }

    acfStatus = pProcessExtDescTrain->ConnectIndirectInput("INPUT", mTrainSrc, mTrainChunkOffset);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->ConnectIO("PROCESS_SIZE", mProcessSizeTrain);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescTrain->ConnectIO("OUTPUT", aDescriptor);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    // If here => reconnect is successful
    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ReconnectPredictIO(vsdk::SUMat& aModel,
                                            int aModelNum,
                                            vsdk::SUMat& aSrc,
                                            int aSrcWidth,
                                            int aSrcHeight,
                                            vsdk::SUMat& aDescriptor,
                                            vsdk::SUMat& aClosestID,
                                            vsdk::SUMat& aDistance)
  {
    int32_t acfStatus;
    int processWidth, processHeight;
    int chunkHeight, chunkWidth;
    int numCU;

    // Check if initialized
    if(!mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    // Check if arguments are valid
    if(aSrcWidth == 0 || aSrcHeight == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Check input data types
    if((aModel.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)) ||
       (aSrc.type() != VSDK_CV_8UC1) ||
       (aDescriptor.type() != VSDK_CV_MAKETYPE(VSDK_CV_8U, 128)) ||
       (aClosestID.type() != VSDK_CV_16UC1) ||
       (aDistance.type() != VSDK_CV_32SC1))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescPredict = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescPredict;

    processWidth  = (aSrcWidth - 2 * LBP_RADIUS) / LBP_GRID_X;
    processHeight = (aSrcHeight - 2 * LBP_RADIUS) / LBP_GRID_Y;
    chunkHeight   = processHeight + 2 * LBP_RADIUS;
    chunkWidth    = 0;

    // Find HW friendly chunk width
    for(unsigned int i = 0; i < sizeof(acceptableChunkWidthIndirectInput) / sizeof(acceptableChunkWidthIndirectInput[0]); i++)
    {
      if((processWidth + 2 * LBP_RADIUS) <= acceptableChunkWidthIndirectInput[i])
      {
        chunkWidth = acceptableChunkWidthIndirectInput[i];
        break;
      }
    }

    // Update source buffer
    mPredictSrc = aSrc;

    // Check if using a larger buffer is needed
    if((processWidth * (LBP_GRID_X - 1) + chunkWidth) > aSrcWidth)
    {
      // Copy src image data to larger buffer to avoid memory leak
      int margin = (processWidth * (LBP_GRID_X - 1) + chunkWidth) - aSrcWidth;
      mPredictSrc =
        vsdk::SUMat(vsdk::SUMat(aSrc.rows + margin, aSrc.cols, VSDK_CV_8UC1),
                   vsdk::Rect(0, 0, aSrc.cols, aSrc.rows));

      if(mPredictSrc.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }

      vsdk::SMat srcMat         = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      vsdk::SMat lPredictSrcMat = mPredictSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      uint8_t *pOriginal       = (uint8_t *)srcMat.data;
      uint8_t *pPredictSrc     = (uint8_t *)lPredictSrcMat.data;

      memcpy(pPredictSrc, pOriginal, sizeof(uint8_t) * aSrc.cols * aSrc.rows);
    }

    // Prepare to connect IO
    numCU        = pProcessExtDescPredict->RetCuArrayWidth();
    mChunkOffset = vsdk::SUMat((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU, numCU, VSDK_CV_32SC1);

    vsdk::CustomPointerSUMat lModel(aModelNum * ((LBP_GRID_X *LBP_GRID_Y + numCU - 1) / numCU) * 2,
                                   numCU, VSDK_CV_MAKETYPE(VSDK_CV_8U, 128), (uint8_t*)aModel.u->handle, (uint8_t*)0);
    lModel.rows    = ((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU) * aModelNum;
    lModel.step[0] = numCU * 128;
    lModel.step[2] = 2;

    vsdk::CustomPointerSUMat lTest(((LBP_GRID_X *LBP_GRID_Y + numCU - 1) / numCU) * 2,
                                  numCU, VSDK_CV_MAKETYPE(VSDK_CV_8U, 128), (uint8_t*)aDescriptor.u->handle, (uint8_t*)0);
    lTest.rows    = ((LBP_GRID_X * LBP_GRID_Y + numCU - 1) / numCU);
    lTest.step[0] = numCU * 128;
    lTest.step[2] = 2;

    vsdk::SMat lChunkOffsetMat      = mChunkOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    vsdk::SMat lProcessSizeTrainMat = mProcessSizeTrain.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

    int32_t *pChunkOffset  = (int32_t *)lChunkOffsetMat.data;
    uint16_t *pProcessSize = (uint16_t *)lProcessSizeTrainMat.data;

    for(int h = 0; h < LBP_GRID_Y; h++)
    {
      for(int w = 0; w < LBP_GRID_X; w++)
      {
        pChunkOffset[h * LBP_GRID_X + w] = h * processHeight * aSrcWidth + w * processWidth;
      }
    }
    pProcessSize[0] = processWidth;
    pProcessSize[1] = processHeight;

    // Connect IO
    acfStatus = pProcessExtDescPredict->OverridePortChunkSize("INPUT", chunkWidth, chunkHeight);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_OVERRIDEPORTCHUNKSIZE;
    }

    acfStatus = pProcessExtDescPredict->ConnectIndirectInput("INPUT", mPredictSrc, mChunkOffset);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescPredict->ConnectIO("PROCESS_SIZE", mProcessSizePredict);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessExtDescPredict->ConnectIO("OUTPUT", aDescriptor);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    
    LBP_COMPARE_DESCRIPTOR * pProcessCompareDesc = (LBP_COMPARE_DESCRIPTOR *) mpProcessCompareDesc;

    acfStatus |= pProcessCompareDesc->ConnectIO("INPUT_MODEL", lModel);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    if(numCU < 64)
    {
      acfStatus = pProcessCompareDesc->SelectScenario("INPUT_MODEL", 1, 2);
    }
    else
    {
      acfStatus = pProcessCompareDesc->SelectScenario("INPUT_MODEL", 1, 1);
    }

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("INPUT_TEST", lTest);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("CLOSEST_ID", aClosestID);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    acfStatus = pProcessCompareDesc->ConnectIO("DISTANCE", aDistance);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    // If here => reconnect is successful
    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ProcessTrain()
  {
    int32_t acfStatus = ACF_SUCCESS;

    if(!mInitStateTrain)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescTrain = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescTrain;

    pProcessExtDescTrain->FlushAndInvalidateAllIO();
    acfStatus = pProcessExtDescTrain->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    acfStatus = pProcessExtDescTrain->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ProcessPredict()
  {
    int32_t acfStatus = ACF_SUCCESS;

    if(!mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescPredict = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescPredict;

    pProcessExtDescPredict->FlushAndInvalidateAllIO();
    acfStatus = pProcessExtDescPredict->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    acfStatus = pProcessExtDescPredict->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }
    
    LBP_COMPARE_DESCRIPTOR * pProcessCompareDesc = (LBP_COMPARE_DESCRIPTOR *) mpProcessCompareDesc;

    pProcessCompareDesc->FlushAndInvalidateAllIO();
    acfStatus = pProcessCompareDesc->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    acfStatus = pProcessCompareDesc->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ProcessPredictExtract()
  {
    int32_t acfStatus = ACF_SUCCESS;

    if(!mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescPredict = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescPredict;

    pProcessExtDescPredict->FlushAndInvalidateAllIO();

    acfStatus = pProcessExtDescPredict->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    acfStatus = pProcessExtDescPredict->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::ProcessPredictCompare()
  {
    int32_t acfStatus = ACF_SUCCESS;

    if(!mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    LBP_COMPARE_DESCRIPTOR * pProcessCompareDesc = (LBP_COMPARE_DESCRIPTOR *) mpProcessCompareDesc;
    
    pProcessCompareDesc->FlushAndInvalidateAllIO();

    acfStatus = pProcessCompareDesc->Start();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    acfStatus = pProcessCompareDesc->Wait();

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    return APEXCV_SUCCESS;
  }

  /***************************************************************************/
  APEXCV_LIB_RESULT Lbp::SelectApexCore(int32_t aApexId)
  {
    int32_t acfStatus = ACF_SUCCESS;

    // Apex APU Config can be set only after initialization is done
    if(0 == mInitStateTrain)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if(0 == mInitStatePredict)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescTrain = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescTrain;
    LBP_EXTRACT_DESCRIPTOR * pProcessExtDescPredict = (LBP_EXTRACT_DESCRIPTOR *) mpProcessExtDescPredict;
    LBP_COMPARE_DESCRIPTOR * pProcessCompareDesc = (LBP_COMPARE_DESCRIPTOR *) mpProcessCompareDesc;

    // Update each of the processes & check for errors
    acfStatus = pProcessExtDescTrain->SelectApuConfiguration(mApuConfig, aApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    acfStatus = pProcessExtDescPredict->SelectApuConfiguration(mApuConfig, aApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    acfStatus = pProcessCompareDesc->SelectApuConfiguration(mApuConfig, aApexId);

    if(ACF_SUCCESS != acfStatus)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    // If here => APU Config updated successfully
    // => update mApexId
    mApexId = aApexId;

    return APEXCV_SUCCESS;
  }
} // namespace apexcv
