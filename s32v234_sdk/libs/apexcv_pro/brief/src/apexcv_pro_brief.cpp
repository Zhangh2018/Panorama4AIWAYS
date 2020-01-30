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
/*!*****************************************************************************
 *  \file ApexCV_brief.cpp
 *  \brief ApexCV Host wrapper for BRIEF algorithm
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <icp_data.h>
#include <vector>
#include "apexcv_pro_brief.h"
#include <oal.h>
#include <COMPUTE_BRIEF_DESCRIPTOR.hpp>

#if (BRIEF_PERF_PROB != 0)
#include <common_time_measure.h>
#include "math.h"

#ifdef __STANDALONE__
const uint32_t cTimeScale = 1;
#else
const uint32_t cTimeScale = 1000;
#endif

volatile int64_t gBriefTime[BRIEF_PROFILED_FUNC_CNT][PROBES_CNT];
#endif

#define QDRNT0 0
#define QDRNT1 1
#define QDRNT2 2
#define QDRNT3 3

namespace apexcv
{
  // Helper function - flush data from cache to ddr
  static void FlushUMat(vsdk::SUMat& aIn)
  {
    void *pData   = aIn.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
    uint32_t size = aIn.total() * aIn.elemSize();
    OAL_MemoryFlushAndInvalidate(pData, size);
  }

  // BRIEF works on patches of 48x48, to fit the data in APEX we split the patch
  // in 4 regionsm each regions is called a "quadrant"
  // The sampling pattern has pair of coordinates that are distributed all over
  // the main 48x48 patch
  // The following function maps the the sampling pattern coordinates into 
  // a valid range inside APEX
  static void MoveToFirstQuadrant(int aX, int aY, int aHalfPatch, int aQuadrant, int &aNewX, int &aNewY)
  {
    switch(aQuadrant)
    {
      case QDRNT0:
      {
        aNewX = aX + aHalfPatch;
        aNewY = aHalfPatch - aY;
        break;
      }

      case QDRNT1:
      {
        aNewX = aX;
        aNewY = aHalfPatch - aY;
        break;
      }

      case QDRNT2:
      {
        aNewX = aX + aHalfPatch;
        aNewY = -aY;
        break;
      }

      case QDRNT3:
      {
        aNewX = aX;
        aNewY = -aY;
        break;
      }

      default:
      {
        aNewX = 0;
        aNewY = 0;
        break;
      }
    }
  }
  
  static bool IsInBounds(int aX, int aLeft, int aRight)
  {
    if((aX >= aLeft) && (aX <= aRight))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // Helper function that returns the quadrant ID  
  static int GetQuadrant(int aX, int aY)
  {
    if(aX >= 0)
    {
      if(aY >= 0)
      {
        return QDRNT1;
      }
      else
      {
        return QDRNT3;
      }
    }
    else
    {
      if(aY >= 0)
      {
        return QDRNT0;
      }
      else
      {
        return QDRNT2;
      }
    }
  }

  Brief::Brief() : mInit(false),
                   mpProcess(NULL),
                   mApexId(0), 
                   mApuConfig(ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1),
                   mFilterPadding(0),
                   mVectorSize(32),
                   mPatchSize(48),
                   mNrOfKpnts(0),
                   mImgW(0),
                   mImgH(0),
                   mImgSpan(0)
                   
  {
  }

  Brief::~Brief()
  {
    if(NULL != mpProcess)
    {
      delete (COMPUTE_BRIEF_DESCRIPTOR *) mpProcess;
    }
  }

  APEXCV_LIB_RESULT CalcChunkOffsets(vsdk::SUMat &aInputImg,
                                     std::vector<unsigned int> &aKeypoints,
                                     int aPatchSize,
                                     int aVectSize,
                                     int filtPad,
                                     vsdk::SUMat &aChunkOffsets)
  {
    // Each keypoint is place inside a chunk
    // This helper function calculate the offset in bytes inside the image for each chunk   
    int span          = aInputImg.step[0];
    int halfPatch     = aPatchSize >> 1;
    int *pChunkOffset = (int *)aChunkOffsets.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

    if(pChunkOffset == NULL)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aPatchSize <= 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aKeypoints.size() == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aVectSize == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aVectSize > 32)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    const int X = 0;
    const int Y = 1;
    int patchOffsets[4][2];
    patchOffsets[0][X] = -halfPatch - filtPad;
    patchOffsets[0][Y] = -halfPatch - filtPad;

    patchOffsets[1][X] = -filtPad;
    patchOffsets[1][Y] = -halfPatch - filtPad;

    patchOffsets[2][X] = -halfPatch - filtPad;
    patchOffsets[2][Y] = -filtPad;

    patchOffsets[3][X] = -filtPad;
    patchOffsets[3][Y] = -filtPad;

    // Care must be taken to distribute the chunks between the CUs
    // The current brief algorithms needs to get one chunk after the other inside one CU
    // CU0 Processing: Uper Left, Upper Right, Lower Left and Lower Right    
    // Read the number of keypoints
    int nrOfKeypoints = (int)aKeypoints.size();

    for(int kpntId = 0; kpntId < nrOfKeypoints; kpntId += aVectSize)
    {
      for(int patchId = 0; patchId < 4; patchId++)
      {
        for(int blockId = 0; blockId < aVectSize; blockId++)
        {
          // Read data from database
          unsigned int data = aKeypoints[kpntId + blockId];

          // Calculate the offset in bytes for the keypoint
          int centerX = (int)((data >> 16) & 0xFFFF);
          int centerY = (int)((data >> 0) & 0xFFFF);
          int x       = centerX + patchOffsets[patchId][X];
          int y       = centerY + patchOffsets[patchId][Y];
          int offset  = x + y * span;
          *pChunkOffset++ = offset;
        }
      }
    }

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Brief::Initialize(vsdk::SUMat &aImage,
                                      std::vector<signed char> &aSmplPattern,
                                      std::vector<unsigned int> &aKeypoints,
                                      FilteringType aFilterType,
                                      unsigned char aDescrSizeBytes,
                                      unsigned char aBorderSize,
                                      vsdk::SUMat &aDescriptors)
  {
#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_INIT][Brief_Init_Start] = FSL_Ticks();
#endif

    FlushUMat(aImage);
    APEXCV_LIB_RESULT status;
    const int byteToBit   = 8;
    const int smplCluster = 4;
    std::vector<unsigned int> validKeypoints;
    const int nrOfPartitions = 4;
    mVectorSize            = 32;
    mFilterPadding         = 4;
    mPatchSize             = 56;
    mFilterType            = vsdk::SUMat(1, 1, VSDK_CV_8U);
    mDescriptorSizeInBytes = vsdk::SUMat(1, 1, VSDK_CV_8U);
    mPatchSizeInPixels     = vsdk::SUMat(1, 1, VSDK_CV_8U);
    mULPacketsCount        = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mURPacketsCount        = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mLLPacketsCount        = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mLRPacketsCount        = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mCount                 = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
    mSamplingPattern       = vsdk::SUMat(1, 2048, VSDK_CV_32SC1);

    mImgW    = aImage.cols;
    mImgH    = aImage.rows;
    mImgSpan = aImage.step[0];

    COMPUTE_BRIEF_DESCRIPTOR * pProcess = new COMPUTE_BRIEF_DESCRIPTOR;
    if(NULL == pProcess)
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    mpProcess = (void *) pProcess;
    
    if((aFilterType != NO_FILTERING) && (aFilterType != BOX_9x9_FILTERING) && (aFilterType != GAUSSIAN_9x9_FILTERING))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if((aDescrSizeBytes != 16) && (aDescrSizeBytes != 32) && (aDescrSizeBytes != 64))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    *((char *)mFilterType.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)                     = aFilterType;
    *((unsigned char *)mPatchSizeInPixels.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)     = mPatchSize;
    *((unsigned char *)mDescriptorSizeInBytes.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data) = aDescrSizeBytes;
    *((unsigned int *)mCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)                  = 0;

#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_INIT][Brief_Init_Mallocs] = abs(FSL_Ticks() - gBriefTime[BRIEF_INIT][Brief_Init_Start]) / cTimeScale;
    gBriefTime[BRIEF_INIT][Brief_Init_Start]   = FSL_Ticks();
#endif

    int halfPatch = (mPatchSize >> 1);

    // The sampling coordinates are valid only in a patch of 48x48
    // The values taken by the pairs of points are inside: [-24, 24]
    // These need to be split between the smaller chunks and adapted for each of them
    // Let's take as an example the coordinate pair:
    // A: x = -20 y = -20
    // B: x = 10 y = -1
    // Both of them are inside the Quadrant 0
    // Now, how these two pairs translte in Chunk Upper Left ?
    // A': x = 4 y = 4
    // B': x = 23 y = 14
    // The chunk axis are:
    // -------------------------------------> (X)
    // |
    // |
    // |
    // |
    // |
    // |
    // |
    // V (Y)
    
    // The main patch axis are:
    //                    (Y)
    //                    ^
    //                    |
    //                    |
    //                    |
    //                    |
    //                    |
    // -------------------------------------> (X)    
    //                    |
    //                    |
    //                    |
    //                    |
    //                    |

    for(int bit = 0; bit < aDescrSizeBytes * byteToBit; bit++)
    {
      // Each bit needs a comparison
      // Each comparison needs a set of two pairs of coordinates
      // bit = data(x0, y0) < data(x1, y1)
      // data(x0, y0) is always first !
      // Translation table: Tx = halfPatch + fltPad + Cx;
      // Translation table: Ty = halfPatch + fltPad - Cy;
      int x0 = (int)aSmplPattern[smplCluster * bit + 0];
      int y0 = (int)aSmplPattern[smplCluster * bit + 1];
      int x1 = (int)aSmplPattern[smplCluster * bit + 2];
      int y1 = (int)aSmplPattern[smplCluster * bit + 3];

      int quad0 = GetQuadrant(x0, y0);
      int quad1 = GetQuadrant(x1, y1);
      MoveToFirstQuadrant(x0, y0, halfPatch, quad0, x0, y0);
      MoveToFirstQuadrant(x1, y1, halfPatch, quad1, x1, y1);

      unsigned int data0 = ((x0 << 24) | (y0 << 16) | (1 << 15) | ((unsigned int)bit));
      unsigned int data1 = ((x1 << 24) | (y1 << 16) | ((unsigned int)bit));
      mPartions[quad0].push_back(data0);
      mPartions[quad1].push_back(data1);
    }

    unsigned int *pSmplPtrn = (unsigned int *)mSamplingPattern.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

    for(int partition = 0; partition < nrOfPartitions; partition++)
    {
      for(int idx = 0; idx < (int)(mPartions[partition].size()); idx++)
      {
        *pSmplPtrn++ = (unsigned int)mPartions[partition][idx];
      }
    }
    
    *((int *)(mULPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[0].size());
    *((int *)(mURPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[1].size());
    *((int *)(mLLPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[2].size());
    *((int *)(mLRPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[3].size());

    // Remove keypoints close to border
    for(int kpntId = 0; kpntId < (int)aKeypoints.size(); kpntId++)
    {
      unsigned int data = aKeypoints[kpntId];
      unsigned long x   = (data >> 16) & 0xFFFF;
      unsigned long y   = (data >> 0) & 0xFFFF;

      if(IsInBounds(x, aBorderSize, mImgW - aBorderSize) &&
         IsInBounds(y, aBorderSize, mImgH - aBorderSize))
      {
        validKeypoints.push_back(data);
      }
    }

    // Calculate chunk offsets
    mNrOfKpnts    = (int)validKeypoints.size();
    mChunkOffsets = vsdk::SUMat(1, mNrOfKpnts * mPatchIsSplitInFour, VSDK_CV_32SC1);
    status        = CalcChunkOffsets(aImage, validKeypoints, mPatchSize, mVectorSize, mFilterPadding, mChunkOffsets);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }

#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_INIT][Brief_Init_Sampling_Pattern_Parsing] = abs(FSL_Ticks() - gBriefTime[BRIEF_INIT][Brief_Init_Start]) / cTimeScale;
    gBriefTime[BRIEF_INIT][Brief_Init_Start]                    = FSL_Ticks();
#endif

    status = pProcess->Initialize();

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }
        
    // Connecting the graph connection to the exterior buffers
    FlushUMat(mFilterType);
    status = pProcess->ConnectIO("FILTER_TYPE", mFilterType);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mChunkOffsets);
    status = pProcess->ConnectIndirectInput("INPUT", aImage, mChunkOffsets);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mSamplingPattern);
    status = pProcess->ConnectIO("SMPL_PACKET", mSamplingPattern);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mULPacketsCount);
    status = pProcess->ConnectIO("NR_PACKETS_UL", mULPacketsCount);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mURPacketsCount);
    status = pProcess->ConnectIO("NR_PACKETS_UR", mURPacketsCount);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mLLPacketsCount);
    status = pProcess->ConnectIO("NR_PACKETS_LL", mLLPacketsCount);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mLRPacketsCount);
    status = pProcess->ConnectIO("NR_PACKETS_LR", mLRPacketsCount);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mDescriptorSizeInBytes);
    status = pProcess->ConnectIO("DESC_SIZE", mDescriptorSizeInBytes);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mPatchSizeInPixels);
    status = pProcess->ConnectIO("PATCH_SIZE", mPatchSizeInPixels);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    FlushUMat(mCount);
    status = pProcess->ConnectIO("COUNT", mCount);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    status = pProcess->ConnectIO("DESCRIPTOR_OUT", aDescriptors);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }

    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("COMPUTE_BRIEF_DESCRIPTOR", pProcess);
      lInfo.push_PortName("INPUT");
      mvInfoClusters.push_back(lInfo);
    }

    mInit = true;

#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_INIT][Brief_Init_Process_Init] = abs(FSL_Ticks() - gBriefTime[BRIEF_INIT][Brief_Init_Start]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Brief::ReconnectIO(vsdk::SUMat &aImage,
                                       std::vector<signed char> &aSmplPattern,
                                       std::vector<unsigned int> &aKeypoints,
                                       FilteringType aFilterType,
                                       unsigned char aDescrSizeBytes,
                                       unsigned char aBorderSize,
                                       vsdk::SUMat &aDescriptors)
  {
#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_RECONN][Brief_Reconn_Start] = FSL_Ticks();
#endif
    
    FlushUMat(aImage);    
    APEXCV_LIB_RESULT status;
    const int byteToBit      = 8;
    const int smplCluster    = 4;
    const int nrOfPartitions = 4;
    std::vector<unsigned int> validKeypoints;

    mApexId    = 0;
    mApuConfig = ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1;
    mImgW      = aImage.cols;
    mImgH      = aImage.rows;
    mImgSpan   = aImage.step[0];

    if((aFilterType != NO_FILTERING) && (aFilterType != BOX_9x9_FILTERING) && (aFilterType != GAUSSIAN_9x9_FILTERING))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if((aDescrSizeBytes != 16) && (aDescrSizeBytes != 32) && (aDescrSizeBytes != 64))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(mInit == true)
    {
      COMPUTE_BRIEF_DESCRIPTOR * pProcess = (COMPUTE_BRIEF_DESCRIPTOR *) mpProcess;
      
      // Clear the sampling partitions
      for(int partition = 0; partition < nrOfPartitions; partition++)
      {
        mPartions[partition].clear();
      }

      *((char *)mFilterType.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)                     = aFilterType;
      *((unsigned char *)mPatchSizeInPixels.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)     = mPatchSize;
      *((unsigned char *)mDescriptorSizeInBytes.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data) = aDescrSizeBytes;
      *((unsigned int *)mCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)                  = 0;

      int halfPatch = (mPatchSize >> 1);

      // Parse the coordinates into 4 bins
      for(int bit = 0; bit < aDescrSizeBytes * byteToBit; bit++)
      {
        // Each bit needs a comparison
        // Each comparison needs a set of two pairs of coordinates
        // bit = data(x0, y0) < data(x1, y1)
        // data(x0, y0) is always first !
        // Translation table: Tx = halfPatch + fltPad + Cx;
        // Translation table: Ty = halfPatch + fltPad - Cy;
        int x0 = (int)aSmplPattern[smplCluster * bit + 0];
        int y0 = (int)aSmplPattern[smplCluster * bit + 1];
        int x1 = (int)aSmplPattern[smplCluster * bit + 2];
        int y1 = (int)aSmplPattern[smplCluster * bit + 3];

        int quad0 = GetQuadrant(x0, y0);
        int quad1 = GetQuadrant(x1, y1);
        MoveToFirstQuadrant(x0, y0, halfPatch, quad0, x0, y0);
        MoveToFirstQuadrant(x1, y1, halfPatch, quad1, x1, y1);

        unsigned int data0 = ((x0 << 24) | (y0 << 16) | (1 << 15) | ((unsigned int)bit));
        unsigned int data1 = ((x1 << 24) | (y1 << 16) | ((unsigned int)bit));
        mPartions[quad0].push_back(data0);
        mPartions[quad1].push_back(data1);
      }

      unsigned int *pSmplPtrn = (unsigned int *)mSamplingPattern.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

      for(int partition = 0; partition < nrOfPartitions; partition++)
      {
        for(int idx = 0; idx < (int)(mPartions[partition].size()); idx++)
        {
          *pSmplPtrn++ = (unsigned int)mPartions[partition][idx];
        }
      }

      *((int *)(mULPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[0].size());
      *((int *)(mURPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[1].size());
      *((int *)(mLLPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[2].size());
      *((int *)(mLRPacketsCount.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)) = (int)(mPartions[3].size());

      // Remove keypoints close to border
      for(int kpntId = 0; kpntId < (int)aKeypoints.size(); kpntId++)
      {
        unsigned int data = aKeypoints[kpntId];
        unsigned long x   = (data >> 16) & 0xFFFF;
        unsigned long y   = (data >> 0) & 0xFFFF;

        if(IsInBounds(x, aBorderSize, mImgW - aBorderSize) &&
           IsInBounds(y, aBorderSize, mImgH - aBorderSize))
        {
          validKeypoints.push_back(data);
        }
      }

      // Calculate chunk offsets
      mNrOfKpnts    = (int)validKeypoints.size();
      mChunkOffsets = vsdk::SUMat(1, mNrOfKpnts * mPatchIsSplitInFour, VSDK_CV_32SC1);
      status        = CalcChunkOffsets(aImage, validKeypoints, mPatchSize, mVectorSize, mFilterPadding, mChunkOffsets);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
     
      // Connecting the graph connection to the exterior buffers
      FlushUMat(mFilterType);
      status = pProcess->ConnectIO("FILTER_TYPE", mFilterType);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mChunkOffsets);         
      status = pProcess->ConnectIndirectInput("INPUT", aImage, mChunkOffsets);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mSamplingPattern);
      status = pProcess->ConnectIO("SMPL_PACKET", mSamplingPattern);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mULPacketsCount);
      status = pProcess->ConnectIO("NR_PACKETS_UL", mULPacketsCount);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mURPacketsCount);
      status = pProcess->ConnectIO("NR_PACKETS_UR", mURPacketsCount);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mLLPacketsCount);
      status = pProcess->ConnectIO("NR_PACKETS_LL", mLLPacketsCount);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mLRPacketsCount);
      status = pProcess->ConnectIO("NR_PACKETS_LR", mLRPacketsCount);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mDescriptorSizeInBytes);
      status = pProcess->ConnectIO("DESC_SIZE", mDescriptorSizeInBytes);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mPatchSizeInPixels);
      status = pProcess->ConnectIO("PATCH_SIZE", mPatchSizeInPixels);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      FlushUMat(mCount);
      status = pProcess->ConnectIO("COUNT", mCount);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }

      status = pProcess->ConnectIO("DESCRIPTOR_OUT", aDescriptors);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_RECONN][Brief_Reconn_Run] = abs(FSL_Ticks() - gBriefTime[BRIEF_RECONN][Brief_Reconn_Start]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Brief::Process()
  {
#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_PROCESS][Brief_Process_Start] = FSL_Ticks();
#endif

    APEXCV_LIB_RESULT status;

    if(true == mInit)
    {
      COMPUTE_BRIEF_DESCRIPTOR * pProcess = (COMPUTE_BRIEF_DESCRIPTOR *) mpProcess;

      status = pProcess->SelectApuConfiguration(mApuConfig, mApexId);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }

      status = pProcess->Start();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }

      status = pProcess->Wait();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

#if (BRIEF_PERF_PROB != 0)
    gBriefTime[BRIEF_PROCESS][Brief_Process_Run] = abs(FSL_Ticks() - gBriefTime[BRIEF_PROCESS][Brief_Process_Start]) / cTimeScale;
#endif

    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Brief::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                                  int32_t aApexId)
  {
    APEXCV_LIB_RESULT status;
    
    if(ACF_APU_CFG__APU_0_CU_0_63_SMEM_0_3 == aApuConfig)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }

    if(mInit == true)
    {
      COMPUTE_BRIEF_DESCRIPTOR * pProcess = (COMPUTE_BRIEF_DESCRIPTOR *) mpProcess;
      mApuConfig = aApuConfig;
      mApexId    = aApexId;
      status     = pProcess->SelectApuConfiguration(aApuConfig, (int32_t)aApexId);

      if(status != APEXCV_SUCCESS)
      {
        return status;
      }
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    return APEXCV_SUCCESS;
  }
} /* namespace apexcv */
