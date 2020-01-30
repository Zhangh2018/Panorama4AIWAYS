/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2016-2017 NXP Semiconductor;
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

#include <oal.h>
#include <algorithm>
#include <vector>
#include "apexcv_pro_orb.h"
#include "apexcv_pro_fast.h"
#include <APU_ORB_RBRIEF.hpp>
#include <APU_ORB_ORIENTATION.hpp>

/* This is the standardized sampling pattern for Orb, this will work with descriptor
* sizes of a maximum 256 bits 
* If bigger descriptors are needed plese generate a new sampling pattern and plug-it into
* APEX.
* https://github.com/opencv/opencv/blob/master/modules/features2d/src/orb.cpp :: line 375
*/

static int8_t samplingPattern[1024] =
{
	8, -3, 9, 5, 4, 2, 7, -12, -11, 9, -8, 2, 7, -12, 12, -13, 
	2, -13, 2, 12, 1, -7, 1, 6, -2, -10, -2, -4, -13, -13, -11, -8, 
	-13, -3, -12, -9, 10, 4, 11, 9, -13, -8, -8, -9, -11, 7, -9, 12, 
	7, 7, 12, 6, -4, -5, -3, 0, -13, 2, -12, -3, -9, 0, -7, 5, 
	12, -6, 12, -1, -3, 6, -2, 12, -6, -13, -4, -8, 11, -13, 12, -8, 
	4, 7, 5, 1, 5, -3, 10, -3, 3, -7, 6, 12, -8, -7, -6, -2, 
	-2, 11, -1, -10, -13, 12, -8, 10, -7, 3, -5, -3, -4, 2, -3, 7, 
	-10, -12, -6, 11, 5, -12, 6, -7, 5, -6, 7, -1, 1, 0, 4, -5, 
	9, 11, 11, -13, 4, 7, 4, 12, 2, -1, 4, 4, -4, -12, -2, 7, 
	-8, -5, -7, -10, 4, 11, 9, 12, 0, -8, 1, -13, -13, -2, -8, 2, 
	-3, -2, -2, 3, -6, 9, -4, -9, 8, 12, 10, 7, 0, 9, 1, 3, 
	7, -5, 11, -10, -13, -6, -11, 0, 10, 7, 12, 1, -6, -3, -6, 12, 
	10, -9, 12, -4, -13, 8, -8, -12, -13, 0, -8, -4, 3, 3, 7, 8, 
	5, 7, 10, -7, -1, 7, 1, -12, 3, -10, 5, 6, 2, -4, 3, -10, 
	-13, 0, -13, 5, -13, -7, -12, 12, -13, 3, -11, 8, -7, 12, -4, 7, 
	6, -10, 12, 8, -9, -1, -7, -6, -2, -5, 0, 12, -12, 5, -7, 5, 
	3, -10, 8, -13, -7, -7, -4, 5, -3, -2, -1, -7, 2, 9, 5, -11, 
	-11, -13, -5, -13, -1, 6, 0, -1, 5, -3, 5, 2, -4, -13, -4, 12, 
	-9, -6, -9, 6, -12, -10, -8, -4, 10, 2, 12, -3, 7, 12, 12, 12, 
	-7, -13, -6, 5, -4, 9, -3, 4, 7, -1, 12, 2, -7, 6, -5, 1, 
	-13, 11, -12, 5, -3, 7, -2, -6, 7, -8, 12, -7, -13, -7, -11, -12, 
	1, -3, 12, 12, 2, -6, 3, 0, -4, 3, -2, -13, -1, -13, 1, 9, 
	7, 1, 8, -6, 1, -1, 3, 12, 9, 1, 12, 6, -1, -9, -1, 3, 
	-13, -13, -10, 5, 7, 7, 10, 12, 12, -5, 12, 9, 6, 3, 7, 11, 
	5, -13, 6, 10, 2, -12, 2, 3, 3, 8, 4, -6, 2, 6, 12, -13, 
	9, -12, 10, 3, -8, 4, -7, 9, -11, 12, -4, -6, 1, 12, 2, -8, 
	6, -9, 7, -4, 2, 3, 3, -2, 6, 3, 11, 0, 3, -3, 8, -8, 
	7, 8, 9, 3, -11, -5, -6, -4, -10, 11, -5, 10, -5, -8, -3, 12, 
	-10, 5, -9, 0, 8, -1, 12, -6, 4, -6, 6, -11, -10, 12, -8, 7, 
	4, -2, 6, 7, -2, 0, -2, 12, -5, -8, -5, 2, 7, -6, 10, 12, 
	-9, -13, -8, -8, -5, -13, -5, -2, 8, -8, 9, -13, -9, -11, -9, 0, 
	1, -8, 1, -2, 7, -4, 9, 1, -2, 1, -1, -4, 11, -6, 12, -11, 
	-12, -9, -6, 4, 3, 7, 7, 12, 5, 5, 10, 8, 0, -4, 2, 8, 
	-9, 12, -5, -13, 0, 7, 2, 12, -1, 2, 1, 7, 5, 11, 7, -9, 
	3, 5, 6, -8, -13, -4, -8, 9, -5, 9, -3, -3, -4, -7, -3, -12, 
	6, 5, 8, 0, -7, 6, -6, 12, -13, 6, -5, -2, 1, -10, 3, 10, 
	4, 1, 8, -4, -2, -2, 2, -13, 2, -12, 12, 12, -2, -13, 0, -6, 
	4, 1, 9, 3, -6, -10, -3, -5, -3, -13, -1, 1, 7, 5, 12, -11, 
	4, -2, 5, -7, -13, 9, -9, -5, 7, 1, 8, 6, 7, -8, 7, 6, 
	-7, -4, -7, 1, -8, 11, -7, -8, -13, 6, -12, -8, 2, 4, 3, 9, 
	10, -5, 12, 3, -6, -5, -6, 7, 8, -3, 9, -8, 2, -12, 2, 8, 
	-11, -2, -10, 3, -12, -13, -7, -9, -11, 0, -10, -5, 5, -3, 11, 8, 
	-2, -13, -1, 12, -1, -8, 0, 9, -13, -11, -12, -5, -10, -2, -10, 11, 
	-3, 9, -2, -13, 2, -3, 3, 2, -9, -13, -4, 0, -4, 6, -3, -10, 
	-4, 12, -2, -7, -6, -11, -4, 9, 6, -3, 6, 11, -13, 11, -5, 5, 
	11, 11, 12, 6, 7, -5, 12, -2, -1, 12, 0, 7, -4, -8, -3, -2, 
	-7, 1, -6, 7, -13, -12, -8, -13, -7, -2, -6, -8, -8, 5, -6, -9, 
	-5, -1, -4, 5, -13, 7, -8, 10, 1, 5, 5, -13, 1, 0, 10, -13, 
	9, 12, 10, -1, 5, -8, 10, -9, -1, 11, 1, -13, -9, -3, -6, 2, 
	-1, -10, 1, 12, -13, 1, -8, -10, 8, -11, 10, -6, 2, -13, 3, -6, 
	7, -13, 12, -9, -10, -10, -5, -7, -10, -8, -8, -13, 4, -6, 8, 5, 
	3, 12, 8, -13, -4, 2, -3, -3, 5, -13, 10, -12, 4, -13, 5, -1, 
	-9, 9, -4, 3, 0, 3, 3, -9, -12, 1, -6, 1, 3, 2, 4, -8, 
	-10, -10, -10, 9, 8, -13, 12, 12, -8, -12, -6, -5, 2, 2, 3, 7, 
	10, 6, 11, -8, 6, 8, 8, -12, -7, 10, -6, 5, -3, -9, -3, 9, 
	-1, -13, -1, 5, -3, -7, -3, 4, -8, -2, -8, 3, 4, 2, 12, 12, 
	2, -5, 3, 11, 6, -9, 11, -13, 3, -1, 7, 12, 11, -1, 12, 4, 
	-3, 0, -3, 6, 4, -11, 4, 12, 2, -4, 2, 1, -10, -6, -8, 1, 
	-13, 7, -11, 1, -13, 12, -11, -13, 6, 0, 11, -13, 0, -1, 1, 4, 
	-13, 3, -9, -2, -9, 8, -6, -3, -13, -6, -8, -2, 5, -9, 8, 10, 
	2, 7, 3, -9, -1, -6, -1, -1, 9, 5, 11, -2, 11, -3, 12, -8, 
	3, 0, 3, 5, -1, 4, 0, 10, 3, -6, 4, 5, -13, 0, -10, 5, 
	5, 8, 12, 11, 8, 9, 9, -6, 7, -4, 8, -12, -10, 4, -10, 9, 
	7, 3, 12, 4, 9, -7, 10, -2, 7, 0, 12, -2, -1, -6, 0, -11};

namespace apexcv
{ 
  // Helper function used to sort the keypoints
  static bool CmpGreaterFeature(Orb::Corner aLeft, Orb::Corner aRight)
  {
    return(aLeft.strength > aRight.strength);
  }

  static inline void flushUMat(vsdk::SUMat& aTarget)
  {
    // Helper function used to flush the cache data into the DDR
    uint8_t * pData = (uint8_t *)aTarget.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
    int size        = (int)(aTarget.total() * aTarget.elemSize());
    OAL_MemoryFlushAndInvalidate((void *)pData, size);
    return;
  }

  // Helper function that calculates Harris corner metric
  float
  HarrisScore(uint8_t *__restrict__ apCenter,
              int aStep,
              const int acBlockSize,
              const float acHarrisK)
  {
    const int radius = (acBlockSize >> 1);
    const float scale = 1.0f / ((1 << 2) * acBlockSize * 255.f);
    const float sclPow2 = scale * scale;
    const float sclPow4 = (sclPow2 * sclPow2);  
    register int a = 0, b = 0, c = 0;

    for(int y = -radius; y < radius; y++)
    {
      // Point to the center of the image row
      const uint8_t *__restrict__ pStartRow = (uint8_t * __restrict__) & apCenter[y * aStep];

      for(int x = -radius; x < radius; x++)
      {        
        // Row access
        const uint8_t *__restrict__ ptr = (const uint8_t * __restrict__) & pStartRow[x];
        
        // Compute dx and dy
        // Data from first line
        int mat00 = ptr[-aStep - 1];
        int mat01 = ptr[-aStep + 0];
        int mat02 = ptr[-aStep + 1];

        // Data from the second line
        int mat10 = ptr[-1];
        int mat12 = ptr[+1];

        // Data from the third line
        int mat20 = ptr[aStep - 1];
        int mat21 = ptr[aStep + 0];
        int mat22 = ptr[aStep + 1];

        // Image derivative for both x and y
        int dx = (mat02 - mat00) + 2 * (mat12 - mat10) + (mat22 - mat20);
        int dy = (mat20 - mat00) + 2 * (mat21 - mat01) + (mat22 - mat02);

        // Compute the products of the derivation at each pixel
        a += (dx * dx);
        b += (dy * dy);
        c += (dx * dy);
      }
    }

    // H(x, y) = | Sx*x(x, y) Sx*y(x, y) | in our code = | a c |  
    //           | Sy*x(x, y) Sy*y(x, y) |               | c b |
    // Harris Cornerness = Det(H) - k * trace(H) ^ 2
    float score = ((float)a * b - (float)c * c - acHarrisK * ((float)a + b) * ((float)a + b)) * sclPow4;

    // Return the Harris Corner Metric
    return score;
  }

  APEXCV_LIB_RESULT CalcChunkOffsets(vsdk::SUMat &aInputImg, std::vector<Orb::Corner> &aKeypoints, int aNrOfkeypoints, int aPatchSize, vsdk::SUMat &aChunkOffsets)
  {
    // Each keypoint is place inside a chunk
    // This helper function calculate the offset in bytes inside the image for each chunk        
    uint16_t span         = aInputImg.step[0];
    uint16_t halfPatch    = aPatchSize >> 1;
    int32_t *pChunkOffset = (int32_t *)aChunkOffsets.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
    int32_t sizeBytes     = aChunkOffsets.total() * aChunkOffsets.elemSize();

    // Clear the buffer    
    memset((uint8_t *)pChunkOffset, 0, sizeBytes);

    // Basic error checks
    if(aPatchSize <= 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aNrOfkeypoints <= 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aKeypoints.size() == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Loop all the keypoints
    for(int kpntId = 0; kpntId < aNrOfkeypoints; kpntId++)
    {
       // Read the <x, y> coordinates of the keypoints
      Orb::Corner data = aKeypoints[kpntId];

      // Calculate the offset in bytes for the keypoint
      uint16_t x     = data.x;
      uint16_t y     = data.y;
      uint16_t newX  = (x - halfPatch);
      uint16_t newY  = (y - halfPatch);
      int32_t offset = newX + newY * span;
      pChunkOffset[kpntId] = offset;
    }

    OAL_MemoryFlushAndInvalidate((void *)pChunkOffset, sizeBytes);

    return APEXCV_SUCCESS;
  }

  Orb::Orb() :
      mpChunkOffsets(nullptr),
      mpAngles(nullptr),
      mpDescriptorCount(nullptr),
      mpBinPatternOut(nullptr),
      mInit(false),
      mpProcessCalcOrientation(nullptr),
      mpProcessCalcRbrief(nullptr)
  {
  }

  Orb::~Orb()
  {
    // Class destructor
    if(NULL != mpProcessCalcRbrief)
    {
      delete[] (APU_ORB_RBRIEF *) mpProcessCalcRbrief;      
    }

    if (NULL != mpProcessCalcOrientation)
    {
      delete[](APU_ORB_ORIENTATION *) mpProcessCalcOrientation;
    }

    if (NULL != mpBinPatternOut)
    {
      delete[] mpBinPatternOut;
    }

    if (NULL != mpDescriptorCount)
    {
      delete[] mpDescriptorCount;
    }

    if (NULL != mpAngles)
    {
      delete[] mpAngles;
    }

    if (NULL != mpChunkOffsets)
    {
      delete[] mpChunkOffsets;
    }

  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

APEXCV_LIB_RESULT Orb::Create(unsigned char aApexId,
                              unsigned char aNrOfThreads,                                              
                              unsigned int aBorderSize,
                              unsigned int aPatchSize,
                              unsigned int aRadius,
                              unsigned int aDescriptorSizeInBytes,
                              unsigned int aFastCircumference,
                              unsigned int aFastThreshold,
                              float aHarrisK,
                              unsigned int aNrOfKeypoints)
  {    
    // Basic error checks
    if(mInit != false)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }    

    if(aBorderSize < aRadius)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aPatchSize < (2 * aRadius))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aDescriptorSizeInBytes == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aNrOfKeypoints == 0)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aNrOfThreads > 2)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(aApexId >= 2)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
   
    APEXCV_LIB_RESULT status;
    const int pairXY                 = 2;
    const int nrOfPairsForComparison = 2;
    const int smpltPtrnApexDmaBytes  = 2048;    
    const int maxNrOfKeypoints = FAST_MAX_LIST_ELEMENTS;    

    // Class parameters
    mApexId                          = aApexId;
    mNrOfThreads                     = aNrOfThreads;    
    mDistanceFromBorder              = aBorderSize;
    mFastCirc                        = aFastCircumference;
    mFastTh                          = aFastThreshold;
    mHarrisK                         = aHarrisK;
    mNrOfFeatures                    = aNrOfKeypoints;

    int descriptorSizeInBits         = aDescriptorSizeInBytes * 8;
    int samplingPoints               = descriptorSizeInBits * nrOfPairsForComparison * pairXY;

    // Allocate the buffer to maximum
    // The sampling buffer is tagged as static_fixed inside the ACF
    msmplBitPattern = vsdk::SUMat(1, smpltPtrnApexDmaBytes, VSDK_CV_8SC1);
    if (!msmplBitPattern.empty())
    {
      // Fill-in the buffer that APEX will "see" as the sampling bit pattern
      int8_t * pData = (int8_t *)msmplBitPattern.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

      // Clear the buffer
      memset(pData, 0, smpltPtrnApexDmaBytes);

      for (int idx = 0; idx < samplingPoints; idx++)
      {
        pData[idx] = samplingPattern[idx];
      }
    }

    // Dynamic allocation of APU processes
    APU_ORB_ORIENTATION * pProcessCalcOrientation = new APU_ORB_ORIENTATION[mNrOfThreads];

    if(NULL == pProcessCalcOrientation)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    
    // Store to the class variables
    mpProcessCalcOrientation = (void *) pProcessCalcOrientation;

    {
        for(int32_t i=0;i<mNrOfThreads;i++)
        {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            char lTemp[50];
            sprintf(lTemp, "%s_THREAD[%d]","APU_ORB_ORIENTATION",i);
            lInfo.set_ACF(lTemp, (void*) &pProcessCalcOrientation[i]);
            lInfo.push_PortName("INPUT");
            mvInfoClusters.push_back(lInfo);
        }
    }

    APU_ORB_RBRIEF * pProcessCalcRbrief = new APU_ORB_RBRIEF[mNrOfThreads];

    if(NULL == pProcessCalcRbrief)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    // Store to the class variables
    mpProcessCalcRbrief = (void *) pProcessCalcRbrief;

    {
        for(int32_t i=0;i<mNrOfThreads;i++)
        {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            char lTemp[50];
            sprintf(lTemp, "%s_THREAD[%d]","APU_ORB_RBRIEF",i);
            lInfo.set_ACF(lTemp, (void *) &pProcessCalcRbrief[i]);
            lInfo.push_PortName("INPUT");
            mvInfoClusters.push_back(lInfo);
        }
    }

    // Dynamic allocation of the data structures linked to the parallel processes
    mpChunkOffsets    = new vsdk::SUMat[mNrOfThreads];
    mpAngles          = new vsdk::SUMat[mNrOfThreads];
    mpDescriptorCount = new vsdk::SUMat[mNrOfThreads];
    mpBinPatternOut   = new vsdk::SUMat[mNrOfThreads];

    // Allocating memory and filling in the parameters that will be sent to APEX
    mPatchSizeInPixels = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
    mRadiusInPixels    = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
    mDescrSizeBytes    = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
    mFastCornerList    = vsdk::SUMat(1, maxNrOfKeypoints, VSDK_CV_32SC1);

    mDescrSizeBytes.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int8_t>(0u)    = (int8_t)aDescriptorSizeInBytes;
    flushUMat(mDescrSizeBytes);
    mRadiusInPixels.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int8_t>(0u)    = (int8_t)aRadius;
    flushUMat(mRadiusInPixels);
    mPatchSizeInPixels.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int8_t>(0u) = (int8_t)aPatchSize;
    flushUMat(mPatchSizeInPixels);
    
    // Splitting the work between the APEXes
    int nrOfElements     = mNrOfFeatures / mNrOfThreads;
    int descrSizeInBytes = (int)(mDescrSizeBytes.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int8_t>(0u));
  
    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      mpChunkOffsets[threadId]    = vsdk::SUMat(1, nrOfElements, VSDK_CV_32SC1);       
      mpAngles[threadId]          = vsdk::SUMat(1, nrOfElements, VSDK_CV_16UC1);
      mpDescriptorCount[threadId] = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      mpBinPatternOut[threadId]   = vsdk::SUMat(1, nrOfElements * descrSizeInBytes, VSDK_CV_8UC1);

      // IC Orientation Initialization
      status = pProcessCalcOrientation[threadId].Initialize();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      }     
      
      // Connecting the external buffers to the kernel graphs
      status = pProcessCalcOrientation[threadId].ConnectIO("PATCH_SIZE", mPatchSizeInPixels);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      // Connecting the external buffers to the kernel graphs
      status = pProcessCalcOrientation[threadId].ConnectIO("RADIUS", mRadiusInPixels);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      // Connecting the external buffers to the kernel graphs
      status = pProcessCalcOrientation[threadId].ConnectIO("OUTPUT", mpAngles[threadId]);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }    

      // Initializing the Rotated Brief for APEX
      status = pProcessCalcRbrief[threadId].Initialize();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      }
      
      // BIT_PATTERN - Connecting the sampling pattern to the APEX
      // This buffer will indicate to the hw what samples to pick from the input chunk
      status = pProcessCalcRbrief[threadId].ConnectIO("BIT_PATTERN", msmplBitPattern);
      
      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      // ORIENTATION - Connecting the orientation of each chunk to the APEX
      status = pProcessCalcRbrief[threadId].ConnectIO("ORIENTATION", mpAngles[threadId]);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      // PATCH_SIZE - Connecting the dimension of the patch
      status = pProcessCalcRbrief[threadId].ConnectIO("PATCH_SIZE", mPatchSizeInPixels);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      // PATCH_SIZE - Connecting the size of the descriptor in bytes
      status = pProcessCalcRbrief[threadId].ConnectIO("DESCR_SIZE_B", mDescrSizeBytes);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
   
      // COUNT_DESCR - Connecting the number of output descriptors
      status = pProcessCalcRbrief[threadId].ConnectIO("COUNT_DESCR", mpDescriptorCount[threadId]);
      
      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
    }    
    
    // Class is initialized correctly only if the code arrives at this point
    mInit     = true;
    
    // Everythig is fine until this point
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Orb::Detect(vsdk::SUMat &aInImg)
  {
    APEXCV_LIB_RESULT status = EXIT_FAILURE;
    mInGrayScale             = aInImg;
    mImgW                    = aInImg.cols;
    mImgH                    = aInImg.rows;
    mImgSpan                 = aInImg.step[0];    
    unsigned char *lpBaseImg                = aInImg.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
    int patchSize;
    
    if(!mPatchSizeInPixels.empty())
    {
      patchSize          = (int)(*((int8_t *)mPatchSizeInPixels.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data));
    }
    else
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if(mDistanceFromBorder >= (unsigned int)aInImg.cols)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if(mDistanceFromBorder >= (unsigned int)aInImg.rows)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    // Initializing Fast9 detector
    status = mFast9.Initialize(mFastCornerList,
                               aInImg,
                               mFastTh,
                               false,
                               mFastCirc,
                               true);

    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    // Configure APEX to run with 64 CUs
    status = mFast9.SelectApuConfiguration(ACF_APU_CFG__APU_0_CU_0_63_SMEM_0_3, mApexId);

    if(status != APEXCV_SUCCESS)
    {
      return status;
    }
    // Start crunching the data in the APEX
    status = mFast9.Process();
       
    if(status != APEXCV_SUCCESS)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

    // Return the number of detected keypoints    
    mDetectedKeypoints = mFast9.GetNrOfFeatures();
    
    // mDetectedKeypoints is to allocate intermediate buffers so it needs to exit 
    if(mDetectedKeypoints == 0)
    {
      return APEXCV_SUCCESS;
    }

    if(mDetectedKeypoints < 0)
    {
      // This is the special case when FAST9 has an internal error when running in serialize mode!
      // Check FAST9 APU kernel code to see the status!
      printf("Fast9 went crazy !\n Status is: %d\n", mDetectedKeypoints);
      return APEXCV_ERROR_FAILURE;
    }

    APU_ORB_ORIENTATION * pProcessCalcOrientation = (APU_ORB_ORIENTATION *) mpProcessCalcOrientation;

    // FAST9 outputs a list of coordinates in the form of <x, y>
    uint16_t *pOutPackedList = (uint16_t *)(mFastCornerList.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data);

    // Constant value
    const int harrisWindowSize = 7; 
    const int pair = 2;

    // The keypoint array is not sorted at the beginning
    bool isSorted = false;

    for(int idx = 0; idx < mDetectedKeypoints; idx++)
    {
      // Read the keypoint coordinates
      unsigned int x = pOutPackedList[pair * idx + 0];
      unsigned int y = pOutPackedList[pair * idx + 1];

      // Filter out the keypoints that are outside of the borderline
      if((x > mDistanceFromBorder) &&
         (y > mDistanceFromBorder) &&
         (x < (mImgW - mDistanceFromBorder)) &&
         (y < (mImgH - mDistanceFromBorder)))
      {
        // Place the pointer to the center of the keypoint
        uint8_t *__restrict__ pCenter = (uint8_t * __restrict__) & lpBaseImg[y * mImgSpan + x];
        
        // This variable will be inserted in the keypoint array
        Orb::Corner keypoint;

        keypoint.x        = x;
        keypoint.y        = y;
        keypoint.strength = HarrisScore(pCenter, mImgSpan, harrisWindowSize, mHarrisK);

        // Don't keep more keypoints than you need
        if(mKeypoints.size() < mNrOfFeatures)
        {
          // Place the values into the vector in no particular order
          mKeypoints.push_back(keypoint);          
        }
        else
        {
          // "mKeypoints" is sorted only once when its size is equal to the desired nr of mKeypoints
          if(isSorted == false)
          {
            // Sort the mKeypoints
            std::sort(mKeypoints.begin(), mKeypoints.end(), CmpGreaterFeature);
            isSorted = true;
          }

          // The new keypoint is compared to the last element in the vector which is also the smallest
          // It's insert in the queue only if CmpGreaterFeature() returns bool true
          if(CmpGreaterFeature(keypoint, mKeypoints.back()))
          {
            // Remove the last element
            mKeypoints.pop_back();

            // Place the new element in the vector
            mKeypoints.insert(std::lower_bound(mKeypoints.begin(), mKeypoints.end(), keypoint, CmpGreaterFeature), keypoint);
          }
        }
      }
    }

    // If fast detects a keypoint, find it's corresponding "cornerness" from Harris
    // Orb needs the best FAST points based on the Harris score
    // Sort only if fast did not find enough mKeypoints
    if(isSorted == false)
    {
      std::sort(mKeypoints.begin(), mKeypoints.end(), CmpGreaterFeature);
    }     

    // After detection update the number of keypoints
    mNrOfFeatures = mKeypoints.size();
    
    if(0 == mNrOfFeatures)
    {
      return APEXCV_SUCCESS;
    }
    
    // Calculate chunk offsets
    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      ACF_APU_CFG apuConfig = ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1;

      if(threadId)
      {
        apuConfig = ACF_APU_CFG__APU_1_CU_32_63_SMEM_2_3;
      }    

      // Creating a subvector of keypoints for each APU
      int nrOfElem = (mNrOfFeatures / mNrOfThreads);
      int start    = threadId * nrOfElem;
      int stop     = start + nrOfElem;
      std::vector<Orb::Corner> keypointSubVector(mKeypoints.begin() + start, mKeypoints.begin() + stop);

      // Calculating the chunk offsets for each of the detected keypoints
      status = CalcChunkOffsets(mInGrayScale, keypointSubVector, nrOfElem, patchSize, mpChunkOffsets[threadId]);
      
      // Exit with error
      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }

      // APUs must run only with 32 active CUs because of DMA size
      status = pProcessCalcOrientation[threadId].SelectApuConfiguration(apuConfig, (int32_t) mApexId);

      // Exit with error
      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }

      status = pProcessCalcOrientation[threadId].ConnectIndirectInput("INPUT", aInImg, mpChunkOffsets[threadId]);
      
      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      // Start all processes
      status = pProcessCalcOrientation[threadId].Start();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    // Wait for the APU processes to finish
    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      status = pProcessCalcOrientation[threadId].Wait();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    // Exit the function
    return APEXCV_SUCCESS;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  APEXCV_LIB_RESULT Orb::SetBriefSamplingPattern(vsdk::SUMat &aInBitPattern)
  { 
    // Helper function used to connect the sampling pattern to the apex graph
    if(mInit == true)
    {      
      APEXCV_LIB_RESULT status;
      APU_ORB_RBRIEF * pProcessCalcRbrief = (APU_ORB_RBRIEF *) mpProcessCalcRbrief;
      
      for(int threadId = 0; threadId < mNrOfThreads; threadId++)
      {
        status = pProcessCalcRbrief[threadId].ConnectIO("BIT_PATTERN", aInBitPattern);
        if(status != APEXCV_SUCCESS)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
      }    
    }
    else
    {
      // Exit with error
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    // Exit the function
    return APEXCV_SUCCESS; 
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  APEXCV_LIB_RESULT Orb::Compute(vsdk::SUMat &aInSmoothedImg,
                                 vsdk::SUMat &aOutDescriptors)
  {
    APEXCV_LIB_RESULT status;
    APU_ORB_RBRIEF * pProcessCalcRbrief = (APU_ORB_RBRIEF *) mpProcessCalcRbrief;

    if ((0 == mNrOfFeatures) || (0 == mDetectedKeypoints))
    {
      return APEXCV_SUCCESS;
    }
    
    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      ACF_APU_CFG apuConfig = ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1;

      if(threadId)
      {
        apuConfig = ACF_APU_CFG__APU_1_CU_32_63_SMEM_2_3;
      }
      
      status = pProcessCalcRbrief[threadId].ConnectIndirectInput("INPUT", aInSmoothedImg, mpChunkOffsets[threadId]);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      status = pProcessCalcRbrief[threadId].ConnectIO("OUTPUT", mpBinPatternOut[threadId]);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      status = pProcessCalcRbrief[threadId].SelectApuConfiguration(apuConfig, (int32_t)mApexId);

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }

      status = pProcessCalcRbrief[threadId].Start();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    // Wait for the APU processes to finish
    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      status = pProcessCalcRbrief[threadId].Wait();

      if(status != APEXCV_SUCCESS)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    // Pointing to the base of the output container
    uint8_t *pBase = (uint8_t *)aOutDescriptors.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

    // Calculating the necessary offset to jump between thread output clusters
    int descrSizeB    = (int)(mDescrSizeBytes.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int8_t>(0u));
    int nrDescrToCopy = (mNrOfFeatures / mNrOfThreads);
    int clusterSizeB  = (descrSizeB * nrDescrToCopy);

    for(int threadId = 0; threadId < mNrOfThreads; threadId++)
    {
      uint8_t *pDst = (uint8_t *)(pBase + threadId * clusterSizeB);
      uint8_t *pSrc = (uint8_t *)mpBinPatternOut[threadId].getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
      memcpy(pDst, pSrc, clusterSizeB);
    }

    // Everything is fine until here
    return APEXCV_SUCCESS;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  vsdk::SUMat &Orb::GetPatchSize()
  {
    return mPatchSizeInPixels;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  vsdk::SUMat &Orb::GetRadius()
  {
    return mRadiusInPixels;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  vsdk::SUMat &Orb::GetFastOut()
  {
    return mFastCornerList;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  int Orb::GetNrOfDetectedKeypoints()
  {
    return mDetectedKeypoints;
  }

  int Orb::GetNrOfValidKeypoints()
  {
    return mNrOfFeatures;
  }

  bool Orb::DataIsValid()
  {
    return ((mNrOfFeatures != 0) && (mDetectedKeypoints != 0));
  }

  vsdk::SUMat &Orb::GetChunkOffsets()
  {
    if(DataIsValid() == true)
    {
      // Allocating the container
      mAllChunkOffsets = vsdk::SUMat(1, mNrOfFeatures, VSDK_CV_32S);

      // Pointing to the base of the container
      uint8_t *pBaseDst = (uint8_t *)mAllChunkOffsets.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

      for(int threadId = 0; threadId < mNrOfThreads; threadId++)
      {
        // Size of the cluster
        int clusterSizeB = (int)(mpChunkOffsets[threadId].total() * mpChunkOffsets[threadId].elemSize());

        uint8_t *pSrc = (uint8_t *)mpChunkOffsets[threadId].getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
        uint8_t *pDst = (uint8_t *)(pBaseDst + threadId * clusterSizeB);
        memcpy(pDst, pSrc, clusterSizeB);
      }
    }

    return mAllChunkOffsets;
  }

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

  vsdk::SUMat &Orb::GetIcoAngles()
  {
    if(DataIsValid() == true)
    {
      // Allocating the container
      mAllAngles = vsdk::SUMat(1, mNrOfFeatures, VSDK_CV_16U);

      // Pointing to the base of the container
      uint8_t *pBaseDst = (uint8_t *)mAllAngles.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;

      for(int threadId = 0; threadId < mNrOfThreads; threadId++)
      {
        // Size of the cluster
        int clusterSizeB = (int)(mpAngles[threadId].total() * mpAngles[threadId].elemSize());
        uint8_t *pSrc    = (uint8_t *)mpAngles[threadId].getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data;
        OAL_MemoryFlushAndInvalidate((void *)pSrc, clusterSizeB);

        uint8_t *pDst = (uint8_t *)(pBaseDst + threadId * clusterSizeB);
        memcpy(pDst, pSrc, clusterSizeB);
      }
    }

    return mAllAngles;
  }

  std::vector<Orb::Corner> &Orb::GetKeypoints()
  { 
    return mKeypoints;          
  }
} /* namespace apexcv */
