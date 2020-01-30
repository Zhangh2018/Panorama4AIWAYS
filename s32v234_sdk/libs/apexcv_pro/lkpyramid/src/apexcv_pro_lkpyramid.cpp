/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2016-2018 NXP Semiconductor;
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
 * *  @file apexcv_lkpyramid.cpp
 * *  @brief ApexCV for L-K Pyramid Optical Flow 
 * ***********************************************************************************/

#include <oal.h>
#include <mcdma_hal.h>
#include <icp_types.h>
#include <apexcv_pro_lkpyramid.h>
#include <apexcv_pro_util.h>
#include <stdio.h>

// GENERATED ACF PROCESS
#include "LKPYRAMID_FULL_7X7.hpp"
#include <PYRAMID_MULTI_DOWN.hpp>

#define IMAGE_PAD            16
#define LLE_SIZE_1D_MEM_MEM  8

#ifdef __STANDALONE__
const uint32_t cTimeScale = 1;    // In standalone values are already in microseconds
#else
const uint32_t cTimeScale = 1000; // In linux values are in nanoseconds
#endif

namespace apexcv {

/**************
 * Constructor
 *************/
LKPyramidOpticalFlow::LKPyramidOpticalFlow():mInit(false), mPyrLayers(0), mBoxSize(0),
mNumIter(0), mFrmProc(0), mNumFeatures(0), mNumFeatures_ext(0), mProcess(nullptr)
{
   mCoor0Oal    = NULL;
   mCoor1Oal    = NULL;
   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
   mSrc1offsBaseOal = NULL;
   mSrc0AllOal = NULL;
   mSrc1AllOal = NULL;
   mpCoor0     = NULL;
   mpCoor1     = NULL;
   mProcessDownMulti[0]=nullptr;
   mProcessDownMulti[1]=nullptr;
   mProcessDownMulti[2]=nullptr;
   mProcessDownMulti[3]=nullptr;
}

/****************
 * deconstructor
 ***************/
LKPyramidOpticalFlow::~LKPyramidOpticalFlow()
{
   Release();
}

/***************************************************************
 * Release internal allocated buffer and reset member variables
 **************************************************************/
void LKPyramidOpticalFlow::Release()
{
   // Release internal allocated buffers
   if (mCoor0Oal)    OAL_MemoryFree(mCoor0Oal);
   if (mCoor1Oal)    OAL_MemoryFree(mCoor1Oal);
   if (mSrc0offsOal) OAL_MemoryFree(mSrc0offsOal);
   if (mSrc1offsOal) OAL_MemoryFree(mSrc1offsOal);
   if (mSrc1offsBaseOal) OAL_MemoryFree(mSrc1offsBaseOal);
   if (mSrc0AllOal)  OAL_MemoryFree(mSrc0AllOal);
   if (mSrc1AllOal)  OAL_MemoryFree(mSrc1AllOal);

   if (nullptr != mProcess)
   {
      delete (LKPYRAMID_FULL_7X7*) mProcess;
   }
   mProcess = nullptr;
   for (int i=0; i< apexcv::PyramidMultiCreation::PYRAMID_LEVELS; i++)
   {
      if(nullptr != mProcessDownMulti[i])
      {
         delete (PYRAMID_MULTI_DOWN *) mProcessDownMulti[i];
      }
      mProcessDownMulti[i] = nullptr;
   }

   // Reset member variables
   mInit = false;
   mPyrLayers = 0;
   mBoxSize   = 0;
   mNumIter   = 0;
   mCoor0Oal    = NULL;
   mCoor1Oal    = NULL;
   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
   mSrc1offsBaseOal = NULL;
   mSrc0AllOal  = NULL;
   mSrc1AllOal  = NULL;
   mpCoor0      = NULL;
   mpCoor1      = NULL;
}

/*************************
 * Initialization routine
 ************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::Initialize (
         vsdk::SUMat& aSrc0Desc,
         vsdk::SUMat& aSrc1Desc,
         icp::Feature32SDescriptor& aCoor0Desc,
         icp::Feature32SDescriptor& aCoor1Desc,
         icp::Feature32SDescriptor& aCoor1Desc_O,
         int aMaxCorners,
         int aW,
         int aH,
         int aPyrLayers,
         int aBoxSize,
         int aNumIter,
         int aReqPadding
)
{
   int result = APEXCV_SUCCESS;
   if (!mInit) 
   {
      int multi_numFeatures;
      int numFeatures;
      int numFeatures_ext;
      int ncu; // Number of CU
      /* create APU process */
      LKPYRAMID_FULL_7X7* lProcess = new LKPYRAMID_FULL_7X7;
      if (nullptr == lProcess)
      {
         result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         goto EXIT;
      }
      mProcess = lProcess;

      /* Initialize pyramid layer, box size and number of interations */
      if (aSrc0Desc.type() == VSDK_CV_8UC1 &&
         aSrc1Desc.type() == VSDK_CV_8UC1 &&
         (aCoor0Desc.GetDataPtr() != NULL || aCoor0Desc.GetCount() == 0) &&
         (aCoor1Desc.GetDataPtr() != NULL || aCoor1Desc.GetCount() == 0) &&
         (aCoor1Desc_O.GetDataPtr() != NULL || aCoor1Desc_O.GetCount() == 0) &&
         aCoor1Desc_O.GetCount() >= aCoor1Desc.GetCount() &&
         aPyrLayers >= 0 && aPyrLayers <= apexcv::PyramidMultiCreation::PYRAMID_LEVELS &&
         aBoxSize == 7  &&
         aNumIter > 0   &&
         aW > 0 &&
         aH > 0 &&
         aMaxCorners > 0 &&
         (aReqPadding == 0 || aReqPadding == 1)
          )
      {
         mWidth     = aW;
         mHeight    = aH;
         mPyrLayers = aPyrLayers;
         mBoxSize   = aBoxSize;
         mNumIter   = aNumIter;
         mReqPadding = aReqPadding;
      }
      else {
         //*******************************
         // INVALID PARAMETERS, WE RETURN
         //*******************************
         result = APEXCV_ERROR_INVALID_ARGUMENT;
         goto EXIT;
      }
      /* early exit with 0 features to track */
      if (aCoor0Desc.GetCount() == 0 ||
         aCoor1Desc.GetCount() == 0)
      {
         mNumFeatures = 0;
         mNumFeatures_ext = 0;
      }
      mSrc0DescMat = aSrc0Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      mSrc1DescMat = aSrc1Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      mNumFeatures = aCoor0Desc.GetCount();
      mOutCoorBuf = aCoor1Desc_O.GetDataPtr();
      mpCoor0 = (icp::Feature32S*)aCoor0Desc.GetDataPtr();
      mpCoor1 = (icp::Feature32S*)aCoor1Desc.GetDataPtr();
      numFeatures = aMaxCorners;
      numFeatures_ext = numFeatures;
      if (apexcv::RetNumCU(0, ncu))
      {
         //************************************
         // CANNOT GET NUMBER OF CU, WE RETURN
         //************************************
         result = APEXCV_ERROR_INTERNAL_ERROR;
         goto EXIT;
      }
      //**************************************************************************
      // REQUIRED BY DUAL BUFFERING DELAY:
      // MINIMUM NUMBER OF FEATURES TO TRACK CANNOT BE LESS THAN 4 x NUMBER OF CU
      //************************************************************************** 
      if (numFeatures_ext < 4*ncu)
      {
         numFeatures_ext = 4*ncu;
      }
      //**********************************************************
      // Number of features to track across whole pyrmiad levels, 
      // round up to 4 for better DMA
      //**********************************************************
      multi_numFeatures = (numFeatures_ext*(mPyrLayers+1)+3)&(~0x3);

      // Parameters that needed by kernel
      {
         mParamsDesc = vsdk::SUMat(1, 8, VSDK_CV_8SC1);
         if (mParamsDesc.empty())
         {
            //**********************************************
            // INTERNAL MEMORY ALLOCATION FAILED, WE RETURN
            //**********************************************
            result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
            goto EXIT;
         }
         vsdk::SMat lParamMat = mParamsDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
         int8_t *lpData = (int8_t*)lParamMat.data;
         // Number of iterations
         *((int8_t*)lpData + 4 ) = mNumIter;
         // Scharr gradient coef 
         *((int8_t*)lpData + 5 ) = 3;
         *((int8_t*)lpData + 6 ) = 10;
         *((int8_t*)lpData + 7 ) = 3;
      }

      // Allocate multi-pyramids frame buffers for template and patch frames
      mSrc0AllOal = OAL_MemoryAllocFlag((aW + 2 * IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS) * (aH + (aH >> 1) + 4 * IMAGE_PAD), 
                        OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
      mSrc1AllOal = OAL_MemoryAllocFlag((aW + 2 * IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS) * (aH + (aH >> 1) + 4 * IMAGE_PAD), 
                        OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
      if (!(mSrc0AllOal && mSrc1AllOal))
      {
         //**********************************************
         // INTERNAL MEMORY ALLOCATION FAILED, WE RETURN
         //**********************************************
         result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         goto EXIT;
      }
      memset(OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB), 0, 
         (aW + 2 * IMAGE_PAD * apexcv::PyramidMultiCreation::PYRAMID_LEVELS) * (aH + (aH >> 1) + 4 * IMAGE_PAD));
      memset(OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB), 0, 
         (aW + 2 * IMAGE_PAD * apexcv::PyramidMultiCreation::PYRAMID_LEVELS) * (aH + (aH >> 1) + 4 * IMAGE_PAD));

      //**************************************************
      // Setup SUMat data structure for each pyramid level
      //**************************************************
      {
         mSrc0AllDesc = vsdk::CustomPointerSUMat(aH + (aH >> 1) + 4 * IMAGE_PAD, aW + 2 * IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS, VSDK_CV_8UC1, 
            (uint8_t *)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB));
         mSrc1AllDesc = vsdk::CustomPointerSUMat(aH + (aH >> 1) + 4 * IMAGE_PAD, aW + 2 * IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS, VSDK_CV_8UC1, 
            (uint8_t *)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB));

         mSpan = aW + 2 * IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS;
         int c_width = aW; int c_height = aH;
         uint8_t *pSrc0    = (uint8_t*)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB) + mSpan*IMAGE_PAD + IMAGE_PAD;
         uint8_t *pSrc1    = (uint8_t*)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB) + mSpan*IMAGE_PAD + IMAGE_PAD;
         uint8_t *pSrc0Phy = (uint8_t*)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_PHY) + mSpan*IMAGE_PAD + IMAGE_PAD;
         uint8_t *pSrc1Phy = (uint8_t*)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_PHY) + mSpan*IMAGE_PAD + IMAGE_PAD;
         for (int i = 0; i <= apexcv::PyramidMultiCreation::PYRAMID_LEVELS; i++)
         {
            mSrc0PyrDesc[i] = vsdk::CustomPointerSUMat(c_height, c_width, VSDK_CV_8UC1, pSrc0Phy, pSrc0);
            mSrc1PyrDesc[i] = vsdk::CustomPointerSUMat(c_height, c_width, VSDK_CV_8UC1, pSrc1Phy, pSrc1);
            mSrc0PyrDesc[i].step[0] = mSpan;
            mSrc1PyrDesc[i].step[0] = mSpan;
            if (i == 0)
            {
               pSrc0    += ((c_height+2*IMAGE_PAD)*mSpan);
               pSrc1    += ((c_height+2*IMAGE_PAD)*mSpan);
               pSrc0Phy += ((c_height+2*IMAGE_PAD)*mSpan);
               pSrc1Phy += ((c_height+2*IMAGE_PAD)*mSpan);
            }
            else
            {
               pSrc0    += (c_width+2*IMAGE_PAD);
               pSrc1    += (c_width+2*IMAGE_PAD);
               pSrc0Phy += (c_width+2*IMAGE_PAD);
               pSrc1Phy += (c_width+2*IMAGE_PAD);
            }
            c_width >>= 1; c_height >>= 1;
         }
         mParamsDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = mSpan;
      }

      //******************************************************************
      // Initialize template / patch and output features' coordinate SUMat
      //******************************************************************
      //************************************************************************
      // Allocate motion comp DMA offset buffers for template and patch windows
      //************************************************************************
      {
         mCoor0Oal = OAL_MemoryAllocFlag(sizeof(int32_t)*4*(multi_numFeatures+numFeatures_ext),
                        OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mCoor1Oal = OAL_MemoryAllocFlag(sizeof(int32_t)*4*(multi_numFeatures+numFeatures_ext),
                        OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc0offsOal = OAL_MemoryAllocFlag(sizeof(int32_t)*(multi_numFeatures+numFeatures_ext),
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc1offsOal = OAL_MemoryAllocFlag(sizeof(int32_t)*(multi_numFeatures+numFeatures_ext),
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc1offsBaseOal = OAL_MemoryAllocFlag(sizeof(int32_t)*(multi_numFeatures),
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

         if (!(mCoor0Oal && mCoor1Oal && mSrc0offsOal && mSrc1offsOal && mSrc1offsBaseOal))
         {
            //**********************************************
            // INTERNAL MEMORY ALLOCATION FAILED, WE RETURN
            //**********************************************
            result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
            goto EXIT;
         }
         /*Update Internal SUMat*/
         UpdateInternalUMat(multi_numFeatures, numFeatures_ext);
      }
      
      //*****************************************
      // INITIALIZE APEX-CV PyramidMultiCreation
      //*****************************************
      for(int32_t i=0;i<apexcv::PyramidMultiCreation::PYRAMID_LEVELS;i++)
      {
         mSrc0PyrMulDst[i]=(vsdk::SUMat)mSrc0PyrDesc[i+1];
         mSrc1PyrMulDst[i]=(vsdk::SUMat)mSrc1PyrDesc[i+1];
      }
      result = InitPyramidMulti(mSrc0PyrDesc[0], mSrc1PyrDesc[0], mSrc0PyrMulDst, mSrc1PyrMulDst );
      if (result)
      {
         printf("Initialize Pyramid Multi: FAILED.\n");
         goto EXIT;
      }
      // Initialized processes and connect IO 
      {
         if (lProcess->Initialize())
         {
            result = APEXCV_ERROR_ACF_PROC_INITIALIZE;
            goto EXIT;
         }
         if (lProcess->ConnectIO("PARAMS", mParamsDesc)
            || lProcess->ConnectIO("INPUT_COOR_XY0",    mCoor0Desc)
            || lProcess->ConnectIO("INPUT_COOR_XY1",    mCoor1Desc)
            || lProcess->ConnectIO("OUTPUT_COOR_XY1",   mCoor1Desc_O)
            || lProcess->ConnectIO("INPUT_OFFSET_XY1",  mSrc1OffsBaseDesc)
            || lProcess->ConnectIO("OUTPUT_OFFSET_XY1", mSrc1OffsDesc_O)
            || lProcess->ConnectIO("TEMPL_COOR_NEXT", mCoor0Desc_Next)
            || lProcess->ConnectIO("OUTPUT_TEMPL_OFFSET_NEXT", mSrc0OffsDesc_Next)
            || lProcess->ConnectIndirectInput("INPUT_TEMPLATE", mSrc0AllDesc, mSrc0OffsDesc)
            || lProcess->ConnectIndirectInput("INPUT_PATCH",    mSrc1AllDesc, mSrc1OffsDesc))
         {
            result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            goto EXIT;
         }
         {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("LKPYRAMID_FULL_7X7", lProcess);
            lInfo.push_PortName("INPUT_COOR_XY0");
            mvInfoClusters.push_back(lInfo);
         }
      }
      if (!result)
      {
         mInit=true;
      }
      else
      {
         result = APEXCV_ERROR_INVALID_ARGUMENT;
         goto EXIT;
      }
   }
   else
   {
      //*******************************************
      // HAVE ALREADY BEEN INITIALIZED, WE RETURN
      //*******************************************
      result = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;//TODO: add standardized apex cv error codes
      goto EXIT;
   }
   EXIT:
   if((result != APEXCV_SUCCESS) && (result != APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED))
   {
      Release();
   }
   return result;
}
/***************************************
 * Set pyramid levels
 **************************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::SetPyrLayers(int aPyrLayers) 
{
   if (aPyrLayers >=0 && aPyrLayers <= apexcv::PyramidMultiCreation::PYRAMID_LEVELS) 
   {
      mPyrLayers = aPyrLayers;
      return APEXCV_SUCCESS;
   }
   return APEXCV_ERROR_INVALID_ARGUMENT;
}

/***************************************
 * Set box size, only support 7 for now
 **************************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::SetBoxSize(int aBoxSize) 
{
   if (7 == aBoxSize )
   {
      mBoxSize = aBoxSize;
      return APEXCV_SUCCESS;
   }
   // invalid parameter
   return APEXCV_ERROR_INVALID_ARGUMENT;
}

/***************************
 * Set number of iterations
 **************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::SetNumIter(int aNumIter) 
{
   if (aNumIter > 0)
   {
      mNumIter = aNumIter;
      return APEXCV_SUCCESS;
   }
   // invalid parameter
   return APEXCV_ERROR_INVALID_ARGUMENT;
}

APEXCV_LIB_RESULT LKPyramidOpticalFlow::InitPyramidMulti (
            vsdk::SUMat&  aSrc0   /*!< Source 0 memory buffer. */,
            vsdk::SUMat&  aSrc1   /*!< Source 1 memory buffer. */,
            vsdk::SUMat (&aDst0Array)[apexcv::PyramidMultiCreation::PYRAMID_LEVELS]   /*!< Destination 0 memory buffer. */,
            vsdk::SUMat (&aDst1Array)[apexcv::PyramidMultiCreation::PYRAMID_LEVELS]   /*!< Destination 1 memory buffer. */
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;
   PYRAMID_MULTI_DOWN *pDownProcess[apexcv::PyramidMultiCreation::PYRAMID_LEVELS];
   for( int i=0; i<apexcv::PyramidMultiCreation::PYRAMID_LEVELS; i++)
   {
      pDownProcess[i] = new PYRAMID_MULTI_DOWN;
      if( nullptr == pDownProcess[i])
      {
         result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         goto EXIT;
      }
      mProcessDownMulti[i] = pDownProcess[i];
   }

   if ((aSrc0.type() == VSDK_CV_8UC1) && 
      (aDst0Array[0].type() == VSDK_CV_8UC1) && 
      (aDst0Array[1].type() == VSDK_CV_8UC1) && 
      (aDst0Array[2].type() == VSDK_CV_8UC1) && 
      (aDst0Array[3].type() == VSDK_CV_8UC1) && 
      (aSrc1.type() == VSDK_CV_8UC1) && 
      (aDst1Array[0].type() == VSDK_CV_8UC1) && 
      (aDst1Array[1].type() == VSDK_CV_8UC1) && 
      (aDst1Array[2].type() == VSDK_CV_8UC1) && 
      (aDst1Array[3].type() == VSDK_CV_8UC1))
   {
      if ( pDownProcess[0]->Initialize() || pDownProcess[1]->Initialize() || pDownProcess[2]->Initialize() || pDownProcess[3]->Initialize())
      {
         result = APEXCV_ERROR_ACF_PROC_INITIALIZE;
         goto EXIT;
      }
      if( pDownProcess[0]->ConnectIO("INPUT",    aSrc0) ||
          pDownProcess[0]->ConnectIO("OUTPUT_0", aDst0Array[0]) ||
          pDownProcess[0]->ConnectIO("OUTPUT_1", aDst0Array[1]) ||
          pDownProcess[1]->ConnectIO("INPUT",    aDst0Array[1]) ||
          pDownProcess[1]->ConnectIO("OUTPUT_0", aDst0Array[2]) ||
          pDownProcess[1]->ConnectIO("OUTPUT_1", aDst0Array[3]) ||

          pDownProcess[2]->ConnectIO("INPUT",    aSrc1) ||
          pDownProcess[2]->ConnectIO("OUTPUT_0", aDst1Array[0]) ||
          pDownProcess[2]->ConnectIO("OUTPUT_1", aDst1Array[1]) ||
          pDownProcess[3]->ConnectIO("INPUT",    aDst1Array[1]) ||
          pDownProcess[3]->ConnectIO("OUTPUT_0", aDst1Array[2]) ||
          pDownProcess[3]->ConnectIO("OUTPUT_1", aDst1Array[3]) )
      {
         result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
         goto EXIT;
      }
   }
   else
   {
      result = APEXCV_ERROR_INVALID_ARGUMENT;//TODO: add standardized apex cv error codes
      goto EXIT;
   }
   EXIT:
   if(result)
   {
      for( int i=0; i<apexcv::PyramidMultiCreation::PYRAMID_LEVELS; i++)
      {
         if(nullptr != mProcessDownMulti[i])
         {
            delete (PYRAMID_MULTI_DOWN *) mProcessDownMulti[i];
         }
         mProcessDownMulti[i] = nullptr;
      }
   }
   return result;
}

//**********************************************************
// Update internal SUMat
//**********************************************************
void LKPyramidOpticalFlow::UpdateInternalUMat(int aMultiNumFeature, int aNumFeaturesExt)
{
   mCoor0Desc = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC4, 
      (uint8_t *)OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_CH_WB));
   mCoor0Desc_Next = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC4, 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_PHY)+4*aNumFeaturesExt), 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_CH_WB)+4*aNumFeaturesExt));
   mCoor1Desc = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC4, 
      (uint8_t *)OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_CH_WB));
   mCoor1Desc_O = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC4, 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_PHY)+4*aNumFeaturesExt), 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_CH_WB)+4*aNumFeaturesExt));
   mSrc0OffsDesc = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC1, 
      (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB));
   mSrc0OffsDesc_Next = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC1, 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY)+aNumFeaturesExt), 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB)+aNumFeaturesExt));
   mSrc1OffsDesc = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC1, 
      (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_CH_WB));
   mSrc1OffsBaseDesc = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC1, 
      (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsBaseOal, ACCESS_PHY), 
      (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsBaseOal, ACCESS_CH_WB));
   mSrc1OffsDesc_O = vsdk::CustomPointerSUMat(1, aMultiNumFeature, VSDK_CV_32SC1, 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY)+aNumFeaturesExt), 
      (uint8_t *)((int32_t*)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_CH_WB)+aNumFeaturesExt));
}

/***************************************
 * Fill in level=0 image
 **************************************/
void LKPyramidOpticalFlow::FillImageZeroLevel()
{
   uint8_t *s = (uint8_t*)mSrc0DescMat.data;
   uint8_t *p = (uint8_t*)mSrc0PyrDesc[0].u->data;
   int src_span   = mSrc0DescMat.step[0];
   for (int y = 0; y < (int)mHeight; y++)
   {
      memcpy(p, s, mWidth);
      p += mSpan;
      s += src_span;
   }
   s = (uint8_t*)mSrc1DescMat.data;
   p = (uint8_t*)mSrc1PyrDesc[0].u->data;
   src_span   = mSrc1DescMat.step[0];
   for (int y = 0; y < (int)mHeight; y++)
   {
      memcpy(p, s, mWidth);
      p += mSpan;
      s += src_span;
   }
}

/***************************************
 * Fill in pyramid levels coordinates and offsets
 **************************************/
void LKPyramidOpticalFlow::FillPyramidLevel(int aMultiNumFeature)
{
   icp::Feature32S *pmCoor0 = (icp::Feature32S*)mCoor0Desc.u->data;
   icp::Feature32S *pmCoor1 = (icp::Feature32S*)mCoor1Desc.u->data;
   memset((void*)pmCoor0, 0, sizeof(icp::Feature32S)*aMultiNumFeature);
   memset((void*)pmCoor1, 0, sizeof(icp::Feature32S)*aMultiNumFeature);
   pmCoor0 += mPyrLayers * mNumFeatures_ext;
   pmCoor1 += mPyrLayers * mNumFeatures_ext;
   memcpy((void*)pmCoor0, mpCoor0, sizeof(icp::Feature32S)*mNumFeatures);
   if (mPyrLayers == 0)
   {
      memcpy((void*)pmCoor1, mpCoor1, sizeof(icp::Feature32S)*mNumFeatures);
   }
   else
   {
      for (int i = 1; i <= mPyrLayers; i++)
      {
         pmCoor0 -= mNumFeatures_ext;
         pmCoor1 -= mNumFeatures_ext;
         for (int j = mNumFeatures-1; j >= 0; j--)
         {
            pmCoor0[j].position.x = (mpCoor0[j].position.x >> i);
            pmCoor0[j].position.y = (mpCoor0[j].position.y >> i);
            if (i == mPyrLayers)
            {
               pmCoor1[j].position.x = (mpCoor1[j].position.x >> i);
               pmCoor1[j].position.y = (mpCoor1[j].position.y >> i);
            }
         }
      }
   }
}

/***************************************
 * Fill in offset
 **************************************/
void LKPyramidOpticalFlow::FillOffset(int aMultiNumFeature)
{
   int32_t *pOffs0     = (int32_t*)mSrc0OffsDesc.u->data;
   int32_t *pOffs1     = (int32_t*)mSrc1OffsDesc.u->data;
   int32_t *pOffs1Base = (int32_t*)mSrc1OffsBaseDesc.u->data - mNumFeatures_ext;
   icp::Feature32S *pmCoor0 = (icp::Feature32S*)mCoor0Desc.u->data;
   icp::Feature32S *pmCoor1 = (icp::Feature32S*)mCoor1Desc.u->data;
   memset((void*)pOffs0, 0, sizeof(int32_t)*aMultiNumFeature);
   memset((void*)pOffs1, 0, sizeof(int32_t)*aMultiNumFeature);
   memset((void*)mSrc1OffsBaseDesc.u->data, 0, sizeof(int32_t)*aMultiNumFeature);
   for (int i = 0; i <= mPyrLayers; i++)
   {
      // Base offset of each pyramid level's frame within mega buffer
      int32_t top_left_off_0 = mSpan * ((mBoxSize>>1) + 1) + ((mBoxSize>>1) + 1);
      int32_t top_left_off_1 = mSpan * ((mBoxSize>>1) + LKPYRAMID_PATCH_BORDER) + ((mBoxSize>>1) + LKPYRAMID_PATCH_BORDER);
      int32_t Src0BaseOff = (int32_t)((int64_t)mSrc0PyrDesc[mPyrLayers-i].u->data - (int64_t)mSrc0AllDesc.u->data) - top_left_off_0;
      int32_t Src1BaseOff = (int32_t)((int64_t)mSrc1PyrDesc[mPyrLayers-i].u->data - (int64_t)mSrc1AllDesc.u->data) - top_left_off_1;
      for (int j = 0; j < mNumFeatures; j++)
      {
         if (i == 0)
         {
            // Fill in first pyramid level features offset buffer for motoin comp DMA
            // Other levels will be filled in on-the-fly
            int32_t i_x = pmCoor0[j].position.x >> LKPYRAMID_FLOATING_COOR_Q;
            int32_t i_y = pmCoor0[j].position.y >> LKPYRAMID_FLOATING_COOR_Q;
            pOffs0[j] = Src0BaseOff + mSpan * i_y + i_x;
            i_x = pmCoor1[j].position.x >> LKPYRAMID_FLOATING_COOR_Q;
            i_y = pmCoor1[j].position.y >> LKPYRAMID_FLOATING_COOR_Q;
            pOffs1[j] = Src1BaseOff + mSpan * i_y + i_x;
         }
         else
         {
            pOffs1Base[j] = Src1BaseOff;
         }
      }
      pOffs0     += mNumFeatures_ext;
      pOffs1     += mNumFeatures_ext;
      pOffs1Base += mNumFeatures_ext;
      pmCoor0 += mNumFeatures_ext;
      pmCoor1 += mNumFeatures_ext;
   }
}

APEXCV_LIB_RESULT LKPyramidOpticalFlow::ProcessPyramidMulti()
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;
   PYRAMID_MULTI_DOWN *pDownProcess0 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[0];
   PYRAMID_MULTI_DOWN *pDownProcess1 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[1];
   PYRAMID_MULTI_DOWN *pDownProcess2 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[2];
   PYRAMID_MULTI_DOWN *pDownProcess3 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[3];

   result |= pDownProcess0->Start();
   result |= pDownProcess0->Wait();
   if(result)
   {
      result = APEXCV_ERROR_ACF_PROC_EXEC;
      goto EXIT;
   }
   result |= pDownProcess1->Start();
   result |= pDownProcess1->Wait();
   if(result)
   {
      result = APEXCV_ERROR_ACF_PROC_EXEC;
      goto EXIT;
   }
   result |= pDownProcess2->Start();
   result |= pDownProcess2->Wait();
   if(result)
   {
      result = APEXCV_ERROR_ACF_PROC_EXEC;
      goto EXIT;
   }
   result |= pDownProcess3->Start();
   result |= pDownProcess3->Wait();
   if(result)
   {
      result = APEXCV_ERROR_ACF_PROC_EXEC;
      goto EXIT;
   }
   EXIT:
   return result;
}

/*******************************
 * ReconnectIO to feed in input
 ******************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::ReconnectIO(
            vsdk::SUMat& aSrc0Desc,
            vsdk::SUMat& aSrc1Desc,
            icp::Feature32SDescriptor& aCoor0Desc,
            icp::Feature32SDescriptor& aCoor1Desc,
            icp::Feature32SDescriptor& aCoor1Desc_O
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;
   if (aSrc0Desc.type() != VSDK_CV_8UC1 ||
      aSrc1Desc.type() != VSDK_CV_8UC1 ||
      (aCoor0Desc.GetDataPtr() == NULL   && aCoor0Desc.GetCount() != 0)   ||
      (aCoor1Desc.GetDataPtr() == NULL   && aCoor1Desc.GetCount() != 0)   ||
      (aCoor1Desc_O.GetDataPtr() == NULL && aCoor1Desc_O.GetCount() != 0) ||
      aCoor1Desc_O.GetCount() < aCoor1Desc.GetCount())
   {
      //*******************************
      // INVALID PARAMETERS, WE RETURN
      //*******************************
      result = APEXCV_ERROR_INVALID_ARGUMENT;
      goto EXIT;
   }

   /* early exit with 0 features to track */
   if (0 == aCoor0Desc.GetCount() ||
      0 == aCoor1Desc.GetCount() )
   {
      mNumFeatures = 0;
      mNumFeatures_ext = 0;
      return APEXCV_SUCCESS;
   }

   if (mInit)
   {
      LKPYRAMID_FULL_7X7* lProcess = (LKPYRAMID_FULL_7X7*) mProcess;
      mSrc0DescMat = aSrc0Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      mSrc1DescMat = aSrc1Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      mpCoor0 = (icp::Feature32S*)aCoor0Desc.GetDataPtr();
      mpCoor1 = (icp::Feature32S*)aCoor1Desc.GetDataPtr();
      mNumFeatures     = aCoor0Desc.GetCount();
      //**********************************
      // create maximum level of pyramids 
      //**********************************
      {
         PYRAMID_MULTI_DOWN *pDownProcess0 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[0];
         PYRAMID_MULTI_DOWN *pDownProcess1 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[1];
         PYRAMID_MULTI_DOWN *pDownProcess2 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[2];
         PYRAMID_MULTI_DOWN *pDownProcess3 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti[3];

         if ((mSrc0PyrDesc[0].type() == VSDK_CV_8UC1) && (mSrc0PyrMulDst[0].type() == VSDK_CV_8UC1)
         && (mSrc0PyrMulDst[1].type() == VSDK_CV_8UC1) && (mSrc0PyrMulDst[2].type() == VSDK_CV_8UC1)
         && (mSrc0PyrMulDst[3].type() == VSDK_CV_8UC1)&&(mSrc1PyrDesc[0].type() == VSDK_CV_8UC1)
         && (mSrc1PyrMulDst[0].type() == VSDK_CV_8UC1)&& (mSrc1PyrMulDst[1].type() == VSDK_CV_8UC1)
         && (mSrc1PyrMulDst[2].type() == VSDK_CV_8UC1)&& (mSrc1PyrMulDst[3].type() == VSDK_CV_8UC1))
         {
            if( pDownProcess0->ConnectIO("INPUT",    mSrc0PyrDesc[0]) ||
               pDownProcess0->ConnectIO("OUTPUT_0", mSrc0PyrMulDst[0])||
               pDownProcess0->ConnectIO("OUTPUT_1", mSrc0PyrMulDst[1])||
               pDownProcess1->ConnectIO("INPUT",    mSrc0PyrMulDst[1])||
               pDownProcess1->ConnectIO("OUTPUT_0", mSrc0PyrMulDst[2])||
               pDownProcess1->ConnectIO("OUTPUT_1", mSrc0PyrMulDst[3])||
               pDownProcess2->ConnectIO("INPUT",    mSrc1PyrDesc[0]) ||
               pDownProcess2->ConnectIO("OUTPUT_0", mSrc1PyrMulDst[0])||
               pDownProcess2->ConnectIO("OUTPUT_1", mSrc1PyrMulDst[1])||
               pDownProcess3->ConnectIO("INPUT",    mSrc1PyrMulDst[1])||
               pDownProcess3->ConnectIO("OUTPUT_0", mSrc1PyrMulDst[2])||
               pDownProcess3->ConnectIO("OUTPUT_1", mSrc1PyrMulDst[3]) )
            {
               result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }
         }
         else
         {
            result = APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }

      /* reconnect IO */
      {
         if (lProcess->ConnectIO("INPUT_COOR_XY0",    mCoor0Desc)
            || lProcess->ConnectIO("INPUT_COOR_XY1",    mCoor1Desc)
            || lProcess->ConnectIO("OUTPUT_COOR_XY1",   mCoor1Desc_O)
            || lProcess->ConnectIO("INPUT_OFFSET_XY1",  mSrc1OffsBaseDesc)
            || lProcess->ConnectIO("OUTPUT_OFFSET_XY1", mSrc1OffsDesc_O)
            || lProcess->ConnectIO("TEMPL_COOR_NEXT", mCoor0Desc_Next)
            || lProcess->ConnectIO("OUTPUT_TEMPL_OFFSET_NEXT", mSrc0OffsDesc_Next)
            || lProcess->ConnectIndirectInput("INPUT_TEMPLATE", mSrc0AllDesc, mSrc0OffsDesc)
            || lProcess->ConnectIndirectInput("INPUT_PATCH",    mSrc1AllDesc, mSrc1OffsDesc))
         {
            result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            goto EXIT;
         }
      }
      mOutCoorBuf = aCoor1Desc_O.GetDataPtr();
   }
   else
   {
      result = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      goto EXIT;
   }
   EXIT:
   return result;
}

/**************
 * APU process
 *************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::Process()
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;
   if (mInit)
   {
      mNumFeatures_ext = mNumFeatures;
      int ncu;
      if (apexcv::RetNumCU(0, ncu))
      {
         result = APEXCV_ERROR_INTERNAL_ERROR;
         goto EXIT;
      }
      //**************************************************************************
      // REQUIRED BY DUAL BUFFERING DELAY:
      // MINIMUM NUMBER OF FEATURES TO TRACK CANNOT BE LESS THAN 4 x NUMBER OF CU
      //************************************************************************** 
      if (mNumFeatures_ext < 4*ncu)
      {
         mNumFeatures_ext = 4*ncu;
      }
      //**********************************************************
      // Number of features to track across whole pyrmiad levels, 
      // round up to 4 for better DMA
      //**********************************************************
      int multi_numFeatures = (mNumFeatures_ext*(mPyrLayers+1)+3)&(~0x3);

      {
         if (multi_numFeatures != mSrc0OffsDesc.cols)
         {
            //**********************************************************
            // Number of features changed, need to update internal SUMat
            //**********************************************************
            UpdateInternalUMat(multi_numFeatures, mNumFeatures_ext);
         }
      }
      //***********************
      // fill in level=0 image
      //***********************
      FillImageZeroLevel();

      //************************************************
      // fill in pyramid levels coordinates and offsets
      //************************************************
      FillPyramidLevel(multi_numFeatures);

      //****************
      // fill in offset
      //****************
      FillOffset(multi_numFeatures);

      OAL_MemoryFlushAndInvalidate(mSrc0AllDesc.u->data, mSrc0AllDesc.rows * mSrc0AllDesc.step[0]);
      OAL_MemoryFlushAndInvalidate(mSrc1AllDesc.u->data, mSrc1AllDesc.rows * mSrc1AllDesc.step[0]);

      result = ProcessPyramidMulti();
      if (result)
      {
         printf("Process Pyramid Multi: FAILED.\n");
         goto EXIT;
      }

      // perform image border padding: pixel repllication
      if (mReqPadding)
      {
        for (int i = 0; i <= mPyrLayers; i++)
        {
            icp::DataDescriptor first, second;
            first.InitManual(mSrc0PyrDesc[i].cols, mSrc0PyrDesc[i].rows, mSrc0PyrDesc[i].u->data, 
               (uint8_t*) (mSrc0PyrDesc[i].u->handle) + mSrc0PyrDesc[i].offset, icp::DATATYPE_08U);
            second.InitManual(mSrc1PyrDesc[i].cols, mSrc1PyrDesc[i].rows, mSrc1PyrDesc[i].u->data, 
               (uint8_t*) (mSrc1PyrDesc[i].u->handle) + mSrc1PyrDesc[i].offset, icp::DATATYPE_08U);
            first.SetSpan(mSrc0PyrDesc[i].step[0]);
            second.SetSpan(mSrc1PyrDesc[i].step[0]);
            first.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
            second.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
        }
      }
      {
         LKPYRAMID_FULL_7X7* lProcess = (LKPYRAMID_FULL_7X7*) mProcess;
         /* nothing needs to be processed */
         if (0 == mNumFeatures)
         {
            result = APEXCV_SUCCESS;
            goto EXIT;
         }
   
         if (3 == mBoxSize)
         {
            // TODO: unsupported box window size yet
         }
         else if (5 == mBoxSize)
         {
          // TODO: unsupported box window size yet
         }
         else if (7 == mBoxSize)
         {
            OAL_MemoryFlushAndInvalidate(OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB), 
               (mWidth+2*IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS)*(mHeight+(mHeight>>1)+4*IMAGE_PAD));
            OAL_MemoryFlushAndInvalidate(OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB), 
               (mWidth+2*IMAGE_PAD*apexcv::PyramidMultiCreation::PYRAMID_LEVELS)*(mHeight+(mHeight>>1)+4*IMAGE_PAD));
            OAL_MemoryFlushAndInvalidate(mCoor0Desc.u->data, mCoor0Desc.rows * mCoor0Desc.step[0]);
            OAL_MemoryFlushAndInvalidate(mCoor1Desc.u->data, mCoor1Desc.rows * mCoor1Desc.step[0]);
            OAL_MemoryFlushAndInvalidate(mCoor1Desc_O.u->data, mCoor1Desc_O.rows * mCoor1Desc_O.step[0]);
            OAL_MemoryFlushAndInvalidate(mSrc1OffsBaseDesc.u->data, 
               mSrc1OffsBaseDesc.rows * mSrc1OffsBaseDesc.step[0]);
            OAL_MemoryFlushAndInvalidate(mSrc1OffsDesc_O.u->data, mSrc1OffsDesc_O.rows * mSrc1OffsDesc_O.step[0]);
            OAL_MemoryFlushAndInvalidate(mCoor0Desc_Next.u->data, mCoor0Desc_Next.rows * mCoor0Desc_Next.step[0]);
            OAL_MemoryFlushAndInvalidate(mSrc0OffsDesc_Next.u->data, 
               mSrc0OffsDesc_Next.rows * mSrc0OffsDesc_Next.step[0]);
            OAL_MemoryFlushAndInvalidate(mSrc0OffsDesc.u->data, mSrc0OffsDesc.rows * mSrc0OffsDesc.step[0]);
            OAL_MemoryFlushAndInvalidate(mSrc1OffsDesc.u->data, mSrc1OffsDesc.rows * mSrc1OffsDesc.step[0]);

            result |= lProcess->Start();
            result |= lProcess->Wait();
            if (result)
            {
               result = APEXCV_ERROR_ACF_PROC_EXEC;
               goto EXIT;
            }
         }
         else  /* == 9 */
         {
         // TODO: unsupported box window size yet
         }
      }
      /* output coordinate */
         {
            icp::Feature32S *psCoor1 = (icp::Feature32S*)mCoor1Desc_O.u->data;
            icp::Feature32S *pdCoor1 = (icp::Feature32S*)mOutCoorBuf;
            psCoor1 += mPyrLayers * mNumFeatures_ext;
            for (int j = 0; j < mNumFeatures; j++)
            {
               pdCoor1[j].position.x = psCoor1[j].position.x >> 1;
               pdCoor1[j].position.y = psCoor1[j].position.y >> 1;
               pdCoor1[j].strength    = psCoor1[j].strength;
               pdCoor1[j].reserve     = psCoor1[j].reserve;
            }
         }
      }
      else
      {
         // class hasn't been initialized yet, return error
         result = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
         goto EXIT;
      }
   EXIT:
   return result;
}

/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT LKPyramidOpticalFlow::SelectApexCore(int32_t aApexId)
{
   if (mInit)
   {
      LKPYRAMID_FULL_7X7* lProcess = (LKPYRAMID_FULL_7X7*) mProcess;

      if (lProcess->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
      mApexId = aApexId;
   }
   else
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   return APEXCV_SUCCESS;
}

}  /* namespace apex */
