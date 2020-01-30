/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2016-2017 NXP Semiconductor;
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
* @file apexcv_lktracker.cpp
* @brief ApexCV for L-K tracker Optical Flow
* ***********************************************************************************/

//***********************************************
// TOGGLE THIS MACRO TO ENABLE/DISABLE PROFILING
//***********************************************

#include <oal.h>
#include <icp_types.h>
#include <apexcv_pro_lktracker.h>
#include <apexcv_pro_util.h>
#include <stdio.h>
#include <apexcv_error_codes.hpp>

// GENERATED ACF PROCESS
#include <LKTRACKER_FULL_7X7.hpp>

#define IMAGE_PAD 16

#ifdef __STANDALONE__
const uint32_t cTimeScale = 1;    // In standalone values are already in microseconds
#else
const uint32_t cTimeScale = 1000; // In linux values are in nanoseconds
#endif


/************************************************************************************************************************************* 
 * helper function to calculate motion comp DMA buffer address offset based on features' X/Y coordinate and image's width/height/span
 *************************************************************************************************************************************/
static void LKTrackerCalcOffset(const vsdk::SUMat& apSrcDesc, icp::Feature32SDescriptor& aCoorDesc, vsdk::CustomPointerSUMat& aOffsDesc, int aHalfWin, int aPad, bool aImgReplicated);

namespace apexcv {

/**************
 * Constructor
 *************/
LKTrackerOpticalFlow::LKTrackerOpticalFlow():mInit(false), mBoxSize(0), mNumIter(0), mSrcPadded(0), mNumFeatures(0),
mApuConfig(ACF_APU_CFG__DEFAULT),
mApexId(0),
mProcess(nullptr),
mWeSelectedApuConfiguration(false)
{
   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
   mSrc0PadOal  = NULL;
   mSrc1PadOal  = NULL;
}

/****************
 * deconstructor
 ***************/
LKTrackerOpticalFlow::~LKTrackerOpticalFlow()
{
   Release();
}

/***************************************************************
 * Release internal allocated buffer and reset member variables 
 **************************************************************/
void LKTrackerOpticalFlow::Release()
{
   // Release internal allocated buffers
   if (mSrc0offsOal) OAL_MemoryFree(mSrc0offsOal);
   if (mSrc1offsOal) OAL_MemoryFree(mSrc1offsOal);
   if (mSrc0PadOal)  OAL_MemoryFree(mSrc0PadOal);
   if (mSrc1PadOal)  OAL_MemoryFree(mSrc1PadOal);

   if (nullptr != mProcess)
   {
      delete (LKTRACKER_FULL_7X7*) mProcess;
   }
   mProcess = nullptr;
   
   // Reset member variables
   mInit = false;
   mBoxSize   = 0;
   mNumIter   = 0;
   mSrcPadded = 0;

   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
   mSrc0PadOal  = NULL;
   mSrc1PadOal  = NULL;
}

/************************* 
 * Initialization routine
 ************************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::Initialize (
               vsdk::SUMat& aSrc0Desc,
               vsdk::SUMat& aSrc1Desc,
               icp::Feature32SDescriptor& aCoor0Desc,
               icp::Feature32SDescriptor& aCoor1Desc,
               icp::Feature32SDescriptor& aCoor1Desc_O,
               int aMaxCorners,
               int aW, 
               int aH,
               int aBoxSize,
               int aNumIter,
               int aSrcPadded,
               int aPadStartWidth
)
{
   int numFeatures = (aMaxCorners+3)&(~0x3);


   if (mInit)
   {
      //*******************************************
      // HAVE ALREADY B EEN INITIALIZED, WE RETURN
      //*******************************************
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }
   mInit = true;

   /* create APU process */
   LKTRACKER_FULL_7X7* lProcess = new LKTRACKER_FULL_7X7;
   if (nullptr == lProcess)
   {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }
   mProcess = lProcess;

   /* Setup box size and nubmer of interations */
    if (aBoxSize == 7 &&
       aNumIter > 0 &&
       aW > 0 &&
       aH > 0 &&
       aSrcPadded >= 0)
   {
      mBoxSize   = aBoxSize;
      mNumIter   = aNumIter;
      mSrcPadded = aSrcPadded;
      mPadStartWidth = aPadStartWidth;
   }
   else {
      //*******************************
      // INVALID PARAMETERS, WE RETURN
      //*******************************
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   /* Allocate internal buffers */
   {
      // Parameters that needed by kernel
      {
         mParamsDesc = vsdk::SUMat(1, 12, VSDK_CV_8SC1);
         if (mParamsDesc.empty())
         {
            //**********************************************
            // INTERNAL MEMORY ALLOCATION FAILED, WE RETURN
            //**********************************************
            Release();
            return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }
 
         vsdk::SMat lParamMat = mParamsDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
         int8_t *lpData = (int8_t*)lParamMat.data;
         // Maximum X and Y cooridinates in Q23.8 format
         *((int32_t*)lpData    ) = (aSrc1Desc.cols << LKTRACKER_FLOATING_COOR_Q);
         *((int32_t*)lpData + 1) = (aSrc1Desc.rows << LKTRACKER_FLOATING_COOR_Q);
         // Number of iterations
         *((int8_t*)lpData + 8 ) = mNumIter;
         // Scharr gradient coefficients 
         *((int8_t*)lpData + 9 ) = 3;
         *((int8_t*)lpData + 10) = 10;
         *((int8_t*)lpData + 11) = 3;
      }

      // Initialize template / patch and output features' coordinate SUMat
      {
        mCoor0Desc = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor0Desc.GetDataPtrPhys(), (uint8_t *)aCoor0Desc.GetDataPtr());
        mCoor1Desc = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor1Desc.GetDataPtrPhys(), (uint8_t *)aCoor1Desc.GetDataPtr());
        mCoor1Desc_O = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor1Desc_O.GetDataPtrPhys(), (uint8_t *)aCoor1Desc_O.GetDataPtr());
      }

      // Allocate motion comp DMA offset buffers for template and patch windows
      {
         mSrc0offsOal = OAL_MemoryAllocFlag(sizeof(int32_t)*numFeatures,
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc1offsOal = OAL_MemoryAllocFlag(sizeof(int32_t)*numFeatures,
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

         if (!(mSrc0offsOal && mSrc1offsOal))
         {
            //**********************************************
            // INTERNAL MEMORY ALLOCATION FAILED, WE RETURN
            //**********************************************
            Release();
            return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }

         mSrc0Offs = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC1, (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB));
         mSrc1Offs = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC1, (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_CH_WB));
      }

      // Allocate image border padded buffers for template and patch frames 
      {
         mSrc0PadOal = OAL_MemoryAllocFlag((aW + IMAGE_PAD * 2) * (aH + IMAGE_PAD * 2),
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc1PadOal = OAL_MemoryAllocFlag((aW + IMAGE_PAD * 2) * (aH + IMAGE_PAD * 2),
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

         if (!(mSrc0PadOal && mSrc1PadOal))
         {
            // internal memory allocation failed.
            Release();
            return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }

         mSrc0PadDesc = vsdk::CustomPointerSUMat(aH, aW, VSDK_CV_8UC1, 
                                                (uint8_t*)OAL_MemoryReturnAddress(mSrc0PadOal, ACCESS_PHY)   + IMAGE_PAD + IMAGE_PAD * (aW + 2 * IMAGE_PAD), 
                                                (uint8_t*)OAL_MemoryReturnAddress(mSrc0PadOal, ACCESS_CH_WB) + IMAGE_PAD + IMAGE_PAD * (aW + 2 * IMAGE_PAD));
         mSrc0PadDesc.step[0] = aW + 2 * IMAGE_PAD;
         mSrc1PadDesc = vsdk::CustomPointerSUMat(aH, aW, VSDK_CV_8UC1, 
                                                (uint8_t*)OAL_MemoryReturnAddress(mSrc1PadOal, ACCESS_PHY)   + IMAGE_PAD + IMAGE_PAD * (aW + 2 * IMAGE_PAD), 
                                                (uint8_t*)OAL_MemoryReturnAddress(mSrc1PadOal, ACCESS_CH_WB) + IMAGE_PAD + IMAGE_PAD * (aW + 2 * IMAGE_PAD));
         mSrc1PadDesc.step[0] = aW + 2 * IMAGE_PAD;
      }
   }

   /* Initialized processes and connect IO */
   {
      if (lProcess->Initialize())
      {
         Release();
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      if(mWeSelectedApuConfiguration == true)
      {
         if (lProcess->SelectApuConfiguration(mApuConfig, mApexId))
         {
            Release();
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         }
      }
      if (lProcess->ConnectIO("PARAMS", mParamsDesc)
          || lProcess->ConnectIO("INPUT_COOR_XY0",  mCoor0Desc)
          || lProcess->ConnectIO("INPUT_COOR_XY1",  mCoor1Desc)
          || lProcess->ConnectIO("OUTPUT_COOR_XY1", mCoor1Desc_O))
      {
        Release();
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("LKTRACKER_FULL_7X7", lProcess);
         lInfo.push_PortName("INPUT_COOR_XY0");
         mvInfoClusters.push_back(lInfo);
      }
   }

   // call ReconnectIO to feed in input
   if (ReconnectIO(aSrc0Desc, aSrc1Desc, aCoor0Desc, aCoor1Desc, aCoor1Desc_O))
   { 
      Release();
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
   }

   return APEXCV_SUCCESS;
}

/*************************************** 
 * Set box size, only support 7 for now 
 **************************************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::SetBoxSize(int aBoxSize)
{
   if (aBoxSize == 7)
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
APEXCV_LIB_RESULT LKTrackerOpticalFlow::SetNumIter(int aNumIter)
{
   if (aNumIter > 0)
   {
      mNumIter = aNumIter;
      return APEXCV_SUCCESS;
   }
   // invalid parameter
   return APEXCV_ERROR_INVALID_ARGUMENT;
}


/*******************************
 * ReconnectIO to feed in input
 ******************************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::ReconnectIO(
            vsdk::SUMat& aSrc0Desc,
            vsdk::SUMat& aSrc1Desc,
            icp::Feature32SDescriptor& aCoor0Desc,
            icp::Feature32SDescriptor& aCoor1Desc,
            icp::Feature32SDescriptor& aCoor1Desc_O
)
{


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
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   /* early exit with 0 features to track */
   if (aCoor0Desc.GetCount() == 0 ||
         aCoor1Desc.GetCount() == 0)
   {
      mNumFeatures = 0;
      return APEXCV_SUCCESS;
   }

   if (mInit)
   {

      int width  = aSrc0Desc.cols;
      int height = aSrc0Desc.rows;
      int span   = aSrc0Desc.step[0];
      vsdk::SMat Src0DescMat = aSrc0Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
      vsdk::SMat Src1DescMat = aSrc1Desc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   
      int span_padded =  mSrc0PadDesc.step[0];
      mParamsDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = (aSrc1Desc.cols << LKTRACKER_FLOATING_COOR_Q);
      mParamsDesc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(1u) = (aSrc1Desc.rows<< LKTRACKER_FLOATING_COOR_Q);

      LKTRACKER_FULL_7X7* lProcess = (LKTRACKER_FULL_7X7*) mProcess;

      //**************************************************
      // connect feature coordiates buffer to APU process 
      //**************************************************
      {
         mNumFeatures = aCoor0Desc.GetCount();
         int numFeatures = (mNumFeatures+3)&(~0x3);
         
         mCoor0Desc = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor0Desc.GetDataPtrPhys(), (uint8_t *)aCoor0Desc.GetDataPtr());
         mCoor1Desc = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor1Desc.GetDataPtrPhys(), (uint8_t *)aCoor1Desc.GetDataPtr());
         mCoor1Desc_O = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC4, (uint8_t *)aCoor1Desc_O.GetDataPtrPhys(), (uint8_t *)aCoor1Desc_O.GetDataPtr());
         mSrc0Offs = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC1, (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB));
         mSrc1Offs = vsdk::CustomPointerSUMat(1, numFeatures, VSDK_CV_32SC1, (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY), (uint8_t *)OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_CH_WB));
         
         if (lProcess->ConnectIO("INPUT_COOR_XY0",  mCoor0Desc)
             || lProcess->ConnectIO("INPUT_COOR_XY1",  mCoor1Desc)
             || lProcess->ConnectIO("OUTPUT_COOR_XY1", mCoor1Desc_O))
         {
            return APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }
      }

      if (mSrcPadded == 0) /* Input frames does not have image border padding area,
                              we need us internal allocated frame buffer and do memory copy and border replication */
      {
        //******************************************************
        // COPY INPUT TEMPLATE FRAME INTO INTERNAL FRAME BUFFER
        //******************************************************
         uint8_t *p = (uint8_t*)mSrc0PadDesc.u->data;
         uint8_t *s = (uint8_t*)Src0DescMat.data;
         for (int y = 0; y < (int)height; y++)
         {
            memcpy(p, s, width);
            p += span_padded;
            s += span;
         }
         
         //***********************************
         // PERFORM BORDER PIXELS REPLICATION 
         //***********************************
         icp::DataDescriptor Src0DescPadDD, Src1DescPadDD;
         Src0DescPadDD.InitManual(mSrc0PadDesc.cols, mSrc0PadDesc.rows, mSrc0PadDesc.u->data, mSrc0PadDesc.u->handle, icp::DATATYPE_08U);
         Src0DescPadDD.SetSpan(mSrc0PadDesc.step[0]);
         Src1DescPadDD.InitManual(mSrc1PadDesc.cols, mSrc1PadDesc.rows, mSrc1PadDesc.u->data, mSrc1PadDesc.u->handle, icp::DATATYPE_08U);
         Src1DescPadDD.SetSpan(mSrc1PadDesc.step[0]);
         Src0DescPadDD.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);

        //******************************************************
        // COPY INPUT PATCH FRAME INTO INTERNAL FRAME BUFFER
        //******************************************************
         p = (uint8_t*)mSrc1PadDesc.u->data;
         s = (uint8_t*)Src1DescMat.data;
         for (int y = 0; y < (int)height; y++)
         {
            memcpy(p, s, width);
            p += span_padded;
            s += span;
         }
         
         //***********************************
         // PERFORM BORDER PIXELS REPLICATION 
         //***********************************
         icp::DataDescriptor Src0DescDD, Src1DescDD;
         Src0DescDD.InitManual(Src0DescMat.cols, Src0DescMat.rows, Src0DescMat.data, (int8_t *)aSrc0Desc.u->handle + aSrc0Desc.offset, icp::DATATYPE_08U);
         Src0DescDD.SetSpan(Src0DescMat.step[0]);
         Src1DescDD.InitManual(Src1DescMat.cols, Src1DescMat.rows, Src1DescMat.data, (int8_t *)aSrc0Desc.u->handle + aSrc1Desc.offset, icp::DATATYPE_08U);
         Src1DescDD.SetSpan(Src1DescMat.step[0]);
         Src1DescPadDD.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);

         //****************************************
         // FLUSH FILLED FRAME BUFFERS INTO MEMORY
         //****************************************
         int full_height = 2*IMAGE_PAD + height;
         OAL_MemoryFlushAndInvalidate((uint8_t*)mSrc0PadDesc.u->data-IMAGE_PAD-IMAGE_PAD*span_padded, full_height*span_padded);
         OAL_MemoryFlushAndInvalidate((uint8_t*)mSrc1PadDesc.u->data-IMAGE_PAD-IMAGE_PAD*span_padded, full_height*span_padded);
         //******************************************************************
         // CALCULATE TEMPLATE AND PATCH FEATURES OFFSET FOR MOTION COMP DMA
         //******************************************************************
         LKTrackerCalcOffset(mSrc0PadDesc, aCoor0Desc, mSrc0Offs, mBoxSize>>1, 1, 
               mSrcPadded == 0 || (mSrcPadded == 1 && width < mPadStartWidth) || mSrcPadded == 2);
         LKTrackerCalcOffset(mSrc1PadDesc, aCoor1Desc, mSrc1Offs, mBoxSize>>1, LKTRACKER_PATCH_BORDER, 
               mSrcPadded == 0 || (mSrcPadded == 1 && width < mPadStartWidth) || mSrcPadded == 2);
         if (lProcess->ConnectIndirectInput("INPUT_TEMPLATE", mSrc0PadDesc, mSrc0Offs)
             || lProcess->ConnectIndirectInput("INPUT_PATCH",    mSrc1PadDesc, mSrc1Offs))
         {
             return APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }
      }
      else /* Input frames does have image border padding area allocated already */
      {
         if (mSrcPadded == 1 && width < mPadStartWidth) /* border pixels hasn't been replicated yet and image width is less than threshold, 
                                                           we perform replication here */
         {
            //*************************
            // REPLICATE BORDER PIXELS
            //*************************
            icp::DataDescriptor Src0DescDD, Src1DescDD;
            Src0DescDD.InitManual(Src0DescMat.cols, Src0DescMat.rows, Src0DescMat.data, (int8_t *)aSrc0Desc.u->handle + aSrc0Desc.offset, icp::DATATYPE_08U);
            Src0DescDD.SetSpan(Src0DescMat.step[0]);
            Src1DescDD.InitManual(Src1DescMat.cols, Src1DescMat.rows, Src1DescMat.data, (int8_t *)aSrc0Desc.u->handle + aSrc1Desc.offset, icp::DATATYPE_08U);
            Src1DescDD.SetSpan(Src1DescMat.step[0]);
         
            int full_height = 2*IMAGE_PAD + height;
            Src0DescDD.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
            Src1DescDD.E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
            OAL_MemoryFlushAndInvalidate((uint8_t*)Src0DescMat.data-IMAGE_PAD-IMAGE_PAD*span, full_height*span);
            OAL_MemoryFlushAndInvalidate((uint8_t*)Src1DescMat.data-IMAGE_PAD-IMAGE_PAD*span, full_height*span);
         }

         //******************************************************************
         // CALCULATE TEMPLATE AND PATCH FEATURES OFFSET FOR MOTION COMP DMA
         //******************************************************************
         LKTrackerCalcOffset(aSrc0Desc, aCoor0Desc, mSrc0Offs, mBoxSize>>1, 1, 
            mSrcPadded == 0 || (mSrcPadded == 1 && width < mPadStartWidth) || mSrcPadded == 2);
         LKTrackerCalcOffset(aSrc1Desc, aCoor1Desc, mSrc1Offs, mBoxSize>>1, LKTRACKER_PATCH_BORDER, 
            mSrcPadded == 0 || (mSrcPadded == 1 && width < mPadStartWidth) || mSrcPadded == 2);

         if (lProcess->ConnectIndirectInput("INPUT_TEMPLATE", aSrc0Desc, mSrc0Offs)
             || lProcess->ConnectIndirectInput("INPUT_PATCH", aSrc1Desc, mSrc1Offs))
         {
            return APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }
      }
   }
   else
   {
      //******************************************
      // CLASS HASN'T BEEN INITIALIZED, WE RETURN
      //******************************************
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

/**************
 * APU process
 *************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::Process(
)
{

   int reval = 0;

   //********************************
   // NOTHING NEEDS TO DO, WE RETURN
   //********************************
   if (mNumFeatures == 0)
   {
      return APEXCV_SUCCESS;
   }
 
   if (mInit)
   {
     LKTRACKER_FULL_7X7* lProcess = (LKTRACKER_FULL_7X7*) mProcess;

     if (mBoxSize == 3)
      {
         // TODO: unsupported box window size yet
      }
      else if (mBoxSize == 5)
      {
         // TODO: unsupported box window size yet
      }
      else if (mBoxSize == 7)
      {
         //**************************
         //EVERYTHING IS READY TO GO
         //**************************
         OAL_MemoryFlushAndInvalidate((uint8_t*)mCoor0Desc.u->data,   mCoor0Desc.rows   * mCoor0Desc.step[0]);
         OAL_MemoryFlushAndInvalidate((uint8_t*)mCoor1Desc.u->data,   mCoor1Desc.rows   * mCoor1Desc.step[0]);
         OAL_MemoryFlushAndInvalidate((uint8_t*)mCoor1Desc_O.u->data, mCoor1Desc_O.rows * mCoor1Desc_O.step[0]);
         OAL_MemoryFlushAndInvalidate((uint8_t*)mSrc0Offs.u->data,    mSrc0Offs.rows    * mSrc0Offs.step[0]);
         OAL_MemoryFlushAndInvalidate((uint8_t*)mSrc1Offs.u->data,    mSrc1Offs.rows    * mSrc1Offs.step[0]);
         reval  = lProcess->Start();
         reval |= lProcess->Wait();
         if (reval)
         {
            return APEXCV_ERROR_ACF_PROC_EXEC;
         }
      }
      else  /* == 9 */
      {
         // TODO: unsupported box window size yet
      }
   }
   else
   {
      // class hasn't been initialized yet, return error
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }


   return APEXCV_SUCCESS;
}

/************************************
 * Select APU and APEX configuration
 ***********************************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                                 int32_t aApexId)
{
   mApuConfig = aApuConfig;
   mApexId    = aApexId;

   mWeSelectedApuConfiguration = true;

   return APEXCV_SUCCESS;
}

/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT LKTrackerOpticalFlow::SelectApexCore(int32_t aApexId)
{
   LKTRACKER_FULL_7X7* lProcess = (LKTRACKER_FULL_7X7*) mProcess;

   if (!mInit)
   {
      //******************************************
      // CLASS HASN'T BEEN INITIALIZED, WE RETURN
      //******************************************
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   if (lProcess->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
   {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
   }

   mApexId = aApexId;
   return APEXCV_SUCCESS;
}

}  /* namespace apexcv */


/****************************************************************************************************************** 
 * calculate motion comp DMA buffer address offset based on features' X/Y coordinate and image's width/height/span
 *****************************************************************************************************************/
static void LKTrackerCalcOffset(const vsdk::SUMat& aSrcDesc, icp::Feature32SDescriptor& aCoorDesc, vsdk::CustomPointerSUMat& aOffsDesc, int aHalfWin, int aPad, bool aImgReplicated)
{
   // targe offset buffer we need to fill in for motion comp DMA
   int32_t* p_offset = (int32_t*)aOffsDesc.u->data;

   // feature descriptor's buffer
   icp::Feature32S *mpData = (icp::Feature32S*)aCoorDesc.GetDataPtr();

   int srcWidth   = aSrcDesc.cols;
   int srcHeight  = aSrcDesc.rows;
   int srcSpan    = aSrcDesc.step[0];
   int srcElmSize = aSrcDesc.elemSize1();

   int co_max_x = srcWidth  - (LKTRACKER_PATCH_BW_7x7 - (aHalfWin + LKTRACKER_PATCH_BORDER));
   int co_max_y = srcHeight - (LKTRACKER_PATCH_BW_7x7 - (aHalfWin + LKTRACKER_PATCH_BORDER));
   int co_min   = aHalfWin + LKTRACKER_PATCH_BORDER;

   // calculate the offset between windown's top left corner with the center of the window
   int top_left_off = srcSpan*(aHalfWin + aPad) + (aHalfWin + aPad) * srcElmSize;

   for (int i = 0; i < aCoorDesc.GetCount(); i++)
   {
      // All feature's X/Y coordinate are in Q23.8 format
      int32_t x = mpData->position.x; 
      int32_t y = mpData->position.y; 
      // right shift to get the integer part
      int32_t i_x = x >> LKTRACKER_FLOATING_COOR_Q;
      int32_t i_y = y >> LKTRACKER_FLOATING_COOR_Q;

      // img haven't be replicated, need to shift into boundary */
      if (!aImgReplicated)
      {
         if (co_max_x < 0) co_max_x = 0;
         if (co_max_y < 0) co_max_y = 0;
         if (i_x < co_min) i_x = co_min;
         if (i_y < co_min) i_y = co_min;
         if (i_x > co_max_x) i_x = co_max_x;
         if (i_y > co_max_y) i_y = co_max_y;
      }
      p_offset[i] = srcSpan * i_y + srcElmSize *i_x;
      p_offset[i] -= top_left_off;
      mpData+=1;
   }
   //*****************************************************************
   // we have to zero out all unsured offset to make sure DMA is safe
   //*****************************************************************
   if (aOffsDesc.cols > aCoorDesc.GetCount())
   {
      for (int i = aCoorDesc.GetCount(); i < aOffsDesc.cols; i++)
      {
         p_offset[i] = 0;
      }
   }
}
