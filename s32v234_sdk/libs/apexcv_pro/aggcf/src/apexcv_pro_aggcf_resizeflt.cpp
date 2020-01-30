/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2017 NXP
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

// ACF HOST LIB
#include <icp_data.h>
#include <oal.h>
#include <mcdma_hal.h>
#include <global_hal.h>
#include <cmem_if_hal.h>
#include <iostream>
#include <cstdio>

// USER DEFINED HOST API
#include <apexcv_pro_aggcf.h>
#include "resize_definitions.h"
#include <RESIZE16_H_V_FLT.hpp>
#include <RESIZE16_V_H_FLT.hpp>
#include <RESIZE16_H_FLT.hpp>
#include <RESIZE16_V_FLT.hpp>
#include <cmath>
#include <acf_common.h>
#include <linked_list_utilities.h>
#include <icp_datatype.h>


using namespace std;

// Prototype

static int32_t QueryApuArraySize(int32_t& aArrayWidth);
static APEXCV_LIB_RESULT testResizeParams(int aSrcWidth, int aSrcHeight, int aDstWidth, int aDstHeight);


///// C++ interface //////////////////////////////////////////////////
namespace apexcv{

AGGCFResizeFlt::AGGCFResizeFlt()
   : mInitState(0),
     mApuConfig(ACF_APU_CFG__DEFAULT),
     //AcfProfilingInfo mApuProfiling;
     mApexId(0),
     mWeSelectedApuConfiguration(false)
{
}

AGGCFResizeFlt::~AGGCFResizeFlt()
{
}

APEXCV_LIB_RESULT AGGCFResizeFlt::Initialize(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                                             vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */,
                                             int aP  )
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState == 0)
   {
      mSrcImage_umat = aSrcImage;
      mDstImage_umat = aDestImage;

      // Test AGGCFResizeFlt parameters to ensure they are supported
      lRetVal = testResizeParams(aSrcImage.cols,
                                  aSrcImage.rows,
                                  aDestImage.cols,
                                  aDestImage.rows);
      if ( APEXCV_SUCCESS == lRetVal)
      {
         lRetVal = InitParams(aP);
         if(APEXCV_SUCCESS == lRetVal )
         {
            mInitState = 1;
         }
      }
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::ReconnectIO(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                                              vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */,
                                              int aP )
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   mInitState = 0;
   mSrcImage_umat.release();
   mDstImage_umat.release();
   lRetVal = Initialize(aSrcImage, aDestImage, aP);
   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::InitParams(int aP)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if( aP < 1 || aP > 5 )
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   mParams = vsdk::SUMat(1, 2, VSDK_CV_32SC1);

   if(!mParams.empty())
   {
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = aP;
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(1u) = 0;
   }
   else
   {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }
 
   return lRetVal;

}


void initMoOffsets16Umat(vsdk::SUMat &aSrc, vsdk::SUMat &aMoOffs, int32_t aDw, int32_t aDh, resize_descriptor* aHrsz, resize_descriptor* vrsz)
{
   const int32_t hdscl = HRSZ_DECIMAL_SCL;
   const int32_t vdscl = VRSZ_DECIMAL_SCL;

   uint32_t moWidth  = aMoOffs.cols;
   uint32_t moHeight = aMoOffs.rows;

   int srcSpan = aSrc.step[0]; 
 

   vsdk::SMat matTemp = aMoOffs.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   int32_t* p = (int32_t *)matTemp.data; 

   if (vrsz == NULL)
   {
      for (uint32_t y = 0; y < moHeight; ++y)
      {
         int32_t hsrcOffs = aHrsz->src_offs;
         for (uint32_t x = 0; x < moWidth; ++x)
         {
            p[y * moWidth + x] = srcSpan * y * aDh + (hsrcOffs >> hdscl)*sizeof(uint16_t);
            hsrcOffs += aHrsz->scl_fact * aDw;
         }
      }
   }
   else if (aHrsz == NULL)
   {
      int32_t vsrcOffs = vrsz->src_offs;

      for (uint32_t y = 0; y < moHeight; ++y)
      {
         for (uint32_t x = 0; x < moWidth; ++x)
         {
            p[y * moWidth + x] = srcSpan * (vsrcOffs >> vdscl) + x * aDw*sizeof(uint16_t);
         }

         vsrcOffs += vrsz->scl_fact * aDh;
      }
   }
   else
   {
      int32_t vsrcOffs = vrsz->src_offs;

      for (uint32_t y = 0; y < moHeight; ++y)
      {
         int32_t hsrcOffs = aHrsz->src_offs;
         for (uint32_t x = 0; x < moWidth; ++x)
         {
            p[y * moWidth + x] = srcSpan * (vsrcOffs >> vdscl) + (hsrcOffs >> hdscl)*sizeof(uint16_t);
            hsrcOffs += aHrsz->scl_fact * aDw;
         }

         vsrcOffs += vrsz->scl_fact * aDh;
      }
   }
}

int getPolyphaseParams(int16_t aSsiz, int16_t aDsiz, int32_t aDscl, resize_descriptor* aRsz_desc)
{
   if (aSsiz >= 2*aDsiz)
   {
      aRsz_desc->phases = 2; //! 4 Phases
      aRsz_desc->taps   = 8; 
   }
   else if (aSsiz > aDsiz)
   {
      aRsz_desc->phases = 3; //! 8 Phases
      aRsz_desc->taps   = 4;
   }
   else if  (aSsiz < aDsiz)
   {
      aRsz_desc->phases = 3; //! 8 Phases
      aRsz_desc->taps   = 4;
   }
   else
   {
      return 1;
   }

   aRsz_desc->scl_fact  = (aSsiz * (1 << aDscl)) / aDsiz;
   aRsz_desc->src_offs  = (aRsz_desc->scl_fact >> 1) - (((aRsz_desc->taps - 1) >> 1) << aDscl);
   aRsz_desc->out_scale = 8;
   aRsz_desc->out_round = 1 << (aRsz_desc->out_scale - 1);
  
   return 0;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::Process(vsdk::SUMat &aSrc,
                                  vsdk::SUMat &aDst)
{
   APEXCV_LIB_RESULT intResult = APEXCV_SUCCESS;
   
   if( 0 == mInitState)
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   if (aSrc.cols == aDst.cols && aSrc.rows == aDst.rows)
   {
      return copy16(aSrc, aDst);
   }
   else if (aSrc.cols == aDst.cols)
   {
      return ACFVrszFlt16(aSrc, aDst);
   }
   else if (aSrc.rows == aDst.rows)
   {
      return ACFHrszFlt16(aSrc, aDst);
   }
   else
   {
      return AcfResizeFlt16(aSrc, aDst);
   }

   return intResult;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                                                         int32_t aApexId)
{
   // hard coded to 0/1 for valid input parameter check
   if( aApexId < 0 || aApexId > 1)
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   mApuConfig = aApuConfig;
   mApexId    = aApexId;

   mWeSelectedApuConfiguration = true;

   return APEXCV_SUCCESS;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::copy16(vsdk::SUMat& aSrc,
                                         vsdk::SUMat& aDst)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   vsdk::SMat dstMat = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   uint16_t *dst16 = (uint16_t *)dstMat.data;
   uint16_t *src16 = (uint16_t *)srcMat.data;

   int src_width  = aSrc.cols;
   int src_height = aSrc.rows;

   int instride = aSrc.step[0]/2;
   int outstride = aDst.step[0]/2;

   for(int i = 0; i < src_height; i++)
   {
      memcpy(dst16, src16, src_width*sizeof(uint16_t));
      dst16 += outstride;
      src16 += instride;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::ACFHrszFlt16(vsdk::SUMat& aSrc,
                                               vsdk::SUMat& aDst)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   bool weKeepGoing = true;
   vsdk::SUMat moInvalid;
   vsdk::SUMat moOffs;
   vsdk::SUMat srcWidth;
   vsdk::SUMat dstWidth;

   uint32_t tileSizeInBytes = 0xffffffff;
   int32_t dstChunkWidth;
   int32_t dstChunkHeight;
   int32_t srcChunkWidth;
   int32_t srcChunkHeight;
   int32_t tileWidthInChunks;
   int32_t oldChunkWidth;
   int32_t oldChunkHeight;
   int32_t moH;

   resize_descriptor hrsz;
   getPolyphaseParams(aSrc.cols,
                        aDst.cols,
                        HRSZ_DECIMAL_SCL, &hrsz);

   /*
   ** maintain cache coherent
   */
   uint16_t * lsrc    = (uint16_t *)((uintptr_t)aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data);
   OAL_MemoryFlushAndInvalidate( lsrc, aSrc.rows*aSrc.step[0]/2*sizeof(uint16_t));

   RESIZE16_H_FLT* lProcess = new RESIZE16_H_FLT();

   lRetVal = lProcess->Initialize();

   if( APEXCV_SUCCESS != lRetVal )
   {
      if (lProcess != 0) delete lProcess;
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;   
   } 
   
   if( mWeSelectedApuConfiguration == true )
   { 
      if(lProcess->SelectApuConfiguration(mApuConfig, mApexId))
      {
         if (lProcess != 0) delete lProcess;
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }   
   }

   int32_t numCU = lProcess->RetCuArrayWidth();

   srcWidth  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   srcWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;

   dstWidth  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   dstWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;

   dstChunkWidth = (aDst.cols + numCU - 1) / numCU;
   dstChunkWidth = (dstChunkWidth + 0x1) & ~0x1;
   dstChunkWidth = dstChunkWidth > 0 ? dstChunkWidth : 2;

   srcChunkWidth = (dstChunkWidth * aSrc.cols + aDst.cols - 1) / aDst.cols;
   srcChunkWidth += hrsz.taps;
   srcChunkWidth = (srcChunkWidth + 0x3) & ~0x3;

   tileWidthInChunks = (aDst.cols + dstChunkWidth - 1) / dstChunkWidth;
   tileWidthInChunks = (tileWidthInChunks + 0x3) & ~0x3;

   vsdk::Rect lRect_t(0, 0, dstChunkWidth * tileWidthInChunks, aDst.rows);
   vsdk::SUMat dstTemp(aDst, lRect_t);

   moInvalid = vsdk::SUMat( 1, tileWidthInChunks, VSDK_CV_32SC1 );

   if( lProcess->ConnectIndirectInput("SRC", aSrc, moInvalid)
       || lProcess->ConnectIO("DST", dstTemp)
       || lProcess->ConnectIO("SRC_WIDTH", srcWidth)
       || lProcess->ConnectIO("DST_WIDTH", dstWidth) )
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      weKeepGoing = false; 
   }

   if (lProcess->SelectScenario("", 0, 0) != 0) 
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      weKeepGoing = false;
   }

   if (weKeepGoing == true)
   {
      lProcess->QueryPortChunkSize("SRC", oldChunkWidth, oldChunkHeight);

      int availCmem;
      availCmem = lProcess->RetCmemHeapSizeInBytes() - 320;
      dstChunkHeight = (lProcess->RetCmemHeapSizeInBytes() - 320) / (((2 * srcChunkWidth) + 2 * dstChunkWidth + 3*(dstChunkWidth+2))*sizeof(uint16_t));
      dstChunkHeight &= ~0x1;
      dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
      dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;

      srcChunkHeight = dstChunkHeight;

      tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);

      int memSize;
      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
      memSize += tileWidthInChunks*4; 
  
      while  ( ((tileSizeInBytes >= (1<<16) || srcChunkHeight > 64) && dstChunkHeight > 1) || memSize > availCmem )
      {
         dstChunkHeight -= 2;

         dstChunkHeight &= ~0x1;
         dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
         dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;

         srcChunkHeight = dstChunkHeight;

         tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);

         memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
         memSize += tileWidthInChunks*4; 
      }

      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
      memSize += tileWidthInChunks*4; 
   
      if (tileSizeInBytes >= (1<<16) || memSize > availCmem) 
      {
         lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         weKeepGoing = false;
      }

      if (weKeepGoing == true)
      {
         lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = srcChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = srcChunkHeight;
         lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dstChunkHeight;
         lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = dstChunkHeight;

         moH = (aDst.rows + dstChunkHeight - 1) / dstChunkHeight;

         if (weKeepGoing == true)
         {
            moOffs = vsdk::SUMat( moH, tileWidthInChunks, VSDK_CV_32SC1 );
            initMoOffsets16Umat(aSrc, moOffs, dstChunkWidth, dstChunkHeight, &hrsz, NULL);

            vsdk::Rect lRect(0,
                             0,
                             dstChunkWidth * tileWidthInChunks,
                             aDst.rows);

            vsdk::SUMat dst_t(aDst, lRect);

            if (aDst.empty() == true)
            {
               lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
               weKeepGoing = false;
            }

            if (weKeepGoing == true)
            {
               {
                   vsdk::SMat matTemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
                   //vsdk::SMat srctemp = src.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
               }

               if( lProcess->ConnectIndirectInput("SRC", aSrc, moOffs)
                   || lProcess->ConnectIO("DST", dst_t)
                   || lProcess->ConnectIO("PARAMS", mParams))
               {
                  lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
               }
               else
               {  
                  lProcess->FlushAndInvalidateAllIO();
                  int status = 0; 
                  status  = lProcess->Start();
                  status |= lProcess->Wait();

                  if( status)
                  {
                     lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
                  }
                  else
                  {
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = oldChunkHeight;
                  } 
               }
            }
         }
      }
   }

//exit:
   if (lProcess != 0) delete lProcess;

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFResizeFlt::ACFVrszFlt16(vsdk::SUMat& aSrc,
                                               vsdk::SUMat& aDst)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   bool weKeepGoing = true;
   
   vsdk::SUMat moInvalid;
   vsdk::SUMat moOffs;  
   vsdk::SUMat srcHeight;
   vsdk::SUMat dstHeight;
   
   uint32_t tileSizeInBytes = 0xffffffff;
   int32_t dstChunkWidth;
   int32_t dstChunkHeight;
   int32_t srcChunkWidth;
   int32_t srcChunkHeight;
   int32_t tileWidthInChunks;
   int32_t oldChunkWidth;
   int32_t oldChunkHeight;
   int32_t moH;
   float vScale;

   resize_descriptor vrsz;
   getPolyphaseParams(aSrc.rows,
                        aDst.rows,
                        VRSZ_DECIMAL_SCL, &vrsz);
   /*
   ** maintain cache coherent
   */
   vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   uint16_t * lsrc    = (uint16_t *)srcMat.data;
   OAL_MemoryFlushAndInvalidate( lsrc, aSrc.rows*aSrc.step[0]/2*sizeof(uint16_t));
      
   RESIZE16_V_FLT *lProcess = new RESIZE16_V_FLT();
   lRetVal |= lProcess->Initialize();

   if( APEXCV_SUCCESS != lRetVal )
   {  
      if (lProcess != 0) delete lProcess;
      return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
   }

   if(mWeSelectedApuConfiguration == true)
   {
      if( lProcess->SelectApuConfiguration(mApuConfig, mApexId))
      {
         if (lProcess != 0) delete lProcess;
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;      
      }   
   }
   
   int32_t numCU = lProcess->RetCuArrayWidth();

   srcHeight = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   srcHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
   dstHeight = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   dstHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;

   dstChunkWidth = (aDst.cols + numCU - 1) / numCU;
   dstChunkWidth = (dstChunkWidth + 0x1) & ~0x1; 
   dstChunkWidth = dstChunkWidth > 0 ? dstChunkWidth : 2;
   
   srcChunkWidth = dstChunkWidth;
   srcChunkWidth = (srcChunkWidth + 0x3) & ~0x3;
   
   tileWidthInChunks = (aDst.cols + dstChunkWidth - 1) / dstChunkWidth;
   tileWidthInChunks = (tileWidthInChunks + 0x3) & ~0x3;
   
   vsdk::Rect lRect_t(0, 0, dstChunkWidth * tileWidthInChunks, aDst.rows);
   vsdk::SUMat dstTemp(aDst, lRect_t);

   moInvalid = vsdk::SUMat( 1, tileWidthInChunks, VSDK_CV_32SC1 ); 

   if( lProcess->ConnectIndirectInput("SRC", aSrc, moInvalid)
       || lProcess->ConnectIO("DST", dstTemp)
       || lProcess->ConnectIO("SRC_HEIGHT", srcHeight)
       || lProcess->ConnectIO("DST_HEIGHT", dstHeight))
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      weKeepGoing = false;
   } 
   
   if (lProcess->SelectScenario("", 0, 0) != 0)
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      weKeepGoing = false;
   }

   if (weKeepGoing == true)
   {
      lRetVal |= lProcess->QueryPortChunkSize("SRC", oldChunkWidth, oldChunkHeight);

      int availCmem;
      availCmem = lProcess->RetCmemHeapSizeInBytes() - 320;
      
      vScale = (float)aSrc.rows / (float)aDst.rows;
      dstChunkHeight = (lProcess->RetCmemHeapSizeInBytes() - 320) / (((int)(2 * vScale * srcChunkWidth) + 2 * dstChunkWidth + 3*(dstChunkWidth+2))*sizeof(uint16_t));
      dstChunkHeight &= ~0x1;
      dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
      dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;
      
      srcChunkHeight = ceil(dstChunkHeight * vScale);
      srcChunkHeight += vrsz.taps;
      srcChunkHeight = (srcChunkHeight + 0x1) & ~0x1;

      tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);

      int memSize;
      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
      memSize += tileWidthInChunks*4; 
   
      while ( ((tileSizeInBytes >= (1<<16) || srcChunkHeight > 64) && dstChunkHeight > 1) || memSize > availCmem )
      {
         dstChunkHeight -= 2;

         dstChunkHeight &= ~0x1;
         dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
         dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;

         srcChunkHeight = ceil(dstChunkHeight * vScale);
         srcChunkHeight += vrsz.taps;
         srcChunkHeight = (srcChunkHeight + 0x1) & ~0x1;

         tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);

         memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
         memSize += tileWidthInChunks*4; 
      }

      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2 + (dstChunkWidth+2) * dstChunkHeight * 3+(dstChunkWidth+2)*2)*sizeof(uint16_t);
      memSize += tileWidthInChunks*4; 

      if (tileSizeInBytes >= (1<<16) || memSize > availCmem) 
      {
         lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         weKeepGoing = false;
      }
   
      if (weKeepGoing == true)
      {
         lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = srcChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = srcChunkHeight;
         lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dstChunkHeight;
         lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = dstChunkHeight;

         moH = (aDst.rows + dstChunkHeight - 1) / dstChunkHeight;

         //printf("  dst.rows %d mo_h %d  dst_chunk_height %d  tile_width_in_chunks %d dst_chunk_width %d \n ", dst.rows, mo_h, dst_chunk_height, tile_width_in_chunks, dst_chunk_width);

         if (weKeepGoing == true)
         {

            moOffs = vsdk::SUMat( moH, tileWidthInChunks, VSDK_CV_32SC1 );       

            initMoOffsets16Umat(aSrc, moOffs, dstChunkWidth, dstChunkHeight, NULL, &vrsz);

            vsdk::Rect lRect(0,
                             0,
                             dstChunkWidth * tileWidthInChunks,
                             aDst.rows);

            vsdk::SUMat dst_t(aDst, lRect);

            if (aDst.empty() == true)
            {
               lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
               weKeepGoing = false;
            }

            if (weKeepGoing == true)
            {
               {
                  vsdk::SMat matTemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
               }

               if( lProcess->ConnectIndirectInput("SRC", aSrc, moOffs)
                    || lProcess->ConnectIO("DST", dst_t) 
                    || lProcess->ConnectIO("PARAMS", mParams) )
               {
                  lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
               } 
               else
               {  
                  lProcess->FlushAndInvalidateAllIO();   
                  int status = 0;  
                  status  = lProcess->Start();
                  status |= lProcess->Wait();
   
                  if( status)
                  {
                     lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
                  }
                  else
                  {
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = oldChunkHeight;
                  } 
               }  
            }
         }
      }
   }

//exit:
   if (lProcess != 0) delete lProcess;
   
   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFResizeFlt::AcfResizeFlt16(vsdk::SUMat& aSrc,
                                                 vsdk::SUMat& aDst)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   bool weKeepGoing = true;
  
   vsdk::SUMat moInvalid;
   vsdk::SUMat moOffs;
   vsdk::SUMat srcWidth;
   vsdk::SUMat dstWidth;    
   vsdk::SUMat srcHeight;
   vsdk::SUMat dstHeight;
      
   uint32_t tileSizeInBytes = 0xffffffff;
   int32_t dstChunkWidth;
   int32_t dstChunkHeight;
   int32_t srcChunkWidth;
   int32_t srcChunkHeight;
   int32_t tileWidthInChunks;
   int32_t oldChunkWidth;
   int32_t oldChunkHeight;
   int32_t moH;
   float vScale;

   resize_descriptor hrsz;
   getPolyphaseParams(aSrc.cols, aDst.cols, HRSZ_DECIMAL_SCL, &hrsz);
   
   resize_descriptor vrsz;
   getPolyphaseParams(aSrc.rows, aDst.rows, VRSZ_DECIMAL_SCL, &vrsz);
   
   /* 
   ** maintain cache coherent
   */ 
   vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   uint16_t * lsrc    = (uint16_t *)srcMat.data;
   OAL_MemoryFlushAndInvalidate( lsrc, aSrc.rows*aSrc.step[0]/2*sizeof(uint16_t));
   
   /*
   ** HRSZ is slower than VRSZ, so pick the order based on performance ratio
   */
   bool vrsz_first;
   ACF_Process_APU *lProcess;
   int src_size = aSrc.cols * aSrc.rows;

   if ((0.75 * (float)src_size + (float)(aSrc.cols * aDst.rows)) < ((float)src_size + (float)aDst.cols * (float)aSrc.rows * 0.75))
   {
      lProcess = new RESIZE16_V_H_FLT();
      vrsz_first = true;
   }
   else
   {
      lProcess = new RESIZE16_H_V_FLT();
      vrsz_first = false;
   }
   lRetVal = lProcess->Initialize();
   if( APEXCV_SUCCESS != lRetVal )
   {
      if (lProcess != 0) delete lProcess;
      return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
   }

   //printf(" Initialize %d  ", lRetVal);

   if(mWeSelectedApuConfiguration == true)
   {
      if(lProcess->SelectApuConfiguration(mApuConfig, mApexId))
      { 
         if (lProcess != 0) delete lProcess;
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;    
      }
   }

   int32_t numCU = lProcess->RetCuArrayWidth();
   
   /*
   ** setup and connect ports to be able to call select scenario
   */
   
   srcWidth  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   srcWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;

   dstWidth  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 ); 
   dstWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;

   srcHeight = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   srcHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
   
   dstHeight = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
   dstHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;

   /* 
   ** Need to calculate chunk width right now to ensure the DST port
   ** is a multiple. Otherwise calling select scenario with the raw
   ** DST width may fail
   */
   dstChunkWidth = (aDst.cols + numCU - 1) / numCU;
   dstChunkWidth = (dstChunkWidth + 0x1) & ~0x1; 
   dstChunkWidth = dstChunkWidth > 0 ? dstChunkWidth : 2;
   
   srcChunkWidth = (dstChunkWidth * aSrc.cols + aDst.cols - 1) / aDst.cols;
   srcChunkWidth += hrsz.taps;
   srcChunkWidth = (srcChunkWidth + 0x3) & ~0x3;
   
   tileWidthInChunks = (aDst.cols + dstChunkWidth - 1) / dstChunkWidth;
   tileWidthInChunks = (tileWidthInChunks + 0x3) & ~0x3;
   
   moInvalid = vsdk::SUMat( 1, tileWidthInChunks, VSDK_CV_32SC1 );    

   vsdk::Rect lRect_t(0, 0, dstChunkWidth * tileWidthInChunks,
                             aDst.rows);

   vsdk::SUMat dstTemp(aDst, lRect_t);

   if( lProcess->ConnectIndirectInput("SRC", aSrc, moInvalid)
       || lProcess->ConnectIO("DST", dstTemp)
       || lProcess->ConnectIO("SRC_WIDTH", srcWidth)
       || lProcess->ConnectIO("DST_WIDTH", dstWidth)
       || lProcess->ConnectIO("SRC_HEIGHT", srcHeight)
       || lProcess->ConnectIO("DST_HEIGHT", dstHeight) )
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      weKeepGoing = false;
   }
  
   //printf(" connectIO %d  ", lRetVal);

   if (lProcess->SelectScenario("", 0, 0) != 0) 
   {
      lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      weKeepGoing = false;
   }

   if (weKeepGoing == true)
   {
      lProcess->QueryPortChunkSize("SRC", oldChunkWidth, oldChunkHeight);

      /*
      ** Now that a scenario has been selected we can modify/hack it however we want
      ** since we know the available CMEM. The chunk heights are determined by creating a
      ** system of linear equations/inequalities that includes: the relationship between
      ** every non-fixed port size and an inequality between the available CMEM and the sum
      ** of every port buffer in the graph. Simplifying the system of equations leaves one unknown
      ** in the inequality which is the dst chunk height. Solving for this inequality gives us
      ** the dst chunk height which allows us to calculate the src chunk height as well.
      */

      int availCmem;
      availCmem = lProcess->RetCmemHeapSizeInBytes() - 320;
      
      vScale = (float)aSrc.rows / (float)aDst.rows;
   
      if (vrsz_first) 
      {
         // vertical resize kernel: dual input ==> output => horizontal resize kernel => output => filtering => dual output buffer   
         // 2*vScale*dstChunkHeight*srcChunkWidth  +  dstChunkHeight*srcChunkWidth +  3*dstChunkHeight*(dstChunkWidth+2) + 2*dstChunkHeight*dstChunkWidth 
         dstChunkHeight = (lProcess->RetCmemHeapSizeInBytes() - 320) / ((int)((2 * vScale + 1) * srcChunkWidth) + dstChunkWidth*2 + 3*(dstChunkWidth+2));
      } else 
      {
         // horizontal resize kernel: dual input ==> output => vertical resize kernel => output => filtering => dual output buffer   
         // 2*vScale*dstChunkHeight*srcChunkWidth  +  vscale*dstChunkHeight*dstChunkWidth + dstChunkHeight*dstChunkWidth + 2*dstChunkHeight*dstChunkWidth
         dstChunkHeight = (lProcess->RetCmemHeapSizeInBytes() - 320) / ((int)((2 * vScale + 1) * srcChunkWidth) + dstChunkWidth*2 + 3*(dstChunkWidth+2));
      }

      dstChunkHeight &= ~0x1;
      dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
      dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;
      
      srcChunkHeight = ceil(dstChunkHeight * vScale);
      srcChunkHeight += vrsz.taps;
      srcChunkHeight = (srcChunkHeight + 0x1) & ~0x1;
           
      tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);

      int memSize;
      // input and output (dual buffer)
      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2)*sizeof(uint16_t);
      // intermediate buffer between first and second resize kernel
      if (vrsz_first) {
          memSize += srcChunkWidth * dstChunkHeight*sizeof(uint16_t);
      } else {
          memSize += dstChunkWidth * srcChunkHeight*sizeof(uint16_t);
      }
      // intermediate buffer between 2nd resize kernel and filtering kernel  	  
      memSize += (dstChunkWidth+2)*sizeof(uint16_t)*(4*dstChunkHeight + 2);

   
      while (((tileSizeInBytes >= (1<<16) || srcChunkHeight > 64) && dstChunkHeight > 1) || memSize > availCmem) 
      {
         dstChunkHeight -= 2;

         dstChunkHeight &= ~0x1;
         dstChunkHeight = dstChunkHeight > 0 ? dstChunkHeight : 1;
         dstChunkHeight = dstChunkHeight < 32 ? dstChunkHeight : 32;

         srcChunkHeight = ceil(dstChunkHeight * vScale);
         srcChunkHeight += vrsz.taps;
         srcChunkHeight = (srcChunkHeight + 0x1) & ~0x1;

         tileSizeInBytes = srcChunkWidth * srcChunkHeight * tileWidthInChunks*sizeof(uint16_t);
   
         memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2)*sizeof(uint16_t);
         if (vrsz_first) {
             memSize += srcChunkWidth * dstChunkHeight*sizeof(uint16_t);
         } else {
             memSize += dstChunkWidth * srcChunkHeight*sizeof(uint16_t);
         }
         memSize += (dstChunkWidth+2)*sizeof(uint16_t)*(4*dstChunkHeight + 2);
      }

      memSize = (srcChunkWidth * srcChunkHeight * 2 + dstChunkWidth * dstChunkHeight * 2)*sizeof(uint16_t);
      if (vrsz_first) {
         memSize += srcChunkWidth * dstChunkHeight*sizeof(uint16_t);
      } else {
         memSize += dstChunkWidth * srcChunkHeight*sizeof(uint16_t);
      }
      memSize += (dstChunkWidth+2)*sizeof(uint16_t)*(4*dstChunkHeight + 2);
      //printf(" tile_size_in_bytes %d  mem_size %d, avail_cmem %d", tile_size_in_bytes, mem_size, avail_cmem);
   
      if (tileSizeInBytes >= (1<<16) || memSize > availCmem) 
      {
         lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         weKeepGoing = false;
      }

      if (weKeepGoing == true)
      {
         //printf("%d %d,%d,%d,%d,", tileWidthInChunks, srcChunkWidth, srcChunkHeight, dstChunkWidth, dstChunkHeight);

         /*
         ** Now we override the scenario table with the assumption index 0 = SRC, 1 = intermediate, 2 = DST
         */
         lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = srcChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = srcChunkHeight;

         if (vrsz_first) {
            lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = srcChunkWidth;
            lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dstChunkHeight;
         } else {
            lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dstChunkWidth;
            lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = srcChunkHeight;
         }

         lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = dstChunkHeight;
 
         lProcess->RetSelectedScenario()->pBufferData[3].chunkWidth  = dstChunkWidth;
         lProcess->RetSelectedScenario()->pBufferData[3].chunkHeight = dstChunkHeight;

         /*
         ** Now that we know the chunk sizes for resize we can create the real motion comp offset table
         */
         moH = (aDst.rows + dstChunkHeight - 1) / dstChunkHeight;

         if (weKeepGoing == true)
         {

            moOffs = vsdk::SUMat( moH, tileWidthInChunks, VSDK_CV_32SC1 );

            initMoOffsets16Umat(aSrc, moOffs, dstChunkWidth, dstChunkHeight, &hrsz, &vrsz);

            vsdk::Rect lRect(0,
                             0,
                             dstChunkWidth * tileWidthInChunks,
                             aDst.rows);
            vsdk::SUMat dst_t(aDst, lRect);


            if (aDst.empty() == true) {
               lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
               weKeepGoing = false;
            }

            if (weKeepGoing == true)
            {
               {
                   vsdk::SMat matTemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
               } 

               /*
               ** Finally connect the ports, set the src/dst ports to calculated sizes
               */
               if( lProcess->ConnectIndirectInput("SRC", aSrc, moOffs)
                   || lProcess->ConnectIO("DST", dst_t)
                   || lProcess->ConnectIO("PARAMS", mParams) ) 
               {
                  lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
               }
               else
               {    
                  lProcess->FlushAndInvalidateAllIO();
                  int status = 0;  
                  status  = lProcess->Start();
                  status |= lProcess->Wait();

                  if(status)    
                  {
                     lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
                  }
                  else
                  {
                     /*
                     ** Set scenario back to original values for future resize calls
                     */
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = oldChunkHeight;
                     lProcess->RetSelectedScenario()->pBufferData[3].chunkWidth  = oldChunkWidth;
                     lProcess->RetSelectedScenario()->pBufferData[3].chunkHeight = oldChunkHeight;
                  }
               }
            }
         }
      }
   }

//exit:
   if (lProcess != 0) delete lProcess;
   
   return lRetVal;
}

}

int32_t QueryApuArraySize(int32_t& aArrayWidth)
{
   int32_t lRetVal = 0;
   int32_t lHwVersion = global_hal_RetHwVersion(0);
   switch(lHwVersion)
   {
   case 0x321:
      aArrayWidth = 32;
      cmem_if_hal_set_apu_cfg(0, CFG_321__APU_0_CU_0_31_SMEM_0_1);
      break;
   case 0x642:

      cmem_if_hal_set_apu_cfg(0, CFG_642__APU_0_CU_0_63_SMEM_0_3);
      aArrayWidth = 64;
      break;
   default:
          printf("Start_Resize() -> unknown APU configuration detected (%X)\n", lHwVersion);
      lRetVal = 1;
   }

   return lRetVal;
}



APEXCV_LIB_RESULT testResizeParams(int aSrcWidth,
                                   int aSrcHeight,
                                   int aDstWidth,
                                   int aDstHeight)
{
   int ret = 0;

   // Reject invalid configurations
   // NB: integer scale factors only
   // max 1024 width on input, output
   // max vertical scale factor is 32, for now.
   // DMA considerations

   float vScale = (float)aDstHeight / (float)aSrcHeight;
   float hScale = (float)aDstWidth  / (float)aSrcWidth;

   if (hScale <= 0.25) 
   {
       //printf("Reference resize will not match for scale factors <= 0.25. See CLOUD-495.\n");
   }
    
   // Test for valid vertical scale factor (limited to 32 for upscale)
   if (vScale > 32) 
   {
      printf("Invalid vertical scale factor - upscaling currently limited to 32x \n");
      ret++;
   }
   if (aDstWidth>1024 || aSrcWidth>1024) 
   {
      printf("Invalid width - width limited to 1024 for both in put and output \n");;
      ret++;
   }
   int32_t numCU = 1;
   if (QueryApuArraySize(numCU)) 
   {
      ret++;
   }

   if( ret )
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;   
   }
   return APEXCV_SUCCESS;
}

