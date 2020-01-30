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
#include <apexcv_pro_resize.h>
#include <RESIZE_H_V.hpp>
#include <RESIZE16_H_V.hpp>
#include <RESIZE_V_H.hpp>
#include <RESIZE16_V_H.hpp>
#include <RESIZE_V.hpp>
#include <RESIZE16_V.hpp>
#include <RESIZE_H.hpp>
#include <RESIZE16_H.hpp>
#include <cmath>
#include <acf_common.h>
#include "resize_definitions.h"
#include <linked_list_utilities.h>
#include <icp_datatype.h>

#define LLE_SIZE_1D_MEM_MEM       8

using namespace std;

enum
{
    UNINIT_STATE      = 0,
    COPY_PROCESSING   = 1, 
    HRSZ_PROCESSING   = 2,
    VRSZ_PROCESSING   = 3,
    RRSZ_PROCESSING   = 4,
    HRSZ16_PROCESSING = 5,
    VRSZ16_PROCESSING = 6,
    RRSZ16_PROCESSING = 7,
};

// Prototype

int32_t QueryApuArraySize(int32_t& aArrayWidth);
APEXCV_LIB_RESULT TestResizeParams(int aSrcWidth, int aSrcHeight, int aDstWidth, int aDstHeight);

// See APU_resize.hpp - SetProcessIdentifier() or resize_apu_process_desc.hpp - Initialize() for htis string value.
std::string resizeProcessIdentifier = "APU_resize";

///// C++ interface //////////////////////////////////////////////////
namespace apexcv{

Resize::Resize()
     :mInitState(0),
      mApuConfig(ACF_APU_CFG__DEFAULT),
      mApexId(0),
      mWeSelectedApuConfiguration(false),
      mDataType(1)
{
    mProcessType = 0;
    mSrcWidthUMat  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
    mDstWidthUMat  = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
    mSrcHeightUMat = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
    mDstHeightUMat = vsdk::SUMat( 1, 1, VSDK_CV_16SC1 );
    mSrcWidth = mDstWidth = mSrcHeight = mDstHeight = 0;
    mpProcess = 0;
    mInit = 0;
    mIsResizeInitialized = false;
    mIsResizeHInitialized = false;
    mIsResizeVInitialized = false;
    mIsCopyInitialized = false;
    mIsResize16Initialized = false;
    mIsResize16HInitialized = false;
    mIsResize16VInitialized = false;
    mIsCopy16Initialized = false;
    mpProcessVH = nullptr;
    mpProcessHV = nullptr;
    mpProcessV = nullptr;
    mpProcessH = nullptr;
    mpProcess16VH = nullptr;
    mpProcess16HV = nullptr;
    mpProcess16H = nullptr;
    mpProcess16V = nullptr;
    mpProcess = nullptr;
}

Resize::~Resize()
{
    if(mpProcessVH)
    {
        delete mpProcessVH;
        mpProcessVH = nullptr;
    }
    if(mpProcessHV)
    {
        delete mpProcessHV;
        mpProcessHV = nullptr;
    }
    if(mpProcessV)
    {
        delete mpProcessV;
        mpProcessV = nullptr;
    }
    if(mpProcessH)
    {
        delete mpProcessH;
        mpProcessH = nullptr;
    }
    if(mpProcess16VH)
    {
        delete mpProcess16VH;
        mpProcess16VH = nullptr;
    }
    if(mpProcess16HV)
    {
        delete mpProcess16HV;
        mpProcess16HV = nullptr;
    }
    if(mpProcess16V)
    {
        delete mpProcess16V;
        mpProcess16V = nullptr;
    }
    if(mpProcess16H)
    {
        delete mpProcess16H;
        mpProcess16H = nullptr;
    }
}

APEXCV_LIB_RESULT Resize::Initialize(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                                     vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    // Test resize parameters to ensure they are supported
    lRetVal = TestResizeParams(aSrcImage.cols,
                                  aSrcImage.rows,
                                  aDestImage.cols,
                                  aDestImage.rows);
    if ((APEXCV_SUCCESS == lRetVal) && (0 == mInit))
    {
        mpProcessH = new RESIZE_H();
        lRetVal = mpProcessH->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcessH) 
            {
                delete mpProcessH;
                mpProcessH = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcessV = new RESIZE_V();
        lRetVal = mpProcessV->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcessV) 
            {
                delete mpProcessV;
                mpProcessV = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcessVH = new RESIZE_V_H();
        lRetVal = mpProcessVH->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcessVH) 
            {
                delete mpProcessVH;
                mpProcessVH = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcessHV = new RESIZE_H_V();
        lRetVal = mpProcessHV->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcessHV) 
            {
                delete mpProcessHV;
                mpProcessHV = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcess16H = new RESIZE16_H();
        lRetVal = mpProcess16H->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcess16H) 
            {
                delete mpProcess16H;
                mpProcess16H = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcess16V = new RESIZE16_V();
        lRetVal = mpProcess16V->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcess16V) 
            {
                delete mpProcess16V;
                mpProcess16V = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcess16VH = new RESIZE16_V_H();
        lRetVal = mpProcess16VH->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcess16VH) 
            {
                delete mpProcess16VH;
                mpProcess16VH = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }
        mpProcess16HV = new RESIZE16_H_V();
        lRetVal = mpProcess16HV->Initialize();
        if( APEXCV_SUCCESS != lRetVal )
        {
            if (mpProcess16HV) 
            {
                delete mpProcess16HV;
                mpProcess16HV = nullptr;
            }
            return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
        }

        if (aSrcImage.type() == VSDK_CV_8UC1 && aDestImage.type() == VSDK_CV_8UC1 ) 
        {
            if ( aSrcImage.cols == aDestImage.cols && aSrcImage.rows == aDestImage.rows)
            {
                lRetVal = copy_init(aSrcImage, aDestImage);
            }
            else if (aSrcImage.cols == aDestImage.cols)
            {
                lRetVal = acf_vrsz_init(aSrcImage, aDestImage);
            }
            else if (aSrcImage.rows == aDestImage.rows)
            {
                lRetVal = acf_hrsz_init(aSrcImage, aDestImage);
            }
            else
            {
                lRetVal = acf_resize_init(aSrcImage, aDestImage);
            }
        }
        else if (aSrcImage.type() == VSDK_CV_16U && aDestImage.type() == VSDK_CV_16U )        
        {
            if ( aSrcImage.cols == aDestImage.cols && aSrcImage.rows == aDestImage.rows)
            {
                lRetVal = copy16_init(aSrcImage, aDestImage);
            }
            else if (aSrcImage.cols == aDestImage.cols)
            {
                lRetVal = acf_vrsz16_init(aSrcImage, aDestImage);
            }
            else if (aSrcImage.rows == aDestImage.rows)
            {
                lRetVal = acf_hrsz16_init(aSrcImage, aDestImage);
            }
            else
            {
                lRetVal = acf_resize16_init(aSrcImage, aDestImage);
            }
        }
        else
        {
            lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
        }
        if(lRetVal == APEXCV_SUCCESS)
        {
          mInit = 1;
        }
    }
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    return lRetVal;
}

APEXCV_LIB_RESULT Resize::ReconnectIO(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                                      vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    if(0 == mInit)
    {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    else
    {
      // Test resize parameters to ensure they are supported
      lRetVal = TestResizeParams(aSrcImage.cols,
                                    aSrcImage.rows,
                                    aDestImage.cols,
                                    aDestImage.rows);
      if ( APEXCV_SUCCESS == lRetVal)
      {
          if (aSrcImage.type() == VSDK_CV_8UC1 && aDestImage.type() == VSDK_CV_8UC1 ) 
          {
              if ( aSrcImage.cols == aDestImage.cols && aSrcImage.rows == aDestImage.rows)
              {
                  lRetVal = copy(aSrcImage, aDestImage);
              }
              else if (aSrcImage.cols == aDestImage.cols)
              {
                  lRetVal = acf_vrsz_conf(aSrcImage, aDestImage);
              }
              else if (aSrcImage.rows == aDestImage.rows)
              {
                  lRetVal = acf_hrsz_conf(aSrcImage, aDestImage);
              }
              else
              {
                  lRetVal = acf_resize_conf(aSrcImage, aDestImage);
              }
          }
          else if (aSrcImage.type() == VSDK_CV_16U && aDestImage.type() == VSDK_CV_16U )        
          {
              if ( aSrcImage.cols == aDestImage.cols && aSrcImage.rows == aDestImage.rows)
              {
                  lRetVal = copy16(aSrcImage, aDestImage);
              }
              else if (aSrcImage.cols == aDestImage.cols)
              {
                  lRetVal = acf_vrsz16_conf(aSrcImage, aDestImage);
              }
              else if (aSrcImage.rows == aDestImage.rows)
              {
                  lRetVal = acf_hrsz16_conf(aSrcImage, aDestImage);
              }
              else
              {
                  lRetVal = acf_resize16_conf(aSrcImage, aDestImage);
              }
          }
          else
          {
              lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
          }
      }
      else
      {
          lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    return lRetVal;
}

void init_moOffsUMatets_umat(int32_t aSrc_span, vsdk::SUMat &aMoOffsUMat, int32_t aDw, int32_t aDh, resize_descriptor* aHrsz, resize_descriptor* aVrsz)
{
   const int32_t hdscl = HRSZ_DECIMAL_SCL;
   const int32_t vdscl = VRSZ_DECIMAL_SCL;

   uint32_t mo_width  = aMoOffsUMat.cols;
   uint32_t mo_height = aMoOffsUMat.rows;

   vsdk::SMat mat_temp = aMoOffsUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

   int32_t* p = (int32_t *)mat_temp.data;

   if (aVrsz == NULL)
   {
      for (uint32_t y = 0; y < mo_height; ++y)
      {
         int32_t hsrc_offs = aHrsz->src_offs;
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = aSrc_span * y * aDh + (hsrc_offs >> hdscl);
            hsrc_offs += aHrsz->scl_fact * aDw;
         }
      }
   }
   else if (aHrsz == NULL)
   {
      int32_t vsrc_offs = aVrsz->src_offs;

      for (uint32_t y = 0; y < mo_height; ++y)
      {
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = aSrc_span * (vsrc_offs >> vdscl) + x *aDw;
         }

         vsrc_offs += aVrsz->scl_fact * aDh;
      }
   }
   else
   {
      int32_t vsrc_offs = aVrsz->src_offs;

      for (uint32_t y = 0; y < mo_height; ++y)
      {
         int32_t hsrc_offs = aHrsz->src_offs;
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = aSrc_span * (vsrc_offs >> vdscl) + (hsrc_offs >> hdscl);
            hsrc_offs += aHrsz->scl_fact * aDw;
         }

         vsrc_offs += aVrsz->scl_fact * aDh;
      }
   }
}

void init_moOffsUMatets16_umat(vsdk::SUMat &aSrc, vsdk::SUMat &aMoOffsUMat, int32_t aDw, int32_t aDh, resize_descriptor* aHrsz, resize_descriptor* aVrsz)
{
   const int32_t hdscl = HRSZ_DECIMAL_SCL;
   const int32_t vdscl = VRSZ_DECIMAL_SCL;

   uint32_t mo_width  = aMoOffsUMat.cols;
   uint32_t mo_height = aMoOffsUMat.rows;

   int src_span = aSrc.step[0]; 
 

   vsdk::SMat mat_temp = aMoOffsUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   int32_t* p = (int32_t *)mat_temp.data; 

   if (aVrsz == NULL)
   {
      for (uint32_t y = 0; y < mo_height; ++y)
      {
         int32_t hsrc_offs = aHrsz->src_offs;
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = src_span * y * aDh + (hsrc_offs >> hdscl)*sizeof(uint16_t);
            hsrc_offs += aHrsz->scl_fact * aDw;
         }
      }
   }
   else if (aHrsz == NULL)
   {
      int32_t vsrc_offs = aVrsz->src_offs;

      for (uint32_t y = 0; y < mo_height; ++y)
      {
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = src_span * (vsrc_offs >> vdscl) + x * aDw*sizeof(uint16_t);
         }

         vsrc_offs += aVrsz->scl_fact * aDh;
      }
   }
   else
   {
      int32_t vsrc_offs = aVrsz->src_offs;

      for (uint32_t y = 0; y < mo_height; ++y)
      {
         int32_t hsrc_offs = aHrsz->src_offs;
         for (uint32_t x = 0; x < mo_width; ++x)
         {
            p[y * mo_width + x] = src_span * (vsrc_offs >> vdscl) + (hsrc_offs >> hdscl)*sizeof(uint16_t);
            hsrc_offs += aHrsz->scl_fact * aDw;
         }

         vsrc_offs += aVrsz->scl_fact * aDh;
      }
   }
}

int get_polyphase_params(int16_t aSsiz, int16_t aDsiz, int32_t aDscl, resize_descriptor* aRsz_desc)
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

APEXCV_LIB_RESULT Resize::Process()
{
    APEXCV_LIB_RESULT intResult = APEXCV_SUCCESS;

    if(mProcessType == HRSZ_PROCESSING)
    {
        intResult =  acf_hrsz();
    }
    else if(mProcessType == VRSZ_PROCESSING)
    {
        intResult =  acf_vrsz();
    }
    else if(mProcessType == RRSZ_PROCESSING)
    {
        intResult =  acf_resize();
    }
    else if(mProcessType == HRSZ16_PROCESSING)
    {
        intResult =  acf_hrsz16();
    }
    else if(mProcessType == VRSZ16_PROCESSING)
    {
        intResult =  acf_vrsz16();
    }
    else if(mProcessType == RRSZ16_PROCESSING)
    {
        intResult =  acf_resize16();
    }
    else if (mProcessType == COPY_PROCESSING )
    {
        intResult = APEXCV_SUCCESS;
    }
    else
    {
        intResult = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    return intResult;
}

APEXCV_LIB_RESULT Resize::SelectApuConfiguration(ACF_APU_CFG aApuConfig,
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

APEXCV_LIB_RESULT Resize::copy_init(vsdk::SUMat& aSrc,
                                 vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    copy(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::copy(vsdk::SUMat& aSrc,
                               vsdk::SUMat& aDst)
{
   mProcessType = COPY_PROCESSING;
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   vsdk::SMat dstMat = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
     
   uint8_t *dst8 = (uint8_t *)dstMat.data;
   uint8_t *src8 = (uint8_t *)srcMat.data;

   int mSrcWidthUMat  = aSrc.cols;
   int mSrcHeightUMat = aSrc.rows;

   int instride = aSrc.step[0];
   int outstride = aDst.step[0];

   for(int i = 0; i < mSrcHeightUMat; i++)
   {
      memcpy(dst8, src8, mSrcWidthUMat*sizeof(uint8_t));
      dst8 += outstride;
      src8 += instride;
   }

   return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_hrsz_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_hrsz_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_hrsz_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;

    if ( aSrc.cols != mSrcWidth || aDst.cols != mDstWidth || mProcessType != HRSZ_PROCESSING || !mpProcess)
    {
        mSrcWidth = aSrc.cols;
        mDstWidth = aDst.cols;
 
        mpProcess = mpProcessH;
        mProcessType = HRSZ_PROCESSING;
        resize_descriptor hrsz;
        get_polyphase_params(aSrc.cols,
                         aDst.cols,
                         HRSZ_DECIMAL_SCL, &hrsz);

        if(mWeSelectedApuConfiguration == true)
        {
            if (mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            {
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    mpProcessH = nullptr;
                }
                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;  
            }  
        }

        int32_t numCU = mpProcess->RetCuArrayWidth();

        mSrcWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;
        mDstWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;

        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1; 
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
   
        src_chunk_width = (dst_chunk_width * aSrc.cols + aDst.cols - 1) / aDst.cols;
        src_chunk_width += hrsz.taps;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
        
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
   
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 );    

        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks,
                             aDst.rows);

        vsdk::SUMat dst_temp(aDst, mRect_t); 
   
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
            || mpProcess->ConnectIO("DST", dst_temp)
            || mpProcess->ConnectIO("SRC_WIDTH", mSrcWidthUMat)
            || mpProcess->ConnectIO("DST_WIDTH", mDstWidthUMat) )
        {
           lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO; 
           weKeepGoing = false;
        }
        else
        {
          if(false == mIsResizeHInitialized)
          {            
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("HRSZ", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResizeHInitialized = true;
          }
        }
        if (weKeepGoing == true)
        {
            if (mpProcess->SelectScenario("", 0, 0) != 0)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
        }
        if (weKeepGoing == true)
        {
     
            int avail_cmem;
          
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 11 * sizeof(int32_t) - 2 * sizeof(int32_t);
     
            dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20*src_chunk_width) / (2 * (src_chunk_width + dst_chunk_width));
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
            src_chunk_height = dst_chunk_height;
     
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            while ((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1)
            {
                dst_chunk_height -= 2;
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
                src_chunk_height = dst_chunk_height;
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            }
     
            int mem_size;
            mem_size = src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2;
     
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            } 
     
            if (weKeepGoing == true)
            {
                //printf("%d,%d,%d,%d,", src_chunk_width, src_chunk_height, dst_chunk_width, dst_chunk_height);
     
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
     
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
     
                if (weKeepGoing == true)
                {
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );
                    init_moOffsUMatets_umat(aSrc.step[0], moOffsUMat, dst_chunk_width, dst_chunk_height, &hrsz, NULL);
     
                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);

                    int padDstWidth = aDst.step[0];
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                        weKeepGoing = false;
                    }
                }     
            }
        }
    }

    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcessH = nullptr;
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED); 
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        }      
        
        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);

        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat)
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcessH = nullptr;
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;   
        }  

    }

    return lRetVal;
}



APEXCV_LIB_RESULT Resize::acf_hrsz()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    if (mpProcess)
    {
        mpProcess->FlushAndInvalidateAllIO();
        int status = 0;  
        status = mpProcess->Start();
        status |= mpProcess->Wait();
                  
        if( status )
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC; 
        }
    }
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_vrsz_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_vrsz_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_vrsz_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;
    float v_scale;
  
    if ( aSrc.rows != mSrcHeight || aDst.rows != mDstHeight || mProcessType != VRSZ_PROCESSING || !mpProcess)
    {
        mSrcHeight = aSrc.rows;
        mDstHeight = aDst.rows;

        mpProcess = mpProcessV;
        mProcessType = VRSZ_PROCESSING;
        resize_descriptor vrsz;
        get_polyphase_params(aSrc.rows,
                          aDst.rows,
                          VRSZ_DECIMAL_SCL, &vrsz); 
  
        if(mWeSelectedApuConfiguration == true)
        {
            if( mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            {
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    mpProcessV = nullptr;
                }
                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            }   
        }
     
        int32_t numCU = mpProcess->RetCuArrayWidth();
     
        mSrcHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
        mDstHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;
  
        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1; 
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
     
        src_chunk_width = dst_chunk_width;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
     
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
    
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 );    
  
        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks,
                               aDst.rows);
  
        vsdk::SUMat dst_temp(aDst, mRect_t);
   
        if ( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
              || mpProcess->ConnectIO("DST", dst_temp)
              || mpProcess->ConnectIO("SRC_HEIGHT", mSrcHeightUMat)
              || mpProcess->ConnectIO("DST_HEIGHT", mDstHeightUMat) )
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            weKeepGoing = false;
        }
        else
        {
          if(false == mIsResizeVInitialized)
          {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("VRSZ", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResizeVInitialized = true;
          }          
        }
        if (weKeepGoing == true)
        {
            if (mpProcess->SelectScenario("", 0, 0) != 0)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
        }
     
        if (weKeepGoing == true)
        {
  
            int avail_cmem;
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 11 * sizeof(int32_t) - 2 * sizeof(int32_t);
  
            v_scale = (float)aSrc.rows / (float)aDst.rows;
            dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width) / (float)(2 * v_scale * (float)src_chunk_width + 2 * (float)dst_chunk_width);
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
  
            src_chunk_height = ceil(dst_chunk_height * v_scale);
            src_chunk_height += vrsz.taps;
            src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
  
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            while ((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1)
            {
                dst_chunk_height -= 2;
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
  
                src_chunk_height = ceil(dst_chunk_height * v_scale);
                src_chunk_height += vrsz.taps;
                src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
  
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            }
  
            int mem_size;
            mem_size = src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2;
  
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG; 
                weKeepGoing = false;
            }
  
            if (weKeepGoing == true)
            {
  
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
  
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
  
                if (weKeepGoing == true)
                {
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );
                    init_moOffsUMatets_umat(aSrc.step[0], moOffsUMat, dst_chunk_width, dst_chunk_height, NULL, &vrsz);
  
                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);

                    int padDstWidth = aDst.step[0];
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                        weKeepGoing = false;
                    }
                }
            }
        }
    }

    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcessV = nullptr;
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED); 
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        }

        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);

        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat) 
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcessV = nullptr;
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
    }
    return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_vrsz()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    if (mpProcess)
    {
        mpProcess->FlushAndInvalidateAllIO();
        int status = 0;   
        status = mpProcess->Start();
        status |= mpProcess->Wait();
                 
        if(status)
        {  
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;  
        }
    } 
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    } 
  
    return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_resize_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_resize_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_resize_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;
    float v_scale;
    bool vrsz_first;

    if(aSrc.rows != mSrcHeight || aDst.rows != mDstHeight || mProcessType != RRSZ_PROCESSING || !mpProcess
       || aSrc.cols != mSrcWidth ||aDst.cols != mDstWidth  )
    {
        mSrcWidth = aSrc.cols;
        mDstWidth = aDst.cols; 
        mSrcHeight = aSrc.rows;
        mDstHeight = aDst.rows;
        int src_size = aSrc.cols * aSrc.rows;
        if ((0.75 * (float)src_size + (float)(aSrc.cols * aDst.rows)) < ((float)src_size + (float)aDst.cols * (float)aSrc.rows * 0.75))
        {
            vrsz_first = true;
        }
        else
        {
            vrsz_first = false;
        }
        /*
        ** HRSZ is slower than VRSZ, so pick the order based on performance ratio
        */
        if(vrsz_first)
        {
            mpProcess = mpProcessVH;
        }
        else
        {
            mpProcess = mpProcessHV;
        }
        mProcessType = RRSZ_PROCESSING;
        resize_descriptor hrsz;
        get_polyphase_params(aSrc.cols, aDst.cols, HRSZ_DECIMAL_SCL, &hrsz);
    
        resize_descriptor vrsz;
        get_polyphase_params(aSrc.rows, aDst.rows, VRSZ_DECIMAL_SCL, &vrsz);
 
        if(mWeSelectedApuConfiguration == true)
        {
            if( mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            {  
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    if(vrsz_first)
                    {
                      mpProcessVH = nullptr;
                    }
                    else
                    {
                      mpProcessHV = nullptr;
                    }
                }
                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            } 
        }
        int32_t numCU = mpProcess->RetCuArrayWidth();
        
        /*
        ** setup and connect ports to be able to call select scenario
        */
    
        mSrcWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;
        mDstWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;
        mSrcHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
        mDstHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;
 
        /* 
        ** Need to calculate chunk width right now to ensure the DST port
        ** is a multiple. Otherwise calling select scenario with the raw
        ** DST width may fail
        */
        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1; 
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
    
        src_chunk_width = (dst_chunk_width * aSrc.cols + aDst.cols - 1) / aDst.cols;
        src_chunk_width += hrsz.taps;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
    
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
    
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 );    
 
        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks,
                              aDst.rows);
 
        vsdk::SUMat dst_temp(aDst, mRect_t);
  
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
            || mpProcess->ConnectIO("DST", dst_temp)
            || mpProcess->ConnectIO("SRC_WIDTH", mSrcWidthUMat)
            || mpProcess->ConnectIO("DST_WIDTH", mDstWidthUMat)
            || mpProcess->ConnectIO("SRC_HEIGHT", mSrcHeightUMat)
            || mpProcess->ConnectIO("DST_HEIGHT", mDstHeightUMat) )
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            weKeepGoing = false;  
        }
        else
        {
          if(false == mIsResizeInitialized)
          {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("RESIZE", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResizeInitialized = true;
          }
        }
        if( weKeepGoing == true )
        {
            if (mpProcess->SelectScenario("", 0, 0) != 0)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
                weKeepGoing = false;
            }
        }
 
        if (weKeepGoing == true)
        {
            /*
            ** Now that a scenario has been selected we can modify/hack it however we want
            ** since we know the available CMEM. The chunk heights are determined by creating a
            ** system of linear equations/inequalities that includes: the relationship between
            ** every non-fixed port size and an inequality between the available CMEM and the sum
            ** of every port buffer in the graph. Simplifying the system of equations leaves one unknown
            ** in the inequality which is the dst chunk height. Solving for this inequality gives us
            ** the dst chunk height which allows us to calculate the src chunk height as well.
            */
 
            int avail_cmem;
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 22 * sizeof(int32_t) - 4 * sizeof(int32_t);
 
            v_scale = (float)aSrc.rows / (float)aDst.rows;
 
            if (vrsz_first)
            {
                dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width) / (float)((2 * v_scale + 1) * (float)src_chunk_width + 2 * (float)dst_chunk_width);
            }
            else
            {
                dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width - 10 * dst_chunk_width) / (float)(2 * v_scale * (float)src_chunk_width + (2 + v_scale) * (float)dst_chunk_width);
            }
 
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
 
            src_chunk_height = ceil(dst_chunk_height * v_scale);
            src_chunk_height += vrsz.taps;
            src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
 
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            while ((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1)
            {
                dst_chunk_height -= 2;
 
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
 
                src_chunk_height = ceil(dst_chunk_height * v_scale);
                src_chunk_height += vrsz.taps;
                src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
 
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks;
            }
 
            int mem_size;
            mem_size = src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2;
            if (vrsz_first)
            {
                mem_size += src_chunk_width * dst_chunk_height;
            }
            else
            {
                mem_size += dst_chunk_width * src_chunk_height;
            }
 
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem)
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG; 
                weKeepGoing = false;
            }
    
            if (weKeepGoing == true)
            {
                /*
                ** Now we override the scenario table with the assumption index 0 = SRC, 1 = intermediate, 2 = DST
                */
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
    
                if (vrsz_first)
                {
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = src_chunk_width;
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
                }
                else
                {
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = src_chunk_height;
                }
       
                mpProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = dst_chunk_height;
    
                /*
                ** Now that we know the chunk sizes for resize we can create the real motion comp offset table
                */
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
 
                if (weKeepGoing == true)
                {
 
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );
                    init_moOffsUMatets_umat(aSrc.step[0], moOffsUMat, dst_chunk_width, dst_chunk_height, &hrsz, &vrsz);
 
                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);

                    int padDstWidth = aDst.step[0];
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                        weKeepGoing = false;
                    }
                }
            }
        }
    }

    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
          delete mpProcess;
          mpProcess = nullptr;
          if(vrsz_first)
          {
            mpProcessVH = nullptr;
          }
          else
          {
            mpProcessHV = nullptr;
          }
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        }
        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);
        /*
        ** Finally connect the ports, set the src/dst ports to calculated sizes
        */
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat)
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            if(vrsz_first)
            {
              mpProcessVH = nullptr;
            }
            else
            {
              mpProcessHV = nullptr;
            }
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }

    }
    return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_resize()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
 
    if (mpProcess)
    {
        mpProcess->FlushAndInvalidateAllIO();
        int status = 0;  
        status  = mpProcess->Start();
        status |= mpProcess->Wait();

        if(status)
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        }
    } 
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
   
    return lRetVal;
}

APEXCV_LIB_RESULT Resize::copy16_init(vsdk::SUMat& aSrc,
                                 vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = copy16(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::copy16(vsdk::SUMat& aSrc,
                                 vsdk::SUMat& aDst)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   mProcessType = COPY_PROCESSING;
   vsdk::SMat dstMat = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
   uint16_t *dst16 = (uint16_t *)dstMat.data;
   uint16_t *src16 = (uint16_t *)srcMat.data;

   int mSrcWidthUMat  = aSrc.cols;
   int mSrcHeightUMat = aSrc.rows;

   int instride = aSrc.step[0]/2;
   int outstride = aDst.step[0]/2;

   for(int i = 0; i < mSrcHeightUMat; i++)
   {
      memcpy(dst16, src16, mSrcWidthUMat*sizeof(uint16_t));
      dst16 += outstride;
      src16 += instride;
   }

   return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_hrsz16_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_hrsz16_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_hrsz16_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;

    if ( aSrc.cols != mSrcWidth || aDst.cols != mDstWidth || mProcessType != HRSZ_PROCESSING || !mpProcess)
    {
        mSrcWidth = aSrc.cols;
        mDstWidth = aDst.cols;
        mpProcess = mpProcess16H;
        mProcessType = HRSZ16_PROCESSING;

        resize_descriptor hrsz;
        get_polyphase_params(aSrc.cols,
                                aDst.cols,
                                HRSZ_DECIMAL_SCL, &hrsz);
           
        if( mWeSelectedApuConfiguration == true )
        { 
            if(mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            {
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    mpProcess16H = nullptr;
                }
                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            }   
        }
        
        int32_t numCU = mpProcess->RetCuArrayWidth();
        
        mSrcWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;
        
        mDstWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;
        
        //printf(" hrsz16 %d %d \n",  aSrc.cols, aDst.cols);
        
        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1;
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
        
        src_chunk_width = (dst_chunk_width * aSrc.cols + aDst.cols - 1) / aDst.cols;
        src_chunk_width += hrsz.taps;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
        
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
        
        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);
        vsdk::SUMat dst_temp(aDst, mRect_t);
        
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 );

        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
               || mpProcess->ConnectIO("DST", dst_temp)
               || mpProcess->ConnectIO("SRC_WIDTH", mSrcWidthUMat)
               || mpProcess->ConnectIO("DST_WIDTH", mDstWidthUMat) )
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            weKeepGoing = false; 
        }
        else
        {
          if(false == mIsResize16HInitialized)
          {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("HRSZ16", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResize16HInitialized = true;
          }
        }
        if (weKeepGoing == true)
        {
            if (mpProcess->SelectScenario("", 0, 0) != 0) 
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
        }
        
        if (weKeepGoing == true)
        {
            int avail_cmem;
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 11 * sizeof(int32_t) - 2 * sizeof(int32_t);
        
            dst_chunk_height = mpProcess->RetCmemHeapSizeInBytes() / (2 * (src_chunk_width + dst_chunk_width));
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
        
            src_chunk_height = dst_chunk_height;
        
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
        
            int mem_size;
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
        
            while  ( ((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1) || mem_size > avail_cmem )
            {
                dst_chunk_height -= 2;
        
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
        
                src_chunk_height = dst_chunk_height;
        
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
        
                mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
            }
        
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
           
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem) 
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
        
            if (weKeepGoing == true)
            {
                //printf("%d,%d,%d,%d,", src_chunk_width, src_chunk_height, dst_chunk_width, dst_chunk_height);
        
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
        
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
        
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
        
                if (weKeepGoing == true)
                {
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );
                    init_moOffsUMatets16_umat(aSrc, moOffsUMat, dst_chunk_width, dst_chunk_height, &hrsz, NULL);

                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);
        
                    int padDstWidth = aDst.step[0]/sizeof(int16_t);
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                       lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                       weKeepGoing = false;
                    }
                }
            }
        }
    }

    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcess16H = nullptr;
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        }
        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat)
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcess16H = nullptr;
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }

    }
    return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_hrsz16()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    if (mpProcess)
    {
        mpProcess->FlushAndInvalidateAllIO();
        int status = 0; 
        status  = mpProcess->Start();
        status |= mpProcess->Wait();

        if( status)
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        }
    }
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_vrsz16_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_vrsz16_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_vrsz16_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;
    float v_scale;
    
    if ( aSrc.rows != mSrcHeight || aDst.rows != mDstHeight || mProcessType != VRSZ_PROCESSING || !mpProcess )
    {
        mSrcHeight = aSrc.rows;
        mDstHeight = aDst.rows; 
        mpProcess = mpProcess16V;
        mProcessType = VRSZ16_PROCESSING;

        resize_descriptor vrsz;
        get_polyphase_params(aSrc.rows,
                             aDst.rows,
                             VRSZ_DECIMAL_SCL, &vrsz);
     
        if(mWeSelectedApuConfiguration == true)
        {
            if( mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            {
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    mpProcess16V = nullptr;
                }

                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;      
            }   
        }
        
        int32_t numCU = mpProcess->RetCuArrayWidth();
     
        mSrcHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
        mDstHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;
     
        //printf(" vrsz16 %d %d \n",  aSrc.rows, aDst.rows);
     
        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1; 
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
        
        src_chunk_width = dst_chunk_width;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
        
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
        
        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);
        vsdk::SUMat dst_temp(aDst, mRect_t);
     
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 ); 
     
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
            || mpProcess->ConnectIO("DST", dst_temp)
            || mpProcess->ConnectIO("SRC_HEIGHT", mSrcHeightUMat)
            || mpProcess->ConnectIO("DST_HEIGHT", mDstHeightUMat))
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            weKeepGoing = false;
        }
        else
        {
          if(false == mIsResize16VInitialized)
          {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("VRSZ16", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResize16VInitialized = true;
          }
        }
        if (mpProcess->SelectScenario("", 0, 0) != 0)
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            weKeepGoing = false;
        }
     
        if (weKeepGoing == true)
        {
            int avail_cmem;
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 11 * sizeof(int32_t) - 2 * sizeof(int32_t);
           
            v_scale = (float)aSrc.rows / (float)aDst.rows;
            dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width) / (float)(2 * v_scale * (float)src_chunk_width + 2 * (float)dst_chunk_width);
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
           
            src_chunk_height = ceil(dst_chunk_height * v_scale);
            src_chunk_height += vrsz.taps;
            src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
     
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
     
            int mem_size;
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
        
            while ( ((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1) || mem_size > avail_cmem )
            {
                dst_chunk_height -= 2;
     
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
     
                src_chunk_height = ceil(dst_chunk_height * v_scale);
                src_chunk_height += vrsz.taps;
                src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
     
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
     
                mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
            }
     
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
     
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem) 
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
        
            if (weKeepGoing == true)
            {
                //printf("%d,%d,%d,%d,", src_chunk_width, src_chunk_height, dst_chunk_width, dst_chunk_height);
     
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
     
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
     
                //printf("  aDst.rows %d mo_h %d  dst_chunk_height %d  tile_width_in_chunks %d dst_chunk_width %d \n ", aDst.rows, mo_h, dst_chunk_height, tile_width_in_chunks, dst_chunk_width);
     
                if (weKeepGoing == true)
                {
     
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );	       
                    init_moOffsUMatets16_umat(aSrc, moOffsUMat, dst_chunk_width, dst_chunk_height, NULL, &vrsz);
     
                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);

                    int padDstWidth = aDst.step[0]/sizeof(int16_t);
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                        weKeepGoing = false;
                    }
     
                    if (aDst.empty() == true)
                    {
                        lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
                        weKeepGoing = false;
                    }
                }
            }
        }
    }

    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcess16V = nullptr;
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        }

        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);

        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat)
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            mpProcess16V = nullptr;
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        } 
    }
    return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_vrsz16()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

    if (mpProcess)
    {
        mpProcess->FlushAndInvalidateAllIO();   
        int status = 0;  
        status  = mpProcess->Start();
        status |= mpProcess->Wait();
   
        if( status)
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        }
    }
    else
    {
        lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
  
    return lRetVal;
}


APEXCV_LIB_RESULT Resize::acf_resize16_init(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
  APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
  if(0 == mInit)
  {
    lRetVal = acf_resize16_conf(aSrc, aDst);
  }
  else
  {
    lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_resize16_conf(vsdk::SUMat& aSrc,
    vsdk::SUMat& aDst)
{
    uint32_t tile_size_in_bytes = 0xffffffff;
    int32_t dst_chunk_width;
    int32_t dst_chunk_height;
    int32_t src_chunk_width;
    int32_t src_chunk_height;
    int32_t tile_width_in_chunks;
    int32_t mo_h;
    float v_scale;
    bool vrsz_first;

    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
    bool weKeepGoing = true;

    if(aSrc.rows != mSrcHeight || aDst.rows != mDstHeight || mProcessType != RRSZ_PROCESSING || !mpProcess
        || aSrc.cols != mSrcWidth || aDst.cols != mDstWidth  )
    {
        mSrcWidth = aSrc.cols;
        mDstWidth = aDst.cols; 
        mSrcHeight = aSrc.rows;
        mDstHeight = aDst.rows;
        int src_size = aSrc.cols * aSrc.rows;
        if ((0.75 * (float)src_size + (float)(aSrc.cols * aDst.rows)) < ((float)src_size + (float)aDst.cols * (float)aSrc.rows * 0.75))
        {
            vrsz_first = true;
        }
        else
        {
            vrsz_first = false;
        }
        if (vrsz_first == true)
        {
           mpProcess = mpProcess16VH;
        }
        else
        {
           mpProcess = mpProcess16HV;
        }
        mProcessType = RRSZ16_PROCESSING;

        resize_descriptor hrsz;
        get_polyphase_params(aSrc.cols, aDst.cols, HRSZ_DECIMAL_SCL, &hrsz);
            
        resize_descriptor vrsz;
        get_polyphase_params(aSrc.rows, aDst.rows, VRSZ_DECIMAL_SCL, &vrsz);
         
        if(mWeSelectedApuConfiguration == true)
        {
            if(mpProcess->SelectApuConfiguration(mApuConfig, mApexId))
            { 
                if (mpProcess) 
                {
                    delete mpProcess;
                    mpProcess = nullptr;
                    if (vrsz_first == true)
                    {
                       mpProcess16VH = nullptr;
                    }
                    else
                    {
                       mpProcess16HV = nullptr;
                    }
                }
                return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;    
            }
        }
         
        int32_t numCU = mpProcess->RetCuArrayWidth();
            
        mSrcWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.cols;
        mDstWidthUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.cols;
        mSrcHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aSrc.rows;
        mDstHeightUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) = aDst.rows;
         
        /* 
        ** Need to calculate chunk width right now to ensure the DST port
        ** is a multiple. Otherwise calling select scenario with the raw
        ** DST width may fail
        */
        dst_chunk_width = (aDst.cols + numCU - 1) / numCU;
        dst_chunk_width = (dst_chunk_width + 0x1) & ~0x1; 
        dst_chunk_width = dst_chunk_width > 0 ? dst_chunk_width : 2;
            
        src_chunk_width = (dst_chunk_width * aSrc.cols + aDst.cols - 1) / aDst.cols;
        src_chunk_width += hrsz.taps;
        src_chunk_width = (src_chunk_width + 0x3) & ~0x3;
            
        tile_width_in_chunks = (aDst.cols + dst_chunk_width - 1) / dst_chunk_width;
        tile_width_in_chunks = (tile_width_in_chunks + 0x3) & ~0x3;
            
        moInvalidUMat = vsdk::SUMat( 1, tile_width_in_chunks, VSDK_CV_32SC1 );    
         
        vsdk::Rect mRect_t(0, 0, dst_chunk_width * tile_width_in_chunks,
                                      aDst.rows);
         
        vsdk::SUMat dst_temp(aDst, mRect_t);
         
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moInvalidUMat)
                || mpProcess->ConnectIO("DST", dst_temp)
                || mpProcess->ConnectIO("SRC_WIDTH", mSrcWidthUMat)
                || mpProcess->ConnectIO("DST_WIDTH", mDstWidthUMat)
                || mpProcess->ConnectIO("SRC_HEIGHT", mSrcHeightUMat)
                || mpProcess->ConnectIO("DST_HEIGHT", mDstHeightUMat) )
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            weKeepGoing = false;
        }
        else
        {
          if(false == mIsResize16Initialized)
          {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("RESIZE16", mpProcess);
            lInfo.push_PortName("SRC");
            mvInfoClusters.push_back(lInfo);
            mIsResize16Initialized = true;
          }
        }
        //printf(" connectIO %d  ", lRetVal);
         
        if (mpProcess->SelectScenario("", 0, 0) != 0) 
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            weKeepGoing = false;
        }
         
        if (weKeepGoing == true)
        {
            /*
            ** Now that a scenario has been selected we can modify/hack it however we want
            ** since we know the available CMEM. The chunk heights are determined by creating a
            ** system of linear equations/inequalities that includes: the relationship between
            ** every non-fixed port size and an inequality between the available CMEM and the sum
            ** of every port buffer in the graph. Simplifying the system of equations leaves one unknown
            ** in the inequality which is the dst chunk height. Solving for this inequality gives us
            ** the dst chunk height which allows us to calculate the src chunk height as well.
            */
         
            int avail_cmem;
            avail_cmem = mpProcess->RetCmemHeapSizeInBytes() - 22 * sizeof(int32_t) - 4 * sizeof(int32_t);
               
            v_scale = (float)aSrc.rows / (float)aDst.rows;
            
            if (vrsz_first) {
                dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width) / (float)((2 * v_scale + 1) * (float)src_chunk_width + 2 * (float)dst_chunk_width);
            } else {
                dst_chunk_height = (mpProcess->RetCmemHeapSizeInBytes() - 20 * src_chunk_width - 10 * dst_chunk_width) / (float)(2 * v_scale * (float)src_chunk_width + (2 + v_scale) * (float)dst_chunk_width);
            }
         
            dst_chunk_height &= ~0x1;
            dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
            dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
               
            src_chunk_height = ceil(dst_chunk_height * v_scale);
            src_chunk_height += vrsz.taps;
            src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
                    
            tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
         
            int mem_size;
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
            if (vrsz_first) {
                mem_size += src_chunk_width * dst_chunk_height*sizeof(uint16_t);
            } else {
                mem_size += dst_chunk_width * src_chunk_height*sizeof(uint16_t);
            }
            
            while (((tile_size_in_bytes >= (1<<16) || src_chunk_height > 64) && dst_chunk_height > 1) || mem_size > avail_cmem) 
            {
                dst_chunk_height -= 2;
         
                dst_chunk_height &= ~0x1;
                dst_chunk_height = dst_chunk_height > 0 ? dst_chunk_height : 1;
                dst_chunk_height = dst_chunk_height < 32 ? dst_chunk_height : 32;
         
                src_chunk_height = ceil(dst_chunk_height * v_scale);
                src_chunk_height += vrsz.taps;
                src_chunk_height = (src_chunk_height + 0x1) & ~0x1;
         
                tile_size_in_bytes = src_chunk_width * src_chunk_height * tile_width_in_chunks*sizeof(uint16_t);
            
                mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
                if (vrsz_first) {
                    mem_size += src_chunk_width * dst_chunk_height*sizeof(uint16_t);
                } else {
                    mem_size += dst_chunk_width * src_chunk_height*sizeof(uint16_t);
                }
            }
         
            mem_size = (src_chunk_width * src_chunk_height * 2 + dst_chunk_width * dst_chunk_height * 2)*sizeof(uint16_t);
            if (vrsz_first) {
                mem_size += src_chunk_width * dst_chunk_height*sizeof(uint16_t);
            } else {
                mem_size += dst_chunk_width * src_chunk_height*sizeof(uint16_t);
            }
         
            //printf(" tile_size_in_bytes %d  mem_size %d, avail_cmem %d", tile_size_in_bytes, mem_size, avail_cmem);
          
            if (tile_size_in_bytes >= (1<<16) || mem_size > avail_cmem) 
            {
                lRetVal = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
                weKeepGoing = false;
            }
         
            if (weKeepGoing == true)
            {
                //printf("%d,%d,%d,%d,", src_chunk_width, src_chunk_height, dst_chunk_width, dst_chunk_height);
         
                /*
                ** Now we override the scenario table with the assumption index 0 = SRC, 1 = intermediate, 2 = DST
                */
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkWidth  = src_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[0].chunkHeight = src_chunk_height;
         
                if (vrsz_first) {
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = src_chunk_width;
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = dst_chunk_height;
                } else {
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkWidth  = dst_chunk_width;
                    mpProcess->RetSelectedScenario()->pBufferData[1].chunkHeight = src_chunk_height;
                }
         
                mpProcess->RetSelectedScenario()->pBufferData[2].chunkWidth  = dst_chunk_width;
                mpProcess->RetSelectedScenario()->pBufferData[2].chunkHeight = dst_chunk_height;
         
                /*
                ** Now that we know the chunk sizes for resize we can create the real motion comp offset table
                */
                mo_h = (aDst.rows + dst_chunk_height - 1) / dst_chunk_height;
         
                if (weKeepGoing == true)
                {
                    //printf( " mo_oal allcoated \n " );
         
                    moOffsUMat = vsdk::SUMat( mo_h, tile_width_in_chunks, VSDK_CV_32SC1 );
                    init_moOffsUMatets16_umat(aSrc, moOffsUMat, dst_chunk_width, dst_chunk_height, &hrsz, &vrsz);

                    mRect = vsdk::Rect(0, 0, dst_chunk_width * tile_width_in_chunks, aDst.rows);

                    int padDstWidth = aDst.step[0]/sizeof(int16_t);
                    if(mo_h*dst_chunk_height > aDst.rows + 32 || dst_chunk_width * tile_width_in_chunks > padDstWidth )
                    {
                        lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
                        weKeepGoing = false; 
                    }   
         
                    if (aDst.empty() == true) {
                        lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
                        weKeepGoing = false;
                    }
                }
            }
        }
    }
    if (false == weKeepGoing)
    {
        if(mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
            if (vrsz_first == true)
            {
               mpProcess16VH = nullptr;
            }
            else
            {
               mpProcess16HV = nullptr;
            }
        }
    }
    else
    {
        {
            vsdk::SMat srcMat = aSrc.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
            vsdk::SMat mattemp = aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
        } 
        vsdk::SUMat dst_t = vsdk::SUMat(aDst, mRect);
     
        if( mpProcess->ConnectIndirectInput("SRC", aSrc, moOffsUMat)
            || mpProcess->ConnectIO("DST", dst_t))
        {
            delete mpProcess;
            mpProcess = nullptr;
            if (vrsz_first == true)
            {
               mpProcess16VH = nullptr;
            }
            else
            {
               mpProcess16HV = nullptr;
            }
            lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
    }

    return lRetVal;
}

APEXCV_LIB_RESULT Resize::acf_resize16()
{
    APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
 
    if (mpProcess)
    {    
        mpProcess->FlushAndInvalidateAllIO();
        int status = 0;  
        status  = mpProcess->Start();
        status |= mpProcess->Wait();

        //printf("  finished caculation %d \n", lRetVal);
        if(status)    
        {
            lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
        }
    }

   return lRetVal;
}

}

int32_t QueryApuArraySize(int32_t& lArrayWidth)
{
   int32_t lRetVal = 0;
   int32_t lHwVersion = global_hal_RetHwVersion(0);
   switch(lHwVersion)
   {
   case 0x321:
      lArrayWidth = 32;
      cmem_if_hal_set_apu_cfg(0, CFG_321__APU_0_CU_0_31_SMEM_0_1);
      break;
   case 0x642:

      cmem_if_hal_set_apu_cfg(0, CFG_642__APU_0_CU_0_63_SMEM_0_3);
      lArrayWidth = 64;
      break;
   default:
      printf("Start_Resize() -> unknown APU configuration detected (%X)\n", lHwVersion);
      lRetVal = 1;
   }

   return lRetVal;
}



APEXCV_LIB_RESULT TestResizeParams(int aSrcWidth,
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
    if (aSrcWidth > 0 && aSrcHeight > 0 && aDstWidth > 0 && aDstHeight > 0 )
    {

        float vScale = (float)aDstHeight / (float)aSrcHeight;
        float hScale = (float)aDstWidth  / (float)aSrcWidth;

        if (hScale <= 0.25) 
        {
            //cout << "Reference resize will not match for scale factors <= 0.25. See CLOUD-495." << endl;
        }

        // Test for valid vertical scale factor (limited to 32 for upscale)
        if (vScale > 32) 
        {
            printf("Invalid vertical scale factor - upscaling currently limited to 32x\n");
            ret++;
        }
        int32_t numCU = 1;
        if (QueryApuArraySize(numCU)) 
        {
            ret = 1;
        }
    }else{
        ret = 1;
    }

   if( ret )
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;   
   }
   return APEXCV_SUCCESS;
}

