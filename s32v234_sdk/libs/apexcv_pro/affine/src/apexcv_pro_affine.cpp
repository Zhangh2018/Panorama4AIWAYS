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
 
#include <apexcv_pro_affine.h>
#include <stdint.h>
#include <icp_data.h>
#include <oal.h>
#include <affine_definitions.h>
#include <stdlib.h>
#include <apexcv_error_codes.hpp>

// GENERATED ACF PROCESS
#include <AFFINE_BILINEAR_INTERPOLATE.hpp>

#define AFFINE_PERF_PROF 0

using namespace apexcv;

Affine::Affine() : 
  mpProcess(nullptr),
  mbInitialized(false)
{
}

Affine::~Affine()
{
  if(nullptr != mpProcess)
  {
    delete ((ACF_Process_APU *)mpProcess);
    mpProcess = nullptr;
  }
}

APEXCV_LIB_RESULT Affine::Initialize( vsdk::SUMat& aSrc            /*!< Source image buffer.*/,
                                   vsdk::SUMat& aMat            /*!< Affine matrix.*/,
                                   vsdk::SUMat& aDst            /*!< Destination image buffer.*/)
{
   int rval = 0;

   if (mbInitialized)
   {
     rval = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
     goto EXIT;
   }

   {

     mInvMat = vsdk::SUMat(1, 1, VSDK_CV_MAKETYPE(VSDK_CV_32S, 6));

     if (mInvMat.empty())
     {
       rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
       goto EXIT;
     }

     AFFINE_BILINEAR_INTERPOLATE* pf = new AFFINE_BILINEAR_INTERPOLATE;
     mpProcess = (ACF_Process_APU*)pf;

     if (pf->Initialize())
     {
       rval = APEXCV_ERROR_ACF_PROC_INITIALIZE;
       goto EXIT;
     }


     {
       vsdk::SMat lMatMat = aMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
       vsdk::SMat invMatMat = mInvMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

       int32_t *pMat; pMat = (int32_t*)lMatMat.data;
       int32_t *pInvMat; pInvMat = (int32_t*)invMatMat.data;
       int32_t det; det = pMat[0] * pMat[4] - pMat[1] * pMat[3];
       det >>= AB_BITS;

       if (det != 0)
       {
         int32_t delta = det >> 1;
         int32_t d0 = ((pMat[4] << INTER_BITS) + delta) / det;
         int32_t d1 = ((pMat[1] << INTER_BITS) + delta) / det;
         int32_t d2 = ((pMat[3] << INTER_BITS) + delta) / det;
         int32_t d3 = ((pMat[0] << INTER_BITS) + delta) / det;

         pInvMat[0] = d0;
         pInvMat[1] = -d1;
         pInvMat[2] = (d1 * pMat[5] - d0 * pMat[2]) >> AB_BITS;
         pInvMat[3] = -d2;
         pInvMat[4] = d3;
         pInvMat[5] = (d2 * pMat[2] - d3 * pMat[5]) >> AB_BITS;
       }
       else
       {
         pInvMat[0] = 0;
         pInvMat[1] = 0;
         pInvMat[2] = 0;
         pInvMat[3] = 0;
         pInvMat[4] = 0;
         pInvMat[5] = 0;
       }

       float maxRatio; maxRatio = 0;

       bool validCandidate = false;

       int32_t numCU = pf->RetCuArrayWidth();
       int32_t dstChunkWidth = (aDst.cols + numCU - 1) / numCU;
       dstChunkWidth = (dstChunkWidth + 1) & ~1;

       int32_t tileWidthInChunks; tileWidthInChunks = (aDst.cols + dstChunkWidth - 1) / dstChunkWidth;

       for (int i = 0; i < 7; ++i)
       {
         blockDesc b;

         b.dstWidth = dstChunkWidth;
         b.dstHeight = 1 << i;
         b.srcWidth = ((b.dstWidth * abs(pInvMat[0]) + INTER_CEIL) >> INTER_BITS) + ((b.dstHeight * abs(pInvMat[1]) + INTER_CEIL) >> INTER_BITS);
         b.srcWidth = b.srcWidth > 0 ? b.srcWidth : 1;
         b.srcWidth += 2;
         b.srcWidth = (b.srcWidth + 0x3) & ~0x3;
         b.srcHeight = ((b.dstWidth * abs(pInvMat[3]) + INTER_CEIL) >> INTER_BITS) + ((b.dstHeight * abs(pInvMat[4]) + INTER_CEIL) >> INTER_BITS);
         b.srcHeight = b.srcHeight > 0 ? b.srcHeight : 1;
         b.srcHeight += 2;

         b.ratio = (float)(b.dstWidth * b.dstHeight) / (float)(b.srcWidth * b.srcHeight);
         b.memSize = b.srcWidth * b.srcHeight * 2 + b.dstWidth * b.dstHeight * 2;

         if (b.memSize > pf->RetCmemHeapSizeInBytes())
         {
           continue;
         }

         if (b.srcWidth * b.srcHeight * tileWidthInChunks >= (1 << 16))
         {
           continue;
         }

         if ((b.ratio > maxRatio) && ((aDst.rows%b.dstHeight) == 0))
         {
           maxRatio = b.ratio;
           mValidConfig = b;
           validCandidate = true;
         }
       }

       if (!validCandidate)
       {
         rval = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
         goto EXIT;
       }

       {
         int32_t blocksX;
         int32_t blocksY;
         int32_t *o;
         int32_t ox;
         int32_t oy;

         //void *moOal = 0;
         //void *iwOal = 0;
         //void *ihOal = 0;

         blocksX = (aDst.cols + mValidConfig.dstWidth - 1) / mValidConfig.dstWidth;
         blocksY = (aDst.rows + mValidConfig.dstHeight - 1) / mValidConfig.dstHeight;


         mOffset = vsdk::SUMat(blocksY, blocksX, VSDK_CV_32SC1);

         if (mOffset.empty())
         {
           rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
           goto EXIT;
         }

         {
           vsdk::SMat mOffsetMat = mOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
           o = (int32_t*)mOffsetMat.data;

           ox = pInvMat[2];
           oy = pInvMat[5];

           if (pInvMat[0] < 0)
           {
             ox += pInvMat[0] * mValidConfig.dstWidth;
           }

           if (pInvMat[1] < 0)
           {
             ox += pInvMat[1] * mValidConfig.dstHeight;
           }

           if (pInvMat[3] < 0)
           {
             oy += pInvMat[3] * mValidConfig.dstWidth;
           }

           if (pInvMat[4] < 0)
           {
             oy += pInvMat[4] * mValidConfig.dstHeight;
           }

           for (int y = 0; y < aDst.rows; y += mValidConfig.dstHeight)
           {
             for (int x = 0; x < aDst.cols; x += mValidConfig.dstWidth)
             {
               int32_t bx = x * pInvMat[0] + y * pInvMat[1] + ox;
               int32_t by = x * pInvMat[3] + y * pInvMat[4] + oy;

               bx >>= INTER_BITS;
               by >>= INTER_BITS;

               bx = bx > 0 ? bx : 0;
               bx = bx < aDst.cols - mValidConfig.srcWidth ? bx : aDst.cols - mValidConfig.srcWidth;

               by = by > 0 ? by : 0;
               by = by < aDst.rows - mValidConfig.srcHeight ? by : aDst.rows - mValidConfig.srcHeight;

               *o = by * aDst.cols + bx;
               o++;
             }
           }
         }
       }
     }


     mImageWidth = vsdk::SUMat(1, 1, VSDK_CV_16SC1);
     mImageHeight = vsdk::SUMat(1, 1, VSDK_CV_16SC1);

     if (mImageWidth.empty() || mImageHeight.empty())
     {
       rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
       goto EXIT;
     }
     {
       mImageWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED).at<int16_t>(0u) = aSrc.cols;
       mImageHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED).at<int16_t>(0u) = aSrc.rows;
     }



     if (pf->ConnectIndirectInput("INPUT", aSrc, mOffset) ||
       pf->ConnectIO("MATRIX", mInvMat) ||
       pf->ConnectIO("IMAGE_WIDTH", mImageWidth) ||
       pf->ConnectIO("IMAGE_HEIGHT", mImageHeight) ||
       pf->ConnectIO("OUTPUT", aDst))
     {
       rval = APEXCV_ERROR_ACF_PROC_CONNECTIO;
       goto EXIT;
     }



     if (pf->SelectScenario("", 0, 0))
     {
       rval = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
       goto EXIT;
     }

     {
       //uint16_t oldChunkW0, oldChunkH0, oldChunkW1, oldChunkH1;
       acf_scenario *s;
       s = pf->RetSelectedScenario();

       //oldChunkW0 = s->pBufferData[0].chunkWidth;
       //oldChunkH0 = s->pBufferData[0].chunkHeight;
       //oldChunkW1 = s->pBufferData[1].chunkWidth;
       //oldChunkH1 = s->pBufferData[1].chunkHeight;

       s->pBufferData[0].chunkWidth = mValidConfig.srcWidth;
       s->pBufferData[0].chunkHeight = mValidConfig.srcHeight;
       s->pBufferData[1].chunkWidth = mValidConfig.dstWidth;
       s->pBufferData[1].chunkHeight = mValidConfig.dstHeight;
     }

     {
       ApexcvHostBaseBaseClass::InfoCluster lInfo;
       lInfo.set_ACF("AFFINE_BILINEAR_INTERPOLATE", pf);
       lInfo.push_PortName("INPUT");
       mvInfoClusters.push_back(lInfo);
     }
   }
   mbInitialized = true;

   EXIT:
   if((rval != APEXCV_SUCCESS) && (rval != APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED))
   {
     if(nullptr != mpProcess)
     {
        delete ((ACF_Process_APU *)mpProcess);
        mpProcess = nullptr;
     }
   }

   return rval;
}

APEXCV_LIB_RESULT Affine::ReconnectIO(vsdk::SUMat& aSrc            /*!< Source image buffer.*/,
  vsdk::SUMat& aDst            /*!< Destination image buffer.*/)
{
  APEXCV_LIB_RESULT lRetVal;
  if (nullptr == mpProcess)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    AFFINE_BILINEAR_INTERPOLATE* pf = (AFFINE_BILINEAR_INTERPOLATE*)mpProcess;
    //int status = 0;


    if ((mImageWidth.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) == aSrc.cols) &&
      (mImageHeight.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int16_t>(0u) == aSrc.rows))
    {
      lRetVal = APEXCV_ERROR_INVALID_ARGUMENT;
    }
    else
    {
      if (pf->ConnectIndirectInput("INPUT", aSrc, mOffset) ||
        pf->ConnectIO("OUTPUT", aDst))
      {
        lRetVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      else
      {
        if (pf->SelectScenario("", 0, 0))
        {
          lRetVal = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
        }
        else
        {
          //uint16_t oldChunkW0, oldChunkH0, oldChunkW1, oldChunkH1;
          acf_scenario *s;
          s = pf->RetSelectedScenario();

          //oldChunkW0 = s->pBufferData[0].chunkWidth;
          //oldChunkH0 = s->pBufferData[0].chunkHeight;
          //oldChunkW1 = s->pBufferData[1].chunkWidth;
          //oldChunkH1 = s->pBufferData[1].chunkHeight;

          s->pBufferData[0].chunkWidth = mValidConfig.srcWidth;
          s->pBufferData[0].chunkHeight = mValidConfig.srcHeight;
          s->pBufferData[1].chunkWidth = mValidConfig.dstWidth;
          s->pBufferData[1].chunkHeight = mValidConfig.dstHeight;
          lRetVal = APEXCV_SUCCESS;
        }
      }
    }
  }
  return lRetVal;
}


APEXCV_LIB_RESULT Affine::Process()
{
  APEXCV_LIB_RESULT lRetVal;
  if (nullptr == mpProcess)
  {
    lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  else
  {
    AFFINE_BILINEAR_INTERPOLATE* pf = (AFFINE_BILINEAR_INTERPOLATE*)mpProcess;
    int status = 0;
    status |= pf->Start();
    status |= pf->Wait();

    if (status)
    {
      lRetVal = APEXCV_ERROR_ACF_PROC_EXEC;
    }
    else
    {
      lRetVal = APEXCV_SUCCESS;
    }
  }
  return lRetVal;
}
