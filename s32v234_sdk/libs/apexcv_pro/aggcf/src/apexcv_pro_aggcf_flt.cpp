/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016-2017 NXP
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

#include <oal.h>
#include "apexcv_pro_aggcf.h"
#include "icp_feature.h"
#include "icp_types.h"
#include <apexcv_error_codes.hpp>

namespace apexcv{

// init algorithm parameters
APEXCV_LIB_RESULT AGGCFflt3x3::InitParams(uint32_t aP, uint32_t aNormConst)
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
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(1u) = aNormConst;
      
      mFilterP = aP;
      mNormConst = aNormConst;
   }
   else
   {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }
 
   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFflt3x3::SetParameters(uint32_t aP, uint32_t aNormConst) 
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   lRetVal = InitParams(aP, aNormConst);
   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFflt3x3::ReconnectIO(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOut,
               uint32_t aP, uint32_t aNormConst 
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize(aIn, aOut, aP, aNormConst);
   }
   else
   {
      lRetVal = InitParams(aP, aNormConst);
      if (lRetVal != APEXCV_SUCCESS) 
      {
         return lRetVal;
      }

      mImageWidth  = aIn.cols;
      mImageHeight = aIn.rows;

      if(0 == aNormConst)
      {
          if( mProcessAggFlt.ConnectIO("INPUT",   aIn)
              || mProcessAggFlt.ConnectIO("OUTPUT",  aOut )
              || mProcessAggFlt.ConnectIO("PARAMS",  mParams) )
          {
             return APEXCV_ERROR_ACF_PROC_CONNECTIO; 
          }
      }
      else
      {
          if( mProcessAggFltNorm.ConnectIO("INPUT",   aIn)
              || mProcessAggFltNorm.ConnectIO("OUTPUT",  aOut )
              || mProcessAggFltNorm.ConnectIO("PARAMS",  mParams))
          {
             return APEXCV_ERROR_ACF_PROC_CONNECTIO;
          } 
      }
   }

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFflt3x3::Initialize(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut,
       uint32_t aP, uint32_t aNormConst 
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState)
   {
      // Keep the image size info
      mImageWidth  = aIn.cols;
      mImageHeight = aIn.rows;

      lRetVal = InitParams(aP, aNormConst);

      if ( APEXCV_SUCCESS != lRetVal)
      {
         return lRetVal;
      }

      if( mProcessAggFlt.Initialize() || mProcessAggFltNorm.Initialize())
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }

      if ( 0 == aNormConst)
      {
         if(1 == mSelectedApuConfiguration)
         {
            if( mProcessAggFlt.SelectApuConfiguration(mApuConfig, mApexId) )
            {
               return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            }
         }
         
         if( mProcessAggFlt.ConnectIO("INPUT",    aIn)
             || mProcessAggFlt.ConnectIO("OUTPUT",   aOut)
             || mProcessAggFlt.ConnectIO("PARAMS", mParams))
         {
            return APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }

         {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("AGGCF_FLT", (void*) &mProcessAggFlt);
            lInfo.push_PortName("INPUT");
            mvInfoClusters.push_back(lInfo);
         }
      }
      else
      {
         if(1 == mSelectedApuConfiguration)
         {
            if(mProcessAggFltNorm.SelectApuConfiguration(mApuConfig, mApexId))
            {
               return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
            }
         }

         if( mProcessAggFltNorm.ConnectIO("INPUT",    aIn)
             || mProcessAggFltNorm.ConnectIO("OUTPUT",   aOut)
             || mProcessAggFltNorm.ConnectIO("PARAMS",   mParams) )
         {
            return APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }

         {
            ApexcvHostBaseBaseClass::InfoCluster lInfo;
            lInfo.set_ACF("AGGCF_FLT_NORM", (void*) &mProcessAggFltNorm);
            lInfo.push_PortName("INPUT");
            mvInfoClusters.push_back(lInfo);
         }
      }

      mInitState = 1;
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }

   return lRetVal; 
}

APEXCV_LIB_RESULT AGGCFflt3x3::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState)
   {
      lRetVal = ComputeFlt();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFflt3x3::ComputeFlt()
{
   int status = 0;
//   int inChunkWidth = 0, inChunkHeight = 0, outChunkWidth = 0, outChunkHeight = 0;

   if (mInitState)
   {
      if( 0 == mNormConst)
      {
          int32_t numCU = mProcessAggFlt.RetCuArrayWidth();

          int32_t chunkWidth = (mImageWidth + numCU - 1) / numCU;
          chunkWidth = (chunkWidth + 0x1) & ~0x1;
          chunkWidth = chunkWidth > 0 ? chunkWidth : 2;
          status = mProcessAggFlt.SelectScenario("INPUT", chunkWidth, 0);
          if(status)
          {
             return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
          }
        
          mProcessAggFlt.FlushAndInvalidateAllIO();
          status  = mProcessAggFlt.Start();
          status |= mProcessAggFlt.Wait();
          if( status)
          {
             return APEXCV_ERROR_ACF_PROC_EXEC;
          }
      }
      else
      {
          int32_t numCU = mProcessAggFltNorm.RetCuArrayWidth();

          int32_t chunkWidth = (mImageWidth + numCU - 1) / numCU;
          chunkWidth = (chunkWidth + 0x1) & ~0x1;
          chunkWidth = chunkWidth > 0 ? chunkWidth : 2;

          status = mProcessAggFltNorm.SelectScenario("INPUT", chunkWidth, 0);
          if(status)
          {
             return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
          }    

          mProcessAggFltNorm.FlushAndInvalidateAllIO();
          status  = mProcessAggFltNorm.Start();
          status |= mProcessAggFltNorm.Wait();
          if(status)
          {
             return APEXCV_ERROR_ACF_PROC_EXEC;
          }
      }

   } 
   else
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

void AGGCFflt3x3::Release()
{
   mParams = vsdk::SUMat();
}

AGGCFflt3x3::AGGCFflt3x3()
{
   mInitState = mImageWidth = mImageHeight = 0;

   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0;

   mSelectedApuConfiguration = 0;

   mFilterP = 2; mNormConst = 0;
}

AGGCFflt3x3::~AGGCFflt3x3()
{
   Release();
}

APEXCV_LIB_RESULT AGGCFflt3x3::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
{
   // hard coded to 0/1 for valid input parameter check
   if( aApexId < 0 || aApexId > 1)
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   mApuConfig = aApuConfig;
   mApexId = aApexId;
   mSelectedApuConfiguration = 1;
   return APEXCV_SUCCESS;
}

AGGCFfltRef::AGGCFfltRef()
{
}

AGGCFfltRef::~AGGCFfltRef()
{

}

int AGGCFfltRef::Process(vsdk::SUMat &aIn, vsdk::SUMat &aOut, uint32_t aP)
{

   int width = aIn.cols, height = aIn.rows;
   uint32_t *T = new uint32_t[width+2];
   vsdk::SMat inMat = aIn.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint16_t *inPtr = (uint16_t *)inMat.data;  

   vsdk::SMat outMat = aOut.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint16_t *outPtr = (uint16_t *)outMat.data;

   uint16_t* inputPad = image_pad(inPtr, width, height, 1, 1, 1, 1);
   int32_t inStride = width + 2;
   int32_t outStride = width;

   uint16_t* inputPadPtr = inputPad + inStride + 1;

   triangleFilt1_c_row(inputPadPtr, outPtr, height, width, inStride, outStride, T, aP, 1);

   delete[] inputPad;
   delete[] T;
   return 0;
}


}


