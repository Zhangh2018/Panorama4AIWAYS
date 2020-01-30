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

namespace apexcv{



APEXCV_LIB_RESULT AGGCFluv::ReconnectIO(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOutl,
               vsdk::SUMat &aOutu,
               vsdk::SUMat &aOutv
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize( aIn, aOutl, aOutu, aOutv );
   }
   else
   {
      if( mProcessAggLuv.ConnectIO("INPUT",    aIn)
          || mProcessAggLuv.ConnectIO("OUTPUTL",  aOutl )
          || mProcessAggLuv.ConnectIO("OUTPUTU",  aOutu )
          || mProcessAggLuv.ConnectIO("OUTPUTV",  aOutv ) )
      { 
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }  
   } 

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFluv::Initialize(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOutl,
               vsdk::SUMat &aOutu,
               vsdk::SUMat &aOutv
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      // Keep the image size info
      mImageWidth  = aIn.cols;
      mImageHeight = aIn.rows;    

     
      if( mProcessAggLuv.Initialize() )
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }   
 
      if(1 == mSelectedApuConfiguration)
      {
         if(mProcessAggLuv.SelectApuConfiguration(mApuConfig, mApexId))
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;  
         } 
      }
         
      if( mProcessAggLuv.ConnectIO("INPUT",     aIn)
          || mProcessAggLuv.ConnectIO("OUTPUTL",   aOutl)
          || mProcessAggLuv.ConnectIO("OUTPUTU",   aOutu)
          || mProcessAggLuv.ConnectIO("OUTPUTV",   aOutv) )
      { 
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;   
      }

      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("AGGCF_LUV", (void*) &mProcessAggLuv);
         lInfo.push_PortName("INPUT");
         mvInfoClusters.push_back(lInfo);
      }

      mInitState = 1;
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;

   }

   return lRetVal; 
}

APEXCV_LIB_RESULT AGGCFluv::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState) 
   {
      lRetVal = Computeluv();
   }
   else
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFluv::Computeluv()
{
   int status = 0;
   if (mInitState) 
   {
      mProcessAggLuv.FlushAndInvalidateAllIO();
      status = mProcessAggLuv.Start();
      status |= mProcessAggLuv.Wait();
      
      if(status)
      {
         return APEXCV_ERROR_ACF_PROC_EXEC;
      }

#if AGGCF_PERF_PROF
      mProcessAggLuv.DumpApuProfilingInfoToStdout();
      mProcessAggLuv.DumpHostProfilingInfoToStdout();

      ACF_Process_PortInfo* lPortInfo;
      lPortInfo = mProcessAggLuv.RetPortInfo("INPUT");
      printf("aggcf luv input chunkwidth %d, chunkheight %d", lPortInfo->mChunkWidth, lPortInfo->mChunkHeight);
      lPortInfo = mProcessAggLuv.RetPortInfo("OUTPUTL");
      printf("output chunkwidth %d, chunkheight %d \n", lPortInfo->mChunkWidth, lPortInfo->mChunkHeight);

#endif

   } 
   else 
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

AGGCFluv::AGGCFluv()
{
   mInitState = mImageWidth = mImageHeight = 0;

   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0;

   mSelectedApuConfiguration = 0;
}

void AGGCFluv::Release()
{
   return;
}

AGGCFluv::~AGGCFluv()
{
   Release();
}
   
APEXCV_LIB_RESULT AGGCFluv::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

AGGCFluvRef::AGGCFluvRef()
{

}

AGGCFluvRef::~AGGCFluvRef()
{

}

int AGGCFluvRef::Process(vsdk::SUMat &aIn,
                         vsdk::SUMat &aOutl,
                         vsdk::SUMat &aOutu,
                         vsdk::SUMat &aOutv)
{
   int width = aIn.cols, height = aIn.rows;
 
   vsdk::SMat inMat = aIn.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint32_t *inPtr = (uint32_t *)inMat.data;
 
   vsdk::SMat outlMat = aOutl.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint16_t *outlPtr = (uint16_t *)outlMat.data;

   vsdk::SMat outuMat = aOutu.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint16_t *outuPtr = (uint16_t *)outuMat.data;

   vsdk::SMat outvMat = aOutv.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
   uint16_t *outvPtr = (uint16_t *)outvMat.data;

   apu_aggcf_rgb2luv( inPtr, outlPtr, outuPtr, outvPtr, width, width, width, height );
   
   return 0;
}

}


