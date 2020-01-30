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


APEXCV_LIB_RESULT AGGCFtriflt::InitParams(int aR)
{
   if( aR < 1 || aR > 5)
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   
   mParams = vsdk::SUMat(1, 1, VSDK_CV_32SC1); 
   if( !mParams.empty())
   {
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = aR;
      mFilterRadius = aR;
   }
   else
   {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }
 
   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFtriflt::SetParameters(int aR) 
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   lRetVal = InitParams(aR);
   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFtriflt::ReconnectIO(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut,
       int aR 
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal |= Initialize( aIn, aOut, aR);
   }
   else
   {
      if ((aIn.cols != mImageWidth) || (aIn.rows != mImageHeight)) 
      {
         mImageWidth  = aIn.cols;
         mImageHeight = aIn.rows;
      }
       
      if( mProcessAggFlt.ConnectIO("INPUT",   aIn)
          || mProcessAggFlt.ConnectIO("OUTPUT",  aOut ) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      } 
       
      if( aR !=  mFilterRadius)
      {
         lRetVal = InitParams(aR); 
         if( APEXCV_SUCCESS == lRetVal )
         {   
            if( mProcessAggFlt.ConnectIO("PARAMS",   mParams))
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }   
         }
         else
         {
            return lRetVal;
         } 
      }  
   } 

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFtriflt::Initialize(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut,
       int aR 
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      // Keep the image size info
      mImageWidth  = aIn.cols;
      mImageHeight = aIn.rows;    

      lRetVal = InitParams(aR);

      if (APEXCV_SUCCESS != lRetVal)
      {
         return lRetVal; 
      } 

      if( mProcessAggFlt.Initialize())
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
  
      if(1 == mSelectedApuConfiguration)
      {
         if( mProcessAggFlt.SelectApuConfiguration(mApuConfig, mApexId))
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         } 
      }
         
      if( mProcessAggFlt.ConnectIO("INPUT",    aIn)
          || mProcessAggFlt.ConnectIO("OUTPUT",   aOut)
          || mProcessAggFlt.ConnectIO("PARAMS",   mParams) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      } 
      
      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("AGGCF_TRIFLT", (void*) &mProcessAggFlt);
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

APEXCV_LIB_RESULT AGGCFtriflt::Process()
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

APEXCV_LIB_RESULT AGGCFtriflt::ComputeFlt()
{
   int status = 0;
   if (mInitState) 
   {
      mProcessAggFlt.FlushAndInvalidateAllIO();
      status  = mProcessAggFlt.Start();
      status |= mProcessAggFlt.Wait();
      if(status)
      {
         return APEXCV_ERROR_ACF_PROC_EXEC;
      }  
   } 
   else 
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

void AGGCFtriflt::Release()
{

   mParams = vsdk::SUMat();
   
}

AGGCFtriflt::AGGCFtriflt()
{
   mInitState = mImageWidth = mImageHeight = 0;

   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0;

   mSelectedApuConfiguration = 0;
}

AGGCFtriflt::~AGGCFtriflt()
{
   Release();
}
   
APEXCV_LIB_RESULT AGGCFtriflt::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

}


