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


APEXCV_LIB_RESULT AGGCFScaleDown2::ReconnectIO(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize(aIn, aOut);
   }
   else
   {

      if( mProcessAggScaleDown2.ConnectIO("SRC",   aIn)
            || mProcessAggScaleDown2.ConnectIO("DST",  aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO; 
      }
   }

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFScaleDown2::Initialize(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState)
   {
      if( mProcessAggScaleDown2.Initialize())
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }

      if(1 == mSelectedApuConfiguration)
      {
         if( mProcessAggScaleDown2.SelectApuConfiguration(mApuConfig, mApexId) )
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         }
      }
         
      if( mProcessAggScaleDown2.ConnectIO("SRC",    aIn)
             || mProcessAggScaleDown2.ConnectIO("DST",   aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("AGGCF_SCALE_DOWN2", (void*) &mProcessAggScaleDown2);
         lInfo.push_PortName("SRC");
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

APEXCV_LIB_RESULT AGGCFScaleDown2::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState)
   {
      lRetVal = ScaleDown2();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFScaleDown2::ScaleDown2()
{
   int status = 0;

   if (mInitState)
   {
        
      mProcessAggScaleDown2.FlushAndInvalidateAllIO();
      status  = mProcessAggScaleDown2.Start();
      status |= mProcessAggScaleDown2.Wait();
      if( status)
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


AGGCFScaleDown2::AGGCFScaleDown2()
{
   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0; mInitState =0;

   mSelectedApuConfiguration = 0;
}

AGGCFScaleDown2::~AGGCFScaleDown2()
{
}

APEXCV_LIB_RESULT AGGCFScaleDown2::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

APEXCV_LIB_RESULT AGGCFScaleDown4::ReconnectIO(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize(aIn, aOut);
   }
   else
   {

      if( mProcessAggScaleDown4.ConnectIO("SRC",   aIn)
            || mProcessAggScaleDown4.ConnectIO("DST",  aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO; 
      }
   }

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFScaleDown4::Initialize(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState)
   {
      if( mProcessAggScaleDown4.Initialize())
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }

      if(1 == mSelectedApuConfiguration)
      {
         if( mProcessAggScaleDown4.SelectApuConfiguration(mApuConfig, mApexId) )
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         }
      }
 
      if( mProcessAggScaleDown4.ConnectIO("SRC",    aIn)
             || mProcessAggScaleDown4.ConnectIO("DST",   aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("AGGCF_SCALE_DOWN4", (void*) &mProcessAggScaleDown4);
         lInfo.push_PortName("SRC");
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

APEXCV_LIB_RESULT AGGCFScaleDown4::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState)
   {
      lRetVal = ScaleDown4();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFScaleDown4::ScaleDown4()
{
   int status = 0;

   if (mInitState)
   {
        
      mProcessAggScaleDown4.FlushAndInvalidateAllIO();
      status  = mProcessAggScaleDown4.Start();
      status |= mProcessAggScaleDown4.Wait();
      if( status)
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

AGGCFScaleDown4::AGGCFScaleDown4()
{
   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0; mInitState =0;

   mSelectedApuConfiguration = 0;
}

AGGCFScaleDown4::~AGGCFScaleDown4()
{
}

APEXCV_LIB_RESULT AGGCFScaleDown4::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

APEXCV_LIB_RESULT AGGCFScaleDown8::ReconnectIO(
               vsdk::SUMat &aIn,
               vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize(aIn, aOut);
   }
   else
   {

      if( mProcessAggScaleDown8.ConnectIO("SRC",   aIn)
            || mProcessAggScaleDown8.ConnectIO("DST",  aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO; 
      }
   }

   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFScaleDown8::Initialize(
       vsdk::SUMat &aIn,
       vsdk::SUMat &aOut
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState)
   {
      if( mProcessAggScaleDown8.Initialize())
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }

      if(1 == mSelectedApuConfiguration)
      {
         if( mProcessAggScaleDown8.SelectApuConfiguration(mApuConfig, mApexId) )
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         }
      }
         
      if( mProcessAggScaleDown8.ConnectIO("SRC",    aIn)
             || mProcessAggScaleDown8.ConnectIO("DST",   aOut) )
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      {
         ApexcvHostBaseBaseClass::InfoCluster lInfo;
         lInfo.set_ACF("AGGCF_SCALE_DOWN8", (void*) &mProcessAggScaleDown8);
         lInfo.push_PortName("SRC");
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

APEXCV_LIB_RESULT AGGCFScaleDown8::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState)
   {
      lRetVal = ScaleDown8();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFScaleDown8::ScaleDown8()
{
   int status = 0;

   if (mInitState)
   {
        
      mProcessAggScaleDown8.FlushAndInvalidateAllIO();
      status  = mProcessAggScaleDown8.Start();
      status |= mProcessAggScaleDown8.Wait();
      if( status)
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

AGGCFScaleDown8::AGGCFScaleDown8()
{
   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0; mInitState = 0;

   mSelectedApuConfiguration = 0;
}

AGGCFScaleDown8::~AGGCFScaleDown8()
{
}

APEXCV_LIB_RESULT AGGCFScaleDown8::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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
AGGCFScaleRef::AGGCFScaleRef()
{
}

AGGCFScaleRef::~AGGCFScaleRef()
{
}

int AGGCFScaleRef::Process(
           vsdk::SUMat& aIn,
           vsdk::SUMat& aOut,
           int scale )
{
   int width = aIn.cols, height = aIn.rows;

   vsdk::SMat inMat = aIn.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *inPtr = (uint16_t *)inMat.data;

   vsdk::SMat outMat = aOut.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *outPtr = (uint16_t *)outMat.data;

   aggcf_pyramid_downscale_ref(inPtr, outPtr, width, height, scale); 

   return 0;
}

}


