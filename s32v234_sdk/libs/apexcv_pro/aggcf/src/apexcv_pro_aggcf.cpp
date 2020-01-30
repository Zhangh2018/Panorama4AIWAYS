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

// init algorithm parameter
APEXCV_LIB_RESULT AGGCFMagOri::InitParams(int aFull)
{
   if( aFull == 0 || aFull == 1)
   {

      mParams = vsdk::SUMat(1, 1, VSDK_CV_32SC1);

      if(!mParams.empty())
      {
         mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = aFull;
         mFullrange = aFull;
      }
      else
      {
         return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
   }   
   else
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }


   return APEXCV_SUCCESS;

}

APEXCV_LIB_RESULT AGGCFMagOri::SetParameters(int aFull) 
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   lRetVal = InitParams(aFull);
   return lRetVal;
}

// reconnect IO
APEXCV_LIB_RESULT AGGCFMagOri::ReconnectIO(
       vsdk::SUMat &srcl,
       vsdk::SUMat &srcu,
       vsdk::SUMat &srcv,
       vsdk::SUMat &mag,
       vsdk::SUMat &ori,
       int aFull 
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      lRetVal = Initialize( srcl, srcu, srcv, mag, ori, aFull);

   }
   else
   {
      if ((srcl.cols != mImageWidth) || (srcl.rows != mImageHeight)) 
      {
         mImageWidth  = srcl.cols;
         mImageHeight = srcl.rows;
      } 
 
      if( mProcessAggCF.ConnectIO("INPUTL",   srcl)
          || mProcessAggCF.ConnectIO("INPUTU",   srcu)
          || mProcessAggCF.ConnectIO("INPUTV",   srcv)
          || mProcessAggCF.ConnectIO("OUTPUTM",  mag)
          || mProcessAggCF.ConnectIO("OUTPUTO",  ori))
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      if(aFull !=  mFullrange)
      {
         lRetVal = InitParams(aFull);
         if( APEXCV_SUCCESS == lRetVal)
         {
            if( mProcessAggCF.ConnectIO("PARAMS",   mParams))
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

APEXCV_LIB_RESULT AGGCFMagOri::Initialize(
               vsdk::SUMat& aSrcl,
               vsdk::SUMat& aSrcu,
               vsdk::SUMat& aSrcv,
               vsdk::SUMat& aMag,
               vsdk::SUMat& sOri,
               int aFull
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState)
   {
      // Keep the image size info
      mImageWidth  = aSrcl.cols;
      mImageHeight = aSrcl.rows;

      lRetVal = InitParams(aFull);

      if( APEXCV_SUCCESS != lRetVal)
      {
         return lRetVal;
      }

      if( mProcessAggCF.Initialize() )
      {
         return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      
      //mProcessAggCF.mWaitTimeoutInUs = 3000000; 
      if(1 == mSelectedApuConfiguration)
      {
         if ( mProcessAggCF.SelectApuConfiguration(mApuConfig, mApexId))
         {
            return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
         }
      }
      
      if( mProcessAggCF.ConnectIO("INPUTL",   aSrcl)
           || mProcessAggCF.ConnectIO("INPUTU",   aSrcu)
           || mProcessAggCF.ConnectIO("INPUTV",   aSrcv)
           || mProcessAggCF.ConnectIO("PARAMS",   mParams)
           || mProcessAggCF.ConnectIO("OUTPUTM",  aMag)
           || mProcessAggCF.ConnectIO("OUTPUTO",  sOri))
      {
         return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      {
        ApexcvHostBaseBaseClass::InfoCluster lInfo;
        lInfo.set_ACF("AGGCF", (void*) &mProcessAggCF);
        lInfo.push_PortName("INPUTL");
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

APEXCV_LIB_RESULT AGGCFMagOri::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState) 
   {
      lRetVal = ComputeMagOri();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFMagOri::ComputeMagOri()
{
   int status = 0;
   if (mInitState) 
   {
      //printf(" aggcf_magori::computeMagOri() \n ");
      mProcessAggCF.FlushAndInvalidateAllIO();
      status  = mProcessAggCF.Start();
      status |= mProcessAggCF.Wait();

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

void AGGCFMagOri::Release()
{
   mParams = vsdk::SUMat();
}

AGGCFMagOri::AGGCFMagOri()
{
   mInitState = mImageWidth = mImageHeight = 0;

   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0;

   mSelectedApuConfiguration = 0;
}

AGGCFMagOri::~AGGCFMagOri()
{
   Release();
}
   
APEXCV_LIB_RESULT AGGCFMagOri::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

AGGCFMagOriRef::AGGCFMagOriRef()
{
}

AGGCFMagOriRef::~AGGCFMagOriRef()
{
}

int AGGCFMagOriRef::Process( 
           vsdk::SUMat& aSrcl,
           vsdk::SUMat& aSrcu,
           vsdk::SUMat& aSrcv,
           vsdk::SUMat& aMag,
           vsdk::SUMat& aOri,
           int )
{
   int width = aSrcl.cols, height = aSrcl.rows;

   vsdk::SMat srclMat = aSrcl.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *srclPtr = (uint16_t *)srclMat.data;

   vsdk::SMat srcuMat = aSrcu.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *srcuPtr = (uint16_t *)srcuMat.data;

   vsdk::SMat srcvMat = aSrcv.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *srcvPtr = (uint16_t *)srcvMat.data;

   vsdk::SMat magMat = aMag.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *magPtr = (uint16_t *)magMat.data;

   vsdk::SMat oriMat = aOri.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);     
   uint16_t *oriPtr = (uint16_t *)oriMat.data;

   int32_t *Gx  = new int32_t[3*width];
   int32_t *Gy  = new int32_t[3*width];
   uint32_t *M2 = new uint32_t[3*width];

   uint16_t* LPad = image_pad(srclPtr, width, height, 1, 1, 1, 1);
   uint16_t* UPad = image_pad(srcuPtr, width, height, 1, 1, 1, 1);
   uint16_t* VPad = image_pad(srcvPtr, width, height, 1, 1, 1, 1);

   int32_t inStride  = width + 2;
   int32_t outStride = width;

   uint16_t* LPadPtr = LPad + inStride + 1;
   uint16_t* UPadPtr = UPad + inStride + 1;
   uint16_t* VPadPtr = VPad + inStride + 1;

   float diff = 0;

   fxgradMag_c_row(LPadPtr, UPadPtr, VPadPtr, inStride, outStride, magPtr, oriPtr, height, width, Gx, Gy, M2, diff);

   delete[] LPad;
   delete[] UPad;
   delete[] VPad;

   delete[] Gx;
   delete[] Gy;
   delete[] M2;

   return 0; 

}

}


