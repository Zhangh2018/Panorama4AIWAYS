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


APEXCV_LIB_RESULT AGGCFhist::InitBuffer(int aBin)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   // algorithm parameter check
   if( aBin < 1 || aBin > 4)
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   offsetRow = vsdk::SUMat(1, 128, VSDK_CV_32SC1);
  
   if(!offsetRow.empty())
   {
      vsdk::SMat offsetRowMat = offsetRow.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
     
      int32_t *ptr = (int32_t *)offsetRowMat.data;
      for(int i = 0; i < 128; i++)
      {
         ptr[i] = i/aBin;
      }
   }
   else
   {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFhist::InitParams(int aFull, int aNumOrientations, int aBin, int aSoftbin)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if( aFull < 0 || aFull > 1 || (aNumOrientations != 6 && aNumOrientations != 8) 
         || aSoftbin < 0 || aSoftbin > 1 || aBin!=2)
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }

   if ( ((0 == aSoftbin ) && (aBin < 1 || aBin > 4)) 
      || ((1 == aSoftbin ) && (aBin < 2 || aBin > 4)) ) 
   {
      return APEXCV_ERROR_INVALID_ARGUMENT;
   }  

   mParams = vsdk::SUMat(1, 4, VSDK_CV_32SC1);

   if(!mParams.empty())
   {
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(0u) = aFull;
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(1u) = aNumOrientations;
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(2u) = aBin;
      mParams.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).at<int32_t>(3u) = aSoftbin;

      mFullrange       = aFull;
      mNumOrientations = aNumOrientations;
      mBin             = aBin;
      mSoftbin         = aSoftbin;
   }
   else
   {
      lRetVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
   }
 
   return lRetVal;

}

APEXCV_LIB_RESULT AGGCFhist::SetParameters(int aFull, int aNumOrientations, int aBin, int aSoftbin) 
{
   APEXCV_LIB_RESULT lRetVal = 0;
   lRetVal |= InitParams(aFull, aNumOrientations, aBin, aSoftbin);
   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFhist::ReconnectIO(
       vsdk::SUMat &aSrcM,
       vsdk::SUMat &aSrcO,
       vsdk::SUMat &aDstH1,
       vsdk::SUMat &aDstH2,
       vsdk::SUMat &aDstH3,
       vsdk::SUMat &aDstH4,
       vsdk::SUMat &aDstH5,
       vsdk::SUMat &aDstH6,
       int aFull, int aNumOrientations, int aBin, int aSoftbin
)
 
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if(!mInitState)
   {
      lRetVal = Initialize(aSrcM, aSrcO, aDstH1, aDstH2, aDstH3, aDstH4, aDstH5, aDstH6, aFull, aNumOrientations, aBin, aSoftbin);
   }
   else
   { 
      mImageWidth  = aSrcM.cols;
      mImageHeight = aSrcM.rows;
 
      lRetVal = InitParams(aFull, aNumOrientations, aBin, aSoftbin);

      if( lRetVal != APEXCV_SUCCESS )
      {
         return lRetVal;
      }
  
      lRetVal = InitBuffer(aBin);
      if( APEXCV_SUCCESS != lRetVal)
      {
         return lRetVal;
      }
  

      if (0 == aSoftbin )
      {
         if( 1 == aBin)
         {
            if( mProcessHistnt1.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt1.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt1.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt1.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt1.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt1.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt1.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt1.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt1.ConnectIO("PARAMS",   mParams) )
             {
                return APEXCV_ERROR_ACF_PROC_CONNECTIO;  
             }    

         }
         else if(2 == aBin)
         {
            if( mProcessHistnt2.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt2.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt2.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt2.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt2.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt2.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt2.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt2.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt2.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }    

         }
         else if(3 == aBin)
         {
            if( mProcessHistnt3.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt3.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt3.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt3.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt3.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt3.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt3.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt3.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt3.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            } 
         }
         else if(4 == aBin)
         {
            if( mProcessHistnt4.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt4.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt4.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt4.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt4.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt4.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt4.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt4.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt4.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }
         }
         else
         {
            return APEXCV_ERROR_INVALID_ARGUMENT; 
         }   

      }
      else
      {
         if(2 == aBin)
         {
            if( mProcessHistti2.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti2.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti2.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti2.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti2.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti2.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti2.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti2.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti2.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO; 
            }  
         }
         else if(3 == aBin)
         {
            if( mProcessHistti3.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti3.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti3.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti3.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti3.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti3.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti3.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti3.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti3.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }  
         }
         else if(4 == aBin)
         {

            if( mProcessHistti4.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti4.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti4.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti4.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti4.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti4.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti4.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti4.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti4.ConnectIO("PARAMS",   mParams) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }   
         }
         else
         {
            return APEXCV_ERROR_INVALID_ARGUMENT; 
         }
      }    
   }

   return lRetVal;
}

APEXCV_LIB_RESULT AGGCFhist::Initialize(
               vsdk::SUMat &aSrcM,
               vsdk::SUMat &aSrcO,
               vsdk::SUMat &aDstH1,
               vsdk::SUMat &aDstH2,
               vsdk::SUMat &aDstH3,
               vsdk::SUMat &aDstH4,
               vsdk::SUMat &aDstH5,
               vsdk::SUMat &aDstH6,
               int aFull, int aNumOrientations, int aBin, int aSoftbin
)
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;

   if (!mInitState) 
   {
      // Keep the image size info
      mImageWidth  = aSrcM.cols;
      mImageHeight = aSrcM.rows;    

      lRetVal = InitParams(aFull, aNumOrientations, aBin, aSoftbin);

      if( APEXCV_SUCCESS != lRetVal )
      {
         return lRetVal;
      } 

      lRetVal = InitBuffer(aBin);
      if( APEXCV_SUCCESS != lRetVal)
      {
         return lRetVal;
      }

      if (0 == aSoftbin) 
      { 
         if( 1 == aBin)
         { 
            if(mProcessHistnt1.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE; 
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistnt1.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;  
               } 
            }
         
            if( mProcessHistnt1.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt1.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt1.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt1.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt1.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt1.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt1.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt1.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt1.ConnectIO("PARAMS",   mParams)
                || mProcessHistnt1.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_NT1", (void*) &mProcessHistnt1);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else if( 2 == aBin)
         {
            if(mProcessHistnt2.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistnt2.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistnt2.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt2.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt2.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt2.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt2.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt2.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt2.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt2.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt2.ConnectIO("PARAMS",   mParams)
                || mProcessHistnt2.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_NT2", (void*) &mProcessHistnt2);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else if (3 == aBin)
         {
            if(mProcessHistnt3.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistnt3.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistnt3.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt3.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt3.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt3.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt3.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt3.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt3.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt3.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt3.ConnectIO("PARAMS",   mParams)
                || mProcessHistnt3.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_NT3", (void*) &mProcessHistnt3);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else if (4 == aBin)
         {
            if(mProcessHistnt4.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistnt4.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistnt4.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistnt4.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistnt4.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistnt4.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistnt4.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistnt4.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistnt4.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistnt4.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistnt4.ConnectIO("PARAMS",   mParams)
                || mProcessHistnt4.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_NT4", (void*) &mProcessHistnt4);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else
         {
            return APEXCV_ERROR_INVALID_ARGUMENT; 
         }
      } 
      else
      {
         if( 2 == aBin)
         {
            if(mProcessHistti2.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistti2.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistti2.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti2.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti2.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti2.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti2.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti2.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti2.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti2.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti2.ConnectIO("PARAMS",   mParams)
                || mProcessHistti2.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_TI2", (void*) &mProcessHistti2);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else if (3 == aBin)
         {
            if(mProcessHistti3.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistti3.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistti3.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti3.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti3.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti3.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti3.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti3.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti3.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti3.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti3.ConnectIO("PARAMS",   mParams)
                || mProcessHistti3.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_TI3", (void*) &mProcessHistti3);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }
         }
         else if (4 == aBin)
         {
            if(mProcessHistti4.Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }

            if(1 == mSelectedApuConfiguration)
            {
               if(mProcessHistti4.SelectApuConfiguration(mApuConfig, mApexId))
               {
                  return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
               }
            }

            if( mProcessHistti4.ConnectIO("INPUTM",   aSrcM)
                || mProcessHistti4.ConnectIO("INPUTO",   aSrcO)
                || mProcessHistti4.ConnectIO("OUTPUTH1", aDstH1)
                || mProcessHistti4.ConnectIO("OUTPUTH2", aDstH2)
                || mProcessHistti4.ConnectIO("OUTPUTH3", aDstH3)
                || mProcessHistti4.ConnectIO("OUTPUTH4", aDstH4)
                || mProcessHistti4.ConnectIO("OUTPUTH5", aDstH5)
                || mProcessHistti4.ConnectIO("OUTPUTH6", aDstH6)
                || mProcessHistti4.ConnectIO("PARAMS",   mParams)
                || mProcessHistti4.ConnectIO("OFFSET",   offsetRow) )
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }

            {
               ApexcvHostBaseBaseClass::InfoCluster lInfo;
               lInfo.set_ACF("AGGCF_HIST_TI4", (void*) &mProcessHistti4);
               lInfo.push_PortName("INPUTM");
               mvInfoClusters.push_back(lInfo);
            }

         }
         else 
         {
            return APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }

      mInitState = 1;
   }
   else
   {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED; 
   }

   return lRetVal; 
}

APEXCV_LIB_RESULT AGGCFhist::Process()
{
   APEXCV_LIB_RESULT lRetVal = APEXCV_SUCCESS;
   if (mInitState) 
   {
      lRetVal = ComputeHist();
   }
   else
   {
      lRetVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return lRetVal;
}


APEXCV_LIB_RESULT AGGCFhist::ComputeHist()
{
   int status = 0;
   if (mInitState) 
   {
      if (0 == mSoftbin)
      {
         if(1 == mBin)
         {
            mProcessHistnt1.FlushAndInvalidateAllIO(); 
            status  = mProcessHistnt1.Start();
            status |= mProcessHistnt1.Wait();

            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;  
            }       
         }
         else if(2 == mBin)
         {
            mProcessHistnt2.FlushAndInvalidateAllIO();
            status  = mProcessHistnt2.Start();
            status |= mProcessHistnt2.Wait();

            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else if (3 == mBin)
         {
            mProcessHistnt3.FlushAndInvalidateAllIO(); 
            status  = mProcessHistnt3.Start();
            status |= mProcessHistnt3.Wait();
            
            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else if(4 == mBin)
         {
            mProcessHistnt4.FlushAndInvalidateAllIO();
            status  = mProcessHistnt4.Start();
            status |= mProcessHistnt4.Wait();
            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else
         {
            return APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }
      else
      {
         if(2 == mBin)
         {
            mProcessHistti2.FlushAndInvalidateAllIO();
            status  = mProcessHistti2.Start();
            status |= mProcessHistti2.Wait();
            
            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else if(3 == mBin)
         {
            mProcessHistti3.FlushAndInvalidateAllIO();
            status  = mProcessHistti3.Start();
            status |= mProcessHistti3.Wait();
            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else if (4 == mBin)
         {
            mProcessHistti4.FlushAndInvalidateAllIO();
            status  = mProcessHistti4.Start();
            status |= mProcessHistti4.Wait();

            if(status)
            {
               return APEXCV_ERROR_ACF_PROC_EXEC;
            }
         }
         else
         {
            return APEXCV_ERROR_INVALID_ARGUMENT;   
         }
      }

   } 
   else 
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

void AGGCFhist::Release()
{
   offsetRow = vsdk::SUMat();
   mParams = vsdk::SUMat();

}

AGGCFhist::AGGCFhist()
{
   mInitState = mImageWidth = mImageHeight = 0;

   mApuConfig = ACF_APU_CFG__DEFAULT;

   mApexId = 0;

   mSelectedApuConfiguration = 0;
}

AGGCFhist::~AGGCFhist()
{
   Release();
}

APEXCV_LIB_RESULT AGGCFhist::SetApuConfiguration(ACF_APU_CFG aApuConfig, int32_t aApexId)
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

AGGCFhistRef::AGGCFhistRef()
{

}

AGGCFhistRef::~AGGCFhistRef()
{


}

int AGGCFhistRef::Process(
               vsdk::SUMat &aSrcM,
               vsdk::SUMat &aSrcO,
               vsdk::SUMat &aDstH1,
               vsdk::SUMat &aDstH2,
               vsdk::SUMat &aDstH3,
               vsdk::SUMat &aDstH4,
               vsdk::SUMat &aDstH5,
               vsdk::SUMat &aDstH6,
               int aFull, int aNumOrientations, int aBin, int aSoftbin
                 )

{
    int width = aSrcM.cols, height = aSrcM.rows;
    vsdk::SMat srcMMat = aSrcM.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *srcMPtr = (uint16_t *)srcMMat.data;

    vsdk::SMat srcOMat = aSrcO.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *srcOPtr = (uint16_t *)srcOMat.data;

    vsdk::SMat bin1Mat = aDstH1.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin1Ptr = (uint16_t *)bin1Mat.data;

    vsdk::SMat bin2Mat = aDstH2.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin2Ptr = (uint16_t *)bin2Mat.data;

    vsdk::SMat bin3Mat = aDstH3.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin3Ptr = (uint16_t *)bin3Mat.data;

    vsdk::SMat bin4Mat = aDstH4.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin4Ptr = (uint16_t *)bin4Mat.data;

    vsdk::SMat bin5Mat = aDstH5.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin5Ptr = (uint16_t *)bin5Mat.data;

    vsdk::SMat bin6Mat = aDstH6.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED);
    uint16_t *bin6Ptr = (uint16_t *)bin6Mat.data;

    uint32_t *M0 = new uint32_t[width+aBin];
    uint32_t *M1 = new uint32_t[width+aBin];
    uint32_t *O0 = new uint32_t[width+aBin];
    uint32_t *O1 = new uint32_t[width+aBin];

    uint16_t* mPad = image_pad(srcMPtr, width, height, aBin, 0, aBin, 0);
    uint16_t* oPad = image_pad(srcOPtr, width, height, aBin, 0, aBin, 0);

    int32_t inStride  = width + aBin;
    int32_t outStride = width/aBin;

    uint16_t* mPadPtr = mPad + inStride*aBin + aBin;
    uint16_t* oPadPtr = oPad + inStride*aBin + aBin;

    fxgradHist_c_row_trilinear(mPadPtr, oPadPtr, bin1Ptr, bin2Ptr, bin3Ptr, bin4Ptr, bin5Ptr, bin6Ptr,
        height, width, inStride, outStride, width / aBin, height / aBin, (width / aBin*height / aBin), aBin, aNumOrientations, aSoftbin, aFull, M0, M1, O0, O1);

    delete[] mPad;
    delete[] oPad;
    delete[] M0;
    delete[] M1;
    delete[] O0;
    delete[] O1;
    return 0;
}

}


