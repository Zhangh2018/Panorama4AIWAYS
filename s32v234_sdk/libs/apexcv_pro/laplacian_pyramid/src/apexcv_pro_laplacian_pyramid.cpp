/**************************************************************************************************
 * 
 * NXP Confidential Proprietary
 *
 * Copyright 2016 NXP 
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
 **************************************************************************************************/

/*==================================================================================================
Revision History:
                              Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Adrian Grigore (nxa11567)     14/12/2016    VSDK-554      Created
---------------------------   ----------    ------------  ------------------------------------------
==================================================================================================*/
/*!****************************************************************************
*  \file laplacian.cpp
*  \brief ApexCV Host wrapper for laplacian pyramid creation algorithm
*******************************************************************************/
#include "apexcv_pro_laplacian_pyramid.h"

// GENERATED ACF PROCESS
#include <LAPLACIAN_PYRAMID_MID.hpp>
#include <LAPLACIAN_PYRAMID_LAST.hpp>

namespace apexcv {

LaplacianPyramid::LaplacianPyramid():mInit(0), mLaplacianMid(nullptr), mLaplacianLast(nullptr)
{
}

LaplacianPyramid::~LaplacianPyramid()
{
   if (nullptr != mLaplacianMid)
   {
      delete (LAPLACIAN_PYRAMID_MID*) mLaplacianMid;
   }
   mLaplacianMid = nullptr;

   if (nullptr != mLaplacianLast)
   {
      delete (LAPLACIAN_PYRAMID_LAST*) mLaplacianLast;
   }
   mLaplacianLast = nullptr;
}

APEXCV_LIB_RESULT LaplacianPyramid::Initialize(
            vsdk::SUMat&  aSrc,
            vsdk::SUMat&  aDstPyramid,
            vsdk::SUMat&  aDstAux,
            bool aIsLastLevel
)
{
   mIsLastLevel = aIsLastLevel;
   if (!mInit)
   {
      LAPLACIAN_PYRAMID_MID*  lLaplacianMid  = new LAPLACIAN_PYRAMID_MID;
      LAPLACIAN_PYRAMID_LAST* lLaplacianLast = new LAPLACIAN_PYRAMID_LAST;
      if (nullptr == lLaplacianMid||
          nullptr == lLaplacianLast)
      {
         return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mLaplacianMid  = lLaplacianMid;
      mLaplacianLast = lLaplacianLast;

      if (aIsLastLevel)
      {
         if (aSrc.type() == VSDK_CV_8UC1 &&
            aDstPyramid.type() == VSDK_CV_16SC1 &&
            aDstAux.type() == VSDK_CV_16SC1 )
         {
            if (lLaplacianLast->Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }
            if (lLaplacianLast->ConnectIO("INPUT",              aSrc) ||
                lLaplacianLast->ConnectIO("OUTPUT_PYRAMID",     aDstPyramid) ||
                lLaplacianLast->ConnectIO("OUTPUT_RECONSTRUCT", aDstAux))
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }
            {
              ApexcvHostBaseBaseClass::InfoCluster lInfo;
              lInfo.set_ACF("LAPLACIAN_PYRAMID_LAST", lLaplacianLast);
              lInfo.push_PortName("INPUT");
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
         if (aSrc.type() == VSDK_CV_8UC1 &&
            aDstPyramid.type() == VSDK_CV_16SC1 &&
            aDstAux.type() == VSDK_CV_8UC1 )
         {
            if (lLaplacianMid->Initialize())
            {
               return APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }
            if (lLaplacianMid->ConnectIO("INPUT",          aSrc) ||
                lLaplacianMid->ConnectIO("OUTPUT_PYRAMID", aDstPyramid) ||
                lLaplacianMid->ConnectIO("OUTPUT_NEXT",    aDstAux))
            {
               return APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }
            {
              ApexcvHostBaseBaseClass::InfoCluster lInfo;
              lInfo.set_ACF("LAPLACIAN_PYRAMID_MID", lLaplacianMid);
              lInfo.push_PortName("INPUT");
              mvInfoClusters.push_back(lInfo);
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
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }

   mInit = true;

   return APEXCV_SUCCESS;
}

APEXCV_LIB_RESULT LaplacianPyramid::ReconnectIO(
            vsdk::SUMat&  aSrc,
            vsdk::SUMat&  aDstPyramid,
            vsdk::SUMat&  aDstAux
)
{
   if (mInit)
   {

      if (mIsLastLevel)
      {
         if (aSrc.type() == VSDK_CV_8UC1 &&
            aDstPyramid.type() == VSDK_CV_16SC1 &&
            aDstAux.type() == VSDK_CV_16SC1 )
         {
             LAPLACIAN_PYRAMID_LAST* lLaplacianLast = (LAPLACIAN_PYRAMID_LAST*) mLaplacianLast;
            if (lLaplacianLast->ConnectIO("INPUT",              aSrc) ||
                lLaplacianLast->ConnectIO("OUTPUT_PYRAMID",     aDstPyramid) ||
                lLaplacianLast->ConnectIO("OUTPUT_RECONSTRUCT", aDstAux))
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
         if (aSrc.type() == VSDK_CV_8UC1 &&
            aDstPyramid.type() == VSDK_CV_16SC1 &&
            aDstAux.type() == VSDK_CV_8UC1 )
         {
            LAPLACIAN_PYRAMID_MID* lLaplacianMid = (LAPLACIAN_PYRAMID_MID*) mLaplacianMid;
            if (lLaplacianMid->ConnectIO("INPUT",          aSrc) ||
                lLaplacianMid->ConnectIO("OUTPUT_PYRAMID", aDstPyramid) ||
                lLaplacianMid->ConnectIO("OUTPUT_NEXT",    aDstAux))
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
   else
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

APEXCV_LIB_RESULT LaplacianPyramid::Process()
{
   if (mInit)
   {
      if (mIsLastLevel)
      {
         LAPLACIAN_PYRAMID_LAST* lLaplacianLast = (LAPLACIAN_PYRAMID_LAST*) mLaplacianLast;
         lLaplacianLast->Start();
         lLaplacianLast->Wait();
      }
      else
      {
         LAPLACIAN_PYRAMID_MID* lLaplacianMid = (LAPLACIAN_PYRAMID_MID*) mLaplacianMid;
         lLaplacianMid->Start();
         lLaplacianMid->Wait();
      }
   }
   else
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return APEXCV_SUCCESS;
}

/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT LaplacianPyramid::SelectApexCore(int32_t aApexId)
{
   if (mIsLastLevel)
   {
      LAPLACIAN_PYRAMID_LAST* lLaplacianLast = (LAPLACIAN_PYRAMID_LAST*) mLaplacianLast;
      if (lLaplacianLast->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }
   else
   {
      LAPLACIAN_PYRAMID_MID* lLaplacianMid = (LAPLACIAN_PYRAMID_MID*) mLaplacianMid;
      if (lLaplacianMid->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }

   mApexId = aApexId;
   return APEXCV_SUCCESS;
}

} /* namespace apex */
