/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2018 NXP
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
/*!****************************************************************************
*  \file pyramid_multi.cpp
*  \brief ApexCV Host wrapper for image pyramid_multi creation algorithm
*******************************************************************************/
#include <oal.h>
#include <stdio.h>
#include "apexcv_pro_pyramid_multi.h"
#include <apexcv_error_codes.hpp>

// GENERATED ACF PROCESS
#include <PYRAMID_MULTI_DOWN.hpp>

namespace apexcv {

PyramidMultiCreation::PyramidMultiCreation():mInitState(0),
      mProcessDownMulti0(nullptr),
      mProcessDownMulti1(nullptr),
      mApexId(0)
{
}

PyramidMultiCreation::~PyramidMultiCreation()
{
   if(nullptr != mProcessDownMulti0)
   {
      delete (PYRAMID_MULTI_DOWN *)mProcessDownMulti0;
   }
   if(nullptr != mProcessDownMulti1)
   {
      delete (PYRAMID_MULTI_DOWN *)mProcessDownMulti1;
   }
}


APEXCV_LIB_RESULT PyramidMultiCreation::Initialize(
            vsdk::SUMat&  aSrc   /*!< Source memory buffer. */,
            vsdk::SUMat (&aDstArray)[PYRAMID_LEVELS]   /*!< Destination memory buffer. */
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;
   if (!mInitState)
   {
      PYRAMID_MULTI_DOWN *pDownProcess0 = new PYRAMID_MULTI_DOWN;
      if( nullptr == pDownProcess0)
      {
         result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         goto EXIT;
      }
      mProcessDownMulti0 = pDownProcess0;

      PYRAMID_MULTI_DOWN *pDownProcess1 = new PYRAMID_MULTI_DOWN;
      if( nullptr == pDownProcess1)
      {
         result = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         goto EXIT;
      }
      mProcessDownMulti1 = pDownProcess1;

      if ((aSrc.type() == VSDK_CV_8UC1) && (aDstArray[0].type() == VSDK_CV_8UC1)
        && (aDstArray[1].type() == VSDK_CV_8UC1) && (aDstArray[2].type() == VSDK_CV_8UC1)
        && (aDstArray[3].type() == VSDK_CV_8UC1))
      {
         if ( pDownProcess0->Initialize() || pDownProcess1->Initialize())
         {
            result = APEXCV_ERROR_ACF_PROC_INITIALIZE;
            goto EXIT;
         }

         if( pDownProcess0->ConnectIO("INPUT",    aSrc) ||
             pDownProcess0->ConnectIO("OUTPUT_0", aDstArray[0]) ||
             pDownProcess0->ConnectIO("OUTPUT_1", aDstArray[1]) ||
             pDownProcess1->ConnectIO("INPUT",    aDstArray[1]) ||
             pDownProcess1->ConnectIO("OUTPUT_0", aDstArray[2]) ||
             pDownProcess1->ConnectIO("OUTPUT_1", aDstArray[3])  )
         {
             result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
             goto EXIT;
         }

         {
           ApexcvHostBaseBaseClass::InfoCluster lInfo;
           lInfo.set_ACF("PYRAMID_PRO_0", pDownProcess0);
           lInfo.push_PortName("INPUT");
           lInfo.push_PortName("OUTPUT_0");
           lInfo.push_PortName("OUTPUT_1");
           mvInfoClusters.push_back(lInfo);
         }
         {
           ApexcvHostBaseBaseClass::InfoCluster lInfo;
           lInfo.set_ACF("PYRAMID_PRO_1", pDownProcess1);
           lInfo.push_PortName("INPUT");
           lInfo.push_PortName("OUTPUT_0");
           lInfo.push_PortName("OUTPUT_1");
           mvInfoClusters.push_back(lInfo);
         }
         
         if (!result)
            mInitState = 1;
      }
      else
      {
         result = APEXCV_ERROR_INVALID_ARGUMENT;//TODO: add standardized apex cv error codes
         goto EXIT;
      }
   }
   else
   {
      result = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;//TODO: add standardized apex cv error codes
      goto EXIT;
   }
   EXIT:
   if((result != APEXCV_SUCCESS) && (result != APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED))
   {
     if(nullptr != mProcessDownMulti0)
     {
        delete (PYRAMID_MULTI_DOWN *)mProcessDownMulti0;
        mProcessDownMulti0=nullptr;
     }
     if(nullptr != mProcessDownMulti1)
     {
        delete (PYRAMID_MULTI_DOWN *)mProcessDownMulti1;
        mProcessDownMulti1=nullptr;
     }
   }

   return result;
}

APEXCV_LIB_RESULT PyramidMultiCreation::ReconnectIO(
            vsdk::SUMat&  aSrc /*!< Source memory buffer. */,
            vsdk::SUMat (&aDstArray)[PYRAMID_LEVELS]   /*!< Destination memory buffer. */
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;

   if (mInitState)
   {
      PYRAMID_MULTI_DOWN *pDownProcess0 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti0;
      PYRAMID_MULTI_DOWN *pDownProcess1 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti1;
      if ((aSrc.type() == VSDK_CV_8UC1) && (aDstArray[0].type() == VSDK_CV_8UC1)
        && (aDstArray[1].type() == VSDK_CV_8UC1) && (aDstArray[2].type() == VSDK_CV_8UC1)
        && (aDstArray[3].type() == VSDK_CV_8UC1))
      {
         if( pDownProcess0->ConnectIO("INPUT",    aSrc) ||
             pDownProcess0->ConnectIO("OUTPUT_0", aDstArray[0])||
             pDownProcess0->ConnectIO("OUTPUT_1", aDstArray[1])||
             pDownProcess1->ConnectIO("INPUT",    aDstArray[1])||
             pDownProcess1->ConnectIO("OUTPUT_0", aDstArray[2])||
             pDownProcess1->ConnectIO("OUTPUT_1", aDstArray[3]) )
         {
            result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
         }
      }
      else
      {
         result = APEXCV_ERROR_INVALID_ARGUMENT;
      }
   }
   else
   {
      result = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return result;
}

APEXCV_LIB_RESULT PyramidMultiCreation::Process()
{
   int status = APEXCV_SUCCESS;
   if (mInitState)
   {
       PYRAMID_MULTI_DOWN *pDownProcess0 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti0;
       PYRAMID_MULTI_DOWN *pDownProcess1 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti1;
       status |= pDownProcess0->Start();
       status |= pDownProcess0->Wait();
       if(status)
       {
          status = APEXCV_ERROR_ACF_PROC_EXEC;
          goto EXIT;
       }
       status |= pDownProcess1->Start();
       status |= pDownProcess1->Wait();
       if(status)
       {
          status = APEXCV_ERROR_ACF_PROC_EXEC;
          goto EXIT;
       }
   }
   else
   {
      status = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      goto EXIT;
   }
   EXIT:
   return status;
}


/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT PyramidMultiCreation::SelectApexCore(int32_t aApexId)
{
   int status = APEXCV_SUCCESS;
   if (!mInitState) 
   {
      status = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   else
   {
     PYRAMID_MULTI_DOWN *pDownProcess0 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti0;
     PYRAMID_MULTI_DOWN *pDownProcess1 = (PYRAMID_MULTI_DOWN *)mProcessDownMulti1;

     if (pDownProcess0->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId) ||
         pDownProcess1->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
     {
        status = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
        goto EXIT;
     }
     mApexId = aApexId;
   }
   EXIT:
   return status;
}

} /* namespace apex */
