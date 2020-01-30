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
/*!****************************************************************************
*  \file ApexCV_pyramid.cpp
*  \brief ApexCV Host wrapper for image pyramid creation algorithm
*******************************************************************************/
#include "apexcv_pro_pyramid.h"
#include <apexcv_error_codes.hpp>

// GENERATED ACF PROCESS
#include <PYRAMID_UP.hpp>
#include <PYRAMID_DOWN.hpp>


namespace apexcv {

PyramidCreation::PyramidCreation():mInitState(0),mpProcessUp(nullptr),mpProcessDown(nullptr),mApexId(0),mIsPyramidUp(true)
{
}

PyramidCreation::~PyramidCreation()
{
   if(nullptr != mpProcessUp)
   {
      delete (PYRAMID_UP*)mpProcessUp;
   }
   if(nullptr != mpProcessDown)
   {
      delete (PYRAMID_DOWN*)mpProcessDown;
   }
}

APEXCV_LIB_RESULT PyramidCreation::Initialize(
            vsdk::SUMat&  aSrc /*!< Source memory buffer. */,
            vsdk::SUMat&  aDst /*!< Destination memory buffer. */
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;

   if (!mInitState)
   {
      if ((aSrc.cols * 2 == aDst.cols) && (aSrc.rows * 2 == aDst.rows))
      {
         PYRAMID_UP *pUpProcess = new PYRAMID_UP;
         if( nullptr == pUpProcess)
         {
            return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }
         mpProcessUp = pUpProcess;

         if (aSrc.type() == VSDK_CV_8UC1 &&
         aDst.type() == VSDK_CV_8UC1)
         {
            if( pUpProcess->Initialize())
            {
              result = APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }  
            else
            { 
               if( pUpProcess->ConnectIO("INPUT",   aSrc) ||
                   pUpProcess->ConnectIO("OUTPUT",  aDst))
               {
                   result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
               } 
               else
               {  
                   mInitState = 1;
                   {
                      ApexcvHostBaseBaseClass::InfoCluster lInfo;
                      lInfo.set_ACF("PYRAMID_UP", pUpProcess);
                      lInfo.push_PortName("INPUT");
                      mvInfoClusters.push_back(lInfo);
                   }
                   mIsPyramidUp = true;
               }
            }
         }
         else
         {
            result = APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }
      else if ((aSrc.cols == aDst.cols * 2) && (aSrc.rows == aDst.rows * 2))
      {
         PYRAMID_DOWN *pDownProcess = new PYRAMID_DOWN;
         if( nullptr == pDownProcess)
         {
            return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
         }
         mpProcessDown = pDownProcess;

         if (aSrc.type() == VSDK_CV_8UC1 &&
         aDst.type() == VSDK_CV_8UC1)
         {
            if( pDownProcess->Initialize())
            {
              result = APEXCV_ERROR_ACF_PROC_INITIALIZE;
            }  
            else
            { 
               if( pDownProcess->ConnectIO("INPUT",    aSrc) ||
                   pDownProcess->ConnectIO("OUTPUT",   aDst))
               {
                   result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
               } 
               else
               {  
                   mInitState = 1;
                   {
                      ApexcvHostBaseBaseClass::InfoCluster lInfo;
                      lInfo.set_ACF("PYRAMID_DOWN", pDownProcess);
                      lInfo.push_PortName("INPUT");
                      mvInfoClusters.push_back(lInfo);
                   }
                   mIsPyramidUp = false;
               }
            }
         }
         else
         {
            result = APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }
      else
      {
         result = APEXCV_ERROR_INVALID_ARGUMENT;
      }
   }
   else
   {
      result = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
   }
   return result;
}

APEXCV_LIB_RESULT PyramidCreation::ReconnectIO(
            vsdk::SUMat&  aSrc /*!< Source memory buffer. */,
            vsdk::SUMat&  aDst /*!< Destination memory buffer. */
)
{
   APEXCV_LIB_RESULT result = APEXCV_SUCCESS;

   if (mInitState)
   {
      if(mIsPyramidUp)
      {
         PYRAMID_UP *pUpProcess = (PYRAMID_UP *)mpProcessUp;
         if (aSrc.type() == VSDK_CV_8UC1 &&
         aDst.type() == VSDK_CV_8UC1)
         {
            if( pUpProcess->ConnectIO("INPUT",      aSrc) ||
                pUpProcess->ConnectIO("OUTPUT",     aDst))
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
         PYRAMID_DOWN *pDownProcess = (PYRAMID_DOWN *)mpProcessDown; 
         if (aSrc.type() == VSDK_CV_8UC1 &&
             aDst.type() == VSDK_CV_8UC1)
         {
            if( pDownProcess->ConnectIO("INPUT",    aSrc) ||
                pDownProcess->ConnectIO("OUTPUT",   aDst))
            {
               result = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            } 
         }
         else
         {
            result = APEXCV_ERROR_INVALID_ARGUMENT;
         }
      }
   }
   else
   {
      result = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }

   return result;
}

APEXCV_LIB_RESULT PyramidCreation::Process()
{ 
   int status = 0;
   if (mInitState)
   {
      if (mIsPyramidUp)
      {
         PYRAMID_UP *pUpProcess = (PYRAMID_UP *)mpProcessUp;
         if(nullptr != mpProcessUp)
         {
            status |= pUpProcess->Start();
            status |= pUpProcess->Wait();
         }
         else
         {
            status = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
         }
      }
      else
      {  
         PYRAMID_DOWN *pDownProcess = (PYRAMID_DOWN *)mpProcessDown;
         if(nullptr != mpProcessDown)
         {
            status |= pDownProcess->Start();
            status |= pDownProcess->Wait();
         }
         else
         {
            status = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
         }
      }
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

/************************************
 * Select APEX to run
 ***********************************/
APEXCV_LIB_RESULT PyramidCreation::SelectApexCore(int32_t aApexId)
{
   if (!mInitState)
   {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
   }
   if(mIsPyramidUp)
   {
      PYRAMID_UP *pUpProcess = (PYRAMID_UP *)mpProcessUp;

      if (pUpProcess->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }
   else
   {
      PYRAMID_DOWN *pDownProcess = (PYRAMID_DOWN *)mpProcessDown;

      if (pDownProcess->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId))
      {
         return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
      }
   }
   mApexId = aApexId;
   return APEXCV_SUCCESS;
}

} /* namespace apex */
