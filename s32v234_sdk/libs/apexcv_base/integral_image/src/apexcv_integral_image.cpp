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
/*!*********************************************************************************
*  \file apexcv_integralImage.cpp
*  \brief Host-ACF interface for integral image
***********************************************************************************/


#include <apexcv_integral_image.h>
#include <sumat.hpp>

#ifndef APEX2_EMULATE
#include <acf_process_apu.h>
#include <INTEGRAL_IMAGE.hpp>
#else
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
//using namespace APEX2;
#include "integral_image_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(INTEGRAL_IMAGE, integral_image_apu_process_desc)

#endif

#include "graph_templates.h"


namespace apexcv{


  IntegralImage::~IntegralImage()
  {
    if (0 != mType){
      delete (INTEGRAL_IMAGE *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT IntegralImage::Initialize(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_32SC1)
    {

      INTEGRAL_IMAGE *pf = new INTEGRAL_IMAGE;
      if (NULL == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }

      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));

      int lRetVal = 0;
      lRetVal |= pf->Initialize();
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
      lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      mType = 1;
      return APEXCV_SUCCESS;
    }
    else{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT IntegralImage::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {

    switch (mType)
    {
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        INTEGRAL_IMAGE *pf = (INTEGRAL_IMAGE*)mpProcess;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
        lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
        if (0 != lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        return APEXCV_SUCCESS;
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
  }



}/* namespace */
