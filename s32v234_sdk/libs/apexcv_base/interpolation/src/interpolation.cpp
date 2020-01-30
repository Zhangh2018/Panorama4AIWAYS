/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016 NXP Semiconductor;
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

#include <apexcv_base_interpolation.h>
#include <stdint.h>
#include <sumat.hpp>

#ifdef APEX2_EMULATE
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include <acf_process_apu.h>
#endif


#include "all_interpol_graph_registrations.h"


namespace apexcv{


  InterpolationLinearGrayscale::~InterpolationLinearGrayscale()
  {
    if (0 != mType)
    {
      delete (INTERP_LINEAR_GRAYSCALE *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT InterpolationLinearGrayscale::Initialize(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDeltaX,
    vsdk::SUMat&    aDst)
  {

    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1 &&
      aDeltaX.type() == VSDK_CV_8UC1)
    {
      INTERP_LINEAR_GRAYSCALE *pf = new INTERP_LINEAR_GRAYSCALE;
      if (nullptr == pf)
      {
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
      lRetVal |= pf->ConnectIO("DELTA_X", aDeltaX);
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
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT InterpolationLinearGrayscale::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDeltaX,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1 &&
        aDeltaX.type() == VSDK_CV_8UC1)
      {
        INTERP_LINEAR_GRAYSCALE *pf = (INTERP_LINEAR_GRAYSCALE*)mpProcess;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
        lRetVal |= pf->ConnectIO("DELTA_X", aDeltaX);
        lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
        if (0 != lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
  }




  InterpolationBilinearGrayscale::~InterpolationBilinearGrayscale()
  {
    if (0 != mType)
    {
      delete (INTERP_BILINEAR_GRAYSCALE *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT InterpolationBilinearGrayscale::Initialize(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDelta,
    vsdk::SUMat&    aDst)
  {

    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1 &&
      aDelta.type() == VSDK_CV_8UC2)
    {
      INTERP_BILINEAR_GRAYSCALE *pf = new INTERP_BILINEAR_GRAYSCALE;
      if (NULL == pf)
      {
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
      lRetVal |= pf->ConnectIO(DELTA_XY, aDelta);
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
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT InterpolationBilinearGrayscale::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDelta,
    vsdk::SUMat&    aDst)
  {

    switch (mType)
    {
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1 &&
        aDelta.type() == VSDK_CV_8UC2)
      {
        INTERP_BILINEAR_GRAYSCALE *pf = (INTERP_BILINEAR_GRAYSCALE*)mpProcess;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
        lRetVal |= pf->ConnectIO(DELTA_XY, aDelta);
        lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
        if (0 != lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
  }






  InterpolationBicubicGrayscale::~InterpolationBicubicGrayscale()
  {
    if (0 != mType){
      delete (INTERP_BICUBIC_GRAYSCALE *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT InterpolationBicubicGrayscale::Initialize(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aOffsetX,
    vsdk::SUMat&    aOffsetY,
    vsdk::SUMat&    aDst)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1 &&
      aOffsetX.type() == VSDK_CV_8UC1 &&
      aOffsetY.type() == VSDK_CV_8UC1)
    {
      INTERP_BICUBIC_GRAYSCALE *pf = new INTERP_BICUBIC_GRAYSCALE;
      if (NULL == pf)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));

      int lRetVal = 0;
      lRetVal |= pf->Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
      lRetVal |= pf->ConnectIO(X_OFFSET, aOffsetX);
      lRetVal |= pf->ConnectIO(Y_OFFSET, aOffsetY);
      lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      mType = 1;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT InterpolationBicubicGrayscale::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aOffsetX,
    vsdk::SUMat&    aOffsetY,
    vsdk::SUMat&    aDst)
  {

    switch (mType)
    {
    case 1:
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1 &&
        aOffsetX.type() == VSDK_CV_8UC1 &&
        aOffsetY.type() == VSDK_CV_8UC1)
      {
        INTERP_BICUBIC_GRAYSCALE *pf = (INTERP_BICUBIC_GRAYSCALE*)mpProcess;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
        lRetVal |= pf->ConnectIO(X_OFFSET, aOffsetX);
        lRetVal |= pf->ConnectIO(Y_OFFSET, aOffsetY);
        lRetVal |= pf->ConnectIO(GR_OUT_0, aDst);
        if (0 != lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
    return APEXCV_SUCCESS;
  }


}  /* namespace apex */
