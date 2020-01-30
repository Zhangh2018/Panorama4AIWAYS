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

#include <apexcv_base_arithmetic.h>
#include <apexcv_arithmetic_int.h>
#include <stdint.h>

#include "all_arithmetic_graph_registrations.h"


namespace apexcv
{


  Abs::~Abs()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<ABS_08U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT Abs::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<ABS_08U>*pf = (apexcv_fct<ABS_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Abs::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8SC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<ABS_08U, Abs>(*this, 1, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  AbsDiff::~AbsDiff()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<ABSDIFF_08U>*)mpProcess;
      }
      break;

      case 2:
      {
        delete (apexcv_fct<ABSDIFF_16S>*)mpProcess;
      }
      break;
    }
  }


  APEXCV_LIB_RESULT AbsDiff::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<ABSDIFF_08U>*pf = (apexcv_fct<ABSDIFF_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;

    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ABSDIFF_16S>*pf = (apexcv_fct<ABSDIFF_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT AbsDiff::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<ABSDIFF_08U, AbsDiff>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        return ApexCVHostUtilAri::WInitialize<ABSDIFF_16S, AbsDiff>(*this, 2, aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
  }


  Accumulate::~Accumulate()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<ACCUMULATE_16S>*)mpProcess;
      }
      break;

      default:
      break;
    }
  }


  APEXCV_LIB_RESULT Accumulate::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ACCUMULATE_16S>*pf = (apexcv_fct<ACCUMULATE_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Accumulate::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<ACCUMULATE_16S, Accumulate>(*this, 1, aSrc, aDst, aDst);
    }  
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  AccumulateSquared::~AccumulateSquared()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<ACCUMULATE_SQUARED_16S>*)mpProcess;
      }
      default:
      break;
    }
  }


  APEXCV_LIB_RESULT AccumulateSquared::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ACCUMULATE_SQUARED_16S>*pf = (apexcv_fct<ACCUMULATE_SQUARED_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }

  APEXCV_LIB_RESULT AccumulateSquared::SetScale(
    uint8_t    aScale)
  {
    if (mScale.empty())
    {
      mScale = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mScale.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<uint8_t>(0u) = aScale;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT AccumulateSquared::GetScale(
    uint8_t&    aScale)
  {
    if (mScale.empty())
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aScale = mat.at<uint8_t>(0u);
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT AccumulateSquared::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    uint8_t    aScale /*!< Scale amount. (0 <= scale <= 15) */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aScale < 0 || aScale > 15)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetScale(aScale);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      return ApexCVHostUtilAri::WInitializeAlpha<ACCUMULATE_SQUARED_16S, AccumulateSquared>(*this, 1, aSrc, aDst, mScale, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  AccumulateWeighted::~AccumulateWeighted()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<ACCUMULATE_WEIGHTED_08U>*)mpProcess;
      }
      break;
      default:
      break;
    }
  }


  APEXCV_LIB_RESULT AccumulateWeighted::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<ACCUMULATE_WEIGHTED_08U>*pf = (apexcv_fct<ACCUMULATE_WEIGHTED_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }

  APEXCV_LIB_RESULT AccumulateWeighted::SetAlpha(
    const float    acAlpha)
  {
    if (mAlpha.empty())
    {
      mAlpha = vsdk::SUMat(1, 1, VSDK_CV_16UC1);
      if (mAlpha.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mAlpha.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<uint16_t>(0u) = (static_cast<uint16_t>(acAlpha*512.0f)+1)>> 1;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT AccumulateWeighted::GetAlpha(
    float&    aAlpha)
  {
    if (mAlpha.empty())
    {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    vsdk::SMat mat = mAlpha.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aAlpha = mat.at<uint16_t>(0u);
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT AccumulateWeighted::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    float    aAlpha /*!< Weight amount. 8 bit fixed point value of [0, 1) */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aAlpha < 0 || aAlpha > 1)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetAlpha(aAlpha);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      return ApexCVHostUtilAri::WInitializeAlpha<ACCUMULATE_WEIGHTED_08U, AccumulateWeighted>(*this, 1, aSrc, aDst, mAlpha, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  Add::~Add()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<ADD_08U>*)mpProcess;
      }break;

      case 2:
      {
        delete (apexcv_fct<ADD_08U_08U_16S>*)mpProcess;
      }break;

      case 3:
      {
        delete (apexcv_fct<ADD_08U_16S_16S>*)mpProcess;
      }break;

      case 4:
      {
        delete (apexcv_fct<ADD_16S>*)mpProcess;
      }break;
      default:
      break;
    }
  }


  APEXCV_LIB_RESULT Add::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<ADD_08U>*pf = (apexcv_fct<ADD_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 2:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ADD_08U_08U_16S>*pf = (apexcv_fct<ADD_08U_08U_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ADD_08U_16S_16S>*pf = (apexcv_fct<ADD_08U_16S_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ADD_16S>*pf = (apexcv_fct<ADD_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 5:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<ADD_08U_16S_16S>*pf = (apexcv_fct<ADD_08U_16S_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc2, aSrc1, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }

  APEXCV_LIB_RESULT Add::SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< shift value. */)
  {
    if (mPolicy.empty())
    {
      mPolicy = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    }

    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    mat.at<uint8_t>(0u) = (uint8_t)aPolicy;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Add::GetPolicy(
    apexcv::eConvertPolicy      &aPolicy /*!< shift value. */)
  {
    if (mPolicy.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    aPolicy = (apexcv::eConvertPolicy) mat.at<uint8_t>(0u);
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Add::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    APEXCV_LIB_RESULT lRetVal;

    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    lRetVal = SetPolicy(apexcv::eConvertPolicy::SATURATE);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<ADD_08U, Add>(*this, 1, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<ADD_08U_08U_16S, Add>(*this, 2, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<ADD_08U_16S_16S, Add>(*this, 3, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<ADD_16S, Add>(*this, 4, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<ADD_08U_16S_16S, Add>(*this, 5, aSrc2, aSrc1, mPolicy, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  BitwiseAND::~BitwiseAND()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<AND_08U>*)mpProcess;
    }
    break;

    case 2:
    {
      delete (apexcv_fct<AND_16U>*)mpProcess;
    }
    break;

    case 3:
    {
      delete (apexcv_fct<AND_32U>*)mpProcess;
    }
    break;
    }
  }


  APEXCV_LIB_RESULT BitwiseAND::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<AND_08U>*pf = (apexcv_fct<AND_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<AND_16U>*pf = (apexcv_fct<AND_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc1.type() == VSDK_CV_32SC1 &&
        aSrc2.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        apexcv_fct<AND_32U>*pf = (apexcv_fct<AND_32U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT BitwiseAND::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<AND_08U, BitwiseAND>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        return ApexCVHostUtilAri::WInitialize<AND_16U, BitwiseAND>(*this, 2, aSrc1, aSrc2, aDst);
      }
      else
      {
        if (aSrc1.type() == VSDK_CV_32SC1 &&
          aSrc2.type() == VSDK_CV_32SC1 &&
          aDst.type() == VSDK_CV_32SC1)
        {
          return ApexCVHostUtilAri::WInitialize<AND_32U, BitwiseAND>(*this, 3, aSrc1, aSrc2, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }






  Clz::~Clz()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<CLZ_08U>*)mpProcess;
    }
    break;

    case 2:
    {
      delete (apexcv_fct<CLZ_08S>*)mpProcess;
    }
    break;

    case 3:
    {
      delete (apexcv_fct<CLZ_16U>*)mpProcess;
    }
    break;

    case 4:
    {
      delete (apexcv_fct<CLZ_16S>*)mpProcess;
    }
    break;
    }
  }


  APEXCV_LIB_RESULT Clz::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<CLZ_08U>*pf = (apexcv_fct<CLZ_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc.type() == VSDK_CV_8SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<CLZ_08S>*pf = (apexcv_fct<CLZ_08S>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<CLZ_16U>*pf = (apexcv_fct<CLZ_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 4:
    {
      if (aSrc.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<CLZ_16S>*pf = (apexcv_fct<CLZ_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Clz::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<CLZ_08U, Clz>(*this, 1, aSrc, aDst);
    }
    else
    {
      if (aSrc.type() == VSDK_CV_8SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilAri::WInitialize<CLZ_08S, Clz>(*this, 2, aSrc, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_16UC1 &&
          aDst.type() == VSDK_CV_8UC1)
        {
          return ApexCVHostUtilAri::WInitialize<CLZ_16U, Clz>(*this, 3, aSrc, aDst);
        }
        else
        {
          if (aSrc.type() == VSDK_CV_16SC1 &&
            aDst.type() == VSDK_CV_8UC1)
          {
            return ApexCVHostUtilAri::WInitialize<CLZ_16S, Clz>(*this, 4, aSrc, aDst);
          }
          else
          {
            return APEXCV_ERROR_INVALID_ARGUMENT;
          }
        }
      }
    }
  }






  Magnitude::~Magnitude()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<MAGNITUDE_16S>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT Magnitude::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<MAGNITUDE_16S>*pf = (apexcv_fct<MAGNITUDE_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Magnitude::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_16SC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16UC1)
    {
      return ApexCVHostUtilAri::WInitialize<MAGNITUDE_16S, Magnitude>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  Max::~Max()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<MAX_08U>*)mpProcess;
    }break;
    case 2:
    {
      delete (apexcv_fct<MAX_16S>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT Max::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MAX_08U>*pf = (apexcv_fct<MAX_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<MAX_16S>*pf = (apexcv_fct<MAX_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Max::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<MAX_08U, Max>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<MAX_16S, Max>(*this, 2, aSrc1, aSrc2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  Min::~Min()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<MIN_08U>*)mpProcess;
    }break;
    case 2:
    {
      delete (apexcv_fct<MIN_16S>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT Min::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MIN_08U>*pf = (apexcv_fct<MIN_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<MIN_16S>*pf = (apexcv_fct<MIN_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Min::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<MIN_08U, Min>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<MIN_16S, Min>(*this, 2, aSrc1, aSrc2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  Mul::~Mul()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<MUL_08U>*)mpProcess;
      }break;
      case 2:
      {
        delete (apexcv_fct<MUL_08U_08U_16S>*)mpProcess;
      }break;
      case 3:
      {
        delete (apexcv_fct<MUL_08U_16S_16S>*)mpProcess;
      }break;
      case 4:
      {
        delete (apexcv_fct<MUL_16S>*)mpProcess;
      }break;
      default:
      break;
    }
  }


  APEXCV_LIB_RESULT Mul::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MUL_08U>*pf = (apexcv_fct<MUL_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    case 2:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<MUL_08U_08U_16S>*pf = (apexcv_fct<MUL_08U_08U_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    case 3:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<MUL_08U_16S_16S>*pf = (apexcv_fct<MUL_08U_16S_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    case 4:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<MUL_16S>*pf = (apexcv_fct<MUL_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Mul::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    APEXCV_LIB_RESULT lRetVal;
    const uint8_t lInitScl = 0U;
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    // Set default value of scale
    lRetVal = SetScale(lInitScl);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    // Set policy type
    lRetVal = SetPolicy(apexcv::eConvertPolicy::WRAP);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<MUL_08U, Mul>(*this, 1, aSrc1, aSrc2, mScale, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<MUL_08U_08U_16S, Mul>(*this, 2, aSrc1, aSrc2, mScale, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<MUL_08U_16S_16S, Mul>(*this, 3, aSrc1, aSrc2, mScale, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<MUL_16S, Mul>(*this, 4, aSrc1, aSrc2, mScale, mPolicy, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT Mul::SetScale(
    const uint8_t    acScale /*!< scale value. */)
  {
    if(((acScale<0) || (acScale>15)) && (acScale != 255))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (mScale.empty())
    {
      mScale = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    }

    /* noncached as it is for a single value */
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    mat.at<uint8_t>(0u) = (uint8_t)acScale;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Mul::GetScale(
    uint8_t      &aScale /*!< scale value. */)
  {
    if (mScale.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    /* noncached as it is for a single value */
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    aScale = mat.at<uint8_t>(0u);
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Mul::SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< policy value. */)
  {
    if (mPolicy.empty())
    {
      mPolicy = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    }

    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    uint8_t lVal;
    if (apexcv::eConvertPolicy::SATURATE == aPolicy)
    {
      lVal = 255U;
    }
    else
    {
      lVal = 0U;
    }
    mat.at<uint8_t>(0u) = lVal;

    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT Mul::GetPolicy(
    apexcv::eConvertPolicy      &aPolicy /*!< policy value. */)
  {
    if (mPolicy.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    uint8_t lVal = mat.at<uint8_t>(0u);
    if (0 != lVal)
    {
      aPolicy = apexcv::eConvertPolicy::SATURATE;
    }
    else
    {
      aPolicy = apexcv::eConvertPolicy::WRAP;
    }
    return APEXCV_SUCCESS;
  }

  BitwiseNOT::~BitwiseNOT()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<NOT_08U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT BitwiseNOT::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<NOT_08U>*pf = (apexcv_fct<NOT_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return 1;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT BitwiseNOT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<NOT_08U, BitwiseNOT>(*this, 1, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  BitwiseOR::~BitwiseOR()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<OR_08U>*)mpProcess;
    }break;

    case 2:
    {
      delete (apexcv_fct<OR_16U>*)mpProcess;
    }break;

    case 3:
    {
      delete (apexcv_fct<OR_32U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT BitwiseOR::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<OR_08U>*pf = (apexcv_fct<OR_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<OR_16U>*pf = (apexcv_fct<OR_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc1.type() == VSDK_CV_32SC1 &&
        aSrc2.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        apexcv_fct<OR_32U>*pf = (apexcv_fct<OR_32U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT BitwiseOR::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<OR_08U, BitwiseOR>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        return ApexCVHostUtilAri::WInitialize<OR_16U, BitwiseOR>(*this, 2, aSrc1, aSrc2, aDst);
      }
      else
      {
        if (aSrc1.type() == VSDK_CV_32SC1 &&
          aSrc2.type() == VSDK_CV_32SC1 &&
          aDst.type() == VSDK_CV_32SC1)
        {
          return ApexCVHostUtilAri::WInitialize<OR_32U, BitwiseOR>(*this, 3, aSrc1, aSrc2, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }


  Phase::~Phase()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<PHASE_08U>*)mpProcess;
      }break;
    }
  }


  APEXCV_LIB_RESULT Phase::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<PHASE_08U>*pf = (apexcv_fct<PHASE_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Phase::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_16SC1 &&
      aSrc2.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<PHASE_08U, Phase>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  Subtract::~Subtract()
  {
    switch (mType)
    {
      case 1:
      {
        delete (apexcv_fct<SUB_08U>*)mpProcess;
      }break;

      case 2:
      {
        delete (apexcv_fct<SUB_08U_08U_16S>*)mpProcess;
      }break;

      case 3:
      {
        delete (apexcv_fct<SUB_08U_16S_16S>*)mpProcess;
      }break;

      case 4:
      {
        delete (apexcv_fct<SUB_16S>*)mpProcess;
      }break;

      case 5:
      {
        delete (apexcv_fct<SUB_16S_08U_16S>*)mpProcess;
      }
      break;
      default:
      break;
    }
  }


  APEXCV_LIB_RESULT Subtract::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<SUB_08U>*pf = (apexcv_fct<SUB_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<SUB_08U_08U_16S>*pf = (apexcv_fct<SUB_08U_08U_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<SUB_08U_16S_16S>*pf = (apexcv_fct<SUB_08U_16S_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 4:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<SUB_16S>*pf = (apexcv_fct<SUB_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 5:
    {
      if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<SUB_16S_08U_16S>*pf = (apexcv_fct<SUB_16S_08U_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }

  APEXCV_LIB_RESULT Subtract::SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< shift value. */)
  {
    if (mPolicy.empty())
    {
      mPolicy = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    }

    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    mat.at<uint8_t>(0u) = (uint8_t)aPolicy;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Subtract::GetPolicy(
    apexcv::eConvertPolicy      &aPolicy /*!< shift value. */)
  {
    if (mPolicy.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    /* noncached as it is for a single value */
    vsdk::SMat mat = mPolicy.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    aPolicy = (apexcv::eConvertPolicy) mat.at<uint8_t>(0u);
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT Subtract::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    APEXCV_LIB_RESULT lRetVal;

    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    lRetVal = SetPolicy(apexcv::eConvertPolicy::SATURATE);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<SUB_08U, Subtract>(*this, 1, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<SUB_08U_08U_16S, Subtract>(*this, 2, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<SUB_08U_16S_16S, Subtract>(*this, 3, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<SUB_16S, Subtract>(*this, 4, aSrc1, aSrc2, mPolicy, aDst);
    }
    else if (aSrc1.type() == VSDK_CV_16SC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilAri::WInitialize<SUB_16S_08U_16S, Subtract>(*this, 5, aSrc1, aSrc2, mPolicy, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

  }






  TableLookup::~TableLookup()
  {
    switch (mType){
    case 1:{
      delete (apexcv_fct<TABLELOOKUP_08U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT TableLookup::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    acLut,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1 &&
        acLut.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<TABLELOOKUP_08U>*pf = (apexcv_fct<TABLELOOKUP_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, acLut, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT TableLookup::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aLut /*!< Look-up table for transformation*/,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1 &&
      aLut.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<TABLELOOKUP_08U, TableLookup>(*this, 1, aSrc, aLut, aDst);
    }
    else{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  /***********************************************************************************/
  /*   Theshold                                                                      */
  /***********************************************************************************/
  Threshold::~Threshold()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<THRESHOLD_08U>*)mpProcess;
    }break;

    case 2:
    {
      delete (apexcv_fct<THRESHOLD_16U>*)mpProcess;
    }break;

    case 3:
    {
      delete (apexcv_fct<THRESHOLD_32U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT Threshold::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_08U>*pf = (apexcv_fct<THRESHOLD_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_16U>*pf = (apexcv_fct<THRESHOLD_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_32U>*pf = (apexcv_fct<THRESHOLD_32U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT Threshold::SetThreshold(
    const uint32_t    acThreshold /*!< Threshold value. */)
  {
    if (mThreshold.empty())
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    vsdk::SMat mat = mThreshold.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    switch (mThreshold.type())
    {
    case VSDK_CV_8UC1:/* 8UC1 */
      if (acThreshold >= 0x0100)
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mat.at<uint8_t>(0u) = (uint8_t)acThreshold;
      break;
    case VSDK_CV_16UC1:/* 16UC1 */
      if (acThreshold >= 0x010000)
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mat.at<uint16_t>((uint32_t) 0u) = (uint16_t)acThreshold;
      break;
    case VSDK_CV_32SC1:/* 32SC1 used as 32UC1 */
      mat.at<uint32_t>((uint32_t) 0u) = acThreshold;
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT Threshold::GetThreshold(
    uint32_t      &threshold /*!< Threshold value. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (mThreshold.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    vsdk::SMat mat = mThreshold.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    switch (mThreshold.type())
    {
    case VSDK_CV_8UC1:/* 8UC1 */
      threshold = mat.at<uint8_t>(0u);
      break;
    case VSDK_CV_16UC1:/* 16UC1 */
      threshold = mat.at<uint16_t>((uint32_t)0u);
      break;
    case VSDK_CV_32SC1:/* 32SC1 used as 32UC1 */
      threshold = mat.at<uint32_t>((uint32_t)0u);
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT Threshold::SetOutputValues(
    const uint8_t    acTrueVal, /*!< true output value. */
    const uint8_t    acFalseVal /*!< false output value. */)
  {
    if (mOutVals.empty())
    {
      mOutVals = vsdk::SUMat(1, 2, VSDK_CV_8UC1);
      if (mOutVals.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mOutVals.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<uint8_t>(0u) = (uint8_t)acTrueVal;
    mat.at<uint8_t>(1) = (uint8_t)acFalseVal;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT Threshold::GetOutputValues(
    uint8_t&    acTrueVal, /*!< true output value. */
    uint8_t&    acFalseVal /*!< false output value. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (mOutVals.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    vsdk::SMat mat = mOutVals.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    acTrueVal = mat.at<uint8_t>(0u);
    acFalseVal = mat.at<uint8_t>(1);
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT Threshold::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    const uint32_t    aThreshold /*!< Threshold value. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;//object already Initialized
    }

    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetOutputValues(255,0);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (aThreshold >= 256)
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mThreshold = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetThreshold(aThreshold);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      return ApexCVHostUtilAri::WInitialize<THRESHOLD_08U, Threshold>(*this, 1, aSrc, mThreshold, mOutVals, aDst);
    }
    else
    {
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        if (aThreshold >= 65536)
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
        mThreshold = vsdk::SUMat(1, 1, VSDK_CV_16UC1);
        APEXCV_LIB_RESULT lRetVal;
        lRetVal = SetThreshold(aThreshold);
        if (APEXCV_SUCCESS != lRetVal)
        {
          return lRetVal;
        }
        return ApexCVHostUtilAri::WInitialize<THRESHOLD_16U, Threshold>(*this, 2, aSrc, mThreshold, mOutVals, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_32SC1 &&
          aDst.type() == VSDK_CV_8UC1)
        {
          mThreshold = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
          APEXCV_LIB_RESULT lRetVal;
          lRetVal = SetThreshold(aThreshold);
          if (APEXCV_SUCCESS != lRetVal)
          {
            return lRetVal;
          }
          return ApexCVHostUtilAri::WInitialize<THRESHOLD_32U, Threshold>(*this, 3, aSrc, mThreshold, mOutVals, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }






  /***********************************************************************************/
  /*   ThesholdRange                                                                     */
  /***********************************************************************************/
  ThresholdRange::~ThresholdRange()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<THRESHOLD_RANGE_08U>*)mpProcess;
    }break;

    case 2:
    {
      delete (apexcv_fct<THRESHOLD_RANGE_16U>*)mpProcess;
    }break;

    case 3:
    {
      delete (apexcv_fct<THRESHOLD_RANGE_32U>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT ThresholdRange::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_RANGE_08U>*pf = (apexcv_fct<THRESHOLD_RANGE_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_RANGE_16U>*pf = (apexcv_fct<THRESHOLD_RANGE_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<THRESHOLD_RANGE_32U>*pf = (apexcv_fct<THRESHOLD_RANGE_32U>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT ThresholdRange::SetThresholds(
    const uint32_t    acLowThreshold /*!< low threshold. */,
    const uint32_t    acHighThreshold /*!< high threshold. */)
  {
    if (mThresholds.empty())
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    vsdk::SMat mat = mThresholds.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    switch (mThresholds.type())
    {
    case VSDK_CV_8UC1:/* 8UC1 */
      if ((acLowThreshold >= 0x0100) || (acHighThreshold >= 0x0100))
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mat.at<uint8_t>(0u) = (uint8_t)acLowThreshold;
      mat.at<uint8_t>(1) = (uint8_t)acHighThreshold;
      break;
    case VSDK_CV_16UC1:/* 16UC1 */
      if ((acLowThreshold >= 0x010000) || (acHighThreshold >= 0x010000))
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mat.at<uint16_t>((uint32_t)0u) = (uint16_t)acLowThreshold;
      mat.at<uint16_t>((uint32_t)1) = (uint16_t)acHighThreshold;
      break;
    case VSDK_CV_32SC1:/* 32SC1 used as 32UC1 */
      mat.at<uint32_t>((uint32_t)0u) = acLowThreshold;
      mat.at<uint32_t>((uint32_t)1) = acHighThreshold;
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ThresholdRange::GetThresholds(
    uint32_t&    aLowThreshold /*!< Threshold value. */,
    uint32_t&    aHighThreshold /*!< Threshold value. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (mThresholds.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    vsdk::SMat mat = mThresholds.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    switch (mThresholds.type())
    {
    case VSDK_CV_8UC1:/* 8UC1 */
      aLowThreshold = mat.at<uint8_t>((uint32_t)0u);
      aHighThreshold = mat.at<uint8_t>((uint32_t)1);
      break;
    case VSDK_CV_16UC1:/* 16UC1 */
      aLowThreshold = mat.at<uint16_t>((uint32_t)0u);
      aHighThreshold = mat.at<uint16_t>((uint32_t)1);
      break;
    case VSDK_CV_32SC1:/* 32SC1 used as 32UC1 */
      aLowThreshold = mat.at<uint32_t>((uint32_t)0u);
      aHighThreshold = mat.at<uint32_t>((uint32_t)1);
      break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ThresholdRange::SetOutputValues(
    const uint8_t    acTrueVal, /*!< true output value. */
    const uint8_t    acFalseVal /*!< false output value. */)
  {
    if (mOutVals.empty())
    {
      mOutVals = vsdk::SUMat(1, 2, VSDK_CV_8UC1);
      if (mOutVals.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mOutVals.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<uint8_t>(0u) = (uint8_t)acTrueVal;
    mat.at<uint8_t>(1) = (uint8_t)acFalseVal;
    return APEXCV_SUCCESS;
  }



  APEXCV_LIB_RESULT ThresholdRange::GetOutputValues(
    uint8_t&    aTrueVal, /*!< true output value. */
    uint8_t&    aFalseVal /*!< false output value. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (mOutVals.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    vsdk::SMat mat = mOutVals.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);/* noncached as it is for a single value */
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aTrueVal = mat.at<uint8_t>(0u);
    aFalseVal = mat.at<uint8_t>(1);
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ThresholdRange::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    const uint32_t    acLowThreshold /*!< Low Threshold value. */,
    const uint32_t    acHighThreshold /*!< High Threshold value. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;//object already Initialized
    }

    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetOutputValues(255,0);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if ((acLowThreshold >= 0x0100) || (acHighThreshold >= 0x0100))
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
      mThresholds = vsdk::SUMat(1, 2, VSDK_CV_8UC1);
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetThresholds(acLowThreshold, acHighThreshold);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      return ApexCVHostUtilAri::WInitialize<THRESHOLD_RANGE_08U, ThresholdRange>(*this, 1, aSrc, mThresholds, mOutVals, aDst);
    }
    else
    {
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        if ((acLowThreshold >= 0x010000) || (acHighThreshold >= 0x010000))
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
        mThresholds = vsdk::SUMat(1, 2, VSDK_CV_16UC1);
        APEXCV_LIB_RESULT lRetVal;
        lRetVal = SetThresholds(acLowThreshold, acHighThreshold);
        if (APEXCV_SUCCESS != lRetVal)
        {
          return lRetVal;
        }
        return ApexCVHostUtilAri::WInitialize<THRESHOLD_RANGE_16U, ThresholdRange>(*this, 2, aSrc, mThresholds, mOutVals, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_32SC1 &&
          aDst.type() == VSDK_CV_8UC1)
        {
          mThresholds = vsdk::SUMat(1, 2, VSDK_CV_32SC1);
          APEXCV_LIB_RESULT lRetVal;
          lRetVal = SetThresholds(acLowThreshold, acHighThreshold);
          if (APEXCV_SUCCESS != lRetVal)
          {
            return lRetVal;
          }
          return ApexCVHostUtilAri::WInitialize<THRESHOLD_RANGE_32U, ThresholdRange>(*this, 3, aSrc, mThresholds, mOutVals, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }






  BitwiseXOR::~BitwiseXOR()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<XOR_08U>*)mpProcess;
    }
    break;

    case 2:
    {
      delete (apexcv_fct<XOR_16U>*)mpProcess;
    }
    break;

    case 3:
    {
      delete (apexcv_fct<XOR_32U>*)mpProcess;
    }
    break;
    }
  }


  APEXCV_LIB_RESULT BitwiseXOR::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType)
    {
    case 0:
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:
    {
      if (aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<XOR_08U>*pf = (apexcv_fct<XOR_08U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<XOR_16U>*pf = (apexcv_fct<XOR_16U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 3:
    {
      if (aSrc1.type() == VSDK_CV_32SC1 &&
        aSrc2.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        apexcv_fct<XOR_32U>*pf = (apexcv_fct<XOR_32U>*)mpProcess;
        return pf->ReconnectIO(aSrc1, aSrc2, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT BitwiseXOR::Initialize(
    vsdk::SUMat&    aSrc1 /*!< Source memory buffer. */,
    vsdk::SUMat&    aSrc2 /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilAri::WInitialize<XOR_08U, BitwiseXOR>(*this, 1, aSrc1, aSrc2, aDst);
    }
    else
    {
      if (aSrc1.type() == VSDK_CV_16UC1 &&
        aSrc2.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        return ApexCVHostUtilAri::WInitialize<XOR_16U, BitwiseXOR>(*this, 2, aSrc1, aSrc2, aDst);
      }
      else
      {
        if (aSrc1.type() == VSDK_CV_32SC1 &&
          aSrc2.type() == VSDK_CV_32SC1 &&
          aDst.type() == VSDK_CV_32SC1)
        {
          return ApexCVHostUtilAri::WInitialize<XOR_32U, BitwiseXOR>(*this, 3, aSrc1, aSrc2, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }


}/* namespace */
