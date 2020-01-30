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

#include <apexcv_base_image_filters.h>
#include "all_filter_graph_registrations.h"

#include "apexcv_filter.h"


namespace apexcv{


  BilateralFilter::~BilateralFilter()
  {
    if (0 != mType)
    {
      delete (apexcv_filter<BILATERAL_5X5> *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT BilateralFilter::ReconnectIO(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      apexcv_filter<BILATERAL_5X5> *pf = (apexcv_filter<BILATERAL_5X5> *)mpProcess;
      return pf->ReconnectIO(aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT BilateralFilter::SetSigmaColor(int aSigmaColor)
  {
    /* should work even if not initialized */
    if (mSigmaColor.empty())
    {
      mSigmaColor = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mSigmaColor.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mSigmaColor.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int32_t>(0u) = aSigmaColor;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT BilateralFilter::SetSigmaSpace(int aSigmaSpace)
  {
    /* should work even if not initialized */
    if (mSigmaSpace.empty())
    {
      mSigmaSpace = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mSigmaSpace.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mSigmaSpace.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int32_t>(0u) = aSigmaSpace;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT BilateralFilter::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    int    aWindowSize, /* Window Size, 5 */
    int    aSigmaColor, /*!< Sigma value for color space. */
    int    aSigmaSpace, /*!< Sigma value for distance space. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already initialized
    }

    if (5 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid parameter
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {

      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetSigmaColor(aSigmaColor);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      lRetVal = SetSigmaSpace(aSigmaSpace);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      return ApexCVHostUtilFilter::WInitialize<BILATERAL_5X5, BilateralFilter>(*this, 1, aSrc, mSigmaColor, mSigmaSpace, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  BoxFilter::~BoxFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<BOX_3X3_08U> *)mpProcess;
    } break;
    case 2:{
      delete (apexcv_filter<BOX_3X3_16S> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<BOX_5X5_08U> *)mpProcess;
    } break;
    case 4:{
      delete (apexcv_filter<BOX_5X5_16S> *)mpProcess;
    } break;
    case 5:{
      delete (apexcv_filter<BOX_7X7_08U> *)mpProcess;
    } break;
    case 6:{
      delete (apexcv_filter<BOX_9X9_08U> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT BoxFilter::ReconnectIO(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if(0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<BOX_3X3_08U> *p = (apexcv_filter<BOX_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (3 == mType)
      {
        apexcv_filter<BOX_5X5_08U> *p = (apexcv_filter<BOX_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (5 == mType)
      {
        apexcv_filter<BOX_7X7_08U> *p = (apexcv_filter<BOX_7X7_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (6 == mType)
      {
        apexcv_filter<BOX_9X9_08U> *p = (apexcv_filter<BOX_9X9_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else if (aSrc.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {

      if (2 == mType)
      {
        apexcv_filter<BOX_3X3_16S> *p = (apexcv_filter<BOX_3X3_16S> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (4 == mType)
      {
        apexcv_filter<BOX_5X5_16S> *p = (apexcv_filter<BOX_5X5_16S> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT BoxFilter::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source memory buffer. */
    int    aWindowSize, /*!< Defines a filter window with dimensions _windowSize_ x _windowSize_. Default is 3. See supported window sizes. */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_3X3_08U, BoxFilter>(*this, 1, aSrc, aDst);
      }
      else if (aSrc.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_3X3_16S, BoxFilter>(*this, 2, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_5X5_08U, BoxFilter>(*this, 3, aSrc, aDst);
      }
      else if (aSrc.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_5X5_16S, BoxFilter>(*this, 4, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 7)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_7X7_08U, BoxFilter>(*this, 5, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 9)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<BOX_9X9_08U, BoxFilter>(*this, 6, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  CensusFilter::~CensusFilter()
  {
    if (0 != mType)
    {
      delete (apexcv_filter<CENSUS_3X3_08U> *)mpProcess;
    }
  }

  APEXCV_LIB_RESULT CensusFilter::ReconnectIO(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if ((VSDK_CV_8UC1 == aSrc.type()) && (VSDK_CV_8UC1 == aDst.type()))
    {
      apexcv_filter<CENSUS_3X3_08U> *p = (apexcv_filter<CENSUS_3X3_08U> *)mpProcess;
      return p->ReconnectIO(aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT CensusFilter::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 && aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitialize<CENSUS_3X3_08U, CensusFilter>(*this, 1, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }






  ConvolveFilter::~ConvolveFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<CONVOLVE_SCALE_3X3_08U> *)mpProcess;
    } break;
    case 2:{
      delete (apexcv_filter<CONVOLVE_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<CONVOLVE_SCALE_5X5_08U> *)mpProcess;
    } break;
    case 4:{
      delete (apexcv_filter<CONVOLVE_5X5_08U> *)mpProcess;
    } break;
    case 5:{
      delete (apexcv_filter<CONVOLVE_SCALE_3X3_16S> *)mpProcess;
    } break;
    case 6:{
      delete (apexcv_filter<CONVOLVE_3X3_16S> *)mpProcess;
    } break;
    case 7:{
      delete (apexcv_filter<CONVOLVE_SCALE_5X5_16S> *)mpProcess;
    } break;
    case 8:{
      delete (apexcv_filter<CONVOLVE_5X5_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT ConvolveFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      switch (mType)
      {
      case 1:{
        apexcv_filter<CONVOLVE_SCALE_3X3_08U> *p = (apexcv_filter<CONVOLVE_SCALE_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      case 2:{
        apexcv_filter<CONVOLVE_3X3_08U> *p = (apexcv_filter<CONVOLVE_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      case 3:{
        apexcv_filter<CONVOLVE_SCALE_5X5_08U> *p = (apexcv_filter<CONVOLVE_SCALE_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      case 4:{
        apexcv_filter<CONVOLVE_5X5_08U> *p = (apexcv_filter<CONVOLVE_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      default:{
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
      }
    }
    else
    {

      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {

        switch (mType)
        {
        case 5:{
          apexcv_filter<CONVOLVE_SCALE_3X3_16S> *p = (apexcv_filter<CONVOLVE_SCALE_3X3_16S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        } break;
        case 6:{
          apexcv_filter<CONVOLVE_3X3_16S> *p = (apexcv_filter<CONVOLVE_3X3_16S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        } break;
        case 7:{
          apexcv_filter<CONVOLVE_SCALE_5X5_16S> *p = (apexcv_filter<CONVOLVE_SCALE_5X5_16S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        } break;
        case 8:{
          apexcv_filter<CONVOLVE_5X5_16S> *p = (apexcv_filter<CONVOLVE_5X5_16S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        } break;
        default:{
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
        }
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
  }


  APEXCV_LIB_RESULT ConvolveFilter::SetFilterScale(int aFilterScale)
  {
    if (mScale.empty())
    {
      mScale = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mScale.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int32_t>(0u) = aFilterScale;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT ConvolveFilter::SetFilterCoeff(signed char(&aFilterCoeff)[9])
  {
    if (mFilterCoeff.empty())
    {
      mFilterCoeff = vsdk::SUMat(1, 9, VSDK_CV_8SC1);
      if (mFilterCoeff.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterCoeff.cols != 9)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    vsdk::SMat mat = mFilterCoeff.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 9; i++)
    {
      mat.at<signed char>(i) = aFilterCoeff[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT ConvolveFilter::SetFilterCoeff(signed char(&aFilterCoeff)[25])
  {
    if (mFilterCoeff.empty())
    {
      mFilterCoeff = vsdk::SUMat(1, 25, VSDK_CV_8SC1);
      if (mFilterCoeff.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterCoeff.cols != 25)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    vsdk::SMat mat = mFilterCoeff.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 25; i++)
    {
      mat.at<signed char>(i) = aFilterCoeff[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT ConvolveFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    signed char(&aFilterCoeff)[9],
    int    aFilterScale,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    if (0 != aFilterScale)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetFilterCoeff(aFilterCoeff);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      lRetVal = SetFilterScale(aFilterScale);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }

      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitializeScale<CONVOLVE_SCALE_3X3_08U, ConvolveFilter>(*this, 1, aSrc, mFilterCoeff, mScale, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_16SC1)
        {
          return ApexCVHostUtilFilter::WInitializeScale<CONVOLVE_SCALE_3X3_16S, ConvolveFilter>(*this, 5, aSrc, mFilterCoeff, mScale, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else if (0 == aFilterScale)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetFilterCoeff(aFilterCoeff);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }

      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<CONVOLVE_3X3_08U, ConvolveFilter>(*this, 2, aSrc, mFilterCoeff, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_16SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<CONVOLVE_3X3_16S, ConvolveFilter>(*this, 6, aSrc, mFilterCoeff, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT ConvolveFilter::Initialize(
    vsdk::SUMat&  aSrc /*!< Source memory buffer. */,
    signed char(&aFilterCoeff)[25],
    int          aFilterScale,
    vsdk::SUMat&  aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (0 != aFilterScale)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetFilterCoeff(aFilterCoeff);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }
      lRetVal = SetFilterScale(aFilterScale);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }

      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitializeScale<CONVOLVE_SCALE_5X5_08U, ConvolveFilter>(*this, 3, aSrc, mFilterCoeff, mScale, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_16SC1)
        {
          return ApexCVHostUtilFilter::WInitializeScale<CONVOLVE_SCALE_5X5_16S, ConvolveFilter>(*this, 7, aSrc, mFilterCoeff, mScale, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else // (0 == aFilterScale)
    {
      APEXCV_LIB_RESULT lRetVal;
      lRetVal = SetFilterCoeff(aFilterCoeff);
      if (APEXCV_SUCCESS != lRetVal)
      {
        return lRetVal;
      }

      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<CONVOLVE_5X5_08U, ConvolveFilter>(*this, 4, aSrc, mFilterCoeff, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_16SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<CONVOLVE_5X5_16S, ConvolveFilter>(*this, 8, aSrc, mFilterCoeff, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
  }







  DilateFilter::~DilateFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<DILATE_3X3_08U> *)mpProcess;
    } break;
    case 2:{
      delete (apexcv_filter<DILATE_3X3_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT DilateFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {

      if (1 == mType)
      {
        apexcv_filter<DILATE_3X3_08U> *p = (apexcv_filter<DILATE_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      if (aSrc.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        if (2 == mType)
        {
          apexcv_filter<DILATE_3X3_16S> *p = (apexcv_filter<DILATE_3X3_16S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
  }

  APEXCV_LIB_RESULT DilateFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitialize<DILATE_3X3_08U, DilateFilter>(*this, 1, aSrc, aDst);
    }
    else if (aSrc.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<DILATE_3X3_16S, DilateFilter>(*this, 2, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  ErodeFilter::~ErodeFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<ERODE_3X3_08U> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT ErodeFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<ERODE_3X3_08U> *p = (apexcv_filter<ERODE_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT ErodeFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitialize<ERODE_3X3_08U, ErodeFilter>(*this, 1, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  GaussianFilter::~GaussianFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<GAUSSIAN_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<GAUSSIAN_5X5_08U> *)mpProcess;
    } break;
    case 5:{
      delete (apexcv_filter<GAUSSIAN_7X7_08U> *)mpProcess;
    } break;
    case 7:{
      delete (apexcv_filter<GAUSSIAN_9X9_08U> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT GaussianFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<GAUSSIAN_3X3_08U> *p = (apexcv_filter<GAUSSIAN_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (3 == mType)
      {
        apexcv_filter<GAUSSIAN_5X5_08U> *p = (apexcv_filter<GAUSSIAN_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (5 == mType)
      {
        apexcv_filter<GAUSSIAN_7X7_08U> *p = (apexcv_filter<GAUSSIAN_7X7_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (7 == mType)
      {
        apexcv_filter<GAUSSIAN_9X9_08U> *p = (apexcv_filter<GAUSSIAN_9X9_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT GaussianFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3, 5, 7 or 9 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<GAUSSIAN_3X3_08U, GaussianFilter>(*this, 1, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<GAUSSIAN_5X5_08U, GaussianFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 7)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<GAUSSIAN_7X7_08U, GaussianFilter>(*this, 5, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 9)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<GAUSSIAN_9X9_08U, GaussianFilter>(*this, 7, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  MedianFilter::~MedianFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<MEDIAN_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<MEDIAN_5X5_08U> *)mpProcess;
    } break;
    case 5:{
      delete (apexcv_filter<MEDIAN_7X7_08U> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT MedianFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<MEDIAN_3X3_08U> *p = (apexcv_filter<MEDIAN_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (3 == mType)
      {
        apexcv_filter<MEDIAN_5X5_08U> *p = (apexcv_filter<MEDIAN_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else if (5 == mType)
      {
        apexcv_filter<MEDIAN_7X7_08U> *p = (apexcv_filter<MEDIAN_7X7_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT MedianFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 or 7 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<MEDIAN_3X3_08U, MedianFilter>(*this, 1, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<MEDIAN_5X5_08U, MedianFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 7)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<MEDIAN_7X7_08U, MedianFilter>(*this, 5, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }








  PrewittXFilter::~PrewittXFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<PREWITT_X_3X3> *)mpProcess;
    } break;

    }
  }

  APEXCV_LIB_RESULT PrewittXFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<PREWITT_X_3X3> *p = (apexcv_filter<PREWITT_X_3X3> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT PrewittXFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<PREWITT_X_3X3, PrewittXFilter>(*this, 1, aSrc, aDst);
    }
    else
    {
      //TODO: add standardized apex cv error codes
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }




  PrewittYFilter::~PrewittYFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<PREWITT_Y_3X3> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT PrewittYFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<PREWITT_Y_3X3> *p = (apexcv_filter<PREWITT_Y_3X3> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT PrewittYFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<PREWITT_Y_3X3, PrewittYFilter>(*this, 1, aSrc, aDst);
    }
    else
    {
      //TODO: add standardized apex cv error codes
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }




  ScharrFilter::~ScharrFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SCHARR_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SCHARR_5X5_08U> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT ScharrFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
    {
      if (aWindowSize == 3)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_3X3_08U, ScharrFilter>(*this, 1, aSrc, aDst);
      }
      else if (aWindowSize == 5)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_5X5_08U, ScharrFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT ScharrFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SCHARR_3X3_08U> *p = (apexcv_filter<SCHARR_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SCHARR_5X5_08U> *p = (apexcv_filter<SCHARR_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  ScharrXFilter::~ScharrXFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SCHARR_X_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SCHARR_X_5X5_08U> *)mpProcess;
    } break;
    default: break;
    }
  }

  APEXCV_LIB_RESULT ScharrXFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      if (aWindowSize == 3)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_X_3X3_08U, ScharrXFilter>(*this, 1, aSrc, aDst);
      }
      else if (aWindowSize == 5)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_X_5X5_08U, ScharrXFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

  }


  APEXCV_LIB_RESULT ScharrXFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SCHARR_X_3X3_08U> *p = (apexcv_filter<SCHARR_X_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SCHARR_X_5X5_08U> *p = (apexcv_filter<SCHARR_X_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  ScharrYFilter::~ScharrYFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SCHARR_Y_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SCHARR_Y_5X5_08U> *)mpProcess;
    } break;
    default: break;
    }
  }

  APEXCV_LIB_RESULT ScharrYFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      if (aWindowSize == 3)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_Y_3X3_08U, ScharrYFilter>(*this, 1, aSrc, aDst);
      }
      else if (aWindowSize == 5)
      {
        return ApexCVHostUtilFilter::WInitialize<SCHARR_Y_5X5_08U, ScharrYFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT ScharrYFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SCHARR_Y_3X3_08U> *p = (apexcv_filter<SCHARR_Y_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SCHARR_Y_5X5_08U> *p = (apexcv_filter<SCHARR_Y_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  ScharrXYFilter::~ScharrXYFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SCHARR_XY_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SCHARR_XY_5X5_08U> *)mpProcess;
    } break;
    default: break;
    }
  }


  APEXCV_LIB_RESULT ScharrXYFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDstX /*!< Destination memory buffer. */,
    vsdk::SUMat&    aDstY /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
        aDstX.type() == VSDK_CV_16SC1 &&
        aDstY.type() == VSDK_CV_16SC1)
    {
      if (aWindowSize == 3)
      {
        return ApexCVHostUtilFilter::WInitialize2Dst<SCHARR_XY_3X3_08U, ScharrXYFilter>(*this, 1, aSrc, aDstX, aDstY);
      }
      else if (aWindowSize == 5)
      {
        return ApexCVHostUtilFilter::WInitialize2Dst<SCHARR_XY_5X5_08U, ScharrXYFilter>(*this, 3, aSrc, aDstX, aDstY);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT ScharrXYFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&  aDstX /*!< Destination Image buffer X. */,
    vsdk::SUMat&  aDstY /*!< Destination Image buffer Y. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDstX.type() == VSDK_CV_16SC1 &&
      aDstY.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SCHARR_XY_3X3_08U> *p = (apexcv_filter<SCHARR_XY_3X3_08U> *)mpProcess;
        return p->ReconnectIO2Dst(aSrc, aDstX, aDstY);
      }
      else if (3 == mType)
      {
        apexcv_filter<SCHARR_XY_5X5_08U> *p = (apexcv_filter<SCHARR_XY_5X5_08U> *)mpProcess;
        return p->ReconnectIO2Dst(aSrc, aDstX, aDstY);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  SobelFilter::~SobelFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SOBEL_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SOBEL_5X5_08U> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SOBEL_3X3_08U> *p = (apexcv_filter<SOBEL_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SOBEL_5X5_08U> *p = (apexcv_filter<SOBEL_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SobelFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_3X3_08U, SobelFilter>(*this, 1, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_5X5_08U, SobelFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  SobelXFilter::~SobelXFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SOBEL_X_3X3_08U> *)mpProcess;
    } break;
    case 2:{
      delete (apexcv_filter<SOBEL_X_3X3_08S> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SOBEL_X_5X5_08U> *)mpProcess;
    } break;
    case 4:{
      delete (apexcv_filter<SOBEL_X_5X5_08S> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelXFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SOBEL_X_3X3_08U> *p = (apexcv_filter<SOBEL_X_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SOBEL_X_5X5_08U> *p = (apexcv_filter<SOBEL_X_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8SC1)
      {
        if (2 == mType)
        {
          apexcv_filter<SOBEL_X_3X3_08S> *p = (apexcv_filter<SOBEL_X_3X3_08S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          if (4 == mType)
          {
            apexcv_filter<SOBEL_X_5X5_08S> *p = (apexcv_filter<SOBEL_X_5X5_08S> *)mpProcess;
            return p->ReconnectIO(aSrc, aDst);
          }
          else
          {
            return APEXCV_ERROR_INTERNAL_ERROR;
          }
        }
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
  }

  APEXCV_LIB_RESULT SobelXFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_X_3X3_08U, SobelXFilter>(*this, 1, aSrc, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_8SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<SOBEL_X_3X3_08S, SobelXFilter>(*this, 2, aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_X_5X5_08U, SobelXFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_8SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<SOBEL_X_5X5_08S, SobelXFilter>(*this, 4, aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

  }




  SobelYFilter::~SobelYFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SOBEL_Y_3X3_08U> *)mpProcess;
    } break;
    case 2:{
      delete (apexcv_filter<SOBEL_Y_3X3_08S> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<SOBEL_Y_5X5_08U> *)mpProcess;
    } break;
    case 4:{
      delete (apexcv_filter<SOBEL_Y_5X5_08S> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelYFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SOBEL_Y_3X3_08U> *p = (apexcv_filter<SOBEL_Y_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        if (3 == mType)
        {
          apexcv_filter<SOBEL_Y_5X5_08U> *p = (apexcv_filter<SOBEL_Y_5X5_08U> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INTERNAL_ERROR;
        }
      }
    }
    else
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8SC1)
      {

        if (2 == mType)
        {
          apexcv_filter<SOBEL_Y_3X3_08S> *p = (apexcv_filter<SOBEL_Y_3X3_08S> *)mpProcess;
          return p->ReconnectIO(aSrc, aDst);
        }
        else
        {
          if (4 == mType)
          {
            apexcv_filter<SOBEL_Y_5X5_08S> *p = (apexcv_filter<SOBEL_Y_5X5_08S> *)mpProcess;
            return p->ReconnectIO(aSrc, aDst);
          }
          else
          {
            return APEXCV_ERROR_INTERNAL_ERROR;
          }
        }
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
  }

  APEXCV_LIB_RESULT SobelYFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 or 5 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_Y_3X3_08U, SobelYFilter>(*this, 1, aSrc, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_8SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<SOBEL_Y_3X3_08S, SobelYFilter>(*this, 2, aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else if (aWindowSize == 5)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<SOBEL_Y_5X5_08U, SobelYFilter>(*this, 3, aSrc, aDst);
      }
      else
      {
        if (aSrc.type() == VSDK_CV_8UC1 &&
          aDst.type() == VSDK_CV_8SC1)
        {
          return ApexCVHostUtilFilter::WInitialize<SOBEL_Y_5X5_08S, SobelYFilter>(*this, 4, aSrc, aDst);
        }
        else
        {
          return APEXCV_ERROR_INVALID_ARGUMENT;
        }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  SobelXYFilter::~SobelXYFilter()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<SOBEL_XY_3X3_08U> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelXYFilter::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&  aDstX /*!< Destination Image buffer X. */,
    vsdk::SUMat&  aDstY /*!< Destination Image buffer Y. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDstX.type() == VSDK_CV_8UC1 &&
      aDstY.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<SOBEL_XY_3X3_08U> *p = (apexcv_filter<SOBEL_XY_3X3_08U> *)mpProcess;
        return p->ReconnectIO2Dst(aSrc, aDstX, aDstY);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SobelXYFilter::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDstX /*!< Destination memory buffer. */,
    vsdk::SUMat&    aDstY /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDstX.type() == VSDK_CV_8UC1 &&
      aDstY.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitialize2Dst<SOBEL_XY_3X3_08U, SobelXYFilter>(*this, 1, aSrc, aDstX, aDstY);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }









  BoxFilterHT::~BoxFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_BOX_3X3_08U> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT BoxFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      if (1 == mType)
      {
        apexcv_filter<HT_BOX_3X3_08U> *p = (apexcv_filter<HT_BOX_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

  }

  APEXCV_LIB_RESULT BoxFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (3 == aWindowSize)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        return ApexCVHostUtilFilter::WInitialize<HT_BOX_3X3_08U, BoxFilterHT>(*this, 1, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }







  DerivativeXFilterHT::~DerivativeXFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_CENTRALDX_3X3_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT DerivativeXFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<HT_CENTRALDX_3X3_16S> *pf = (apexcv_filter<HT_CENTRALDX_3X3_16S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT DerivativeXFilterHT::SetK0(signed char aK0)
  {
    if (mK0.empty())
    {
      mK0 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK0.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK0.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK0;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT DerivativeXFilterHT::SetK1(signed char aK1)
  {
    if (mK1.empty())
    {
      mK1 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK1.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK1.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK1;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT DerivativeXFilterHT::SetK2(signed char aK2)
  {
    if (mK2.empty())
    {
      mK2 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK2.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK2.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK2;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT DerivativeXFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    signed char    aK0,
    signed char    aK1,
    signed char    aK2,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetK0(aK0);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    lRetVal = SetK1(aK1);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    lRetVal = SetK2(aK2);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<HT_CENTRALDX_3X3_16S, DerivativeXFilterHT>(*this, 1, aSrc, mK0, mK1, mK2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  DerivativeYFilterHT::~DerivativeYFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_CENTRALDY_3X3_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT DerivativeYFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<HT_CENTRALDY_3X3_16S> *pf = (apexcv_filter<HT_CENTRALDY_3X3_16S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT DerivativeYFilterHT::SetK0(signed char aK0)
  {
    if (mK0.empty())
    {
      mK0 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK0.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK0.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK0;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT DerivativeYFilterHT::SetK1(signed char aK1)
  {
    if (mK1.empty())
    {
      mK1 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK1.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK1.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK1;
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT DerivativeYFilterHT::SetK2(signed char aK2)
  {
    if (mK2.empty())
    {
      mK2 = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mK2.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mK2.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aK2;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT DerivativeYFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    signed char    aK0,
    signed char    aK1,
    signed char    aK2,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    if (3 != aWindowSize)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetK0(aK0);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    lRetVal = SetK1(aK1);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }
    lRetVal = SetK2(aK2);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<HT_CENTRALDY_3X3_16S, DerivativeYFilterHT>(*this, 1, aSrc, mK0, mK1, mK2, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  ConvolveFilterHT::~ConvolveFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_GENERIC_3X3_08U> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<HT_GENERIC_5X5_08U> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT ConvolveFilterHT::ReconnectIO(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      switch (mType)
      {
      case 1:{
        apexcv_filter<HT_GENERIC_3X3_08U> *p = (apexcv_filter<HT_GENERIC_3X3_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      case 3:{
        apexcv_filter<HT_GENERIC_5X5_08U> *p = (apexcv_filter<HT_GENERIC_5X5_08U> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      } break;
      default:{
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT ConvolveFilterHT::SetFilterCoeff(signed char(&aFilterCoeff)[9])
  {
    const int CL_FILTER_SIZE = ((9 + 3) / 4) * 4;
    if (mFilterCoeff.empty())
    {
      mFilterCoeff = vsdk::SUMat(1, CL_FILTER_SIZE, VSDK_CV_8SC1);// use the nearest 4 bytes number, because optimized kernel implementation requires this requirement
      if (mFilterCoeff.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterCoeff.cols != CL_FILTER_SIZE)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterCoeff.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 9; i++)
    {
      mat.at<char>(i) = aFilterCoeff[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT ConvolveFilterHT::SetFilterCoeff(signed char(&aFilterCoeff)[25])
  {
    const int CL_FILTER_SIZE = ((25 + 3) / 4) * 4;
    if (mFilterCoeff.empty())
    {
      mFilterCoeff = vsdk::SUMat(1, CL_FILTER_SIZE, VSDK_CV_8SC1);// use the nearest 4 bytes number, because optimized kernel implementation requires this requirement
      if (mFilterCoeff.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }

    if (mFilterCoeff.cols != CL_FILTER_SIZE)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterCoeff.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 25; i++)
    {
      mat.at<char>(i) = aFilterCoeff[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT ConvolveFilterHT::SetFilterScale(signed char aFilterScale)
  {
    if (mScale.empty())
    {
      mScale = vsdk::SUMat(1, 1, VSDK_CV_8SC1);
      if (mScale.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    vsdk::SMat mat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    mat.at<int8_t>(0u) = aFilterScale;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ConvolveFilterHT::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    signed char(&aFilterCoeff)[9],
    signed char    aFilterScale,
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetFilterCoeff(aFilterCoeff);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    lRetVal = SetFilterScale(aFilterScale);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitializeScale<HT_GENERIC_3X3_08U, ConvolveFilterHT>(*this, 1, aSrc, mFilterCoeff, mScale, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT ConvolveFilterHT::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    signed char(&aFilterCoeff)[25],
    signed char    aFilterScale,
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetFilterCoeff(aFilterCoeff);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    lRetVal = SetFilterScale(aFilterScale);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      return ApexCVHostUtilFilter::WInitializeScale<HT_GENERIC_5X5_08U, ConvolveFilterHT>(*this, 3, aSrc, mFilterCoeff, mScale, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

  }







  SaturateFilterHT::~SaturateFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_SATURATE_08S> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SaturateFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<HT_SATURATE_08S> *p = (apexcv_filter<HT_SATURATE_08S> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SaturateFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aSrc.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {
      return ApexCVHostUtilFilter::WInitialize<HT_SATURATE_08S, SaturateFilterHT>(*this, 1, aSrc, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }









  SeparableFilterHT::~SeparableFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_SEPARABLE_3X3_16S> *)mpProcess;
    } break;
    case 3:{
      delete (apexcv_filter<HT_SEPARABLE_5X5_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT SeparableFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      switch (mType)
      {
      case 1:{
        apexcv_filter<HT_SEPARABLE_3X3_16S> *pf = (apexcv_filter<HT_SEPARABLE_3X3_16S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      } break;
      case 3:{
        apexcv_filter<HT_SEPARABLE_5X5_16S> *pf = (apexcv_filter<HT_SEPARABLE_5X5_16S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      } break;
      default:{
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }


  APEXCV_LIB_RESULT SeparableFilterHT::SetFilterRow(signed char(&aFilterRow)[3])
  {
    if (mFilterRow.empty())
    {
      mFilterRow = vsdk::SUMat(1, 3, VSDK_CV_8SC1);
      if (mFilterRow.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterRow.cols != 3)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterRow.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 3; i++)
    {
      mat.at<signed char>(i) = aFilterRow[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT SeparableFilterHT::SetFilterCol(signed char(&aFilterCol)[3])
  {
    if (mFilterCol.empty())
    {
      mFilterCol = vsdk::SUMat(1, 3, VSDK_CV_8SC1);
      if (mFilterCol.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterCol.cols != 3)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterCol.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 3; i++)
    {
      mat.at<signed char>(i) = aFilterCol[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT SeparableFilterHT::SetFilterRow(signed char(&aFilterRow)[5])
  {
    if (mFilterRow.empty())
    {
      mFilterRow = vsdk::SUMat(1, 5, VSDK_CV_8SC1);
      if (mFilterRow.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterRow.cols != 5)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterRow.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 5; i++)
    {
      mat.at<signed char>(i) = aFilterRow[i];
    }
    return APEXCV_SUCCESS;
  }

  APEXCV_LIB_RESULT SeparableFilterHT::SetFilterCol(signed char(&aFilterCol)[5])
  {
    if (mFilterCol.empty())
    {
      mFilterCol = vsdk::SUMat(1, 5, VSDK_CV_8SC1);
      if (mFilterCol.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mFilterCol.cols != 5)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    vsdk::SMat mat = mFilterCol.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    for (int i = 0; i < 5; i++)
    {
      mat.at<signed char>(i) = aFilterCol[i];
    }
    return APEXCV_SUCCESS;
  }



  APEXCV_LIB_RESULT SeparableFilterHT::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    signed char(&aFilterRow)[3],
    signed char(&aFilterCol)[3],
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetFilterRow(aFilterRow);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    lRetVal = SetFilterCol(aFilterCol);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitializeSeparable<HT_SEPARABLE_3X3_16S, SeparableFilterHT>(*this, 1, aSrc, mFilterRow, mFilterCol, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  APEXCV_LIB_RESULT SeparableFilterHT::Initialize(
    vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    signed char(&aFilterRow)[5],
    signed char(&aFilterCol)[5],
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    APEXCV_LIB_RESULT lRetVal;
    lRetVal = SetFilterRow(aFilterRow);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    lRetVal = SetFilterCol(aFilterCol);
    if (lRetVal != APEXCV_SUCCESS)
    {
      return lRetVal;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      return ApexCVHostUtilFilter::WInitializeSeparable<HT_SEPARABLE_5X5_16S, SeparableFilterHT>(*this, 3, aSrc, mFilterRow, mFilterCol, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  SobelFilterHT::~SobelFilterHT()
  {
    switch (mType)
    {
    case 1:{
      delete (apexcv_filter<HT_SOBEL_3X3_16S> *)mpProcess;
    } break;
    }
  }

  APEXCV_LIB_RESULT SobelFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_16SC1)
    {
      if (1 == mType)
      {
        apexcv_filter<HT_SOBEL_3X3_16S> *p = (apexcv_filter<HT_SOBEL_3X3_16S> *)mpProcess;
        return p->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SobelFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize == 3)
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        return ApexCVHostUtilFilter::WInitialize<HT_SOBEL_3X3_16S, SobelFilterHT>(*this, 1, aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





  SobelXFilterHT::~SobelXFilterHT()
  {
    switch (mType)
    {
    case 2:{
      delete (apexcv_filter<HT_SOBEL_X_3X3_08S> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelXFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {
      if (2 == mType)
      {
        apexcv_filter<HT_SOBEL_X_3X3_08S> *pf = (apexcv_filter<HT_SOBEL_X_3X3_08S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SobelXFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize != 3)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {

      mBuffer0 = vsdk::SUMat(aSrc.rows, aSrc.cols, VSDK_CV_16SC1);
      if (mBuffer0.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      return ApexCVHostUtilFilter::WInitializeSobelHT<HT_SOBEL_X_3X3_08S, SobelXFilterHT>(*this, 2, aSrc, mBuffer0, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  SobelYFilterHT::~SobelYFilterHT()
  {
    switch (mType)
    {
    case 2:{
      delete (apexcv_filter<HT_SOBEL_Y_3X3_08S> *)mpProcess;
    } break;
    }
  }


  APEXCV_LIB_RESULT SobelYFilterHT::ReconnectIO(vsdk::SUMat&    aSrc, /*!< Source Image buffer. */
    vsdk::SUMat&    aDst /*!< Destination Image buffer. */)
  {
    if (0 == mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {
      if (2 == mType)
      {
        apexcv_filter<HT_SOBEL_Y_3X3_08S> *pf = (apexcv_filter<HT_SOBEL_Y_3X3_08S> *)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INTERNAL_ERROR;
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

  APEXCV_LIB_RESULT SobelYFilterHT::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    int    aWindowSize, /* Window Size, 3 */
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    if (aWindowSize != 3)
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_8SC1)
    {

      mBuffer0 = vsdk::SUMat(aSrc.rows, aSrc.cols, VSDK_CV_16SC1);
      if (mBuffer0.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      return ApexCVHostUtilFilter::WInitializeSobelHT<HT_SOBEL_Y_3X3_08S, SobelYFilterHT>(*this, 2, aSrc, mBuffer0, aDst);
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }





}  /* namespace apex */
