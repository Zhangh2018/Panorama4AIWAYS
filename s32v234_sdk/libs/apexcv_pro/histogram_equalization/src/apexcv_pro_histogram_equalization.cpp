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
*  \file histogram_equalization.cpp
*  \brief Host-ACF interface for histogram equalization
***********************************************************************************/

#include <apexcv_pro_histogram_equalization.h>
#include <histogram_equalization_graph_names.h>
#include <histogram_graph_names.h>

#ifndef APEX2_EMULATE
#include <sumat.hpp>
#include <HISTOGRAM_EQUALIZATION.hpp>
#include <HISTOGRAM.hpp>
#else
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;

#include <histogram_equalization_apu_process_desc.hpp>
REGISTER_PROCESS_TYPE(HISTOGRAM_EQUALIZATION, histogram_equalization_apu_process_desc);
#include <histogram_apu_process_desc.hpp>
REGISTER_PROCESS_TYPE(HISTOGRAM, histogram_apu_process_desc);
#endif

// Activate for debugging
//#define APECV_DEBUG_PRINT 1


namespace apexcv{

  HistogramEqualization::~HistogramEqualization(){
    if (mType)
    {
      delete (HISTOGRAM*)mpProcessHist;
      delete (HISTOGRAM_EQUALIZATION*)mpProcessHE;
      mType = false;
    }
  }


  APEXCV_LIB_RESULT HistogramEqualization::Initialize(
    vsdk::SUMat&    aSrc,        /*!< 8-bit grayscale source image. */
    vsdk::SUMat&    aDst /*!< 8-bit grayscale destination image. */)
  {
    if (mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }
    if (
      (aSrc.type() == VSDK_CV_8UC1) &&
      (aDst.type() == VSDK_CV_8UC1) )
    {
      // create container for the histogram
      if (mSrcHistogram.empty())
      {
        mSrcHistogram = vsdk::SUMat(1, 256, VSDK_CV_32SC1);
        if (mSrcHistogram.empty())
        {
#ifdef APECV_DEBUG_PRINT
            printf("Error returned %s (%d)\n", __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
      }
      {
        vsdk::SMat mat = mSrcHistogram.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
        if (nullptr == mat.data)
        {
#ifdef APECV_DEBUG_PRINT
          printf("Error returned %s (%d)\n", __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
      }
      // create container for the pixel count
      if (mNumPixels.empty())
      {
        mNumPixels = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
        if (mNumPixels.empty())
        {
#ifdef APECV_DEBUG_PRINT
          printf("Error returned %s (%d)\n", __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
      }
      {
        vsdk::SMat mat = mNumPixels.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
        if (nullptr == mat.data)
        {
#ifdef APECV_DEBUG_PRINT
          printf("Error returned %s (%d)\n", __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mat.at<int32_t>(0u) = aSrc.rows * aSrc.cols;
      }
      {
        HISTOGRAM *pf = new HISTOGRAM;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        int lRetVal = 0;
        lRetVal |= pf->Initialize();
        if (lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_INITIALIZE;
        }
        lRetVal |= pf->ConnectIO(HISTOGR_IN_IMG, aSrc);
        lRetVal |= pf->ConnectIO(HISTOGR_OUT_HISTO, mSrcHistogram);

        if (lRetVal)
        {
#ifdef APECV_DEBUG_PRINT
          printf("Error %d returned %s (%d)\n", lRetVal, __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        mpProcessHist = pf;
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("HISTOGRAM", pf);
          lInfo.push_PortName(HISTOGR_IN_IMG);
          mvInfoClusters.push_back(lInfo);
        }
      }
      {
        // allocate object
        HISTOGRAM_EQUALIZATION *pf = new HISTOGRAM_EQUALIZATION;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        int lRetVal = 0;
        lRetVal |= pf->Initialize();
        if (lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_INITIALIZE;
        }
        lRetVal |= pf->ConnectIO(HISTO_EQU_NUM_PIXELS, mNumPixels);
        lRetVal |= pf->ConnectIO(HISTO_EQU_OUT_IMG, aDst);
        lRetVal |= pf->ConnectIO(HISTO_EQU_IN_IMG, aSrc);
        lRetVal |= pf->ConnectIO(HISTO_EQU_HISTO, mSrcHistogram);


        if (lRetVal)
        {
#ifdef APECV_DEBUG_PRINT
          printf("Error %d returned %s (%d)\n", lRetVal, __FILE__, __LINE__);
#endif
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        mpProcessHE = pf;
        {
          ApexcvHostBaseBaseClass::InfoCluster lInfo;
          lInfo.set_ACF("HISTOGRAM_EQUALIZATION", pf);
          lInfo.push_PortName(HISTO_EQU_IN_IMG);
          mvInfoClusters.push_back(lInfo);
        }
      }
      mType = true;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }



  APEXCV_LIB_RESULT HistogramEqualization::Process()
  {
    if (!mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    
    {
      HISTOGRAM *pf = (HISTOGRAM*)mpProcessHist;
      int lRetVal = 0;
      lRetVal |= pf->Start();
      lRetVal |= pf->Wait();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    {
      HISTOGRAM_EQUALIZATION *pf = (HISTOGRAM_EQUALIZATION*)mpProcessHE;
      int lRetVal = 0;
      lRetVal |= pf->Start();
      lRetVal |= pf->Wait();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
    }

    return APEXCV_SUCCESS;
  }



  APEXCV_LIB_RESULT HistogramEqualization::ReconnectIO(
    vsdk::SUMat&    aSrc,        /*!< 8-bit grayscale source image. */
    vsdk::SUMat&    aDst /*!< 8-bit grayscale destination image. */)
  {
    if (!mType)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }


    if (
      (aSrc.type() == VSDK_CV_8UC1) &&
      (aDst.type() == VSDK_CV_8UC1) )
    {

      {
        HISTOGRAM_EQUALIZATION *pf = (HISTOGRAM_EQUALIZATION*)mpProcessHist;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(HISTOGR_IN_IMG, aSrc);
        if (lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
      }

      {
        vsdk::SMat mat = mNumPixels.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
        if (nullptr == mat.data)
        {
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mat.at<int32_t>(0u) = aSrc.rows * aSrc.cols;
      }

      {
        HISTOGRAM_EQUALIZATION *pf = (HISTOGRAM_EQUALIZATION*)mpProcessHE;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(HISTO_EQU_OUT_IMG, aDst);
        lRetVal |= pf->ConnectIO(HISTO_EQU_IN_IMG, aSrc);
        if (lRetVal)
        {
          return APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
      }
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }
} // namespace apecv
