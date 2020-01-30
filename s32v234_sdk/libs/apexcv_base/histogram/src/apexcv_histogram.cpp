/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2016-2017 NXP Semiconductor;
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
*  \file apexcv_histogram.cpp
*  \brief APEX-CV interface for histogram
***********************************************************************************/

#include <apexcv_histogram.h>
#include <sumat.hpp>
#include <math.h>

#ifndef APEX2_EMULATE
#include <acf_process_apu.h>
#include <HISTOGRAM.hpp>
#include <MEAN.hpp>
#include <MEANSTDDEV.hpp>
#else
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#include "histogram_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(HISTOGRAM, histogram_apu_process_desc);
#include "mean_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(MEAN, mean_apu_process_desc);
#include "meanstddev_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(MEANSTDDEV, meanstddev_apu_process_desc);
#endif


#include "graph_templates.h"

#include <apexcv_error_codes.hpp>
#include <../graphs/histogram_graph_names.h>



namespace apexcv{


  Histogram::~Histogram()
  {
    if (0 != mType){
      delete (HISTOGRAM *)mpProcess;
    }
  }


  APEXCV_LIB_RESULT Histogram::Initialize(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    uint32_t retVal;

    if (0 != mType){
      retVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
      goto exit;
    }

    if (aSrc.type() == VSDK_CV_8UC1 &&
      aDst.type() == VSDK_CV_32SC1)
    {

      HISTOGRAM *pf = new HISTOGRAM;
      if (NULL == pf){
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));

      int lRetVal = 0;
      lRetVal |= pf->Initialize();
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto exit;
      }
      lRetVal |= pf->ConnectIO(HISTOGR_IN_IMG, aSrc);
      lRetVal |= pf->ConnectIO(HISTOGR_OUT_HISTO, aDst);
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto exit;
      }
      mType = 1;
      retVal = APEXCV_SUCCESS;
      goto exit;
    }
    else{
      retVal = APEXCV_ERROR_INVALID_ARGUMENT;
      goto exit;
    }

    exit:
    return retVal;
  }


  APEXCV_LIB_RESULT Histogram::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    uint32_t retVal;
    switch (mType)
    {
    case 0:
    {
      retVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    }
    case 1:
    {
      if (aSrc.type() == VSDK_CV_8UC1 && aDst.type() == VSDK_CV_32SC1)
      {
        HISTOGRAM *pf = (HISTOGRAM*)mpProcess;
        int lRetVal = 0;
        lRetVal |= pf->ConnectIO(HISTOGR_IN_IMG, aSrc);
        lRetVal |= pf->ConnectIO(HISTOGR_OUT_HISTO, aDst);
        if (0 != lRetVal)
        {
          retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        else
        {
          retVal = APEXCV_SUCCESS;
        }
      }
      else
      {
        retVal = APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    }
    default:
      retVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
    return retVal;
  }

  Mean::~Mean()
  {
    if (0 != mType)
    {
      delete (MEAN*) mpProcess;
    }
  }

  APEXCV_LIB_RESULT Mean::Initialize(
    vsdk::SUMat& aSrc)
  {
    uint32_t retVal;

    if (0 != mType)
    {
      retVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
      goto exit;
    }

    if (aSrc.type() == VSDK_CV_8UC1)
    {
      MEAN *pf = new MEAN;
      int32_t lRetVal = 0;

      if (NULL == pf)
      {
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }

      lRetVal |= pf-> Initialize();
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto exit;
      }

      mDst = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mDst.empty())
      {
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }

      lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
      lRetVal |= pf->ConnectIO(GR_OUT_0, mDst);

      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto exit;
      }

      mNumel = aSrc.rows * aSrc.cols;

      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      mType = 1;

      retVal = APEXCV_SUCCESS;
    }
    else
    {
      retVal = APEXCV_ERROR_INVALID_ARGUMENT;
    }
    exit:
    return retVal;
  }

  APEXCV_LIB_RESULT Mean::ReconnectIO(
    vsdk::SUMat& aSrc)
  {
    uint32_t retVal;

    switch (mType)
    {
    case 0:
      retVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    case 1:
      if (aSrc.type() == VSDK_CV_8UC1)
      {
        MEAN *pf = (MEAN*)mpProcess;
        int32_t lRetVal = 0;

        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);

        if (0 != lRetVal)
        {
          delete pf;
          mpProcess = nullptr;
          retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        }
        else
        {
          mNumel = aSrc.rows * aSrc.cols;
          retVal = APEXCV_SUCCESS;
        }

      }
      else
      {
        retVal = APEXCV_ERROR_INVALID_ARGUMENT;
      }
      break;
    default:
      retVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
    return retVal;
  }

  APEXCV_LIB_RESULT Mean::Process(
    float& aMean)
  {
    uint32_t retVal;

    switch (mType)
    {
    case 0:
      {
        retVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
        break;
      }
    case 1:
      {
        int32_t lRetVal = 0;

        lRetVal |= ApexcvHostBaseClass::Process();
        if (0 != lRetVal)
        {
          retVal = lRetVal;
        }
        else
        {
          vsdk::SMat dstMat = mDst.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
          aMean = (float)(((double)dstMat.at<uint32_t>(0, 0))/mNumel);
          retVal = APEXCV_SUCCESS;
        }
        break;
      }
    default:
      {
        retVal = APEXCV_ERROR_INTERNAL_ERROR;
        break;
      }
    }

    return retVal;
  }

  MeanStddev::~MeanStddev()
  {
    if (0 != mType)
    {
      delete (MEANSTDDEV*) mpProcess;
    }
  }

  APEXCV_LIB_RESULT MeanStddev::Initialize(
    vsdk::SUMat& aSrc)
  {
    uint32_t retVal;
    if (0 != mType)
    {
      retVal = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
      goto exit;
    }
    if (aSrc.type() == VSDK_CV_8UC1)
    {
      MEANSTDDEV *pf = new MEANSTDDEV;
      int32_t lRetVal = 0;
      int32_t lInputChunkWidth = 0;
      int32_t lInputChunkHeight = 0;
      uint8_t lScale = 0;
      vsdk::SUMat nDst;

      mTileWidthInChunks = 0;
      if (NULL == pf)
      {
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));

      lRetVal |= pf-> Initialize();
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_INITIALIZE;
        goto exit;
      }
      // Caculate size of SUMat Source
      mNumel = aSrc.rows * aSrc.cols;

      // Connect SUMat Source to Graph
      lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
      // For port GR_IN_0, select the scenario where chunk width and height to be left to ACF
      lRetVal |= pf->SelectScenario(GR_IN_0, 0, 0);
      // Return the the chunk width and height associated with port GR_IN0 to lInputChunkWidth and lInputChunkHeight
      lRetVal |= pf->QueryPortChunkSize(GR_IN_0, lInputChunkWidth, lInputChunkHeight);

      if (0 == lInputChunkWidth)
      {
        retVal = APEXCV_ERROR_INTERNAL_ERROR;
        goto exit;
      }
      mTileWidthInChunks = (aSrc.cols + lInputChunkWidth - 1) / lInputChunkWidth;
      lScale = (uint8_t)ceil(log2((double)mNumel*255*255/mTileWidthInChunks/UINT32_MAX));

      mDst = vsdk::SUMat(aSrc.rows, aSrc.cols, VSDK_CV_32SC1);
      if (mDst.empty())
      {
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }
      // Creat a rectangle to collect SUMat from aDst
      vsdk::Rect lRect(0,0,mTileWidthInChunks,2);
      // Collect SUMat from aDst
      nDst = vsdk::SUMat(mDst, lRect);
      mScale = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (nDst.empty() || mScale.empty())
      {
        retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        goto exit;
      }
      lRetVal |= pf->ConnectIO(GR_IN_1, mScale);
      lRetVal |= pf->ConnectIO(GR_OUT_0, nDst);
      if (0 != lRetVal)
      {
        delete pf;
        mpProcess = nullptr;
        retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
        goto exit;
      }
      {
        vsdk::SMat scaleMat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
        vsdk::SMat dstMat = nDst.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
        scaleMat.at<uint8_t>(0, 0) = lScale;
        for (int i = 0; i < dstMat.total(); i++)
        {
          dstMat.at<uint32_t>(i) = 0x00;
        }

      }

      mType = 1;
      retVal = APEXCV_SUCCESS;
    }
    else
    {
      retVal = APEXCV_ERROR_INVALID_ARGUMENT;
    }

    exit:
    return retVal;
  }

  APEXCV_LIB_RESULT MeanStddev::ReconnectIO(
    vsdk::SUMat& aSrc)
  {
    uint32_t retVal;

    if (mType != 1)
    {
      retVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    else
    {
      if (aSrc.type() == VSDK_CV_8UC1)
      {
        MEANSTDDEV *pf = (MEANSTDDEV*)mpProcess;
        int32_t lRetVal = 0;
        int32_t lInputChunkWidth = 0;
        int32_t lInputChunkHeight = 0;
        uint8_t lScale = 0;
        vsdk::SUMat nDst;
        mNumel = aSrc.rows * aSrc.cols;

        mTileWidthInChunks = 0;

        lRetVal |= pf->ConnectIO(GR_IN_0, aSrc);
        lRetVal |= pf->SelectScenario(GR_IN_0, 0, 0);
        lRetVal |= pf->QueryPortChunkSize(GR_IN_0, lInputChunkWidth, lInputChunkHeight);

        mTileWidthInChunks = (aSrc.cols + lInputChunkWidth - 1) / lInputChunkWidth;
        lScale = (uint8_t)ceil(log2((double)mNumel*255*255/mTileWidthInChunks/UINT32_MAX));

        // Creat a rectangle to collect SUMat from aDst
        vsdk::Rect lRect(0,0,mTileWidthInChunks,2);
        if (mDst.empty())
        {
          retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        else
        {
          // Collect SUMat from mDst
          nDst = vsdk::SUMat(mDst, lRect);
          mScale = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
          if (nDst.empty() && mScale.empty())
          {
            retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
          }
          else
          {
            lRetVal |= pf->ConnectIO(GR_OUT_0, nDst);
            if (0 != lRetVal)
            {
              delete pf;
              mpProcess = nullptr;
              retVal = APEXCV_ERROR_ACF_PROC_CONNECTIO;
            }
            else
            {
              vsdk::SMat scaleMat = mScale.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
              vsdk::SMat dstMat = nDst.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
              scaleMat.at<uint8_t>(0, 0) = lScale;
              for (int i = 0; i < dstMat.total(); i++)
              {
                dstMat.at<uint32_t>(i) = 0x00;
              }

              retVal = APEXCV_SUCCESS;
            }
          }
        }
      }
      else
      {
        retVal = APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    return retVal;
  }

  APEXCV_LIB_RESULT MeanStddev::Process(
    float& aMean, float& aStddev)
  {
    uint32_t retVal;

    switch (mType)
    {
    case 0:
      retVal = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
      break;
    case 1:
    {
      int32_t lRetVal = 0;
      uint32_t lSum = 0;
      uint64_t lSqrSum = 0;
      uint8_t lScale = 0;
      vsdk::SUMat nDst;

      lRetVal |= ApexcvHostBaseClass::Process();
      if (0 != lRetVal)
      {
        retVal = lRetVal;
      }
      else
      {
        // Creat a rectangle to collect SUMat from aDst
        vsdk::Rect lRect(0,0,mTileWidthInChunks,2);
        nDst = vsdk::SUMat(mDst, lRect);
        if (nDst.empty())
        {
          retVal = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        else
        {
          vsdk::SMat dstMat = nDst.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
          vsdk::SMat scaleMat = mScale.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
          lScale = scaleMat.at<uint8_t>(0, 0);

          for (int32_t i = 0; i < dstMat.cols; i++)
          {
            lSum += dstMat.at<uint32_t>(0, i);
            lSqrSum += (uint64_t)(dstMat.at<uint32_t>(1, i));
          }

          aMean = (float)(((double)lSum)/mNumel);
          aStddev = (float)sqrt((double)(lSqrSum << lScale)/mNumel - aMean*aMean);

          retVal = APEXCV_SUCCESS;
        }
      }
      break;
    }
    default:
      retVal = APEXCV_ERROR_INTERNAL_ERROR;
      break;
    }
    return retVal;
  }
}
