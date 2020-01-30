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

#include <apexcv_base_color_conversion.h>
#include <apexcv_color_int.h>
#include <stdint.h>


#ifndef APEX2_EMULATE
#include <acf_process_apu.h>

#include <RGB565_TO_RGB888X.hpp>
#include <RGB888X_TO_RGB565.hpp>
#include <RGB888X_TO_YUV.hpp>
#include <RGB888X_TO_Y.hpp>
#include <RGB888_TO_GREY.hpp>
#include <BGR888_TO_GREY.hpp>
#include <GREY_TO_RGB888.hpp>
#include <HT_RGB888X_TO_Y.hpp>
#include <EXTRACT_CHANNEL_2CH.hpp>
#include <EXTRACT_CHANNEL_3CH.hpp>
#include <EXTRACT_CHANNEL_4CH.hpp>
#include <INSERT_CHANNEL_2CH.hpp>
#include <INSERT_CHANNEL_3CH.hpp>
#include <INSERT_CHANNEL_4CH.hpp>
#include <SPLIT_CHANNEL_2CH.hpp>
#include <SPLIT_CHANNEL_3CH.hpp>
#include <SPLIT_CHANNEL_4CH.hpp>
#include <MERGE_CHANNEL_2CH.hpp>
#include <MERGE_CHANNEL_3CH.hpp>
#include <MERGE_CHANNEL_4CH.hpp>
#include <DOWN_CONVERT_16S.hpp>
#include <UP_CONVERT_08U.hpp>

#else
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;

#include "rgb565_to_rgb888x_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB565_TO_RGB888X, rgb565_to_rgb888x_apu_process_desc)
#include "rgb888x_to_rgb565_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_RGB565, rgb888x_to_rgb565_apu_process_desc);
#include "rgb888x_to_yuv_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_YUV, rgb888x_to_yuv_apu_process_desc)
#include "rgb888x_to_y_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_Y, rgb888x_to_y_apu_process_desc)
#include "rgb888_to_grey_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888_TO_GREY, rgb888_to_grey_apu_process_desc)
#include "bgr888_to_grey_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(BGR888_TO_GREY, bgr888_to_grey_apu_process_desc)
#include "grey_to_rgb888_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(GREY_TO_RGB888, grey_to_rgb888_apu_process_desc)
#include "extract_channel_2ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(EXTRACT_CHANNEL_2CH, extract_channel_2ch_apu_process_desc)
#include "extract_channel_3ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(EXTRACT_CHANNEL_3CH, extract_channel_3ch_apu_process_desc)
#include "extract_channel_4ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(EXTRACT_CHANNEL_4CH, extract_channel_4ch_apu_process_desc)
#include "insert_channel_2ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(INSERT_CHANNEL_2CH, insert_channel_2ch_apu_process_desc)
#include "insert_channel_3ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(INSERT_CHANNEL_3CH, insert_channel_3ch_apu_process_desc)
#include "insert_channel_4ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(INSERT_CHANNEL_4CH, insert_channel_4ch_apu_process_desc)
#include "ht_rgb888x_to_y_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(HT_RGB888X_TO_Y, ht_rgb888x_to_y_apu_process_desc)
#include "split_channel_2ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(SPLIT_CHANNEL_2CH, split_channel_2ch_apu_process_desc)
#include "split_channel_3ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(SPLIT_CHANNEL_3CH, split_channel_3ch_apu_process_desc)
#include "split_channel_4ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(SPLIT_CHANNEL_4CH, split_channel_4ch_apu_process_desc)
#include "merge_channel_2ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(MERGE_CHANNEL_2CH, merge_channel_2ch_apu_process_desc)
#include "merge_channel_3ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(MERGE_CHANNEL_3CH, merge_channel_3ch_apu_process_desc)
#include "merge_channel_4ch_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(MERGE_CHANNEL_4CH, merge_channel_4ch_apu_process_desc)
#include "down_convert_16s_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(DOWN_CONVERT_16S, down_convert_16s_apu_process_desc)
#include "up_convert_08u_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(UP_CONVERT_08U, up_convert_08u_apu_process_desc)
#endif



namespace apexcv{

  ColorConverter::~ColorConverter()
  {
    switch (mType){
    case 1:{
      delete (apexcv_fct<RGB565_TO_RGB888X>*)mpProcess;
    }break;
    case 2:{
      delete (apexcv_fct<RGB888X_TO_RGB565>*)mpProcess;
    }break;
    case 3:{
      delete (apexcv_fct<RGB888X_TO_YUV>*)mpProcess;
    }break;
    case 4:{
      delete (apexcv_fct<RGB888X_TO_Y>*)mpProcess;
    }break;
    case 5:{
      delete (apexcv_fct<RGB888_TO_GREY>*)mpProcess;
    }break;
    case 6:{
      delete (apexcv_fct<BGR888_TO_GREY>*)mpProcess;
    }break;
    case 7:{
      delete (apexcv_fct<GREY_TO_RGB888>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT ColorConverter::ReconnectIO(
    vsdk::SUMat&    aSrc  /*!< Source memory buffer. */,
    vsdk::SUMat&    aDst  /*!< Destination memory buffer. */)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 1:{
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        apexcv_fct<RGB565_TO_RGB888X>* pf = (apexcv_fct<RGB565_TO_RGB888X>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 2:{
      if (aSrc.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<RGB888X_TO_RGB565>*pf = (apexcv_fct<RGB888X_TO_RGB565>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_8UC4)
      {
        apexcv_fct<RGB888X_TO_YUV>*pf = (apexcv_fct<RGB888X_TO_YUV>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<RGB888X_TO_Y>*pf = (apexcv_fct<RGB888X_TO_Y>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 5:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<RGB888_TO_GREY>*pf = (apexcv_fct<RGB888_TO_GREY>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 6:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<BGR888_TO_GREY>*pf = (apexcv_fct<BGR888_TO_GREY>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 7:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC3)
      {
        apexcv_fct<GREY_TO_RGB888>*pf = (apexcv_fct<GREY_TO_RGB888>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }break;
    }
  }


  APEXCV_LIB_RESULT ColorConverter::Initialize(
    vsdk::SUMat&    aSrc  /*!< Source memory buffer. */,
    ConversionType    aCT  /*!< Color conversion type. See \ref ConversionType */,
    vsdk::SUMat&    aDst  /*!< Destination memory buffer. */)
  {
    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    switch (aCT){
    case ColorConverter::eRGB565_TO_RGB888X:{
      if (aSrc.type() == VSDK_CV_16UC1 &&
        aDst.type() == VSDK_CV_32SC1)
      {
        apexcv_fct<RGB565_TO_RGB888X>* pf = new apexcv_fct<RGB565_TO_RGB888X>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 1;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;
    case ColorConverter::eRGB888X_TO_RGB565:{
      if (aSrc.type() == VSDK_CV_32SC1 &&
        aDst.type() == VSDK_CV_16UC1)
      {
        apexcv_fct<RGB888X_TO_RGB565>* pf = new apexcv_fct<RGB888X_TO_RGB565>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 2;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;
    case ColorConverter::eRGB888X_TO_YUV:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_8UC4)
      {
        apexcv_fct<RGB888X_TO_YUV>* pf = new apexcv_fct<RGB888X_TO_YUV>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 3;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;

    case ColorConverter::eRGB888X_TO_Y:{
      // cannot be initialized this way, need other parameters, see other Initialize
      return APEXCV_ERROR_INVALID_ARGUMENT;
    } break;

    case ColorConverter::eRGB888_TO_GREY:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<RGB888_TO_GREY>* pf = new apexcv_fct<RGB888_TO_GREY>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 5;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;

    case ColorConverter::eBGR888_TO_GREY:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<BGR888_TO_GREY>* pf = new apexcv_fct<BGR888_TO_GREY>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 6;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;

    case ColorConverter::eGREY_TO_RGB888:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC3)
      {
        apexcv_fct<GREY_TO_RGB888>* pf = new apexcv_fct<GREY_TO_RGB888>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize(aSrc, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 7;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    } break;

    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }

  APEXCV_LIB_RESULT ColorConverter::SetFactors(
    int    aR2YFactor  /*!< Conversion factor for red used with RGB888X_TO_Y */,
    int    aG2YFactor  /*!< Conversion factor for green used with RGB888X_TO_Y */,
    int    aB2YFactor  /*!< Conversion factor for blue used with RGB888X_TO_Y*/)
  {
    /* should work even if not initialized */
    if (mR2Y.empty())
    {
      mR2Y = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mR2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mG2Y.empty())
    {
      mG2Y = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mG2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mB2Y.empty())
    {
      mB2Y = vsdk::SUMat(1, 1, VSDK_CV_32SC1);
      if (mB2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }

    {
      vsdk::SMat mat = mR2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<int32_t>(0u) = aR2YFactor;
    }
    {
      vsdk::SMat mat = mG2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<int32_t>(0u) = aG2YFactor;
    }
    {
      vsdk::SMat mat = mB2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<int32_t>(0u) = aB2YFactor;
    }
    return APEXCV_SUCCESS;
  }




  APEXCV_LIB_RESULT ColorConverter::Initialize(
    vsdk::SUMat&    aSrc  /*!< Source memory buffer. */,
    ConversionType    aCT  /*!< Color conversion type. See \ref ConversionType */,
    int    aR2YFactor  /*!< Conversion factor for red used with RGB888X_TO_Y */,
    int    aG2YFactor  /*!< Conversion factor for green used with RGB888X_TO_Y */,
    int     aB2YFactor  /*!< Conversion factor for blue used with RGB888X_TO_Y*/,
    vsdk::SUMat&    aDst  /*!< Destination memory buffer. */)
  {
    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    switch (aCT){
    case ColorConverter::eRGB888X_TO_Y:
    {
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        APEXCV_LIB_RESULT lRetVal;
        lRetVal = SetFactors(aR2YFactor, aG2YFactor, aB2YFactor);
        if (APEXCV_SUCCESS != lRetVal)
        {
          return lRetVal;
        }

        apexcv_fct<RGB888X_TO_Y>* pf = new apexcv_fct<RGB888X_TO_Y>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        lRetVal = pf->Initialize_RGB(aSrc, mR2Y, mG2Y, mB2Y, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 4;
        return APEXCV_SUCCESS;

      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }
    break;
    default:
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }




  ColorConverterHT::~ColorConverterHT()
  {
    switch (mType){
    case 4:{
      delete (apexcv_fct<HT_RGB888X_TO_Y>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT ColorConverterHT::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 4:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<HT_RGB888X_TO_Y>*pf = (apexcv_fct<HT_RGB888X_TO_Y>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
  }


  APEXCV_LIB_RESULT ColorConverterHT::SetFactors(
    uint8_t    aR2YFactor  /*!< Conversion factor for red used with HT_RGB888X_TO_Y */,
    uint8_t    aG2YFactor  /*!< Conversion factor for green used with HT_RGB888X_TO_Y */,
    uint8_t    aB2YFactor  /*!< Conversion factor for blue used with HT_RGB888X_TO_Y*/,
    uint16_t    aShiftFactor /*!< Shift factor. Use 0 by default. Used in HT_RGB888X_TO_Y. */)
  {
    /* should work even if not initialized */
    if (mR2Y.empty())
    {
      mR2Y = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mR2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mG2Y.empty())
    {
      mG2Y = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mG2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mB2Y.empty())
    {
      mB2Y = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mB2Y.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }
    if (mShiftFactor.empty())
    {
      mShiftFactor = vsdk::SUMat(1, 1, VSDK_CV_16UC1);
      if (mShiftFactor.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
    }

    {
      vsdk::SMat mat = mR2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<uint8_t>(0u) = aR2YFactor;
    }
    {
      vsdk::SMat mat = mG2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<uint8_t>(0u) = aG2YFactor;
    }
    {
      vsdk::SMat mat = mB2Y.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<uint8_t>(0u) = aB2YFactor;
    }
    {
      vsdk::SMat mat = mShiftFactor.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
      if (nullptr == mat.data)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mat.at<uint16_t>(0u) = aShiftFactor;
    }
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ColorConverterHT::Initialize(
    vsdk::SUMat&    aSrc  /*!< Source memory buffer. */,
    ConversionType    aCT  /*!< Color conversion type. See \ref ConversionType */,
    uint8_t    aR2YFactor  /*!< Conversion factor for red used with RGB888X_TO_Y */,
    uint8_t    aG2YFactor  /*!< Conversion factor for green used with RGB888X_TO_Y */,
    uint8_t    aB2YFactor  /*!< Conversion factor for blue used with RGB888X_TO_Y*/,
    uint16_t    aShiftFactor /*!< Shift factor. Use 0 by default. Used in HT_RGB888X_TO_Y. */,
    vsdk::SUMat&    aDst  /*!< Destination memory buffer. */)
  {
    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    }

    switch (aCT){
    case ColorConverterHT::eHT_RGB888X_TO_Y:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        APEXCV_LIB_RESULT lRetVal;
        lRetVal = SetFactors(aR2YFactor, aG2YFactor, aB2YFactor, aShiftFactor);
        if (APEXCV_SUCCESS != lRetVal)
        {
          return lRetVal;
        }

        apexcv_fct<HT_RGB888X_TO_Y>* pf = new apexcv_fct<HT_RGB888X_TO_Y>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        lRetVal = pf->Initialize_KABCR(aSrc, mR2Y, mG2Y, mB2Y, mShiftFactor, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 4;
        return APEXCV_SUCCESS;

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





  ExtractChannel::~ExtractChannel()
  {
    switch (mType){
    case 2:{
      delete (apexcv_fct<EXTRACT_CHANNEL_2CH>*)mpProcess;
    }break;
    case 3:{
      delete (apexcv_fct<EXTRACT_CHANNEL_3CH>*)mpProcess;
    }break;
    case 4:{
      delete (apexcv_fct<EXTRACT_CHANNEL_4CH>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT ExtractChannel::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      if (aSrc.type() == VSDK_CV_8UC2 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<EXTRACT_CHANNEL_2CH>* pf = (apexcv_fct<EXTRACT_CHANNEL_2CH>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<EXTRACT_CHANNEL_3CH>* pf = (apexcv_fct<EXTRACT_CHANNEL_3CH>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<EXTRACT_CHANNEL_4CH>* pf = (apexcv_fct<EXTRACT_CHANNEL_4CH>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT ExtractChannel::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. Accepted buffer types are 8UC2, 8UC3, 8UC4 */,
    int    aChannelIndex /*!< Index of the channel to extract. Starts at 1. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. Accepted buffer type is 8UC1 */)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if ((aSrc.type() == VSDK_CV_8UC2 || aSrc.type() == VSDK_CV_8UC3 || aSrc.type() == VSDK_CV_8UC4) &&
      aDst.type() == VSDK_CV_8UC1)
    {

      if ((aChannelIndex <= 0) || (aChannelIndex > aSrc.channels()))
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;// invalid input
      }

      mChannelIndex = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mChannelIndex.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;// memory allocation error
      }

      {// Set channel index value in SUMat
        vsdk::SMat mChannelMat = mChannelIndex.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
        mChannelMat.at<uint8_t>(0u) = aChannelIndex - 1;
      }

      if (2 == aSrc.channels())
      {
        apexcv_fct<EXTRACT_CHANNEL_2CH>* pf = new apexcv_fct<EXTRACT_CHANNEL_2CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize1I1O_CH_ID(aSrc, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 2;
        return APEXCV_SUCCESS;
      }
      if (3 == aSrc.channels())
      {
        apexcv_fct<EXTRACT_CHANNEL_3CH>* pf = new apexcv_fct<EXTRACT_CHANNEL_3CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize1I1O_CH_ID(aSrc, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 3;
        return APEXCV_SUCCESS;
      }
      if (4 == aSrc.channels())
      {
        apexcv_fct<EXTRACT_CHANNEL_4CH>* pf = new apexcv_fct<EXTRACT_CHANNEL_4CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize1I1O_CH_ID(aSrc, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 4;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }






  InsertChannel::~InsertChannel()
  {
    switch (mType){
    case 2:{
      delete (apexcv_fct<INSERT_CHANNEL_2CH>*)mpProcess;
    }break;
    case 3:{
      delete (apexcv_fct<INSERT_CHANNEL_3CH>*)mpProcess;
    }break;
    case 4:{
      delete (apexcv_fct<INSERT_CHANNEL_4CH>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT InsertChannel::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC2)
      {
        apexcv_fct<INSERT_CHANNEL_2CH>* pf = (apexcv_fct<INSERT_CHANNEL_2CH>*)mpProcess;
        return pf->ReconnectIO_2I1O(aSrc, aDst, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC3)
      {
        apexcv_fct<INSERT_CHANNEL_3CH>* pf = (apexcv_fct<INSERT_CHANNEL_3CH>*)mpProcess;
        return pf->ReconnectIO_2I1O(aSrc, aDst, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:{
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_8UC4)
      {
        apexcv_fct<INSERT_CHANNEL_4CH>* pf = (apexcv_fct<INSERT_CHANNEL_4CH>*)mpProcess;
        return pf->ReconnectIO_2I1O(aSrc, aDst, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT InsertChannel::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. Accepted buffer types are 8UC2, 8UC3, 8UC4 */,
    int    aChannelIndex /*!< Index of the channel to insert. Starts at 1. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. Accepted buffer type is 8UC1 */)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if ((aDst.type() == VSDK_CV_8UC2 || aDst.type() == VSDK_CV_8UC3 || aDst.type() == VSDK_CV_8UC4) &&
      aSrc.type() == VSDK_CV_8UC1)
    {

      if ((aChannelIndex <= 0) || (aChannelIndex > aDst.channels()))
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;// invalid input
      }

      mChannelIndex = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
      if (mChannelIndex.empty())
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;// memory allocation error
      }

      {// Set channel index value in SUMat
        vsdk::SMat mChannelMat = mChannelIndex.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
        mChannelMat.at<uint8_t>(0u) = aChannelIndex - 1;
      }

      if (2 == aDst.channels())
      {
        apexcv_fct<INSERT_CHANNEL_2CH>* pf = new apexcv_fct<INSERT_CHANNEL_2CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize2I1O_CH_ID(aSrc, aDst, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 2;
        return APEXCV_SUCCESS;
      }
      if (3 == aDst.channels())
      {
        apexcv_fct<INSERT_CHANNEL_3CH>* pf = new apexcv_fct<INSERT_CHANNEL_3CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize2I1O_CH_ID(aSrc, aDst, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 3;
        return APEXCV_SUCCESS;
      }
      if (4 == aDst.channels())
      {
        apexcv_fct<INSERT_CHANNEL_4CH>* pf = new apexcv_fct<INSERT_CHANNEL_4CH>;
        if (nullptr == pf){
          return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
        }
        mpProcess = pf;
        mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
        APEXCV_LIB_RESULT lRetVal = pf->Initialize2I1O_CH_ID(aSrc, aDst, mChannelIndex, aDst);
        if (APEXCV_SUCCESS != lRetVal)
        {// ERROR
          delete pf;
          mpProcess = nullptr;
          return lRetVal; //ACF init error
        }
        mType = 4;
        return APEXCV_SUCCESS;
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
      }
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }






  SplitChannel::~SplitChannel()
  {
    switch (mType){
    case 2:{
      delete (apexcv_fct<SPLIT_CHANNEL_2CH>*)mpProcess;
    }break;
    case 3:{
      delete (apexcv_fct<SPLIT_CHANNEL_3CH>*)mpProcess;
    }break;
    case 4:{
      delete (apexcv_fct<SPLIT_CHANNEL_4CH>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT SplitChannel::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst1,
    vsdk::SUMat&    aDst2)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      if (aSrc.type() == VSDK_CV_8UC2 &&
        aDst1.type() == VSDK_CV_8UC1 &&
        aDst2.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<SPLIT_CHANNEL_2CH>* pf = (apexcv_fct<SPLIT_CHANNEL_2CH>*)mpProcess;
        return pf->ReconnectIO_1I2O(aSrc, aDst1, aDst2);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 4:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT SplitChannel::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst1,
    vsdk::SUMat&    aDst2,
    vsdk::SUMat&    aDst3)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 3:{
      if (aSrc.type() == VSDK_CV_8UC3 &&
        aDst1.type() == VSDK_CV_8UC1 &&
        aDst2.type() == VSDK_CV_8UC1 &&
        aDst3.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<SPLIT_CHANNEL_3CH>* pf = (apexcv_fct<SPLIT_CHANNEL_3CH>*)mpProcess;
        return pf->ReconnectIO_1I3O(aSrc, aDst1, aDst2, aDst3);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }

  APEXCV_LIB_RESULT SplitChannel::ReconnectIO(
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst1,
    vsdk::SUMat&    aDst2,
    vsdk::SUMat&    aDst3,
    vsdk::SUMat&    aDst4)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 3:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 4:{
      if (aSrc.type() == VSDK_CV_8UC4 &&
        aDst1.type() == VSDK_CV_8UC1 &&
        aDst2.type() == VSDK_CV_8UC1 &&
        aDst3.type() == VSDK_CV_8UC1 &&
        aDst4.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<SPLIT_CHANNEL_4CH>* pf = (apexcv_fct<SPLIT_CHANNEL_4CH>*)mpProcess;
        return pf->ReconnectIO_1I4O(aSrc, aDst1, aDst2, aDst3, aDst4);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT SplitChannel::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. Accepted buffer types are 8UC2, 8UC3, 8UC4 */,
    vsdk::SUMat&    aDst1 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst2 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst3 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst4 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aSrc.type() == VSDK_CV_8UC4 && 
      aDst1.type() == VSDK_CV_8UC1 &&
      aDst2.type() == VSDK_CV_8UC1 &&
      aDst3.type() == VSDK_CV_8UC1 &&
      aDst4.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<SPLIT_CHANNEL_4CH>* pf = new apexcv_fct<SPLIT_CHANNEL_4CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize1I4O(aSrc, aDst1, aDst2, aDst3, aDst4);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 4;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }


  APEXCV_LIB_RESULT SplitChannel::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. Accepted buffer types are 8UC2, 8UC3, 8UC4 */,
    vsdk::SUMat&    aDst1 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst2 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst3 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aSrc.type() == VSDK_CV_8UC3 &&
      aDst1.type() == VSDK_CV_8UC1 &&
      aDst2.type() == VSDK_CV_8UC1 &&
      aDst3.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<SPLIT_CHANNEL_3CH>* pf = new apexcv_fct<SPLIT_CHANNEL_3CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize1I3O(aSrc, aDst1, aDst2, aDst3);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 3;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }


  APEXCV_LIB_RESULT SplitChannel::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. Accepted buffer types are 8UC2, 8UC3, 8UC4 */,
    vsdk::SUMat&    aDst1 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */,
    vsdk::SUMat&    aDst2 /*!< Destination memory buffer. Accepted buffer type is 8UC1 */)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aSrc.type() == VSDK_CV_8UC2 &&
      aDst1.type() == VSDK_CV_8UC1 &&
      aDst2.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<SPLIT_CHANNEL_2CH>* pf = new apexcv_fct<SPLIT_CHANNEL_2CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize1I2O(aSrc, aDst1, aDst2);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 2;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }






  MergeChannel::~MergeChannel()
  {
    switch (mType){
    case 2:{
      delete (apexcv_fct<MERGE_CHANNEL_2CH>*)mpProcess;
    }break;
    case 3:{
      delete (apexcv_fct<MERGE_CHANNEL_3CH>*)mpProcess;
    }break;
    case 4:{
      delete (apexcv_fct<MERGE_CHANNEL_4CH>*)mpProcess;
    }break;
    }
  }


  APEXCV_LIB_RESULT MergeChannel::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      if (aDst.type() == VSDK_CV_8UC2 &&
        aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MERGE_CHANNEL_2CH>* pf = (apexcv_fct<MERGE_CHANNEL_2CH>*)mpProcess;
        return pf->ReconnectIO_2I1O(aSrc1, aSrc2, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 3:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 4:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT MergeChannel::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aSrc3,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 3:{
      if (aDst.type() == VSDK_CV_8UC3 &&
        aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aSrc3.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MERGE_CHANNEL_3CH>* pf = (apexcv_fct<MERGE_CHANNEL_3CH>*)mpProcess;
        return pf->ReconnectIO_3I1O(aSrc1, aSrc2, aSrc3, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    case 4:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }

  APEXCV_LIB_RESULT MergeChannel::ReconnectIO(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aSrc3,
    vsdk::SUMat&    aSrc4,
    vsdk::SUMat&    aDst)
  {
    switch (mType){
    case 0:{
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }break;
    case 2:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 3:{
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }break;
    case 4:{
      if (aDst.type() == VSDK_CV_8UC4 &&
        aSrc1.type() == VSDK_CV_8UC1 &&
        aSrc2.type() == VSDK_CV_8UC1 &&
        aSrc3.type() == VSDK_CV_8UC1 &&
        aSrc4.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<MERGE_CHANNEL_4CH>* pf = (apexcv_fct<MERGE_CHANNEL_4CH>*)mpProcess;
        return pf->ReconnectIO_4I1O(aSrc1, aSrc2, aSrc3, aSrc4, aDst);
      }
      else{
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;
    default:
      return APEXCV_ERROR_INTERNAL_ERROR; // invalid input
      break;
    }
  }


  APEXCV_LIB_RESULT MergeChannel::Initialize(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aSrc3,
    vsdk::SUMat&    aSrc4,
    vsdk::SUMat&    aDst)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aDst.type() == VSDK_CV_8UC4 &&
      aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aSrc3.type() == VSDK_CV_8UC1 &&
      aSrc4.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<MERGE_CHANNEL_4CH>* pf = new apexcv_fct<MERGE_CHANNEL_4CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize4I1O(aSrc1, aSrc2, aSrc3, aSrc4, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 4;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }


  APEXCV_LIB_RESULT MergeChannel::Initialize(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aSrc3,
    vsdk::SUMat&    aDst)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aDst.type() == VSDK_CV_8UC3 &&
      aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1 &&
      aSrc3.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<MERGE_CHANNEL_3CH>* pf = new apexcv_fct<MERGE_CHANNEL_3CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize3I1O(aSrc1, aSrc2, aSrc3, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 3;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }


  APEXCV_LIB_RESULT MergeChannel::Initialize(
    vsdk::SUMat&    aSrc1,
    vsdk::SUMat&    aSrc2,
    vsdk::SUMat&    aDst)
  {

    if (0 != mType){
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;// already Initialized
    }

    if (aDst.type() == VSDK_CV_8UC2 &&
      aSrc1.type() == VSDK_CV_8UC1 &&
      aSrc2.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<MERGE_CHANNEL_2CH>* pf = new apexcv_fct<MERGE_CHANNEL_2CH>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pf->Initialize2I1O(aSrc1, aSrc2, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mType = 2;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT; //invalid input
    }
  }



  /***********************************************************************************/
  /*   ConvertDepth                                                                  */
  /*   mType = 1: down covert 16s                                                    */
  /*   mType = 2: up covert 08u                                                      */
  /***********************************************************************************/
  ConvertDepth::~ConvertDepth()
  {
    switch (mType)
    {
    case 1:
    {
      delete (apexcv_fct<DOWN_CONVERT_16S>*)mpProcess;
    }break;

    case 2:
    {
      delete (apexcv_fct<UP_CONVERT_08U>*)mpProcess;
    }break;
    default:
      break;
    }
  }


  APEXCV_LIB_RESULT ConvertDepth::ReconnectIO(
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
      if (aSrc.type() == VSDK_CV_16SC1 &&
        aDst.type() == VSDK_CV_8UC1)
      {
        apexcv_fct<DOWN_CONVERT_16S>*pf = (apexcv_fct<DOWN_CONVERT_16S>*)mpProcess;
        return pf->ReconnectIO(aSrc, aDst);
      }
      else
      {
        return APEXCV_ERROR_INVALID_ARGUMENT;
      }
    }break;

    case 2:
    {
      if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
      {
        apexcv_fct<UP_CONVERT_08U>*pf = (apexcv_fct<UP_CONVERT_08U>*)mpProcess;
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


  APEXCV_LIB_RESULT ConvertDepth::SetShift(
    const int32_t    acShift /*!< shift value. */)
  {
    if((acShift<0) || (acShift>=8))
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }

    if (mShift.empty())
    {
      mShift = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
    }

    /* noncached as it is for a single value */
    vsdk::SMat mat = mShift.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    mat.at<uint8_t>(0u) = (uint8_t)acShift;
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ConvertDepth::GetShift(
    int32_t      &aShift /*!< shift value. */)
  {
    if (mShift.empty())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    /* noncached as it is for a single value */
    vsdk::SMat mat = mShift.getMat(vsdk::ACCESS_READ | OAL_USAGE_NONCACHED);
    if (nullptr == mat.data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    aShift = mat.at<uint8_t>(0u);
    return APEXCV_SUCCESS;
  }


  APEXCV_LIB_RESULT ConvertDepth::SetPolicyType(
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


  APEXCV_LIB_RESULT ConvertDepth::GetPolicyType(
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


  APEXCV_LIB_RESULT ConvertDepth::Initialize(
    vsdk::SUMat&    aSrc /*!< Source memory buffer. */,
    const int32_t    acShift /*!< Shift value. */,
    vsdk::SUMat&    aDst /*!< Destination memory buffer. */)
  {
    if (0 != mType)
    {
      return APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;//object already Initialized
    }

    APEXCV_LIB_RESULT lRetVal;

    // Set value shift
    lRetVal = SetShift(acShift);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    // Set policy type
    lRetVal = SetPolicyType(apexcv::eConvertPolicy::SATURATE);
    if (APEXCV_SUCCESS != lRetVal)
    {
      return lRetVal;
    }

    if (aSrc.type() == VSDK_CV_16SC1 &&
      aDst.type() == VSDK_CV_8UC1)
    {
      apexcv_fct<DOWN_CONVERT_16S>* pf = new apexcv_fct<DOWN_CONVERT_16S>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      APEXCV_LIB_RESULT lRetVal = pf->Initialize3I1O(aSrc, mPolicy, mShift, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      mType = 1;
      return APEXCV_SUCCESS;
    }
    else if (aSrc.type() == VSDK_CV_8UC1 &&
        aDst.type() == VSDK_CV_16SC1)
    {
      apexcv_fct<UP_CONVERT_08U>* pf = new apexcv_fct<UP_CONVERT_08U>;
      if (nullptr == pf){
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      APEXCV_LIB_RESULT lRetVal = pf->Initialize2I1O(aSrc, mShift, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {// ERROR
        delete pf;
        mpProcess = nullptr;
        return lRetVal; //ACF init error
      }
      mpProcess = pf;
      mvInfoClusters.push_back(InfoCluster((void*)((ACF_Process_APU*)pf), GR_IN_0));
      mType = 2;
      return APEXCV_SUCCESS;
    }
    else
    {
      return APEXCV_ERROR_INVALID_ARGUMENT;
    }
  }

} // namespace apecv 

