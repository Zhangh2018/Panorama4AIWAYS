/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017 NXP 
* All Rights Reserved
*
******************************************************************************
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
***************************************************************************/
/*!*********************************************************************************
*  \file apexcv_filter.hpp
*  \brief Host-ACF interface for filter operations.
***********************************************************************************/

#ifndef APEXCVARITHMETICINT_H
#define APEXCVARITHMETICINT_H

#include <sumat.hpp>
#include <graph_templates.h>
#include <apexcv_host_base_class.h>
#ifdef APEX2_EMULATE
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include <acf_process_apu.h>
#endif


  /*!*********************************************************************************
  *  \brief Host-ACF interface for filter operations.
  *
  *  This template class is an interface for creating, initializing, processing and releasing
  *  the APU implementation of the filter kernels on the host.
  ***********************************************************************************/

namespace apexcv {


  template <class A>
  class apexcv_fct
  {
  public:

    /*!*********************************************************************************
    *  \brief Initialize the process.
    *
    *  \return Error code for the initialization (zero on success).
    *
    *  We Initialize the process on the host by initializing the ACF process
    *  \snippet apexcv_filter.cpp Initialize
    *  and connecting the memory buffers to the ACF graph ports
    *  \snippet apexcv_filter.cpp connect
    ***********************************************************************************/

    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aInput1,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;

      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aInput1,
      vsdk::SUMat& aInput2,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
      lRetVal |= mProcess.ConnectIO(GR_IN_2, aInput2);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat& lInput0,
      vsdk::SUMat& lInput1,
      vsdk::SUMat& lInput2,
      vsdk::SUMat& lInput3,
      vsdk::SUMat& lOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, lInput0);
      lRetVal |= mProcess.ConnectIO(GR_IN_1, lInput1);
      lRetVal |= mProcess.ConnectIO(GR_IN_2, lInput2);
      lRetVal |= mProcess.ConnectIO(GR_IN_3, lInput3);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, lOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT InitializeAlpha(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aInput1,
      vsdk::SUMat& aAlpha,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
      lRetVal |= mProcess.ConnectIO(GR_ALPHA, aAlpha);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat& aInput0,
      vsdk::SUMat& aInput1,
      vsdk::SUMat& aOutput0)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
      lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


  private:
    A mProcess;
  };







  class ApexCVHostUtilAri
  {
  public:

    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aDst)
    {
      apexcv_fct<ACF_T> *pF = new apexcv_fct<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {
        aHost.mpProcess = nullptr;
        return lRetVal;
      }
      aHost.mType = aType;
      return APEXCV_SUCCESS;
    }


    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc1,
      vsdk::SUMat&    aSrc2,
      vsdk::SUMat&    aDst)
    {
      apexcv_fct<ACF_T> *pF = new apexcv_fct<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc1, aSrc2, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {
        aHost.mpProcess = nullptr;
        return lRetVal;
      }
      aHost.mType = aType;
      return APEXCV_SUCCESS;
    }

    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc1,
      vsdk::SUMat&    aSrc2,
      vsdk::SUMat&    aSrc3,
      vsdk::SUMat&    aDst)
    {
      apexcv_fct<ACF_T> *pF = new apexcv_fct<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc1, aSrc2, aSrc3, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {
        aHost.mpProcess = nullptr;
        return lRetVal;
      }
      aHost.mType = aType;
      return APEXCV_SUCCESS;
    }

    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc1,
      vsdk::SUMat&    aSrc2,
      vsdk::SUMat&    aSrc3,
      vsdk::SUMat&    aSrc4,
      vsdk::SUMat&    aDst)
    {
      apexcv_fct<ACF_T> *pF = new apexcv_fct<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc1, aSrc2, aSrc3, aSrc4, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {
        aHost.mpProcess = nullptr;
        return lRetVal;
      }
      aHost.mType = aType;
      return APEXCV_SUCCESS;
    }

    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitializeAlpha(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc1,
      vsdk::SUMat&    aSrc2,
      vsdk::SUMat&    aAlpha,
      vsdk::SUMat&    aDst)
    {
      apexcv_fct<ACF_T> *pF = new apexcv_fct<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->InitializeAlpha(aSrc1, aSrc2, aAlpha, aDst);
      if (APEXCV_SUCCESS != lRetVal)
      {
        aHost.mpProcess = nullptr;
        return lRetVal;
      }
      aHost.mType = aType;
      return APEXCV_SUCCESS;
    }

  };




} /* namespace */


#endif /* APEXCVARITHMETICINT_H */
