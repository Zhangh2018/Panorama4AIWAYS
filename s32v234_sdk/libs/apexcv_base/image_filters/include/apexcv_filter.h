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
****************************************************************************/

/*!*********************************************************************************
*  \file apexcv_filter.hpp
*  \brief Host-ACF interface for filter operations.
***********************************************************************************/

#ifndef APEXCVFILTER_HPP
#define APEXCVFILTER_HPP
#include <stdint.h>

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
namespace apexcv{

  template <class A>
  class apexcv_filter
  {
  public:
    /*!*********************************************************************************
    *  \brief Default constructor.
    ***********************************************************************************/
    apexcv_filter()  {};

    /*!*********************************************************************************
    *  \brief Destructor.
    ***********************************************************************************/
    ~apexcv_filter() {};

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
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aSigmaR,
      vsdk::SUMat&    aSigmaD,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO("SIGMA_R", aSigmaR);
      lRetVal |= mProcess.ConnectIO("SIGMA_D", aSigmaD);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aK0,
      vsdk::SUMat&    aK1,
      vsdk::SUMat&    aK2,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO("K_0", aK0);
      lRetVal |= mProcess.ConnectIO("K_1", aK1);
      lRetVal |= mProcess.ConnectIO("K_2", aK2);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT InitializeScale(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aFilterCoeff,
      vsdk::SUMat&    aFilterScale,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;

      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_F_COEFF, aFilterCoeff);
      lRetVal |= mProcess.ConnectIO(GR_F_SCALE, aFilterScale);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT InitializeSeparable(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aFilterRow,
      vsdk::SUMat&    aFilterCol,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_F_ROW, aFilterRow);
      lRetVal |= mProcess.ConnectIO(GR_F_COL, aFilterCol);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aFilterCoeff,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_F_COEFF, aFilterCoeff);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT Initialize2Dst(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aDst1,
      vsdk::SUMat&    aDst2)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst1);
      lRetVal |= mProcess.ConnectIO(GR_OUT_1, aDst2);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT InitializeSobelHT(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aBuffer0,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO("BUFFER_0", aBuffer0);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }
      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;

      lRetVal |= mProcess.Initialize();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_INITIALIZE;
      }
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aDst)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      return APEXCV_SUCCESS;
    }

    APEXCV_LIB_RESULT ReconnectIO2Dst(
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aDst1,
      vsdk::SUMat&    aDst2)
    {
      int lRetVal = 0;
      lRetVal |= mProcess.ConnectIO(GR_IN_0, aSrc);
      lRetVal |= mProcess.ConnectIO(GR_OUT_0, aDst1);
      lRetVal |= mProcess.ConnectIO(GR_OUT_1, aDst2);
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_CONNECTIO;
      }

      return APEXCV_SUCCESS;
    }


    APEXCV_LIB_RESULT Process()
    {
      int lRetVal = 0;
      lRetVal |= mProcess.Start();
      lRetVal |= mProcess.Wait();
      if (0 != lRetVal)
      {
        return APEXCV_ERROR_ACF_PROC_EXEC;
      }
      return APEXCV_SUCCESS;
    }

  private:

    /*!*********************************************************************************
    *  \brief Process declaration for filter operations
    ***********************************************************************************/
    A mProcess;
  };








  class ApexCVHostUtilFilter
  {
  public:
    template<class ACF_T, class HOST_T>
    static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
      vsdk::SUMat&    aSrc,
      vsdk::SUMat&    aK0,
      vsdk::SUMat&    aK1,
      vsdk::SUMat&    aK2,
      vsdk::SUMat&    aDst)
    {
      apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
      if (nullptr == pF)
      {
        return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      aHost.mpProcess = pF;
      aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
      APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc, aK0, aK1, aK2, aDst);
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
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aSigmaR,
    vsdk::SUMat&    aSigmaD,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc, aSigmaR, aSigmaD, aDst);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }

  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitializeScale(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aFilterCoeff,
    vsdk::SUMat&    aScale,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->InitializeScale(aSrc, aFilterCoeff, aScale, aDst);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }


  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitializeSeparable(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aFilterRow,
    vsdk::SUMat&    aFilterCol,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->InitializeSeparable(aSrc, aFilterRow, aFilterCol, aDst);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }


  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aFilterCoeff,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->Initialize(aSrc, aFilterCoeff, aDst);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }


  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitializeSobelHT(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aBuffer0,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->InitializeSobelHT(aSrc, aBuffer0, aDst);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }


  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitialize2Dst(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst1,
    vsdk::SUMat&    aDst2)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
    if (nullptr == pF)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    aHost.mpProcess = pF;
    aHost.mvInfoClusters.push_back(ApexcvHostBaseBaseClass::InfoCluster((void*)((ACF_Process_APU*)pF), GR_IN_0));
    APEXCV_LIB_RESULT lRetVal = pF->Initialize2Dst(aSrc, aDst1, aDst2);
    if (APEXCV_SUCCESS != lRetVal)
    {
      delete pF;
      aHost.mpProcess = nullptr;
      return lRetVal;
    }
    aHost.mType = aType;
    return APEXCV_SUCCESS;
  }



  template<class ACF_T, class HOST_T>
  static inline APEXCV_LIB_RESULT WInitialize(HOST_T &aHost, int aType,
    vsdk::SUMat&    aSrc,
    vsdk::SUMat&    aDst)
  {
    apexcv_filter<ACF_T> *pF = new apexcv_filter<ACF_T>;
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


  };



}/* namespace*/
#endif /* APEXCVFILTER_HPP */
