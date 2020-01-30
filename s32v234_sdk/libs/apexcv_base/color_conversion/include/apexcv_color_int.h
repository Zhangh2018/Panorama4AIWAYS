/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

#ifndef APEXCVCOLOR_H
#define APEXCVCOLOR_H

#include <sumat.hpp>
#include "graph_templates.h"


/*!*********************************************************************************
*  \brief Host-ACF interface for filter operations.
*
*  This template class is an interface for creating, initializing, processing and releasing
*  the APU implementation of the filter kernels on the host.
***********************************************************************************/

template <class A>
class apexcv_fct
{
public:
  /*!*********************************************************************************
  *  \brief Default constructor.
  ***********************************************************************************/
  apexcv_fct()  {};

  /*!*********************************************************************************
  *  \brief Destructor.
  ***********************************************************************************/
  ~apexcv_fct() {};

  /*!*********************************************************************************
  *  \brief Initialize the process.
  *
  *  \return Error code for the initialization (zero on success).
  *
  *  We initialize the process on the host by initializing the ACF process
  *  \snippet apexcv_filter.cpp initialize
  *  and connecting the memory buffers to the ACF graph ports
  *  \snippet apexcv_filter.cpp connect
  ***********************************************************************************/


  APEXCV_LIB_RESULT Initialize(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0){
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
  };


  APEXCV_LIB_RESULT Initialize_RGB(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aR,
    vsdk::SUMat& aG,
    vsdk::SUMat& aB,
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
    lRetVal |= mProcess.ConnectIO("R2Y", aR);
    lRetVal |= mProcess.ConnectIO("G2Y", aG);
    lRetVal |= mProcess.ConnectIO("B2Y", aB);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT Initialize_KABCR(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aR,
    vsdk::SUMat& aG,
    vsdk::SUMat& aB,
    vsdk::SUMat& aS,
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
    lRetVal |= mProcess.ConnectIO("KA", aR);
    lRetVal |= mProcess.ConnectIO("KB", aG);
    lRetVal |= mProcess.ConnectIO("KC", aB);
    lRetVal |= mProcess.ConnectIO("KR", aS);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT Initialize1I1O_CH_ID(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aChannel,
    vsdk::SUMat& aOutput0){
    int lRetVal = 0;

    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO("CH_ID", aChannel);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT Initialize2I1O_CH_ID(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aChannel,
    vsdk::SUMat& aOutput0){
    int lRetVal = 0;
    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO("CH_ID", aChannel);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };


  APEXCV_LIB_RESULT Initialize1I4O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1,
    vsdk::SUMat& aOutput2,
    vsdk::SUMat& aOutput3){
    int lRetVal = 0;
    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    lRetVal |= mProcess.ConnectIO(GR_OUT_2, aOutput2);
    lRetVal |= mProcess.ConnectIO(GR_OUT_3, aOutput3);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };



  APEXCV_LIB_RESULT Initialize1I3O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1,
    vsdk::SUMat& aOutput2){
    int lRetVal = 0;
    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    lRetVal |= mProcess.ConnectIO(GR_OUT_2, aOutput2);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };



  APEXCV_LIB_RESULT Initialize1I2O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1){
    int lRetVal = 0;
    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };



  APEXCV_LIB_RESULT Initialize4I1O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aInput2,
    vsdk::SUMat& aInput3,
    vsdk::SUMat& aOutput0){
    int lRetVal = 0;
    lRetVal |= mProcess.Initialize();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
    lRetVal |= mProcess.ConnectIO(GR_IN_2, aInput2);
    lRetVal |= mProcess.ConnectIO(GR_IN_3, aInput3);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };


  APEXCV_LIB_RESULT Initialize3I1O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aInput2,
    vsdk::SUMat& aOutput0){
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
  };


  APEXCV_LIB_RESULT Initialize2I1O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aOutput0){
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
  };






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
  };



  APEXCV_LIB_RESULT ReconnectIO_1I2O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1)
  {
    int lRetVal = 0;

    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT ReconnectIO_1I3O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1,
    vsdk::SUMat& aOutput2)
  {
    int lRetVal = 0;

    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    lRetVal |= mProcess.ConnectIO(GR_OUT_2, aOutput2);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT ReconnectIO_1I4O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aOutput0,
    vsdk::SUMat& aOutput1,
    vsdk::SUMat& aOutput2,
    vsdk::SUMat& aOutput3)
  {
    int lRetVal = 0;

    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    lRetVal |= mProcess.ConnectIO(GR_OUT_1, aOutput1);
    lRetVal |= mProcess.ConnectIO(GR_OUT_2, aOutput2);
    lRetVal |= mProcess.ConnectIO(GR_OUT_3, aOutput3);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    return APEXCV_SUCCESS;
  };


  APEXCV_LIB_RESULT ReconnectIO_4I1O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aInput2,
    vsdk::SUMat& aInput3,
    vsdk::SUMat& aOutput0)
  {
    int lRetVal = 0;
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
    lRetVal |= mProcess.ConnectIO(GR_IN_2, aInput2);
    lRetVal |= mProcess.ConnectIO(GR_IN_3, aInput3);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT ReconnectIO_3I1O(
    vsdk::SUMat& aInput0,
    vsdk::SUMat& aInput1,
    vsdk::SUMat& aInput2,
    vsdk::SUMat& aOutput0)
  {
    int lRetVal = 0;
    lRetVal |= mProcess.ConnectIO(GR_IN_0, aInput0);
    lRetVal |= mProcess.ConnectIO(GR_IN_1, aInput1);
    lRetVal |= mProcess.ConnectIO(GR_IN_2, aInput2);
    lRetVal |= mProcess.ConnectIO(GR_OUT_0, aOutput0);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }
    return APEXCV_SUCCESS;
  };

  APEXCV_LIB_RESULT ReconnectIO_2I1O(
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
  };



  /*!*********************************************************************************
  *  \brief Execute the process.
  *
  *  \return Error code for the execution (zero on success).
  *
  *  Execution consists of starting the ACF process
  *  \snippet apexcv_filter.cpp start
  *  and waiting for it to complete
  *  \snippet apexcv_filter.cpp wait
  ***********************************************************************************/
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
  };

private:

  /*!*********************************************************************************
  *  \brief Process declaration for filter operations
  ***********************************************************************************/
  A mProcess;
};

#endif /* APEXCVCOLOR_H */
