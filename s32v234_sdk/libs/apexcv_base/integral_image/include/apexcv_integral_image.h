/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor
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
*  \file apexcv_integral_image.h
*  \brief Host-ACF interface for integral image
***********************************************************************************/

#ifndef APEXCVINTEGRALIMAGE_H
#define APEXCVINTEGRALIMAGE_H

#include <sumat.hpp>
#include <apexcv_host_base_class.h>


namespace apexcv{

  /*!*********************************************************************************
  *  \brief          Integral Image value
  *  \details        Object of this class computes the sum of the pixel values located 
  *                  above and to the left of a given pixel.<br>
  *                  Output dimensions are same as input.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/

  class IntegralImage : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~IntegralImage();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_32SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_32SC1). */);

  };
}
#endif /* APEXCVINTEGRALIMAGE_H */
