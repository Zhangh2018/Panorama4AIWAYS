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

#ifndef APEXCVBASEINTERPOLATION_H
#define APEXCVBASEINTERPOLATION_H

#include <sumat.hpp>
#include <apexcv_host_base_class.h>


namespace apexcv{

  /*!*********************************************************************************
  *  \brief          Linear Grayscale Interpolation.
  *  \details        Object of this class computes the horizontal linear interpolation between pixels.<br>
  *                  Output dimensions are same as input dimensions.<br>
  *                  Dst(x,y) = Src(x,y) + ((Src(x+1,y) - Src(x,y)) * Delta(x,y) + 128)/256<br>
  *                  Supported input type: VSDK_CV_8UC1, output is of identical type VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class InterpolationLinearGrayscale : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~InterpolationLinearGrayscale();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDeltaX/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDeltaX/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };



  /*!*********************************************************************************
  *  \brief          Bilinear Grayscale Interpolation.
  *  \details        Object of this class computes the horizontal linear interpolation, followed by the vertical linear interpolation.<br>
  *                  Dst'(x,y) = Src(x,y) + ((Src(x+1,y) - Src(x,y)) * Delta[0](x,y) + 128)/256<br>
  *                  Dst(x,y) = Dst'(x,y) + ((Dst'(x,y+1) - Dst'(x,y)) * Delta[1](x,y) + 128)/256<br>
  *                  Output dimensions are same as input dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output is of identical type VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class InterpolationBilinearGrayscale : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~InterpolationBilinearGrayscale();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDelta/*!< [in] Delta image buffer (VSDK_CV_8UC2). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDelta/*!< [in] Delta image buffer (VSDK_CV_8UC2). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Bicubic Grayscale Interpolation.
  *  \details        Object of this class computes the horizontal cubic interpolation, followed by the vertical cubic interpolation on 4x4 patches.<br>
  *                  Output dimensions are same as input dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output is of identical type VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class InterpolationBicubicGrayscale : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~InterpolationBicubicGrayscale();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aOffsetX/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aOffsetY/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aOffsetX/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aOffsetY/*!< [in] Delta image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };

}  /* namespace apex */
#endif /* APEXCVBASEINTERPOLATION_H */
