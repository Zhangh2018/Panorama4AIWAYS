/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP
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
*  \file apexcv_base_image_filters.h
*  \brief APEX-CV Base Library, Image Filters Module
***********************************************************************************/

#ifndef APEXCVIMAGEFILTERS_H
#define APEXCVIMAGEFILTERS_H

#include <sumat.hpp>
#include <apexcv_host_base_class.h>

namespace apexcv{

  /*!*********************************************************************************
  *  \brief          Bilateral filter
  *  \details        Object of this class applies a bilateral filter on _aSrc_. _aSigmaColor_ represents the weight of
  *                  color/intensity difference and _aSigmaSpace_ represents the weight of spacial
  *                  difference. See:  \cite BILATERAL for more information.<br>
  *                  Supported window size: 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BilateralFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~BilateralFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 5. */
      int    aSigmaColor, /*!< [in] Sigma value for color space. */
      int    aSigmaSpace, /*!< [in] Sigma value for distance space. */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);


    /*!*********************************************************************************
    *  \brief        Set sigmaColor.
    *  \details      This function allows to change the value of sigmaColor
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetSigmaColor(
      int    aSigmaColor /*!< [in] Sigma value for color space. */);

    /*!*********************************************************************************
    *  \brief        Set sigmaSpace.
    *  \details      This function allows to change the value of sigmaSpace
    *  \return       APEXCV_LIB_RESULT Error code.
   ***********************************************************************************/
    APEXCV_LIB_RESULT SetSigmaSpace(
      int    aSigmaSpace /*!< Sigma value for distance space. */);

  private:
    vsdk::SUMat    mSigmaColor /*!< Sigma value for color space. */;
    vsdk::SUMat    mSigmaSpace /*!< Sigma value for distance space. */;
  };



  /*!*********************************************************************************
  *  \brief          Box filter
  *  \details        Object of this class applies a box filter to _aSrc_.
  *                  Supported window size: 3x3, 5x5, 7x7 and 9x9 (for VSDK_CV_8UC1)<br>
  *                  Supported window size: 3x3 and 5x5 (for VSDK_CV_16SC1)<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1 and VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BoxFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~BoxFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1) */
      int    aWindowSize, /*!< [in] Window size (3, 5, 7 or 9) */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1) */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Census filter
  *  \details        Object of this class applies a census filter to _aSrc_.
  *                  Supported window size: 3 x 3<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class CensusFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~CensusFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size (3). */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Dilate filter
  *  \details        Object of this class applies a Dilate filter to _aSrc_.
  *                  Supported window size: 3 x 3<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1 and VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class DilateFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~DilateFilter();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */
      int    aWindowSize, /*!< [in] Window Size (3). */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Erode filter
  *  \details        Object of this class applies a Erode filter to _aSrc_.
  *                  Supported window size: 3 x 3<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ErodeFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ErodeFilter();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size (3). */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };



  /*!*********************************************************************************
  *  \brief          Median filter
  *  \details        Object of this class applies a Median filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5 and 7x7<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class MedianFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~MedianFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 or 7 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Gaussian filter
  *  \details        Object of this class applies a Gaussian filter to _aSrc_.
  *                  Supported window size: 3x3, 5x5, 7x7 or 9x9<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class GaussianFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~GaussianFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size: 3, 5, 7 or 9*/
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);
  };





  /*!*********************************************************************************
  *  \brief          Sobel filter
  *  \details        Object of this class applies a Sobel filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Sobel X filter
  *  \details        Object of this class applies a Sobel X filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelXFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelXFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Sobel Y filter
  *  \details        Object of this class applies a Sobel Y filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelYFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelYFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Sobel XY filter
  *  \details        Object of this class applies a Sobel X and Y filter to _aSrc_.
  *                  Supported window size: 3 x 3<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelXYFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelXYFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDstX /*!< [in,out] Destination Image buffer, X (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDstY /*!< [in,ou] Destination Image buffer, Y (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDstX /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDstY /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);
  };




  /*!*********************************************************************************
  *  \brief          Convolve filter
  *  \details        Object of this class applies a Convolve filter to _aSrc_.
  *                  Supported window size: 3 x 3 or 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ConvolveFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ConvolveFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterCoeff)[9], /*!< [in] 9 Coefficients for 3x3 kernel. */
      int    aFilterScale, /*!< [in] Right Shift to scale the data. */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterCoeff)[25], /*!< [in] 25 Coefficients for 5x5 kernel. */
      int    aFilterScale, /*!< [in] Right Shift to scale the data. */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Set Filter Scale.
    *  \details      This function allows to change the filter scale (right shift).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterScale(
      int aFilterScale /*!< [in] Set the filter Scale. */);
    /*!*********************************************************************************
    *  \brief        Set Filter Coefficients.
    *  \details      This function allows to change the filter 9 coefficients for 3x3 kernels
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCoeff(
      signed char(&filterCoeff)[9] /*!< [in] 9 Coefficients for 3x3 kernel. */);
    /*!*********************************************************************************
    *  \brief        Set Filter Coefficients.
    *  \details      This function allows to change the filter 25 coefficients for 5x5 kernels
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCoeff(
      signed char(&filterCoeff)[25] /*!< [in] 25 Coefficients for 5x5 kernel. */);
  private:
    vsdk::SUMat  mFilterCoeff /*!< Filter Coefficients. */;
    vsdk::SUMat  mScale /*!< Scale. */;
  };





  /*!*********************************************************************************
  *  \brief          Prewitt X filter
  *  \details        Object of this class applies a Prewitt X filter to _aSrc_.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class PrewittXFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~PrewittXFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Prewitt Y filter
  *  \details        Object of this class applies a Prewitt Y filter to _aSrc_.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class PrewittYFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~PrewittYFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Box filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Box filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BoxFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~BoxFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Saturate filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Saturate filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_16SC1, output type: VSDK_CV_8SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SaturateFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SaturateFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8SC1). */);

  };


/*!*********************************************************************************
  *  \brief          Scharr filter
  *  \details        Object of this class applies a Scharr filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported output type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ScharrFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ScharrFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };

  /*!*********************************************************************************
  *  \brief          Scharr X filter
  *  \details        Object of this class applies a Scharr X filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ScharrXFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ScharrXFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

  };

  /*!*********************************************************************************
  *  \brief          Scharr Y filter
  *  \details        Object of this class applies a Scharr Y filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ScharrYFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ScharrYFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

  };


  /*!*********************************************************************************
  *  \brief          Scharr XY filter
  *  \details        Object of this class applies a Scharr X and Y filter to _aSrc_.
  *                  Supported window size: 3 x 3 and 5 x 5<br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1.<br>
  *                  Supported output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ScharrXYFilter : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ScharrXYFilter();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 or 5*/
      vsdk::SUMat&    aDstX /*!< [in,out] Destination Image buffer, X (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDstY /*!< [in,out] Destination Image buffer, Y (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDstX /*!< [in,out] Destination image buffer, X (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDstY /*!< [in,out] Destination image buffer, Y (VSDK_CV_16SC1). */);
  };


  /*!*********************************************************************************
  *  \brief          Sobel filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Sobel filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< Window Size, 3 or 5 */
      vsdk::SUMat&    aDst /*!< Destination Image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Sobel X filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Sobel X filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelXFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelXFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8SC1). */);

  private:
    vsdk::SUMat mBuffer0;
  };





  /*!*********************************************************************************
  *  \brief          Sobel Y filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Sobel Y filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SobelYFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SobelYFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8SC1). */);

  private:
    vsdk::SUMat mBuffer0;
  };






  /*!*********************************************************************************
  *  \brief          Convolve filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a generic convolution filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 and 5 x 5 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ConvolveFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~ConvolveFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterCoeff)[9], /*!< [in] 9 Coefficients for 3x3 kernel. */
      signed char aFilterScale, /*!< [in] Right Shift to scale the data. */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterCoeff)[25], /*!< [in] 25 Coefficients for 5x5 kernel. */
      signed char    aFilterScale, /*!< [in] Right Shift to scale the data. */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);
    /*!*********************************************************************************
    *  \brief        Set Filter Scale.
    *  \details      This function allows to change the filter scale (right shift).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterScale(
      signed char aFilterScale /*!< [in] Set the filter Scale. */);
    /*!*********************************************************************************
    *  \brief        Set Filter Coefficients.
    *  \details      This function allows to change the filter 9 coefficients for 3x3 kernels
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCoeff(
      signed char(&filterCoeff)[9] /*!< [in] 9 Coefficients for 3x3 kernel. */);
    /*!*********************************************************************************
    *  \brief        Set Filter Coefficients.
    *  \details      This function allows to change the filter 25 coefficients for 5x5 kernels
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCoeff(
      signed char(&filterCoeff)[25] /*!< [in] 25 Coefficients for 5x5 kernel. */);
  private:
    vsdk::SUMat  mFilterCoeff /*!< Filter Coefficients. */;
    vsdk::SUMat  mScale /*!< Scale. */;
  };





  /*!*********************************************************************************
  *  \brief          Derivative X filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Derivative X filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class DerivativeXFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~DerivativeXFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      signed char    aK0, /*!< [in] K0 */
      signed char    aK1, /*!< [in] K1 */
      signed char    aK2, /*!< [in] K2 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);
    /*!*********************************************************************************
    *  \brief        Set K0.
    *  \details      This function allows to change the Input parameter K0
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK0(
      signed char    k0);
    /*!*********************************************************************************
    *  \brief        Set K1.
    *  \details      This function allows to change the Input parameter K1
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK1(
      signed char    k1);
    /*!*********************************************************************************
    *  \brief        Set K2.
    *  \details      This function allows to change the Input parameter K2
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK2(
      signed char    k2);
  private:
    vsdk::SUMat  mK0 /*!< K0. */;
    vsdk::SUMat  mK1 /*!< K1. */;
    vsdk::SUMat  mK2 /*!< K2. */;
  };




  /*!*********************************************************************************
  *  \brief          Derivative Y filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Derivative Y filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class DerivativeYFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~DerivativeYFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      int    aWindowSize, /*!< [in] Window Size, 3 */
      signed char    aK0, /*!< [in] K0 */
      signed char    aK1, /*!< [in] K1 */
      signed char    aK2, /*!< [in] K2 */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);
    /*!*********************************************************************************
    *  \brief        Set K0.
    *  \details      This function allows to change the Input parameter K0
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK0(
      signed char    k0);
    /*!*********************************************************************************
    *  \brief        Set K1.
    *  \details      This function allows to change the Input parameter K1
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK1(
      signed char    k1);
    /*!*********************************************************************************
    *  \brief        Set K2.
    *  \details      This function allows to change the Input parameter K2
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetK2(
      signed char    k2);
  private:
    vsdk::SUMat  mK0 /*!< K0. */;
    vsdk::SUMat  mK1 /*!< K1. */;
    vsdk::SUMat  mK2 /*!< K2. */;
  };





  /*!*********************************************************************************
  *  \brief          Separable filter, Hand Tuned (optimized).
  *  \details        Object of this class applies a Separable filter to _aSrc_.
  *                  This is a hand tuned (HT) implementation providing faster processing times.
  *                  Supported window size: 3 x 3 and 5 x 5 <br>
  *                  _aDst_ and _aSrc_ must have identical dimensions.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class SeparableFilterHT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilFilter;
  public:
    ~SeparableFilterHT();
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterRow)[3], /*!< [in] 3x1 Horizontal filter parameters */
      signed char(&aFilterCol)[3], /*!< [in] 1x3 Vertical filter parameters */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);
    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc, /*!< [in] Source Image buffer (VSDK_CV_8UC1). */
      signed char(&aFilterRow)[5], /*!< [in] 5x1 Horizontal filter parameters */
      signed char(&aFilterCol)[5], /*!< [in] 1x5 Vertical filter parameters */
      vsdk::SUMat&    aDst /*!< [in,out] Destination Image buffer (VSDK_CV_16SC1). */);
    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        set Filter Row.
    *  \details      This function allows to change filter coefficients.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterRow(
      signed char(&aFilterRow)[3]); /*!< [in] 3x1 Horizontal filter parameters */
    /*!*********************************************************************************
    *  \brief        set Filter Col.
    *  \details      This function allows to change filter coefficients.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCol(
      signed char(&aFilterCol)[3]); /*!< [in] 1x3 Vertical filter parameters */
    /*!*********************************************************************************
    *  \brief        set Filter Row.
    *  \details      This function allows to change filter coefficients.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterRow(
      signed char(&aFilterRow)[5]); /*!< [in] 5x1 Horizontal filter parameters */
    /*!*********************************************************************************
    *  \brief        set Filter Column.
    *  \details      This function allows to change filter coefficients.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFilterCol(
      signed char(&aFilterCol)[5]); /*!< [in] 1x5 Vertical filter parameters */
  private:
    vsdk::SUMat  mFilterRow /*!< mfilterRow. */;
    vsdk::SUMat  mFilterCol /*!< mfilterCol. */;
  };


}  /* namespace apexcv */
#endif /* APEXCVIMAGEFILTERS_HPP */
