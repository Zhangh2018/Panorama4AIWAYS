/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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
*  \file
*  \brief APEX-CV Base Library
***********************************************************************************/

#ifndef APEXCVCOLORCONVERSION_HPP
#define APEXCVCOLORCONVERSION_HPP

#include <sumat.hpp>
#include <apexcv_host_base_class.h>

namespace apexcv{

  /*!*********************************************************************************
  *  \brief          Color converter class
  *  \details        Object of this class performs color conversions of images.<br>
  *                  See ConversionType for supported converions.<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ColorConverter : public ApexcvHostBaseClass
  {
  public:
    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~ColorConverter();

    /*!*********************************************************************************
    *  \brief List of conversion types
    ***********************************************************************************/
    enum ConversionType {
      eRGB565_TO_RGB888X = 1 /*!< 16-bit RGB565 (VSDK_CV_16UC1) to 32-bit representation of RGB888X (VSDK_CV_32SC1) */,
      eRGB888X_TO_RGB565 /*!< 32-bit representation of RGB888X (VSDK_CV_32SC1) to 16-bit RGB565 (VSDK_CV_16UC1) */,
      eRGB888X_TO_Y      /*!< 4-tuple 8-bit R, G, B, X (VSDK_CV_8UC4) to 8-bit Y (VSDK_CV_8UC1) */,
      eRGB888X_TO_YUV    /*!< 4-tuple 8-bit R, G, B, X (VSDK_CV_8UC4) to 4-tuple 8-bit Y, U, V, X (VSDK_CV_8UC4) */,
      eRGB888_TO_GREY    /*!< 3-tuple 8-bit R, G, B (VSDK_CV_8UC3) to 8-bit Grey (VSDK_CV_8UC1), (R*21 + G*72 + B*7) */,
      eBGR888_TO_GREY    /*!< 3-tuple 8-bit B, G, R (VSDK_CV_8UC3) to 8-bit Grey (VSDK_CV_8UC1), (R*21 + G*72 + B*7) */,
      eGREY_TO_RGB888    /*!< 8-bit grey (VSDK_CV_8UC1) to 3-tuple 8-bit B, G, R (VSDK_CV_8UC3), duplication on all 3 channels*/
    };

    /*!*********************************************************************************
    *  \brief        Convert function.
    *  \details      Converts an image from one type to another based on \ref ConversionType.
    *                R2YFactor, G2YFactor and B2YFactor are Q0.8 fixed point values used with RGB888X_TO_Y following the formula:
    *                \f$Y = \left \lfloor{\frac{R2YFactor}{256} * R + \frac{G2YFactor}{256} * G + \frac{B2YFactor}{256} * B + 0.5}\right \rfloor\f$
    *                For example, conversion following Recommendation ITU-R BT.601-7 (http://www.itu.int/rec/R-REC-BT.601-7-201103-I/en) would use factor values of 77(0.299), 150(0.587) and 29(0.114).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc  /*!< [in] Source memory buffer (Use the type corresponding to the \ref ConversionType selected). */,
      ConversionType    aCT  /*!< [in] Color conversion type. See \ref ConversionType */,
      int    aR2YFactor  /*!< [in] Conversion factor for red used with RGB888X_TO_Y */,
      int    aG2YFactor  /*!< [in] Conversion factor for green used with RGB888X_TO_Y */,
      int    aB2YFactor  /*!< [in] Conversion factor for blue used with RGB888X_TO_Y*/,
      vsdk::SUMat&    aDst  /*!< [in,out] Destination memory buffer (Use the type corresponding to the \ref ConversionType selected). */);

    /*!*********************************************************************************
    *  \brief        Convert function.
    *..\details      Converts an image from one type to another based on \ref ConversionType.
    *                R2YFactor, G2YFactor and B2YFactor are Q0.8 fixed point values used with RGB888X_TO_Y following the formula:
    *                \f$Y = \left \lfloor{\frac{R2YFactor}{256} * R + \frac{G2YFactor}{256} * G + \frac{B2YFactor}{256} * B + 0.5}\right \rfloor\f$
    *                For example, conversion following Recommendation ITU-R BT.601-7 (http://www.itu.int/rec/R-REC-BT.601-7-201103-I/en) would use factor values of 77(0.299), 150(0.587) and 29(0.114).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc  /*!< [in] Source memory buffer. */,
      ConversionType    aCT  /*!< [in] Color conversion type. See \ref ConversionType */,
      vsdk::SUMat&    aDst  /*!< [in,out] Destination memory buffer. */);



    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *..\details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc  /*!< [in] Source memory buffer (type should be the same one used when calling \ref Initialize). */,
      vsdk::SUMat&    aDst  /*!< [in,out] Destination memory buffer (type should be the same one used when calling \ref Initialize). */);

    /*!*********************************************************************************
    *  \brief        Set factors for RGB888x to Y.
    *  \details      This function allows to change factors without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFactors(
      int    aR2YFactor  /*!< Conversion factor for red used with RGB888X_TO_Y */,
      int    aG2YFactor  /*!< Conversion factor for green used with RGB888X_TO_Y */,
      int    aB2YFactor  /*!< Conversion factor for blue used with RGB888X_TO_Y*/);

  private:
    vsdk::SUMat mR2Y;
    vsdk::SUMat mG2Y;
    vsdk::SUMat mB2Y;

  }; /* class colorConverter */

  /*!*********************************************************************************
  *  \brief          Optimized color converter class containing support for converting image color types
  *  \details        This class is an interface for using color conversion functions on the host.
  ***********************************************************************************/
  class ColorConverterHT : public ApexcvHostBaseClass
  {
  public:
    /**********************************************************************************
    *  Destructor.
    ***********************************************************************************/
    ~ColorConverterHT();

    /*!*********************************************************************************
    *  \brief        List of conversion types
    ***********************************************************************************/
    enum ConversionType {
      eHT_RGB888X_TO_Y = 1     /*!< 4-tuple 8 bit A, B, C, X (VSDK_CV_8UC4) to signed 16 bit Y (VSDK_CV_16SC1) */
    };

    /*!*********************************************************************************
    *  \brief        Convert function.
    *  \details      Converts an image from one type to another based on \ref ConversionType.
    *                R2YFactor, G2YFactor and B2YFactor are Q0.8 fixed point values used with RGB888X_TO_Y following the formula:
    *                \f$Y = \left \lfloor{\frac{R2YFactor}{256} * R + \frac{G2YFactor}{256} * G + \frac{B2YFactor}{256} * B + 0.5}\right \rfloor\f$
    *                For example, conversion following Recommendation ITU-R BT.601-7 (http://www.itu.int/rec/R-REC-BT.601-7-201103-I/en) would use factor values of 77(0.299), 150(0.587) and 29(0.114).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc  /*!< [in] Source memory buffer (Use the type corresponding to the \ref ConversionType selected). */,
      ConversionType    aCT  /*!< [in] Color conversion type. See \ref ConversionType */,
      uint8_t    aR2YFactor  /*!< [in] Conversion factor for red used with RGB888X_TO_Y */,
      uint8_t    aG2YFactor  /*!< [in] Conversion factor for green used with RGB888X_TO_Y */,
      uint8_t    aB2YFactor  /*!< [in] Conversion factor for blue used with RGB888X_TO_Y*/,
      uint16_t    aShiftFactor /*!< [in] Shift factor. Use 0 by default. Used in HT_RGB888X_TO_Y. */,
      vsdk::SUMat&    aDst  /*!< [in,out] Destination memory buffer (Use the type corresponding to the \ref ConversionType selected). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc  /*!< [in] Source memory buffer (type should be the same one used when calling \ref Initialize). */,
      vsdk::SUMat&    aDst  /*!< [in,out] Destination memory buffer (type should be the same one used when calling \ref Initialize). */);

    /*!*********************************************************************************
    *  \brief        Set factors for RGB888x to Y.
    *  \details      This function allows to change factors without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetFactors(
      uint8_t    aR2YFactor  /*!< [in] Conversion factor for red used with HT_RGB888X_TO_Y */,
      uint8_t    aG2YFactor  /*!< [in] Conversion factor for green used with HT_RGB888X_TO_Y */,
      uint8_t    aB2YFactor  /*!< [in] Conversion factor for blue used with HT_RGB888X_TO_Y*/,
      uint16_t    aShiftFactor /*!< [in] Shift factor. Use 0 by default. Used in HT_RGB888X_TO_Y. */);

  private:
    vsdk::SUMat mR2Y;
    vsdk::SUMat mG2Y;
    vsdk::SUMat mB2Y;
    vsdk::SUMat mShiftFactor;
  }; /* class colorConverter */


  /*!*********************************************************************************
  *  \brief          Channel extract class containing support for extracting a single channel from a multi-channel image
  *  \details        This class is an interface for using channel extract functions on the host.
  ***********************************************************************************/
  class ExtractChannel : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~ExtractChannel();

    /*!*********************************************************************************
    *  \brief        Channel Extract function.
    *  \details      Extracts a channel from a multiple channel image based on its index.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2, VSDK_CV_8UC3, VSDK_CV_8UC4 */,
      int    aChannelIndex /*!< [in] Index of the channel to extract. Starts at 1. */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2, VSDK_CV_8UC3, VSDK_CV_8UC4 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

  private:
    vsdk::SUMat mChannelIndex;
  }; /* class ExtractChannel */




  /*!*********************************************************************************
  *  \brief          Channel insert class containing support for inserting a single channel in a multi-channel image
  *  \details        This class is an interface for using channel insert functions on the host.
  ***********************************************************************************/
  class InsertChannel : public ApexcvHostBaseClass
  {
  public:
    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~InsertChannel();

    /*!*********************************************************************************
    *  \brief        Channel Insert function.
    *  \details      Inserts a channel from a multiple channel image based on its index.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2, VSDK_CV_8UC3, VSDK_CV_8UC4 */,
      int    aChannelIndex /*!< [in] Index of the channel to insert. Starts at 1. */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2, VSDK_CV_8UC3, VSDK_CV_8UC4 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

  private:
    vsdk::SUMat mChannelIndex;
  }; /* class InsertChannel */





  /*!*********************************************************************************
  *  \brief          Channel split class containing support for spliting a single channel from a multi-channel image
  *  \details        This class is an interface for using channel split functions on the host.
  ***********************************************************************************/
  class SplitChannel : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~SplitChannel();

    /*!*********************************************************************************
    *  \brief        Channel Split function.
    *  \details      Splits a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC4 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst3 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst4 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Channel Split function.
    *  \details      Splits a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC3 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst3 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Channel Split function.
    *  \details      Splits a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);


    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC4 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst3 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst4 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC3 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst3 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC2 */,
      vsdk::SUMat&    aDst1 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst2 /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC1 */);

  private:
    vsdk::SUMat mChannelIndex;
  }; /* class SplitChannel */





  /*!*********************************************************************************
  *  \brief          Channel merge class containing support for merging multiple single channels images into a single multi-channel image
  *  \details        This class is an interface for using channel merge functions on the host.
  ***********************************************************************************/
  class MergeChannel : public ApexcvHostBaseClass
  {
  public:

    /***********************************************************************************
    *  Destructor, not documented.
    ***********************************************************************************/
    ~MergeChannel();

    /*!*********************************************************************************
    *  \brief        Channel Merge function.
    *  \details      Merges a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc3 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc4 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC4 */);

    /*!*********************************************************************************
    *  \brief        Channel Merge function.
    *  \details      Merges a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc3 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC3 */);

    /*!*********************************************************************************
    *  \brief        Channel Merge function.
    *  \details      Merges a channel from a multiple channel image.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC2 */);


    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc3 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc4 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC4 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc3 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC3 */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO.
    *  \details      This function allows to change the Input and Output images without re-initializing
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source memory buffer. Accepted buffer types are VSDK_CV_8UC1 */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. Accepted buffer type is VSDK_CV_8UC2 */);

  private:
    vsdk::SUMat mChannelIndex;
  }; /* class MergeChannel */




/*!*********************************************************************************
  *  \brief          Converts image bit depth.
  *  \details        Object of this class performs converts image bit depth.<br>
  *                  Up convert, shifting left. Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1<br>
  *                  Down convert, shifting right. Supported input type: VSDK_CV_16SC1, output type: VSDK_CV_8UC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ConvertDepth : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~ConvertDepth();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      const  int32_t   acShift /*!< [in] Source pixel value shift amount (0 <= aShift < 8 ). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Set Shift.
    *  \details      This function allows to change the shift fact value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetShift(
    const int32_t    acShift /*!< [in] Source pixel value shift amount (0 <= aShift < 8 )*/);


    /*!*********************************************************************************
    *  \brief        Get Shift.
    *  \details      This function allows to read the value of the shift.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetShift(
    int32_t&   aShift /*!< [out] shift. */);


    /*!*********************************************************************************
    *  \brief        Set Policy type.
    *  \details      This function allows to change the Policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetPolicyType(
    apexcv::eConvertPolicy    aPolicy /*!< [in] Policy type */);


    /*!*********************************************************************************
    *  \brief        Get Policy type.
    *  \details      This function allows to read the value of the Policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetPolicyType(
    apexcv::eConvertPolicy&   aPolicy /*!< [out] Policy. */);

  private:
    vsdk::SUMat mShift;
    vsdk::SUMat mPolicy;

  };


} // namespace apexcv
#endif /* APEXCVCOLORCONVERSION_HPP */
