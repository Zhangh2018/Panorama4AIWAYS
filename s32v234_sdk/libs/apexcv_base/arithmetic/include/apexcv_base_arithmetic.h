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
***************************************************************************/

/*!*********************************************************************************
*  \file apexcv_base_arithmetic.h
*  \brief APEX-CV Base Library
***********************************************************************************/

#ifndef APEXCVBASEARITHMETIC_HPP
#define APEXCVBASEARITHMETIC_HPP

#include <sumat.hpp>
#include <apexcv_host_base_class.h>


namespace apexcv{

  /*!*********************************************************************************
  *  \brief          Absolute value, _aDst_(x,y) = abs(_aSrc_(x,y))
  *  \details        Object of this class computes the absolute value of every pixel.<br>
  *                  Output dimensions are same as input.<br>
  *                  Supported input type: VSDK_CV_8SC1, output type: VSDK_CV_8UC1.<br>
  *                  Supported width: 128 to 2048 pixels 
  ***********************************************************************************/
  class Abs : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    ~Abs();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process 
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Absolute difference
  *  \details        Object of this class computes the absolute difference pixel for every pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1 <br>
  *                  Supported input type: VSDK_CV_16SC1, output type: VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels 
  ***********************************************************************************/
  class AbsDiff : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~AbsDiff();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1 or VSDK_CV_16SC1). */);

  };



  /*!*********************************************************************************
  *  \brief          Accumulate
  *  \details        Object of this class accumulates _aSrc_ into _aDst_. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels 
  ***********************************************************************************/
  class Accumulate : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Accumulate();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

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
  *  \brief          Accumulate Squared
  *  \details        Object of this class accumulates a squared value from _aSrc_ to _aDst.<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels 
  ***********************************************************************************/
  class AccumulateSquared : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~AccumulateSquared();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      const uint8_t    acScale /*!< [in] Scale amount. Right shift the square of aSrc by aScale (0 <= aScale <= 15) */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Set Scale.
    *  \details      This function allows to change the scale value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetScale(
      const uint8_t    aScale /*!< [in] Scale amount. Right shift the square of aSrc by aScale (0 <= aScale <= 15) */);

    /*!*********************************************************************************
    *  \brief        Get Scale.
    *  \details      This function allows to read the scale value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetScale(
      uint8_t&    aScale /*!< [in] Scale amount. Right shift the square of aSrc by aScale (0 <= aScale <= 15) */);

  private:
    vsdk::SUMat mScale;
  };




  /*!*********************************************************************************
  *  \brief          Accumulate Weighted
  *  \details        Object of this class accumulates a weight value from _aSrc_ to _aDst_<br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1  or <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class AccumulateWeighted : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~AccumulateWeighted();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
    const float    acAlpha /*!< [in] Weight amount. Scalar value with a value in the range of [0, 1]*/,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Set Alpha.
    *  \details      This function allows to change the alpha value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetAlpha(
    const float    acAlpha /*!< [in] Weight amount. Scalar value with a value in the range of [0, 1] */);

    /*!*********************************************************************************
    *  \brief        Set Alpha.
    *  \details      This function allows to read the alpha value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetAlpha(
    float&    aAlpha /*!< [in] Weight amount. Scalar value with a value in the range of [0, 1] */);

  private:
    vsdk::SUMat mAlpha;
  };


  /*!*********************************************************************************
  *  \brief          Add
  *  \details        Object of this class adds pixel value from _aSrc1_ and _aSrc2_ pixel by pixel.
  *                  _aDst_ can be VSDK_CV_8UC1 only if both source images are VSDK_CV_8UC1 and _aDst_ is explicitly set to VSDK_CV_8UC1. 
  *                  It is otherwise VSDK_CV_16SC1.<br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1  or <br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1  or <br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_16SC1, output type: VSDK_CV_16SC1  or <br>
  *                  Supported input 1 type: VSDK_CV_16SC1, input 2 type: VSDK_CV_16SC1, output type: VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Add : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Add();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Set Policy type.
    *  \details      This function allows to change the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< [in] Overflow policy type */);

    /*!*********************************************************************************
    *  \brief        Get Policy type.
    *  \details      This function allows to read the value of the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetPolicy(
    apexcv::eConvertPolicy&   aPolicy /*!< [out] Overflow policy type. */);

  private:
  vsdk::SUMat mPolicy;
  };


  /*!*********************************************************************************
  *  \brief          Bitwise AND
  *  \details        Object of this class performs a bitwise AND between pixel value of _aSrc1_ and _aSrc2_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1, or <br>
  *                  Supported input type: VSDK_CV_16UC1, output type: VSDK_CV_16UC1, or <br>
  *                  Supported input type: VSDK_CV_32UC1, output type: VSDK_CV_32UC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BitwiseAND : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~BitwiseAND();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Bitwise NOT
  *  \details        Object of this class performs a bitwise NOT of pixel value of _aSrc_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BitwiseNOT : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~BitwiseNOT();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

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
  *  \brief          Bitwise OR
  *  \details        Object of this class performs a bitwise OR between pixel value of _aSrc1_ and _aSrc2_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1, or <br>
  *                  Supported input type: VSDK_CV_16UC1, output type: VSDK_CV_16UC1, or <br>
  *                  Supported input type: VSDK_CV_32UC1, output type: VSDK_CV_32UC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BitwiseOR : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~BitwiseOR();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

  };



  /*!*********************************************************************************
  *  \brief          Bitwise exclusive OR
  *  \details        Object of this class performs a bitwise XOR between pixel value of _aSrc1_ and _aSrc2_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1, or <br>
  *                  Supported input type: VSDK_CV_16UC1, output type: VSDK_CV_16UC1, or <br>
  *                  Supported input type: VSDK_CV_32UC1, output type: VSDK_CV_32UC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class BitwiseXOR : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~BitwiseXOR();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16UC1 or VSDK_CV_32UC1). */);

  };






  /*!*********************************************************************************
  *  \brief          Count of Leading Zeros
  *  \details        Object of this class counts the number of leading zeros in the pixel value of _aSrc_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1, VSDK_CV_8SC1, VSDK_CV_16UC1 and VSDK_CV_16SC1 <br>
  *                  Supported output type: VSDK_CV_8UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Clz : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Clz();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1, VSDK_CV_8SC1, VSDK_CV_16UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1, VSDK_CV_8SC1, VSDK_CV_16UC1 or VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

  };





  /*!*********************************************************************************
  *  \brief          Magnitude
  *  \details        Object of this class computes the magnitude of pixel value of _aSrc1_ and _aSrc2_ pixel by pixel.
  *                  Where _aDst_ = SQRT( square(_aSrc1_) +  square(_aSrc2_) ) <br>
  *                  Supported input type: VSDK_CV_16SC1, output type: VSDK_CV_16UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Magnitude : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Magnitude();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_16UC1). */);

  };




  /*!*********************************************************************************
  *  \brief          Max
  *  \details        Object of this class returns the pixel-wise max values of two images. <br>
  *                  Supported input type: VSDK_CV_8UC1, VSDK_CV_16SC1, output type: VSDK_CV_8UC1, VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Max : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Max();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

  };






/*!*********************************************************************************
  *  \brief          Min
  *  \details        Object of this class returns the pixel-wise min values of two images. <br>
  *                  Supported input type: VSDK_CV_8UC1, VSDK_CV_16SC1, output type: VSDK_CV_8UC1, VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Min : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Min();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

  };

  /*!*********************************************************************************
  *  \brief          Multiplication
  *  \details        Object of this class returns element-wise multiplication between two images
  *                  then divide by 255 or right shift by a value in range of [0,15] (default is 0).
  *                  _aDst_ can be VSDK_CV_8UC1 only if both source images are VSDK_CV_8UC1 and _aDst_ is explicitly set to VSDK_CV_8UC1.
  *                  It is otherwise VSDK_CV_16SC1.<br>
  *                  Supported aSrc1 type: VSDK_CV_8UC1, aSrc1 type: VSDK_CV_8UC1, aDst type: VSDK_CV_8UC1 or <br>
  *                  Supported aSrc1 type: VSDK_CV_8UC1, aSrc1 type: VSDK_CV_8UC1, aDst type: VSDK_CV_16SC1 or <br>
  *                  Supported aSrc1 type: VSDK_CV_8UC1, aSrc1 type: VSDK_CV_16SC1, aDst type: VSDK_CV_16SC1 or <br>
  *                  Supported aSrc1 type: VSDK_CV_16SC1, aSrc1 type: VSDK_CV_16SC1, aDst type: VSDK_CV_16SC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Mul : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Mul();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1  /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2  /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Set Scale.
    *  \details      This function allows to change the scale value:
    *                (aSrc1*aSrc2)/255 for 255, (aSrc1*aSrc2)>>acScale for [0,15]
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetScale(
      const uint8_t    acScale /*!< [in] Scale amount (a value in range of [0,15] or 255)*/);

    /*!*********************************************************************************
    *  \brief        Get Scale.
    *  \details      This function allows to read the scale value.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetScale(
      uint8_t&    aScale /*!< [in] Scale amount (a value in range of [0,15] or 255)*/);

    /*!*********************************************************************************
    *  \brief        Set Policy type.
    *  \details      This function allows to change the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< [in] Overflow policy type */);

    /*!*********************************************************************************
    *  \brief        Get Policy type.
    *  \details      This function allows to read the value of the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetPolicy(
    apexcv::eConvertPolicy&   aPolicy /*!< [out] Overflow policy type. */);

  private:
    vsdk::SUMat mScale;
    vsdk::SUMat mPolicy;
  };

  /*!*********************************************************************************
  *  \brief          Phase
  *  \details        Object of this class is computes the angles for each pixel and stores this in a VSDK_CV_8UC1 image.
  *                  Where result is then translated to 0 <= result< 2pi.
  *                  Each result value is then mapped to the range 0 to 255 inclusive.<br>
  *                  Supported input type: VSDK_CV_16SC1, output type: VSDK_CV_8UC1<br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Phase : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Phase();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1  /*!< [in] Horizontal gradient (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2  /*!< [in] Vertical gradient (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Horizontal gradient (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Vertical gradient (VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);
  };


  /*!*********************************************************************************
  *  \brief          Subtract
  *  \details        Object of this class subtracts the pixel value of _aSrc2_ from _aSrc1_ pixel by pixel.
  *                  _aDst_ can be VSDK_CV_8UC1 only if both source images are VSDK_CV_8UC1 and _aDst_ is explicitly set to VSDK_CV_8UC1. 
  *                  It is otherwise VSDK_CV_16SC1. <br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_8UC1, output type: VSDK_CV_8UC1  or <br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1  or <br>
  *                  Supported input 1 type: VSDK_CV_8UC1, input 2 type: VSDK_CV_16SC1, output type: VSDK_CV_16SC1  or <br>
  *                  Supported input 1 type: VSDK_CV_16SC1, input 2 type: VSDK_CV_8UC1, output type: VSDK_CV_16SC1  or <br>
  *                  Supported input 1 type: VSDK_CV_16SC1, input 2 type: VSDK_CV_16SC1, output type: VSDK_CV_16SC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Subtract : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Subtract();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc1 /*!< [in] Source image buffer 1 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aSrc2 /*!< [in] Source image buffer 2 (VSDK_CV_8UC1, VSDK_CV_16SC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1, VSDK_CV_16SC1). */);

    /*!*********************************************************************************
    *  \brief        Set Policy type.
    *  \details      This function allows to change the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetPolicy(
    apexcv::eConvertPolicy    aPolicy /*!< [in] Overflow policy type */);

    /*!*********************************************************************************
    *  \brief        Get Policy type.
    *  \details      This function allows to read the value of the overflow policy type.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetPolicy(
    apexcv::eConvertPolicy&   aPolicy /*!< [out] Overflow policy type. */);

  private:
    vsdk::SUMat mPolicy;
  };






  /*!*********************************************************************************
  *  \brief          Table Lookup
  *  \details        Object of this class translates the pixel value of _aSrc_ through the lookup table _acLut_ pixel by pixel. <br>
  *                  Supported input type: VSDK_CV_8UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class TableLookup : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~TableLookup();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    acLut /*!< [in] Look-up table for the transformation (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    acLut /*!< [in] Look-up table for the transformation (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);
  };






  /*!*********************************************************************************
  *  \brief          Threshold
  *  \details        Object of this class thresholds the pixel value of _aSrc_ with the value of _aThreshold_ pixel by pixel. <br>
  *                  True when _aSrc_(x,y) > _aThreshold_, otherwise false. <br>
  *                  Default output values are 255 when true and 0 when false. <br>
  *                  Supported input type: VSDK_CV_8UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class Threshold : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor, Not documented.
    ***********************************************************************************/
    ~Threshold();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      const uint32_t    aThreshold /*!< [in] Threshold value. */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source image buffer (VSDK_CV_8UC1). */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination image buffer (VSDK_CV_8UC1). */);

    /*!*********************************************************************************
    *  \brief        Set Threshold.
    *  \details      This function allows to change the value of the threshold.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetThreshold(
      const uint32_t    acThreshold /*!< [in] threshold. */);

    /*!*********************************************************************************
    *  \brief        Get Threshold.
    *  \details      This function allows to read the value of the threshold.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetThreshold(
      uint32_t&    aThreshold /*!< [out] threshold. */);

    /*!*********************************************************************************
    *  \brief        Set Output Values.
    *  \details      This function allows to change the low and high output values.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetOutputValues(
      const uint8_t    acTrueVal, /*!< [in] true output value. */
      const uint8_t    acFalseVal /*!< [in] false output value. */);

    /*!*********************************************************************************
    *  \brief        Get Output Values.
    *  \details      This function allows to read the low and high output values.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetOutputValues(
      uint8_t&    aTrueVal, /*!< [out] true output value. */
      uint8_t&    aFalseVal /*!< [out] false output value. */);

  private:
    vsdk::SUMat mThreshold;
    vsdk::SUMat mOutVals;
  };






  /*!*********************************************************************************
  *  \brief          Threshold Range
  *  \details        Object of this class thresholds the pixel value of _aSrc_ with the following scheme pixel by pixel. <br>
  *                  False when _aSrc_(x,y) > _acHighThreshold_, False when _aSrc_(x,y) < _acLowThreshold_, otherwise true. <br>
  *                  Default output values are 255 when true and 0 when false. <br>
  *                  Supported input type: VSDK_CV_8UC1 <br>
  *                  Supported width: 128 to 2048 pixels.
  ***********************************************************************************/
  class ThresholdRange : public ApexcvHostBaseClass
  {
    friend class ApexCVHostUtilAri;
  public:
    /***********************************************************************************
    *  Destructor.
    ***********************************************************************************/
    ~ThresholdRange();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. */,
      const uint32_t    acLowThreshold /*!< [in] Low Threshold value. */,
      const uint32_t    acHighThreshold /*!< [in] High Threshold value. */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. */);

    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat&    aSrc /*!< [in] Source memory buffer. */,
      vsdk::SUMat&    aDst /*!< [in,out] Destination memory buffer. */);

    /*!*********************************************************************************
    *  \brief        Set Thresholds.
    *  \details      This function allows to change the value of the thresholds.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetThresholds(
      const uint32_t    acLowThreshold /*!< [in] low threshold. */,
      const uint32_t    acHighThreshold /*!< [in] high threshold. */);

    /*!*********************************************************************************
    *  \brief        Get Thresholds.
    *  \details      This function allows to read the value of the thresholds.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetThresholds(
      uint32_t&    aLowThreshold /*!< [out] low threshold. */,
      uint32_t&    aHighThreshold /*!< [out] high threshold. */);

    /*!*********************************************************************************
    *  \brief        SetOutputValues.
    *  \details      This function allows to change the true and false output values.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT SetOutputValues(
      const uint8_t    acTrueVal, /*!< [in] true output value. */
      const uint8_t    acFalseVal /*!< [in] false output value. */);

    /*!*********************************************************************************
    *  \brief        Get Output Values.
    *  \details      This function allows to read the true and false output values.
    *  \return       APEXCV_LIB_RESULT Error code.
    ***********************************************************************************/
    APEXCV_LIB_RESULT GetOutputValues(
      uint8_t&    aTrueVal, /*!< [out] true output value. */
      uint8_t&    aFalseVal /*!< [out] false output value. */);

  private:
    vsdk::SUMat mThresholds;
    vsdk::SUMat mOutVals;
  };


}  /* namespace apexcv */
#endif /* APEXCVBASEARITHMETIC_HPP */
