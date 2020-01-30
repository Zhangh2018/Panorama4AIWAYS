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
*  \file apexcv_image_filters_ref.hpp
*  \brief Reference code for filter kernels
***********************************************************************************/

#ifndef IMAGEFILTERREF_H
#define IMAGEFILTERREF_H

#include <stdint.h>
#include <sumat.hpp>


/*!
   Calculate a 3x3 dilate filter.

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
   */
void dilate_3x3_08u_ref(uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh);

/*!
   Calculate a 3x3 dilate filter.

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
   */
void dilate_3x3_16s_ref(int16_t* dst, int dstr,
  const int16_t* src, int sstr,
  int bw, int bh);

/*!
   Calculate a 3x3 erode filter.

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
   */
void erode_3x3_08u_ref(uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh);

/*!
   Calculates 5x5 bilateral filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param sigmaR - [Input]  Intensity sigma
   \param sigmaD - [Input]  Distance sigma
   */
void
bilateral_5x5_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int sigmaR, int sigmaD);

/*!
   Calculates 3x3 box filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
box_3x3_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 3x3 box filter of image with no rounding

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
box_3x3_nr_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 3x3 box filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
box_3x3_ref(int16_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 5x5 box filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
box_5x5_16s_ref(int16_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 7x7 box filter of image

   \param aDst    - [Output] Destination block pointer
   \param aSrc    - [Input]  Source value block pointer
   */
void box_7x7_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst);

/*!
   Calculates box filter of image

   \param aDst         - [Output] Destination block pointer
   \param aSrc         - [Input]  Source value block pointer
   \param aWindowSize  - [Input]  Window size 7 or 9
   */
void box_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst, int aWindowSize);

void
census_3x3_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 3x3 central x derivative of an image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param k1     - [Input]  1st Coefficient
   \param k2     - [Input]  2nd Coefficient
   \param k3     - [Input]  3rd Coefficient
   */
void
derivative_x_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int k0, int k1, int k2);

/*!
   Calculates 3x3 central y derivative of an image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param k1     - [Input]  1st Coefficient
   \param k2     - [Input]  2nd Coefficient
   \param k3     - [Input]  3rd Coefficient
   */
void
derivative_y_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int k0, int k1, int k2);

/*!
   Calculate a 3x3 median filter.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
median_3x3_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate a 5x5 median filter.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
median_5x5_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 7x7 median filter of image
   
   \param aSrc    - [Input]  Source array
   \param aDst    - [Output] Destination array
   \param aWindowSize - [Input] value window size
   */
void median_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst, int aWindowSize);

/*!
   Calculates 3x3 gaussian filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
gaussian_3x3_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 5x5 gaussian filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   */
void
gaussian_5x5_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculates 7x7 gaussian filter of image
   
   \param aSrc    - [Input]  Source array
   \param aDst    - [Output] Destination array
   */
void
gaussian_7x7_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst);

/*!
   Calculates 9x9 gaussian filter of image
   
   \param aSrc    - [Input]  Source array
   \param aDst    - [Output] Destination array
   */
void
gaussian_9x9_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst);

/*!
   Perform a 16s to 08s saturation. Scales down by 8 and
   rounds before saturation.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
saturate_08s_ref(
int8_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh);

/*!
   Calculate sum of absolute values of first order derivatives x and y using 3x3 and 5x5.
   \param aDst  - [Output] Destination block pointer
   \param aSrc  - [Input]  Source block pointer
   \param aWindowSize  - [Input]  Window size 3 or 5
   */
void
scharr_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst);

/*!
   Calculate first order derivative x using scharr 3x3 and 5x5.
   \param aDst  - [Output] Destination block pointer
   \param aSrc  - [Input]  Source block pointer
   \param aWindowSize  - [Input]  Window size 3 or 5
   */
void
scharr_x_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst);

/*!
   Calculate first order derivative y using scharr 3x3 and 5x5.
   \param aDst  - [Output] Destination block pointer
   \param aSrc  - [Input]  Source block pointer
   \param aWindowSize  - [Input]  Window size 3 or 5
   */
void
scharr_y_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst);


/*!
   Calculate first order derivative xy using scharr 3x3 and 5x5.
   \param aDst0  - [Output] Destination block pointer
   \param aDst1  - [Output] Destination block pointer
   \param aSrc   - [Input]  Source block pointer
   \param aWindowSize  - [Input]  Window size 3 or 5
   */
void
scharr_xy_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst0, vsdk::SMat& aDst1);

/*!
   Calculates 3x3 separable filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param fRow   - [Input]  Row coefficients
   \param fCol   - [Input]  Column Coefficients
   */
void
separable_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const signed char* fRow,
const signed char* fCol);

/*!
   Calculates 5x5 separable filter of image

   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source value block pointer
   \param sstr   - [Input]  Source value block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param fRow   - [Input]  Row coefficients
   \param fCol   - [Input]  Column Coefficients
   */
void
separable_5x5_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const signed char* fRow,
const signed char* fCol);

/*!
   Calculate first order derivative x using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_x_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate first order derivative x using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_x_3x3_08s_ref(
int8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate first order derivative y using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_y_3x3_08s_ref(
int8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate first order derivative y using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_y_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate first order derivative x using sobel 5x5.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_x_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate first order derivative y using sobel 5x5.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_y_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate sum of absolute values of first order derivatives x and y using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate sum of absolute values of first order derivatives x and y using sobel 3x3.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_3x3_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Calculate sum of absolute values of first order derivatives x and y using sobel 5x5.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
sobel_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Convolve: Calculate the output of generic (fw x fh) filter.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param flt  - [Input]  Filter Coeffiencients (pointer)
   \param fw   - [Input]  Filter width
   \param fh   - [Input]  Filter height
   */
void
convolve_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh);

/*!
   Convolve: Calculate the output of generic (fw x fh) filter with scaling.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param flt  - [Input]  Filter Coeffiencients (pointer)
   \param fw   - [Input]  Filter width
   \param fh   - [Input]  Filter height
   \param fscl - [Input]  Filter Scale (2^fscl)
   */
void
convolve_scale_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl);

/*!
   Convolve: Calculate the output of generic (fw x fh) filter with scaling.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param flt  - [Input]  Filter Coeffiencients (pointer)
   \param fw   - [Input]  Filter width
   \param fh   - [Input]  Filter height
   \param fscl - [Input]  Filter Scale (2^fscl)
   */
void
convolve_scale_08u_nr_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl);

/*!
   Convolve: Calculate the output of generic (fw x fh) filter.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param flt  - [Input]  Filter Coeffiencients (pointer)
   \param fw   - [Input]  Filter width
   \param fh   - [Input]  Filter height
   */
void
convolve_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh);

/*!
   Convolve: Calculate the output of generic (fw x fh) filter with scaling.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param flt  - [Input]  Filter Coeffiencients (pointer)
   \param fw   - [Input]  Filter width
   \param fh   - [Input]  Filter height
   \param fscl - [Input]  Filter Scale (2^fscl)
   */
void
convolve_scale_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl);

/*!
   Applies a 3x3 prewitt filter in the x direction.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
prewitt_x_3x3_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

/*!
   Applies a 3x3 prewitt filter in the y direction.

   \param dst  - [Output] Destination block pointer
   \param dstr - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer
   \param sstr - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   */
void
prewitt_y_3x3_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh);

#endif /* IMAGEFILTERREF_H */
