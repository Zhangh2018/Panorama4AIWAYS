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
*  \file apexcv_color_conversion_ref.hpp
*  \brief Reference code for color conversion kernels
***********************************************************************************/

#ifndef COLORCONVERSIONREF_H
#define COLORCONVERSIONREF_H

#include <stdint.h>
#include <apexcv_enum_types.hpp>
#include <sumat.hpp>

/*!
   Converts 16 bit RGB565 to 24 bit RGB888

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
   */
void rgb565_to_rgb888x_ref(uint32_t* dst, uint32_t dstr,
  const uint16_t* src, uint32_t sstr,
  uint32_t bw, uint32_t bh);


/*!
   Converts 24 bit RGB888 to 16 bit RGB565

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
   */
void rgb888x_to_rgb565_ref(uint16_t* dst, uint32_t dstr,
  const uint32_t * src, uint32_t sstr,
  uint32_t bw, uint32_t bh);

/*!
   Calculate grayscale pixel from RGBX pixel.

   \param dst  - [Output] Destination block pointer   (Grayscale)
   \param ds   - [Input]  Destination block stride
   \param src  - [Input]  Source block pointer        (RGBX)
   \param ss   - [Input]  Source block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param R2Y  - [Input]  Red to gray conversion factor
   \param G2Y  - [Input]  Green to gray conversion factor
   \param B2Y  - [Input]  Blue to gray conversion factor
   */
void rgb888x_to_y_ref(uint8_t* dst, uint32_t ds,
  const uint8_t* src, uint32_t ss,
  uint32_t bw, uint32_t bh,
  uint32_t R2Y, uint32_t G2Y, uint32_t B2Y);

/*!
   Calculate YUVX pixel from RGBX pixel.

   References on formula and values used for conversion:
   http://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.601-7-201103-I!!PDF-E.pdf
   http://www.poynton.com/PDFs/ColorFAQ.pdf

   \param dst  - [Output] Destination block pointer  (YUVX)
   \param ds   - [Input]  Destination block span
   \param src  - [Input]  Source block pointer       (RGBX)
   \param ss   - [Input]  Source block span
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height

   */
void rgb888x_to_yuv_ref(uint8_t* dst, uint32_t ds,
  const uint8_t* src, uint32_t ss,
  uint32_t bw, uint32_t bh);

/*!
Calculate grey value pixel from RGB pixel.

References on formula and values used for conversion:
https://en.wikipedia.org/wiki/Grayscale#Converting_color_to_grayscale

\param dst  - [Output] Destination block pointer  (Grey)
\param ds   - [Input]  Destination block span
\param src  - [Input]  Source block pointer       (RGB)
\param ss   - [Input]  Source block span
\param bw   - [Input]  Block width
\param bh   - [Input]  Block height

*/
void rgb888_to_grey_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  );

/*!
Calculate grey value pixel from BGR pixel.

References on formula and values used for conversion:
https://en.wikipedia.org/wiki/Grayscale#Converting_color_to_grayscale

\param dst  - [Output] Destination block pointer  (Grey)
\param ds   - [Input]  Destination block span
\param src  - [Input]  Source block pointer       (BGR)
\param ss   - [Input]  Source block span
\param bw   - [Input]  Block width
\param bh   - [Input]  Block height

*/
void bgr888_to_grey_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  );

/*!
copy grey value pixel to RGB pixel.

\param dst  - [Output] Destination block pointer  (RGB)
\param ds   - [Input]  Destination block span
\param src  - [Input]  Source block pointer       (Grey)
\param ss   - [Input]  Source block span
\param bw   - [Input]  Block width
\param bh   - [Input]  Block height

*/
void grey_to_rgb888_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  );


/*!
   Calculate grayscale pixel from 3-Channel pixel.

   \param dst  - [Output] Destination block pointer   (Grayscale)
   \param ds   - [Input]  Destination block span
   \param src  - [Input]  Source block pointer        (RGBX)
   \param ss   - [Input]  Source block span
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height
   \param KA   - [Input]  Channel 1 to gray conversion factor
   \param KB   - [Input]  Channel 2 to gray conversion factor
   \param KC   - [Input]  Channel 3 to gray conversion factor
   \param KR   - [Input]  Pixel Addition factor
   */
void csc_8UC3_to_y_ref(int16_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh,
  uint8_t KA, uint8_t KB, uint8_t KC, uint16_t KR);

/*!
   Extract channel

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract
   \param channel - [Input]  Number of Channels in source
   */
void extract_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh,
  int ch_index, int num_ch);

/*!
   Insert channel.

   \param op      - [Output] Destination block pointer
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer   (Grayscale)
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract
   \param channel - [Input]  Number of Channels in source
   */
void insert_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh,
  int ch_index, int num_ch);


/*!
Split channel

\param dst      - [Output] Destination block pointer   (each single channel)
\param dstr      - [Input]  Destination block span
\param src      - [Input]  Source block pointer   (multi channel)
\param sstr      - [Input]  Source block span
\param width   - [Input]  Block width
\param height  - [Input]  Block height
*/
void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  uint8_t* dst3, int dstr3,
  uint8_t* dst4, int dstr4,
  const uint8_t* src, int sstr,
  int bw, int bh
  );

void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  uint8_t* dst3, int dstr3,
  const uint8_t* src, int sstr,
  int bw, int bh
  );

void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  const uint8_t* src, int sstr,
  int bw, int bh
  );


/*!
Merge channel

\param dst      - [Output] Destination block pointer   (multi channel)
\param dstr      - [Input]  Destination block span
\param src      - [Input]  Source block pointer  (each single channel)
\param sstr      - [Input]  Source block span
\param width   - [Input]  Block width
\param height  - [Input]  Block height
*/

void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  const uint8_t* src3, int sstr3,
  const uint8_t* src4, int sstr4,
  int bw, int bh
  );

  void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  const uint8_t* src3, int sstr3,
  int bw, int bh
  );

  void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  int bw, int bh
  );


/*!
   Extract channel from YUYV.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void YUYV_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Extract channel from UYVY.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void UYVY_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Extract channel from YUV4.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void YUV4_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Extract channel from IYUV.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void IYUV_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Extract channel from NV12.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void NV12_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Extract channel from NV21.

   \param op      - [Output] Destination block pointer   (Grayscale)
   \param os      - [Input]  Destination block span
   \param ip      - [Input]  Source block pointer
   \param is      - [Input]  Source block span
   \param width   - [Input]  Block width
   \param height  - [Input]  Block height
   \param channel - [Input]  Channel to extract 0(Y), 1(U), 2(V)
   */
void NV21_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel);

/*!
   Object of this class performs a converts image bit depth.
   \param aDst    - [Output] Destination block pointer
   \param aShift  - [Input] Source pixel value shift amount (0 <= aShift < 8 )
   \param aSrc    - [Input] Source block pointer
   */
void up_convert_ref( vsdk::SMat&    aSrc,
      int32_t    aShift,
      vsdk::SMat&    aDst);


/*!
   Object of this class performs a converts image bit depth.
   \param aDst    - [Output] Destination block pointer
   \param aPolicy - [Input] Source policy type
   \param aShift  - [Input] Source pixel value shift amount (0 <= aShift < 8 )
   \param aSrc    - [Input] Source block pointer
   */
void down_convert_ref( vsdk::SMat&    aSrc,
      apexcv::eConvertPolicy    aPolicy,
      int32_t    aShift,
      vsdk::SMat&    aDst);

#endif /* COLORCONVERSIONREF_HPP */
