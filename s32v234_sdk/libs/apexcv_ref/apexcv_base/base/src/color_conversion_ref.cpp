/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2018 NXP
* All Rights Reserved
*
*****************************************************************************
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
*  \file apexcv_color_conversion_ref.cpp
*  \brief Reference code for color conversion kernels
***********************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <color_conversion_ref.h>

void rgb565_to_rgb888x_ref(uint32_t* dst, uint32_t dstr,
  const uint16_t* src, uint32_t sstr,
  uint32_t bw, uint32_t bh)
{
  uint8_t r, g, b;

  for (uint32_t y = 0; y < bh; ++y)
  {
    for (uint32_t x = 0; x < bw; ++x)
    {

      r = (src[x] >> 8) & 0xF8;
      g = (src[x] >> 3) & 0xFC;
      b = (src[x] << 3) & 0xF8;

      r = r | (r >> 5);
      g = g | (g >> 6);
      b = b | (b >> 5);

      dst[x] = (r << 16) | (g << 8) | b;
    }

    dst += dstr;
    src += sstr;
  }
}

void rgb888x_to_rgb565_ref(uint16_t* dst, uint32_t dstr,
  const uint32_t * src, uint32_t sstr,
  uint32_t bw, uint32_t bh)
{
  uint8_t r, g, b;

  for (uint32_t y = 0; y < bh; ++y)
  {
    for (uint32_t x = 0; x < bw; ++x)
    {

      r = src[x] >> 19;
      g = (src[x] >> 10) & 0x3F;
      b = (src[x] >> 3) & 0x1F;

      dst[x] = (r << 11) | (g << 5) | b;
    }

    dst += dstr;
    src += sstr;
  }
}

void rgb888x_to_y_ref(uint8_t* dst, uint32_t ds,
  const uint8_t* src, uint32_t ss,
  uint32_t bw, uint32_t bh,
  uint32_t R2Y, uint32_t G2Y, uint32_t B2Y)
{
  for (uint32_t y = 0; y < bh; ++y)

  {
    const uint8_t* s = (const uint8_t*)src;
    uint8_t* d = (uint8_t*)dst;

    const uint32_t rnd = 128;
    const uint32_t scl = 8;

    for (uint32_t x = 0; x < bw; x += 16)
    {
      // absolutly required by chunck of 3 (otherwise it is less efficient)
      const uint16_t r0 = s[0];
      const uint16_t g0 = s[1];
      const uint16_t b0 = s[2]; s += 4;

      const uint16_t y0 =
        (rnd
        + (r0 * R2Y)
        + (g0 * G2Y)
        + (b0 * B2Y)) >> scl;

      const uint16_t r1 = s[0];
      const uint16_t g1 = s[1];
      const uint16_t b1 = s[2]; s += 4;

      const uint16_t r2 = s[0];
      const uint16_t g2 = s[1];
      const uint16_t b2 = s[2]; s += 4;

      const uint16_t r3 = s[0];
      const uint16_t g3 = s[1];
      const uint16_t b3 = s[2]; s += 4;

      const uint16_t y1 =
        (rnd
        + (r1 * R2Y)
        + (g1 * G2Y)
        + (b1 * B2Y)) >> scl;

      const uint16_t y2 =
        (rnd
        + (r2 * R2Y)
        + (g2 * G2Y)
        + (b2 * B2Y)) >> scl;
      const uint16_t y3 =
        (rnd
        + (r3 * R2Y)
        + (g3 * G2Y)
        + (b3 * B2Y)) >> scl;

      d[0] = (y0 > 255) ? 255 : (uint8_t)y0;
      d[1] = (y1 > 255) ? 255 : (uint8_t)y1;
      d[2] = (y2 > 255) ? 255 : (uint8_t)y2;
      d[3] = (y3 > 255) ? 255 : (uint8_t)y3; d += 4;
    }

    src += ss;
    dst += ds;
  }
}

void rgb888x_to_yuv_ref(uint8_t* dst, uint32_t ds,
  const uint8_t* src, uint32_t ss,
  uint32_t bw, uint32_t bh)
{
  for (uint32_t i = 0; i < bh; ++i)
  {
    const uint8_t* s = (const uint8_t*)src;
    uint8_t* d = (uint8_t*)dst;

    for (uint32_t x = 0; x < 4 * bw; x += 4)
    {
      // must be vec16s
      const uint16_t r = (uint16_t)s[x + 0];
      const uint16_t g = (uint16_t)s[x + 1];
      const uint16_t b = (uint16_t)s[x + 2];


      const uint16_t y = (
        +(b * 29)
        + (r * 77)
        + (g * 150)
        + 128
        ) >> 8;

      const uint16_t u = (
        +(r * -43)
        + (g * -85)
        + (b * 128)
        + 128
        ) >> 8;

      const uint16_t v = (
        +(b * -21)
        + (g *-107)
        + (r * 128)
        + 128
        ) >> 8;

      d[x + 0] = (uint8_t)(y);
      d[x + 1] = (uint8_t)(u + 128);
      d[x + 2] = (uint8_t)(v + 128);
      d[x + 3] = 0;

    }

    src += ss;
    dst += ds;
  }
}

void csc_8UC3_to_y_ref(int16_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh,
  uint8_t KA, uint8_t KB, uint8_t KC, uint16_t KR
  )
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      uint16_t c1 = KA*src[4 * j];
      uint16_t c2 = KB*src[4 * j + 1];
      uint16_t c3 = KC*src[4 * j + 2];
      dst[j] = (int16_t)(c1 + c2 + c3 + KR) >> 8;
    }
    dst += ds;
    src += ss;
  }
}

void rgb888_to_grey_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  )
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      uint16_t c1 = 27*src[3 * j + 0];
      uint16_t c2 = 92*src[3 * j + 1];
      uint16_t c3 = 9*src[3 * j + 2];
      dst[j] = (uint8_t)((c1 + c2 + c3 ) >> 7);
    }
    dst += ds;
    src += ss;
  }
}

void bgr888_to_grey_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  )
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      uint16_t c1 = 27 * src[3 * j + 2];
      uint16_t c2 = 92 * src[3 * j + 1];
      uint16_t c3 = 9 * src[3 * j + 0];
      dst[j] = (uint8_t)((c1 + c2 + c3) >> 7);
    }
    dst += ds;
    src += ss;
  }
}

void grey_to_rgb888_ref(uint8_t* dst, int ds,
  const uint8_t* src, int ss,
  int bw, int bh
  )
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      uint8_t c1 = src[j];
      dst[3 * j + 0] = c1;
      dst[3 * j + 1] = c1;
      dst[3 * j + 2] = c1;
    }
    dst += ds;
    src += ss;
  }
}

void extract_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh,
  int ch_index, int num_ch)
{
  int ch_offset = ch_index - 1;
  const uint8_t* src_offset = src + ch_offset;

  for (int y = 0; y < bh; ++y)
  {
    int offset = 0;
    for (int x = 0; x < bw; ++x)
    {
      dst[x] = src_offset[offset];
      offset += num_ch;
    }
    src_offset += sstr;
    dst += dstr;
  }
}

void insert_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh,
  int ch_index, int num_ch)
{
  int ch_offset = ch_index - 1;
  uint8_t* dst_offset = dst + ch_offset;

  for (int y = 0; y < bh; ++y)
  {
    int offset = 0;
    for (int x = 0; x < bw; ++x)
    {
      dst_offset[offset] = src[x];
      offset += num_ch;
    }
    src += sstr;
    dst_offset += dstr;
  }
}


void ch_extr_ref(
  uint8_t* dst, int dstr,
  uint8_t* src, int sstr,
  int bw, int bh, int num_ch)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst[x] = src[num_ch * x];
    }
    src += sstr;
    dst += dstr;
  }
}



void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  uint8_t* dst3, int dstr3,
  uint8_t* dst4, int dstr4,
  const uint8_t* src, int sstr,
  int bw, int bh
)
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst1[x] = src[x * 4 + 0];
      dst2[x] = src[x * 4 + 1];
      dst3[x] = src[x * 4 + 2];
      dst4[x] = src[x * 4 + 3];
    }
    src += sstr;
    dst1 += dstr1;
    dst2 += dstr2;
    dst3 += dstr3;
    dst4 += dstr4;
  }
}


void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  uint8_t* dst3, int dstr3,
  const uint8_t* src, int sstr,
  int bw, int bh
  )
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst1[x] = src[x * 3 + 0];
      dst2[x] = src[x * 3 + 1];
      dst3[x] = src[x * 3 + 2];
    }
    src += sstr;
    dst1 += dstr1;
    dst2 += dstr2;
    dst3 += dstr3;
  }
}


void split_channel_ref(
  uint8_t* dst1, int dstr1,
  uint8_t* dst2, int dstr2,
  const uint8_t* src, int sstr,
  int bw, int bh
  )
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst1[x] = src[x * 2 + 0];
      dst2[x] = src[x * 2 + 1];
    }
    src += sstr;
    dst1 += dstr1;
    dst2 += dstr2;
  }
}




void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  const uint8_t* src3, int sstr3,
  const uint8_t* src4, int sstr4,
  int bw, int bh
  )
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst[x * 4 + 0] = src1[x];
      dst[x * 4 + 1] = src2[x];
      dst[x * 4 + 2] = src3[x];
      dst[x * 4 + 3] = src4[x];
    }
    src1 += sstr1;
    src2 += sstr2;
    src3 += sstr3;
    src4 += sstr4;
    dst += dstr;
  }
}


void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  const uint8_t* src3, int sstr3,
  int bw, int bh
  )
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst[x * 3 + 0] = src1[x];
      dst[x * 3 + 1] = src2[x];
      dst[x * 3 + 2] = src3[x];
    }
    src1 += sstr1;
    src2 += sstr2;
    src3 += sstr3;
    dst += dstr;
  }
}


void merge_channel_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src1, int sstr1,
  const uint8_t* src2, int sstr2,
  int bw, int bh
  )
{

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      dst[x * 2 + 0] = src1[x];
      dst[x * 2 + 1] = src2[x];
    }
    src1 += sstr1;
    src2 += sstr2;
    dst += dstr;
  }
}




void ch_passthrough_ref(
  uint8_t* dst, int dstr,
  uint8_t* src, int sstr,
  int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint8_t data0 = src[x];
      dst[x] = data0;
    }
    src += sstr;
    dst += dstr;
  }
}

void extract_channel2_ref(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  ch_extr_ref(op, os, ip + channel, is, width, height, 2);
}

void extract_channel3_ref(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  ch_extr_ref(op, os, ip + channel, is, width, height, 3);
}

void extract_channel4_ref(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  ch_extr_ref(op, os, ip + channel, is, width, height, 4);
}

void YUYV_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  if (0 == channel)
  {
    ch_extr_ref(op, os, ip, is, width, height, 2);
  }
  else
  {
    int ch_to_extract = 2 * channel - 1;
    ch_extr_ref(op, os, ip + ch_to_extract, is, width / 2, height, 4);
  }
}

void UYVY_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  if (0 == channel)
  {
    ch_extr_ref(op, os, ip + 1, is, width, height, 2);
  }
  else
  {
    int ch_to_extract = 2 * channel - 2;
    ch_extr_ref(op, os, ip + ch_to_extract, is, width / 2, height, 4);
  }
}

void YUV4_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  int plane_offset = channel * is * height;
  ch_passthrough_ref(op, os, ip + plane_offset, is, width, height);
}

void IYUV_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  if (0 == channel)
  {
    ch_passthrough_ref(op, os, ip, is, width, height);
  }
  else
  {
    int plane_offset = is*height + (channel - 1) * (is / 2) * (height / 2);
    ch_passthrough_ref(op, os, ip + plane_offset, is / 2, width / 2, height / 2);
  }
}
void NV12_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  if (0 == channel)
  {
    ch_passthrough_ref(op, os, ip, is, width, height);
  }
  else
  {
    int ch_to_extract = channel - 1;
    int plane_offset = is * height;
    ch_extr_ref(op, os, ip + plane_offset + ch_to_extract, is, width / 2, height / 2, 2);
  }
}
void NV21_extract(
  uint8_t* op, int os,
  uint8_t* ip, int is,
  int width, int height,
  int channel)
{
  if (0 == channel)
  {
    ch_passthrough_ref(op, os, ip, is, width, height);
  }
  else
  {
    int ch_to_extract = 1 - (channel - 1);
    int plane_offset = is * height;
    ch_extr_ref(op, os, ip + plane_offset + ch_to_extract, is, width / 2, height / 2, 2);
  }
}


/*!*********************************************************************************
*  \brief List of conversion types
***********************************************************************************/

void up_convert_ref( vsdk::SMat&    aSrc,
      int32_t    aShift,
      vsdk::SMat&    aDst)
{
  if((aShift<0) || (aShift>=8))
  {
    printf("Error: convert_depth_ref::aShift invalid argument\n");
    return;
  }
  int32_t bw = aSrc.cols;
  int32_t bh = aSrc.rows;

  for(int32_t y=0;y<bh;y++)
  {
    for(int32_t x=0;x<bw;x++)
    {
      aDst.at<int16_t>(y*bw+x)=aSrc.at<uint8_t>(y*bw+x)<<aShift;
    }
  }
}

void down_convert_ref( vsdk::SMat&    aSrc,
      apexcv::eConvertPolicy    aPolicy,
      int32_t    aShift,
      vsdk::SMat&    aDst)
{
  if((aShift<0) || (aShift>=8))
  {
    printf("Error: convert_depth_ref::aShift invalid argument\n");
    return;
  }
  int32_t bw = aSrc.cols;
  int32_t bh = aSrc.rows;

  if(aPolicy==apexcv::eConvertPolicy::WRAP)
  {
    for(int32_t y=0;y<bh;y++)
    {
      for(int32_t x=0;x<bw;x++)
      {
        aDst.at<uint8_t>(y*bw+x)=aSrc.at<int16_t>(y*bw+x)>>aShift;
      }
    }
  }
  else if(aPolicy==apexcv::eConvertPolicy::SATURATE)
  {
    int16_t value;
    for(int32_t y=0;y<bh;y++)
    {
      for(int32_t x=0;x<bw;x++)
      {
        value = aSrc.at<int16_t>(y*bw+x)>>aShift;
        value = (value < 0 ? 0 : value);
        value = (value > 255 ? 255 : value);
        aDst.at<uint8_t>(y*bw+x)=value;
      }
    }
  }
  else
  {
    printf("Error: convert_depth_ref::aPolicy invalid argument\n");
    return;
  }
}
