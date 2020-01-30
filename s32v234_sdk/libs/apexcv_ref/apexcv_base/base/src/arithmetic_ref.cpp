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
*  \file apexcv_arithmetic_ref-impl.cpp
*  \brief Contains template definitions for reference code
***********************************************************************************/

#include <arithmetic_ref.hpp>
#include <stdint.h>

template void abs_ref<int8_t, uint8_t>(const int8_t* src, int sstr, uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void abs_ref<int16_t, uint16_t>(const int16_t* src, int sstr, uint16_t* dst, int dstr, int blockWidth, int blockHeight);
template void abs_ref<int32_t, uint32_t>(const int32_t* src, int sstr, uint32_t* dst, int dstr, int blockWidth, int blockHeight);

template void absdiff_ref<unsigned char>(const unsigned char* src0, int sstr0, const unsigned char* src1, int sstr1,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight);
template void absdiff_ref<unsigned short>(const unsigned short* src0, int sstr0, const unsigned short* src1, int sstr1,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);
template void absdiff_ref<unsigned int>(const unsigned int* src0, int sstr0, const unsigned int* src1, int sstr1,
  unsigned int* dst, int dstr, int blockWidth, int blockHeight);
template void absdiff_ref<char>(const char* src0, int sstr0, const char* src1, int sstr1,
  char* dst, int dstr, int blockWidth, int blockHeight);
template void absdiff_ref<short>(const short* src0, int sstr0, const short* src1, int sstr1,
  short* dst, int dstr, int blockWidth, int blockHeight);
template void absdiff_ref<int>(const int* src0, int sstr0, const int* src1, int sstr1,
  int* dst, int dstr, int blockWidth, int blockHeight);


void accumulate_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst)
{
  int bh = aDst.rows;
  int bw = aDst.cols;
  const uint8_t *src0 = aSrc0.data;
  const int16_t *src1 = (int16_t*) aSrc1.data;
  int16_t *dst = (int16_t*) aDst.data;
  long mx = numeric_limits<int16_t>::max();
  long mn = numeric_limits<int16_t>::min();
  for (int by = 0; by < bh; by++)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int idx = bx + bw* by;

      int16_t tx = (int16_t)src0[idx] + (int16_t)src1[idx];
      if (tx > mx)
      {
        dst[idx] = (int16_t)mx;
      }
      else if (tx < mn)
      {
        dst[idx] = (int16_t)mn;
      }
      else
      {
        dst[idx] = (int16_t)tx;
      }
    }
  }
}

void accumulate_squared_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, uint8_t aScale, vsdk::SMat& aDst)
{
  int bh = aDst.rows;
  int bw = aDst.cols;
  const uint8_t *src0 = aSrc0.data;
  const int16_t *src1 = (int16_t*) aSrc1.data;
  int16_t *dst = (int16_t*) aDst.data;
  long mx = numeric_limits<int16_t>::max();
  long mn = numeric_limits<int16_t>::min();
  for (int by = 0; by < bh; by++)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int idx = bx + bw* by;

      int32_t s1i = (int32_t)src0[idx];
      int32_t s2i = (int32_t)src1[idx];
      int32_t tx = (int32_t)(s2i + ((int32_t)(s1i*s1i) >> aScale));
      if (tx > mx)
      {
        dst[idx] = (int16_t)mx;
      }
      else if (tx < mn)
      {
        dst[idx] = (int16_t)mn;
      }
      else
      {
        dst[idx] = (int16_t)tx;
      }
    }
  }
}


void accumulate_weighted_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, float aAlpha ,vsdk::SMat& aDst)
{
  int bh = aDst.rows;
  int bw = aDst.cols;
  double s1i;
  double s2i;
  const uint8_t *src0 = aSrc0.data;
  const uint8_t *src1 = aSrc1.data;
  for (int by = 0; by < bh; by++)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int idx = bx + bw* by;
      s1i = (double)src0[idx] * (double)aAlpha;
      s2i = (double)src1[idx] * (1.0f - (double)aAlpha);
      aDst.data[idx] = static_cast<uint8_t>(s1i + s2i);// valid value is this or +1
    }
  }
}




template void _add_ref<uint8_t, uint8_t, uint8_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _add_ref<uint8_t, uint8_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _add_ref<int16_t, int16_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _add_ref<uint8_t, int16_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);

void add_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_8UC1))
  {
    _add_ref<uint8_t, uint8_t, uint8_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _add_ref<uint8_t, uint8_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _add_ref<uint8_t, int16_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_16SC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _add_ref<int16_t, int16_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
}


template void _sub_ref<uint8_t, uint8_t, uint8_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _sub_ref<uint8_t, uint8_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _sub_ref<int16_t, int16_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _sub_ref<int16_t, uint8_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);
template void _sub_ref<uint8_t, int16_t, int16_t>(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);

void sub_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_8UC1))
  {
    _sub_ref<uint8_t, uint8_t, uint8_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _sub_ref<uint8_t, uint8_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _sub_ref<uint8_t, int16_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_16SC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _sub_ref<int16_t, uint8_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_16SC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    _sub_ref<int16_t, int16_t, int16_t>(aSrc0, aSrc1, aDst, aPolicy);
  }
}


template void and_ref<unsigned char>(const unsigned char* src0, int sstr0, const unsigned char* src1, int sstr1,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight);
template void and_ref<unsigned short>(const unsigned short* src0, int sstr0, const unsigned short* src1, int sstr1,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);
template void and_ref<unsigned int>(const unsigned int* src0, int sstr0, const unsigned int* src1, int sstr1,
  unsigned int* dst, int dstr, int blockWidth, int blockHeight);
template void and_ref<char>(const char* src0, int sstr0, const char* src1, int sstr1,
  char* dst, int dstr, int blockWidth, int blockHeight);
template void and_ref<short>(const short* src0, int sstr0, const short* src1, int sstr1,
  short* dst, int dstr, int blockWidth, int blockHeight);
template void and_ref<int>(const int* src0, int sstr0, const int* src1, int sstr1,
  int* dst, int dstr, int blockWidth, int blockHeight);

template void clz_ref<uint8_t, uint8_t>(uint8_t* src, int sstr, uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void clz_ref<int8_t, uint8_t>(int8_t* src, int sstr, uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void clz_ref<uint16_t, uint8_t>(uint16_t* src, int sstr, uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void clz_ref<int16_t, uint8_t>(int16_t* src, int sstr, uint8_t* dst, int dstr, int blockWidth, int blockHeight);

template void magnitude_ref<short, unsigned short>(const short* src0, int sstr0, const short* src1, int sstr1,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);

template void max_ref<uint8_t, uint8_t>(const uint8_t* src0, int sstr0, const uint8_t* src1, int sstr1,
  uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void max_ref<int16_t, int16_t>(const int16_t* src0, int sstr0, const int16_t* src1, int sstr1,
  int16_t* dst, int dstr, int blockWidth, int blockHeight);

template void min_ref<uint8_t, uint8_t>(const uint8_t* src0, int sstr0, const uint8_t* src1, int sstr1,
  uint8_t* dst, int dstr, int blockWidth, int blockHeight);
template void min_ref<int16_t, int16_t>(const int16_t* src0, int sstr0, const int16_t* src1, int sstr1,
  int16_t* dst, int dstr, int blockWidth, int blockHeight);


template void not_ref<unsigned char>(const unsigned char* src0, int sstr0,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight);
template void not_ref<unsigned short>(const unsigned short* src0, int sstr0,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);
template void not_ref<unsigned int>(const unsigned int* src0, int sstr0,
  unsigned int* dst, int dstr, int blockWidth, int blockHeight);
template void not_ref<char>(const char* src0, int sstr0,
  char* dst, int dstr, int blockWidth, int blockHeight);
template void not_ref<short>(const short* src0, int sstr0,
  short* dst, int dstr, int blockWidth, int blockHeight);
template void not_ref<int>(const int* src0, int sstr0,
  int* dst, int dstr, int blockWidth, int blockHeight);

template void or_ref<unsigned char>(const unsigned char* src0, int sstr0, const unsigned char* src1, int sstr1,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight);
template void or_ref<unsigned short>(const unsigned short* src0, int sstr0, const unsigned short* src1, int sstr1,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);
template void or_ref<unsigned int>(const unsigned int* src0, int sstr0, const unsigned int* src1, int sstr1,
  unsigned int* dst, int dstr, int blockWidth, int blockHeight);
template void or_ref<char>(const char* src0, int sstr0, const char* src1, int sstr1,
  char* dst, int dstr, int blockWidth, int blockHeight);
template void or_ref<short>(const short* src0, int sstr0, const short* src1, int sstr1,
  short* dst, int dstr, int blockWidth, int blockHeight);
template void or_ref<int>(const int* src0, int sstr0, const int* src1, int sstr1,
  int* dst, int dstr, int blockWidth, int blockHeight);


static uint16_t clz16(uint16_t a)
{
  uint16_t n;

  if (a == 0)
    return 16;

  n = 0;
  if (a <= 0x00FF)
  {
    n = n + 8;
    a = a << 8;
  }

  if (a <= 0x0FFF)
  {
    n = n + 4;
    a = a << 4;
  }

  if (a <= 0x3FFF)
  {
    n = n + 2;
    a = a << 2;
  }

  if (a <= 0x7FFF)
  {
    n = n + 1;
  }

  return n;

}

int16_t rfxp_catan2(int16_t y, int16_t x)
{
  const int16_t R_S1Q15_MINUS_ONE = -32768;
  /* Precomputed values for arctan */
  const uint16_t atanLUT[] = {
    16384,    /* atan(2^-0) / (PI / 2) * (1 << 17) */
    9672,     /* atan(2^-1) / (PI / 2) * (1 << 17) */
    5110,     /* atan(2^-2) / (PI / 2) * (1 << 17) */
    2594,     /* ... */
    1302,
    652,
    326,
    163,
    81,
    41,
    20,
    10,
    5,
    3,
    1
  };

  /* Exclude -32768 from consideration.
  -32768 is the vfx_s0015 representation of -1.
  It is excluded because its abs cannot be represented on 15bits */
  if (R_S1Q15_MINUS_ONE == x)
    x = R_S1Q15_MINUS_ONE + 1;

  if (R_S1Q15_MINUS_ONE == y)
    y = R_S1Q15_MINUS_ONE + 1;

  /* Compute |x|, |y|.
  These are the values to be further used by our CORDIC implementation. */
  uint16_t xi = abs(x);
  int16_t yi = abs(y);

  /* Normalize to as high a magnitude as possible.
  - vclz: counts leading zeroes
  - (-2): see details above about xi, yi representation
  */
  int16_t n = clz16(xi | yi) - 2;

  /* if (n > 0) => shift left by n, else shift right by (-n) */
  if (n > 0)
  {
    xi = xi << n;
    yi = yi << n;
  }
  else
  {
    xi = xi >> (-n);
    yi = yi >> (-n);
  }

  int16_t ti = 0;

  /* Main CORDIC loop */
  for (int i = 0; i < 15; i++)
  {
    uint16_t xtemp;
    int16_t ytemp;

    if (yi <= 0)
    {
      xtemp = xi - (yi >> i);
      ytemp = yi + (xi >> i);
      ti = ti - atanLUT[i];
    }
    else
    {
      xtemp = xi + (yi >> i);
      ytemp = yi - (xi >> i);
      ti = ti + atanLUT[i];
    }

    xi = xtemp;
    yi = ytemp;
  }

  /* Because we started with |x|, |y| => the computed arctan should be positive.
  Due to precision limitations, ti may be negative when its correct value is close to 0.
  Convert ti to abs(ti) before further computations */
  ti = abs(ti);

  /* ti has the same representation as xi, yi.
  * => shift ti to bring it to the representation unsigned 16bit for decimals. */
  ti >>= 2;

  /* Convert to signed. Offset to be applied later. */
  int16_t angle = (int16_t)ti;

  /* Computed angle is for |x|, |y|
  Adjust the final angle based on the real values of x, y. */
  if (y == 0)
  {
    angle = 0;
  }

  if (x < 0)
  {
    angle = 16384 - angle;
  }

  if (y < 0)
  {
    angle = -angle;
  }

  return angle;
}

void phase_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst)
{
  int width = aSrc0.cols;
  int height = aSrc0.rows;

  const int16_t* pSrc0 = (int16_t*) aSrc0.data;
  const int16_t* pSrc1 = (int16_t*) aSrc1.data;

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      int idx = x + width* y;

      int16_t in1 = pSrc0[idx];
      int16_t in2 = pSrc1[idx];

      const int16_t shift = 7;
      int16_t lOut = rfxp_catan2(in1, in2);

      // transform to all positive

      uint16_t luAbsOut = lOut > 0 ? lOut : -lOut;

      uint16_t luOut = lOut > 0 ? lOut : 32768U - luAbsOut;

      // round and shift

      luOut = (luOut + (1 << (shift - 1))) >> shift;

      // handle overflow, when rounding

      luOut = luOut >= 256 ? luOut - 256 : luOut;

      // store as uint8_t

      aDst.data[idx] = static_cast<uint8_t>(luOut);
    }
  }
}

template void tablelookup_ref<unsigned char>(const unsigned char* src0, int sstr0, unsigned char* dst, int dstr,
  int blockWidth, int blockHeight, const unsigned char* lut);

template void threshold_ref<unsigned char>(const unsigned char* src0, int sstr0, unsigned char threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold_ref<unsigned short>(const unsigned short* src0, int sstr0, unsigned short threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold_ref<unsigned int>(const unsigned int* src0, int sstr0, unsigned int threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold_ref<char>(const char* src0, int sstr0, char threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold_ref<short>(const short* src0, int sstr0, short threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold_ref<int>(const int* src0, int sstr0, int threshold,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);

template void threshold2_ref<unsigned char>(const unsigned char* src0, int sstr0, unsigned char threshold1, unsigned char threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold2_ref<unsigned short>(const unsigned short* src0, int sstr0, unsigned short threshold1, unsigned short threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold2_ref<unsigned int>(const unsigned int* src0, int sstr0, unsigned int threshold1, unsigned int threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold2_ref<char>(const char* src0, int sstr0, char threshold1, char threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold2_ref<short>(const short* src0, int sstr0, short threshold1, short threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);
template void threshold2_ref<int>(const int* src0, int sstr0, int threshold1, int threshold2,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight,
  const uint8_t val_true, const uint8_t val_false);



template void xor_ref<unsigned char>(const unsigned char* src0, int sstr0, const unsigned char* src1, int sstr1,
  unsigned char* dst, int dstr, int blockWidth, int blockHeight);
template void xor_ref<unsigned short>(const unsigned short* src0, int sstr0, const unsigned short* src1, int sstr1,
  unsigned short* dst, int dstr, int blockWidth, int blockHeight);
template void xor_ref<unsigned int>(const unsigned int* src0, int sstr0, const unsigned int* src1, int sstr1,
  unsigned int* dst, int dstr, int blockWidth, int blockHeight);
template void xor_ref<char>(const char* src0, int sstr0, const char* src1, int sstr1,
  char* dst, int dstr, int blockWidth, int blockHeight);
template void xor_ref<short>(const short* src0, int sstr0, const short* src1, int sstr1,
  short* dst, int dstr, int blockWidth, int blockHeight);
template void xor_ref<int>(const int* src0, int sstr0, const int* src1, int sstr1,
  int* dst, int dstr, int blockWidth, int blockHeight);
