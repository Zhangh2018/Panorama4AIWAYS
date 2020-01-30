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
*  \file apexcv_filter_ref.cpp
*  \brief Reference code for filter kernels
***********************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>

#include <image_filters_ref.h>

//#define vswap(a, b, c) if(c) { uint8_t tmp = a; a = b; b = tmp; }
template <class D>
inline void vswap(D& a, D& b, bool c){
  if (c){
    D t = a;
    a = b;
    b = t;
  }
}
template void vswap<int8_t>(int8_t& a, int8_t& b, bool c);
template void vswap<int16_t>(int16_t& a, int16_t& b, bool c);
template void vswap<uint8_t>(uint8_t& a, uint8_t& b, bool c);
template void vswap<uint16_t>(uint16_t& a, uint16_t& b, bool c);

using namespace std;

void dilate_3x3_08u_ref(uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh)
{
  // Structuring element: Rectangular - hardcoded
  //  1,  1,  1,
  //  1,  1,  1,
  //  1,  1,  1,

  // Neighbors:
  const uint8_t* s0 = src - sstr;
  const uint8_t* s1 = src;
  const uint8_t* s2 = src + sstr;

  // Loop
  int16_t a_max, a0, a1, a2, a3, a4, a5, a6, a7, a8;
  int16_t b_max, b6, b7, b8;

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; x += 2)
    {
      // Copy src cells
      a0 = s0[x];
      a1 = s0[x + 1];
      a2 = s1[x];
      a3 = s1[x + 1];
      a4 = s2[x];
      a5 = s2[x + 1];
      a6 = s0[x - 1];
      a7 = s1[x - 1];
      a8 = s2[x - 1];

      b6 = s0[x + 2];
      b7 = s1[x + 2];
      b8 = s2[x + 2];

      // Compare and find max
      a_max = a0;

      vswap(a1, a_max, a1 > a_max);
      vswap(a2, a_max, a2 > a_max);
      vswap(a3, a_max, a3 > a_max);
      vswap(a4, a_max, a4 > a_max);
      vswap(a5, a_max, a5 > a_max);

      //save max of common pixels
      b_max = a_max;

      vswap(a6, a_max, a6 > a_max);
      vswap(a7, a_max, a7 > a_max);
      vswap(a8, a_max, a8 > a_max);

      vswap(b6, b_max, b6 > b_max);
      vswap(b7, b_max, b7 > b_max);
      vswap(b8, b_max, b8 > b_max);



      // Assign to output
      dst[x] = (uint8_t)a_max;
      dst[x + 1] = (uint8_t)b_max;
    }

    // Proceed to next block
    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

void dilate_3x3_16s_ref(int16_t* dst, int dstr,
  const int16_t* src, int sstr,
  int bw, int bh)
{

  // Structuring element: Rectangular - hardcoded
  //  1,  1,  1,
  //  1,  1,  1,
  //  1,  1,  1,

  // Neighbors:
  const int16_t* s0 = src - sstr;
  const int16_t* s1 = src;
  const int16_t* s2 = src + sstr;

  // Loop
  int16_t a_max, a0, a1, a2, a3, a4, a5, a6, a7, a8;
  int16_t b_max, b6, b7, b8;

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; x += 2)
    {
      // Copy src cells
      a0 = s0[x];
      a1 = s0[x + 1];
      a2 = s1[x];
      a3 = s1[x + 1];
      a4 = s2[x];
      a5 = s2[x + 1];
      a6 = s0[x - 1];
      a7 = s1[x - 1];
      a8 = s2[x - 1];

      b6 = s0[x + 2];
      b7 = s1[x + 2];
      b8 = s2[x + 2];

      // Compare and find max
      a_max = a0;

      vswap(a1, a_max, a1 > a_max);
      vswap(a2, a_max, a2 > a_max);
      vswap(a3, a_max, a3 > a_max);
      vswap(a4, a_max, a4 > a_max);
      vswap(a5, a_max, a5 > a_max);

      //save max of common pixels
      b_max = a_max;

      vswap(a6, a_max, a6 > a_max);
      vswap(a7, a_max, a7 > a_max);
      vswap(a8, a_max, a8 > a_max);

      vswap(b6, b_max, b6 > b_max);
      vswap(b7, b_max, b7 > b_max);
      vswap(b8, b_max, b8 > b_max);

      // Assign to output
      dst[x] = (int16_t)a_max;
      dst[x + 1] = (int16_t)b_max;
    }

    // Proceed to next block
    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

void erode_3x3_08u_ref(uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh)
{
  // Structuring element: Rectangular - hardcoded
  //  1,  1,  1,
  //  1,  1,  1,
  //  1,  1,  1,

  // Neighbors:
  const uint8_t* s0 = src - sstr;
  const uint8_t* s1 = src;
  const uint8_t* s2 = src + sstr;

  // Loop
  int16_t a_min, a0, a1, a2, a3, a4, a5, a6, a7, a8;
  int16_t b_min, b6, b7, b8;

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; x += 2)
    {
      // Copy src cells
      a0 = s0[x];
      a1 = s0[x + 1];
      a2 = s1[x];
      a3 = s1[x + 1];
      a4 = s2[x];
      a5 = s2[x + 1];
      a6 = s0[x - 1];
      a7 = s1[x - 1];
      a8 = s2[x - 1];

      b6 = s0[x + 2];
      b7 = s1[x + 2];
      b8 = s2[x + 2];

      // Compare and find min
      a_min = a0;

      vswap(a1, a_min, a1 < a_min);
      vswap(a2, a_min, a2 < a_min);
      vswap(a3, a_min, a3 < a_min);
      vswap(a4, a_min, a4 < a_min);
      vswap(a5, a_min, a5 < a_min);

      //save min of common pixels
      b_min = a_min;

      vswap(a6, a_min, a6 < a_min);
      vswap(a7, a_min, a7 < a_min);
      vswap(a8, a_min, a8 < a_min);

      vswap(b6, b_min, b6 < b_min);
      vswap(b7, b_min, b7 < b_min);
      vswap(b8, b_min, b8 < b_min);

      // Assign to output
      dst[x] = (uint8_t)a_min;
      dst[x + 1] = (uint8_t)b_min;
    }

    // Proceed to next block
    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

void
bilateral_5x5_init(uint32_t *r_lut, uint32_t *d_lut, int sigmaR, int sigmaD)
{
  float inv_sigmaR = -0.5f / (sigmaR * sigmaR);
  float inv_sigmaD = -0.5f / (sigmaD * sigmaD);
  float y;

  for (int i = 0; i < 256; ++i)
  {
    y = expf((i * i) * inv_sigmaR);
    r_lut[i] = (uint32_t)(y * 255 + 0.5f);
  }

  for (int j = 0; j < 9; ++j)
  {
    y = expf(j * inv_sigmaD);
    d_lut[j] = (uint32_t)(y * 255 + 0.5f);
  }
}

void
bilateral_5x5_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int sigmaR, int sigmaD)
{
  uint32_t weight;          //FP  0_16
  uint32_t sum_wi;          //FP 13_16
  uint32_t sum_weight;      //FP  5_16
  uint32_t r_lut[256];
  uint32_t d_lut[9];

  bilateral_5x5_init(r_lut, d_lut, sigmaR, sigmaD);


  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint32_t center_i = src[y * sstr + x];
      sum_weight = 0;
      sum_wi = 0;

      for (int fy = -2; fy < 3; ++fy)
      {
        for (int fx = -2; fx < 3; ++fx)
        {
          uint32_t filter_i = src[(y + fy) * sstr + (x + fx)];
          int32_t vidx = max(center_i, filter_i) - min(center_i, filter_i);

          weight = r_lut[vidx] * d_lut[fx*fx + fy*fy];
          sum_weight += weight;
          sum_wi += filter_i * weight;
        }
      }
      //TODO: temporary divide
      //dst[y*dstr+x] = (uint8_t)(((sum_wi / (sum_weight >> 1)) + 1) >> 1);
      uint32_t sTemp32 = (sum_wi + ((sum_weight + 1) >> 1));
      uint16_t sTemp = sTemp32 / sum_weight;
      dst[y*dstr + x] = (uint8_t)(sTemp);
      //dst[y*dstr + x] = (vec08u)(vec16u)((sum_wi + ((sum_weight + 1) >> 1)) / sum_weight);
      //dst[y*dstr + x] = (uint8_t)(uint16_t)((sum_wi + ((sum_weight + 1) >> 1)) / sum_weight);
    }
  }
}

void
box_3x3_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint32_t sum = 0;

      for (int fy = -1; fy < 2; ++fy)
      {
        for (int fx = -1; fx < 2; ++fx)
        {
          sum += src[(y + fy) * sstr + (x + fx)];
        }
      }
      dst[y*dstr + x] = (uint8_t)round((double)sum / 9.0);
    }
  }
}

void
box_3x3_nr_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint32_t sum = 0;

      for (int fy = -1; fy < 2; ++fy)
      {
        for (int fx = -1; fx < 2; ++fx)
        {
          sum += src[(y + fy) * sstr + (x + fx)];
        }
      }
      dst[y*dstr + x] = (uint8_t)(sum / 9);
    }
  }
}

void
box_3x3_ref(int16_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint32_t sum = 0;

      for (int fy = -1; fy < 2; ++fy)
      {
        for (int fx = -1; fx < 2; ++fx)
        {
          sum += (short)(src[(y + fy) * sstr + (x + fx)]);
        }
      }
      dst[y*dstr + x] = (int16_t)sum;
    }
  }
}

void
box_5x5_16s_ref(int16_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int32_t sum = 0;

      for (int fy = -2; fy < 3; ++fy)
      {
        for (int fx = -2; fx < 3; ++fx)
        {
          sum += src[(y + fy) * sstr + (x + fx)];
        }
      }
      dst[y*dstr + x] = (int16_t)sum;
    }
  }
}

/*
aSrc: already have spatial dependencies source, in 4 directions
sDst: non-padded destination
*/
void box_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst, int aWindowSize)
{
  int T_PAD = aWindowSize/2;
  int B_PAD = aWindowSize/2;
  int w = aDst.cols;
  int h = aDst.rows;
  int tw = aSrc.cols;
  int th = aSrc.rows - T_PAD - B_PAD;
  
  /* 
  tempMat: temporary matrix to store step 1 result
  */
  vsdk::SMat tempMat(th, tw, aSrc.type());
  
  if (tempMat.empty())
  {
    printf("%s: Cannot allocate tempMat\n", __FUNCTION__);
    return;
  }

  for ( int x = 0; x < tw; x++)
  {
    for ( int y = 0; y < th; y++)
    {
      uint16_t sum = 0;
      for (int i = 0; i < aWindowSize; i++)
      {
        sum += aSrc.at<uint8_t>(y + i, x);
      }
      tempMat.at<uint8_t>(y, x) = (uint8_t)((sum + aWindowSize/2)/aWindowSize);
    }
  }
  
  for ( int y = 0; y < h; y++)
  {
    for ( int x = 0; x < w; x++)
    {
      uint16_t sum = 0;
      for (int i = 0; i < aWindowSize; i++)
      {
        sum += tempMat.at<uint8_t>(y, x + i);
      }
      aDst.at<uint8_t>(y, x) = (uint8_t)((sum + aWindowSize/2)/aWindowSize);
    }
  }
}

void census_3x3_ref(uint8_t* dst, int dstr, const uint8_t* src, int sstr, int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      /*
      |a|b|c|
      |d|e|f|
      |g|h|i|
      */
      uint32_t out = 0;
      uint32_t a = src[(y - 1) * sstr + x - 1];
      uint32_t b = src[(y - 1) * sstr + x];
      uint32_t c = src[(y - 1) * sstr + x + 1];
      uint32_t d = src[(y)* sstr + x - 1];
      uint32_t e = src[(y)* sstr + x];
      uint32_t f = src[(y)* sstr + x + 1];
      uint32_t g = src[(y + 1) * sstr + x - 1];
      uint32_t h = src[(y + 1) * sstr + x];
      uint32_t i = src[(y + 1) * sstr + x + 1];
      out = (((uint32_t)(a > e)) << 7) +
        (((uint32_t)(b > e)) << 6) +
        (((uint32_t)(c > e)) << 5) +
        (((uint32_t)(d > e)) << 4) +
        (((uint32_t)(f > e)) << 3) +
        (((uint32_t)(g > e)) << 2) +
        ((((uint32_t)h > e)) << 1) +
        (uint32_t)(i > e);
      dst[y*dstr + x] = (uint8_t)out;
    }
  }
}

void
derivative_x_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int k0, int k1, int k2
)
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      uint8_t lColNorth = src[-sstr + j - 1];
      uint8_t lColCenter = src[j - 1];
      uint8_t lColSouth = src[+sstr + j - 1];

      uint8_t rColNorth = src[-sstr + j + 1];
      uint8_t rColCenter = src[j + 1];
      uint8_t rColSouth = src[+sstr + j + 1];

      int16_t negSum = k0 * lColNorth + k1 * lColCenter + k2 * lColSouth;
      int16_t posSum = k0 * rColNorth + k1 * rColCenter + k2 * rColSouth;
      dst[j] = posSum - negSum;
      //dst[j] = k0 * (rColNorth - lColNorth) + k1*(rColCenter - lColCenter) + k2*(rColSouth - lColSouth);
    }
    dst += dstr;
    src += sstr;
  }
}

void
derivative_y_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
int k0, int k1, int k2
)
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      dst[j] = -k0*src[-sstr + j - 1] - k1*src[-sstr + j] - k2*src[-sstr + j + 1]
        + k0*src[+sstr + j - 1] + k1*src[+sstr + j] + k2*src[+sstr + j + 1];
    }
    dst += dstr;
    src += sstr;
  }
}

#define sort2El(a, b) vswap(a, b, a > b);
#define min3(a, b, c) sort2El(a, b); sort2El(a, c);
#define max3(a, b, c) sort2El(a, c); sort2El(b, c);
#define min4(a, b, c, d) sort2El(a, b); sort2El(c, d); sort2El(a, c);
#define max4(a, b, c, d) sort2El(a, b); sort2El(c, d); sort2El(b, d);
#define min5(a, b, c, d, e) sort2El(a, b); sort2El(c, d); min3(a, c, e);
#define max5(a, b, c, d, e) sort2El(a, b); sort2El(c, d); max3(b, d, e);
#define min6(a, b, c, d, e, f) sort2El(a, b); sort2El(c, d); sort2El(e, f); min3(a, c, e);
#define max6(a, b, c, d, e, f) sort2El(a, b); sort2El(c, d); sort2El(e, f); max3(b, d, f);

#define min7(a, b, c, d, e, f, g) sort2El(a, b); sort2El(c, d); sort2El(e, f); min4(a, c, e, g);
#define max7(a, b, c, d, e, f, g) sort2El(a, b); sort2El(c, d); sort2El(e, f); max4(b, d, f, g);

#define min8(a, b, c, d, e, f, g, h) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); min4(a, c, e, g);
#define max8(a, b, c, d, e, f, g, h) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); max4(b, d, f, h);

#define min9(a, b, c, d, e, f, g, h, i) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); min5(a, c, e, g, i);
#define max9(a, b, c, d, e, f, g, h, i) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); max5(b, d, f, h, i);

#define min10(a, b, c, d, e, f, g, h, i, j) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); min5(a, c, e, g, i);
#define max10(a, b, c, d, e, f, g, h, i, j) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); max5(b, d, f, h, j);

#define min11(a, b, c, d, e, f, g, h, i, j, k) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); min6(a, c, e, g, i, k);
#define max11(a, b, c, d, e, f, g, h, i, j, k) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); max6(b, d, f, h, j, k);

#define min12(a, b, c, d, e, f, g, h, i, j, k, l) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); min6(a, c, e, g, i, k);
#define max12(a, b, c, d, e, f, g, h, i, j, k, l) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); max6(b, d, f, h, j, l);

#define min13(a, b, c, d, e, f, g, h, i, j, k, l, m) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); min7(a, c, e, g, i, k, m);
#define max13(a, b, c, d, e, f, g, h, i, j, k, l, m) sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); max7(b, d, f, h, j, l, m);


#define minmax3(a, b, c) min3(a, b, c); sort2El(b, c);
#define minmax4(a, b, c, d) sort2El(a, b); sort2El(c, d); sort2El(a, c); sort2El(b, d);
#define minmax5(a, b, c, d, e) sort2El(a, b); sort2El(c, d); min3(a, c, e); max3(b, d, e);

#define minmax6(a, b, c, d, e, f) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); min3(a, c, e); max3(b, d, f);

#define minmax7(a, b, c, d, e, f, g) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); min4(a, c, e, g); max4(b, d, f, g);

#define minmax8(a, b, c, d, e, f, g, h) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); min4(a, c, e, g); max4(b, d, f, h);

#define minmax9(a, b, c, d, e, f, g, h, i) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); min5(a, c, e, g, i); max5(b, d, f, h, i);

#define minmax10(a, b, c, d, e, f, g, h, i, j) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); min5(a, c, e, g, i); \
max5(b, d, f, h, j);

#define minmax11(a, b, c, d, e, f, g, h, i, j, k) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); min6(a, c, e, g, i, k); \
max6(b, d, f, h, j, k);

#define minmax12(a, b, c, d, e, f, g, h, i, j, k, l) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); \
min6(a, c, e, g, i, k); max6(b, d, f, h, j, l);

#define minmax13(a, b, c, d, e, f, g, h, i, j, k, l, m) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); min3(a, c, e); \
min3(g, i, k); min3(a, g, m); max3(b, d, f); max3(h, j, l); max3(f, l, m);

#define minmax14(a, b, c, d, e, f, g, h, i, j, k, l, m, n) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); sort2El(m, n); \
min3(a, c, e); min3(g, i, k); min3(a, g, m); max3(b, d, f); max3(h, j, l); max3(f, l, n);


#define minmax15(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o)\
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); sort2El(m, n); \
min3(a,c,e); min3(g,i,k); sort2El(m,o); min3(a,g,m); max3(b,d,f); max3(h,j,l);sort2El(n,o);max3(f,l,o);


#define minmax16(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p) \
sort2El(a, b); sort2El(c, d); sort2El(e, f); sort2El(g, h); sort2El(i, j); sort2El(k, l); sort2El(m, n); sort2El(o, p); \
minmax8(a, c, e, g, i, k, m, o); minmax8(b, d, f, h, j, l, n, p); sort2El(a, b); sort2El(o, p);

#define minmax17(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14); sort2El(a15, a16);\
min8(a1,a3,a5,a7,a9,a11,a13,a15); sort2El(a1,a17); max8(a2,a4,a6,a8,a10,a12,a14,a16); sort2El(a16, a17);

#define minmax18(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14); sort2El(a15, a16); sort2El(a17, a18);\
min9(a1,a3,a5,a7,a9,a11,a13,a15,a17); max9(a2, a4, a6, a8, a10, a12, a14, a16, a18);

#define minmax19(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14); sort2El(a15, a16); sort2El(a17, a18);\
min9(a1,a3,a5,a7,a9,a11,a13,a15,a17); sort2El(a1,a19); max9(a2, a4, a6, a8, a10, a12, a14, a16, a18); sort2El(a18,a19);

#define minmax20(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14);\
sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20);\
min10(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19); max10(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20);

#define minmax21(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14);\
sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20);\
min10(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19);sort2El(a1,a21); max10(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20); sort2El(a20,a21);

#define minmax22(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14);\
sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20); sort2El(a21, a22);\
min11(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19,a21); max11(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20, a22);

#define minmax23(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14); sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20); sort2El(a21, a22);\
min11(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19,a21); sort2El(a1,a23); max11(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20, a22); sort2El(a22, a23);

#define minmax24(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14); sort2El(a15, a16); sort2El(a17, a18);\
sort2El(a19, a20); sort2El(a21, a22); sort2El(a23, a24);\
min12(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19,a21,a23); max12(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20, a22, a24);

#define minmax25(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14);\
sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20); sort2El(a21, a22); sort2El(a23, a24);\
min12(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19,a21,a23); sort2El(a1, a25) ; max12(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20, a22, a24); sort2El(a24, a25);

#define minmax26(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26)\
sort2El(a1, a2); sort2El(a3, a4); sort2El(a5, a6); sort2El(a7, a8); sort2El(a9, a10); sort2El(a11, a12); sort2El(a13, a14);\
sort2El(a15, a16); sort2El(a17, a18); sort2El(a19, a20); sort2El(a21, a22); sort2El(a23, a24); sort2El(a25, a26);\
min13(a1,a3,a5,a7,a9,a11,a13,a15,a17,a19,a21,a23,a25); max13(a2, a4, a6, a8, a10, a12, a14, a16, a18, a20, a22, a24, a26);

void
median_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  const uint8_t* s0 = src - sstr;
  const uint8_t* s1 = src;
  const uint8_t* s2 = src + sstr;

  int16_t a0, a1, a2, a3, a4, a5;
  int16_t b1, b2, b3, b4, b5;

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; x += 2)
    {
      a0 = s0[x];
      a1 = s0[x + 1];
      a2 = s1[x];
      a3 = s1[x + 1];
      a4 = s2[x];
      a5 = s2[x + 1];

      minmax6(a0, a1, a2, a3, a4, a5);

      b1 = a1; b2 = a2; b3 = a3; b4 = a4;

      a5 = s0[x - 1];
      b5 = s0[x + 2];
      minmax5(a1, a2, a3, a4, a5);
      minmax5(b1, b2, b3, b4, b5);
      a5 = s1[x - 1];
      b5 = s1[x + 2];
      minmax4(a2, a3, a4, a5);
      minmax4(b2, b3, b4, b5);
      a5 = s2[x - 1];
      b5 = s2[x + 2];
      minmax3(a3, a4, a5);
      minmax3(b3, b4, b5);

      dst[x] = (uint8_t)a4;
      dst[x + 1] = (uint8_t)b4;
    }

    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

void
median_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  const uint8_t* s0 = src - sstr - sstr;
  const uint8_t* s1 = src - sstr;
  const uint8_t* s2 = src;
  const uint8_t* s3 = src + sstr;
  const uint8_t* s4 = src + sstr + sstr;

  int16_t a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13;
  int16_t b7, b8, b9, b10, b11, b12, b13;

  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; x += 2)
    {
      a0 = s0[x - 1];
      a1 = s0[x];
      a2 = s0[x + 1];
      a3 = s0[x + 2];
      a4 = s1[x - 1];
      a5 = s1[x];
      a6 = s1[x + 1];
      a7 = s1[x + 2];
      a8 = s2[x - 1];
      a9 = s2[x];
      a10 = s2[x + 1];
      a11 = s2[x + 2];
      a12 = s3[x - 1];
      a13 = s3[x];

      minmax14(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s3[x + 1];
      minmax13(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s3[x + 2];
      minmax12(a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s4[x - 1];
      minmax11(a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s4[x];
      minmax10(a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s4[x + 1];
      minmax9(a5, a6, a7, a8, a9, a10, a11, a12, a13);
      a13 = s4[x + 2];
      minmax8(a6, a7, a8, a9, a10, a11, a12, a13);

      b7 = a7; b8 = a8; b9 = a9; b10 = a10; b11 = a11; b12 = a12;

      a13 = s0[x - 2];
      b13 = s0[x + 3];
      minmax7(a7, a8, a9, a10, a11, a12, a13);
      minmax7(b7, b8, b9, b10, b11, b12, b13);
      a13 = s1[x - 2];
      b13 = s1[x + 3];
      minmax6(a8, a9, a10, a11, a12, a13);
      minmax6(b8, b9, b10, b11, b12, b13);
      a13 = s2[x - 2];
      b13 = s2[x + 3];
      minmax5(a9, a10, a11, a12, a13);
      minmax5(b9, b10, b11, b12, b13);
      a13 = s3[x - 2];
      b13 = s3[x + 3];
      minmax4(a10, a11, a12, a13);
      minmax4(b10, b11, b12, b13)
        a13 = s4[x - 2];
      b13 = s4[x + 3];
      minmax3(a11, a12, a13);
      minmax3(b11, b12, b13);

      dst[x] = (uint8_t)a12;
      dst[x + 1] = (uint8_t)b12;
    }

    s0 = s1;
    s1 = s2;
    s2 = s3;
    s3 = s4;
    s4 += sstr;
    dst += dstr;
  }
}

void median_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst, int aWindowSize)
{
  int L_PAD = aWindowSize/2;
  int R_PAD = aWindowSize/2;
  int T_PAD = aWindowSize/2;
  int B_PAD = aWindowSize/2;

  int width = aSrc.cols - T_PAD - B_PAD;
  int height = aSrc.rows - L_PAD - R_PAD;

  uint16_t newElement;

  switch (aWindowSize)
  {
    case 3:
    {
      for(int y = 0; y < height ; ++y)
      {
        for(int x = 0; x < width; ++x)
        {
          uint16_t a0 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 1 + L_PAD);
          uint16_t a1 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + L_PAD);
          uint16_t a2 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 1 + L_PAD);

          uint16_t a3 = aSrc.at<uint8_t>(y + T_PAD, x - 1 + L_PAD);
          uint16_t a4 = aSrc.at<uint8_t>(y + T_PAD, x + L_PAD);
          uint16_t a5 = aSrc.at<uint8_t>(y + T_PAD, x + 1 + L_PAD);

          minmax6(a0, a1, a2, a3, a4, a5);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x -1 + L_PAD);
          minmax5(a1, a2, a3, a4, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + L_PAD);
          minmax4(a2, a3, a4, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 1 + L_PAD);
          minmax3(a3, a4, newElement);

          aDst.at<uint8_t>(y, x) = (uint8_t)a4;
        }
      }
      break;
    }

    case 5:
    {
      for(int y = 0; y < height ; ++y)
      {
        for(int x = 0; x < width; ++x)
        {
          uint16_t a0 = aSrc.at<uint8_t>(y - 2 + T_PAD, x - 2 + L_PAD);
          uint16_t a1 = aSrc.at<uint8_t>(y - 2 + T_PAD, x - 1 + L_PAD);
          uint16_t a2 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + L_PAD);
          uint16_t a3 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + 1 + L_PAD);
          uint16_t a4 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + 2 + L_PAD);

          uint16_t a5 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 2 + L_PAD);
          uint16_t a6 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 1 + L_PAD);
          uint16_t a7 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + L_PAD);
          uint16_t a8 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 1 + L_PAD);
          uint16_t a9 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 2 + L_PAD);

          uint16_t a10 = aSrc.at<uint8_t>(y + T_PAD, x - 2 + L_PAD);
          uint16_t a11 = aSrc.at<uint8_t>(y + T_PAD, x - 1 + L_PAD);
          uint16_t a12 = aSrc.at<uint8_t>(y + T_PAD, x + L_PAD);
          uint16_t a13 = aSrc.at<uint8_t>(y + T_PAD, x + 1 + L_PAD);

          minmax14(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
          newElement = aSrc.at<uint8_t>(y + T_PAD, x + 2 + L_PAD);
          minmax13(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x - 2 + L_PAD);
          minmax12(a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x - 1 + L_PAD);
          minmax11(a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + L_PAD);
          minmax10(a4, a5, a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 1 + L_PAD);
          minmax9(a5, a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 2 + L_PAD);
          minmax8(a6, a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x - 2 + L_PAD);
          minmax7(a7, a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x - 1 + L_PAD);
          minmax6(a8, a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + L_PAD);
          minmax5(a9, a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + 1 + L_PAD);
          minmax4(a10, a11, a12, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + 2 + L_PAD);
          minmax3(a11, a12, newElement);

          aDst.at<uint8_t>(y, x) = (uint8_t)a12;
        }
      }
      break;
    }

    case 7:
    {
      for(int y = 0; y < height ; ++y)
      {
        for(int x = 0; x < width; ++x)
        {
          uint16_t a0 = aSrc.at<uint8_t>(y - 3 + T_PAD, x - 3 + L_PAD);
          uint16_t a1 = aSrc.at<uint8_t>(y - 3 + T_PAD, x - 2 + L_PAD);
          uint16_t a2 = aSrc.at<uint8_t>(y - 3 + T_PAD, x - 1 + L_PAD);
          uint16_t a3 = aSrc.at<uint8_t>(y - 3 + T_PAD, x + L_PAD);
          uint16_t a4 = aSrc.at<uint8_t>(y - 3 + T_PAD, x + 1 + L_PAD);
          uint16_t a5 = aSrc.at<uint8_t>(y - 3 + T_PAD, x + 2 + L_PAD);
          uint16_t a6 = aSrc.at<uint8_t>(y - 3 + T_PAD, x + 3 + L_PAD);

          uint16_t a7 = aSrc.at<uint8_t>(y - 2 + T_PAD, x - 3 + L_PAD);
          uint16_t a8 = aSrc.at<uint8_t>(y - 2 + T_PAD, x - 2 + L_PAD);
          uint16_t a9 = aSrc.at<uint8_t>(y - 2 + T_PAD, x - 1 + L_PAD);
          uint16_t a10 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + L_PAD);
          uint16_t a11 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + 1 + L_PAD);
          uint16_t a12 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + 2 + L_PAD);
          uint16_t a13 = aSrc.at<uint8_t>(y - 2 + T_PAD, x + 3 + L_PAD);

          uint16_t a14 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 3 + L_PAD);
          uint16_t a15 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 2 + L_PAD);
          uint16_t a16 = aSrc.at<uint8_t>(y - 1 + T_PAD, x - 1 + L_PAD);
          uint16_t a17 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + L_PAD);
          uint16_t a18 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 1 + L_PAD);
          uint16_t a19 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 2 + L_PAD);
          uint16_t a20 = aSrc.at<uint8_t>(y - 1 + T_PAD, x + 3 + L_PAD);

          uint16_t a21 = aSrc.at<uint8_t>(y + T_PAD, x - 3 + L_PAD);
          uint16_t a22 = aSrc.at<uint8_t>(y + T_PAD, x - 2 + L_PAD);
          uint16_t a23 = aSrc.at<uint8_t>(y + T_PAD, x - 1 + L_PAD);
          uint16_t a24 = aSrc.at<uint8_t>(y + T_PAD, x + L_PAD);
          uint16_t a25 = aSrc.at<uint8_t>(y + T_PAD, x + 1 + L_PAD);

          minmax26(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25);
          newElement = aSrc.at<uint8_t>(y + T_PAD, x + 2 + L_PAD);
          minmax25(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + T_PAD, x + 3 + L_PAD);
          minmax24(a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x - 3 + L_PAD);
          minmax23(a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x - 2 + L_PAD);
          minmax22(a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x - 1 + L_PAD);
          minmax21(a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + L_PAD);
          minmax20(a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 1 + L_PAD);
          minmax19(a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 2 + L_PAD);
          minmax18(a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 1 + T_PAD, x + 3 + L_PAD);
          minmax17(a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x - 3 + L_PAD);
          minmax16(a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x - 2 + L_PAD);
          minmax15(a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x - 1 + L_PAD);
          minmax14(a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + L_PAD);
          minmax13(a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + 1 + L_PAD);
          minmax12(a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + 2 + L_PAD);
          minmax11(a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 2 + T_PAD, x + 3 + L_PAD);
          minmax10(a16, a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x - 3 + L_PAD);
          minmax9(a17, a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x - 2 + L_PAD);
          minmax8(a18, a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x - 1 + L_PAD);
          minmax7(a19, a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x + L_PAD);
          minmax6(a20, a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x + 1 + L_PAD);
          minmax5(a21, a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x + 2 + L_PAD);
          minmax4(a22, a23, a24, newElement);
          newElement = aSrc.at<uint8_t>(y + 3 + T_PAD, x + 3 + L_PAD);
          minmax3(a23, a24, newElement);

          aDst.at<uint8_t>(y, x) = (uint8_t)a24;
        }
      }
      break;
    }
    default:
    break;
  }
}

// Local helper functions
static inline uint16_t apu_filter_gaussian_3x3(const uint8_t* src, int sstr);
static inline uint16_t apu_filter_gaussian_5x5(const uint8_t* src, int sstr);

void
gaussian_3x3_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint16_t tx = apu_filter_gaussian_3x3(&src[x], sstr);

      dst[x] = (uint8_t)tx;
    }

    src += sstr;
    dst += dstr;
  }
}

void
gaussian_5x5_08u_ref(uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      uint16_t tx = apu_filter_gaussian_5x5(&src[x], sstr);

      dst[x] = (uint8_t)tx;
    }

    src += sstr;
    dst += dstr;
  }
}


void
gaussian_7x7_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst)
{
  int L_PAD = 3;
  int R_PAD = 3;
  int T_PAD = 3;
  int B_PAD = 3;

  vsdk::SUMat temp(aSrc.rows, aSrc.cols - L_PAD - R_PAD, VSDK_CV_8UC1);
  vsdk::SMat tempMat = temp.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

  for (int y = 0; y < temp.rows; ++y)
  {
    for (int x = 0; x < temp.cols; ++x)
    {
      uint16_t t1 = aSrc.at<uint8_t>(y, x - 3 + L_PAD) + 5 * (aSrc.at<uint8_t>(y, x - 2 + L_PAD) + aSrc.at<uint8_t>(y, x + 2 + L_PAD)) 
      + 15 * (aSrc.at<uint8_t>(y, x - 1 + L_PAD) + aSrc.at<uint8_t>(y, x + 1 + L_PAD)) + 22 * aSrc.at<uint8_t>(y, x + L_PAD) + aSrc.at<uint8_t>(y, x + 3 + L_PAD);

      uint16_t tx = t1 >> 6;

      tempMat.at<uint8_t>(y , x) = (uint8_t)tx;
    }
  }

  for (int y = 0; y < aDst.rows; ++y)
  {
    for (int x = 0; x < aDst.cols; ++x)
    {
      uint16_t t1 = tempMat.at<uint8_t>(y - 3 + T_PAD, x) + 5 * (tempMat.at<uint8_t>(y - 2 + T_PAD, x) + tempMat.at<uint8_t>(y + 2 + T_PAD, x)) 
      + 15 * (tempMat.at<uint8_t>(y - 1 + T_PAD, x) + tempMat.at<uint8_t>(y + 1 + T_PAD, x)) + 22 * tempMat.at<uint8_t>(y + T_PAD, x) + tempMat.at<uint8_t>(y + 3 + T_PAD, x);

      uint16_t tx = t1 >> 6;

      aDst.at<uint8_t>(y, x) = (uint8_t)tx;
    }
  }
}

void
gaussian_9x9_08u_ref(vsdk::SMat& aSrc, vsdk::SMat& aDst)
{
  int L_PAD = 4;
  int R_PAD = 4;
  int T_PAD = 4;
  int B_PAD = 4;

  vsdk::SUMat temp(aSrc.rows, aSrc.cols - L_PAD - R_PAD, VSDK_CV_8UC1);
  vsdk::SMat tempMat = temp.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

  for (int y = 0; y < temp.rows; ++y)
  {
    for (int x = 0; x < temp.cols; ++x)
    {
      uint16_t t1 = (uint16_t)aSrc.at<uint8_t>(y, x - 4 + L_PAD) + 7 * ((uint16_t)aSrc.at<uint8_t>(y, x - 3 + L_PAD) + (uint16_t)aSrc.at<uint8_t>(y, x + 3 + L_PAD)) 
      + 26 * ((uint16_t)aSrc.at<uint8_t>(y, x - 2 + L_PAD) + (uint16_t)aSrc.at<uint8_t>(y, x + 2 + L_PAD)) + 57 * ((uint16_t)aSrc.at<uint8_t>(y, x - 1 + L_PAD) 
      + (uint16_t)aSrc.at<uint8_t>(y, x + 1 + L_PAD)) + 74 * (uint16_t)aSrc.at<uint8_t>(y, x + L_PAD) + (uint16_t)aSrc.at<uint8_t>(y, x + 4 + L_PAD);

      uint16_t tx = t1 >> 8;

      tempMat.at<uint8_t>(y , x) = (uint8_t)tx;
    }
  }

  for (int y = 0; y < aDst.rows; ++y)
  {
    for (int x = 0; x < aDst.cols; ++x)
    {
      uint16_t t1 = (uint16_t)tempMat.at<uint8_t>(y - 4 + T_PAD, x) + 7 * ((uint16_t)tempMat.at<uint8_t>(y - 3 + T_PAD, x) + (uint16_t)tempMat.at<uint8_t>(y + 3 + T_PAD, x)) 
      + 26 * ((uint16_t)tempMat.at<uint8_t>(y - 2 + T_PAD, x) + (uint16_t)tempMat.at<uint8_t>(y + 2 + T_PAD, x)) + 57 * ((uint16_t)tempMat.at<uint8_t>(y - 1 + T_PAD, x) 
      + (uint16_t)tempMat.at<uint8_t>(y + 1 + T_PAD, x)) + 74 * (uint16_t)tempMat.at<uint8_t>(y + T_PAD, x) + (uint16_t)tempMat.at<uint8_t>(y + 4 + T_PAD, x);

      uint16_t tx = t1 >> 8;

      aDst.at<uint8_t>(y, x) = (uint8_t)tx;
    }
  }
}

// LOCAL HELPER FUNCTION
//  1,  2,  1,
//  2,  4,  2,  * 1/16
//  1,  2,  1,
static
inline
uint16_t
apu_filter_gaussian_3x3(const uint8_t* src, int sstr)
{
  uint16_t t1 = +src[-sstr - 1] + 2 * src[-sstr] + src[-sstr + 1]
    + 2 * src[-1] + 4 * src[0] + 2 * src[+1]
    + src[+sstr - 1] + 2 * src[+sstr] + src[+sstr + 1];

  uint16_t vd = t1 >> 4;

  return vd;
}

// LOCAL HELPER FUNCTION
//  1,   4,   6,  4,  1,
//  4,  16,  24, 16,  4, 
//  6,  24,  36, 24,  6, * 1/256
//  4,  16,  24, 16,  4,
//  1,   4,   6,  4,  1,
static
inline
uint16_t
apu_filter_gaussian_5x5(const uint8_t* src, int sstr)
{
  uint16_t t1 = +src[-2 * sstr - 2] + 4 * src[-2 * sstr - 1] + 6 * src[-2 * sstr] + 4 * src[-2 * sstr + 1] + src[-2 * sstr + 2]
    + 4 * src[-sstr - 2] + 16 * src[-sstr - 1] + 24 * src[-sstr] + 16 * src[-sstr + 1] + 4 * src[-sstr + 2]
    + 6 * src[-2] + 24 * src[-1] + 36 * src[0] + 24 * src[+1] + 6 * src[+2]
    + 4 * src[+sstr - 2] + 16 * src[+sstr - 1] + 24 * src[+sstr] + 16 * src[+sstr + 1] + 4 * src[+sstr + 2]
    + src[+2 * sstr - 2] + 4 * src[+2 * sstr - 1] + 6 * src[+2 * sstr] + 4 * src[+2 * sstr + 1] + src[+2 * sstr + 2];

  uint16_t vd = t1 >> 8;

  return vd;
}

void
separable_3x3_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const signed char* fRow,
const signed char* fCol
)
{
  int16_t c1, c2, c3;
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      c1 = fCol[0] * src[-sstr + j - 1] + fCol[1] * src[j - 1] + fCol[2] * src[sstr + j - 1];
      c2 = fCol[0] * src[-sstr + j] + fCol[1] * src[j] + fCol[2] * src[sstr + j];
      c3 = fCol[0] * src[-sstr + j + 1] + fCol[1] * src[j + 1] + fCol[2] * src[sstr + j + 1];
      dst[j] = fRow[0] * c1 + fRow[1] * c2 + fRow[2] * c3;
    }
    src += sstr;
    dst += dstr;
  }
}

void
separable_5x5_16s_ref(int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const signed char* fRow,
const signed char* fCol
)
{
  int16_t c1, c2, c3, c4, c5;
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      c1 = fCol[0] * src[-2 * sstr + j - 2] + fCol[1] * src[-sstr + j - 2] + fCol[2] * src[j - 2] + fCol[3] * src[sstr + j - 2] + fCol[4] * src[2 * sstr + j - 2];
      c2 = fCol[0] * src[-2 * sstr + j - 1] + fCol[1] * src[-sstr + j - 1] + fCol[2] * src[j - 1] + fCol[3] * src[sstr + j - 1] + fCol[4] * src[2 * sstr + j - 1];
      c3 = fCol[0] * src[-2 * sstr + j] + fCol[1] * src[-sstr + j] + fCol[2] * src[j] + fCol[3] * src[sstr + j] + fCol[4] * src[2 * sstr + j];
      c4 = fCol[0] * src[-2 * sstr + j + 1] + fCol[1] * src[-sstr + j + 1] + fCol[2] * src[j + 1] + fCol[3] * src[sstr + j + 1] + fCol[4] * src[2 * sstr + j + 1];
      c5 = fCol[0] * src[-2 * sstr + j + 2] + fCol[1] * src[-sstr + j + 2] + fCol[2] * src[j + 2] + fCol[3] * src[sstr + j + 2] + fCol[4] * src[2 * sstr + j + 2];
      dst[j] = fRow[0] * c1 + fRow[1] * c2 + fRow[2] * c3 + fRow[3] * c4 + fRow[4] * c5;
    }
    src += sstr;
    dst += dstr;
  }
}

// Local helper functions
static inline int16_t apu_filter_scharr_3x3_x(const uint8_t* src, int sstr);
static inline int16_t apu_filter_scharr_3x3_y(const uint8_t* src, int sstr);
static inline int16_t apu_filter_scharr_5x5_x(const uint8_t* src, int sstr);
static inline int16_t apu_filter_scharr_5x5_y(const uint8_t* src, int sstr);

static inline int16_t apu_filter_sobel_3x3_x(const uint8_t* src, int sstr);
static inline int16_t apu_filter_sobel_3x3_y(const uint8_t* src, int sstr);
static inline int16_t apu_filter_sobel_5x5_x(const uint8_t* src, int sstr);
static inline int16_t apu_filter_sobel_5x5_y(const uint8_t* src, int sstr);


void
scharr_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst)
{
  if((aWindowSize != 3) && (aWindowSize != 5))
  {
    printf("Argument: aWindowSize is incorrect. Window size must to is 3 or 5\n");
    return;
  }
  int bw = aDst.cols;
  int bh = aDst.rows;
  int sstr = bw + 2*(aWindowSize/2);
  const uint8_t* src = (uint8_t*) aSrc.data  + sstr*(aWindowSize/2) + (aWindowSize/2);
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx, ty;
      uint16_t t;
      if(aWindowSize == 3)
      {
        tx = apu_filter_scharr_3x3_x(&src[x], sstr);
        ty = apu_filter_scharr_3x3_y(&src[x], sstr);
      }
      else
      {
        tx = apu_filter_scharr_5x5_x(&src[x], sstr);
        ty = apu_filter_scharr_5x5_y(&src[x], sstr);
      }

      if (tx < 0)
      {
        tx = -tx;
      }

      if (ty < 0)
      {
        ty = -ty;
      }

      t = tx + ty;
      if (t > 255)
      {
        t = 255;
      }
      aDst.at<uint8_t>(x+y*bw) = (uint8_t)t;
    }
    src += sstr;
  }
}

void
scharr_x_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst)
{
  if((aWindowSize != 3) && (aWindowSize != 5))
  {
    printf("Argument: aWindowSize is incorrect. Window size must to is 3 or 5\n");
    return;
  }
  int bw = aDst.cols;
  int bh = aDst.rows;
  int sstr = bw + 2*(aWindowSize/2);
  const uint8_t* src = (uint8_t*) aSrc.data  + sstr*(aWindowSize/2) + (aWindowSize/2);
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      if(aWindowSize == 3)
      {
        aDst.at<int16_t>(x+y*bw) = apu_filter_scharr_3x3_x(&src[x], sstr);
      }
      else
      {
        aDst.at<int16_t>(x+y*bw) = apu_filter_scharr_5x5_x(&src[x], sstr);
      }
    }
    src += sstr;
  }
}

void
scharr_xy_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst0, vsdk::SMat& aDst1)
{
  if((aWindowSize != 3) && (aWindowSize != 5))
  {
    printf("Argument: aWindowSize is incorrect. Window size must to is 3 or 5\n");
    return;
  }
  int bw = aDst0.cols;
  int bh = aDst0.rows;
  int sstr = bw + 2*(aWindowSize/2);
  const uint8_t* src = (uint8_t*) aSrc.data  + sstr*(aWindowSize/2) + (aWindowSize/2);
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      if(aWindowSize == 3)
      {
        aDst0.at<int16_t>(x+y*bw) = apu_filter_scharr_3x3_x(&src[x], sstr);
        aDst1.at<int16_t>(x+y*bw) = apu_filter_scharr_3x3_y(&src[x], sstr);
      }
      else
      {
        aDst0.at<int16_t>(x+y*bw) = apu_filter_scharr_5x5_x(&src[x], sstr);
        aDst1.at<int16_t>(x+y*bw) = apu_filter_scharr_5x5_y(&src[x], sstr);
      }
    }
    src += sstr;
  }
}

void
scharr_y_08u_ref( vsdk::SMat& aSrc, int aWindowSize, vsdk::SMat& aDst)
{
  if((aWindowSize != 3) && (aWindowSize != 5))
  {
    printf("Argument: aWindowSize is incorrect. Window size must to is 3 or 5\n");
    return;
  }
  int bw = aDst.cols;
  int bh = aDst.rows;
  int sstr = bw + 2*(aWindowSize/2);
  const uint8_t* src = (uint8_t*) aSrc.data  + sstr*(aWindowSize/2) + (aWindowSize/2);
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      if(aWindowSize == 3)
      {
        aDst.at<int16_t>(x+y*bw) = apu_filter_scharr_3x3_y(&src[x], sstr);
      }
      else
      {
        aDst.at<int16_t>(x+y*bw) = apu_filter_scharr_5x5_y(&src[x], sstr);
      }
    }
    src += sstr;
  }
}

void
sobel_x_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_3x3_x(&src[x], sstr);

      // Absolute Value & Saturation
      if (tx > 255)
      {
        tx = 255;
      }
      else if (tx < 0)
      {
        tx = 0;
      }

      dst[x] = (uint8_t)tx;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_x_3x3_08s_ref(
int8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_3x3_x(&src[x], sstr);

      tx = (tx + 4) >> 3;

      // Absolute Value & Saturation
      if (tx > 127)
      {
        tx = 127;
      }
      else if (tx < -128)
      {
        tx = -128;
      }

      dst[x] = (int8_t)tx;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_y_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t ty = apu_filter_sobel_3x3_y(&src[x], sstr);

      // Absolute Value & Saturation
      if (ty > 255)
      {
        ty = 255;
      }
      else if (ty < 0)
      {
        ty = 0;
      }

      dst[x] = (uint8_t)ty;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_y_3x3_08s_ref(
int8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t ty = apu_filter_sobel_3x3_y(&src[x], sstr);

      ty = (ty + 4) >> 3;

      // Absolute Value & Saturation
      if (ty > 127)
      {
        ty = 127;
      }
      else if (ty < -128)
      {
        ty = -128;
      }

      dst[x] = (int8_t)ty;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_x_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_5x5_x(&src[x], sstr);

      // Absolute Value & Saturation
      if (tx > 255)
      {
        tx = 255;
      }
      else if (tx < 0)
      {
        tx = 0;
      }

      dst[x] = (uint8_t)tx;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_y_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t ty = apu_filter_sobel_5x5_y(&src[x], sstr);

      // Absolute Value & Saturation
      if (ty > 255)
      {
        ty = 255;
      }
      else if (ty < 0)
      {
        ty = 0;
      }

      dst[x] = (uint8_t)ty;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_3x3_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_3x3_x(&src[x], sstr);
      if (tx < 0)     { tx = -tx; }

      int16_t ty = apu_filter_sobel_3x3_y(&src[x], sstr);
      if (ty < 0)     { ty = -ty; }


      int16_t t = tx + ty;
      if (t > 255)  { t = 255; }

      dst[x] = (uint8_t)t;
    }

    src += sstr;
    dst += dstr;
  }
}

void
sobel_3x3_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_3x3_x(&src[x], sstr);
      int16_t ty = apu_filter_sobel_3x3_y(&src[x], sstr);

      uint16_t t = abs(tx) + abs(ty);
      if (t > 255) t = 255;

      dst[x] = (uint8_t)t;
    }

    src += sstr;
    dst += dstr;
  }
}


void
sobel_5x5_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh)
{
  for (int y = 0; y < bh; ++y)
  {
    for (int x = 0; x < bw; ++x)
    {
      int16_t tx = apu_filter_sobel_5x5_x(&src[x], sstr);
      if (tx < 0)     { tx = -tx; }

      int16_t ty = apu_filter_sobel_5x5_y(&src[x], sstr);
      if (ty < 0)     { ty = -ty; }

      int16_t t = tx + ty;
      if (t > 255)  { t = 255; }

      dst[x] = (uint8_t)t;
    }

    src += sstr;
    dst += dstr;
  }
}

// LOCAL HELPER FUNCTIONS


//  3,  0,  -3,
//  10, 0,  -10,
//  3,  0,  -3,
static
inline
int16_t
apu_filter_scharr_3x3_x(const uint8_t* src, int sstr)
{
  int16_t vd = +src[-sstr - 1] * 3 - src[-sstr + 1] * 3
    + 10 * src[-1] - 10 * src[+1]
    + src[+sstr - 1] * 3 - src[+sstr + 1] * 3;


  return vd;
}

// +3, +10, +3,
//  0,  0,   0,
// -3, -10, -3,
static
inline
int16_t
apu_filter_scharr_3x3_y(const uint8_t* src, int sstr)
{
  int16_t t7 = +src[-sstr + 1] - src[+sstr - 1];
  int16_t t9 = +src[-sstr - 1] - src[+sstr + 1];
  int16_t t8 = +src[-sstr] - src[+sstr];
  int16_t vd = 3 * t7 + 10 * t8 + 3 * t9;

  return vd;
}

/*
    -1,  -1,  0,  +1,  +1,
    -2,  -2,  0,  +2,  +2,
    -3,  -6,  0,  +6,  +3,
    -2,  -2,  0,  +2,  +2,
    -1,  -1,  0,  +1,  +1,
    */
static
inline
int16_t
apu_filter_scharr_5x5_x(const uint8_t* src, int sstr)
{
  const int16_t t22 = -src[-2 * sstr - 2] + src[-2 * sstr + 2] - src[+2 * sstr - 2] + src[+2 * sstr + 2];
  const int16_t t12 = -src[-2 * sstr - 1] + src[-2 * sstr + 1] - src[+2 * sstr - 1] + src[+2 * sstr + 1];
  const int16_t t21 = -src[-1 * sstr - 2] + src[-1 * sstr + 2] - src[+1 * sstr - 2] + src[+1 * sstr + 2];
  const int16_t t11 = -src[-1 * sstr - 1] + src[-1 * sstr + 1] - src[+1 * sstr - 1] + src[+1 * sstr + 1];
  const int16_t t20 = -src[-2] + src[+2];
  const int16_t t10 = -src[-1] + src[+1];

  const int16_t tx = t22 + t12 + 2 * t21 + 2 * t11 + 3 * t20 + 6 * t10;

  return tx;
}

/*
    -1,  -2,  -3,  -2,  -1,
    -1,  -2,  -6,  -2,  -1,
     0,   0,   0,   0,   0,
    +1,  +2,  +6,  +2,  +1,
    +1,  +2,  +3,  +2,  +1,
    */
static
inline
int16_t
apu_filter_scharr_5x5_y(const uint8_t* src, int sstr)
{
  const int16_t t22 = -src[-2 * sstr - 2] - src[-2 * sstr + 2] + src[+2 * sstr - 2] + src[+2 * sstr + 2];
  const int16_t t12 = -src[-2 * sstr - 1] - src[-2 * sstr + 1] + src[+2 * sstr - 1] + src[+2 * sstr + 1];
  const int16_t t21 = -src[-1 * sstr - 2] - src[-1 * sstr + 2] + src[+1 * sstr - 2] + src[+1 * sstr + 2];
  const int16_t t11 = -src[-1 * sstr - 1] - src[-1 * sstr + 1] + src[+1 * sstr - 1] + src[+1 * sstr + 1];
  const int16_t t02 = -src[-2 * sstr] + src[+2 * sstr];
  const int16_t t01 = -src[-1 * sstr] + src[+1 * sstr];

  const int16_t tx = t22 + 2 * t12 + t21 + 2 * t11 + 3 * t02 + 6 * t01;

  return tx;
}

//  -1,  0,  1,
//  -2,  0,  2,
//  -1,  0,  1,
static
inline
int16_t
apu_filter_sobel_3x3_x(const uint8_t* src, int sstr)
{
  int16_t vd = +src[-sstr + 1] - src[-sstr - 1]
    + 2 * src[+1] - 2 * src[-1]
    + src[+sstr + 1] - src[+sstr - 1];


  return vd;
}

// -1, -2, -1,
//  0,  0,  0,
// +1, +2, +1,
static
inline
int16_t
apu_filter_sobel_3x3_y(const uint8_t* src, int sstr)
{
  int16_t t7 = +src[+sstr + 1] - src[-sstr - 1];
  int16_t t9 = +src[+sstr - 1] - src[-sstr + 1];
  int16_t t8 = +src[+sstr] - src[-sstr];
  int16_t vd = t7 + 2 * t8 + t9;

  return vd;
}


/*
    -1,  -2,  0,  +2,  +1,
    -4,  -8,  0,  +8,  +4,
    -6, -12,  0, +12,  +6,
    -4,  -8,  0,  +8,  +4,
    -1,  -2,  0,  +2,  +1,
    */
static
inline
int16_t
apu_filter_sobel_5x5_x(const uint8_t* src, int sstr)
{
  const int16_t t22 = -src[-2 * sstr - 2] + src[-2 * sstr + 2] - src[+2 * sstr - 2] + src[+2 * sstr + 2];
  const int16_t t12 = -src[-2 * sstr - 1] + src[-2 * sstr + 1] - src[+2 * sstr - 1] + src[+2 * sstr + 1];
  const int16_t t21 = -src[-1 * sstr - 2] + src[-1 * sstr + 2] - src[+1 * sstr - 2] + src[+1 * sstr + 2];
  const int16_t t11 = -src[-1 * sstr - 1] + src[-1 * sstr + 1] - src[+1 * sstr - 1] + src[+1 * sstr + 1];
  const int16_t t20 = -src[-2] + src[+2];
  const int16_t t10 = -src[-1] + src[+1];

  const int16_t tx = t22 + 2 * t12 + 4 * t21 + 8 * t11 + 6 * t20 + 12 * t10;

  return tx;
}

/*
    -1,  -4,  -6,  -4,  -1,
    -2,  -8, -12,  -8,  -2,
    0,   0,   0,   0,   0,
    +2,  +8, +12,  +8,  +2,
    +1,  +4,  +6,  +4,  +1,
    */
static
inline
int16_t
apu_filter_sobel_5x5_y(const uint8_t* src, int sstr)
{
  const int16_t t22 = -src[-2 * sstr - 2] - src[-2 * sstr + 2] + src[+2 * sstr - 2] + src[+2 * sstr + 2];
  const int16_t t12 = -src[-2 * sstr - 1] - src[-2 * sstr + 1] + src[+2 * sstr - 1] + src[+2 * sstr + 1];
  const int16_t t21 = -src[-1 * sstr - 2] - src[-1 * sstr + 2] + src[+1 * sstr - 2] + src[+1 * sstr + 2];
  const int16_t t11 = -src[-1 * sstr - 1] - src[-1 * sstr + 1] + src[+1 * sstr - 1] + src[+1 * sstr + 1];
  const int16_t t02 = -src[-2 * sstr] + src[+2 * sstr];
  const int16_t t01 = -src[-1 * sstr] + src[+1 * sstr];

  const int16_t tx = t22 + 4 * t12 + 2 * t21 + 8 * t11 + 6 * t02 + 12 * t01;

  return tx;
}

void
convolve_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh)
{
  // Adjust src
  src -= ((fh >> 1)*sstr + (fw >> 1));

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int16_t r = 0;
      for (int fy = 0, fk = 0; fy < fh; ++fy)
      {
        for (int fx = 0; fx < fw; ++fx, ++fk)
        {
          r += ((int16_t)flt[fk] * src[(by + fy)*sstr + (bx + fx)]);
        }
      }

      // Saturation
      if (r > 255)
      {
        r = 255;
      }
      else if (r < 0)
      {
        r = 0;
      }

      dst[by*dstr + bx] = (uint8_t)r;
    }
  }
}

void
convolve_scale_08u_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl)
{
  // Adjust src
  src -= ((fh >> 1)*sstr + (fw >> 1));

  const int fone = (1 << fscl);

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int16_t r = 0;
      for (int fy = 0, fk = 0; fy < fh; ++fy)
      {
        for (int fx = 0; fx < fw; ++fx, ++fk)
        {
          r += ((int16_t)flt[fk] * src[(by + fy)*sstr + (bx + fx)]);
        }
      }
      // Rounding & Scaling
      r += (fone >> 1);
      r >>= fscl;

      // Saturation
      if (r > 255)
      {
        r = 255;
      }
      else if (r < 0)
      {
        r = 0;
      }

      dst[by*dstr + bx] = (uint8_t)r;
    }
  }
}

void
convolve_scale_08u_nr_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl)
{
  // Adjust src
  src -= ((fh >> 1)*sstr + (fw >> 1));

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int16_t r = 0;
      for (int fy = 0, fk = 0; fy < fh; ++fy)
      {
        for (int fx = 0; fx < fw; ++fx, ++fk)
        {
          r += ((int16_t)flt[fk] * src[(by + fy)*sstr + (bx + fx)]);
        }
      }
      // Rounding & Scaling
      r >>= fscl;

      // Saturation
      if (r > 255)
      {
        r = 255;
      }
      else if (r < 0)
      {
        r = 0;
      }

      dst[by*dstr + bx] = (uint8_t)r;
    }
  }
}

void
convolve_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh)
{
  // Adjust src
  src -= ((fh >> 1)*sstr + (fw >> 1));

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int32_t r = 0;
      for (int fy = 0, fk = 0; fy < fh; ++fy)
      {
        for (int fx = 0; fx < fw; ++fx, ++fk)
        {
          r += ((int16_t)flt[fk] * (int16_t)src[(by + fy)*sstr + (bx + fx)]);
        }
      }

      // Saturation
      if (r > 32767)
      {
        r = 32767;
      }
      else if (r < -32768)
      {
        r = -32768;
      }

      dst[by*dstr + bx] = (int16_t)r;
    }
  }
}

void
convolve_scale_16s_ref(
int16_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const int8_t* flt, int fw, int fh,
int fscl)
{
  // Adjust src
  src -= ((fh >> 1)*sstr + (fw >> 1));

  const int fone = (1 << fscl);

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int32_t r = 0;
      for (int fy = 0, fk = 0; fy < fh; ++fy)
      {
        for (int fx = 0; fx < fw; ++fx, ++fk)
        {
          r += ((int16_t)flt[fk] * src[(by + fy)*sstr + (bx + fx)]);
        }
      }
      // Rounding & Scaling
      r += (fone >> 1);
      r >>= fscl;

      // Saturation
      if (r > 32767)
      {
        r = 32767;
      }
      else if (r < -32768)
      {
        r = -32768;
      };

      dst[by*dstr + bx] = (int16_t)r;
    }
  }
}

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
int bw, int bh)
{
  const uint8_t* s0 = src - sstr;
  const uint8_t* s1 = src;
  const uint8_t* s2 = src + sstr;

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int16_t r;
      r = s0[bx + 1] - s0[bx - 1];
      r += s1[bx + 1] - s1[bx - 1];
      r += s2[bx + 1] - s2[bx - 1];

      dst[bx] = r;
    }

    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

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
int bw, int bh)
{
  const uint8_t* s0 = src - sstr;
  const uint8_t* s1 = src;
  const uint8_t* s2 = src + sstr;

  for (int by = 0; by < bh; ++by)
  {
    for (int bx = 0; bx < bw; ++bx)
    {
      int16_t r;
      r = s0[bx - 1] + s0[bx] + s0[bx + 1];
      r -= (s2[bx - 1] + s2[bx] + s2[bx + 1]);

      dst[bx] = r;
    }

    s0 = s1;
    s1 = s2;
    s2 += sstr;
    dst += dstr;
  }
}

void
saturate_08s_ref(
int8_t* dst, int dstr,
const int16_t* src, int sstr,
int bw, int bh
)
{
  for (int i = 0; i < bh; ++i)
  {
    for (int j = 0; j < bw; ++j)
    {
      int16_t val = (src[j] + 4) >> 3;
      if (val < -128)
      {
        val = -128;
      }
      else if (val > 127)
      {
        val = 127;
      }

      dst[j] = (int8_t)val;
    }
    src += sstr;
    dst += dstr;
  }
}
