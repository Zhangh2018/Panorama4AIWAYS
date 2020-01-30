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
*  \file apexcv_arithmetic_ref.cpp
*  \brief Reference code for arithmetic kernels
***********************************************************************************/
#ifndef ARITHMETICREF_HPP
#define ARITHMETICREF_HPP

#include <cmath>
#include <apexcv_enum_types.hpp>
#include <stdint.h>
#include <stdlib.h>
#include <algorithm>
#include <limits>
#include <sumat.hpp>
#include <cmath>
#define _USE_MATH_DEFINES

//TODO: this reference code badly needs to be refactored

using namespace std;

template <typename S, typename D>
void abs_ref(const S* src, int sstr,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  long mx = numeric_limits<D>::max();

  for (int y = 0; y < blockHeight; ++y)
  {
    for (int x = 0; x < blockWidth; ++x)
    {
      long abs_val = abs((long)src[x]);

      if (abs_val > mx)
      {
        dst[x] = mx;
      }
      else
      {
        dst[x] = abs_val;
      }
    }

    dst += dstr;
    src += sstr;
  }
}

template <typename D>
void absdiff_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  long mx = numeric_limits<D>::max();

  for (int y = 0; y < blockHeight; ++y)
  {

    for (int x = 0; x < blockWidth; ++x)
    {
      long absdiff = abs((long)max(src0[x], src1[x]) - (long)min(src0[x], src1[x]));

      if (absdiff > mx)
      {
        dst[x] = mx;
      }
      else
      {
        dst[x] = absdiff;
      }
    }

    dst += dstr;
    src0 += sstr0;
    src1 += sstr1;
  }
}


void add_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);


template<typename S1, typename S2, typename D>
void _add_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  int bh = aDst.rows;
  int bw = aDst.cols;

  long mx = numeric_limits<D>::max();
  long mn = numeric_limits<D>::min();

  if(aPolicy == apexcv::eConvertPolicy::WRAP)
  {
    for (int y = 0; y < bh; ++y)
    {
      for (int x = 0; x < bw; ++x)
      {
        long sum = aSrc0.at<S1>(x + y * bw) + aSrc1.at<S2>(x + y * bw);
        aDst.at<D>(x + y * bw) = (D)sum;
      }
    }
  }
  else if (aPolicy == apexcv::eConvertPolicy::SATURATE)
  {
    for (int y = 0; y < bh; ++y)
    {
      for (int x = 0; x < bw; ++x)
      {
        long sum = aSrc0.at<S1>(x + y * bw) + aSrc1.at<S2>(x + y * bw);
        if (sum > mx)
        {
          aDst.at<D>(x + y * bw) = (D)mx;
        }
        else if (sum < mn)
        {
          aDst.at<D>(x + y * bw) = (D)mn;
        }
        else
        {
          aDst.at<D>(x + y * bw) = (D)sum;
        }
      }
    }
  }
}

template<typename S1, typename S2, typename D>
void _sub_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  int bh = aDst.rows;
  int bw = aDst.cols;

  long mx = numeric_limits<D>::max();
  long mn = numeric_limits<D>::min();

  if(aPolicy == apexcv::eConvertPolicy::WRAP)
  {
    for (int y = 0; y < bh; ++y)
    {
      for (int x = 0; x < bw; ++x)
      {
        long sub = aSrc0.at<S1>(x + y * bw) - aSrc1.at<S2>(x + y * bw);
        aDst.at<D>(x + y * bw) = (D)sub;
      }
    }
  }
  else if (aPolicy == apexcv::eConvertPolicy::SATURATE)
  {
    for (int y = 0; y < bh; ++y)
    {
      for (int x = 0; x < bw; ++x)
      {
        long sub = aSrc0.at<S1>(x + y * bw) - aSrc1.at<S2>(x + y * bw);
        if (sub > mx)
        {
          aDst.at<D>(x + y * bw) = (D)mx;
        }
        else if (sub < mn)
        {
          aDst.at<D>(x + y * bw) = (D)mn;
        }
        else
        {
          aDst.at<D>(x + y * bw) = (D)sub;
        }
      }
    }
  }
}

void sub_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy);


template <typename D>
void and_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  for (int y = 0; y < blockHeight; ++y)
  {

    for (int x = 0; x < blockWidth; ++x)
    {

      dst[x] = src0[x] & src1[x];
    }

    dst += dstr;
    src0 += sstr0;
    src1 += sstr1;
  }
}

template <typename S, typename D>
void clz_ref(S* src, int sstr,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  uint8_t bits = sizeof(src[0]) * 8;
  D numZeros;
  uint8_t msb_08 = 128;
  uint16_t msb_16 = 32768;


  for (int y = 0; y < blockHeight; ++y)
  {
    for (int x = 0; x < blockWidth; ++x)
    {

      S src_x = src[x];
      numZeros = 0;
      if (src_x == 0)
      {
        numZeros = (D)bits;
      }
      else
      {
        for (int i = 1; i < bits; ++i)
        {
          if ((bits == 8 && (src_x & msb_08)) || (bits == 16 && (src_x & msb_16)))
          {
            break;
          }
          ++numZeros;
          src_x <<= 1;
        }
      }
      dst[x] = numZeros;
    }

    dst += dstr;
    src += sstr;
  }
}

template <typename S, typename D>
void magnitude_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  const S* sx;
  const S* sy;

  for (int by = 0; by < blockHeight; by++)
  {
    sx = (const S*)src0;
    sy = (const S*)src1;

    for (int bx = 0; bx < blockWidth; bx++)  {
      // Compute magnitude       
      uint32_t ax = (uint32_t)abs(sx[bx]);
      uint32_t ay = (uint32_t)abs(sy[bx]);

      uint32_t sx2 = (uint32_t)ax * (uint32_t)ax;
      uint32_t sy2 = (uint32_t)ay * (uint32_t)ay;

      uint32_t s2 = sx2 + sy2;

      dst[bx] = (D)sqrt((double)s2);
    }
    src0 += sstr0;
    src1 += sstr1;
    dst += dstr;
  }
}

template <typename S, typename D>
void max_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  const S* sx;
  const S* sy;

  for (int by = 0; by < blockHeight; by++)
  {
    sx = (const S*)src0;
    sy = (const S*)src1;

    for (int bx = 0; bx < blockWidth; bx++)  {
      // Compute max
      dst[bx] = (D)((sx[bx] > sy[bx]) ? sx[bx] : sy[bx]);
    }
    src0 += sstr0;
    src1 += sstr1;
    dst += dstr;
  }
}

template <typename S, typename D>
void min_ref(const S* src0, int sstr0,
  const S* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  const S* sx;
  const S* sy;

  for (int by = 0; by < blockHeight; by++)
  {
    sx = (const S*)src0;
    sy = (const S*)src1;

    for (int bx = 0; bx < blockWidth; bx++)  {
      // Compute max
      dst[bx] = (D)((sx[bx] < sy[bx]) ? sx[bx] : sy[bx]);
    }
    src0 += sstr0;
    src1 += sstr1;
    dst += dstr;
  }
}

template <typename S1, typename S2, typename D>
int _mul_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aScale, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  int bh = aDst.rows;
  int bw = aDst.cols;
  int status = 0;
  long mx = numeric_limits<D>::max();
  long mn = numeric_limits<D>::min();

  if(aPolicy == apexcv::eConvertPolicy::WRAP)
  { 
    if (aScale.at<uint8_t>(0u) == 255)
    {
      for (int y = 0; y < bh; y++)
      {
        for (int x = 0; x < bw; ++x)
        {
           int s1i = (int) aSrc0.at<S1>(x + y * bw);
           int s2i = (int) aSrc1.at<S2>(x + y * bw);
           int tx = s2i * s1i;
           double tx_unscaled = (double)tx;
           double tx_scaled = (double)(tx_unscaled / (double)255);
           int tx_result0 = (int)floor(tx_scaled);
           int tx_result1 = tx_result0 + 1;
           if(aDst.at<D>(x + y * bw) != (D)tx_result0 && aDst.at<D>(x + y * bw) != (D)tx_result1)
           {
              status = 1;
              return status;
           }
        }
      }
    }
    else if (0 <= aScale.at<uint8_t>(0u) <= 15)
    {
      for (int y = 0; y < bh; y++)
      {
        for (int x = 0; x < bw; ++x)
        {
           int s1i = (int) aSrc0.at<S1>(x + y * bw);
           int s2i = (int) aSrc1.at<S2>(x + y * bw);
           int tx = s2i * s1i;
           double tx_unscaled = (double)tx;
           uint8_t lScale = aScale.at<uint8_t>(0u);
           double tx_scaled = (double)(tx_unscaled / (pow(2, lScale)));
           int tx_unround = (int)floor(tx_scaled);
           if(aDst.at<D>(x + y * bw) != (D)tx_unround)
           {
              status = 1;
              return status;
           }
        }
      }
    }
  }
  else if (aPolicy == apexcv::eConvertPolicy::SATURATE)
  {
    if (aScale.at<uint8_t>(0u) == 255)
    {  
      for (int y = 0; y < bh; y++)
      {
        for (int x = 0; x < bw; ++x)
        {
           int s1i = (int) aSrc0.at<S1>(x + y * bw);
           int s2i = (int) aSrc1.at<S2>(x + y * bw);
           int tx = s2i * s1i;
           double tx_unscaled = (double)tx;
           double tx_scaled = (double)(tx_unscaled / (double)255);
           int tx_result0 = (int)floor(tx_scaled);
           int tx_result1 = tx_result0 + 1;
           if (tx_result0 > mx)
           {
             tx_result0 = (int)mx;
           }
           else if (tx_result0 < mn)
           {
             tx_result0 = (int)mn;
           }

           if (tx_result1 > mx)
           {
             tx_result1 = (int)mx;
           }
           else if (tx_result1 < mn)
           {
             tx_result1 = (int)mn;
           }
           if(aDst.at<D>(x + y * bw) != (D)tx_result0 && aDst.at<D>(x + y * bw) != (D)tx_result1)
           {
              status = 1;
              return status;
           }
        }
      }
    }
    else if (0 <= aScale.at<uint8_t>(0u) <= 15)
    {
      for (int y = 0; y < bh; y++)
      {
        for (int x = 0; x < bw; ++x)
        {
           int s1i = (int) aSrc0.at<S1>(x + y * bw);
           int s2i = (int) aSrc1.at<S2>(x + y * bw);
           int tx = s2i * s1i;           
           double tx_unscaled = (double)tx;
           uint8_t lScale = aScale.at<uint8_t>(0u);
           double tx_scaled = (double)(tx_unscaled / (pow(2, lScale)));
           int tx_unround = (int)floor(tx_scaled);
           if (tx_unround > mx)
           {
             tx_unround = (int)mx;
           }
           else if (tx_unround < mn)
           {
             tx_unround = (int)mn;
           }
           if(aDst.at<D>(x + y * bw) != (D)tx_unround)
           {
              status = 1;
              return status;
           }
        }
      }
    }
  }
  return status;
}

int mul_ref(vsdk::SMat& aSrc0, vsdk::SMat& aSrc1, vsdk::SMat& aScale, vsdk::SMat& aDst, apexcv::eConvertPolicy aPolicy)
{
  int lRetVal=0;
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_8UC1))
  {
    lRetVal = _mul_ref<uint8_t, uint8_t, uint8_t>(aSrc0, aSrc1, aScale, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_8UC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    lRetVal = _mul_ref<uint8_t, uint8_t, int16_t>(aSrc0, aSrc1, aScale, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_8UC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    lRetVal = _mul_ref<uint8_t, int16_t, int16_t>(aSrc0, aSrc1, aScale, aDst, aPolicy);
  }
  if ((aSrc0.type() == VSDK_CV_16SC1) && (aSrc1.type() == VSDK_CV_16SC1) && (aDst.type() == VSDK_CV_16SC1))
  {
    lRetVal = _mul_ref<int16_t, int16_t, int16_t>(aSrc0, aSrc1, aScale, aDst, aPolicy);
  }
  return lRetVal;
}


template <typename D>
void not_ref(const D* src0, int sstr0,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  for (int y = 0; y < blockHeight; ++y)
  {

    for (int x = 0; x < blockWidth; ++x)
    {

      dst[x] = ~src0[x];
    }

    dst += dstr;
    src0 += sstr0;
  }
}

template <typename D>
void or_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  for (int y = 0; y < blockHeight; ++y)
  {

    for (int x = 0; x < blockWidth; ++x)
    {

      dst[x] = src0[x] | src1[x];
    }

    dst += dstr;
    src0 += sstr0;
    src1 += sstr1;
  }
}

template <typename D>
void tablelookup_ref(const D* src0, int sstr0,
  D* dst, int dstr,
  int blockWidth, int blockHeight,
  const D* lut)
{
  for (int y = 0; y < blockHeight; ++y)
  {
    for (int x = 0; x < blockWidth; ++x)
    {
      dst[x + y*dstr] = lut[src0[x + y*sstr0]];
    }
  }
}


// Compatible with OpenVX Binary Threshold
template <typename D>
void threshold_ref(const D* src0, const int sstr0,
  const D threshold,
  unsigned char* dst, const int dstr,
  const int blockWidth, const int blockHeight,
  const uint8_t val_true, const uint8_t val_false)
{
  for (int y = 0; y < blockHeight; ++y)
  {
    for (int x = 0; x < blockWidth; ++x)
    {
      if (src0[x] > threshold)
      {
        dst[x] = val_true;
      }
      else
      {
        dst[x] = val_false;
      }
    }
    dst += dstr;
    src0 += sstr0;
  }
}


// Compatible with OpenVX Range Threshold
template <typename D>
void threshold2_ref(const D* src0, const int sstr0,
  const D threshold1, const D threshold2,
  unsigned char* dst, const int dstr,
  const int blockWidth, const int blockHeight,
  const uint8_t val_true, const uint8_t val_false)
{
  for (int y = 0; y < blockHeight; ++y)
  {
    for (int x = 0; x < blockWidth; ++x)
    {
      if ((src0[x] >= threshold1) && (src0[x] <= threshold2))
      {
        dst[x] = val_true;
      }
      else
      {
        dst[x] = val_false;
      }
    }
    dst += dstr;
    src0 += sstr0;
  }
}



template <typename D>
void xor_ref(const D* src0, int sstr0,
  const D* src1, int sstr1,
  D* dst, int dstr,
  int blockWidth, int blockHeight)
{
  for (int y = 0; y < blockHeight; ++y)
  {

    for (int x = 0; x < blockWidth; ++x)
    {

      dst[x] = src0[x] ^ src1[x];
    }

    dst += dstr;
    src0 += sstr0;
    src1 += sstr1;
  }
}

#endif /* ARITHMETICREF_HPP */


