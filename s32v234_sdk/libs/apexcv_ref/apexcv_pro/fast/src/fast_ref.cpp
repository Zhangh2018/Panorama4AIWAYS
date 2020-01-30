/*
   Copyright (c) 2006, 2008 Edward Rosten
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

 * Neither the name of the University of Cambridge nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
   The references are:
 * Machine learning for high-speed corner detection,
   E. Rosten and T. Drummond, ECCV 2006
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
 */

/*
 * Copyright 2017 NXP
 */

#define _CRT_SECURE_NO_DEPRECATE

// TODO Replace this macro with an APEXCV MAX_IMG_SIZE macro,
// when one becomes available.
#define MAX_IMG_SIZE 4096

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "fast_ref.h"

typedef uint8_t uchar;

static void MakeOffsets25(int apPixel[25], int aRowStride, int aPatternSize);
static int CornerScore16(const uchar *apcPtr, const int apcPixel[], int aThreshold);
static void TestCorner(const uchar *apcPtr, const int apcPixel[], int aK, int aN, int aThreshold);
static int ReplicatePadding(const uint8_t *apcSrc, uint8_t *apDst, int aWidth, int aHeight, int aTop, int aBottom, int aLeft, int aRight);
static int RefFast16(uint8_t *apOimg, const uint8_t *apcIimg, const int acIw, const int acIh, int aThreshold, int aNonmaxSuppression);

inline int StdMin(int a, int b)
{
  return (a < b) ? a : b;
}

inline int StdMax(int a, int b)
{
  return (a > b) ? a : b;
}

static int ReplicatePadding(const uint8_t *apcSrc, uint8_t *apDst, int aWidth, int aHeight, int aTop, int aBottom, int aLeft, int aRight)
{
  int sstr;
  int dstr;
  uint8_t *buf;
  int w, h;

  sstr = aWidth;
  dstr = aWidth + aLeft + aRight;
  buf  = apDst + aTop * dstr;

  for(h = 0; h < aHeight; h++)
  {
    for(w = 0; w < aLeft; w++)
    {
      buf[h * dstr + w] = apcSrc[h * sstr];
    }

    for(; w < aLeft + aWidth; w++)
    {
      buf[h * dstr + w] = apcSrc[h * sstr + (w - aLeft)];
    }

    for(; w < dstr; w++)
    {
      buf[h * dstr + w] = apcSrc[h * sstr + aWidth - 1];
    }
  }

  for(w = 0; w < dstr; w++)
  {
    for(h = 0; h < aTop; h++)
    {
      apDst[h * dstr + w] = buf[w];
    }

    for(int h = aTop + aHeight; h < aTop + aHeight + aBottom; h++)
    {
      apDst[h * dstr + w] = buf[(aHeight - 1) * dstr + w];
    }
  }

  return 0;
}

static int Nms3x3(uint8_t *apImg, int aIw, int aIh)
{
  int stride;
  int radius;
  uint8_t *input_padded;
  uint8_t *input_padded_tmp;
  uint8_t *mask;
  int final_ncorners;

  radius         = 3;
  stride         = aIw + (radius - 1);
  final_ncorners = 0;

  // Pad output image for NMS
  input_padded = (uint8_t *)malloc(sizeof(uint8_t) * (aIw + radius - 1) * (aIh + radius - 1));

  // Replicate padding
  ReplicatePadding(apImg, input_padded, aIw, aIh, radius >> 1, radius >> 1, radius >> 1, radius >> 1);

  //
  input_padded_tmp = input_padded;
  input_padded     = input_padded + (radius >> 1) * stride + (radius >> 1);

  // NMS
  mask = (uint8_t *)malloc(sizeof(uint8_t) * aIw * aIh);

  for(int i = 0; i < aIh; i++)
  {
    for(int j = 0; j < aIw; j++)
    {
      const uint8_t *ptr = &input_padded[i * stride + j];
      int cval;

      cval = ptr[0];

      mask[i * aIw + j] = cval >= ptr[-1 - stride] &&
                          cval >= ptr[-stride] &&
                          cval >= ptr[-stride + 1] &&
                          cval >= ptr[-1] &&
                          cval > ptr[-1 + stride] &&
                          cval > ptr[stride] &&
                          cval > ptr[stride + 1] &&
                          cval > ptr[1];
    }
  }

  for(int i = 0; i < aIh; i++)
  {
    for(int j = 0; j < aIw; j++)
    {
      if(mask[i * aIw + j])
      {
        final_ncorners++;
      }
      else
      {
        apImg[i * aIw + j] = 0;
      }
    }
  }

  // Release allocated memory
  free(input_padded_tmp);
  free(mask);

  // Return the number of corners
  return final_ncorners;
}

static void MakeOffsets25(int apPixel[25], int aRowStride, int aPatternSize)
{
  int k;

  static const int offsets16[][2] =
  {
    { 0, 3 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 3, 0 }, { 3, -1 }, { 2, -2 }, { 1, -3 },
    { 0, -3 }, { -1, -3 }, { -2, -2 }, { -3, -1 }, { -3, 0 }, { -3, 1 }, { -2, 2 }, { -1, 3 }
  };
  static const int offsets12[][2] =
  {
    { 0, 2 }, { 1, 2 }, { 2, 1 }, { 2, 0 }, { 2, -1 }, { 1, -2 },
    { 0, -2 }, { -1, -2 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -1, 2 }
  };
  static const int offsets8[][2] =
  {
    { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 },
    { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 }
  };
  const int (*offsets)[2] = aPatternSize == 16 ? offsets16 :
                            aPatternSize == 12 ? offsets12 :
                            aPatternSize == 8  ? offsets8  : 0;

  for(k = 0; k < aPatternSize; k++)
  {
    apPixel[k] = offsets[k][0] + offsets[k][1] * aRowStride;
  }

  for(; k < 25; k++)
  {
    apPixel[k] = apPixel[k - aPatternSize];
  }
}

static void TestCorner(const uchar *apcPtr, const int apcPixel[], int aK, int aN, int aThreshold)
{
  // Check that with the computed "threshold" the pixel is still a corner
  // and that with the increased-by-1 "threshold" the pixel is not a corner anymore
  for(int delta = 0; delta <= 1; delta++)
  {
    int v0, v1;
    int c0, c1;

    v0 = StdMin(apcPtr[0] + aThreshold + delta, 255);
    v1 = StdMax(apcPtr[0] - aThreshold - delta, 0);
    c0 = 0;
    c1 = 0;

    for(int k = 0; k < aN; k++)
    {
      int x = apcPtr[apcPixel[k]];

      if(x > v0)
      {
        if(++c0 > aK)
        {
          break;
        }
        c1 = 0;
      }
      else
      {
        if(x < v1)
        {
          if(++c1 > aK)
          {
            break;
          }
          c0 = 0;
        }
        else
        {
          c0 = c1 = 0;
        }
      }
    }
  }
}

static int CornerScore16(const uchar *apcPtr, const int apcPixel[], int aThreshold)
{
  const int K = 8;
  const int N = K * 3 + 1;
  int k, v;
  int a0, b0;
  short d[N];

  v = apcPtr[0];

  for(k = 0; k < N; k++)
  {
    d[k] = (short)(v - apcPtr[apcPixel[k]]);
  }

  a0 = aThreshold;

  for(k = 0; k < 16; k += 2)
  {
    int a;

    a = StdMin((int)d[k + 1], (int)d[k + 2]);
    a = StdMin(a, (int)d[k + 3]);

    if(a <= a0)
    {
      continue;
    }

    a  = StdMin(a, (int)d[k + 4]);
    a  = StdMin(a, (int)d[k + 5]);
    a  = StdMin(a, (int)d[k + 6]);
    a  = StdMin(a, (int)d[k + 7]);
    a  = StdMin(a, (int)d[k + 8]);
    a0 = StdMax(a0, StdMin(a, (int)d[k]));
    a0 = StdMax(a0, StdMin(a, (int)d[k + 9]));
  }

  b0 = -a0;

  for(k = 0; k < 16; k += 2)
  {
    int b;

    b = StdMax((int)d[k + 1], (int)d[k + 2]);
    b = StdMax(b, (int)d[k + 3]);
    b = StdMax(b, (int)d[k + 4]);
    b = StdMax(b, (int)d[k + 5]);

    if(b >= b0)
    {
      continue;
    }

    b  = StdMax(b, (int)d[k + 6]);
    b  = StdMax(b, (int)d[k + 7]);
    b  = StdMax(b, (int)d[k + 8]);
    b0 = StdMin(b0, StdMax(b, (int)d[k]));
    b0 = StdMin(b0, StdMax(b, (int)d[k + 9]));
  }
  aThreshold = -b0 - 1;

  #if VERIFY_CORNERS
  TestCorner(apcPtr, apcPixel, K, N, aThreshold);
  #endif

  return aThreshold;
}

static int RefFast16(uint8_t *apOimg, const uint8_t *apcIimg, const int acIw, const int acIh, int aThreshold, int aNonmaxSuppression)
{
  uint8_t *iimg_padded;

  int final_ncorners;
  int istride;
  int i, j, k;
  int pixel[25];
  uchar threshold_tab[512];
  int _buf[4][MAX_IMG_SIZE];
  uchar *buf[3];
  int *cpbuf[3];
  uint8_t *iimg_padded_tmp;

  const int fast9_filter_size = 7;
  const int patternSize       = 16;
  const int K                 = patternSize / 2;
  const int N                 = patternSize + K + 1;
  final_ncorners = 0;

  // Create replicate padding on input image
  // fast9 is like 7x7 filter -> apply 3 replicate padding
  iimg_padded = (uint8_t *)malloc(sizeof(uint8_t) *
                                  (acIw + fast9_filter_size - 1) *
                                  (acIh + fast9_filter_size - 1));

  istride = acIw + (fast9_filter_size - 1);
  ReplicatePadding(apcIimg, iimg_padded, acIw, acIh,
                   fast9_filter_size >> 1, fast9_filter_size >> 1,
                   fast9_filter_size >> 1, fast9_filter_size >> 1);

  MakeOffsets25(pixel, istride, patternSize);

  aThreshold = StdMin(StdMax(aThreshold, 0), 255);

  for(i = -255; i <= 255; i++)
  {
    threshold_tab[i + 255] = (uchar)(i < -aThreshold ? 1 : i > aThreshold ? 2 : 0);
  }

  buf[0] = (uchar *)(&_buf[0][0]);
  buf[1] = buf[0] + MAX_IMG_SIZE;
  buf[2] = buf[1] + MAX_IMG_SIZE;

  cpbuf[0] = _buf[1];
  cpbuf[1] = _buf[2];
  cpbuf[2] = _buf[2];

  memset(buf[0], 0, MAX_IMG_SIZE * sizeof(int));

  iimg_padded_tmp = iimg_padded;
  iimg_padded     = iimg_padded + (fast9_filter_size >> 1) * istride + (fast9_filter_size >> 1);

  for(i = 0; i < acIh; i++)
  {
    uchar *curr;
    int *cornerpos;
    int ncorners;
    const uchar *ptr = iimg_padded + i * istride;

    curr      = buf[i % 3];
    cornerpos = cpbuf[i % 3];

    memset(curr, 0, acIw);
    ncorners = 0;

    if(i < acIw)
    {
      for(j = 0; j < acIw; j++, ptr++)
      {
        int d;
        const int v      = ptr[0];
        const uchar *tab = &threshold_tab[0] - v + 255;

        apOimg[i * acIw + j] = 0;
        d                    = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

        if(d == 0)
        {
          continue;
        }
        d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
        d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
        d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

        if(d == 0)
        {
          continue;
        }
        d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
        d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
        d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
        d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

        if(d & 1)
        {
          int vt;
          int count;

          vt    = v - aThreshold;
          count = 0;

          for(k = 0; k < N; k++)
          {
            int x = ptr[pixel[k]];

            if(x < vt)
            {
              if(++count > K)
              {
                char c;

                cornerpos[ncorners++] = j;
                apOimg[i * acIw + j]  = 1;

                c = (uchar)CornerScore16(ptr, pixel, aThreshold);

                if(aNonmaxSuppression)
                {
                  curr[j]              = c;
                  apOimg[i * acIw + j] = c;
                }
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }

        if(d & 2)
        {
          int vt;
          int count;

          vt    = v + aThreshold;
          count = 0;

          for(k = 0; k < N; k++)
          {
            int x = ptr[pixel[k]];

            if(x > vt)
            {
              if(++count > K)
              {
                char c;

                cornerpos[ncorners++] = j;
                apOimg[i * acIw + j]  = 1;

                c = (uchar)CornerScore16(ptr, pixel, aThreshold);

                if(aNonmaxSuppression)
                {
                  curr[j]              = c;
                  apOimg[i * acIw + j] = c;
                }
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }
      }
    }
    cornerpos[-1] = ncorners; // error

    if(i == 0)
    {
      continue;
    }

    {
      const uchar *prev  = buf[(i - 1 + 3) % 3];
      const uchar *pprev = buf[(i - 2 + 3) % 3];

      cornerpos = cpbuf[(i - 1 + 3) % 3];
      ncorners  = cornerpos[-1];

      for(k = 0; k < ncorners; k++)
      {
        int score;

        j     = cornerpos[k];
        score = prev[j];

        if(!aNonmaxSuppression ||
           (score > prev[j + 1] && score > prev[j - 1] &&
            score > pprev[j - 1] && score > pprev[j] && score > pprev[j + 1] &&
            score > curr[j - 1] && score > curr[j] && score > curr[j + 1]))
        {
          final_ncorners++;
        }
      }
    }
  }

  // Do non maxima suppression in the output score image
  if(aNonmaxSuppression)
  {
    final_ncorners = Nms3x3(apOimg, acIw, acIh);
  }

  free(iimg_padded_tmp);

  return final_ncorners;
}

namespace ref
{
  FastRef::FastRef()
  {
  }

  FastRef::~FastRef()
  {
  }

  int FastRef::RefFast9(uint8_t *apOimg, const uint8_t *apcIimg, int aThreshold, int aNms, int aW, int aH)
  {
    return RefFast16(apOimg, apcIimg, aW, aH, aThreshold, aNms);
  }
}

int RefFast9(uint8_t *apOimg, const uint8_t *apcIimg, int aThreshold, int aNms, int aW, int aH)
{
  ref::FastRef ref;

  return ref.RefFast9(apOimg, apcIimg, aThreshold, aNms, aW, aH);
}
