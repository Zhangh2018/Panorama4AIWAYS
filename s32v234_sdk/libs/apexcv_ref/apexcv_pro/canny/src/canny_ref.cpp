/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2017 NXP
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
*  \file apexcv_canny_ref.cpp
*  \brief Reference kernels for Canny edge detection algorithm
***********************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include "canny_ref.h"

#define MAX_STACK_SIZE (512*512)

template<typename T>
void pad_image(T* dst, const T* src, int sw, int sh, int pw, int ph)
{
    const T* s = src;
    T* d = dst;
    if (ph < 0) ph = 0;
    if (pw < 0) pw = 0;
    
    int spanw = 2*pw+sw;
    
    if (pw > 0) {
        d += ph*spanw;
        for (int i = 0; i < sh; ++i) {
            for (int j = 0; j < pw; ++j)
                d[j] = s[0];
            
            for (int j = 0; j < sw; ++j)
                d[j+pw] = s[j];
            
            for (int j = 0; j < pw; ++j)
                d[pw + sw + j] = s[sw-1];
                
            d += spanw;
            s += sw;
        }
    } else {
        d += ph*sw;
        for (int i = 0; i < sh; ++i, d+=sw, s+=sw)
            for (int j = 0; j < sw; ++j)
                d[j] = s[j];
    }
    
    if (ph > 0) {
        d = dst;
        s = dst + ph*spanw;
        for (int i = 0; i < ph; ++i) {
            for (int j = 0; j < spanw; ++j) {
                d[j] = s[j];
                
                d[(sh+ph)*spanw+j] = s[(sh-1)*spanw+j];
            }
            d += spanw;
        }
    }
}

template void pad_image(uint8_t* dst, const uint8_t* src, int sw, int sh, int pw, int ph);
template void pad_image(uint16_t* dst, const uint16_t* src, int sw, int sh, int pw, int ph);
template void pad_image_rep(uint16_t* blk, int width, int height, int x_pad, int y_pad);

template<typename T>
void pad_image_rep(T* blk, int width, int height, int x_pad, int y_pad)
{
   T* d = blk+y_pad*(width+x_pad*2);
   T* s = blk+y_pad*(width+x_pad*2)+x_pad;
   int bstr = width + 2*x_pad;
   
   if (x_pad > 0) {
      for (int i = 0; i < height; ++i, d+=bstr) {
         for (int j = 0; j < x_pad; ++j) {
            d[j] = s[0];
            d[j+width+x_pad] = s[width-1];
         }
      }
   }
   if (y_pad > 0) {
      d = blk;
      s = blk + y_pad*(width+x_pad*2);
      for (int i = 0; i < y_pad; ++i, d+=bstr) {
         for (int j = 0; j < width+2*x_pad; ++j) {
            d[j] = s[j];
            d[(y_pad+height)*bstr+j] = s[(height-1)*bstr+j];
         }
      }
   }
}

template void pad_image_rep(uint8_t* blk, int width, int height, int x_pad, int y_pad);

// calculate magnitude and angle of gradient, perform non-maxima suppression,
// and perform hysteresis.
// fill the map with one of the following values:
//   0 - the pixel can not belong to an edge
//   1 - the pixel might belong to an edge
//   2 - the pixel does belong to an edge
void ref_canny_suppress(
               uint8_t* dEdges, int estr,
         const uint16_t* sMag, int mstr,
         const int16_t* dx, int xstr,
         const int16_t* dy, int ystr,
               int bw, int bh,
               uint16_t low, uint16_t high
)
{
   const uint16_t* pm = sMag;
   uint8_t* pe = dEdges;
   const int16_t* px = dx;
   const int16_t* py = dy;
   static int32_t stack[MAX_STACK_SIZE];
   int32_t stack_top = 0;
   
   #define CANNY_SHIFT 15
   const int TG22 = (int32_t)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5);
   
   for (int i = 0; i < bh; ++i, pe+=estr, pm+=mstr,px+=xstr,py+=ystr) {
      bool prev_flag = false;
      for (int j = 0; j < bw; ++j) {
         uint16_t mag = pm[j];
         int32_t xs = (int32_t)px[j];
         int32_t ys = (int32_t)py[j];
         int32_t x = (int32_t)abs(xs);
         int32_t y = ((int32_t)abs(ys)) << CANNY_SHIFT;
         bool add = false;
         
         int32_t tg22x = x*TG22;
         if (mag > low) {
            // 0 degrees
            if (y < tg22x) {
               if (mag > pm[j-1] && mag >= pm[j+1]) add = true;
            } else {
               // 90 degrees
               int32_t tg67x = tg22x + (x << (CANNY_SHIFT+1));
               if (y > tg67x) {
                  if (mag > pm[j-mstr] && mag >= pm[j+mstr]) add = true;
               }
               else {
                  // 135 degrees or 45 degrees
                  if ((xs ^ ys) < 0) {
                     if (mag > pm[j-mstr+1] && mag > pm[j+mstr-1]) {
                        add = true;
                     }
                  } else {
                     if (mag > pm[j-mstr-1] && mag > pm[j+mstr+1]) {
                        add = true;
                     }
                  }
               }
            }
            if (add){
               if (!prev_flag && mag > high && pe[j-estr] != 2) {
                  pe[j] = 2;
                  stack[stack_top] = i*estr+j;
                  stack_top++;
                  prev_flag = true;
               } else {
                  pe[j] = 1;
               }
            } else {
               prev_flag = false;
               pe[j] = 0;
            }
         } else {
            prev_flag = false;
            pe[j] = 0;
         }
      }
   }

   uint8_t p11, p12, p13, p21, p23, p31, p32, p33;
   while(stack_top > 0) {
#define PUSH(idx) { \
                     dEdges[(idx)] = 2; \
                     stack[stack_top] = (idx); \
                     stack_top++; \
                  }
      stack_top -= 1;
      int32_t ind = stack[stack_top];
      uint8_t* m = &dEdges[ind];
      
      if (m[     -1]==1) PUSH(ind     -1)
      if (m[     +1]==1) PUSH(ind     +1)
      if (m[-estr-1]==1) PUSH(ind-estr-1)
      if (m[-estr  ]==1) PUSH(ind-estr  )
      if (m[-estr+1]==1) PUSH(ind-estr+1)
      if (m[+estr-1]==1) PUSH(ind+estr-1)
      if (m[+estr  ]==1) PUSH(ind+estr  )
      if (m[+estr+1]==1) PUSH(ind+estr+1)
#undef PUSH
   }
}

//  -1,  0,  1,
//  -2,  0,  2,
//  -1,  0,  1,
static inline int16_t
ref_sobel_3x3_x(const uint8_t* src, int sstr)
{
   int16_t vd = +   src[-sstr+1] -   src[-sstr-1]
                + 2*src[     +1] - 2*src[     -1]
                +   src[+sstr+1] -   src[+sstr-1]; 
               
               
   return vd;
}

// -1, -2, -1,
//  0,  0,  0,
// +1, +2, +1,
static inline int16_t
ref_sobel_3x3_y(const uint8_t* src, int sstr)
{
   int16_t t7 = + src[+sstr-1] - src[-sstr-1];
   int16_t t8 = + src[+sstr  ] - src[-sstr  ];
   int16_t t9 = + src[+sstr+1] - src[-sstr+1];
   int16_t vd = t7 + 2*t8 + t9;
   
   return vd;
}


void
ref_canny_sobel_3x3_x(
            int16_t* dst, int dstr,
      const uint8_t* src, int sstr,
            int bw, int bh)
{
   int16_t* pDst = dst;
   const uint8_t* pSrc = src;
   for (int y=0; y<bh; ++y, pSrc+=sstr, pDst+=dstr)
      for (int x=0; x<bw; ++x)
         pDst[x] = ref_sobel_3x3_x(&pSrc[x], sstr);
}

void
ref_canny_sobel_3x3_y(
            int16_t* dst, int dstr,
      const uint8_t* src, int sstr,
            int bw, int bh)
{
   int16_t* pDst = dst;
   const uint8_t* pSrc = src;
   for (int y=0; y<bh; ++y, pSrc+=sstr, pDst+=dstr)
      for (int x=0; x<bw; ++x)
         pDst[x] = ref_sobel_3x3_y(&pSrc[x], sstr);
}

void 
ref_canny_magnitude(
               uint16_t* dst,  int dstr,
         const int16_t* srcx, int sstrx,
         const int16_t* srcy, int sstry,
               int bw, int bh )
{
   const int16_t* sx = srcx;
   const int16_t* sy = srcy;

   for (int by = 0; by < bh; by++) {
      for (int bx = 0; bx < bw; bx++) {
         uint16_t ax = (uint16_t)abs(sx[bx]);
         uint16_t ay = (uint16_t)abs(sy[bx]);
         dst[bx] = ax + ay;
      }
      sx += sstrx;
      sy += sstry;
      dst  += dstr;
   }
}      

void ref_canny_create_image(
               uint8_t* dst, int dstr,
         const uint8_t* src, int sstr,
         const uint8_t* edge, int estr,
               int bw, int bh
)
{
   for (int i = 0; i < bh; ++i)
      for (int j = 0; j < bw; ++j)
         // dst[i*dstr+j] = (edge[i*estr+j]>>1)*src[i*sstr+j];
         dst[i*dstr+j] = (edge[i*estr+j]>>1)*255;

}
