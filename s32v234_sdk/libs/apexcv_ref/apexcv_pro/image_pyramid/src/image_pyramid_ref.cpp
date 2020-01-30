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
*  \file pyramid_apex_ref.cpp
*  \brief Reference kernels for image pyramid creation algorithm
***********************************************************************************/

#include "image_pyramid_ref.h"

static inline short _apply_horizontal_gaus(
   unsigned char *src   // [in] ptr to src image
)
{
   short t0 = (short)src[-2] + (short)src[2];
   short t1 = (short)src[-1] + (short)src[1];
   short t2 = 6*(short)src[0];

   return (t0 + (t1<<2) + t2);
}

static inline short _apply_horizontal_gaus_odd(
   unsigned char *src   // [in] ptr to src image
)
{
   short t0 = (short)src[-1] + (short)src[1];
   short t2 = 6*(short)src[0];

   return (t0 + t2);
}

static inline short _apply_horizontal_gaus_even(
   unsigned char *src   // [in] ptr to src image
)
{
   short t1 = (short)src[0] + (short)src[1];

   return (t1<<2);
}

static inline unsigned char _apply_vertical_gaus(
   short *src,    // [in] ptr to src image
   int sstr      // [in] stride of src image
)
{
   short t0 = src[-2*sstr] + src[2*sstr];
   short t1 = src[-sstr] + src[sstr];
   short t2 = 6*src[0];
   short d = t0 + (t1<<2) + t2;
   d = (((d>>5)+1)>>1)&0xFF;
   
   return (unsigned char)d;   // clamp to [0,255]
}

static inline unsigned char _apply_vertical_gaus_and_reduce(
   short *src,    // [in] ptr to src image
   int sstr       // [in] stride of src image
)
{
   short t0 = src[-2*sstr] + src[2*sstr];
   short t1 = src[-sstr] + src[sstr];
   short t2 = 6*src[0];
   short d = (t0) + (t1<<2) + t2;
   d = (d>>8)&0xFF;
   
   return (unsigned char)d;
}

void horizontal_gaus_and_expand(
   unsigned char *src,  // [in] ptr to src image
   int sstr,         // [in] stride of src image
   short *dst,       // [out] ptr to dst image
   int dstr,         // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
)
{
   short *next = dst + dstr;
   for (int h = 0; h < height; h++)
   {
      for (int w = 0; w < width; w++)
      {
         dst[w<<1] = _apply_horizontal_gaus_odd(&src[w]);
         dst[(w<<1)+1] = _apply_horizontal_gaus_even(&src[w]);
         next[w<<1] = 0;
         next[(w<<1)+1] = 0;
      }
      src += sstr;
      dst += dstr<<1;
      next += dstr<<1;
   }
}

void vertical_gaus(
   short *src,       // [in] ptr to src image
   int sstr,         // [in] stride of src image
   unsigned char *dst,  // [out] ptr to dst image
   int dstr,         // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
)
{
   for (int h = 0; h < height; h++)
   {
      for (int w = 0; w < width; w++)
      {
         dst[w] = _apply_vertical_gaus(&src[w], sstr);
      }
      src += sstr;
      dst += dstr;
   }
}

void horizontal_gaus(
   unsigned char *src,  // [in] ptr to src image
   int sstr,         // [in] stride of src image
   short *dst,       // [out] ptr to dst image
   int dstr,         // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
)
{
   for (int h = 0; h < height; h++)
   {
      for (int w = 0; w < width; w++)
      {
         dst[w] = _apply_horizontal_gaus(&src[w]);
      }
      src += sstr;
      dst += dstr;
   }
}

void vertical_gaus_and_reduce(
   short *src,       // [in] ptr to src image
   int sstr,         // [in] stride of src image
   unsigned char *dst,  // [out] ptr to dst image
   int dstr,         // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
)
{
   for (int h = 0; h < height; h++)
   {
      for (int w = 0; w < width; w++)
      {
         dst[w] = _apply_vertical_gaus_and_reduce(&src[w<<1], sstr);
      }
      src += sstr<<1;
      dst += dstr;
   }
}
