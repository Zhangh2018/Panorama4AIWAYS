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
*  \file apu_arithmetic.c
*  \brief APU basic arithmetic kernels.
***********************************************************************************/

#ifdef ACF_KERNEL_IMPLEMENTATION

#include "arithmetic_apu.h"
#include <stdint.h>

/**
* @brief          Centesimal arctangent function with two parameters
* @details        The implementation is based on the CORDIC algorithm.
*                 See magatan2 for more details.
*
* @param[in]      y Fixed point representation of the abscissa [-1, 1)
* @param[in]      x Fixed point representation of the ordinate [-1, 1)
*
* @return         The output is between [-0.5, 0.5], which is the
*                 equivalent of [-PI, PI].
* @note           Accuracy (ULP) -> Mean: 15.90, Worst: 12.97
*                 Latency: ~92 Cycles
*/

inline vfxp_s1q15 vfxp_catan2(vfxp_s1q15 y, vfxp_s1q15 x)
{
  /* Precomputed values for arctan */
  const vfxp_s1q15 V_S1Q15_ONE = 0x7FFF;
  const vfxp_s1q15 V_S1Q15_MINUS_ONE = 0x8000;
  static const uint16_t atanLUT[] = {
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
  -32768 is the vfxp_s1q15 representation of -1.
  It is excluded because its abs cannot be represented on 15bits */
  y = vsat(y, V_S1Q15_MINUS_ONE + 1, V_S1Q15_ONE);
  x = vsat(x, V_S1Q15_MINUS_ONE + 1, V_S1Q15_ONE);

  /* Compute |x|, |y|.
  These are the values to be further used by our CORDIC implementation. */
  vec16u xi = vabs(x);
  vec16s yi = vec16s(vabs(y));

  /* Normalize to as high a magnitude as possible.
  - vclz: counts leading zeroes
  - (-2): see details above about xi, yi representation
  */
  vec16s n = vec16s(vclz(xi | yi)) - 2;

  /* if (n > 0) => shift left by n, else shift right by (-n) */

  xi = vselect(xi << n, xi >> (-n), n > 0);
  yi = vselect(yi << n, yi >> (-n), n > 0);

  vec16s ti = 0;

  /* Main CORDIC loop */
  for (int i = 0; i < 15; i++)
    chess_loop_range(1, )
    chess_unroll_loop(15)
  {
    vbool ccw = (yi > 0);

    vec16u xtemp = (vec16u)vasb(xi, vec16u(vsr(yi, (vec16s)i)), ccw);
    vec16s ytemp = (vec16s)vasb(yi, vec16s((-1) * (xi >> i)), ccw);

    ti = vasb(vec16s(ti), vec16s(atanLUT[i]), ccw);
    xi = xtemp;
    yi = ytemp;
  }

  /* Because we started with |x|, |y| => the computed arctan should be positive.
  Due to precision limitations, ti may be negative when its correct value is close to 0.
  Convert ti to abs(ti) before further computations */
  ti = vec16s(vabs(ti));

  /* ti has the same representation as xi, yi.
  * => shift ti to bring it to the representation unsigned 16bit for decimals. */
  ti >>= 2;

  /* Convert to signed. */
  vfxp_s1q15 angle = (vec16s)ti;

  /* Computed angle is for |x|, |y|
  Adjust the final angle based on the real values of x, y. */
  angle = vselect(0, angle, y == 0);
  angle = vselect(16384 - angle, angle, x < 0);
  angle = vselect(-angle, angle, y < 0);

  return angle;
}

/*! 
  Computes Square root of its input 
  \param a - [Input] Source value
  \return     [Output] Square root of input value
*/
vec32u vsqrt_32(vec32u a)
{
  vec16u ah = (vec16u)(a >> 16);
  vec16u u = 0x4000, r = u + u;
  vec16u tmp;
  vec32u prod;
  vbool  c;
  vec32u res;
  vif(a > 1) {
    for (int i = 6; i >= 0; i--, u >>= 1){
      r = vasb(r, u, vmul_uhuh(r, r) <= ah);
    }

    for (int i = 7; i >= 0; i--, u >>= 1){
      prod = vec32u(vmul_uhuh(r, r), vmul_ulul(r, r));
      tmp = vmul_uhul(r, r);
      vacm(prod, tmp);
      vacm(prod, tmp);
      vsub(a, prod, c);
      r = vasb(r, u, ~c);/* prod <= a    ==     a >= prod, ergo a-prod does not carry. */
      /* r = vasb(r, u, prod <= a); */
    }

    prod = vec32u(vmul_uhuh(r, r), vmul_ulul(r, r));
    tmp = vmul_uhul(r, r);
    vacm(prod, tmp);
    vacm(prod, tmp);

    vsub(a, prod, c);
    res = (vec32u)vsubx(r, (vec16u)0, c);
  } velse{
    res = a;
  } vendif

  return res;
}

/*!
   Calculates the absolute value from one sources
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source value block pointer
   \param sstr  - [Input]  Source value block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_abs(    vec08u* dst, int dstr,
      const vec08s* src, int sstr,
                 int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vabs(src[bx]);
      }
      src += sstr;
      dst += dstr;      
   }                    
}

/*!
   Calculates the absolute difference between two sources
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_absdiff(    vec08u* dst, int dstr,
         const vec08u* src1, int sstr1,
         const vec08u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vabs_diffu((vec16u)src1[bx], (vec16u)src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }                    
}

/*!
   Calculates the absolute difference between two sources
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_absdiff( vec16s* dst, int dstr,
           const vec16s* src1, int sstr1,
           const vec16s* src2, int sstr2,
                 int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)
   {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)
      {
          vec16u lval = (vec16u)(vabs_diff((vec16s)src1[bx], (vec16s)src2[bx]));
          dst[bx] = (vec16s)vselect((vec16u)32767,lval,lval>32767);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;
   }
}

/*!
   Calculates sum of two images
 
   \param apDst   - [Output] Destination block pointer
   \param aDstr   - [Input]  Destination block stride
   \param apcSrc1 - [Input]  Source 1 block pointer
   \param aSstr1  - [Input]  Source 1 block stride
   \param apcSrc2 - [Input]  Source 2 block pointer
   \param aSstr2  - [Input]  Source 2 block stride
   \param aBw     - [Input]  Block width
   \param aBh     - [Input]  Block height
*/
void 
apu_accumulate(vec16s* apDst,  int aDstr,
               const vec08u* apcSrc1,  int aSstr1,
               const vec16s* apcSrc2,  int aSstr2,
               int aBw, int aBh) 
{
   for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {  
        vec32s res = (vec32s)apcSrc1[bx] + (vec32s)apcSrc2[bx];
        res = vselect(res, (vec32s) INT16_MAX, res < INT16_MAX);
        res = vselect(res, (vec32s) INT16_MIN, res > INT16_MIN);
        apDst[bx] = (vec16s)res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }  
}

/*!
   Calculates squared sum of two images
 
   \param apDst    - [Output] Destination block pointer
   \param aDstr    - [Input]  Destination block stride
   \param apcSrc1  - [Input]  Source 1 block pointer
   \param aSstr1   - [Input]  Source 1 block stride
   \param apcSrc2  - [Input]  Source 2 block pointer
   \param aSstr2   - [Input]  Source 2 block stride
   \param aBw      - [Input]  Block width
   \param aBh      - [Input]  Block height
   \param aScale   - [Input]  Image 2 Scale
*/
void 
apu_accumulate_squared(vec16s* apDst,  int aDstr,
               const vec08u* apcSrc1,  int aSstr1,
               const vec16s* apcSrc2,  int aSstr2,
                int aBw, int aBh, uint8_t aScale)
{
   for ( int by = 0; by < aBh; by++ ) chess_loop_range(1,) {
      for ( int bx = 0; bx < aBw; ++bx ) chess_loop_range(1,) {
         vec16u s1i = (vec16u)apcSrc1[bx];
         vec16s s2i = apcSrc2[bx];
         vec32s tx = (vec32s)((s1i*s1i) >> aScale) + (vec32s)s2i;
         tx = vselect(tx, (vec32s) INT16_MAX, tx < INT16_MAX);
         tx = vselect(tx, (vec32s) INT16_MIN, tx > INT16_MIN);
         apDst[bx] = (vec16s)tx;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst  += aDstr;
   }
}

/*!
   Calculates weighted sum of two images
 
   \param apDst   - [Output] Destination block pointer
   \param aDstr   - [Input]  Destination block stride
   \param apcSrc1 - [Input]  Source 1 block pointer
   \param aSstr1  - [Input]  Source 1 block stride
   \param apcSrc2 - [Input]  Source 2 block pointer
   \param aSstr2  - [Input]  Source 2 block stride
   \param aBw     - [Input]  Block width
   \param aBh     - [Input]  Block height
   \param aAlpha  - [Input]  Weight between [0 and 256]
  
   ASSUMPTION: both sources are of same size. Weight is determined by alpha which ranges from [0-256].
               ie. ~70% weight is 179. Accuracy is lost if user of API wants exactly 70% but result is 
               accurate w.r.t alpha.
*/
void 
apu_accumulate_weighted(vec08u* apDst,  int aDstr,
                  const vec08u* apcSrc1, int aSstr1,
                  const vec08u* apcSrc2, int aSstr2,
                  int aBw, int aBh, uint16_t aAlpha) {
   
   // Normally: here would check source sizes and normalize if needed
   for ( int by = 0; by < aBh; by++ ) chess_loop_range(1,) {
   
      for ( int bx = 0; bx < aBw; ++bx ) chess_loop_range(1,) {
         vec16u s1i = (vec16u)apcSrc1[bx] * aAlpha;
         vec16u s2i = (vec16u)apcSrc2[bx] * (256 - aAlpha);
         apDst[bx] = (vec08u)((s1i + s2i + 128) >> 8);
      }
   
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst  += aDstr;
   }
}

/*!
   Adds source 1 to source 2
 
   \param apDst      - [Output] Destination block pointer
   \param aDstr      - [Input]  Destination block stride
   \param apcSrc1    - [Input]  Source 1 values block pointer
   \param aSstr1     - [Input]  Source 1 values block stride
   \param apcSrc2    - [Input]  Source 2 values block pointer
   \param aSstr2     - [Input]  Source 2 values block stride
   \param aBw        - [Input]  Block width
   \param aBh        - [Input]  Block height
   \param aSaturate  - [Input]  Overflow policy
 */
void 
apu_add(        vec08u* apDst, int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec08u* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
   if(aSaturate)
   {
     for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {  
          vec16u tmp0 = ((vec16u) apcSrc1[bx]);
          vec16u tmp1 = ((vec16u) apcSrc2[bx]);
          vec16u sum = (tmp0 + tmp1);
          vec16u sat = vsat(sum, 0, 255);
          apDst[bx] = (vec08u) sat;
        }
        apcSrc1 += aSstr1;
        apcSrc2 += aSstr2;
        apDst += aDstr;
     }
   }
   else
   {  
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {  
          vec16u tmp0 = ((vec16u) apcSrc1[bx]);
          vec16u tmp1 = ((vec16u) apcSrc2[bx]);
          vec16u sum = (tmp0 + tmp1);
          apDst[bx] = (vec08u) sum;
        }
        apcSrc1 += aSstr1;
        apcSrc2 += aSstr2;
        apDst += aDstr;
     }
   }
}

/*!
   Adds source 1 to source 2
 
   \param apDst      - [Output] Destination block pointer
   \param aDstr      - [Input]  Destination block stride
   \param apcSrc1    - [Input]  Source 1 values block pointer
   \param aSstr1     - [Input]  Source 1 values block stride
   \param apcSrc2    - [Input]  Source 2 values block pointer
   \param aSstr2     - [Input]  Source 2 values block stride
   \param aBw        - [Input]  Block width
   \param aBh        - [Input]  Block height
   \param aSaturate  - [Input]  Overflow policy
 */
void 
apu_add(        vec16s* apDst, int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec08u* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {  
        vec16u tmp0 = ((vec16u) apcSrc1[bx]);
        vec16u tmp1 = ((vec16u) apcSrc2[bx]);
        vec16s sum = (vec16s)(tmp0 + tmp1);
        apDst[bx] = sum;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
}

/*!
   Adds source 1 to source 2
 
   \param apDst      - [Output] Destination block pointer
   \param aDstr      - [Input]  Destination block stride
   \param apcSrc1    - [Input]  Source 1 values block pointer
   \param aSstr1     - [Input]  Source 1 values block stride
   \param apcSrc2    - [Input]  Source 2 values block pointer
   \param aSstr2     - [Input]  Source 2 values block stride
   \param aBw        - [Input]  Block width
   \param aBh        - [Input]  Block height
   \param aSaturate  - [Input]  Overflow policy
 */
void 
apu_add(        vec16s* apDst, int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec16s* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
   for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
         vec32s res = (vec32s)apcSrc2[bx] + (vec32s)apcSrc1[bx];
         res = vselect(res, (vec32s)INT16_MAX, res < INT16_MAX);
         res = vselect(res, (vec32s)INT16_MIN, res > INT16_MIN);
         apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
         vec32s res = (vec32s)apcSrc2[bx] + (vec32s)apcSrc1[bx];
         apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
}

/*!
   Adds source 1 to source 2
 
   \param apDst      - [Output] Destination block pointer
   \param aDstr      - [Input]  Destination block stride
   \param apcSrc1    - [Input]  Source 1 values block pointer
   \param aSstr1     - [Input]  Source 1 values block stride
   \param apcSrc2    - [Input]  Source 2 values block pointer
   \param aSstr2     - [Input]  Source 2 values block stride
   \param aBw        - [Input]  Block width
   \param aBh        - [Input]  Block height
   \param aSaturate  - [Input]  Overflow policy
 */
void 
apu_add(        vec16s* apDst, int aDstr,
         const vec16s* apcSrc1, int aSstr1,
         const vec16s* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
   for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
         vec32s res = (vec32s)apcSrc2[bx] + (vec32s)apcSrc1[bx];
         res = vselect(res, (vec32s)INT16_MAX, res < INT16_MAX);
         res = vselect(res, (vec32s)INT16_MIN, res > INT16_MIN);
         apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
         vec32s res = (vec32s)apcSrc2[bx] + (vec32s)apcSrc1[bx];
         apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
}

/*!
   Calculates the bitwise AND from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_and(        vec08u* dst, int dstr,
         const vec08u* src1, int sstr1,
         const vec08u* src2, int sstr2,
                     int bw, int bh) 
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vand(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }   
}                     
                     
/*!
   Calculates the bitwise AND from two sources (unsigned 16 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_and(        vec16u* dst, int dstr,
         const vec16u* src1, int sstr1,
         const vec16u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vand(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }  
}                     

/*!
   Calculates the bitwise AND from two sources (unsigned 32 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_and(        vec32u* dst, int dstr,
         const vec32u* src1, int sstr1,
         const vec32u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vand(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }  
}

/*!
   Counts leading zeros (signed 16 bit)

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source value block pointer
   \param sstr  - [Input]  Source value block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void
apu_clz(       vec08u* dst, int dstr,
               const vec16s* src, int sstr,
               int bw, int bh)
{
    for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {
            dst[bx] = (vec08u)vclz(src[bx]);
        }
        src += sstr;
        dst += dstr;
    }
}

/*!
   Counts leading zeros (unsigned 16 bit)

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source value block pointer
   \param sstr  - [Input]  Source value block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void
apu_clz(       vec08u* dst, int dstr,
               const vec16u* src, int sstr,
               int bw, int bh)
{
    for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {
            dst[bx] = (vec08u)vclz(src[bx]);
        }
        src += sstr;
        dst += dstr;
    }
}

/*!
   Counts leading zeros (signed 8 bit)

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source value block pointer
   \param sstr  - [Input]  Source value block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
*/

void
apu_clz(       vec08u* dst, int dstr,
               const vec08s* src, int sstr,
               int bw, int bh)
{    
    vec08u* src_08u = (vec08u*) src;
    for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {
            dst[bx] = (vec08u)(vclz((vec16s)src_08u[bx]) - 8);
        }
        src_08u += sstr;
        dst += dstr;
    }
    
}
/*!
   Counts leading zeros (unsigned 8 bit)

   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source value block pointer
   \param sstr  - [Input]  Source value block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void
apu_clz(       vec08u* dst, int dstr,
               const vec08u* src, int sstr,
               int bw, int bh)
{
    for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {
            dst[bx] = (vec08u)(vclz((vec16u)src[bx]) - 8);
        }
        src += sstr;
        dst += dstr;
    }
}


/*!
   Calculates magnitude of image matrix
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param srcx  - [Input]  Source x values block pointer
   \param sstrx - [Input]  Source x values block stride
   \param srcy  - [Input]  Source y values block pointer
   \param sstry - [Input]  Source y values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_magnitude( vec16u* dst, int dstr,
        const vec16s* srcx, int sstrx,
        const vec16s* srcy, int sstry,
                    int bw, int bh) {
   const vec16s* sx;
   const vec16s* sy;
   
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      sx = (const vec16s*) srcx;
      sy = (const vec16s*) srcy;
      
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {         
         // Compute magnitude       
         vec32u ax = (vec32u)vabs(sx[bx]);
         vec32u ay = (vec32u)vabs(sy[bx]);
         
         vec32u sx2 = (vec32u)ax * (vec32u)ax;
         vec32u sy2 = (vec32u)ay * (vec32u)ay;
                  
         vec32u s2 = sx2 + sy2;
         
         dst[bx] = (vec16u)vsqrt_32(s2);
      }
      srcx += sstrx;
      srcy += sstry;
      dst  += dstr;      
   }
}

/*!
   Calculates max from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param srcx  - [Input]  Source x values block pointer
   \param sstrx - [Input]  Source x values block stride
   \param srcy  - [Input]  Source y values block pointer
   \param sstry - [Input]  Source y values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_max( vec08u* dst, int dstr,
        const vec08u* srcx, int sstrx,
        const vec08u* srcy, int sstry,
                    int bw, int bh) {
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {         
         // Compute max
         dst[bx] = (vec08u)vselect(srcx[bx], srcy[bx], srcx[bx] > srcy[bx]);
      }
      srcx += sstrx;
      srcy += sstry;
      dst  += dstr;      
   }
}

/*!
   Calculates max from two sources (signed 16 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param srcx  - [Input]  Source x values block pointer
   \param sstrx - [Input]  Source x values block stride
   \param srcy  - [Input]  Source y values block pointer
   \param sstry - [Input]  Source y values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_max( vec16s* dst, int dstr,
        const vec16s* srcx, int sstrx,
        const vec16s* srcy, int sstry,
                    int bw, int bh) {
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {         
         // Compute max
         dst[bx] = (vec16s)vselect(srcx[bx], srcy[bx], srcx[bx] > srcy[bx]);
      }
      srcx += sstrx;
      srcy += sstry;
      dst  += dstr;      
   }
}

/*!
   Calculates min from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param srcx  - [Input]  Source x values block pointer
   \param sstrx - [Input]  Source x values block stride
   \param srcy  - [Input]  Source y values block pointer
   \param sstry - [Input]  Source y values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_min( vec08u* dst, int dstr,
        const vec08u* srcx, int sstrx,
        const vec08u* srcy, int sstry,
                    int bw, int bh) {
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {         
         // Compute min
         dst[bx] = (vec08u)vselect(srcx[bx], srcy[bx], srcx[bx] < srcy[bx]);
      }
      srcx += sstrx;
      srcy += sstry;
      dst  += dstr;      
   }
}

/*!
   Calculates min from two sources (signed 16 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param srcx  - [Input]  Source x values block pointer
   \param sstrx - [Input]  Source x values block stride
   \param srcy  - [Input]  Source y values block pointer
   \param sstry - [Input]  Source y values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_min( vec16s* dst, int dstr,
        const vec16s* srcx, int sstrx,
        const vec16s* srcy, int sstry,
                    int bw, int bh) {
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {         
         // Compute min
         dst[bx] = (vec16s)vselect(srcx[bx], srcy[bx], srcx[bx] < srcy[bx]);
      }
      srcx += sstrx;
      srcy += sstry;
      dst  += dstr;      
   }
}

/*!
   Calculates multiplication of two sources
 
   \param aDst      - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param aSrcx     - [Input]  Source x values block pointer
   \param aSstrx    - [Input]  Source x values block stride
   \param aSrcy     - [Input]  Source y values block pointer
   \param aSstry    - [Input]  Source y values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aScale    - [Input]  Scale value
   \param aSaturate - [Input]  Overflow policy
 */

void 
apu_mul( vec08u* aDst, int aDstr,
        const vec08u* aSrcx, int aSstrx,
        const vec08u* aSrcy, int aSstry,
        int aBw, int aBh, uint8_t aScale, bool aSaturate)
{
  if (aSaturate)
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec16u s1i = (vec16u) aSrcx[bx];
           vec16u s2i = (vec16u) aSrcy[bx];
           vec16u tx = s2i * s1i;
           vec16u temp1 = (vec16u)(tx >> 6);
           vec16u temp2 = (vec16u)(temp1 >> 8);
           vec16u tx_scaled = (temp1 + temp2 + 3) >> 2;
           tx_scaled = vselect(tx_scaled, (vec16u)0, tx_scaled > 0);
           tx_scaled = vselect(tx_scaled, (vec16u)255, tx_scaled < 255);
           aDst[bx] = (vec08u)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec16u s1i = (vec16u) aSrcx[bx];
             vec16u s2i = (vec16u) aSrcy[bx];
             vec16u tx = s2i * s1i;
             vec16u tx_scaled = (vec16u)tx >> aScale;
             tx_scaled = vselect(tx_scaled, (vec16u)0, tx_scaled > 0);
             tx_scaled = vselect(tx_scaled, (vec16u)255, tx_scaled < 255);
             aDst[bx] = (vec08u)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
  else
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec16u s1i = (vec16u) aSrcx[bx];
           vec16u s2i = (vec16u) aSrcy[bx];
           vec16u tx = s2i * s1i;
           vec16u temp1 = (vec16u)(tx >> 6);
           vec16u temp2 = (vec16u)(temp1 >> 8);
           vec16u tx_scaled = (temp1 + temp2 + 3) >> 2;
           aDst[bx] = (vec08u)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec16u s1i = (vec16u) aSrcx[bx];
             vec16u s2i = (vec16u) aSrcy[bx];
             vec16u tx = s2i * s1i;
             vec16u tx_scaled = (vec16u)tx >> aScale;
             aDst[bx] = (vec08u)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
}

/*!
   Calculates multiplication of two sources
 
   \param aDst      - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param aSrcx     - [Input]  Source x values block pointer
   \param aSstrx    - [Input]  Source x values block stride
   \param aSrcy     - [Input]  Source y values block pointer
   \param aSstry    - [Input]  Source y values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aScale    - [Input]  Scale value
   \param aSaturate - [Input]  Overflow policy
 */

void 
apu_mul( vec16s* aDst, int aDstr,
        const vec08u* aSrcx, int aSstrx,
        const vec08u* aSrcy, int aSstry,
        int aBw, int aBh, uint8_t aScale, bool aSaturate)
{
  if (aSaturate)
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (temp1 + temp2 + temp3 + 3) >> 2;
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
      }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = tx >> aScale;
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
    }
  }
  else
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (temp1 + temp2 + temp3 + 3) >> 2;
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
      }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = tx >> aScale;
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
    }
  }
}

/*!
   Calculates multiplication of two sources
 
   \param aDst      - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param aSrcx     - [Input]  Source x values block pointer
   \param aSstrx    - [Input]  Source x values block stride
   \param aSrcy     - [Input]  Source y values block pointer
   \param aSstry    - [Input]  Source y values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aScale    - [Input]  Scale value
   \param aSaturate - [Input]  Overflow policy
 */

void 
apu_mul( vec16s* aDst, int aDstr,
        const vec08u* aSrcx, int aSstrx,
        const vec16s* aSrcy, int aSstry,
        int aBw, int aBh, uint8_t aScale, bool aSaturate)
{
  if (aSaturate)
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (temp1 + temp2 + temp3 + 3) >> 2;
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = (vec32s)tx >> aScale;
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
  else
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (temp1 + temp2 + temp3 + 3) >> 2;
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = tx >> aScale;
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
}

/*!
   Calculates multiplication of two sources
 
   \param aDst      - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param aSrcx     - [Input]  Source x values block pointer
   \param aSstrx    - [Input]  Source x values block stride
   \param aSrcy     - [Input]  Source y values block pointer
   \param aSstry    - [Input]  Source y values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aScale    - [Input]  Scale value
   \param aSaturate - [Input]  Overflow policy
 */

void 
apu_mul( vec16s* aDst, int aDstr,
        const vec16s* aSrcx, int aSstrx,
        const vec16s* aSrcy, int aSstry,
        int aBw, int aBh, uint8_t aScale, bool aSaturate)
{
  if (aSaturate)
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (vec32s)((temp1 + temp2 + temp3 + 3) >> 2);
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
           tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = (vec32s)tx >> aScale;
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MIN, tx_scaled > INT16_MIN);
             tx_scaled = vselect(tx_scaled, (vec32s)INT16_MAX, tx_scaled < INT16_MAX);
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
  else
  {
    if (aScale == 255)
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
        for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
           vec32s s1i = (vec32s) aSrcx[bx];
           vec32s s2i = (vec32s) aSrcy[bx];
           vec32s tx = s2i * s1i;
           vec32s temp1 = (vec32s)(tx >> 6);
           vec32s temp2 = (vec32s)(temp1 >> 8);
           vec32s temp3 = (vec32s)(temp2 >> 8);
           vec32s tx_scaled = (vec32s)((temp1 + temp2 + temp3 + 3) >> 2);
           aDst[bx] = (vec16s)tx_scaled;
        }
        aSrcx += aSstrx;
        aSrcy += aSstry;
        aDst  += aDstr;
     }
    }
    else
    {
      for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
          for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
             vec32s s1i = (vec32s) aSrcx[bx];
             vec32s s2i = (vec32s) aSrcy[bx];
             vec32s tx = s2i * s1i;
             vec32s tx_scaled = tx >> aScale;
             aDst[bx] = (vec16s)tx_scaled;
          }
          aSrcx += aSstrx;
          aSrcy += aSstry;
          aDst  += aDstr;
       }
     }
  }
}

/*!
   Calculates the bitwise NOT of the source.
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src   - [Input]  Source values block pointer
   \param sstr  - [Input]  Source values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_not(       vec08u* dst, int dstr,
         const vec08u* src, int sstr,
                    int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vcomplement(src[bx]);
      }
      src += sstr;
      dst += dstr;      
   }  
}

/*!
   Calculates the bitwise OR from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_or(       vec08u* dst,  int dstr,
        const vec08u* src1, int sstr1,
        const vec08u* src2, int sstr2,
                    int bw, int bh) 
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }   
}                     
                     
/*!
   Calculates the bitwise OR from two sources (unsigned 16 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_or(        vec16u* dst,  int dstr,
         const vec16u* src1, int sstr1,
         const vec16u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }  
}                     

/*!
   Calculates the bitwise OR from two sources (unsigned 32 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_or(        vec32u* dst, int dstr,
        const vec32u* src1, int sstr1,
        const vec32u* src2, int sstr2,
                    int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }  
}

/*!
   Calculates the bitwise PHASE from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_phase(        vec08u* dst, int dstr,
        const vec16s* src1, int sstr1,
        const vec16s* src2, int sstr2,
                    int bw, int bh)
{
  for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {
        const int shift = 7;
        vec16s out = vfxp_catan2(src1[bx], src2[bx]); // call atan2()
        // transform to all positive
        vec16u luAbsOut = (vec16u)vselect(out, -out, out > 0);
        vec16u luOut = vselect((vec16u)out, 32768U - luAbsOut, out > 0);
        // round and shift
        luOut = (luOut + (1 << (shift - 1))) >> shift;
        // handle overflow, when rounding
        luOut = vselect(luOut - 256, luOut, luOut >= 256);
        dst[bx] = (vec08u)luOut; // cast to unsigned char
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;
   }
}

/*!
   Subtracts source 2 from source 1
 
   \param apDst     - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param apcSrc1   - [Input]  Source 1 values block pointer
   \param aSstr1    - [Input]  Source 1 values block stride
   \param apcSrc2   - [Input]  Source 2 values block pointer
   \param aSstr2    - [Input]  Source 2 values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aSaturate - [Input]  Overflow policy
 */
void 
apu_sub(       vec08u* apDst,  int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec08u* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec16s tmp0 = ((vec16s) apcSrc1[bx]);
        vec16s tmp1 = ((vec16s) apcSrc2[bx]);
        vec16s sub = (tmp0 - tmp1);
        vec16s sat = vsat(sub, 0, 255);
        apDst[bx] = (vec08u) sat;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec16s tmp0 = ((vec16s) apcSrc1[bx]);
        vec16s tmp1 = ((vec16s) apcSrc2[bx]);
        vec16s sub = (tmp0 - tmp1);
        apDst[bx] = (vec08u) sub;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
   }
  }
}

/*!
   Subtracts source 2 from source 1
 
   \param apDst     - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param apcSrc1   - [Input]  Source 1 values block pointer
   \param aSstr1    - [Input]  Source 1 values block stride
   \param apcSrc2   - [Input]  Source 2 values block pointer
   \param aSstr2    - [Input]  Source 2 values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aSaturate - [Input]  Overflow policy
 */
void 
apu_sub(       vec16s* apDst,  int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec08u* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
    for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
      vec16s tmp0 = ((vec16s) apcSrc1[bx]);
      vec16s tmp1 = ((vec16s) apcSrc2[bx]);
      vec16s sub = (tmp0 - tmp1);
      apDst[bx] = (vec16s) sub;
    }
    apcSrc1 += aSstr1;
    apcSrc2 += aSstr2;
    apDst += aDstr;
  }
}

/*!
   Subtracts source 2 from source 1
 
   \param apDst     - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param apcSrc1   - [Input]  Source 1 values block pointer
   \param aSstr1    - [Input]  Source 1 values block stride
   \param apcSrc2   - [Input]  Source 2 values block pointer
   \param aSstr2    - [Input]  Source 2 values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aSaturate - [Input]  Overflow policy
 */
void 
apu_sub(       vec16s* apDst,  int aDstr,
         const vec08u* apcSrc1, int aSstr1,
         const vec16s* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        res = vselect(res, (vec32s)INT16_MAX, res < INT16_MAX);
        res = vselect(res, (vec32s)INT16_MIN, res > INT16_MIN);
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
}        

/*!
   Subtracts source 2 from source 1
 
   \param apDst     - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param apcSrc1   - [Input]  Source 1 values block pointer
   \param aSstr1    - [Input]  Source 1 values block stride
   \param apcSrc2   - [Input]  Source 2 values block pointer
   \param aSstr2    - [Input]  Source 2 values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aSaturate - [Input]  Overflow policy
 */
void 
apu_sub(       vec16s* apDst,  int aDstr,
         const vec16s* apcSrc1, int aSstr1,
         const vec16s* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        res = vselect(res, (vec32s)INT16_MAX, res < INT16_MAX);
        res = vselect(res, (vec32s)INT16_MIN, res > INT16_MIN);
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
}

/*!
   Subtracts source 2 from source 1
 
   \param apDst     - [Output] Destination block pointer
   \param aDstr     - [Input]  Destination block stride
   \param apcSrc1   - [Input]  Source 1 values block pointer
   \param aSstr1    - [Input]  Source 1 values block stride
   \param apcSrc2   - [Input]  Source 2 values block pointer
   \param aSstr2    - [Input]  Source 2 values block stride
   \param aBw       - [Input]  Block width
   \param aBh       - [Input]  Block height
   \param aSaturate - [Input]  Overflow policy
 */
void 
apu_sub(       vec16s* apDst,  int aDstr,
         const vec16s* apcSrc1, int aSstr1,
         const vec08u* apcSrc2, int aSstr2,
         int aBw, int aBh, bool aSaturate)
{
  if(aSaturate)
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        res = vselect(res, (vec32s)INT16_MAX, res < INT16_MAX);
        res = vselect(res, (vec32s)INT16_MIN, res > INT16_MIN);
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
  else
  {
    for (int by = 0; by < aBh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < aBw; bx++) chess_loop_range(1,)  {
        vec32s res = (vec32s)apcSrc1[bx] - (vec32s)apcSrc2[bx];
        apDst[bx] = (vec16s) res;
      }
      apcSrc1 += aSstr1;
      apcSrc2 += aSstr2;
      apDst += aDstr;
    }
  }
}

/*!
   Calculates an unsigned 8 bit output by substituting each of the unsigned 8 bit input values using a provided loop-up table.
 
   \param dst    - [Output] Destination block pointer
   \param dstr   - [Input]  Destination block stride
   \param src    - [Input]  Source values block pointer
   \param sstr   - [Input]  Source values block stride
   \param bw     - [Input]  Block width
   \param bh     - [Input]  Block height
   \param lpvLUT - [Input]  Vectorized LUT to be used for stubstitution
 */
void apu_lut_8b( vec08u* dst, int16u  dstr,
           const vec08u* src, int16u  sstr,
                 int16u  bw , int16u  bh,
                 vec08u* lpvLUT)
{
#ifdef APEX2_EMULATE
  for (int y = 0; y < (bh); y++)
  {
    for (int x = 0; x < (bw); x++)
    {
      dst[x] = vload(lpvLUT, src[x]);
    }
    dst += dstr;
    src += sstr;
  }
#else
   for (int y = 0; y < bh; ++y) chess_loop_range(1,)
   {
      vec08u srcData = src[0];
      vec08u dstData = vload(lpvLUT,srcData);
      for(int x = 1; x < bw; ++x) chess_loop_range(1,)
      {
         srcData = src[x];
         dst[x - 1] = dstData;
         dstData = vload(lpvLUT,srcData);
      }
      dst[bw - 1] = dstData;
      src += sstr;
      dst += dstr;
   }
#endif
}


/*!
Calculates an unsigned 8 bit boolean output image based on a threshold with configurable true and false values

\param dst       - [Output] Destination block pointer
\param dstr      - [Input]  Destination block stride
\param src       - [Input]  Source values block pointer
\param sstr      - [Input]  Source values block stride
\param bw        - [Input]  Block width
\param bh        - [Input]  Block height
\param threshold - [Input]  Threshold value
\param val_true  - [Input]  Value for TRUE pixels
\param val_false - [Input]  Value for FALSE fixels

https://www.khronos.org/registry/OpenVX/specs/1.0/html/d3/d1e/group__group__vision__function__threshold.html
255 if src(x,y)>threshold
0   otherwise

Kernel has no dependencies, so data is available as one continuous block of data
*/



void apu_binary_threshold(vec08u* dst, int16u  dstr,
  const vec08u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint8_t threshold)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, src[x] >(vec08u)threshold);
    }
    dst += dstr;
    src += sstr;
  }
}

void apu_binary_threshold(vec08u* dst, int16u  dstr,
  const vec16u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint16_t threshold)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, src[x] >(vec16u)threshold);
    }
    dst += dstr;
    src += sstr;
  }
}

void apu_binary_threshold(vec08u* dst, int16u  dstr,
  const vec32u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint32_t threshold)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, src[x] >(vec32u)threshold);
    }
    dst += dstr;
    src += sstr;
  }
}




/*!
Calculates an unsigned 8 bit boolean output image based on a threshold with configurable true and false values

\param dst       - [Output] Destination block pointer
\param dstr      - [Input]  Destination block stride
\param src       - [Input]  Source values block pointer
\param sstr      - [Input]  Source values block stride
\param bw        - [Input]  Block width
\param bh        - [Input]  Block height
\param val_true  - [Input]  Value for TRUE pixels
\param val_false - [Input]  Value for FALSE fixels
\param threshold_low - [Input]  Low Threshold value
\param threshold2 - [Input]  Hight Threshold value

https://www.khronos.org/registry/OpenVX/specs/1.0/html/d3/d1e/group__group__vision__function__threshold.html
0   if src(x,y)>upper
0   if src(x,y)<lower
255 otherwise
Kernel has no dependencies, so data is available as one continuous block of data
*/


void apu_range_threshold(vec08u* dst, int16u  dstr,
  const vec08u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint8_t threshold_low, uint8_t threshold_high)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      const vec08u vsrc = src[x];
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, ((vsrc >= (vec08u)threshold_low) && (vsrc <= (vec08u)threshold_high)));
    }
    dst += dstr;
    src += sstr;
  }
}

void apu_range_threshold(vec08u* dst, int16u  dstr,
  const vec16u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint16_t threshold_low, uint16_t threshold_high)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      const vec16u vsrc = src[x];
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, ((vsrc >= (vec16u)threshold_low) && (vsrc <= (vec16u)threshold_high)));
    }
    dst += dstr;
    src += sstr;
  }
}

void apu_range_threshold(vec08u* dst, int16u  dstr,
  const vec32u* src, int16u  sstr,
  int16u bw, int16u bh,
  int08u val_true, int08u val_false, uint32_t threshold_low, uint32_t threshold_high)
{
  for (int y = 0; y<bh; ++y) chess_loop_range(1, )
  {
    for (int x = 0; x<bw; ++x) chess_loop_range(1, )
    {
      const vec32u vsrc = src[x];
      dst[x] = (vec08u)vselect((vec08u)val_true, (vec08u)val_false, ((vsrc >= (vec32u)threshold_low) && (vsrc <= (vec32u)threshold_high)));
    }
    dst += dstr;
    src += sstr;
  }
}


/*!
   Calculates the bitwise XOR from two sources (unsigned 8 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_xor(        vec08u* dst, int dstr,
         const vec08u* src1, int sstr1,
         const vec08u* src2, int sstr2,
                     int bw, int bh) 
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = (vec08u)vxor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;
   }
}

/*!
   Calculates the bitwise XOR from two sources (unsigned 16 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_xor(        vec16u* dst, int dstr,
         const vec16u* src1, int sstr1,
         const vec16u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vxor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;      
   }  
}                     

/*!
   Calculates the bitwise XOR from two sources (unsigned 32 bit)
 
   \param dst   - [Output] Destination block pointer
   \param dstr  - [Input]  Destination block stride
   \param src1  - [Input]  Source 1 values block pointer
   \param sstr1 - [Input]  Source 1 values block stride
   \param src2  - [Input]  Source 2 values block pointer
   \param sstr2 - [Input]  Source 2 values block stride
   \param bw    - [Input]  Block width
   \param bh    - [Input]  Block height
 */
void 
apu_xor(        vec32u* dst, int dstr,
         const vec32u* src1, int sstr1,
         const vec32u* src2, int sstr2,
                     int bw, int bh)
{
   for (int by = 0; by < bh; by++) chess_loop_range(1,)  {
      for (int bx = 0; bx < bw; bx++) chess_loop_range(1,)  {  
         dst[bx] = vxor(src1[bx], src2[bx]);
      }
      src1 += sstr1;
      src2 += sstr2;
      dst += dstr;
   }  
}

inline
vec16u
apu_sad_full(
         const vec08u* aTemplate,   const int32_t ts,
         const vec08u* aWindow,     const int32_t ws,
         const int32_t tw, const int32_t th
         )
{
   vec16u s = 0;

   for (int32_t y=0; y<th; ++y)
   {
      for (int32_t x=0; x<tw; ++x)
      {
         vec16s d = (vec16s)(aTemplate[x] - aWindow[x]);
         #if 1
         s += vabs(d);
         #else
         vif (d < 0)
             s -= d;
         velse
             s += d;
         vendif
         #endif
      }
      aTemplate += ts;
      aWindow   += ws;
   }

   return s;
}

/*!
   Calculate minimum SAD & location given a template and a window
 
   \param lpvIn0           - [Input]  Source template block pointer
   \param lStrideIn0       - [Input]  Template block stride
   \param lChunkWidthIn0   - [Input]  Template block width
   \param lChunkHeightIn0  - [Input]  Template block height
   \param lpvIn1           - [Input]  Source window block pointer
   \param lStrideIn1       - [Input]  Window block stride
   \param lChunkWidthIn1   - [Input]  Window block width
   \param lChunkHeightIn1  - [Input]  Window block height
   \param lpvOut0          - [Output] Destination block pointer
   \param lStrideOut0      - [Input]  Destination block stride
   \param lChunkWidthOut0  - [Input]  Template block width
   \param lChunkHeightOut0 - [Input]  Template block height
 */

void
apu_sad (vec08u* lpvIn0, int16_t lStrideIn0, int16_t lChunkWidthIn0, int16_t lChunkHeightIn0,
         vec08u* lpvIn1, int16_t lStrideIn1, int16_t lChunkWidthIn1, int16_t lChunkHeightIn1,
         vec32u* lpvOut0, int16_t lStrideOut0, int16_t lChunkWidthOut0, int16_t lChunkHeightOut0)
{
   //implementation
   vec08u xmin = 0xFF, ymin = 0xFF;
   vec16u smin = 0xFFFF;

   vec16u* aMinSAD = (vec16u*)lpvOut0;

   vec08u* aLocation0 = (((vec08u*)lpvOut0) + 2);
   vec08u* aLocation1 = (((vec08u*)lpvOut0) + 3);


   vec08u* aTemplate = lpvIn0;
   vec08u* aWindow   = lpvIn1;

   const int32_t tw = lChunkWidthIn0;
   const int32_t th = lChunkHeightIn0;
   const int32_t ts = lStrideIn0;

   const int32_t ww = lChunkWidthIn1;
   const int32_t wh = lChunkHeightIn1;
   const int32_t ws = lStrideIn1;
 
   for (int y=0; y<(wh-th+1); ++y) chess_loop_range(1,)
   {
      for (int x=0; x<(ww-tw+1); ++x) chess_loop_range(1,)
      {
         vec16u s = apu_sad_full(aTemplate, ts, aWindow+x+y*ws, ws, tw, th);

         vif (s < smin)
         {
            smin = s;
            xmin = x;
            ymin = y;
         }
         vendif
      } 
   }

   aMinSAD[0]    = smin;
   aLocation0[0] = xmin;
   aLocation1[0] = ymin;
}

#endif /* ACF_KERNEL_IMPLEMENTATION */
