/**************************************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2018, NXP Semiconductor
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
 **************************************************************************************************/

#ifndef VFXP_MATH_INLINE_H__
#define VFXP_MATH_INLINE_H__
/**
* @file vfxp_math_inline.h
*/

/**
* \defgroup UserAPI
* This is the group of enum, structure and functions needs to be exposed to APEX MATH library user
*/

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "vfxp_types.h"
#include "math.h"
#define ALWAYS_INLINE static inline

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/


/**
* @ingroup        UserAPI
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
*/

ALWAYS_INLINE vfxp_s1q15 vfxp_catan2(vfxp_s1q15 y, vfxp_s1q15 x)
{
  const vfxp_s1q15 V_S1Q15_ONE = 0x7FFF;
  const vfxp_s1q15 V_S1Q15_MINUS_ONE = 0x8000;

  /* Precomputed values for arctan */
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
#if __chess__
  xi = vssll(xi, n);
  yi = vssll(yi, n);
#else
  xi = vselect(xi << n, xi >> (-n), n > 0);
  yi = vselect(yi << n, yi >> (-n), n > 0);
#endif

  vec16s ti = 0;

  /* Main CORDIC loop */
  for (int i = 0; i < 15; i++)
    chess_loop_range(1, )
    chess_unroll_loop(15)
  {
    vbool ccw = (yi > 0);

    vec16u xtemp = (vec16u)vasb(xi, vec16u(vsr(yi, i)), ccw);
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

/**
* @ingroup        UserAPI
* @brief          Fractional multiplication with scale
* @details        32-bit multiplication with scaling
*
* @param[in]      a Fixed point value
* @param[in]      b Fixed point value
* @param[in]      scale Const Integer
*
* @return         out = (a * b) >> scale
*/

ALWAYS_INLINE vec16s vfxp_qmult_scale(vec16s a, vec16s b, const int scale)
{
  vec16s dh;
  vec16u dl;

  vmul(dh, dl, a, b);

  vec32s d = vec32s(dh, dl);

  return (vec16s)(d >> scale);
}

/**
* @ingroup        UserAPI
* @brief          Fractional multiplication with scale
* @details        32-bit multiplication with scaling
*
* @param[in]      a Fixed point value
* @param[in]      b Fixed point value
* @param[in]      scale Fixed point value
*
* @return         out = (a * b) >> scale
*/

ALWAYS_INLINE vec16s vfxp_qmult_vscale(vec16s a, vec16s b, vec16s scale)
{
  vec16s dh;
  vec16u dl;

  vmul(dh, dl, a, b);

  vec32s d = vec32s(dh, dl);

  return (vec16s)(d >> (vec32s)scale);
}

/**
* @ingroup        UserAPI
* @brief          Fractional multiplication with scale and round
* @details        32-bit multiplication with scaling and rounding
*
* @param[in]      a Fixed point value
* @param[in]      b Fixed point value
* @param[in]      scale Const Integer
*
* @return         out = ((a * b) + (1 << scale - 1)) >> scale
*/

ALWAYS_INLINE vec16s vfxp_qmult_scale_round(vec16s a, vec16s b, const int scale)
{
  vec16s dh;
  vec16u dl;

  vmul(dh, dl, a, b);

  vec32s d = vec32s(dh, dl) + (1 << (scale - 1));

  return (vec16s)(d >> scale);
}



/**
* @ingroup        UserAPI
* @brief          Signum
* @details        Extracts the sign information from the input variable
*                 (a == 0 ? 0 : (a < 0 ? -1 : 1))
* @param[in]      a Fixed point value
*
* @return         -1, 0, 1
* @note
*/

ALWAYS_INLINE vec16s vfxp_sgn(vec16s a)
{
    return vsr(a, 15) | vsrl(-a, 15);
}

/**
* @ingroup        UserAPI
* @brief          Inverse square root
* @details        The approximation method is based on Newton-Rhapson, which
*                 refines an initial estimate with each step of the algorithm
*                 xInit is calculated using a second order polynomial:
*                 x0(x) = a.* x.^2 + b.*x + c; where a, b, c are calculated using
*                 the least - square polynomial curve fitting technique:
*                 a = round(0.812139973184789 * 2^Q);
*                 b = round(-2.02427236492453 * 2^Q);
*                 c = round(2.21607718374657 * 2^Q);
*                 Newton-Raphson step: x(n+1) = x(n) + x(n)*(1 - in * x * x) / 2
*
* @param[in]      x Fixed point representation of the interval [0.5, 1)
*
* @return         The output is between (1, sqrt(2)] - Q2.14
* @note           Accuracy (ULP) -> Mean: 15.41, Worst: 13.51
*/

ALWAYS_INLINE vfxp_s2q14 vfxp_rsqrt(vfxp_s1q15 in)
{
  const vfxp_u1q15 V_U1Q15_ONE = 0x8000;
  /* Input format */
  const int16_t inQRep = 15;
  const int16_t initEstQFormat = 13;
  const int16_t newRhapQFormat = 14;

  /* Constants */
  /* scaleIn = 1 << inQRep; scaleInitEst = 1 << initEstQFormat; scaleInitEst = 1 << newRhapQFormat */
  const int16_t a = (int16_t)( 27123); /*  27123 == (rfxp_s1q15)round(0.827716910305256 * scaleIn) */
  const int16_t b = (int16_t)(-16758); /* -16758 == (rfxp_s3q13)round(-2.04562033773103 * scaleInitEst); */
  const int16_t c = (int16_t)( 18210); /*  18210 == (rfxp_s3q13)round(2.22295332804143 * scaleInitEst); */

  /* Approximate 1/sqrt(x) => a*x^2 + b*x + c then apply one step of Newton - Rhapson to increase precision */
  /* Initial estimate */
  vfxp_s1q15 in2 = vfxp_qmult_scale(in, in, inQRep);
  vfxp_s3q13 xInit = vfxp_qmult_scale(a, in2, inQRep + (inQRep - initEstQFormat)) + vfxp_qmult_scale(b, in, inQRep)+ c;
  vfxp_s3q13 xn2 = vfxp_qmult_scale(xInit, xInit, initEstQFormat);
  vfxp_s1q15 t = (vfxp_s1q15)(V_U1Q15_ONE - (vfxp_u1q15)vfxp_qmult_scale(in, xn2, initEstQFormat)); /* Compute: t = (1 - in * x * x) */
  vfxp_s1q15 tmp0 = vfxp_qmult_scale_round(xInit, t, inQRep - (newRhapQFormat - initEstQFormat)); /* multiply x(n) and t and scale the result to newRhapQFormat (Q2.14) */
  return ((tmp0 >> 1) + (xInit << (newRhapQFormat - initEstQFormat))); /* right shift tmp0 to divide the previous result by 2 according to formula and left shift for xInit to change from Q3.13 to Q2.14. Add the 2 values to obtain rsqrt(2) */
}

/**
* @ingroup        UserAPI
* @brief          Square root
* @details        The algorithm calculates sqrt(x) by using the following formula: sqrt(x) = x / sqrt(x).
*                 X is between [0,1) so a scaling is needed to place the value inside [0.5, 1).
*                 The scaled value will be sent to vfxp_rsqrt(), the results is then multiplied by x
*                 and the scale is compensated.
*                 The result of rsqrt will be weighted by sqrt(2) in case the scale is odd and rescaled by
*                 (scale - 1) / 2 if the scale is odd and scale / 2 if the scale is even.
*
* @param[in]      x Fixed point representation of the interval [0, 1)
*
* @return         The output is between [0, 1) - Q1.15
* @note           Accuracy (ULP) -> Mean: 15.52, Worst: 13.32
*/

ALWAYS_INLINE vfxp_s1q15 vfxp_sqrt(vfxp_s1q15 in)
{
  const vfxp_s2q14 V_S2Q14_SQRT_OF_TWO = 0x5A82;
  const vfxp_s2q14 V_S2Q14_ONE = 0x4000;
  /* Input format */
  const uint16_t inQRep = 15;
  const uint16_t outRSqrtQRep = 14;

  vec16s scale = vclz(in) - 1; // "-1" removes the sign bit
  vec16s scaleOut;
  vfxp_s1q15 inScl = in << scale;
  vfxp_s2q14 invSqrt = vfxp_rsqrt(inScl);
  vbool cond = (vbool)(scale & 1); // check if scale is odd or even

  /* If scale is odd we need to substract 1 from it before dividing by 2 and multiply the x /sqrt(x) with sqrt(2)  */
  vec16s weight = vselect((vec16s)(V_S2Q14_SQRT_OF_TWO), (vec16s)(V_S2Q14_ONE), cond);
  scaleOut = (vselect((vec16s)(scale - 1), (vec16s)(scale), cond)) >> 1;
  invSqrt = vfxp_qmult_scale(invSqrt, weight, (outRSqrtQRep));

  vfxp_s1q15 sqrt = vfxp_qmult_vscale(invSqrt, in, (outRSqrtQRep - scaleOut));
  return sqrt;
}

/**
* @ingroup        UserAPI
* @brief          Integer Square Root
* @details        Uses binary search to determine the square root of the given value. A trial
*                 multiplication is made whose result is compared to the argument; Depending
*                 on if it is greater or lesser, the binary search takes the upper or lower
*                 branch
* @param[in]      x Integer value
*
* @return         floor(sqrt(x))
* @note           Integer square root doesn't have an accuracy class
*/

ALWAYS_INLINE vfxp_u32q0 vfxp_isqrt32(vfxp_u32q0 x)
{
    vec16u ah = (vec16u)(x >> 16);
    vec16u u = 0x4000, r = u+u;
    vec16u tmp;
    vec32u prod;
    vbool  c;

    for(int i = 6;i >= 0;i--, u >>= 1)
    {
        r = vasb(r, u, vmul_uhuh(r,r) <= ah);
    }

    for(int i = 7;i >= 0;i--, u >>= 1)
    {
        prod = vec32u(vmul_uhuh(r,r), vmul_ulul(r,r));
        tmp = vmul_uhul(r,r);
        vacm(prod, tmp);
        vacm(prod, tmp);
        vsub(x, prod, c);
        r = vasb(r, u, ~c);
        /* prod <= a    ==     a >= prod, ergo a-prod does not carry. */
        /* r = vasb(r, u, prod <= a); */
    }

    prod = vec32u(vmul_uhuh(r,r), vmul_ulul(r,r));
    tmp = vmul_uhul(r,r);
    vacm(prod, tmp);
    vacm(prod, tmp);

    vsub(x, prod, c);
    return (vec32u)vsubx(r, 0, c);
}


/**
* @ingroup        UserAPI
* @brief          Tangent
* @details        Polynomial approximation using the Least square polynomial approximation,
*                 the degree is 7 and to speed up the processing a Horner scheme is applied.
*
* @param[in]      x Fixed point representation in RADIANS of the interval [0, pi/4)
*
* @return         The output is between [0, 1) - Q1.15
* @note           Accuracy (ULP) -> Mean: 13.87, Worst: 12.56
*/

ALWAYS_INLINE vfxp_s1q15 vfxp_tan(vfxp_s1q15 x)
{
  // Input format
  const int inQRep = 15;
  const int outQRep = 15;

  // Constants
  const vfxp_s1q15 a = 12611; //(vfxp_s1q15)roundf(0.384842740396587 * scaleIn);
  const vfxp_s1q15 b = -14306; //(vfxp_s1q15)roundf(-0.436574366565305 * scaleIn);
  const vfxp_s1q15 c = 12014; //(vfxp_s1q15)roundf(0.36665178792031 * scaleIn);
  const vfxp_s1q15 d = 7110; //(vfxp_s1q15)roundf(0.216981565670855 * scaleIn);
  const vfxp_s1q15 e = 580; //(vfxp_s1q15)roundf(0.017710495438575 * scaleIn);
  const vfxp_s1q15 f = 32732; //(vfxp_s1q15)roundf(0.998911414343721 * scaleIn);

  // Horner decomposition
  vfxp_s1q15 tmp0 = (vfxp_s1q15)(vfxp_qmult_scale(   a, x, inQRep) + b);    //(((int32_t)(a * x)) >> inQRep)     + b;
  vfxp_s1q15 tmp1 = (vfxp_s1q15)(vfxp_qmult_scale(tmp0, x, inQRep) + c);    //(((int32_t)(tmp0 * x)) >> inQRep) + c;
  vfxp_s1q15 tmp2 = (vfxp_s1q15)(vfxp_qmult_scale(tmp1, x, inQRep) + d);    //(((int32_t)(tmp1 * x)) >> inQRep) + d;
  vfxp_s1q15 tmp3 = (vfxp_s1q15)(vfxp_qmult_scale(tmp2, x, inQRep) + e);    //(((int32_t)(tmp2 * x)) >> inQRep) + e;
  vfxp_s1q15 tmp4 = (vfxp_s1q15)(vfxp_qmult_scale(tmp3, x, inQRep));    //(((int32_t)(tmp3 * x)) >> inQRep);
  vfxp_s1q15 tmp5 = (vfxp_s1q15)(vfxp_qmult_scale(tmp4, x, inQRep));    //(((int32_t)(tmp4 * x)) >> inQRep);
  vfxp_s1q15 tmp6 = (vfxp_s1q15)(vfxp_qmult_scale(   f, x, inQRep));    //(((int32_t)(f * x)) >> inQRep);
  vfxp_s1q15 tmp7 = tmp5 + tmp6;
  return tmp7;
}

/**
* @ingroup        UserAPI
* @brief          ArcSinus
* @details        The approximation is take from: "The Handbook Of Mathematical Functions"
*                 Chapter 4.4.46.
*                 arcsin(x) = pi/2 - sqrt(1 - x) * poly(x)
*                 poly(x) = a*x^7 + b*x^6 + c*x^5 + d*x^4 + e*x^3 + f*x^2 + g*x^1 + h
*                 a =  0.0012624911
*                 b =  0.0066700901
*                 c = -0.0170881256
*                 d =  0.0308918810
*                 e = -0.0501743046
*                 f =  0.0889789874
*                 g = -0.2145988016
*                 h =  1.5707963050
*
* @param[in]      x Fixed point representation of the interval [-1, 1)
*
* @return         The output is between [-pi/2, pi/2) - Q2.14
* @note           Accuracy (ULP) -> Mean: 13.31, Worst: 11.23
*/

ALWAYS_INLINE vfxp_s2q14 vfxp_asin(vfxp_s1q15 x)
{
    const vfxp_s2q14 V_S2Q14_PI_DIV_2 = 0x6488;
    const vfxp_s1q15 V_S1Q15_ONE = 0x7FFF;
    const vfxp_s1q15 V_S1Q15_MINUS_ONE = 0x8000;
    // Input format
    const int16_t inQRep = 15;
    const int16_t outQRep = 14;

    vec16s sgn = vfxp_sgn(x);
    x = (vec16s)vabs(x);

    // Constants
    // a = roundf(0.0012624911 * pow(2.0, inQRep));
    // b = roundf(0.0066700901 * pow(2.0, inQRep));
    // c = roundf(-0.0170881256 * pow(2.0, inQRep));
    // d = roundf(0.0308918810 * pow(2.0, inQRep));
    // e = roundf(-0.0501743046 * pow(2.0, inQRep));
    // f = roundf(0.0889789874 * pow(2.0, inQRep));
    // g = roundf(-0.2145988016 * pow(2.0, inQRep));
    // h = roundf(1.5707963050 * pow(2.0, outQRep));
    const vfxp_s1q15 a = 41;
    const vfxp_s1q15 b = 219;
    const vfxp_s1q15 c = -560;
    const vfxp_s1q15 d = 1012;
    const vfxp_s1q15 e = -1644;
    const vfxp_s1q15 f = 2916;
    const vfxp_s1q15 g = -7032;
    const vfxp_s1q15 h = 25736;

    // Mirrored X
    // To ensure correct results, the order of operations
    // needs to be preserved as indicated by the parantheses.
    vfxp_s1q15 mirror = ((V_S1Q15_ONE - x) + 1);
    vfxp_s1q15 weight = vfxp_sqrt(mirror);

    // Horner decomposition
    vfxp_s1q15 tmp0 = vfxp_qmult_scale(a, x, inQRep) + b;
    vfxp_s1q15 tmp1 = vfxp_qmult_scale(tmp0, x, inQRep) + c;
    vfxp_s1q15 tmp2 = vfxp_qmult_scale(tmp1, x, inQRep) + d;
    vfxp_s1q15 tmp3 = vfxp_qmult_scale(tmp2, x, inQRep) + e;
    vfxp_s1q15 tmp4 = vfxp_qmult_scale(tmp3, x, inQRep) + f;
    vfxp_s1q15 tmp5 = vfxp_qmult_scale(tmp4, x, inQRep) + g;
    vfxp_s2q14 tmp6 = vfxp_qmult_scale(tmp5, x, inQRep + (inQRep - outQRep)) + h;
    vfxp_s1q15 tmp7 = vfxp_qmult_scale_round(tmp6, weight, inQRep);
    vfxp_s2q14 out = (V_S2Q14_PI_DIV_2 - tmp7) * sgn;

    // out = (-1 == x) ? -PI/2 : out;
    out = vselect(-V_S2Q14_PI_DIV_2, out, (V_S1Q15_MINUS_ONE == x));

    return out;
}

/**
* @ingroup        UserAPI
* @brief          ArcCosinus
* @details        vfxp_acos(x) = V_S2Q14_PI_DIV_2 - vfxp_asin(x)
* @param[in]      x Fixed point representation of the interval [-1, 1)
*
* @return         The output is between [pi, 0) - Q3.13
* @note           Accuracy (ULP) -> Mean: 13.18, Worst: 11.03
*/

ALWAYS_INLINE vfxp_s3q13 vfxp_acos(vfxp_s1q15 x)
{
    const vfxp_s2q14 V_S3Q13_PI_DIV_2 = 0x3244;
    vfxp_s2q14 s = vfxp_asin(x);
    return (V_S3Q13_PI_DIV_2 - (s >> 1));
}

/**
* @ingroup        UserAPI
* @brief          ArcTangent
* @details        The functions takes advantage of the arctangent's property of odd symmetry.
*                 An absolute value is applied on the input data, the value is then used inside
*                 a high precision 5-th degree polynomial specially optimized for [0, 1]
*                 absX = abs(x)
*                 return poly(absX)
*                 poly(x) = a*x^5 + b*x^4 + c*x^3 + d*x^2 + e*x + f
*                 a = -0.0542819708651922
*                 b =  0.277205766796652
*                 c = -0.463942427831086
*                 d =  0.0290088104185372
*                 e =  0.997378122323713
*                 f =  5.73559304367514e-05
*
* @param[in]      x Fixed point representation of the interval [-1, 1)
*
* @return         The output is between [-1, 1) - Q1.15
* @note           Accuracy (ULP) -> Mean: 15.56, Worst: 13.20
*/

ALWAYS_INLINE vfxp_s1q15 vfxp_atan(vfxp_s1q15 x)
{
    const vfxp_s2q14 V_S1Q15_PI_DIV_4 = 0x6488;
    const vfxp_s1q15 V_S1Q15_MINUS_ONE = 0x8000;
    // Input format
    const int inQRep = 15;

    // Constants
    /*
     * a = round(-0.0542819708651922 * (2^inQRep));
     * b = round(0.277205766796652 * (2^inQRep));
     * c = round(-0.463942427831086 * (2^inQRep));
     * d = round(0.0290088104185372 * (2^inQRep));
     * e = round(0.997378122323713 * (2^inQRep));
     * f = round(5.73559304367514e-05 * (2^inQRep));
     */
    const int16_t a = (int16_t)(-1779);
    const int16_t b = (int16_t)(9083);
    const int16_t c = (int16_t)(-15202);
    const int16_t d = (int16_t)(951);
    const int16_t e = (int16_t)(32682);
    const int16_t f = (int16_t)(2);

    // Use tan simmetry
    vfxp_s1q15 sgn = vfxp_sgn(x);
    x = (vec16s)vabs(x);

    // Horner decomposition
    vfxp_s1q15 tmp0 = (vfxp_s1q15)vfxp_qmult_scale(a, x, inQRep) + b;
    vfxp_s1q15 tmp1 = (vfxp_s1q15)vfxp_qmult_scale(tmp0, x, inQRep) + c;
    vfxp_s1q15 tmp2 = (vfxp_s1q15)vfxp_qmult_scale(tmp1, x, inQRep) + d;
    vfxp_s1q15 tmp3 = (vfxp_s1q15)vfxp_qmult_scale(tmp2, x, inQRep) + e;
    vfxp_s1q15 tmp4 = (vfxp_s1q15)vfxp_qmult_scale_round(tmp3, x, inQRep) + f;

    // atan(0) must be 0
    // vfxp_sgn(0) = 0 => tmp4 * sgn = 0
    tmp4 = tmp4 * sgn;

    // tmp4 = (-1 == x) ? -PI/4 : tmp4
    tmp4 = vselect(-V_S1Q15_PI_DIV_4, tmp4, V_S1Q15_MINUS_ONE == x);

    return tmp4;
}

/**
* @ingroup        UserAPI
* @brief          Arctangent function with two parameters
* @details        The function uses catan2.
*
* @param[in]      y Fixed point representation of the abscissa [-1, 1)
* @param[in]      x Fixed point representation of the ordinate [-1, 1)
*
* @return         The output is between [-pi, pi] - Q3.13
* @note           Accuracy (ULP) -> Mean: 13.18, Worst: 10.28
*/

ALWAYS_INLINE vfxp_s3q13 vfxp_atan2(vfxp_s1q15 y, vfxp_s1q15 x)
{
  const vfxp_s3q13 V_S3Q13_PI = 0x6488;
  return vfxp_qmult_scale_round(vfxp_catan2(y, x), V_S3Q13_PI, 14);
}

/**
* @ingroup        UserAPI
* @brief          Hyperbolic Sinus
* @details        Polynomial approximation using the famous Taylor decomposition,
*                 the degree is 7 and to speed up the processing a Horner scheme is applied.
*                 Chebyshev approximations can be used also but the coefficient values
*                 are very big in comparison with the input data format. Big issue with precision !
* @param[in]      x Fixed point representation of the interval [0, 1)
*
* @return         The output is between [0, 1.18) - Q2.14
* @note           Accuracy (ULP) -> Mean: 14.16, Worst: 12.52
*/

ALWAYS_INLINE vfxp_s2q14 vfxp_sinh(vfxp_s1q15 x)
{
  const vfxp_s2q14 V_S2Q14_ONE = 0x4000;
  // Input format
  const int inQRep = 15;
  const int intermedQRep = 14;

  // Constants
  const vfxp_s1q15 a = 5461;//(vfxp_s1q15)roundf(1.0f / 6.0f * scaleIn);
  const vfxp_s1q15 b = 273;//(vfxp_s1q15)roundf(1.0f / 120.0f * scaleIn);
  const vfxp_s1q15 c = 7;//(vfxp_s1q15)roundf(1.0f / 5040.0f * scaleIn);

  // Horner decomposition - precalculate x^2
  // out = x .* (1 + x.^2 * (1/factorial(3) + x.^2 .* (1/factorial(5) + 1/factorial(7) .* x.^2)))
  vfxp_s1q15  x2 = vfxp_qmult_scale(x, x, inQRep);//(vfxp_s1q15)(((int32_t)(x * x)) >> inQRep);

  // Horner cont'd
  vfxp_s1q15 tmp0 = (vfxp_s1q15)(vfxp_qmult_scale(c, x2, inQRep) + b); //(vfxp_s1q15)(((int32_t)(c * x2)) >> inQRep) + b;
  vfxp_s1q15 tmp1 = (vfxp_s1q15)(vfxp_qmult_scale(tmp0, x2, inQRep) + a); //(((int32_t)(tmp0 * x2)) >> inQRep) + a;
  vfxp_s1q15 tmp2 = (vfxp_s1q15)(vfxp_qmult_scale(tmp1, x2, inQRep)); //(((int32_t)(tmp1 * x2)) >> inQRep);

  // Convert from inQRep to intermedQRep
  vfxp_s1q15 tmp3 = (vfxp_s2q14)(tmp2 >> (inQRep - intermedQRep)) + V_S2Q14_ONE;

  // Convert from intermedQRep to outQRep
  vfxp_s2q14 tmp4 = (vfxp_s2q14)(vfxp_qmult_scale(tmp3, x, inQRep)); //(((int32_t)(tmp3 * x)) >> inQRep);

  return tmp4;
}

/**
* @ingroup        UserAPI
* @brief          Hyperbolic Cosine
* @details        Polynomial approximation using the famous Taylor decomposition,
*                 the degree is 7 and to speed up the processing a Horner scheme is applied.
*                 Chebyshev approximations can be used also but the coefficient values
*                 are very big in comparison with the input data format. Big issue with precision !
* @param[in]      x Fixed point representation of the interval [0, 1)
*
* @return         The output is between [1, 1.54) - Q2.14
* @note           Accuracy (ULP) -> Mean: 14.33, Worst: 12.82
*/

ALWAYS_INLINE vfxp_s2q14 vfxp_cosh(vfxp_s1q15 x)
{
  const vfxp_s2q14 V_S2Q14_ONE = 0x4000;
  /* Input format */
  const int inQRep = 15;
  const int intermedQRep = 14;

  // Constants
  const vfxp_s1q15 a = 16384;    //(vfxp_s1q15)roundf(1.0f / 2.0f * scaleIn);
  const vfxp_s1q15 b = 1365;     //(vfxp_s1q15)roundf(1.0f / 24.0f * scaleIn);
  const vfxp_s1q15 c = 46;       //(vfxp_s1q15)roundf(1.0f / 720.0f * scaleIn);

  vfxp_s1q15 x2 = (vfxp_s1q15)(vfxp_qmult_scale(x, x, inQRep));

  vfxp_s1q15 tmp0 = (vfxp_s1q15)(vfxp_qmult_scale(c, x2, inQRep) + b);
  vfxp_s1q15 tmp1 = (vfxp_s1q15)(vfxp_qmult_scale(tmp0, x2, inQRep) + a); //(((int32_t)(tmp0 * x2)) >> inQRep) + a;
  vfxp_s1q15 tmp2 = (vfxp_s1q15)(vfxp_qmult_scale(tmp1, x2, inQRep)); //(((int32_t)(tmp1 * x2)) >> inQRep);

  // Convert from inQRep to intermedQRep
  vfxp_s1q15 tmp3 = (vfxp_s2q14)(tmp2 >> (inQRep - intermedQRep)) + V_S2Q14_ONE;
  return tmp3;
}

/**
* @ingroup        UserAPI
* @brief          Hyperbolic Tangent
* @details        Polynomial approximation using a 7th degree best fit polynomial,
*                 and to speed up the processing a Horner scheme is applied.
*                 The function uses the fact that tanh(x) has an odd symmetry, this means
*                 that the code calculates the tanh(x) over
* @param[in]      x Fixed point representation of the interval [-1, 1)- Q1.15
*
* @return         The output is between [-0.76159, 0.76159) - Q1.15
* @note           Accuracy (ULP) -> Mean: 15.96, Worst: 13.62
*/

ALWAYS_INLINE vfxp_s1q15 vfxp_tanh(vfxp_s1q15 x)
{
  const int inQRep = 15;
  const vfxp_s1q15 a6 = -1521;  //(rfxp_s1q15)roundf(-0.0464306998420377 * scaleIn);
  const vfxp_s1q15 a5 = 4002;   //(rfxp_s1q15)roundf(0.122120148708601 * scaleIn);
  const vfxp_s1q15 a4 = 1078;   //(rfxp_s1q15)roundf(0.0329111298824921  * scaleIn);
  const vfxp_s1q15 a3 = -11480; //(rfxp_s1q15)roundf(-0.350335160459025 * scaleIn);
  const vfxp_s1q15 a2 = 118;    //(rfxp_s1q15)roundf(0.00361015979071007 * scaleIn);
  const vfxp_s1q15 a1 = 32758;  //(rfxp_s1q15)roundf(0.999706068613686 * scaleIn);
  const vfxp_s1q15 a0 = 0;      //(rfxp_s1q15)roundf(5.5727405468594e-06 * scaleIn);

  vec16s sgn = vfxp_sgn(x);
  x = (vec16s)vabs(x);

  x = vselect(0x7FFF, x, (x < 0));

  vfxp_s1q15 horner = a6;
  horner = vfxp_qmult_scale_round(horner, x, inQRep) + a5;
  horner = vfxp_qmult_scale_round(horner, x, inQRep) + a4;
  horner = vfxp_qmult_scale_round(horner, x, inQRep) + a3;
  horner = vfxp_qmult_scale_round(horner, x, inQRep) + a2;
  horner = vfxp_qmult_scale_round(horner, x, inQRep) + a1;
  horner = vfxp_qmult_scale_round(horner, x, inQRep);

  return horner * sgn;
}

/**
* @ingroup        UserAPI
* @brief          Natural logarithm
* @details        Polynomial approximation using best fit polynomial coefficients with constraints.
*                 The degree is 5 and to speed up the processing a Horner scheme is applied. Two multiply-scale
*                 operations are rounded to increase precision and it was the optimal number.
*                 Chebyshev approximations can be used also but the coefficient values
*                 are very big in comparison with the input data format. Big issue with precision !
* @param[in]      x Fixed point representation of the interval [0.5, 1)
*
* @return         The output is between [-1, 0) - Q2.14
* @note           Accuracy (ULP) -> Mean: 15.79, Worst: 14.02
*/

ALWAYS_INLINE vfxp_s2q14 vfxp_log(vfxp_s1q15 x)
{
  const vfxp_s1q15 V_S1Q15_ONE = 0x7FFF;
  // Constants
  const int inQRep = 15;
  const int intermedQRep = 14;

  const int16_t a0 = 0;
  const int16_t a1 = -16384;
  const int16_t a2 = -16102;
  const int16_t a3 = -14277;
  const int16_t a4 = 5563;
  const int16_t a5 = -27696;

  x = ((V_S1Q15_ONE - x) + 1);
  vfxp_s1q15 horner_s1q15 = a5;
  vfxp_s2q14 horner_s2q14;
  horner_s1q15 = vfxp_qmult_scale(horner_s1q15, x, inQRep) + a4;
  horner_s1q15 = vfxp_qmult_scale(horner_s1q15, x, inQRep) + a3;
  horner_s1q15 = vfxp_qmult_scale(horner_s1q15, x, inQRep) + a2;
  horner_s2q14 = vfxp_qmult_scale_round(horner_s1q15, x, (inQRep + (inQRep - intermedQRep))) + a1;
  horner_s2q14 = vfxp_qmult_scale_round(horner_s2q14, x, inQRep);

  return horner_s2q14;
}

/**
* @ingroup        UserAPI
* @brief          Natural exponent
* @details        Computes the natural exponent using an 5-th degree polynomial
*
* @param[in]      x Fixed point representation of the interval [0, 1)
*
* @return         The output is between [1, 2.7183) - Q3.13
* @note           Accuracy (ULP) -> Mean: 13.36, Worst: 11.59
*/

ALWAYS_INLINE vfxp_s3q13 vfxp_exp(vfxp_s1q15 x)
{
  /* Input format */
  const int16_t inQRep = 15;
  const int16_t intermedQRep = 14;
  const int16_t outQRep = 13;

  /*
     scaleIntermed = pow(2.0, intermedQRep);
     scaleOut = pow(2.0, outQRep);
     a = round(0.013872012936445 * scaleIntermed);
     b = round(0.0348011434560224 * scaleIntermed);
     c = round(0.170489461320865 * scaleIntermed);
     d = round(0.499019231968827 * scaleIntermed);
     e = round(1.000099786672 * scaleIntermed);
     f = round(0.999997595032286 * scaleOut);
  */

  const int16_t a = 227;
  const int16_t b = 570;
  const int16_t c = 2793;
  const int16_t d = 8176;
  const int16_t e = 16386;
  const int16_t f = 8192;

  vec16s tmp0 = vfxp_qmult_scale(a, x, inQRep) + b;
  vec16s tmp1 = vfxp_qmult_scale(tmp0, x, inQRep) + c;
  vec16s tmp2 = vfxp_qmult_scale(tmp1, x, inQRep) + d;
  vec16s tmp3 = vfxp_qmult_scale(tmp2, x, inQRep) + e;
  vec16s tmp4 = vfxp_qmult_scale(tmp3, x, inQRep + (intermedQRep - outQRep)) + f;
  return tmp4;
}

#endif /* VFXP_MATH_INLINE_H__ */

/** @} */