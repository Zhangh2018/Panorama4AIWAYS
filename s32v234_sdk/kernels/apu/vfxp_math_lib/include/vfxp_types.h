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


#ifndef VFX_TYPES_H__
#define VFX_TYPES_H__
#ifdef ACF_KERNEL_IMPLEMENTATION
#ifdef __cplusplus
extern "C" {
#endif

/**
* @file           vfxp_types.h
* @implements     Math Library
*/



/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*==================================================================================================
*                                TYPEDEFS
==================================================================================================*/
/**
* @brief          Fixed point representation of fractional numbers, Qm.m
* @details        vfxp<sign><m>q<n>
*                 sign = u - unsigned/ s - signed
*                 m = number of bits allocated for the integer part
*                 n = number of bits allocated for the fractional part
*                 Example: vfxp_s1q15 - signed, 1 bit for int and 15 for fractional
* @note           Please check: https://en.wikipedia.org/wiki/Q_(number_format)
*                 Unsigned Range: [0, 2^(m) - 2^(-n)]
*                 Signed Range: [-2^(m-1), 2^(m-1) - 2^(-n)]
*/

/* 16 bits */
/* Q1.15 */
typedef vec16s vfxp_s1q15;
typedef vec16u vfxp_u1q15;

/* Q2.14 */
typedef vec16s vfxp_s2q14;
typedef vec16u vfxp_u2q14;

/* Q3.13 */
typedef vec16s vfxp_s3q13;
typedef vec16u vfxp_u3q13;

/* Q4.12 */
typedef vec16s vfxp_s4q12;
typedef vec16u vfxp_u4q12;

/* Q0.16 */
typedef vec16u vfxp_u0q16;

/* 32 bits */
/* Q1.31 */
typedef vec32s vfxp_s1q31;
typedef vec32u vfxp_u1q31;

/* Q2.30 */
typedef vec32s vfxp_s2q30;
typedef vec32u vfxp_u2q30;

/* Q3.29 */
typedef vec32s vfxp_s3q29;
typedef vec32u vfxp_u3q29;

/* Q4.28 */
typedef vec32s vfxp_s4q28;
typedef vec32u vfxp_u4q28;

typedef vec32u vfxp_u0q32;

/* Integer */
typedef vec32s vfxp_s32q0;
typedef vec32u vfxp_u32q0;

#ifdef __cplusplus
}
#endif
#endif
#endif /* VFX_TYPES_H__ */
/** @} */