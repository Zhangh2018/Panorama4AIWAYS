/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
* All Rights Reserved
*
******************************************************************************
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
*  \file
*  \brief Fixed point types and primitive functions.
*
*  fixed.h contains all fixed point types and primitive functions.  These include
*  - Bit shifts
*  - Type casts
*  - Floor
*  - Fixed point conversion
*  - Addition
*  - Subtraction
*  - Multiplication
*  - Division
*  - Floor
*  - Ceiling
*  - Round
*  - Square Root
*  - Inverse square root
***********************************************************************************/

#ifndef FIXED_H
#define FIXED_H

/*! Include standard integer definitions. */
#include <stdint.h>

typedef unsigned int uint;    /*!< Shorthand for the unsigned integer. */
typedef int8_t   fix8s;       /*!< Signed 8-bit fixed point type. */
typedef uint8_t  fix8u;       /*!< Unsigned 8-bit fixed point type. */
typedef int16_t  fix16s;      /*!< Signed 16-bit fixed point type. */
typedef uint16_t fix16u;      /*!< Unsigned 16-bit fixed point type. */
typedef int32_t  fix32s;      /*!< Signed 32-bit fixed point type. */
typedef uint32_t fix32u;      /*!< Unsigned 32-bit fixed point type. */
typedef int64_t  fix64s;      /*!< Signed 64-bit fixed point type. */
typedef uint64_t fix64u;      /*!< Unsigned 64-bit fixed point type. */

/*!*********************************************************************************
*  ASSERT definition for fixed point primitives.
***********************************************************************************/
#ifndef FIXED_ASSERT
#ifdef __cplusplus
#include <assert.h>
#define FIXED_ASSERT(X) //assert(X)
#else
#define FIXED_ASSERT(X)
#endif   // #ifdef __cplusplus
#endif   // #ifndef FIXED_ASSERT

/*! Number of Newton-Gauss iterations in the 8-bit inverse square root function.
 *   Valid values are 1 and 2.  if the value is less than 1, one iteration is used.
 *   If the value is greater than two, two iterations are used.*/
#define INV_SQRT_8BIT_ITER_COUNT  1

/*! Number of Newton-Gauss iterations in the 16-bit inverse square root function.
 *   Valid values are 1-3.  if the value is less than 1, one iteration is used.
 *   If the value is greater than 3, three iterations are used.*/
#define INV_SQRT_16BIT_ITER_COUNT 1

/*!*********************************************************************************
*  Most significant bit of an unsigned 8-bit integer.
*
*  \return Most significant bit of an unsigned 8-bit integer.
***********************************************************************************/
inline int msb(uint8_t x /*! Unsigned 8-bit integer.*/)
{
  int n;

  if(x > 15)
  {
    x >>= 4;
    n = 4;
  }
  else
  {
    n = 0;
  }

  if(x > 3)
  {
    x >>= 2;
    n += 2;
  }

  if(x > 1)
  {
    x >>= 1;
    n += 1;
  }

  if(x > 0)
  {
    n += 1;
  }

  return n;
}

/*!*********************************************************************************
*  Most significant bit of an unsigned 16-bit integer.
*
*  \return Most significant bit of an unsigned 16-bit integer.
***********************************************************************************/
inline int msb(uint16_t x /*! Unsigned 16-bit integer.*/)
{
  int n;

  if(x > 255)
  {
    x >>= 8;
    n = 8;
  }
  else
  {
    n = 0;
  }

  if(x > 15)
  {
    x >>= 4;
    n += 4;
  }

  if(x > 3)
  {
    x >>= 2;
    n += 2;
  }

  if(x > 1)
  {
    x >>= 1;
    n += 1;
  }

  if(x > 0)
  {
    n += 1;
  }

  return n;
}

/*!*********************************************************************************
*  Most significant bit of an unsigned 32-bit integer.
*
*  \return Most significant bit of an unsigned 32-bit integer.
***********************************************************************************/
inline int msb(uint32_t x /*! Unsigned 32-bit integer.*/)
{
  int n;

  if(x > 65535)
  {
    x >>= 16;
    n = 16;
  }
  else
  {
    n = 0;
  }

  if(x > 255)
  {
    x >>= 8;
    n += 8;
  }

  if(x > 15)
  {
    x >>= 4;
    n += 4;
  }

  if(x > 3)
  {
    x >>= 2;
    n += 2;
  }

  if(x > 1)
  {
    x >>= 1;
    n += 1;
  }

  if(x > 0)
  {
    n += 1;
  }

  return n;
}

/*!*********************************************************************************
*  Most significant bit of an unsigned 64-bit integer.
*
*  \return Most significant bit of an unsigned 64-bit integer.
***********************************************************************************/
inline int msb(uint64_t x /*! Unsigned 64-bit integer.*/)
{
  int n;

  if(x > (uint64_t)4294967295ull)
  {
    x >>= 32;
    n = 32;
  }
  else
  {
    n = 0;
  }

  if(x > (uint64_t)65535)
  {
    x >>= 16;
    n += 16;
  }

  if(x > (uint64_t)255)
  {
    x >>= 8;
    n += 8;
  }

  if(x > (uint64_t)15)
  {
    x >>= 4;
    n += 4;
  }

  if(x > (uint64_t)3)
  {
    x >>= 2;
    n += 2;
  }

  if(x > (uint64_t)1)
  {
    x >>= 1;
    n += 1;
  }

  if(x > (uint64_t)0)
  {
    n += 1;
  }

  return n;
}



/***********************************************************************************/
/*                   1.  UNSIGNED FIXED POINT ARITHMETIC                           */
/***********************************************************************************/


/******************************* BIT SHIFTS ****************************************/



/*!*********************************************************************************
*  Left arithmetic bit shift (8-bit unsigned).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix8u fxsl(fix8u x /*!< Unsigned 8-bit fixed point number. */,
                  uint Q  /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((uint16_t)x << Q) == (uint8_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (16-bit unsigned).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix16u fxsl(fix16u x /*!< Unsigned 16-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((uint32_t)x << Q) == (uint16_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (32-bit unsigned).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix32u fxsl(fix32u x /*!< Unsigned 32-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((uint64_t)x << Q) == (uint32_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (64-bit unsigned).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix64u fxsl(fix64u x /*!< Unsigned 64-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(msb(x) + Q <= 64);       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (8-bit unsigned).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix8u fxsr(fix8u x /*!< Unsigned 8-bit fixed point number. */,
                  uint Q  /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (16-bit unsigned).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix16u fxsr(fix16u x /*!< Unsigned 16-bit fixed point number. */,
                   uint Q  /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (32-bit unsigned).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix32u fxsr(fix32u x /*!< Unsigned 32-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (64-bit unsigned).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix64u fxsr(fix64u x /*!< Unsigned 64-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}



/******************************* TYPE CASTS ***************************************/



/*!*********************************************************************************
*  Cast a 16-bit unsigned fixed point number to an 8-bit unsigned fixed point number.
*
*  \return 8-bit unsigned fixed point number.
***********************************************************************************/
inline fix8u fxcast8(fix16u x /*!< Unsigned 16-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8u)x);     // Overflow Check
  return (fix8u)x;
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned fixed point number to an 8-bit unsigned fixed point number.
*
*  \return 8-bit unsigned fixed point number.
***********************************************************************************/
inline fix8u fxcast8(fix32u x /*!< Unsigned 32-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8u)x);     // Overflow Check
  return (fix8u)x;
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned fixed point number to an 8-bit unsigned fixed point number.
*
*  \return 8-bit unsigned fixed point number.
***********************************************************************************/
inline fix8u fxcast8(fix64u x /*!< Unsigned 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8u)x);     // Overflow Check
  return (fix8u)x;
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned fixed point number to a 16-bit unsigned fixed point number.
*
*  \return 16-bit unsigned fixed point number.
***********************************************************************************/
inline fix16u fxcast16(fix32u x /*!< Unsigned 32-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix16u)x);     // Overflow Check
  return (fix16u)x;
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned fixed point number to a 16-bit unsigned fixed point number.
*
*  \return 16-bit unsigned fixed point number.
***********************************************************************************/
inline fix16u fxcast16(fix64u x /*!< Unsigned 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix16u)x);     // Overflow Check
  return (fix16u)x;
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned fixed point number to a 32-bit unsigned fixed point number.
*
*  \return 32-bit unsigned fixed point number.
***********************************************************************************/
inline fix32u fxcast32(fix64u x /*!< Unsigned 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix32u)x);     // Overflow Check
  return (fix32u)x;
}

/*!*********************************************************************************
*  Cast an 8-bit unsigned integer to an 8-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8(uint8_t x /*! Unsigned 8-bit integer */,
                     uint Q    /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 16-bit unsigned integer to an 8-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8(uint16_t x /*! Unsigned 16-bit integer */,
                     uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned integer to an 8-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8(uint32_t x /*! Unsigned 32-bit integer */,
                     uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned integer to an 8-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8(uint64_t x /*! Unsigned 32-bit integer */,
                     uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 16-bit unsigned integer to a 16-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 16-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxcast16(uint16_t x /*! Unsigned 16-bit integer */,
                       uint Q     /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned integer to a 16-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 16-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxcast16(uint32_t x /*! Unsigned 32-bit integer */,
                       uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast16(x), Q);
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned integer to a 16-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 16-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxcast16(uint64_t x /*! Unsigned 64-bit integer */,
                       uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast16(x), Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned integer to a 32-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 32-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxcast32(uint32_t x /*! Unsigned 32-bit integer */,
                       uint Q     /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 64-bit unsigned integer to a 32-bit unsigned fixed point number with
*  Q fractional bits.
*
*  \return 32-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxcast32(uint64_t x /*! Unsigned 64-bit integer */,
                       uint Q     /*! Number of fractional bits */)
{
  return fxsl(fxcast32(x), Q);
}


/*!*********************************************************************************
*  Cast a double to an 8-bit unsigned fixed point number with Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8u(double x /*! Double precision floating point number */,
                      uint Q   /*! Number of fractional bits */)
{
  FIXED_ASSERT(x >= 0);

  return fxcast8((fix16u)(x * fxsl((fix16u)1, Q) + 0.5));
}

/*!*********************************************************************************
*  Cast a double to a 16-bit unsigned fixed point number with Q fractional bits.
*
*  \return 16-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxcast16u(double x /*! Double precision floating point number */,
                        uint Q   /*! Number of fractional bits */)
{
  FIXED_ASSERT(x >= 0);

  return fxcast16((fix32u)(x * fxsl((fix32u)1, Q) + 0.5));
}

/*!*********************************************************************************
*  Cast a double to a 32-bit unsigned fixed point number with Q fractional bits.
*
*  \return 32-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxcast32u(double x /*! Double precision floating point number */,
                        uint Q   /*! Number of fractional bits */)
{
  FIXED_ASSERT(x >= 0);

  return fxcast32((fix64u)(x * fxsl((fix64u)1, Q) + 0.5));
}

/*!*********************************************************************************
*  Cast a float to an 8-bit unsigned fixed point number with Q fractional bits.
*
*  \return 8-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxcast8u(float x /*! Single precision floating point number */,
                      uint Q  /*! Number of fractional bits */)
{
  FIXED_ASSERT(x >= 0);

  return fxcast8((fix16u)(x * (1ul << Q) + 0.5f));
}

/*!*********************************************************************************
*  Cast a float to a 16-bit unsigned fixed point number with Q fractional bits.
*
*  \return 16-bit unsigned fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxcast16u(float x /*! Single precision floating point number */,
                        uint Q  /*! Number of fractional bits */)
{
  FIXED_ASSERT(x >= 0);

  return fxcast16((fix32u)(x * (1ul << Q) + 0.5f));
}

/*!*********************************************************************************
*  Cast an 8-bit unsigned fixed point number with Q fractional bits to an 8-bit
*  unsigned integer.
*
*  \return 8-bit unsigned integer.
***********************************************************************************/
inline uint8_t fxcasti(fix8u x /*! 8-bit unsigned fixed point number with Q fractional bits */,
                       uint Q  /*! Number of fractional bits */)
{
  return fxsr(x, Q);
}

/*!*********************************************************************************
*  Cast a 16-bit unsigned fixed point number with Q fractional bits to a 16-bit
*  unsigned integer.
*
*  \return 16-bit unsigned integer.
***********************************************************************************/
inline uint16_t fxcasti(fix16u x /*! 16-bit unsigned fixed point number with Q fractional bits */,
                        uint Q   /*! Number of fractional bits */)
{
  return fxsr(x, Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned fixed point number with Q fractional bits to a 32-bit
*  unsigned integer.
*
*  \return 32-bit unsigned integer.
***********************************************************************************/
inline uint32_t fxcasti(fix32u x /*! 32-bit unsigned fixed point number with Q fractional bits */,
                        uint Q   /*! Number of fractional bits */)
{
  return fxsr(x, Q);
}

/*!*********************************************************************************
*  Cast an 8-bit unsigned fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix8u x /*! 8-bit unsigned fixed point number with Q fractional bits */,
                     uint Q  /*! Number of fractional bits */)
{
  return (float)x / (float)(1ul << Q);
}

/*!*********************************************************************************
*  Cast a 16-bit unsigned fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix16u x /*! 16-bit unsigned fixed point number with Q fractional bits */,
                     uint Q   /*! Number of fractional bits */)
{
  return (float)x / (float)(1ul << Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix32u x /*! 32-bit unsigned fixed point number with Q fractional bits */,
                     uint Q   /*! Number of fractional bits */)
{
  return (float)x / (float)(1ul << Q);
}

/*!*********************************************************************************
*  Cast an 8-bit unsigned fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix8u x /*! 8-bit unsigned fixed point number with Q fractional bits */,
                      uint Q  /*! Number of fractional bits */)
{
  return (double)x / (double)(1ul << Q);
}

/*!*********************************************************************************
*  Cast a 16-bit unsigned fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix16u x /*! 16-bit unsigned fixed point number with Q fractional bits */,
                      uint Q   /*! Number of fractional bits */)
{
  return (double)x / (double)(1ul << Q);
}

/*!*********************************************************************************
*  Cast a 32-bit unsigned fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix32u x /*! 32-bit unsigned fixed point number with Q fractional bits */,
                      uint Q   /*! Number of fractional bits */)
{
  return (double)x / (double)(1ul << Q);
}



/****************** FIXED POINT TO FIXED POINT CONVERSION *************************/



/*!*********************************************************************************
*  Convert an 8-bit unsigned fixed point number with inQ fractional bits
*  to an 8-bit unsigned fixed point number with outQ fractional bits.
*
* \return Unsigned 8-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix8u fxcvt(fix8u x   /*! Unsigned 8-bit fixed point number with inQ fractional bits.*/,
                   uint inQ  /*! Number input  of fractional bits.*/,
                   uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ <= 8 && outQ <= 8);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}

/*!*********************************************************************************
*  Convert a 16-bit unsigned fixed point number with inQ fractional bits
*  to a 16-bit unsigned fixed point number with outQ fractional bits.
*
* \return Unsigned 16-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix16u fxcvt(fix16u x  /*! Unsigned 16-bit fixed point number with inQ fractional bits.*/,
                    uint inQ  /*! Number input  of fractional bits.*/,
                    uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ <= 16 && outQ <= 16);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}

/*!*********************************************************************************
*  Convert a 32-bit unsigned fixed point number with inQ fractional bits
*  to a 32-bit unsigned fixed point number with outQ fractional bits.
*
* \return Unsigned 32-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix32u fxcvt(fix32u x  /*! Unsigned 32-bit fixed point number with inQ fractional bits.*/,
                    uint inQ  /*! Number input  of fractional bits.*/,
                    uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ <= 32 && outQ <= 32);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}



/******************************* ADDITION *****************************************/



/*!*********************************************************************************
*  Add two 8-bit unsigned fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 8-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix8u fxadd(fix8u a /*! Unsigned 8-bit fixed point number */,
                   fix8u b /*! Unsigned 8-bit fixed point number */)
{
  FIXED_ASSERT(a <= UINT8_MAX - b);      // Overflow check
  return a + b;
}

/*!*********************************************************************************
*  Add two 16-bit unsigned fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 16-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix16u fxadd(fix16u a /*! Unsigned 16-bit fixed point number */,
                    fix16u b /*! Unsigned 16-bit fixed point number */)
{
  FIXED_ASSERT(a <= UINT16_MAX - b);      // Overflow check
  return a + b;
}

/*!*********************************************************************************
*  Add two 32-bit unsigned fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 32-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix32u fxadd(fix32u a /*! Unsigned 32-bit fixed point number */,
                    fix32u b /*! Unsigned 32-bit fixed point number */)
{
  FIXED_ASSERT(a <= UINT32_MAX - b);      // Overflow check
  return a + b;
}



/***************************** SUBTRACTION ****************************************/



/*!*********************************************************************************
*  Subtract an 8-bit unsigned fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 8-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix8u fxsub(fix8u a /*! Unsigned 8-bit fixed point minuend */,
                   fix8u b /*! Unsigned 8-bit fixed point subtrahend */)
{
  FIXED_ASSERT(a >= b);    // overflow check
  return a - b;
}

/*!*********************************************************************************
*  Subtract a 16bit unsigned fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 16-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix16u fxsub(fix16u a /*! Unsigned 16-bit fixed point minuend */,
                    fix16u b /*! Unsigned 16-bit fixed point subtrahend */)
{
  FIXED_ASSERT(a >= b);    // overflow check
  return a - b;
}

/*!*********************************************************************************
*  Subtract a 32-bit unsigned fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 32-bit unsigned fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix32u fxsub(fix32u a /*! Unsigned 32-bit fixed point minuend */,
                    fix32u b /*! Unsigned 32-bit fixed point subtrahend */)
{
  FIXED_ASSERT(a >= b);    // overflow check
  return a - b;
}



/*************************** MULTIPLICATION ****************************************/



/*!*********************************************************************************
*  Multiply two 8-bit fixed point numbers.
*
*  Since the result is not stored in a 16-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 8-bit unsigned fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8u fxmul(fix8u a /*! Unsigned 8-bit fixed point number.*/,
                   fix8u b /*! Unsigned 8-bit fixed point number.*/)
{
  FIXED_ASSERT((uint8_t)(a * b) == (uint16_t)a * (uint16_t)b);   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Multiply two 16-bit fixed point numbers.
*
*  Since the result is not stored in a 32-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 16-bit unsigned fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16u fxmul(fix16u a /*! Unsigned 16-bit fixed point number.*/,
                    fix16u b /*! Unsigned 16-bit fixed point number.*/)
{
  FIXED_ASSERT((uint16_t)(a * b) == (uint32_t)a * (uint32_t)b);   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Multiply two 32-bit fixed point numbers.
*
*  Since the result is not stored in a 64-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 32-bit unsigned fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32u fxmul(fix32u a /*! Unsigned 32-bit fixed point number.*/,
                    fix32u b /*! Unsigned 32-bit fixed point number.*/)
{
  FIXED_ASSERT((uint32_t)(a * b) == (uint64_t)a * (uint64_t)b);   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Unsigned 8-bit fixed point multiplication followed by a right bit shift by Q.
*
*  The intermediate multiplication result is stored in a 16-bit register, so there
*  is no chance of overflow.  The final result is cast back to 8-bits.
*
*  \return 8-bit unsigned fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8u fxmul(fix8u a /*! Unsigned 8-bit fixed point number.*/,
                   fix8u b /*! Unsigned 8-bit fixed point number.*/,
                   uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast8(fxsr((fix16u)((fix16u)a * (fix16u)b), Q));
}

/*!*********************************************************************************
*  Unsigned 16-bit fixed point multiplication followed by a right bit shift by Q.
*
*  The intermediate multiplication result is stored in a 32-bit register, so there
*  is no chance of overflow.  The final result is cast back to 16-bits.
*
*  \return 16-bit unsigned fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16u fxmul(fix16u a /*! Unsigned 16-bit fixed point number.*/,
                    fix16u b /*! Unsigned 16-bit fixed point number.*/,
                    uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast16(fxsr((fix32u)a * (fix32u)b, Q));
}

/*!*********************************************************************************
*  Unsigned 32-bit fixed point multiplication followed by a right bit shift by Q.
*
*  The intermediate multiplication result is stored in a 64-bit register, so there
*  is no chance of overflow.  The final result is cast back to 32-bits.
*
*  \return 32-bit unsigned fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32u fxmul(fix32u a /*! Unsigned 32-bit fixed point number.*/,
                    fix32u b /*! Unsigned 32-bit fixed point number.*/,
                    uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast32(fxsr((fix64u)a * (fix64u)b, Q));
}





/******************************* DIVISION *****************************************/



/*!*********************************************************************************
*  Unsigned 8-bit fixed point division.
*
*  \return 8-bit unsigned fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8u fxdiv(fix8u a /*! Unsigned 8-bit fixed point numerator.   */,
                   fix8u b /*! Unsigned 8-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Unsigned 16-bit fixed point division.
*
*  \return 16-bit unsigned fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16u fxdiv(fix16u a /*! Unsigned 16-bit fixed point numerator.   */,
                    fix16u b /*! Unsigned 16-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Unsigned 32-bit fixed point division.
*
*  \return 32-bit unsigned fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32u fxdiv(fix32u a /*! Unsigned 32-bit fixed point numerator.   */,
                    fix32u b /*! Unsigned 32-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Unsigned 64-bit fixed point division.
*
*  \return 64-bit unsigned fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix64u fxdiv(fix64u a /*! Unsigned 64-bit fixed point numerator.   */,
                    fix64u b /*! Unsigned 64-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Unsigned 8-bit fixed point division Q fractional bits.
*
*  \return Unsigned 8-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix8u fxdiv(fix8u a /*! Unsigned 8-bit fixed point numerator. */,
                   fix8u b /*! Unsigned 8-bit fixed point denominator. */,
                   uint Q  /*! Number of fractional bits. */)
{
  return fxcast8(fxdiv(fxsl((fix16u)a, Q), (fix16u)b));
}

/*!*********************************************************************************
*  Unsigned 16-bit fixed point division preceeded with Q fractional bits.
*
*  \return Unsigned 16-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix16u fxdiv(fix16u a /*! Unsigned 16-bit fixed point numerator. */,
                    fix16u b /*! Unsigned 16-bit fixed point denominator. */,
                    uint Q   /*! Number of fractional bits. */)
{
  return fxcast16(fxdiv(fxsl((fix32u)a, Q), (fix32u)b));
}

/*!*********************************************************************************
*  Unsigned 32-bit fixed point division with Q fractional bits.
*
*  \return Unsigned 32-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix32u fxdiv(fix32u a /*! Unsigned 32-bit fixed point numerator. */,
                    fix32u b /*! Unsigned 32-bit fixed point denominator. */,
                    uint Q   /*! Number of fractional bits. */)
{
  return fxcast32(fxdiv(fxsl((fix64u)a, Q), (fix64u)b));
}


/********************************** FLOOR *****************************************/


/*!*********************************************************************************
*  Floor of an 8-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxfloor(fix8u x  /*! Unsigned 8-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/*!*********************************************************************************
*  Floor of a 16-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxfloor(fix16u x /*! Unsigned 16-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/*!*********************************************************************************
*  Floor of a 32-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxfloor(fix32u x /*! Unsigned 32-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/********************************* CEILING ****************************************/


/*!*********************************************************************************
*  Ceiling of an 8-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxceil(fix8u x  /*! Unsigned 8-bit fixed point number.*/,
                    uint Q   /*! Number of fractional bits.*/)
{
  fix8u y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix8u)1, Q));
}

/*!*********************************************************************************
*  Ceiling of a 16-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxceil(fix16u x /*! Unsigned 16-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  fix16u y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix16u)1, Q));
}

/*!*********************************************************************************
*  Ceiling of a 32-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxceil(fix32u x /*! Unsigned 32-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  fix32u y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix32u)1, Q));
}

/********************************* ROUND ****************************************/


/*!*********************************************************************************
*  Rounding of an 8-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8u fxround(fix8u x  /*! Unsigned 8-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix8u half = fxsl((fix8u)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}

/*!*********************************************************************************
*  Rounding of a 16-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16u fxround(fix16u x /*! Unsigned 16-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix16u half = fxsl((fix16u)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}

/*!*********************************************************************************
*  Rounding of a 32-bit unsigned fixed point number with Q fractional bits.
*
* \return Unsigned 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32u fxround(fix32u x /*! Unsigned 32-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix32u half = fxsl((fix32u)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}


/******************************* SQUARE ROOT **************************************/

/*!*********************************************************************************
*  Unsigned 8-bit fixed point square root.
*  \return The Unsigned 8-bit fixed point of <x>.
***********************************************************************************/
fix8u fxsqrt(fix8u x  /*! Unsigned 8-bit fixed point input*/,
             uint Q    /*! Number of fractional bits of the input.*/);


/*!*********************************************************************************
*  Unsigned 16-bit fixed point square root.
*  \return The Unsigned 16-bit fixed point of <x>.
***********************************************************************************/
fix16u fxsqrt(fix16u x  /*! Unsigned 16-bit fixed point input*/,
              uint Q    /*! Number of fractional bits of the input.*/);

/*!*********************************************************************************
*  Unsigned 32-bit fixed point square root.
*  \return The Unsigned 16-bit fixed point of <x>.
***********************************************************************************/
fix32u fxsqrt(fix32u x  /*! Unsigned 32-bit fixed point input*/,
              uint Q    /*! Number of fractional bits of the input.*/);

/*!*********************************************************************************
*  Fixed point square root.
***********************************************************************************/
void fxsqrt(fix16u& y  /*! Unsigned 16-bit fixed point output*/,
            uint& R    /*! Number of fractional bits of the output.*/,
            fix16u x   /*! Unsigned 16-bit fixed point input*/,
            uint Q     /*! Number of fractional bits of the input.*/);



/**************************** INVERSE SQUARE ROOT *********************************/

/*!*********************************************************************************
*  Fixed point inverse square root approximation for an unsigned 8-bit input
*  with 7 fractional bits.
*
* \return Unsigned 8-bit fixed point number with 7 fractional bits.
***********************************************************************************/
fix8u fxinvsqrtQ7(fix8u x /*! Unsigned 8-bit fixed point value with 7 fractional bits
                              and and in the range [0.5, 2.0[.*/);

/*!*********************************************************************************
*  Fixed point inverse square root approximation for an unsigned 16-bit input
*  with 15 fractional bits.
*
* \return Unsigned 16-bit fixed point number with 15 fractional bits.
***********************************************************************************/
fix16u fxinvsqrtQ15(fix16u x /*! Unsigned 16-bit fixed point value with 15 fractional bits
                                 and and in the range [0.5, 2.0[.*/);

/*!*********************************************************************************
*  8-bit fixed point inverse square root approximation.
***********************************************************************************/
void fxinvsqrt(fix8u& y    /*! Unsigned 8-bit fixed point output*/,
               uint& R     /*! Number of fractional bits of the output.*/,
               fix8u x     /*! Unsigned 16-bit fixed point input*/,
               uint Q      /*! Number of fractional bits of the input.*/);

/*!*********************************************************************************
*  Fixed point inverse square root approximation.
***********************************************************************************/
void fxinvsqrt(fix16u& y   /*! Unsigned 16-bit fixed point output*/,
               uint& R     /*! Number of fractional bits of the output.*/,
               fix16u x    /*! Unsigned 16-bit fixed point input*/,
               uint Q      /*! Number of fractional bits of the input.*/);

/*!*********************************************************************************
*  Fixed point inverse square root approximation.
***********************************************************************************/
void fxinvsqrt(fix16u& y   /*! Unsigned 16-bit fixed point output*/,
               uint& R     /*! Number of fractional bits of the output.*/,
               fix32u x    /*! Unsigned 32-bit fixed point input*/,
               uint Q      /*! Number of fractional bits of the input.*/);

/*!*********************************************************************************
*  8-bit fixed point inverse square root approximation.
*
* \return Unsigned 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
fix8u fxinvsqrt(fix8u x    /*! Unsigned 8-bit fixed point number*/,
                uint Q     /*! Number of fractional bits.*/);

/*!*********************************************************************************
*  16-bit fixed point inverse square root approximation.
*
* \return Unsigned 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
fix16u fxinvsqrt(fix16u x  /*! Unsigned 16-bit fixed point number*/,
                 uint Q    /*! Number of fractional bits.*/);




/***********************************************************************************
*                    2.  SIGNED FIXED POINT ARITHMETIC
***********************************************************************************/


/******************************* BIT SHIFTS ****************************************/



/*!*********************************************************************************
*  Left arithmetic bit shift (8-bit signed).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix8s fxsl(fix8s x /*!< Signed 8-bit fixed point number. */,
                  uint Q  /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((int16_t)x << Q) == (int8_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (16-bit signed).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix16s fxsl(fix16s x /*!< Signed 16-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((int32_t)x << Q) == (int16_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (32-bit signed).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix32s fxsl(fix32s x /*!< Signed 32-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(((int64_t)x << Q) == (int32_t)(x << Q));       // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Left arithmetic bit shift (64-bit signed).
*
*  A left arithmetic bit shift increases the number of fractional bits.
***********************************************************************************/
inline fix64s fxsl(fix64s x /*!< Signed 64-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  FIXED_ASSERT(msb((uint64_t)(x < 0 ? -x : x)) + Q < 64);     // Check for overflow.
  return x << Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (8-bit signed).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix8s fxsr(fix8s x /*!< Signed 8-bit fixed point number. */,
                  uint Q  /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (16-bit signed).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix16s fxsr(fix16s x /*!< Signed 16-bit fixed point number. */,
                   uint Q  /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (32-bit signed).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix32s fxsr(fix32s x /*!< Signed 32-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}

/*!*********************************************************************************
*  Right arithmetic bit shift (64-bit signed).
*
*  A right arithmetic bit shift decreases the number of fractional bits.
***********************************************************************************/
inline fix64s fxsr(fix64s x /*!< Signed 64-bit fixed point number. */,
                   uint Q   /*!< Number of bit shifts. */)
{
  // Going to more integral bits, no check overflow check necessary.
  return x >> Q;
}



/******************************* TYPE CASTS ***************************************/



/*!*********************************************************************************
*  Cast a 16-bit signed fixed point number to an 8-bit signed fixed point number.
*
*  \return 8-bit signed fixed point number.
***********************************************************************************/
inline fix8s fxcast8(fix16s x /*!< Signed 16-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8s)x);     // Overflow Check
  return (fix8s)x;
}

/*!*********************************************************************************
*  Cast a 32-bit signed fixed point number to an 8-bit signed fixed point number.
*
*  \return 8-bit signed fixed point number.
***********************************************************************************/
inline fix8s fxcast8(fix32s x /*!< Signed 32-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8s)x);     // Overflow Check
  return (fix8s)x;
}

/*!*********************************************************************************
*  Cast a 64-bit signed fixed point number to an 8-bit signed fixed point number.
*
*  \return 8-bit signed fixed point number.
***********************************************************************************/
inline fix8s fxcast8(fix64s x /*!< Signed 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix8s)x);     // Overflow Check
  return (fix8s)x;
}

/*!*********************************************************************************
*  Cast a 32-bit signed fixed point number to a 16-bit signed fixed point number.
*
*  \return 16-bit signed fixed point number.
***********************************************************************************/
inline fix16s fxcast16(fix32s x /*!< Signed 32-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix16s)x);     // Overflow Check
  return (fix16s)x;
}

/*!*********************************************************************************
*  Cast a 64-bit signed fixed point number to a 16-bit signed fixed point number.
*
*  \return 16-bit signed fixed point number.
***********************************************************************************/
inline fix16s fxcast16(fix64s x /*!< Signed 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix16s)x);     // Overflow Check
  return (fix16s)x;
}

/*!*********************************************************************************
*  Cast a 64-bit signed fixed point number to a 32-bit signed fixed point number.
*
*  \return 32-bit signed fixed point number.
***********************************************************************************/
inline fix32s fxcast32(fix64s x /*!< Signed 64-bit fixed point number. */)
{
  FIXED_ASSERT(x == (fix16s)x);     // Overflow Check
  return (fix32s)x;
}

/*!*********************************************************************************
*  Cast an 8-bit signed integer to an 8-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(int8_t x /*! Signed 8-bit integer */,
                     uint Q   /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 16-bit signed integer to an 8-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(int16_t x /*! Signed 16-bit integer */,
                     uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed integer to an 8-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(int32_t x /*! Signed 32-bit integer */,
                     uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 64-bit signed integer to an 8-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(int64_t x /*! Signed 64-bit integer */,
                     uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast8(x), Q);
}

/*!*********************************************************************************
*  Cast a 16-bit signed integer to a 16-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 16-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxcast16(int16_t x /*! Signed 16-bit integer */,
                       uint Q    /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed integer to a 16-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 16-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxcast16(int32_t x /*! Signed 32-bit integer */,
                       uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast16(x), Q);
}

/*!*********************************************************************************
*  Cast a 64-bit signed integer to a 16-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 16-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxcast16(int64_t x /*! Signed 64-bit integer */,
                       uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast16(x), Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed integer to a 32-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 32-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxcast32(int32_t x /*! Signed 32-bit integer */,
                       uint Q    /*! Number of fractional bits */)
{
  return fxsl(x, Q);
}

/*!*********************************************************************************
*  Cast a 64-bit signed integer to a 32-bit signed fixed point number with
*  Q fractional bits.
*
*  \return 32-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxcast32(int64_t x /*! Signed 64-bit integer */,
                       uint Q    /*! Number of fractional bits */)
{
  return fxsl(fxcast32(x), Q);
}

/*!*********************************************************************************
*  Cast a double to an 8-bit signed fixed point number with Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(double x /*! Double precision floating point number */,
                     uint Q   /*! Number of fractional bits */)
{
  return fxcast8((fix16s)(x * fxsl((fix16u)1, Q) + (x < 0 ? -0.5 : 0.5)));
}

/*!*********************************************************************************
*  Cast a double to a 16-bit signed fixed point number with Q fractional bits.
*
*  \return 16-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxcast16(double x /*! Double precision floating point number */,
                       uint Q   /*! Number of fractional bits */)
{
  return fxcast16((fix32s)(x * fxsl((fix32u)1, Q) + (x < 0 ? -0.5 : 0.5)));
}

/*!*********************************************************************************
*  Cast a double to a 32-bit signed fixed point number with Q fractional bits.
*
*  \return 32-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxcast32(double x /*! Double precision floating point number */,
                       uint Q   /*! Number of fractional bits */)
{
  return fxcast32((fix64s)(x * fxsl((fix64u)1, Q) + (x < 0 ? -0.5 : 0.5)));
}

/*!*********************************************************************************
*  Cast a float to an 8-bit signed fixed point number with Q fractional bits.
*
*  \return 8-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxcast8(float x /*! Single precision floating point number */,
                     uint Q  /*! Number of fractional bits */)
{
  return fxcast8((fix16s)(x * fxsl((fix16u)1, Q) + (x < 0 ? -0.5f : 0.5f)));
}

/*!*********************************************************************************
*  Cast a float to a 16-bit signed fixed point number with Q fractional bits.
*
*  \return 16-bit signed fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxcast16(float x /*! Single precision floating point number */,
                       uint Q  /*! Number of fractional bits */)
{
  return fxcast16((fix32s)(x * fxsl((fix32u)1, Q) + (x < 0 ? -0.5f : 0.5f)));
}

/*!*********************************************************************************
*  Cast an 8-bit signed fixed point number with Q fractional bits to an 8-bit
*  signed integer.
*
*  \return 8-bit signed integer.
***********************************************************************************/
inline int8_t fxcasti(fix8s x /*! 8-bit signed fixed point number with Q fractional bits */,
                      uint Q  /*! Number of fractional bits */)
{
  return (uint)x > 0x80 ? -fxsr((fix8u) - x, Q) : fxsr((fix8u)x, Q);
}

/*!*********************************************************************************
*  Cast a 16-bit signed fixed point number with Q fractional bits to a 16-bit
*  signed integer.
*
*  \return 16-bit signed integer.
***********************************************************************************/
inline int16_t fxcasti(fix16s x /*! 16-bit signed fixed point number with Q fractional bits */,
                       uint Q   /*! Number of fractional bits */)
{
  return (uint)x > 0x8000 ? -fxsr((fix16u) - x, Q) : fxsr((fix16u)x, Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed fixed point number with Q fractional bits to a 32-bit
*  signed integer.
*
*  \return 32-bit signed integer.
***********************************************************************************/
inline int32_t fxcasti(fix32s x /*! 32-bit signed fixed point number with Q fractional bits */,
                       uint Q   /*! Number of fractional bits */)
{
  return (uint32_t)x > 0x80000000ul ? -fxsr(-x, Q) : fxsr(x, Q);
}

/*!*********************************************************************************
*  Cast an 8-bit signed fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix8s x /*! 8-bit signed fixed point number with Q fractional bits */,
                     uint Q  /*! Number of fractional bits */)
{
  return (float)x / (float)(1 << Q);
}

/*!*********************************************************************************
*  Cast a 16-bit signed fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix16s x /*! 16-bit signed fixed point number with Q fractional bits */,
                     uint Q   /*! Number of fractional bits */)
{
  return (float)x / (float)(1 << Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed fixed point number with Q fractional bits to a float.
*
*  \return Single precision floating point number.
***********************************************************************************/
inline float fxcastf(fix32s x /*! 32-bit signed fixed point number with Q fractional bits */,
                     uint Q   /*! Number of fractional bits */)
{
  return (float)x / (float)(1 << Q);
}

/*!*********************************************************************************
*  Cast an 8-bit signed fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix8s x /*! 8-bit signed fixed point number with Q fractional bits */,
                      uint Q  /*! Number of fractional bits */)
{
  return (double)x / (double)(1 << Q);
}

/*!*********************************************************************************
*  Cast a 16-bit signed fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix16s x /*! 16-bit signed fixed point number with Q fractional bits */,
                      uint Q   /*! Number of fractional bits */)
{
  return (double)x / (double)(1 << Q);
}

/*!*********************************************************************************
*  Cast a 32-bit signed fixed point number with Q fractional bits to a double.
*
*  \return Double precision floating point number.
***********************************************************************************/
inline double fxcastd(fix32s x /*! 32-bit signed fixed point number with Q fractional bits */,
                      uint Q   /*! Number of fractional bits */)
{
  return (double)x / (double)(1 << Q);
}



/****************** FIXED POINT TO FIXED POINT CONVERSION *************************/



/*!*********************************************************************************
*  Convert an 8-bit signed fixed point number with inQ fractional bits
*  to an 8-bit signed fixed point number with outQ fractional bits.
*
* \return Signed 8-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix8s fxcvt(fix8s x   /*! Signed 8-bit fixed point number with inQ fractional bits.*/,
                   uint inQ  /*! Number input  of fractional bits.*/,
                   uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ < 8 && outQ < 8);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}

/*!*********************************************************************************
*  Convert a 16-bit signed fixed point number with inQ fractional bits
*  to a 16-bit signed fixed point number with outQ fractional bits.
*
* \return Signed 16-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix16s fxcvt(fix16s x  /*! Signed 16-bit fixed point number with inQ fractional bits.*/,
                    uint inQ  /*! Number input  of fractional bits.*/,
                    uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ < 16 && outQ < 16);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}

/*!*********************************************************************************
*  Convert a 32-bit signed fixed point number with inQ fractional bits
*  to a 32-bit signed fixed point number with outQ fractional bits.
*
* \return Signed 32-bit fixed point number with outQ fractional bits.
***********************************************************************************/
inline fix32s fxcvt(fix32s x  /*! Signed 32-bit fixed point number with inQ fractional bits.*/,
                    uint inQ  /*! Number input  of fractional bits.*/,
                    uint outQ /*! Number output of fractional bits.*/)
{
  FIXED_ASSERT(inQ < 32 && outQ < 32);

  return (outQ > inQ) ? fxsl(x, outQ - inQ) : fxsr(x, inQ - outQ);
}



/******************************* ADDITION *****************************************/



/*!*********************************************************************************
*  Add two 8-bit signed fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 8-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix8s fxadd(fix8s a /*! Signed 8-bit fixed point number */,
                   fix8s b /*! Signed 8-bit fixed point number */)
{
  FIXED_ASSERT(((int16_t)a + (int16_t)b) == (int16_t)(a + b));      // overflow check
  return a + b;
}

/*!*********************************************************************************
*  Add two 16-bit signed fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 16-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix16s fxadd(fix16s a /*! Signed 16-bit fixed point number */,
                    fix16s b /*! Signed 16-bit fixed point number */)
{
  FIXED_ASSERT(((int32_t)a + (int32_t)b) == (int32_t)(a + b));      // overflow check
  return a + b;
}

/*!*********************************************************************************
*  Add two 32-bit signed fixed point numbers.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 32-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix32s fxadd(fix32s a /*! Signed 32-bit fixed point number */,
                    fix32s b /*! Signed 32-bit fixed point number */)
{
  FIXED_ASSERT(((int64_t)a + (int64_t)b) == (int64_t)(a + b));      // overflow check
  return a + b;
}



/***************************** SUBTRACTION ****************************************/



/*!*********************************************************************************
*  Subtract an 8-bit signed fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 8-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix8s fxsub(fix8s a /*! Signed 8-bit fixed point minuend */,
                   fix8s b /*! Signed 8-bit fixed point subtrahend */)
{
  FIXED_ASSERT(((int16_t)a - (int16_t)b) == (int16_t)(a - b));      // overflow check
  return a - b;
}

/*!*********************************************************************************
*  Subtract a 16bit signed fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 16-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix16s fxsub(fix16s a /*! Signed 16-bit fixed point minuend */,
                    fix16s b /*! Signed 16-bit fixed point subtrahend */)
{
  FIXED_ASSERT(((int32_t)a - (int32_t)b) == (int32_t)(a - b));      // overflow check
  return a - b;
}

/*!*********************************************************************************
*  Subtract a 32-bit signed fixed point number from another.
*
*  It is assumed the arguments have the same number of fractional bits.
*
* \return 32-bit signed fixed point number with the same fractional bits as a and b.
***********************************************************************************/
inline fix32s fxsub(fix32s a /*! Signed 32-bit fixed point minuend */,
                    fix32s b /*! Signed 32-bit fixed point subtrahend */)
{
  FIXED_ASSERT(((int64_t)a - (int64_t)b) == (int64_t)(a - b));      // overflow check
  return a - b;
}



/*************************** MULTIPLICATION ****************************************/



/*!*********************************************************************************
*  Multiply two 8-bit fixed point numbers.
*
*  Since the result is not stored in a 16-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 8-bit signed fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8s fxmul(fix8s a /*! Signed 8-bit fixed point number.*/,
                   fix8s b /*! Signed 8-bit fixed point number.*/)
{
  FIXED_ASSERT((int16_t)a * (int16_t)b == (int8_t)(a * b));   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Multiply two 16-bit fixed point numbers.
*
*  Since the result is not stored in a 32-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 16-bit signed fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16s fxmul(fix16s a /*! Signed 16-bit fixed point number.*/,
                    fix16s b /*! Signed 16-bit fixed point number.*/)
{
  FIXED_ASSERT((int32_t)a * (int32_t)b == (int16_t)(a * b));   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Multiply two 32-bit fixed point numbers.
*
*  Since the result is not stored in a 64-bit register, there is a possibility of
*  overflow.  Overflow is the programmer's responsibillity.
*
*  \return 32-bit signed fixed point number with fractional bits Q_a + Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32s fxmul(fix32s a /*! Signed 32-bit fixed point number.*/,
                    fix32s b /*! Signed 32-bit fixed point number.*/)
{
  FIXED_ASSERT((int64_t)a * (int64_t)b == (int32_t)(a * b));   // overflow check
  return a * b;
}

/*!*********************************************************************************
*  Signed 8-bit fixed point multiplication with Q fractional bits.
*
*  The intermediate multiplication result is stored in a 16-bit register, so there
*  is no chance of overflow.  The final result is cast back to 8-bits.
*
*  \return 8-bit signed fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8s fxmul(fix8s a /*! Signed 8-bit fixed point number.*/,
                   fix8s b /*! Signed 8-bit fixed point number.*/,
                   uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast8(fxsr((fix16s)((fix16s)a * (fix16s)b), Q));
}

/*!*********************************************************************************
*  Signed 16-bit fixed point multiplication with Q fractional bits.
*
*  The intermediate multiplication result is stored in a 32-bit register, so there
*  is no chance of overflow.  The final result is cast back to 16-bits.
*
*  \return 16-bit signed fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16s fxmul(fix16s a /*! Signed 16-bit fixed point number.*/,
                    fix16s b /*! Signed 16-bit fixed point number.*/,
                    uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast16(fxsr((fix32s)((fix32s)a * (fix32s)b), Q));
}

/*!*********************************************************************************
*  Signed 32-bit fixed point multiplication with Q fractional bits.
*
*  The intermediate multiplication result is stored in a 64-bit register, so there
*  is no chance of overflow.  The final result is cast back to 32-bits.
*
*  \return 32-bit signed fixed point number with fractional bits Q_a + Q_b - Q, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32s fxmul(fix32s a /*! Signed 32-bit fixed point number.*/,
                    fix32s b /*! Signed 32-bit fixed point number.*/,
                    uint Q   /*! Number of right bit shifts.*/)
{
  return fxcast32(fxsr((fix64s)a * (fix64s)b, Q));
}



/******************************* DIVISION *****************************************/



/*!*********************************************************************************
*  Signed 8-bit fixed point division.
*
*  \return 8-bit signed fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix8s fxdiv(fix8s a /*! Signed 8-bit fixed point numerator.   */,
                   fix8s b /*! Signed 8-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Signed 16-bit fixed point division.
*
*  \return 16-bit signed fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix16s fxdiv(fix16s a /*! Signed 16-bit fixed point numerator.   */,
                    fix16s b /*! Signed 16-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Signed 32-bit fixed point division.
*
*  \return 32-bit signed fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix32s fxdiv(fix32s a /*! Signed 32-bit fixed point numerator.   */,
                    fix32s b /*! Signed 32-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Signed 64-bit fixed point division.
*
*  \return 64-bit signed fixed point number with fractional bits Q_a - Q_b, where
*  Q_a and Q_b are the respective fractional bits of a and b.
***********************************************************************************/
inline fix64s fxdiv(fix64s a /*! Signed 64-bit fixed point numerator.   */,
                    fix64s b /*! Signed 64-bit fixed point denominator. */)
{
  FIXED_ASSERT(b != 0);               // division by zero check
  return a / b;
}

/*!*********************************************************************************
*  Signed 8-bit fixed point division with Q fractional bits.
*
*  \return Signed 8-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix8s fxdiv(fix8s a /*! Signed 8-bit fixed point numerator. */,
                   fix8s b /*! Signed 8-bit fixed point denominator. */,
                   uint Q  /*! Number of fractional bits. */)
{
  return fxcast8(fxdiv(fxsl((fix16s)a, Q), (fix16s)b));
}

/*!*********************************************************************************
*  Signed 16-bit fixed point division with Q fractional bits.
*
*  \return Signed 16-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix16s fxdiv(fix16s a /*! Signed 16-bit fixed point numerator. */,
                    fix16s b /*! Signed 16-bit fixed point denominator. */,
                    uint Q   /*! Number of fractional bits. */)
{
  return fxcast16(fxdiv(fxsl((fix32s)a, Q), (fix32s)b));
}

/*!*********************************************************************************
*  Signed 32-bit fixed point division with Q fractional bits.
*
*  \return Signed 32-bit fixed point value with Q fractional bits.
***********************************************************************************/
inline fix32s fxdiv(fix32s a /*! Signed 32-bit fixed point numerator. */,
                    fix32s b /*! Signed 32-bit fixed point denominator. */,
                    uint Q   /*! Number of fractional bits. */)
{
  return fxcast32(fxdiv(fxsl((fix64s)a, Q), (fix64s)b));
}


/********************************** FLOOR *****************************************/



/*!*********************************************************************************
*  Floor of an 8-bit signed fixed point number with Q fractional bits.
*
* \return Signed 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxfloor(fix8s x /*! Signed 8-bit fixed point number.*/,
                     uint Q  /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/*!*********************************************************************************
*  Floor of a 16-bit signed fixed point number with Q fractional bits.
*
* \return Signed 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxfloor(fix16s x /*! Signed 16-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/*!*********************************************************************************
*  Floor of a 32-bit signed fixed point number with Q fractional bits.
*
* \return Signed 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxfloor(fix32s x /*! Signed 32-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  return fxsl(fxsr(x, Q), Q);
}

/********************************* CEILING ****************************************/

/*!*********************************************************************************
*  Ceiling of an 8-bit signed fixed point number with Q fractional bits.
*
* \return Signed 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxceil(fix8s x  /*! Signed 8-bit fixed point number.*/,
                    uint Q   /*! Number of fractional bits.*/)
{
  fix8s y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix8s)1, Q));
}

/*!*********************************************************************************
*  Ceiling of a 16-bit signed fixed point number with Q fractional bits.
*
* \return Signed 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxceil(fix16s x /*! Signed 16-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  fix16s y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix16s)1, Q));
}

/*!*********************************************************************************
*  Ceiling of a 32-bit signed fixed point number with Q fractional bits.
*
* \return Signed 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxceil(fix32s x /*! Signed 32-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  fix32s y = fxfloor(x, Q);

  return y == x ? y : fxadd(y, fxsl((fix32s)1, Q));
}

/********************************* ROUND ****************************************/


/*!*********************************************************************************
*  Rounding of an 8-bit signed fixed point number with Q fractional bits.
*
* \return Signed 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxround(fix8s x  /*! Signed 8-bit fixed point number.*/,
                     uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix8s half = fxsl((fix8s)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}

/*!*********************************************************************************
*  Rounding of a 16-bit signed fixed point number with Q fractional bits.
*
* \return Signed 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxround(fix16s x /*! Signed 16-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix16s half = fxsl((fix16s)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}

/*!*********************************************************************************
*  Rounding of a 32-bit signed fixed point number with Q fractional bits.
*
* \return Signed 32-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix32s fxround(fix32s x /*! Signed 32-bit fixed point number.*/,
                      uint Q   /*! Number of fractional bits.*/)
{
  if(Q)
  {
    fix32s half = fxsl((fix32s)1, (Q - 1));

    return fxfloor(fxadd(x, half), Q);
  }
  else
  {
    return x;
  }
}


/*!*********************************************************************************
*  Signed 8-bit fixed point square root.
*  \return The signed 8-bit fixed point of <x>.
***********************************************************************************/
inline fix8s fxsqrt(fix8s x  /*! Signed 8-bit fixed point input*/,
                    uint Q   /*! Number of fractional bits of the input.*/)
{
  FIXED_ASSERT(x >= 0);
  fix8u y = fxsqrt((fix8u)x, Q);
  FIXED_ASSERT((fix8s)y >= 0);

  return (fix8s)y;
}


/*!*********************************************************************************
*  Signed 16-bit fixed point square root.
*  \return The signed 16-bit fixed point of <x>.
***********************************************************************************/
inline fix16s fxsqrt(fix16s x  /*! Signed 16-bit fixed point input*/,
                     uint Q    /*! Number of fractional bits of the input.*/)
{
  FIXED_ASSERT(x >= 0);
  fix16u y = fxsqrt((fix16u)x, Q);
  FIXED_ASSERT((fix16s)y >= 0);

  return (fix16s)y;
}

/*!*********************************************************************************
*  Signed 32-bit fixed point square root.
*  \return The signed 32-bit fixed point of <x>.
***********************************************************************************/
inline fix32s fxsqrt(fix32s x  /*! Signed 32-bit fixed point input*/,
                     uint Q    /*! Number of fractional bits of the input.*/)
{
  FIXED_ASSERT(x >= 0);
  fix32u y = fxsqrt((fix32u)x, Q);
  FIXED_ASSERT((fix32s)y >= 0);

  return (fix32s)y;
}

/*!*********************************************************************************
*  Signed 8-bit fixed point inverse square root.
***********************************************************************************/
inline void fxinvsqrt(fix8s& y   /*! Signed 8-bit fixed point output*/,
                      uint& R     /*! Number of fractional bits of the output.*/,
                      fix8s x    /*! Signed 8-bit fixed point input*/,
                      uint Q      /*! Number of fractional bits of the input.*/)
{
  FIXED_ASSERT(x > 0);
  fix8u yu;
  fxinvsqrt(yu, R, (fix8u)x, Q);

  if(yu > 127)
  {
    y = (fix8s)fxsr(yu, 1);
    --R;
  }
  else
  {
    y = (fix8s)yu;
  }
}

/*!*********************************************************************************
*  Signed 16-bit fixed point inverse square root.
***********************************************************************************/
inline void fxinvsqrt(fix16s& y   /*! Signed 16-bit fixed point output*/,
                      uint& R     /*! Number of fractional bits of the output.*/,
                      fix16s x    /*! Signed 16-bit fixed point input*/,
                      uint Q      /*! Number of fractional bits of the input.*/)
{
  FIXED_ASSERT(x > 0);
  fix16u yu;
  fxinvsqrt(yu, R, (fix16u)x, Q);

  if(yu > 0x7fff)
  {
    y = (fix16s)fxsr(yu, 1);
    --R;
  }
  else
  {
    y = (fix16s)yu;
  }
}

/*!*********************************************************************************
*  Signed 8-bit fixed point inverse square root.
*
* \return Signed 8-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix8s fxinvsqrt(fix8s x  /*! Signed 8-bit fixed point number*/,
                       uint Q    /*! Number of fractional bits.*/)
{
  FIXED_ASSERT(x > 0);
  fix8u result = fxinvsqrt((fix8u)x, Q);
  FIXED_ASSERT(result == (fix8s)result);

  return (fix8s)result;
}

/*!*********************************************************************************
*  Signed 16-bit fixed point inverse square root.
*
* \return Signed 16-bit fixed point number with Q fractional bits.
***********************************************************************************/
inline fix16s fxinvsqrt(fix16s x  /*! Signed 16-bit fixed point number*/,
                        uint Q    /*! Number of fractional bits.*/)
{
  FIXED_ASSERT(x > 0);
  fix16u result = fxinvsqrt((fix16u)x, Q);
  FIXED_ASSERT(result == (fix16s)result);

  return (fix16s)result;
}


#endif   /* FIXED_H */
