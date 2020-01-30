/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015-2017 NXP
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


#include "fixed.h"

// This macro is used in computing the square root of a fixed point number.
#define SQRT_STEP                   \
  tmp >>= 2;                      \
  testVal = tmp + root;           \
  if(testVal <= value)            \
  {                               \
    value -= testVal;           \
    root = (root >> 1) | tmp;   \
  }                               \
  else                            \
  {                               \
    root >>= 1;                 \
  }

fix8u fxsqrt(fix8u x, uint Q)     // square root
{
  uint16_t root = 0,
           tmp = 0x4000L,
           testVal,
           value = ((uint16_t)x) << Q;

  FIXED_ASSERT((value >> Q) == x);

  // bit 0
  if(tmp <= value)
  {
    value -= tmp;
    root = tmp;
  }

  SQRT_STEP;       // bit 2
  SQRT_STEP;       // bit 4
  SQRT_STEP;       // bit 6
  SQRT_STEP;       // bit 8
  SQRT_STEP;       // bit 10
  SQRT_STEP;       // bit 12
  SQRT_STEP;       // bit 14

  // round to the nearest integer, cuts max error in half
  if(root < value)
  {
    ++root;
  }

  return fxcast8(root);
}

fix16u fxsqrt(fix16u x, uint Q)     // square root
{
  uint32_t root = 0,
           tmp = 0x40000000L,
           testVal,
           value = ((uint32_t)x) << Q;

  FIXED_ASSERT((value >> Q) == x);

  // bit 0
  if(tmp <= value)
  {
    value -= tmp;
    root = tmp;
  }

  SQRT_STEP;       // bit 2
  SQRT_STEP;       // bit 4
  SQRT_STEP;       // bit 6
  SQRT_STEP;       // bit 8
  SQRT_STEP;       // bit 10
  SQRT_STEP;       // bit 12
  SQRT_STEP;       // bit 14
  SQRT_STEP;       // bit 16
  SQRT_STEP;       // bit 18
  SQRT_STEP;       // bit 20
  SQRT_STEP;       // bit 22
  SQRT_STEP;       // bit 24
  SQRT_STEP;       // bit 26
  SQRT_STEP;       // bit 28
  SQRT_STEP;       // bit 30

  // round to the nearest integer, cuts max error in half
  if(root < value)
  {
    ++root;
  }

  return fxcast16(root);
}

fix32u fxsqrt(fix32u x, uint Q)     // square root
{
  uint64_t root = 0,
           tmp = 0x4000000000000000ULL,
           testVal,
           value = ((uint64_t)x) << (uint64_t)Q;

  FIXED_ASSERT((value >> Q) == x);

  // bit 0
  if(tmp <= value)
  {
    value -= tmp;
    root = tmp;
  }

  for(uint i = 0; i < 31; ++i)
  {
    SQRT_STEP;
  }

  // round to the nearest integer, cuts max error in half
  if(root < value)
  {
    ++root;
  }

  return fxcast32(root);
}

void fxsqrt(fix16u& y, uint& R, fix16u x, uint Q)     // fixed point square root
{
  const uint P = 15;

  // Find normalisation scale:  s = 2^(2K)
  int TWO_K = (int)Q - msb(x);
  TWO_K += (TWO_K & 1);
  FIXED_ASSERT(((int)Q >= TWO_K) && (TWO_K > 0));
  const uint M = Q - (uint)TWO_K;

  if(P >= M)
  {
    x = fxsl(x, P - M);
  }
  else
  {
    x = fxsr(x, M - P);
  }


  // This algorithm is only accurate if x (in fixed point representation
  // with 15 fractional bits) is in the range [0.5, 2.0[
  FIXED_ASSERT(((0x4000 <= x) && (x <= 0xffff)) || (x == 0));      // Check that 0.5 <= x < 2.0

  y = fxsqrt(x, P);

  // Find the greatest number of fractional bits, R,
  // that can be used to represent the result
  if(TWO_K > 0)
  {
    y = fxsr(y, (TWO_K >> 1));
    R = P;
  }
  else
  {
    R = P + (TWO_K >> 1);
  }
  FIXED_ASSERT(R < 16);
}

fix8u fxinvsqrtQ7(fix8u x)
{
  // This algorithm assumes x is in Q15 and is in the range [0.5, 2.0[
  // Check that 0.5 <= x < 2.0
  FIXED_ASSERT((0x40 <= x) && (x <= 0xff));

  /*****************************************************************************
   *  This algorithm uses Newton-Gauss iterations to approximate the inverse
   *  square root of x.  The Newton-Guass iteration for the inverse square root
   *  is
   *                 y[n+1] = 1.5*y[n] - x*0.5*y[n]^3
   *
   *  We choose the initial guess, y[0], by partitioning the range [0.5, 2.0[
   *  into three and selecting an initial guess that makes 0.5*y[0]^3 a power
   *  of 2.  This way, we don't actually perform fixed point multiplication in the
   *  first iteration and so we don't need 32-bit intermediate values.  Also
   *  the partitions are chosen to minimized the global maximum error in the
   *  result.  The partitions and the corresponding initial guesses are listed
   *  below:
   *
   *     |______Partition_________|_____y[0]_____|___1.5*y[0]___|__0.5*y[0]^3__|
   *     |  0.5   <= x < 0.778    |    2^(1/3)   |   242 (Q7)   |      1       |
   *     |  0.778 <= x < 1.24     |      1       |   192 (Q7)   |     0.5      |
   *     |  1.24  <= x < 2.0      |   2^(-1/3)   |   152 (Q7)   |     0.25     |
   ********************************************************************************/

  const fix8u half_x = fxsr(x, 1);     // x/2
  const fix8u quarter_x = fxsr(x, 2);  // x/4

  if(x < 100)
  {
    x = fxsub((fix8u)242, x);                            // 0.5   <= x < 0.778
  }
  else if(x < 159)
  {
    x = fxsub((fix8u)192, half_x);                       // 0.778 <= x < 1.24
  }
  else
  {
    x = fxsub((fix8u)152, quarter_x);                    // 1.24  <= x < 2.0
  }
#if (INV_SQRT_8BIT_ITER_COUNT <= 1)
  return x;
#else    // (INV_SQRT_8BIT_ITER_COUNT > 1)
  // Second Iteration
  // Note: There is no gain in precision beyond two iterations.


  const uint Q = 7;        // Fractional bits
  const fix16u h = 64;     // 0.5 in Q7 fixed point
  const fix16u c = 192;    // 1.5
  fix16u y = x;            // return value
  fix16u t;                // temporary variable

  t = fxsr(fxadd(fxmul(y, y), h), Q);
  t = fxsr(fxadd(fxmul(t, (fix16u)half_x), h), Q);
  t = fxsub(c, t);
  y = fxsr(fxadd(fxmul(y, t), h), Q);

  y = fxsr(y, Q - 7);

  return fxcast8(y);
#endif   // Second  Iteration
}

fix16u fxinvsqrtQ15(fix16u x)
{
  // This algorithm assumes x is in Q15 and is in the range [0.5, 2.0[
  // Check that 0.5 <= x < 2.0
  FIXED_ASSERT((0x4000 <= x) && (x <= 0xffff));

  /*****************************************************************************
   *  This algorithm uses Newton-Gauss iterations to approximate the inverse
   *  square root of x.  The Newton-Guass iteration for the inverse square root
   *  is
   *                 y[n+1] = 1.5*y[n] - x*0.5*y[n]^3
   *
   *  We choose the initial guess, y[0], by partitioning the range [0.5, 2.0[
   *  into three and selecting an initial guess that makes 0.5*y[0]^3 a power
   *  of 2.  This way, we don't actually perform fixed point multiplication in the
   *  first iteration and so we don't need 32-bit intermediate values.  Also
   *  the partitions are chosen to minimized the global maximum error in the
   *  result.  The partitions and the corresponding initial guesses are listed
   *  below:
   *
   *     |______Partition_________|_____y[0]_____|___1.5*y[0]___|__0.5*y[0]^3__|
   *     |  0.5   <= x < 0.778    |    2^(1/3)   |  61928 (Q15) |      1       |
   *     |  0.778 <= x < 1.24     |      1       |  49152 (Q15) |     0.5      |
   *     |  1.24  <= x < 2.0      |   2^(-1/3)   |  39012 (Q15) |     0.25     |
   ********************************************************************************/

  const fix16u half_x16 = fxsr(x, 1);   // x/2 in 16-bit

  if(x < 25494)
  {
    x = fxsub((fix16u)61928, x);                               // 0.5   <= x < 0.778
  }
  else if(x < 40632)
  {
    x = fxsub((fix16u)49152, half_x16);                        // 0.778 <= x < 1.24
  }
  else
  {
    x = fxsub((fix16u)39012, fxsr(x, 2));                      // 1.24  <= x < 2.0
  }
#if (INV_SQRT_16BIT_ITER_COUNT <= 1)
  return x;
#else    // (INV_SQRT_16BIT_ITER_COUNT > 1)
  const uint Q = 15;                   // Fractional bits
  const fix32u c = 49152;              // 1.5 in Q15
  const fix32u half_x = half_x16;      // x/2 in 32-bit
  fix32u t;        // temporary variable
  fix32u y = x;    // return value

   #define INV_SQRT_ITER   \
  t = fxsr(fxmul(y, y), Q); \
  t = fxsr(fxmul(t, half_x), Q); \
  t = fxsub(c, t);     \
  y = fxsr(fxmul(y, t), Q);

  // Second Iteration
  INV_SQRT_ITER

#if (INV_SQRT_16BIT_ITER_COUNT > 2)
  // Third Iteration
  // Note: There is no gain in precision beyond three iterations.
  INV_SQRT_ITER
#endif   // Third  Iteration

  return fxcast16(y);
#endif   // Second  Iteration
}


void fxinvsqrt(fix8u& y, uint& R, fix8u x, uint Q)     // fixed point inverser square root approximation
{
  const uint P = 7;

  // Find normalisation scale:  s = 2^(2K)
  int TWO_K = (int)Q - msb(x);
  TWO_K += (TWO_K & 1);
  FIXED_ASSERT(((int)Q >= TWO_K) && (TWO_K > 0));
  const uint M = Q - (uint)TWO_K;

  if(P >= M)
  {
    x = fxsl(x, P - M);
  }
  else
  {
    x = fxsr(x, M - P);
  }

  y = fxinvsqrtQ7(x);

  // Find the greatest number of fractional bits, R,
  // that can be used to represent the result
  if(TWO_K < 0)
  {
    y = fxsr(y, -(TWO_K >> 1));
    R = P;
  }
  else
  {
    R = P - (TWO_K >> 1);
  }
  FIXED_ASSERT(R < 8);
}

void fxinvsqrt(fix16u& y, uint& R, fix16u x, uint Q)     // fixed point inverser square root approximation
{
  const uint P = 15;

  // Find normalisation scale:  s = 2^(2K)
  int TWO_K = (int)Q - msb(x);
  TWO_K += (TWO_K & 1);
  FIXED_ASSERT(((int)Q >= TWO_K) && (TWO_K > 0));
  const uint M = Q - (uint)TWO_K;

  if(P >= M)
  {
    x = fxsl(x, P - M);
  }
  else
  {
    x = fxsr(x, M - P);
  }

  y = fxinvsqrtQ15(x);

  // Find the greatest number of fractional bits, R,
  // that can be used to represent the result
  if(TWO_K < 0)
  {
    y = fxsr(y, -(TWO_K >> 1));
    R = P;
  }
  else
  {
    R = P - (TWO_K >> 1);
  }
  FIXED_ASSERT(R < 16);
}

void fxinvsqrt(fix16u& y, uint& R, fix32u x, uint Q)     // fixed point inverser square root approximation
{
  const uint P = 15;

  // Find normalisation scale:  s = 2^(2K)
  int TWO_K = (int)Q - msb(x);
  TWO_K += (TWO_K & 1);
  FIXED_ASSERT(((int)Q >= TWO_K) && (TWO_K > 0));
  const uint M = Q - (uint)TWO_K;

  if(P >= M)
  {
    x = fxsl(x, P - M);
  }
  else
  {
    x = fxsr(x, M - P);
  }

  y = fxinvsqrtQ15(fxcast16(x));

  // Find the greatest number of fractional bits, R,
  // that can be used to represent the result
  if(TWO_K < 0)
  {
    y = fxsr(y, -(TWO_K >> 1));
    R = P;
  }
  else
  {
    R = P - (TWO_K >> 1);
  }
  FIXED_ASSERT(R < 16);
}

fix8u fxinvsqrt(fix8u x, uint Q)     // fixed point inverser square root approximation
{
  uint R;
  fxinvsqrt(x, R, x, Q);

  return fxcvt(x, R, Q);
}

fix16u fxinvsqrt(fix16u x, uint Q)     // fixed point inverser square root approximation
{
  uint R;
  fxinvsqrt(x, R, x, Q);

  return fxcvt(x, R, Q);
}
