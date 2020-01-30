/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright (c) 2017 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/

/*!*****************************************************************************
*  \file
*  \brief Basic data types common to various HOG implementations.
*******************************************************************************/

#ifndef HOGTYPESCOMMON_H
#define HOGTYPESCOMMON_H
#include <utility>
#include <stdint.h>
#include "hog_config.h"

#ifdef __cplusplus
#include "assert.h"
#include "stdio.h"
#define PRINT_ERR(A) printf("ERROR: %s at LINE %i\tin %s\n", A, __LINE__, __FILE__); fflush(stdout)
#define HOG_ASSERT(A) if(!(A)) { PRINT_ERR(#A); while(1) {; } }
//assert(A)
#else
#define HOG_ASSERT(A)
#endif   // __cplusplus

#define HOG_ALLOC(SIZE, TYPE) new TYPE[SIZE]
#define HOG_DELETE(PTR) delete[] PTR

#define INTERPOLATE_ANGLE_BINS 0    /*!< Boolean indicating whether or not angle bins are interpolated. */
#define PIXEL_LUT 0                 /*!< Boolean indicating whether or not to use a pixel lookup table. */

/************************ Common Macros **************************/

#ifndef MIN
/*! Minimum value. */
#define MIN(a, b) \
  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
/*! Maximum value. */
#define MAX(a, b) \
  ((a) < (b) ? (b) : (a))
#endif

/*! Area of the Size data structure. */
inline int AREA(const std::pair<int, int> &_a)
{
  return _a.first * _a.second;
}
inline int AREA(const Size &_a)
{
  return _a.height * _a.width;
}

/*! Return the smallest number that is greater than <SIZE> and a multiple of <N>. */
#define ALIGN_SIZE(SIZE, N) (((SIZE) + N - 1) & - N)


/************************************************************************************
*                                VECTOR MACROS
************************************************************************************/

/*! Allocate vector memory. */
#define VECTOR_RESERVE(VECTOR, CAPACITY, TYPE) \
  VECTOR.size = 0; \
  VECTOR.capacity = CAPACITY; \
  VECTOR.data = HOG_ALLOC(VECTOR.capacity, TYPE); \
  HOG_ASSERT(VECTOR.data)

/*! Delete vector memory. */
#define VECTOR_DELETE(VECTOR) \
  VECTOR.size = 0; \
  VECTOR.capacity = 0; \
  HOG_DELETE(VECTOR.data); \
  VECTOR.data = 0

/*! Increment vector size. */
#define VECTOR_PUSH_BACK(VECTOR, VALUE) \
  HOG_ASSERT((VECTOR.capacity < 0) || VECTOR.size < VECTOR.capacity); \
  VECTOR.data[VECTOR.size++] = VALUE

/*! Assert all conditions for a successful vector resize. */
#define ASSERT_VECTOR_RESIZE(VECTOR, SIZE) HOG_ASSERT(SIZE >= 0 && ((VECTOR.capacity < 0) || (VECTOR.data && (vec.capacity >= SIZE))))

/*! Vector resize. */
#define VECTOR_RESIZE(VECTOR, SIZE) \
  HOG_ASSERT((SIZE) >= 0 && ((VECTOR.capacity < 0) || (VECTOR.data && (VECTOR.capacity >= (SIZE))))); \
  VECTOR.size = SIZE


/************************************************************************************
*                                MATRIX MACROS
************************************************************************************/

#ifdef __cplusplus

/*! Allocate matrix memory. */
#define MATRIX_RESERVE(MATRIX, CAPACITY, TYPE) \
  MATRIX.rows = MATRIX.cols = MATRIX.channels = MATRIX.step = 0; \
  MATRIX.capacity = CAPACITY; \
  MATRIX.data = HOG_ALLOC(MATRIX.capacity, TYPE); \
  HOG_ASSERT(MATRIX.data)

/*! Delete matrix memory. */
#define MATRIX_DELETE(MATRIX) \
  MATRIX.rows = MATRIX.cols = MATRIX.channels = MATRIX.step = 0; \
  MATRIX.capacity = 0; \
  HOG_DELETE(MATRIX.data); \
  MATRIX.data = 0;
#endif

/*! Matrix resize. */
#define MATRIX_RESIZE(MAT, SIZE, CHANNELS) \
  HOG_ASSERT(SIZE.width >= 0 && SIZE.height >= 0 && CHANNELS > 0); \
  HOG_ASSERT((MAT.capacity < 0) || (MAT.data && (MAT.capacity >= SIZE.width * SIZE.height * CHANNELS))); \
  MAT.rows = SIZE.height; \
  MAT.cols = SIZE.width; \
  MAT.channels = CHANNELS; \
  MAT.step = MAT.cols * MAT.channels

/*! Matrix initialization. */
#define MAT_INI(MAT, DATA, SIZE, CHANNELS) \
  MAT.data = DATA; \
  MAT.rows = SIZE.height; \
  MAT.cols = SIZE.width; \
  MAT.channels = CHANNELS; \
  MAT.step = MAT.cols * MAT.channels

/*! Assert the point is within bounds of the matrix. */
#define ASSERT_MAT_PTR(MAT, POINT) HOG_ASSERT(POINT.x >= 0 && POINT.x < MAT.cols && POINT.y >= 0 && POINT.y < MAT.rows && MAT.data)

/*! Return the data pointer at a specific poin in the matrix. */
#define MAT_PTR(PTR, MAT, POINT) \
  HOG_ASSERT(POINT.x >= 0 && POINT.x < MAT.cols && POINT.y >= 0 && POINT.y < MAT.rows && MAT.data); \
  PTR = MAT.data + (POINT.y * MAT.step + POINT.x * MAT.channels)


enum
{
  GAMMA_NONE  /*!< No gamma correction. */,
  GAMMA_SQRT  /*!< Square root gamma correction. */
};
const double GAMMA_SCALE = 16;      /*!< Default gamma scaling factor. */
const int GAMMA_TYPE = GAMMA_SQRT   /*!< Default type of gamma correction. */;

typedef unsigned char uchar;     /*!< Shorthand for unsigned char. */

/*! General 2D point data structure. */
struct Point
{
  int x;
  int y;
};

/*! General 2D rectangle data structure. */
struct Rect
{
  int x;
  int y;
  int width;
  int height;
};


/************************************************************************************
*                                   HOG CACHE
************************************************************************************/

/*! The data describing each block within the detection window. */
struct BlockData
{
  int histOfs;      /*!< The offset giving the beginning of the block histogram(s) for this block within the HOG descriptor. */
  Point imgOffset   /*!< The pixel offset of this block with respect to the detection window. */;
};


/************************************************************************************
*                                    VECTORS
************************************************************************************/

/*! Vector of unsigned 8-bit integers. */
struct Vec8u
{
  int size;
  int capacity;
  uchar* data;
};

/*! Vector of signed 16-bit integers. */
struct Vec16s
{
  int size;
  int capacity;
  int16_t* data;
};

/*! Vector of signed 32-bit integers. */
struct Vec32s
{
  int size;
  int capacity;
  int32_t* data;
};

/*! Vector of floats. */
struct Vecf
{
  int size;
  int capacity;
  float* data;
};

/*! Vector of doubles. */
struct Vecd
{
  int size;
  int capacity;
  double* data;
};

/*! Vector of 2D points. */
struct VecPoint
{
  int size;
  int capacity;
  Point* data;
};

/*! Vector of block data. */
struct VecBlockData
{
  BlockData* data;
  int size;
  int capacity;
};


/************************************************************************************
*                                    MATRICES
************************************************************************************/

/*! Matrix of unsigned 8-bit integers. */
struct Mat8u
{
  int rows;
  int cols;
  int channels;
  int step;
  int capacity;
  uchar* data;
};

/*! Matrix of unsigned 16-bit integers. */
struct Mat16u
{
  int rows;
  int cols;
  int channels;
  int step;
  int capacity;
  uint16_t* data;
};

/*! Matrix of signed 32-bit integers. */
struct Mat32s
{
  int rows;
  int cols;
  int channels;
  int step;
  int capacity;
  int32_t* data;
};

/************************************************************************************
*                           COMMON INLINE FUNCTIONS
************************************************************************************/

/*!*********************************************************************************
*  Greatest common denominator between <a> and <b>.
*  \return The greatest common denominator between <a> and <b>
***********************************************************************************/
inline int gcd(int a, int b)
{
  if(a < b)
  {
    int tmp = a;
    a = b;
    b = tmp;
  }

  while(b > 0)
  {
    int r = a % b;
    a = b;
    b = r;
  }

  return a;
}

/*!*********************************************************************************
*  Compute the HOG cache stride, which is the horizontal and vertical intervals at
*  which block data must be cached.
***********************************************************************************/
inline Size HOG_getCacheStride(const Size& windowStride  /*! The detection window stride. */,
                               const Size& blockStride   /*! The block stride. */)
{
  Size cacheStride;
  cacheStride.width = gcd(windowStride.width, blockStride.width);
  cacheStride.height = gcd(windowStride.height, blockStride.height);

  return cacheStride;
}

#endif   /* HOGTYPESCOMMON_H */
