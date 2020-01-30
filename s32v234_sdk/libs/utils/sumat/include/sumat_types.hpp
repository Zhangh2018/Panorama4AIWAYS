/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright 2017 NXP
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

/*M/////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
//  license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
//     notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
//     products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
// and any express or implied warranties, including, but not limited to, the
// implied warranties of merchantability and fitness for a particular purpose
// are disclaimed. In no event shall the Intel Corporation or contributors be
// liable for any direct, indirect, incidental, special, exemplary, or
// consequential damages (including, but not limited to, procurement of
// substitute goods or services; loss of use, data, or profits; or business
// interruption) however caused and on any theory of liability, whether in
// contract, strict liability, or tort (including negligence or otherwise)
// arising in any way out of the use of this software, even if advised of the
// possibility of such damage.
//
//M*/

#ifndef SUMATTYPES_HPP
#define SUMATTYPES_HPP

#include <cstdint>
#include <algorithm>

namespace vsdk
{
/**
 * @brief          Class encapsulating range definition
 */
class Range
{
public:
  /**
   * @brief          Default constructor
   */
  Range();

  /**
   * @brief          Constructor with specified values
   *
   * @param[in]      aStart      Starting index of the range
   * @param[in]      aEnd        Last index of the range
   */
  Range(const int32_t aStart, const int32_t aEnd);

  /**
   * @brief          Function returns size of the range
   *
   * @return         Computed size of the range
   */
  int32_t size() const;

  /**
   * @brief          Function returns true is range is empty
   *
   * @return         True if size of the range is 0
   */
  bool empty() const;

  /**
   * @brief          Function returns min to max range
   *
   * @return         Range which specifies whole original range
   */
  static vsdk::Range all();

  /**
   * @brief          Min and max index
   *
   */
  int32_t start;
  int32_t end;
}; /* class Range */

/**
 * @brief          Structure encapsulating sizes of the matrix
 */
struct MatSize
{
  /**
   * @brief          Default constructor
   *
   * @param[in]      apSizes            Array of sizes passed to constructor
   *
   */
  explicit MatSize(int32_t* const apSizes);

  /**
   * @brief          Get array element operator - returns the size on specified
   *                 index (const)
   *
   * @param[in]      aIndex             Index to return
   *
   * @return         value of the returned size
   */
  const int32_t& operator[](const int32_t aIndex) const;

  /**
   * @brief          Get array element operator - returns the size on specified
   *                 index
   *
   * @param[in]      aIndex             Index to return
   *
   * @return         value of the returned size
   */
  int32_t& operator[](const int32_t aIndex);

  /**
   * @brief          Pointer retype operator
   *
   * @return         Value of the returned pointer
   */
  operator const int32_t*() const;

  /**
   * @brief          Comparison operator
   *
   * @param[in]      aSize            Second MatSize to compare
   *
   * @return         True if objects are equal
   */
  bool operator==(const vsdk::MatSize& aSize) const;

  /**
   * @brief          Comparison operator
   *
   * @param[in]      aSize            Second MatSize to compare
   *
   * @return         True if objects are not equal
   */
  bool operator!=(const vsdk::MatSize& aSize) const;

  /**
   * @brief          Internal structure pointer
   */
  int32_t* p;
}; /* struct MatSize */

/**
 * @brief          Structure encapsulating Matrix step values
 */
struct MatStep
{
  /**
   * @brief          Default constructor
   */
  MatStep();

  /**
   * @brief          Constructor with specified value
   *
   * @param[in]      aStep             First step value
   *
   */
  explicit MatStep(const uint64_t aStep);

  /**
   * @brief          Get array element operator - returns the step on specified
   *                 index (const)
   *
   * @param[in]      aIndex             Index to return
   *
   * @return         value of the returned step
   */
  const uint64_t& operator[](const int32_t aIndex) const;

  /**
   * @brief          Get array element operator - returns the step on specified
   *                 index
   *
   * @param[in]      aIndex              Index to return
   *
   * @return         value of the returned step
   */
  uint64_t& operator[](const int32_t aIndex);

  /**
   * @brief          Pointer retype operator
   *
   * @return         value of the returned pointer
   */
  operator uint64_t() const;

  /**
   * @brief          Assignment operator
   *
   * @param[in]      aStep              Step value to be assigned
   *
   * @return         Resulting MatStep object
   */
  MatStep& operator=(const uint64_t aStep);

  /**
   * @brief          Internal buffer pointer
   */
  uint64_t* p;

  /**
   * @brief          Internal buffer
   */
  uint64_t buf[3];
}; /* struct MatStep */

/**
 * @brief          Class describing 2D point type
 */
template < typename Type_t >
class Point_
{
public:
  /**
   * @brief          Default constructor
   */
  Point_();

  /**
   * @brief          Constructor - 2D point
   *
   * @param[in]      aX           X value
   * @param[in]      aY           Y value
   */
  Point_(Type_t aX, Type_t aY);

  /**
   * @brief          Copy constructor
   *
   * @param[in]      aPt           Point to be copied
   */
  Point_(const Point_& aPt);

  /**
   * @brief          Assignment operator
   *
   * @param[in]      aPt           Point to be assigned
   */
  Point_& operator=(const Point_& aPt);

  /**
   * @brief          Point coordinates
   */
  Type_t x;
  Type_t y;
}; /* Point_ */

typedef Point_< int32_t > Point2i;
typedef Point2i           Point;

/**
 * @brief          Class describing a rectangle
 */
template < typename Type_t >
class Rect_
{
public:
  /**
   * @brief          Default constructor
   */
  Rect_();

  /**
   * @brief          Constructor - 2D point
   *
   * @param[in]      aX           X origin value
   * @param[in]      aY           Y origin value
   * @param[in]      aWidth       Width
   * @param[in]      aHeight      Height
   */
  Rect_(Type_t aX, Type_t aY, Type_t aWidth, Type_t aHeight);

  /**
   * @brief          Copy constructor
   *
   * @param[in]      aRect            Rectangle to be copied from
   */
  Rect_(const Rect_& aRect);

  /**
   * @brief          Constructor - defined by two corner points
   *
   * @param[in]      aPt1          Point 1
   * @param[in]      aPt2          Point 2
   */
  Rect_(const vsdk::Point_< Type_t >& aPt1, const vsdk::Point_< Type_t >& aPt2);

  /**
   * @brief        Assignment operator
   *
   * @param[in]    aRect              Rectangle to be assigned
   */
  Rect_& operator=(const Rect_& aRect);

  /**
   * @brief        Top left corner
   *
   * @return       Top left corner
   */
  Point_< Type_t > tl() const;

  /**
   * @brief        Bottom right corner
   *
   * @return       Bottom right corner
   */
  Point_< Type_t > br() const;

  /**
   * @brief        The top-left corner, as well as width and height of the
   *               rectangle
   */
  Type_t x;
  Type_t y;
  Type_t width;
  Type_t height;
}; /* Rect_ */
typedef Rect_< int32_t > Rect2i;
typedef Rect2i           Rect;

} /* namespace vsdk */

bool operator==(const vsdk::Range& aRange1, const vsdk::Range& aRange2);
bool operator!=(const vsdk::Range& aRange1, const vsdk::Range& aRange2);

/**
 * Default constructor
 */
template < typename Type_t >
inline vsdk::Point_< Type_t >::Point_() : x(0), y(0)
{
}

/**
 * Constructor - 2D point
 */
template < typename Type_t >
inline vsdk::Point_< Type_t >::Point_(Type_t aX, Type_t aY) : x(aX), y(aY)
{
}

/**
 * Copy constructor
 */
template < typename Type_t >
inline vsdk::Point_< Type_t >::Point_(const Point_& aPt) : x(aPt.x), y(aPt.y)
{
}

/**
 * Assignment operator
 */
template < typename Type_t >
inline vsdk::Point_< Type_t >& vsdk::Point_< Type_t >::operator=(const Point_& aPt)
{
  x = aPt.x;
  y = aPt.y;
  return *this;
}

/**
 * Default constructor
 */
template < typename Type_t >
inline vsdk::Rect_< Type_t >::Rect_() : x(0), y(0), width(0), height(0)
{
}

/**
 * Constructor - 2D point
 */
template < typename Type_t >
inline vsdk::Rect_< Type_t >::Rect_(Type_t aX, Type_t aY, Type_t aWidth, Type_t aHeight)
    : x(aX), y(aY), width(aWidth), height(aHeight)
{
}

/**
 * Copy constructor
 */
template < typename Type_t >
inline vsdk::Rect_< Type_t >::Rect_(const vsdk::Rect_< Type_t >& aRect)
    : x(aRect.x), y(aRect.y), width(aRect.width), height(aRect.height)
{
}

/**
 * Constructor - defined by two corner points
 */
template < typename Type_t >
inline vsdk::Rect_< Type_t >::Rect_(const vsdk::Point_< Type_t >& aPt1, const vsdk::Point_< Type_t >& aPt2)
{
  x      = std::min(aPt1.x, aPt2.x);
  y      = std::min(aPt1.y, aPt2.y);
  width  = std::max(aPt1.x, aPt2.x) - x;
  height = std::max(aPt1.y, aPt2.y) - y;
}

/**
 * Assignment operator
 */
template < typename Type_t >
inline vsdk::Rect_< Type_t >& vsdk::Rect_< Type_t >::operator=(const vsdk::Rect_< Type_t >& aRect)
{
  x      = aRect.x;
  y      = aRect.y;
  width  = aRect.width;
  height = aRect.height;
  return *this;
}

/**
 * Top left corner
 */
template < typename Type_t >
inline vsdk::Point_< Type_t > vsdk::Rect_< Type_t >::tl() const
{
  return vsdk::Point_< Type_t >(x, y);
}

/**
 * Bottom right corner
 */
template < typename Type_t >
inline vsdk::Point_< Type_t > vsdk::Rect_< Type_t >::br() const
{
  return vsdk::Point_< Type_t >(x + width, y + height);
}

#endif /* UMATTYPES_HPP */
