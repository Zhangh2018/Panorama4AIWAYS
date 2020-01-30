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
#ifndef SUMATMAT_HPP
#define SUMATMAT_HPP

#include "sumat_defines.hpp"
#include "sumat_types.hpp"
#include "sumat_data.hpp"

#include <iostream>

namespace vsdk
{
/**
 * @brief          A SMat class copying and enhancing the behavior from OpenCV
 */
class SMat
{
public:
  enum
  {
    MAGIC_VAL       = 0x42FF0000,
    AUTO_STEP       = 0,
    CONTINUOUS_FLAG = 16384U,
    SUBMATRIX_FLAG  = 32768U
  };

  enum
  {
    MAGIC_MASK = 0xFFFF0000,
    TYPE_MASK  = 0x00000FFF,
    DEPTH_MASK = 7
  };

  /**
   * @brief          Constructor, creates and initializes the SMat
   *
   * These are various constructors that form a matrix. As noted in the
   * AutomaticAllocation, often the default constructor is enough, and the
   * proper matrix will be allocated by an OpenCV function. The constructed
   * matrix can further be assigned to another matrix or matrix expression or
   * can be allocated with SMat::create . In the former case, the old content
   * is de-referenced.
   */
  SMat();

  /**
   * @overload
   *
   * @param[in]      aRows        Number of rows in a 2D array.
   * @param[in]      aCols        Number of columns in a 2D array.
   * @param[in]      aType        Array type. Use CV_8UC1, ..., CV_64FC4 to
   *                              create 1-4 channel matrices, or CV_8UC(n),
   *                              ..., CV_64FC(n) to create multi-channel
   *                              (up to CV_CN_MAX channels) matrices.
   */
  SMat(const int32_t aRows, const int32_t aCols, const uint32_t aType);

  /**
   * @overload
   *
   * @param[in]      aNDims       Array dimensionality.
   * @param[in]      apSizes       Array of integers specifying an
   *                              n-dimensional array shape.
   * @param[in]      aType        Array type. Use CV_8UC1, ..., CV_64FC4 to
   *                              create 1-4 channel matrices, or
   *                              CV_8UC(n), ..., CV_64FC(n) to create
   *                              multi-channel (up to CV_CN_MAX channels)
   *                              matrices.
   */
  SMat(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType);

  /**
   * @overload
   *
   * @param[in]      aMatrix      Array that (as a whole or partly) is
   *                              assigned to the constructed matrix.
   *                              No data is copied by these constructors.
   *                              Instead, the header pointing to aMatrix data
   * or its sub-array is constructed and associated with it. The reference
   * counter, if any, is incremented. So, when you
   * modify the matrix formed using such a
   * constructor, you also modify the corresponding
   * elements of aMatrix . If you want to have an
   * independent copy of the sub-array, use
   * SMat::clone() .
   */
  SMat(const vsdk::SMat& aMatrix);

  /**
   * @overload
   *
   * @param[in]      aRows        Number of rows in a 2D array.
   * @param[in]      aCols        Number of columns in a 2D array.
   * @param[in]      aType        Array type. Use CV_8UC1, ..., CV_64FC4 to
   *                              create 1-4 channel matrices, or
   *                              CV_8UC(n), ..., CV_64FC(n) to create
   *                              multi-channel (up to CV_CN_MAX channels)
   *                              matrices.
   * @param[in]      apData       Pointer to the user data. Matrix
   *                              constructors that take data and step
   *                              parameters do not allocate matrix data.
   *                              Instead, they just initialize the matrix
   *                              header that points to the specified data,
   *                              which means that no data is copied. This
   *                              operation is very efficient and can be used
   *                              to process external data using OpenCV
   *                              functions. The external data is not
   *                              automatically deallocated, so
   *                              you should take care of it.
   * @param[in]      aStep        Number of bytes each matrix row occupies.
   *                              The value should include the padding bytes
   *                              at the end of each row, if any. If the
   *                              parameter is missing (set to AUTO_STEP ),
   *                              no padding is assumed and the actual aStep
   *                              is calculated as cols*elemSize().
   *                              See SMat::elemSize.
   */
  SMat(const int32_t  aRows,
       const int32_t  aCols,
       const uint32_t aType,
       void* const    apData,
       uint64_t       aStep = vsdk::SMat::AUTO_STEP);

  /**
   * @overload
   *
   * @param[in]      aNDims       Array dimensionality.
   * @param[in]      apSizes      Array of integers specifying an
   *                              n-dimensional array shape.
   * @param[in]      aType        Array type. Use CV_8UC1, ..., CV_64FC4 to
   *                              create 1-4 channel matrices, or
   *                              CV_8UC(n), ..., CV_64FC(n) to create
   *                              multi-channel (up to CV_CN_MAX channels)
   *                              matrices.
   * @param[in]      apData       Pointer to the user data. Matrix
   *                              constructors that take data and step
   *                              parameters do not allocate matrix data.
   *                              Instead, they just initialize the matrix
   *                              header that points to the specified
   *                              data, which means that no data is copied.
   *                              This operation is very efficient and can be
   *                              used to process external data using OpenCV
   *                              functions. The external data is not
   *                              automatically deallocated, so
   *                              you should take care of it.
   * @param[in]      aSteps       Array of aNDims-1 steps in case of a
   *                              multi-dimensional array (the last step is
   *                              always set to the element size). If not
   *                              specified, the matrix is assumed to be
   *                              continuous.
   */
  SMat(const int32_t         aNDims,
       const int32_t* const  apSizes,
       const uint32_t        aType,
       void* const           apData,
       const uint64_t* const aSteps = 0U);

  /**
   * @overload
   *
   * @param[in]      aMatrix      Array that (as a whole or partly) is
   *                              assigned to the constructed matrix. No data
   *                              is copied by these constructors. Instead,
   *                              the header pointing to aMatrix data or its
   *                              sub-array is constructed and associated
   *                              with it. The reference counter, if any, is
   *                              incremented. So, when you modify the matrix
   *                              formed using such a constructor, you also
   *                              modify the corresponding elements of aMatrix .
   *                              If you want to have an independent copy of
   *                              the sub-array, use SMat::clone() .
   * @param[in]      aRowRange    Range of the aMatrix rows to take. As usual,
   *                              the range start is inclusive and the range
   *                              end is exclusive. Use Range::all() to take
   *                              all the rows.
   * @param[in]      aColRange    Range of the aMatrix columns to take. Use
   *                              Range::all() to take all the columns.
   */
  SMat(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange = vsdk::Range::all());

  /**
   * @overload
   *
   * @param[in]      aMatrix      Array that (as a whole or partly) is
   *                              assigned to the constructed matrix. No data
   *                              is copied by these constructors. Instead,
   *                              the header pointing to aMatrix data or its
   *                              sub-array is constructed and associated
   *                              with it. The reference counter, if any, is
   *                              incremented. So, when you modify the matrix
   *                              formed using such a constructor, you also
   *                              modify the corresponding elements of
   * aMatrix.If you want to have an independent copy of the sub-array, use
   * SMat::clone() .
   * @param[in]      aRoi         Region of interest.
   */
  SMat(const vsdk::SMat& aMatrix, const vsdk::Rect& aRoi);

  /**
   * @overload
   *
   * @param[in]      aMatrix      Array that (as a whole or partly) is
   *                              assigned to the constructed matrix. No data
   *                              is copied by these constructors. Instead,
   *                              the header pointing to aMatrix data or its
   *                              sub-array is constructed and associated
   *                              with it. The reference counter, if any, is
   *                              incremented. So, when you modify the matrix
   *                              formed using such a constructor, you also
   *                              modify the corresponding elements of
   * aMatrix.If you want to have an independent copy of the sub-array, use
   * SMat::clone() .
   * @param[in]      apRanges     Array of selected ranges of aMatrix along each
   *                              dimensionality.
   */
  SMat(const vsdk::SMat& aMatrix, const vsdk::Range* const aRanges);

  /**
   * @brief          Destructor - calls release()
   */
  ~SMat();

  /**
   * @brief          assignment operators
   *
   * These are available assignment operators. Since they all are very
   * different, make sure to read the operator parameters description.
   *
   * @param[in]      aMatrix      Assigned, right-hand-side matrix. Matrix
   *                              assignment is an O(1) operation. This means
   *                              that no data is copied but the data is
   *                              shared and the reference counter, if any,
   *                              is incremented. Before assigning new data,
   *                              the old data is de-referenced via
   *                              SMat::release.
   */
  vsdk::SMat& operator=(const vsdk::SMat& aMatrix);

  /**
   * @brief          Creates a matrix header for the specified matrix row.
   *
   * @param[in]      aRowIndex            A 0-based row index.
   */
  vsdk::SMat row(const int32_t aRowIndex) const;

  /**
   * @brief          Creates a matrix header for the specified matrix column.
   *
   * @param[in]      aColIndex            A 0-based column index.
   */
  vsdk::SMat col(const int32_t aColIndex) const;

  /**
   * @brief          Creates a matrix header for the specified row span.
   *
   * @param[in]      aStartRow    An inclusive 0-based start index of the row
   *                              span.
   * @param[in]      aEndRow      An exclusive 0-based ending index of the
   *                              row span.
   */
  vsdk::SMat rowRange(const int32_t aStartRow, const int32_t aEndRow) const;

  /**
   * @overload
   *
   * @param[in]      aRange       Range structure containing both the start
   *                              and the end indices.
   */
  vsdk::SMat rowRange(const vsdk::Range& aRange) const;

  /**
   * @brief          Creates a matrix header for the specified column span.
   *
   * @param[in]      aStartCol    An inclusive 0-based start index of the
   *                              column span.
   * @param[in]      aEndCol      An exclusive 0-based ending index of the
   *                              column span.
   */
  vsdk::SMat colRange(const int32_t aStartCol, const int32_t aEndCol) const;

  /**
   * @overload
   *
   * @param[in]      aRange       Range structure containing both the start
   *                              and the end indices.
   */
  vsdk::SMat colRange(const vsdk::Range& aRange) const;

  /**
   * @brief          Extracts a diagonal from a matrix
   *
   * @param[in]      aDiag        index of the diagonal, with the following
   *                              values:
   *                               - `aDiag=0` is the main diagonal.
   *                               - `aDiag>0` is a diagonal from the lower
   * half. For example, aDiag=1 means the diagonal is set immediately below the
   * main one.
   *                               - `aDiag<0` is a diagonal from the upper
   * half. For example, aDiag=-1 means the diagonal is set immediately above the
   * main one.
   */
  vsdk::SMat diag(const int32_t aDiag = 0) const;

  /**
   * @brief          Allocates new array data if needed.
   *
   * @param[in]      aRows         New number of rows.
   * @param[in]      aCols         New number of columns.
   * @param[in]      aType         New matrix type.
   */
  void create(const int32_t aRows, const int32_t aCols, uint32_t aType);

  /**
   * @overload
   *
   * @param[in]      aNDims        New array dimensionality.
   * @param[in]      apSizes        Array of integers specifying a new array
   *                               shape.
   * @param[in]      aType         New matrix type.
   */
  void create(const int32_t aNDims, const int32_t* const apSizes, uint32_t aType);

  /**
   * @brief          Increments the reference counter.
   */
  void addref() const;

  /**
   * @brief          Decrements the reference counter and deallocates the
   *                 matrix if needed.
   */
  void release();

  /**
   * @brief          Deallocates the matrix data
   */
  void deallocate();

  /**
   * @brief          Internal use function; properly re-allocates _size,
   *                 _step arrays
   */
  void copySize(const vsdk::SMat& aMatrix);

  /**
   * @brief          Extracts a rectangular submatrix.
   *
   * @param[in]      aRowRange     Start and end row of the extracted
   *                               submatrix. The upper boundary is not
   *                               included. To select all the rows, use
   *                               Range::all().
   * @param[in]      aColRange     Start and end column of the extracted
   *                               submatrix. The upper boundary is not
   *                               included. To select all the columns,
   *                               use Range::all().
   */
  vsdk::SMat operator()(const vsdk::Range aRowRange, const vsdk::Range aColRange) const;

  /**
   * @overload
   *
   * @param[in]      aRoi          Extracted submatrix specified as
   *                               a rectangle.
   */
  vsdk::SMat operator()(const vsdk::Rect& aRoi) const;

  /**
   * @overload
   *
   * @param[in]      aRanges       Array of selected ranges along each array
   *                               dimension.
   */
  vsdk::SMat operator()(const vsdk::Range* const apRanges) const;

  /**
   * @brief          Reports whether the matrix is continuous or not.
   */
  bool isContinuous() const;

  /**
   * @brief          Returns true if the matrix is a submatrix of another
   *                 matrix
   */
  bool isSubmatrix() const;

  /**
   * @brief          Returns the matrix element size in bytes.
   */
  uint64_t elemSize() const;

  /**
   * @brief          Returns the size of each matrix element channel in
   *                 bytes.
   */
  uint64_t elemSize1() const;

  /**
   * @brief          Returns the type of a matrix element.
   */
  uint32_t type() const;

  /**
   * @brief          Returns the depth of a matrix element.
   */
  uint32_t depth() const;

  /**
   * @brief          Returns the number of matrix channels.
   */
  uint32_t channels() const;

  /**
   * @brief          Returns a normalized step.
   */
  uint64_t step1(const int32_t aIndex = 0) const;

  /**
   * @brief          Returns true if the array has no elements.
   */
  bool empty() const;

  /**
   * @brief          Returns the total number of array elements.
   */
  uint64_t total() const;

  /**
   * @brief          Returns N if the matrix is 1-channel (N x ptdim) or
   *                 ptdim-channel (1 x N) or (N x 1); negative number
   *                 otherwise
   */
  int32_t checkVector(const int32_t aElemChannels, const int32_t aDepth = -1, const bool aReqContinuous = true) const;

  /**
   * @brief          Returns a pointer to the specified matrix row.
   *
   * @param[in]      aIndex0            A 0-based row index.
   */
  const uint8_t* ptr(const uint32_t aIndex0 = 0U) const;

  /**
   * @overload
   */
  const uint8_t* ptr(const uint32_t aIndex0, const uint32_t aIndex1) const;

  /**
   * @overload
   */
  const uint8_t* ptr(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2) const;

  /**
   * @overload
   */
  const uint8_t* ptr(const uint32_t* const aIndex) const;

  /**
   * @overload
   */
  template < typename Type_t >
  Type_t* ptr(uint32_t aIndex0 = 0U);

  /**
   * @overload
   */
  template < typename Type_t >
  const Type_t* ptr(uint32_t aIndex0 = 0U) const;

  /**
   * @overload
   */
  template < typename Type_t >
  Type_t* ptr(uint32_t aIndex0, uint32_t aIndex1);

  /**
   * @overload
   */
  template < typename Type_t >
  const Type_t* ptr(uint32_t aIndex0, uint32_t aIndex1) const;

  /**
   * @overload
   */
  template < typename Type_t >
  Type_t* ptr(uint32_t aIndex0, uint32_t aIndex1, uint32_t aIndex2);

  /**
   * @overload
   */
  template < typename Type_t >
  const Type_t* ptr(uint32_t aIndex0, uint32_t aIndex1, uint32_t aIndex2) const;

  /**
   * @brief          Returns a reference to the specified array element.
   *
   * @param[in]      aIndex0            Index along the dimension 0
   */
  template < typename Type_t >
  Type_t& at(const uint32_t aIndex0 = 0U);

  /**
   * @overload
   *
   * @param[in]      aIndex0            Index along the dimension 0
   */
  template < typename Type_t >
  const Type_t& at(const uint32_t aIndex0 = 0U) const;

  /**
   * @overload
   *
   * @param[in]      aIndex0            Index along the dimension 0
   * @param[in]      aIndex1            Index along the dimension 1
   */
  template < typename Type_t >
  Type_t& at(const uint32_t aIndex0, const uint32_t aIndex1);

  /**
   * @overload
   *
   * @param[in]      aIndex0            Index along the dimension 0
   * @param[in]      aIndex1            Index along the dimension 1
   */
  template < typename Type_t >
  const Type_t& at(const uint32_t aIndex0, const uint32_t aIndex1) const;

  /**
   * @overload
   *
   * @param[in]      aIndex0            Index along the dimension 0
   * @param[in]      aIndex1            Index along the dimension 1
   * @param[in]      aIndex2            Index along the dimension 2
   */
  template < typename Type_t >
  Type_t& at(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2);

  /**
   * @overload
   *
   * @param[in]      aIndex0            Index along the dimension 0
   * @param[in]      aIndex1            Index along the dimension 1
   * @param[in]      aIndex2            Index along the dimension 2
   */
  template < typename Type_t >
  const Type_t& at(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2) const;

  /**
   * @overload
   *
   * @param[in]      aIndex            Array of SMat::dims indices.
   */
  template < typename Type_t >
  Type_t& at(const uint32_t* aIndex);

  /**
   * @overload
   *
   * @param[in]      aIndex            Array of SMat::dims indices.
   */
  template < typename Type_t >
  const Type_t& at(const uint32_t* aIndex) const;

  /**
   * @overload
   *
   * @brief          Special versions for 2D arrays (especially convenient
   *                 for referencing image pixels)
   *
   * @param[in]      aPoint             Element position specified as
   * Point(j,i).
   */
  template < typename Type_t >
  Type_t& at(vsdk::Point aPoint);

  /**
   * @overload
   *
   * @brief          Special versions for 2D arrays (especially convenient
   *                 for referencing image pixels)
   *
   * @param[in]      aPoint             Element position specified as
   * Point(j,i).
   */
  template < typename Type_t >
  const Type_t& at(vsdk::Point aPoint) const;

  /**
   * @brief          includes several bit-fields:
   *                  - the magic signature
   *                  - continuity flag
   *                  - depth
   *                  - number of channels
   */
  uint32_t flags;

  /**
   * @brief          the matrix dimensionality, >= 2
   */
  int32_t dims;

  /**
   * @brief          the number of rows and columns or (-1, -1) when
   *                 the matrix has more than 2 dimensions
   */
  int32_t rows;
  int32_t cols;

  /**
   * @brief          pointer to the data
   */
  uint8_t* data;

  /**
   * @brief          helper fields used in locateROI and adjustROI
   */
  const uint8_t* datastart;
  const uint8_t* dataend;
  const uint8_t* datalimit;

  /**
   * @brief          custom allocator
   */
  vsdk::SUMatAllocator* allocator;

  /**
   * @brief          Interaction with SUMat
   */
  vsdk::UMatData* u;

  /**
   * @brief          Sizes along each dimension (in elements)
   */
  vsdk::MatSize size;

  /**
   * @brief          Size in bytes to the next element along each dimansion
   *                 (size in bytes)
   */
  vsdk::MatStep step;

  /*****************************************************************************
   *                                OPENCV DIFFERENCES
   *****************************************************************************/
  /**
   * NOT IMPLEMENTED
   *     void locateROI( Size& wholeSize, Point& ofs ) const;
   *     SMat& adjustROI( int dtop, int dbottom, int dleft, int dright );
   *     SMat(Size size, int type);
   *     SMat(int rows, int cols, int type, const Scalar& s);
   *     SMat(Size size, int type, const Scalar& s);
   *     SMat(int ndims, const int* sizes, int type, const Scalar& s);
   *     SMat(Size size, int type, void* data, size_t step=AUTO_STEP);
   *     template<typename Type_t> explicit SMat(const std::vector<Type_t>& vec,
   *                                         bool copyData=false);
   *     template<typename Type_t, int n> explicit SMat(const Vec<Type_t, n>&
   * vec, bool copyData=true); template<typename Type_t, int m, int n> explicit
   * SMat( const Matx<Type_t, m, n>& mtx, bool copyData=true);
   *     template<typename Type_t> explicit SMat(
   *                      const Point_<Type_t>& pt, bool copyData=true);
   *     template<typename Type_t> explicit SMat(
   *                      const Point3_<Type_t>& pt, bool copyData=true);
   *     template<typename Type_t> explicit SMat(
   *                      const MatCommaInitializer_<Type_t>& commaInitializer);
   *     explicit SMat(const cuda::GpuMat& m);
   *     SMat& operator = (const MatExpr& expr);
   *     static SMat diag(const SMat& d);
   *     SMat clone() const;
   *     void copyTo( OutputArray m ) const;
   *     void copyTo( OutputArray m, InputArray mask ) const;
   *     void convertTo( OutputArray m, int rtype, double alpha=1,
   *                     double beta=0 ) const;
   *     void assignTo( SMat& m, int type=-1 ) const;
   *     SMat& operator = (const Scalar& s);
   *     SMat& setTo(InputArray value, InputArray mask=noAr(ray());
   *     SMat reshape(int cn, int rows=0) const;
   *     SMat reshape(int cn, int newndims, const int* newsz) const;
   *     MatExpr t() const;
   *     MatExpr inv(int method=DECOMP_LU) const;
   *     MatExpr mul(InputArray m, double scale=1) const;
   *     SMat cross(InputArray m) const;
   *     double dot(InputArray m) const;
   *     static MatExpr zeros(int rows, int cols, int type);
   *     static MatExpr zeros(Size size, int type);
   *     static MatExpr zeros(int ndims, const int* sz, int type);
   *     static MatExpr ones(int rows, int cols, int type);
   *     static MatExpr ones(Size size, int type);
   *     static MatExpr ones(int ndims, const int* sz, int type);
   *     static MatExpr eye(int rows, int cols, int type);
   *     static MatExpr eye(Size size, int type);
   *     void reserve(size_t sz);
   *     void resize(size_t sz);
   *     void resize(size_t sz, const Scalar& s);
   *     void push_back_(const void* elem);
   *     template<typename Type_t> void push_back(const Type_t& elem);
   *     template<typename Type_t> void push_back(const Mat_<Type_t>& elem);
   *     void push_back(const SMat& m);
   *     void pop_back(size_t nelems=1);
   *     template<typename Type_t> operator std::vector<Type_t>() const;
   *     template<typename Type_t, int n> operator Vec<Type_t, n>() const;
   *     template<typename Type_t, int m, int n> operator Matx<Type_t, m, n>()
   *         const;
   *     template<int n> uchar* ptr(const Vec<int, n>& idx);
   *     template<int n> const uchar* ptr(const Vec<int, n>& idx) const;
   *     template<typename Type_t, int n> Type_t* ptr(const Vec<int, n>& idx);
   *     template<typename Type_t, int n> const Type_t* ptr(
   *         const Vec<int, n>& idx) const;
   *     template<typename Type_t, int n> Type_t& at(const Vec<int, n>& idx);
   *     template<typename Type_t, int n> const Type_t& at(
   *         const Vec<int, n>& idx) const;
   *     template<typename Type_t> MatIterator_<Type_t> begin();
   *     template<typename Type_t> MatConstIterator_<Type_t> begin() const;
   *     template<typename Type_t> MatIterator_<Type_t> end();
   *     template<typename Type_t> MatConstIterator_<Type_t> end() const;
   *     template<typename Type_t, typename Functor> void forEach(
   *         const Functor& operation);
   *     template<typename Type_t, typename Functor> void forEach(
   *         const Functor& operation) const;
   *     template<typename Type_t, typename Functor> void forEach_impl(
   *         const Functor& operation);
   *     template<typename Type_t> Type_t* ptr(const int32_t* idx);
   *     template<typename Type_t> const Type_t* ptr(const int32_t* idx) const;
   *     void create(Size size, int32_t type);
   */

private:
  /**
   * Helper class for creation of allocator.
   */
  void createAllocator(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType);
  void setDataAndFlags(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange);
  void setRange(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange);
}; /* class SMat */

/**
 * @brief          n-ary multi-dimensional array iterator
 * 
 * Use the class to implement unary, binary, and, generally, n-ary element-wise operations on multi-dimensional arrays. Some of the arguments of 
 * an n-ary function may be continuous arrays, some may be not. It is possible to use conventional MatIterator 's for each array but incrementing 
 * all of the iterators after each small operations may be a big overhead. In this case consider using NAryMatIterator to iterate through several 
 * matrices simultaneously as long as they have the same geometry (dimensionality and all the dimension sizes are the same). On each iteration 
 * it.planes[0], it.planes[1],... will be the slices of the corresponding matrices.
 */
class NAryMatIterator
{
public:
  /**
   * @brief          The default constructor
   * The default constructor
   * Initializes all member function to its default values, i.e. NULL or zeroes
   */
  NAryMatIterator();

  /**
   * @brief          The full constructor taking arbitrary number of n-dim
   *                 matrices
   * 
   * The full constructor taking arbitrary number of n-dim matrices
   * Internally, it uses the init function declared bellow in this code with
   * specific parameters.
   *
   * @param[in]      apArrays
   * @param[in]      apBuffers
   * @param[in]      aNArrays
   */
  NAryMatIterator(const vsdk::SMat** const apArrays, uint8_t** const apBuffers, const int32_t aNArrays = -1);

  /**
   * @brief          The full constructor taking arbitrary number of n-dim
   *                 matrices
   * 
   * The full constructor taking arbitrary number of n-dim matrices
   * Internally, it uses the init function declared bellow in this code with
   * specific parameters.
   *
   * @param[in]      apArrays
   * @param[in]      apPlanes
   * @param[in]      aNArrays
   */
  NAryMatIterator(const vsdk::SMat** const apArrays, vsdk::SMat* const apPlanes, const int32_t aNArrays = -1);

  /**
   * The separate iterator initialization method
   * @param apArrays  Arrays of matrices as an input for iterator
   * @param apPlanes  Planes of matrices
   * @param apBuffers Array of buffers
   * @param aNArrays  Number of arrays
   */
  void init(const vsdk::SMat** const apArrays,
            vsdk::SMat* const        apPlanes,
            uint8_t** const          apBuffers,
            const int32_t            aNArrays = -1);

  /**
   * @brief          Proceeds to the next plane of every iterated matrix
   */
  NAryMatIterator& operator++();

  /**
   * @brief          Proceeds to the next plane of every iterated matrix
   *                 (postfix increment operator)
   */
  NAryMatIterator operator++(int32_t);

  /**
   * @brief          The iterated apArrays
   */
  const vsdk::SMat** arrays;

  /**
   * @brief          The current planes
   */
  vsdk::SMat* planes;

  /**
   * @brief          Data pointers
   */
  uint8_t** ptrs;

  /**
   * @brief          The number of arrays
   */
  int32_t narrays;

  /**
   * @brief          The number of hyper-planes that the iterator steps
   *                 through
   */
  size_t nplanes;

  /**
   * @brief          The size of each segment (in elements)
   */
  size_t size;

  /**
   * @brief
   */
  int32_t iterdepth;

  /**
   * @brief
   */
  size_t idx;
}; /* class NAryMatIterator */

} /* namespace vsdk */

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline Type_t* vsdk::SMat::ptr(uint32_t aIndex0)
{
  return (Type_t*)(data + step.p[0] * aIndex0);
}

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline const Type_t* vsdk::SMat::ptr(uint32_t aIndex0) const
{
  return (const Type_t*)(data + step.p[0] * aIndex0);
}

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline Type_t* vsdk::SMat::ptr(uint32_t aIndex0, uint32_t aIndex1)
{
  return (Type_t*)(data + aIndex0 * step.p[0] + aIndex1 * step.p[1]);
}

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline const Type_t* vsdk::SMat::ptr(uint32_t aIndex0, uint32_t aIndex1) const
{
  return (const Type_t*)(data + aIndex0 * step.p[0] + aIndex1 * step.p[1]);
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline Type_t& vsdk::SMat::at(const uint32_t aIndex0, const uint32_t aIndex1)
{
  return ((Type_t*)(data + step.p[0] * aIndex0))[aIndex1];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline const Type_t& vsdk::SMat::at(const uint32_t aIndex0, const uint32_t aIndex1) const
{
  return ((const Type_t*)(data + step.p[0] * aIndex0))[aIndex1];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline Type_t& vsdk::SMat::at(vsdk::Point aPoint)
{
  return ((Type_t*)(data + step.p[0] * aPoint.y))[aPoint.x];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline const Type_t& vsdk::SMat::at(vsdk::Point aPoint) const
{
  return ((const Type_t*)(data + step.p[0] * aPoint.y))[aPoint.x];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline Type_t& vsdk::SMat::at(const uint32_t aIndex0)
{
  Type_t*  lpRetVal = NULL;
  uint32_t lIndex   = 0;
  if(isContinuous() || size.p[0] == 1)
  {
    lpRetVal = (Type_t*)data;
    lIndex   = aIndex0;
  }
  else
  {
    if(size.p[1] == 1)
    {
      lpRetVal = (Type_t*)(data + step.p[0] * aIndex0);
      lIndex   = 0;
    }
    else
    {
      uint32_t lIt = aIndex0 / cols;
      uint32_t j   = aIndex0 - lIt * cols;
      lpRetVal     = ((Type_t*)(data + step.p[0] * lIt));
      lIndex       = j;
    }
  }
  return lpRetVal[lIndex];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline const Type_t& vsdk::SMat::at(const uint32_t aIndex0) const
{
  const Type_t* lpRetVal = NULL;
  uint32_t      lIndex   = 0;

  if(isContinuous() || size.p[0] == 1)
  {
    lpRetVal = (const Type_t*)data;
    lIndex   = aIndex0;
  }
  else
  {
    if(size.p[1] == 1)
    {
      lpRetVal = (const Type_t*)(data + step.p[0] * aIndex0);
      lIndex   = 0;
    }
    else
    {
      uint32_t lIt = aIndex0 / cols;
      uint32_t j   = aIndex0 - lIt * cols;
      lpRetVal     = ((const Type_t*)(data + step.p[0] * lIt));
      lIndex       = j;
    }
  }
  return lpRetVal[lIndex];
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline Type_t& vsdk::SMat::at(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2)
{
  return *(Type_t*)ptr(aIndex0, aIndex1, aIndex2);
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline const Type_t& vsdk::SMat::at(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2) const
{
  return *(const Type_t*)ptr(aIndex0, aIndex1, aIndex2);
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline Type_t& vsdk::SMat::at(const uint32_t* aIndex)
{
  return *(Type_t*)ptr(aIndex);
}

/**
 * Returns a reference to the specified array element.
 */
template < typename Type_t >
inline const Type_t& vsdk::SMat::at(const uint32_t* aIndex) const
{
  return *(const Type_t*)ptr(aIndex);
}

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline Type_t* vsdk::SMat::ptr(uint32_t aIndex0, uint32_t aIndex1, uint32_t aIndex2)
{
  return (Type_t*)(data + aIndex0 * step.p[0] + aIndex1 * step.p[1] + aIndex2 * step.p[2]);
}

/**
 * Returns a pointer to the specified matrix row.
 */
template < typename Type_t >
inline const Type_t* vsdk::SMat::ptr(uint32_t aIndex0, uint32_t aIndex1, uint32_t aIndex2) const
{
  return (const Type_t*)(data + aIndex0 * step.p[0] + aIndex1 * step.p[1] + aIndex2 * step.p[2]);
}

void        finalizeHdrMat(vsdk::SMat& aMatrix);
void        updateContinuityFlagMat(vsdk::SMat& aMatrix);
void        setSizeMat(vsdk::SMat&           aMatrix,
                       const int32_t         aDims,
                       const int32_t* const  apSz,
                       const uint64_t* const apSteps,
                       const bool            aAutoSteps = false);
void        cpyMatSizes(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix);
void        cpyMatData(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix);
bool        createNewMatrix(const vsdk::SMat& aDst, const int32_t aRows, const int32_t aCols, const uint32_t aType);
void        cpyMatStep(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix);
inline void initializeIter(vsdk::NAryMatIterator& aIter, int32_t& aIt0, int32_t& aDims, int32_t& aD1);
inline void iterateThroughDims(const vsdk::SMat& aMat, int32_t& aD1, const int32_t& aDims);
inline bool setDataIter(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, const int32_t& aIt);
inline void stepInlineValuesIter(vsdk::NAryMatIterator& aIter,
                                 const vsdk::SMat&      aA,
                                 const int32_t&         aDims,
                                 const int32_t&         aD1);
inline void initComputeNPlanesIter(vsdk::NAryMatIterator& aIter,
                                   const int32_t&         aIt0,
                                   const int32_t&         aDims,
                                   const int32_t&         aD1);
inline void initComputePlanesIter(vsdk::NAryMatIterator& aIter, int32_t& aIt, const int32_t& aIt0, const int32_t& aD1);
inline bool initAllocateMatIter(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, const int32_t& aIt);
inline void setData(const vsdk::NAryMatIterator& aIter, uint8_t* const& apData, const int32_t& aIt);
inline void iterateIterAux(const vsdk::NAryMatIterator& aIter, const vsdk::SMat& aA, uint8_t*& apData, int32_t& aIdx);
inline void initPlanesIter(const vsdk::NAryMatIterator& aIter);
inline void initPointersIter(const vsdk::NAryMatIterator& aIter);
inline void initSetAllDataIter(const vsdk::NAryMatIterator& aIter);
inline void iterateIter(const vsdk::NAryMatIterator& aIter);

#endif /* UMATMAT_HPP */
