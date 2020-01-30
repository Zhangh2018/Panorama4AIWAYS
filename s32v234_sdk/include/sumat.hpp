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

#ifndef SUMAT_HPP
#define SUMAT_HPP

#include "../libs/utils/sumat/include/sumat_defines.hpp"
#include "../libs/utils/sumat/include/sumat_types.hpp"
#include "../libs/utils/sumat/include/sumat_data.hpp"
#include "../libs/utils/sumat/include/sumat_mat.hpp"

namespace vsdk
{
/**
 * @brief          A SUMat class which is derived from cv::UMat and is fully
 *                 compatible with it.
 * @details        A custom implementation of the SUMat keeps compatibility with
 *                 cv::UMat and is able to be retyped back and forth in order
 *                 to be used in OpenCV functions as well as in vsdk ApexCV and
 *                 others.
 */
class SUMat
{
public:
  static const uint32_t MAGIC_VAL       = 0x42FF0000U;
  static const uint32_t AUTO_STEP       = 0x0U;
  static const uint32_t CONTINUOUS_FLAG = 16384U;
  static const uint32_t SUBMATRIX_FLAG  = 32768U;

  static const uint32_t MAGIC_MASK = 0xFFFF0000U;
  static const uint32_t TYPE_MASK  = 0x00000FFFU;
  static const uint32_t DEPTH_MASK = 0x7U;

  /**
   * @brief          Default constructor
   * @details
   *
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  SUMat(const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Constructs 2D matrix of the specified size and type
   *
   * @param[in]      aRows        Number of rows (height)
   * @param[in]      aCols        Number of cols (width)
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   *                              CV_32SC(12) etc.
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  SUMat(const int32_t              aRows,
        const int32_t              aCols,
        const uint32_t             aType,
        const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Constructs n-dimensional matrix
   *
   * @param[in]      aNDims       Number of dimensions
   * @param[in]      apSizes      Number of bytes in each dimension
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   *                              CV_32SC(12) etc.
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  SUMat(const int32_t              aNDims,
        const int32_t* const       apSizes,
        const uint32_t             aType,
        const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Copy constructor
   *
   * @param[in]      aMatrix      Original matrix
   */
  SUMat(const vsdk::SUMat& aMatrix);

  /**
   * @brief          Creates a matrix header for a part of the bigger matrix
   *
   * @param[in]      aMatrix      Original matrix
   * @param[in]      aRowRange    Row range
   * @param[in]      aColRange    Column range
   */
  SUMat(const vsdk::SUMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange = Range::all());

  /**
   * @brief          Creates a matrix header for a part of the bigger matrix
   *
   * @param[in]      aMatrix      Original matrix
   * @param[in]      aRoi         ROI specified by a Rectangle class
   */
  SUMat(const vsdk::SUMat& aMatrix, const vsdk::Rect& aRoi);

  /**
   * @brief          Creates a matrix header for a part of the bigger matrix
   *
   * @param[in]      aMatrix      Original matrix
   * @param[in]      apRanges     Ranges specified by a list
   */
  SUMat(const vsdk::SUMat& aMatrix, const vsdk::Range* const apRanges);

  /**
   * @brief          Returns a SUMat from specified ROI
   *
   * @param[in]      apRanges     Ranges specified by a list
   *
   * @return         ROIed SUMat
   */
  vsdk::SUMat operator()(const vsdk::Range* const apRanges) const;

  /**
   * @brief          Assigned operator
   *
   * @param[in]      aMatrix      Original matrix
   *
   * \return
   *    - Assigned matrix
   */
  vsdk::SUMat& operator=(const SUMat& aMatrix);

  /**
   * @brief          Assign ROI operator
   *
   * @param[in]      aRowRange    Original matrix row range
   * @param[in]      aColRange    Original matrix column range
   *
   * @return         Assigned submatrix
   */
  vsdk::SUMat operator()(const vsdk::Range aRowRange, const vsdk::Range aColRange) const;

  /**
   * @brief          Assign ROI operator
   *
   * @param[in]      aRoi         ROI rectangle
   *
   * @return         Assigned submatrix
   */
  vsdk::SUMat operator()(const vsdk::Rect& aRoi) const;

  /**
   * @brief          Destructor - calls release()
   *
   * @param[in]      aMatrix      Original matrix
   */
  ~SUMat();

  /**
   * @brief          Allocates new matrix data unless the matrix already has
   *                 specified size and type.
   *                 previous data is unreferenced if needed.
   *
   * @param[in]      aRows        Number of rows (height)
   * @param[in]      aCols        Number of cols (width)
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   * CV_32SC(12) etc.
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  void create(const int32_t              aRows,
              const int32_t              aCols,
              uint32_t                   aType,
              const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Allocates new matrix data unless the matrix already has
   *                 specified size and type.
   *                 previous data is unreferenced if needed.
   *
   * @param[in]      aNDims       Number of dimensions
   * @param[in]      apSizes       Number of bytes in each dimension
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   * CV_32SC(12) etc.
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  void create(const int32_t              aNDims,
              const int32_t* const       apSizes,
              uint32_t                   aType,
              const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Increases the reference counter; use with care to avoid
   *                 memleaks
   */
  void addref() const;

  /**
   * @brief          Returns element type, similar to
   *                 CV_MAT_TYPE(cvmat->type)
   *
   * @return         Element type (similar to CV_MAT_TYPE)
   */
  uint32_t type() const;

  /**
   * @brief          Returns the total number of matrix elements
   *
   * @return         Total number of matrix elements
   */
  uint64_t total() const;

  /**
   * @brief          Returns element size in bytes
   *
   * @return         Total element size in bytes
   */
  uint64_t elemSize() const;

  /**
   * @brief          Returns true iff the matrix data is continuous
   *                 (i.e. when there are no gaps between successive rows).
   *                 similar to CV_IS_MAT_CONT(cvmat->type)
   *
   * @return         Returns true iff the matrix data is continuous
   */
  bool isContinuous() const;

  /**
   * @brief          Returns true if the matrix is a submatrix of another
   *                 matrix
   *
   * @return         Returns true if the matrix is a submatrix of another
   *                 matrix
   */
  bool isSubmatrix() const;

  /**
   * @brief          Returns the size of element channel in bytes.
   *
   * @return         Returns the size of element channel in bytes.
   */
  uint32_t elemSize1() const;

  /**
   * @brief          Returns element type, similar to
   *                 CV_MAT_DEPTH(cvmat->type)
   *
   * @return         Returns element type, similar to
   *                 CV_MAT_DEPTH(cvmat->type)
   */
  uint32_t depth() const;

  /**
   * @brief          Returns element type, similar to CV_MAT_CN(cvmat->type)
   *
   * @return         Returns element type, similar to CV_MAT_CN(cvmat->type)
   */
  uint32_t channels() const;

  /**
   * @brief          Returns step/elemSize1()
   *
   * @return         Returns step/elemSize1()
   */
  uint64_t step1(const int32_t aI = 0) const;

  /**
   * @brief          Returns true if matrix data is NULL
   *
   * @return         Returns true if matrix data is NULL
   */
  bool empty() const;

  /**
   * @brief          Internal use function; properly re-allocates _size,
   *                 _step arrays
   */
  void copySize(const vsdk::SUMat& aMatrix);

  /**
   * @brief          Returns a new matrix header for the specified row
   *
   * @param[in]      aY           Index of a row
   *
   * @return         New row matrix structure
   */
  vsdk::SUMat row(const int32_t aY) const;

  /**
   * @brief          Returns a new matrix header for the specified column
   *
   * @param[in]      aX           Index of a column
   *
   * @return         New column matrix structure
   */
  vsdk::SUMat col(const int32_t aX) const;

  /**
   * @brief          Returns a new matrix header for the specified row span
   *
   * @param[in]      aStartRow    Starting row
   * @param[in]      aEndRow      End row
   *
   * @return         New matrix structure
   */
  vsdk::SUMat rowRange(const int32_t aStartRow, const int32_t aEndRow) const;

  /**
   * @brief          Returns a new matrix header for the specified row span
   *
   * @param[in]      aRange       Row range
   *
   * @return         New matrix structure
   */
  vsdk::SUMat rowRange(const vsdk::Range& aRange) const;

  /**
   * @brief          Returns a new matrix header for the specified column
   *                 span
   *
   * @param[in]      aStartCol    Starting column
   * @param[in]      aEndCol      End column
   *
   * @return         New matrix structure
   */
  vsdk::SUMat colRange(const int32_t aStartCol, const int32_t aEndCol) const;

  /**
   * @brief          Returns a new matrix header for the specified column
   *                 span
   *
   * @param[in]      aRange       Column range
   *
   * @return         New matrix structure
   */
  vsdk::SUMat colRange(const vsdk::Range& aRange) const;

  /**
   * @brief          Returns a new matrix header for the specified diagonal
   *                 (aDiagonal=0 - the main diagonal,
   *                           >0 - a diagonal from the lower half,
   *                           <0 - a diagonal from the upper half)
   *
   * @param[in]      aDiagonal    Diagonal specification
   *
   * @return         New matrix structure
   */
  vsdk::SUMat diag(const int32_t aDiagonal = 0) const;

  /**
   * @brief          Returns N if the matrix is 1-channel (N x ptdim) or
   *                 ptdim-channel (1 x N) or (N x 1); negative number
   *                 otherwise
   *
   * @param[in]      aElemChannels      Query number of channels
   * @param[in]      aDepth             Query depth
   * @param[in]      aRequireContinuous Query is continuous?
   *
   * @return         Returns N if the matrix is 1-channel (N x ptdim) or
   *                 ptdim-channel (1 x N) or (N x 1); negative number
   *                 otherwise
   */
  int32_t checkVector(const int32_t aElemChannels,
                      const int32_t aDepth             = -1,
                      const bool    aRequireContinuous = true) const;

  /**
   * @brief          Returns SUMat handle for allocated structure
   *
   * @param[in]      aAccessFlags Specified access flags
   *
   * @return         SUMat Handle
   */
  void* handle(const uint32_t aAccessFlags) const;

  /**
   * @brief          Fills ofs parameter with offset for each dimension
   *
   * @param[out]     aOfs        Returned dimension offsets
   */
  void ndoffset(int64_t* const aOfs) const;

  /**
   * @brief          Decreases reference counter;
   *                 Deallocates the data when reference counter reaches 0.
   */
  void release();

  /**
   * @brief          Deallocates the matrix data
   */
  void deallocate();

  /**
   * @brief          Returns a SMat class with a concrete buffer mapping
   *
   * @param          aFlags
   */
  vsdk::SMat getMat(uint32_t aFlags) const;

  /**
   * @brief          includes several bit-fields:
   *                    - the magic signature
   *                    - continuity flag
   *                    - depth
   *                    - number of channels
   */
  uint32_t flags;

  /**
   * @brief          The matrix dimensionality, >= 2
   */
  int32_t dims;

  /**
   * @brief          The number of rows and columns or (-1, -1) when the
   *                 matrix has more than 2 dimensions
   */
  int32_t rows;

  /**
   * @brief          The number of rows and columns or (-1, -1) when the
   *                 matrix has more than 2 dimensions
   */
  int32_t cols;

  /**
   * @brief          Custom allocator
   */
  vsdk::SUMatAllocator* allocator;

  /**
   * @brief          Usage flags for allocator
   */
  vsdk::UMatUsageFlags usageFlags;

  /**
   * @brief          Black-box container of SUMat data
   */
  vsdk::UMatData* u;

  /**
   * @brief          Offset of the submatrix (or 0)
   */
  uint64_t offset;

  /**
   * @brief          Size of the matrix
   */
  vsdk::MatSize size;

  /**
   * @brief          Step in different dimensions: [0] n. of bytes between
   *                 pixels, [1] n. of bytes between rows, [2] n. of rows
   *                 between image row in case of y-dimensional pixel
   */
  vsdk::MatStep step;

  /*==================================================================================================
  *                                OPENCV DIFFERENCES
  ==================================================================================================*/
  /**
   * NOT IMPLEMENTED
   *    static SUMat zeros(int rows, int cols, int type);
   *    static SUMat zeros(Size size, int type);
   *    static SUMat zeros(int ndims, const int* sz, int type);
   *    static SUMat ones(int rows, int cols, int type);
   *    static SUMat ones(Size size, int type);
   *    static SUMat ones(int ndims, const int* sz, int type);
   *    static SUMat eye(int rows, int cols, int type);
   *    static SUMat eye(Size size, int type);
   *    void locateROI( Size& wholeSize, Point& ofs ) const;
   *    SUMat& adjustROI( int dtop, int dbottom, int dleft, int dright );
   *    SUMat(Size size, int type, UMatUsageFlags aUsageFlags = USAGE_DEFAULT);
   *    SUMat(int rows, int cols, int type, const Scalar& s,
   *         UMatUsageFlags aUsageFlags = USAGE_DEFAULT);
   *    SUMat(Size size, int type, const Scalar& s,
   *         UMatUsageFlags aUsageFlags = USAGE_DEFAULT);
   *    SUMat(int ndims, const int* sizes, int type, const Scalar& s,
   *         UMatUsageFlags aUsageFlags = USAGE_DEFAULT);
   *    template<typename _Tp> explicit SUMat(const std::vector<_Tp>& vec,
   *                                         bool copyData=false);
   *    template<typename _Tp, int n> explicit SUMat(const Vec<_Tp, n>& vec,
   *                                                bool copyData=true);
   *    template<typename _Tp, int m, int n> explicit SUMat(
   *                      const Matx<_Tp, m,n>& mtx, bool copyData=true);
   *    template<typename _Tp> explicit SUMat(
   *                      const Point_<_Tp>& pt, bool copyData=true);
   *    template<typename _Tp> explicit SUMat(
   *                      const Point3_<_Tp>& pt, bool copyData=true);
   *    template<typename _Tp> explicit SUMat(
   *                      const MatCommaInitializer_<_Tp>& commaInitializer);
   *    void convertTo(OutputArray m, int rtype, double alpha=1,
   *                   double beta=0 ) const;
   *    SUMat clone() const;
   *    void copyTo( OutputArray m ) const;
   *    void copyTo( OutputArray m, InputArray mask ) const;
   *    void assignTo( SUMat& m, int type=-1 ) const;
   *    SUMat& operator = (const Scalar& s);
   *    SUMat& setTo(InputArray value, InputArray mask=noArray());
   *    SUMat reshape(int cn, int rows=0) const;
   *    SUMat reshape(int cn, int newndims, const int* newsz) const;
   *    SUMat t() const;
   *    SUMat inv(int method=DECOMP_LU) const;
   *    SUMat mul(InputArray m, double scale=1) const;
   *    static SUMat diag(const SUMat& d);
   *    double dot(InputArray m) const;
   *    void create(Size size, int type,
   *                UMatUsageFlags aUsageFlags = USAGE_DEFAULT);
   */
private:
  vsdk::SMat doGetMat(const uint32_t aFlags) const;
  void       createAllocator(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType);
  void       createAllocatorFinalize();
  void       initRange(const vsdk::Range* const apRanges, const int32_t& aIt0);
  void       setDataAndFlags(const vsdk::SUMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange);
  void       setRange(const vsdk::SUMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange);
}; /* class SUMat */

/**
 * @brief          A SUMat derivation for internal purposes. Underlying pointers
 *                 can be specified.
 * @details        A custom implementation of the SUMat which allows to specify
 *                 the hw and virtual pointer. This implementation does not
 *                 manage cache nor memory mapping, it's just a blind wrapper
 *                 for specified buffer/mapping.
 *
 */
class CustomPointerSUMat : public vsdk::SUMat
{
public:
  /**
   * @brief          Constructs 2D matrix of the specified size and type
   *
   * @param[in]      aRows        Number of rows (height)
   * @param[in]      aCols        Number of cols (width)
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   *                              CV_32SC(12) etc.
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   * @param[in]      apHWAddr     HW physical address of the buffer
   * @param[in]      apVirtAddr   virtual mapping of the buffer (can be NULL)
   */
  CustomPointerSUMat(const int32_t  aRows,
                     const int32_t  aCols,
                     uint32_t       aType,
                     uint8_t* const apHWAddr,
                     uint8_t* const apVirtAddr);

  /**
   * @brief          Default constructor
   * @details
   *
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  CustomPointerSUMat(const vsdk::UMatUsageFlags aUsageFlags = USAGE_DEFAULT);

  /**
   * @brief          Allocates new matrix data unless the matrix already has
   *                 specified size and type.
   *                 previous data is unreferenced if needed.
   *
   * @param[in]      aNDims       Number of dimensions
   * @param[in]      apSizes      Number of bytes in each dimension
   * @param[in]      aType        Type of element: CV_8UC1, CV_64FC3,
   *                              CV_32SC(12) etc.
   * @param[in]      apHWAddr     Physical pointer
   * @param[in]      aUsageFlags  Usage flags, if need to be specified
   */
  void createCustom(const int32_t              aNDims,
                    const int32_t* const       apSizes,
                    uint32_t                   aType,
                    void* const                apHWAddr,
                    const vsdk::UMatUsageFlags aUsageFlags = USAGE_DONT_ALLOCATE);

  /**
   * @brief          Returns a SMat class with a concrete buffer mapping
   *
   * @param          aFlags
   */
  vsdk::SMat getMat(uint32_t aFlags) const;

private:
  void createAllocator(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType, void* const apHWAddr);
}; /* class CustomPointerSUMat */

} /* namespace vsdk */

void        updateContinuityFlagUMat(vsdk::SUMat& aMatrix);
void        finalizeHdrUMat(vsdk::SUMat& aMatrix);
void        setSizeUMat(vsdk::SUMat&          aMatrix,
                        const int32_t         aDims,
                        const int32_t* const  apSz,
                        const uint64_t* const apSteps,
                        const bool            autoSteps = false);
inline bool isUMatDataOK(const vsdk::SUMat&   aUMat,
                         const int32_t        aNDims,
                         const int32_t* const apSizes,
                         const uint32_t       aType);

#endif /* UMAT_HPP */
