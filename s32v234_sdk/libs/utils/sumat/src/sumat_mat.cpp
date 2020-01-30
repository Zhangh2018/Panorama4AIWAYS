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

#include "sumat_mat.hpp"

static vsdk::SUMatAllocator gAllocator;

/*******************************************************************************
 * Function for setting size of the matrix
 * Internal use function
 ******************************************************************************/
void setSizeMat(vsdk::SMat&           aMatrix,
                const int32_t         aDims,
                const int32_t* const  apSz,
                const uint64_t* const apSteps,
                const bool            aAutoSteps)
{
  /* If the matrices are of different dimension numbers */
  if(aMatrix.dims != aDims)
  {
    /* If step buffer needs to be reallocated */
    if(aMatrix.step.p != aMatrix.step.buf)
    {
      free(aMatrix.step.p);
      aMatrix.step.p = aMatrix.step.buf;
      aMatrix.size.p = &aMatrix.rows;
    }
    /* For all matrix with dimension bigger than one, we set correct size (-1) */
    if(aDims > 2)
    {
      aMatrix.step.p =
          static_cast< uint64_t* >(malloc((static_cast< uint32_t >(aDims) * sizeof(aMatrix.step.p[0])) +
                                          (static_cast< uint32_t >(aDims + 1) * sizeof(aMatrix.size.p[0]))));
      aMatrix.size.p       = (int32_t*)(aMatrix.step.p + aDims) + 1;
      aMatrix.size.p[(-1)] = aDims;
      aMatrix.rows         = -1;
      aMatrix.cols         = -1;
    }
  }

  aMatrix.dims = aDims;
  /* We need to set all the sizes for eeach dimension */
  if(apSz != static_cast< int32_t* >(NULL))
  {
    const uint64_t lEsz   = VSDK_CV_ELEM_SIZE(aMatrix.flags);
    const uint64_t lEsz1  = VSDK_CV_ELEM_SIZE1(aMatrix.flags);
    uint64_t       lTotal = lEsz;
    int32_t        lIt;

    /* iterate through dimensions */
    for(lIt = aDims - 1; lIt >= 0; lIt--)
    {
      uint32_t lS         = static_cast< uint32_t >(apSz[lIt]);
      aMatrix.size.p[lIt] = static_cast< int32_t >(lS);

      if(apSteps != static_cast< uint64_t* >(NULL))
      {
        /* Check valid data */
        if((apSteps[lIt] % lEsz1) != 0U)
        {
          std::cout << "Step must be a multiple of lEsz1";
        }

        /* Update the step accordingly */
        aMatrix.step.p[lIt] = (lIt < (aDims - 1)) ? apSteps[lIt] : lEsz;
      }
      else
      {
        /* Case of auto step */
        if(aAutoSteps)
        {
          aMatrix.step.p[lIt] = lTotal;
          uint64_t lTotal1    = static_cast< uint64_t >(lTotal) * lS;
          lTotal              = static_cast< uint64_t >(lTotal1);
        }
      }
    }

    /* If we have only one dimension */
    if(aDims == 1)
    {
      aMatrix.dims    = 2;
      aMatrix.cols    = 1;
      aMatrix.step[1] = lEsz;
    }
  }
}

/*******************************************************************************
 * Function updates the continuity flag according to data
 * Internal use function
 ******************************************************************************/
void updateContinuityFlagMat(vsdk::SMat& aMatrix)
{
  int32_t lIt;
  int32_t lIt1;

  /* Iterate through the dimensions, break if we find end */
  for(lIt = 0; lIt < aMatrix.dims; lIt++)
  {
    if(aMatrix.size[lIt] > 1)
    {
      break;
    }
  }

  /* For all lower dims, check the step and size */
  for(lIt1 = aMatrix.dims - 1; lIt1 > lIt; lIt1--)
  {
    if((aMatrix.step[lIt1] * static_cast< uint64_t >(aMatrix.size[lIt1])) < aMatrix.step[lIt1 - 1])
    {
      break;
    }
  }

  /* If we didn't find any discontinuity, we set the correct flag. Otherwise, the flag is removed */
  const uint64_t lT = static_cast< uint64_t >(aMatrix.step[0]) * static_cast< uint64_t >(aMatrix.size[0]);
  if((lIt1 <= lIt) && (lT == static_cast< uint64_t >(lT)))
  {
    aMatrix.flags |= static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
  }
  else
  {
    aMatrix.flags &= ~static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
  }
}

/*******************************************************************************
 * Function finalizes header of the matrix
 * Internal use function
 ******************************************************************************/
void finalizeHdrMat(vsdk::SMat& aMatrix)
{
  /* Check and set continuity */
  updateContinuityFlagMat(aMatrix);

  const int32_t lDims = aMatrix.dims;

  /* If number of dims are bigger than 2, we set size to -1*/
  if(lDims > 2)
  {
    aMatrix.rows = -1;
    aMatrix.cols = -1;
  }

  /* Check and set data pointer */
  if(aMatrix.u != static_cast< vsdk::UMatData* >(NULL))
  {
    aMatrix.datastart = aMatrix.u->data;
    aMatrix.data      = aMatrix.u->data;
  }

  /* Check and set data limit and data end according to original matrix */
  if(aMatrix.data != static_cast< uint8_t* >(NULL))
  {
    aMatrix.datalimit = aMatrix.datastart + (static_cast< uint64_t >(aMatrix.size[0]) * aMatrix.step[0]);
    if(aMatrix.size[0] > 0)
    {
      aMatrix.dataend = aMatrix.ptr() + (static_cast< uint64_t >(aMatrix.size[(lDims - 1)]) * aMatrix.step[lDims - 1]);
      for(int32_t lIt = 0; lIt < (lDims - 1); lIt++)
      {
        aMatrix.dataend += static_cast< uint64_t >(aMatrix.size[lIt] - 1) * aMatrix.step[lIt];
      }
    }
    else
    {
      aMatrix.dataend = aMatrix.datalimit;
    }
  }
  else
  {
    aMatrix.dataend   = static_cast< uint8_t* >(NULL);
    aMatrix.datalimit = static_cast< uint8_t* >(NULL);
  }
}

/*******************************************************************************
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
 ******************************************************************************/
vsdk::SMat::SMat(const int32_t         aNDims,
                 const int32_t* const  apSizes,
                 const uint32_t        aType,
                 void* const           apData,
                 const uint64_t* const aSteps)
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), data(static_cast< uint8_t* >(NULL)),
      datastart(static_cast< uint8_t* >(NULL)), dataend(static_cast< uint8_t* >(NULL)),
      datalimit(static_cast< uint8_t* >(NULL)), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* Basic constructor, set the type of the matrix */
  flags |= VSDK_CV_MAT_TYPE(aType);

  /* Set the data pointer defined by user */
  datastart = static_cast< uint8_t* >(apData);
  data      = static_cast< uint8_t* >(apData);

  /* Use internal function to set sizes of matrix and finalize it's header afterwards */
  setSizeMat(*this, aNDims, apSizes, aSteps, true);
  finalizeHdrMat(*this);
}

/*******************************************************************************
 * Helper function for matrix constructor
 ******************************************************************************/
void vsdk::SMat::setDataAndFlags(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
{
  if((aRowRange != vsdk::Range::all()) && (aRowRange != vsdk::Range(0, rows)))
  {
    rows = aRowRange.size();

    /* Add start index to data */
    data += step * static_cast< uint64_t >(aRowRange.start);
    flags |= static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG);
  }

  /* if column range is smaller than whole image, we set data pointer correctly + submatrix flag */
  if((aColRange != vsdk::Range::all()) && (aColRange != vsdk::Range(0, cols)))
  {
    cols = aColRange.size();

    /* Add start index to data */
    data += static_cast< uint64_t >(aColRange.start) * elemSize();
    flags &= (cols < aMatrix.cols) ? ~static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG) : 0xFFFFFFFFU;
    flags |= static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG);
  }

  /* If there's only one dimension, the matrix stays continuous */
  if(rows == 1)
  {
    flags |= static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
  }

  /* Error state */
  if((rows <= 0) || (cols <= 0))
  {
    release();
    rows = 0;
    cols = 0;
  }
}

/*******************************************************************************
 * Helper function for matrix constructor
 ******************************************************************************/
void vsdk::SMat::setRange(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
{
  /* For each dimension, create a Range object, set the correct ranges and create an iintermediate this SMat */
  vsdk::Range* lRs = new vsdk::Range[aMatrix.dims];
  lRs[0]           = aRowRange;
  lRs[1]           = aColRange;
  for(int32_t lIt = 2; lIt < aMatrix.dims; lIt++)
  {
    lRs[lIt] = Range::all();
  }
  *this = aMatrix(lRs);
  delete[] lRs;
}

/*******************************************************************************
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
 ******************************************************************************/
vsdk::SMat::SMat(const vsdk::SMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL)), dims(0), rows(0), cols(0),
      data(static_cast< uint8_t* >(NULL)), datastart(static_cast< uint8_t* >(NULL)),
      dataend(static_cast< uint8_t* >(NULL)), datalimit(static_cast< uint8_t* >(NULL)),
      allocator(static_cast< vsdk::SUMatAllocator* >(NULL)), u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* In case of dims > 2, we set the range for each of them. */
  if(aMatrix.dims > 2)
  {
    /* Use internal function for setting up the range */
    setRange(aMatrix, aRowRange, aColRange);
  }
  /* Case of dims are 1 or 2 */
  else
  {
    *this = aMatrix;

    /* Use internal function for setting up data and flags */
    setDataAndFlags(aMatrix, aRowRange, aColRange);
  }
}

/*******************************************************************************
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
 ******************************************************************************/
vsdk::SMat::SMat(const vsdk::SMat& aMatrix, const vsdk::Rect& aRoi)
    : flags(aMatrix.flags), dims(2), rows(aRoi.height), cols(aRoi.width),
      data(aMatrix.data + (static_cast< uint64_t >(aRoi.y) * aMatrix.step[0])), datastart(aMatrix.datastart),
      dataend(aMatrix.dataend), datalimit(aMatrix.datalimit), allocator(aMatrix.allocator), u(aMatrix.u), size(&rows)
{
  /* Set continuous flag, if specific conditions are met. Otherwise remove it. */
  flags &= (aRoi.width < aMatrix.cols) ? ~static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG) : 0xFFFFFFFFU;
  flags |= (aRoi.height == 1) ? static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG) : 0U;

  /* Determine element size amd increase reference counter */
  const size_t lEsz = VSDK_CV_ELEM_SIZE(flags);
  data += static_cast< uint64_t >(aRoi.x) * lEsz;

  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    VSDK_CV_XADD_VOID(&u->refcount, 1);
  }

  /* Set submatrix flag, if this was created */
  if((aRoi.width < aMatrix.cols) || (aRoi.height < aMatrix.rows))
  {
    flags |= static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG);
  }

  /* Set correct size */
  step[0] = aMatrix.step[0];
  step[1] = lEsz;

  /* Case of error */
  if((rows <= 0) || (cols <= 0))
  {
    release();
    rows = 0;
    cols = 0;
  }
}

/*******************************************************************************
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
 ******************************************************************************/
vsdk::SMat::SMat(const vsdk::SMat& aMatrix, const vsdk::Range* const aRanges)
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL)), dims(0), rows(0), cols(0),
      data(static_cast< uint8_t* >(NULL)), datastart(static_cast< uint8_t* >(NULL)),
      dataend(static_cast< uint8_t* >(NULL)), datalimit(static_cast< uint8_t* >(NULL)),
      allocator(static_cast< vsdk::SUMatAllocator* >(NULL)), u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  int32_t       lIt;
  const int32_t lDims = aMatrix.dims;

  *this = aMatrix;
  /* Iterate through dimensions and initialize all ranges */
  for(lIt = 0; lIt < lDims; lIt++)
  {
    /* Get range for current dimension*/
    vsdk::Range lR = aRanges[lIt];

    /* If correct, set the sizes */
    if((lR != vsdk::Range::all()) && (lR != vsdk::Range(0, size.p[lIt])))
    {
      size.p[lIt] = lR.end - lR.start;
      data += static_cast< uint64_t >(lR.start) * step.p[lIt];
      flags |= static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG);
    }
  }

  /* Check and update the continuity flag */
  updateContinuityFlagMat(*this);
}

/*******************************************************************************
 * Extracts a diagonal from a matrix
 * 
 * @param[in]      aDiag        index of the diagonal, with the following
 *                              values:
 *                               - `aDiag=0` is the main diagonal.
 *                               - `aDiag>0` is a diagonal from the lower
 * half. For example, aDiag=1 means the diagonal is set immediately below the
 * main one.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::diag(const int32_t aDiag) const
{
  vsdk::SMat     lMatrix = *this;
  const uint64_t lEsz    = elemSize();
  int32_t        len;

  /* Main diagonal or lower half */
  if(aDiag >= 0)
  {
    len = std::min(cols - aDiag, rows);
    lMatrix.data += lEsz * static_cast< uint32_t >(aDiag);
  }
  /* Upper half */
  else
  {
    len = std::min(rows + aDiag, cols);
    lMatrix.data += step[0] * static_cast< uint32_t >(aDiag * (-1));
  }

  /* Set correct size of the diagonal matrix */
  lMatrix.size[0] = len;
  lMatrix.rows    = len;
  lMatrix.size[1] = 1;
  lMatrix.cols    = 1;
  lMatrix.step[0] += ((len > 1) ? lEsz : 0U);

  /* Check if diagonal is continuous */
  if(lMatrix.rows > 1)
  {
    lMatrix.flags &= ~static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
  }
  else
  {
    lMatrix.flags |= static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
  }

  /* Set submatrix flag */
  if((size[0] != 1) || (size[1] != 1))
  {
    lMatrix.flags |= static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG);
  }

  return lMatrix;
}

/*******************************************************************************
 * Helper class for creation of allocator.
 ******************************************************************************/
void vsdk::SMat::createAllocator(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType)
{
  flags = (aType & VSDK_CV_MAT_TYPE_MASK) | static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL);

  /* Set size for this matrix */
  setSizeMat(*this, aNDims, apSizes, static_cast< uint64_t* >(NULL), true);

  /* if the size was set correctly, we allocate the memory */
  if(total() > 0U)
  {
    vsdk::SUMatAllocator* lpA = allocator;
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    const vsdk::SUMatAllocator* const a0 = &gAllocator;
#endif
    if(lpA == static_cast< SUMatAllocator* >(NULL))
    {
      lpA = &gAllocator;
    }
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    try
    {
      u = lpA->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, USAGE_DEFAULT);
      lpA->map(u, 0U);
    }
    catch(...)
    {
      if(lpA != a0)
      {
        u = a0->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, USAGE_DEFAULT);
        lpA->map(u, 0U);
      }
    }
#else
    u = lpA->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, USAGE_DEFAULT);
    lpA->map(u, 0U);
#endif
  }
  /* Add a reference and finalize header */
  addref();
  finalizeHdrMat(*this);
}

/*******************************************************************************
 * Allocates new array data if needed.
 ******************************************************************************/
void vsdk::SMat::create(const int32_t aNDims, const int32_t* const apSizes, uint32_t aType)
{
  aType = VSDK_CV_MAT_TYPE(aType);

  /* var to satisfy one exit point */
  bool lAllOK = true;

  /* A big check for correct data. If the data are not compatible, we won't continue in this function */
  if(((data != static_cast< uint8_t* >(NULL)) && ((aNDims == dims) || ((aNDims == 1) && (dims <= 2)))) &&
     (aType == static_cast< uint32_t >(type())))
  {
    if((aNDims == 2) && ((rows == apSizes[0]) && (cols == apSizes[1])))
    {
      lAllOK = false;
    }
    else
    {
      int32_t lIt;
      for(lIt = 0; lIt < aNDims; lIt++)
      {
        if(size[lIt] != apSizes[lIt])
        {
          break;
        }
      }

      if((lIt == aNDims) && ((aNDims > 1) || (size[1] == 1)))
      {
        lAllOK = false;
      }
    }
  }

  /* If the data check didn no pass, we won't continue. Otherwise release current matrix. */
  if(lAllOK)
  {
    release();

    if(aNDims == 0)
    {
      lAllOK = false;
    }
  }

  /* If the data check didn no pass, we won't continue. Otherwise init current matrix. */
  if(lAllOK)
  {
    createAllocator(aNDims, apSizes, aType);
  }
}

/*******************************************************************************
 * Deallocates the matrix data
 ******************************************************************************/
void vsdk::SMat::deallocate()
{
  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    u->currAllocator->unmap(u);
  }
  u = static_cast< vsdk::UMatData* >(NULL);
}

/*******************************************************************************
 * Internal use function; properly re-allocates _size, _step arrays
 * The function is used for correctly copy sizes and steps of matrices.
 ******************************************************************************/
void vsdk::SMat::copySize(const vsdk::SMat& aMatrix)
{
  /* Use internal function to copy the sizes to this instance */
  setSizeMat(*this, aMatrix.dims, static_cast< int32_t* >(NULL), static_cast< uint64_t* >(NULL));

  /* Secondly, iterate through dims */
  for(int32_t lIt = 0; lIt < dims; lIt++)
  {
    size[lIt] = aMatrix.size[lIt];
    step[lIt] = aMatrix.step[lIt];
  }
}

/*******************************************************************************
 * Returns N if the matrix is 1-channel (N x ptdim) or ptdim-channel (1 x N) or
 * (N x 1); negative number otherwise
 ******************************************************************************/
int32_t vsdk::SMat::checkVector(const int32_t aElemChannels, const int32_t aDepth, const bool aReqContinuous) const
{
  return ((((aDepth <= 0) || (depth() == static_cast< uint32_t >(aDepth))) && (isContinuous() || (!aReqContinuous))) &&
          (((dims == 2) && ((((rows == 1) || (cols == 1)) && (channels() == static_cast< uint32_t >(aElemChannels))) ||
                            ((cols == aElemChannels) && (channels() == 1U)))) ||
           (((((dims == 3) && (channels() == 1U)) && (size.p[2] == aElemChannels)) &&
             ((size.p[0] == 1) || (size.p[1] == 1))) &&
            (isContinuous() || (step.p[1] == (step.p[2] * static_cast< uint64_t >(size.p[2])))))))
             ? static_cast< int32_t >((total() * channels()) / static_cast< uint64_t >(aElemChannels))
             : -1;
}

/*******************************************************************************
 * Function returns true is range is empty
 * These are various constructors that form a matrix. As noted in the
 * AutomaticAllocation, often the default constructor is enough, and the proper
 * matrix will be allocated by an OpenCV function.
 * The constructed matrix can further be assigned to another matrix or matrix
 * expression or can be allocated with SMat::create . In the former case, the old
 * content is de-referenced.
 ******************************************************************************/
vsdk::SMat::SMat()
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL)), dims(0), rows(0), cols(0),
      data(static_cast< uint8_t* >(NULL)), datastart(static_cast< uint8_t* >(NULL)),
      dataend(static_cast< uint8_t* >(NULL)), datalimit(static_cast< uint8_t* >(NULL)),
      allocator(static_cast< vsdk::SUMatAllocator* >(NULL)), u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* All the members are initialized to NULL*/
}

/*******************************************************************************
 * @overload
 * Matrix constructor based on size and type
 * @param aRows  Number of rows
 * @param aCols  Number of cols
 * @param aType  Matrix data type
 ******************************************************************************/
vsdk::SMat::SMat(const int32_t aRows, const int32_t aCols, const uint32_t aType)
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL)), dims(0), rows(0), cols(0),
      data(static_cast< uint8_t* >(NULL)), datastart(static_cast< uint8_t* >(NULL)),
      dataend(static_cast< uint8_t* >(NULL)), datalimit(static_cast< uint8_t* >(NULL)),
      allocator(static_cast< vsdk::SUMatAllocator* >(NULL)), u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* Afterwards, create is called. */
  create(aRows, aCols, aType);
}

/*******************************************************************************
 * @overload
 * Matrix constructor for arbitrary number of dimensions. 
 * @param aNDims  Number of dimensions
 * @param apSizes Size array with specified sizes for each dimensions
 * @param aType   Matrix data type
 ******************************************************************************/
vsdk::SMat::SMat(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType)
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL)), dims(0), rows(0), cols(0),
      data(static_cast< uint8_t* >(NULL)), datastart(static_cast< uint8_t* >(NULL)),
      dataend(static_cast< uint8_t* >(NULL)), datalimit(static_cast< uint8_t* >(NULL)),
      allocator(static_cast< vsdk::SUMatAllocator* >(NULL)), u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* Afterwards, create is called. */
  create(aNDims, apSizes, aType);
}

/*******************************************************************************
 * Helper for copy Matrix 
 ******************************************************************************/
void cpyMatStep(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix)
{
  aDstMatrix.step[0] = aSrcMatrix.step[0];
  aDstMatrix.step[1] = aSrcMatrix.step[1];
}

/*******************************************************************************
 * Helper for operator = 
 ******************************************************************************/
void cpyMatSizes(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix)
{
  /* Copy matrix dimensions */
  aDstMatrix.dims    = aSrcMatrix.dims;
  aDstMatrix.rows    = aSrcMatrix.rows;
  aDstMatrix.cols    = aSrcMatrix.cols;
  aDstMatrix.step[0] = aSrcMatrix.step[0];
  aDstMatrix.step[1] = aSrcMatrix.step[1];
}

/*******************************************************************************
 * Helper for operator = 
 ******************************************************************************/
void cpyMatData(const vsdk::SMat& aSrcMatrix, vsdk::SMat& aDstMatrix)
{
  /* Copy matrix data */
  aDstMatrix.data      = aSrcMatrix.data;
  aDstMatrix.datastart = aSrcMatrix.datastart;
  aDstMatrix.dataend   = aSrcMatrix.dataend;
  aDstMatrix.datalimit = aSrcMatrix.datalimit;
  aDstMatrix.allocator = aSrcMatrix.allocator;
  aDstMatrix.u         = aSrcMatrix.u;
}

/*******************************************************************************
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
 ******************************************************************************/
vsdk::SMat::SMat(const vsdk::SMat& aMatrix)
    : flags(aMatrix.flags), dims(aMatrix.dims), rows(aMatrix.rows), cols(aMatrix.cols), size(&rows)
{
  cpyMatData(aMatrix, *this);
  /* Data check, add reference */
  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    VSDK_CV_XADD_VOID(&u->refcount, 1);
  }

  /* init the steps, which are done after constructor */
  if(aMatrix.dims <= 2)
  {
    cpyMatStep(aMatrix, *this);
  }
  /* If dims are bigger, we use internal copySize function. */
  else
  {
    dims = 0;
    copySize(aMatrix);
  }
}

/*******************************************************************************
 * @overload
 * Matrix constructor based on size, type and user defined pointer
 * @param aRows  Number of rows
 * @param aCols  Number of cols
 * @param aType  Matrix data type
 * @param apData User specified data type
 * @param aStep  Step defined for matrix
 ******************************************************************************/
vsdk::SMat::SMat(const int32_t aRows, const int32_t aCols, const uint32_t aType, void* const apData, uint64_t aStep)
    : flags(static_cast< uint32_t >(vsdk::SMat::MAGIC_VAL) + (aType & static_cast< uint32_t >(vsdk::SMat::TYPE_MASK))),
      dims(2), rows(aRows), cols(aCols), data(static_cast< uint8_t* >(apData)),
      datastart(static_cast< uint8_t* >(apData)), dataend(static_cast< uint8_t* >(NULL)),
      datalimit(static_cast< uint8_t* >(NULL)), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      u(static_cast< vsdk::UMatData* >(NULL)), size(&rows)
{
  /* Data check */
  if((total() == 0U) || (data == static_cast< uint8_t* >(NULL)))
  {
  }
  else
  {
    /* init element size */
    const uint64_t esz     = VSDK_CV_ELEM_SIZE(aType);
    const uint64_t esz1    = VSDK_CV_ELEM_SIZE1(aType);
    const uint64_t minstep = static_cast< uint64_t >(cols) * esz;

    /* case the step is default, the matrix is continuous */
    if(aStep == vsdk::SMat::AUTO_STEP)
    {
      aStep = minstep;
      flags |= static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG);
    }
    /* ekse we need to determine */
    else
    {
      if(rows == 1)
      {
        aStep = minstep;
      }

      if((aStep % esz1) != 0U)
      {
      }

      flags |= ((aStep == minstep) ? static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG) : 0U);
    }

    /* Init all other members */
    step[0]   = aStep;
    step[1]   = esz;
    datalimit = datastart + (aStep * static_cast< uint64_t >(rows));
    dataend   = (datalimit - aStep) + minstep;
  }
}

/*******************************************************************************
 * destructor - calls release()
 ******************************************************************************/
vsdk::SMat::~SMat()
{
  release();
  if(step.p != step.buf)
  {
    free(step.p);
  }
}

/*******************************************************************************
 * Assignment operators
 ******************************************************************************/
vsdk::SMat& vsdk::SMat::operator=(const vsdk::SMat& aMatrix)
{
  /* assignment to yourself guard */
  if(this != &aMatrix)
  {
    /* Increase ref count */
    if(aMatrix.u != static_cast< vsdk::UMatData* >(NULL))
    {
      VSDK_CV_XADD_VOID(&aMatrix.u->refcount, 1);
    }

    release();
    flags = aMatrix.flags;

    /* In case of vector or matrix, init dimensions */
    if((dims <= 2) && (aMatrix.dims <= 2))
    {
      cpyMatSizes(aMatrix, *this);
    }
    /* For other arbitrary dimensions, use copySize func */
    else
    {
      copySize(aMatrix);
    }
    cpyMatData(aMatrix, *this);
  }
  return *this;
}

/*******************************************************************************
 * Creates a matrix header for the specified matrix row.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::row(const int32_t aRowIndex) const
{
  return vsdk::SMat(*this, vsdk::Range(aRowIndex, aRowIndex + 1), vsdk::Range::all());
}

/*******************************************************************************
 * Creates a matrix header for the specified matrix column.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::col(const int32_t aColIndex) const
{
  return vsdk::SMat(*this, vsdk::Range::all(), vsdk::Range(aColIndex, aColIndex + 1));
}

/*******************************************************************************
 * Creates a matrix header for the specified row span.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::rowRange(const int32_t aStartRow, const int32_t aEndRow) const
{
  return vsdk::SMat(*this, vsdk::Range(aStartRow, aEndRow), vsdk::Range::all());
}

/*******************************************************************************
 * @overload
 ******************************************************************************/
vsdk::SMat vsdk::SMat::rowRange(const vsdk::Range& aRange) const
{
  return vsdk::SMat(*this, aRange, vsdk::Range::all());
}

/*******************************************************************************
 * Creates a matrix header for the specified column span.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::colRange(const int32_t aStartCol, const int32_t aEndCol) const
{
  return vsdk::SMat(*this, vsdk::Range::all(), vsdk::Range(aStartCol, aEndCol));
}

/*******************************************************************************
 * @overload
 ******************************************************************************/
vsdk::SMat vsdk::SMat::colRange(const vsdk::Range& aRange) const
{
  return vsdk::SMat(*this, vsdk::Range::all(), aRange);
}

/*******************************************************************************
 * Helper class for VOCF in create file. Will check if the matrix should be created.
 ******************************************************************************/
bool createNewMatrix(const vsdk::SMat& aDst, const int32_t aRows, const int32_t aCols, const uint32_t aType)
{
  bool lRetVal = true;
  /* Go through sizes and check if the type is and size is same */
  if(aDst.dims <= 2)
  {
    if(aDst.rows == aRows)
    {
      if(aDst.cols == aCols)
      {
        if(aDst.type() == aType)
        {
          lRetVal = false;
        }
      }
    }
  }

  return lRetVal;
}

/*******************************************************************************
 * Allocates new array data if needed.
 ******************************************************************************/
void vsdk::SMat::create(const int32_t aRows, const int32_t aCols, uint32_t aType)
{
  aType &= static_cast< uint32_t >(vsdk::SMat::TYPE_MASK);

  bool lCreate = true;

  /* Use helper function to identify the creation flag */
  if(!createNewMatrix(*this, aRows, aCols, aType))
  {
    if(data != static_cast< uint8_t* >(NULL))
    {
      lCreate = false;
    }
  }

  if(lCreate)
  {
    int32_t lSz[] = {aRows, aCols};
    create(2, lSz, aType);
  }
}

/*******************************************************************************
 * Increments the reference counter.
 ******************************************************************************/
void vsdk::SMat::addref() const
{
  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    VSDK_CV_XADD_VOID(&u->refcount, 1);
  }
}

/*******************************************************************************
 * Decrements the reference counter and deallocates the matrix if needed.
 ******************************************************************************/
void vsdk::SMat::release()
{
  /* call deallocate function and decrease ref count */
  if((u != static_cast< vsdk::UMatData* >(NULL)) && (VSDK_CV_XADD(&u->refcount, -1) == 1))
  {
    deallocate();
  }

  /* zero everything */
  u         = static_cast< vsdk::UMatData* >(NULL);
  datastart = static_cast< uint8_t* >(NULL);
  dataend   = static_cast< uint8_t* >(NULL);
  datalimit = static_cast< uint8_t* >(NULL);
  data      = static_cast< uint8_t* >(NULL);
  for(int32_t lIt = 0; lIt < dims; lIt++)
  {
    size.p[lIt] = 0;
  }
}

/*******************************************************************************
 * Extracts a rectangular submatrix.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::operator()(const vsdk::Range aRowRange, const vsdk::Range aColRange) const
{
  return vsdk::SMat(*this, aRowRange, aColRange);
}

/*******************************************************************************
 * Extracts a rectangular submatrix.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::operator()(const vsdk::Rect& aRoi) const
{
  return vsdk::SMat(*this, aRoi);
}

/*******************************************************************************
 * Extracts a rectangular submatrix.
 ******************************************************************************/
vsdk::SMat vsdk::SMat::operator()(const vsdk::Range* const apRanges) const
{
  return vsdk::SMat(*this, apRanges);
}

/*******************************************************************************
 * Reports whether the matrix is continuous or not.
 ******************************************************************************/
bool vsdk::SMat::isContinuous() const
{
  return (flags & static_cast< uint32_t >(vsdk::SMat::CONTINUOUS_FLAG)) != 0U;
}

/*******************************************************************************
 * Returns true if the matrix is a submatrix of another matrix
 ******************************************************************************/
bool vsdk::SMat::isSubmatrix() const
{
  return (flags & static_cast< uint32_t >(vsdk::SMat::SUBMATRIX_FLAG)) != 0U;
}

/*******************************************************************************
 * Returns the matrix element size in bytes.
 ******************************************************************************/
uint64_t vsdk::SMat::elemSize() const
{
  return (dims > 0) ? step.p[(dims - 1)] : 0U;
}

/*******************************************************************************
 * Returns the size of each matrix element channel in bytes.
 ******************************************************************************/
uint64_t vsdk::SMat::elemSize1() const
{
  return static_cast< uint64_t >(VSDK_CV_ELEM_SIZE1(flags));
}

/*******************************************************************************
 * Returns the type of a matrix element.
 ******************************************************************************/
uint32_t vsdk::SMat::type() const
{
  return VSDK_CV_MAT_TYPE(flags);
}

/*******************************************************************************
 * Returns the depth of a matrix element.
 ******************************************************************************/
uint32_t vsdk::SMat::depth() const
{
  return VSDK_CV_MAT_DEPTH(flags);
}

/*******************************************************************************
 * Returns the number of matrix channels.
 ******************************************************************************/
uint32_t vsdk::SMat::channels() const
{
  return VSDK_CV_MAT_CN(flags);
}

/*******************************************************************************
 * Returns a normalized step.
 ******************************************************************************/
uint64_t vsdk::SMat::step1(const int32_t aIndex) const
{
  return step.p[aIndex] / elemSize1();
}

/*******************************************************************************
 * Returns true if the array has no elements.
 ******************************************************************************/
bool vsdk::SMat::empty() const
{
  return (data == static_cast< uint8_t* >(NULL)) || (total() == 0U);
}

/*******************************************************************************
 * Returns the total number of array elements.
 ******************************************************************************/
uint64_t vsdk::SMat::total() const
{
  uint64_t lRetVal;
  /* in case there is a classic vector or matrix */
  if(dims <= 2)
  {
    lRetVal = static_cast< uint64_t >(static_cast< uint64_t >(rows) * static_cast< uint64_t >(cols));
  }
  /* arbitrary dimension size */
  else
  {
    uint64_t lP = 1U;
    for(int32_t lIt = 0; lIt < dims; lIt++)
    {
      lP *= static_cast< uint64_t >(size[lIt]);
    }
    lRetVal = lP;
  }
  return lRetVal;
}

/*******************************************************************************
 * Returns a pointer to the specified matrix row.
 ******************************************************************************/
const uint8_t* vsdk::SMat::ptr(const uint32_t aIndex0) const
{
  return data + (step.p[0] * aIndex0);
}

/*******************************************************************************
 * Returns a pointer to the specified matrix row.
 ******************************************************************************/
const uint8_t* vsdk::SMat::ptr(const uint32_t aIndex0, const uint32_t aIndex1) const
{
  return (data + (aIndex0 * step.p[0])) + (aIndex1 * step.p[1]);
}

/*******************************************************************************
 * Returns a pointer to the specified matrix row.
 ******************************************************************************/
const uint8_t* vsdk::SMat::ptr(const uint32_t aIndex0, const uint32_t aIndex1, const uint32_t aIndex2) const
{
  return ((data + (aIndex0 * step.p[0])) + (aIndex1 * step.p[1])) + (aIndex2 * step.p[2]);
}

/*******************************************************************************
 * Returns a pointer to the specified matrix row.
 ******************************************************************************/
const uint8_t* vsdk::SMat::ptr(const uint32_t* const aIndex) const
{
  uint32_t       lIt;
  const uint32_t lDims  = static_cast< uint32_t >(dims);
  uint8_t*       lpData = data;
  for(lIt = 0U; lIt < lDims; lIt++)
  {
    lpData += aIndex[lIt] * step.p[lIt];
  }
  return lpData;
}
