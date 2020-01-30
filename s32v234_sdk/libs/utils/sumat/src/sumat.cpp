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

#include "sumat_defines.hpp"

#include <iostream>

#include <oal.h>

#include "sumat.hpp"
#include "sumat_mutex.hpp"
#include "sumat_allocator.hpp"

static vsdk::SUMatAllocator gAllocator;

/*******************************************************************************
 * Function updates the continuity flag according to data
 ******************************************************************************/
void updateContinuityFlagUMat(vsdk::SUMat& aMatrix)
{
  int32_t lIt0;
  int32_t lIt1;

  /* Iterate through the dimensions, break if we find end */
  for(lIt0 = 0; lIt0 < aMatrix.dims; lIt0++)
  {
    if(aMatrix.size[lIt0] > 1)
    {
      break;
    }
  }

  /* For all lower dims, check the step and size */
  for(lIt1 = aMatrix.dims - 1; lIt1 > lIt0; lIt1--)
  {
    if((aMatrix.step[lIt1] * static_cast< uint32_t >(aMatrix.size[lIt1])) < (aMatrix.step[lIt1 - 1]))
    {
      break;
    }
  }

  /* If we didn't find any discontinuity, we set the correct flag. Otherwise, the flag is removed */
  if(lIt1 <= lIt0)
  {
    aMatrix.flags |= vsdk::SUMat::CONTINUOUS_FLAG;
  }
  else
  {
    aMatrix.flags &= ~vsdk::SUMat::CONTINUOUS_FLAG;
  }
}

/*******************************************************************************
 * Function finalizes header
 ******************************************************************************/
void finalizeHdrUMat(vsdk::SUMat& aMatrix)
{
  updateContinuityFlagUMat(aMatrix);
  const int32_t lD = aMatrix.dims;
  if(lD > 2)
  {
    aMatrix.cols = -1;
    aMatrix.rows = -1;
  }
}

/*******************************************************************************
 * Function for setting size of the matrix
 ******************************************************************************/
void setSizeUMat(vsdk::SUMat&          aMatrix,
                 const int32_t         aDims,
                 const int32_t* const  apSz,
                 const uint64_t* const apSteps,
                 const bool            autoSteps)
{
  /* If the matrices are of different dimension numbers */
  if(aMatrix.dims != aDims)
  {
    /* If step buffer needs to be reallocated */
    if(aMatrix.step.p != aMatrix.step.buf)
    {
      delete[] aMatrix.step.p;
      aMatrix.step.p = aMatrix.step.buf;
      aMatrix.size.p = &aMatrix.rows;
    }
    /* For all matrix with dimension bigger than one, we set correct size (-1) */
    if(aDims > 2)
    {
      aMatrix.step.p     = new uint64_t[aDims + aDims + 1];
      aMatrix.size.p     = (static_cast< int32_t* >(static_cast< void* >(aMatrix.step.p + aDims))) + 1;
      aMatrix.size.p[-1] = aDims;
      aMatrix.rows       = -1;
      aMatrix.cols       = -1;
    }
  }

  aMatrix.dims = aDims;
  /* We need to set all the sizes for eeach dimension */
  if(apSz != NULL)
  {
    const uint64_t lEsz   = VSDK_CV_ELEM_SIZE(aMatrix.flags);
    uint64_t       lTotal = lEsz;
    int32_t        lIt0;

    /* iterate through dimensions */
    for(lIt0 = aDims - 1; lIt0 >= 0; lIt0--)
    {
      int32_t lS           = apSz[lIt0];
      aMatrix.size.p[lIt0] = lS;

      /* Check valid data */
      if(apSteps != NULL)
      {
        /* Update the step accordingly */
        aMatrix.step[lIt0] = (lIt0 < (aDims - 1)) ? apSteps[lIt0] : lEsz;
      }
      else
      {
        /* Case of auto step */
        if(autoSteps)
        {
          aMatrix.step[lIt0] = lTotal;
          lTotal             = lTotal * static_cast< uint64_t >(lS);
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
 * Helper function for matrix constructor
 ******************************************************************************/
void vsdk::SUMat::setDataAndFlags(const vsdk::SUMat& aMatrix,
                                  const vsdk::Range& aRowRange,
                                  const vsdk::Range& aColRange)
{
  /* if row range is smaller than whole image, we set data pointer correctly + submatrix flag */
  if((aRowRange != Range::all()) && (aRowRange != Range(0, rows)))
  {
    rows = aRowRange.size();
    /* Add start index to data */
    offset += static_cast< uint64_t >(step) * static_cast< uint64_t >(aRowRange.start);
    flags |= SUBMATRIX_FLAG;
  }

  /* if column range is smaller than whole image, we set data pointer correctly + submatrix flag */
  if((aColRange != Range::all()) && (aColRange != Range(0, cols)))
  {
    cols = aColRange.size();
    /* Add start index to data */
    offset += static_cast< uint64_t >(aColRange.start) * elemSize();
    flags &= (cols < aMatrix.cols) ? ~CONTINUOUS_FLAG : 0xFFFFFFFFU;
    flags |= SUBMATRIX_FLAG;
  }

  /* If there's only one dimension, the matrix stays continuous */
  if(rows == 1)
  {
    flags |= CONTINUOUS_FLAG;
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
void vsdk::SUMat::setRange(const vsdk::SUMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
{
  /* For each dimension, create a Range object, set the correct ranges and create an iintermediate this SMat */
  vsdk::Range* lpRs = new vsdk::Range[aMatrix.dims];
  lpRs[0]           = aRowRange;
  lpRs[1]           = aColRange;
  for(int32_t lIt0 = 2; lIt0 < aMatrix.dims; lIt0++)
  {
    lpRs[lIt0] = Range::all();
  }
  *this = aMatrix(lpRs);
  delete[] lpRs;
}

/*******************************************************************************
 * Overload
 ******************************************************************************/
vsdk::SUMat::SUMat(const vsdk::SUMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      usageFlags(USAGE_DEFAULT), u(static_cast< vsdk::UMatData* >(NULL)), offset(0U), size(&rows)
{
  /* In case of dims > 2, we set the range for each of them. */
  if(aMatrix.dims > 2)
  {
    setRange(aMatrix, aRowRange, aColRange);
  }
  /* Case of dims are 1 or 2 */
  else
  {
    *this = aMatrix;

    setDataAndFlags(aMatrix, aRowRange, aColRange);
  }
}

/*******************************************************************************
 * Helper to init ranges
 ******************************************************************************/
void vsdk::SUMat::initRange(const vsdk::Range* const apRanges, const int32_t& aIt0)
{
  /* Get range for current dimension*/
  const vsdk::Range lR = apRanges[aIt0];

  /* If correct, set the sizes */
  if((lR != Range::all()) && (lR != Range(0, size[aIt0])))
  {
    size[aIt0] = lR.end - lR.start;
    offset += static_cast< uint64_t >(lR.start) * step[aIt0];
    flags |= SUBMATRIX_FLAG;
  }
}

/*******************************************************************************
 * Overload
 ******************************************************************************/
vsdk::SUMat::SUMat(const vsdk::SUMat& aMatrix, const vsdk::Range* const apRanges)
    : flags(MAGIC_VAL), dims(0), rows(0), cols(0), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      usageFlags(USAGE_DEFAULT), u(static_cast< vsdk::UMatData* >(NULL)), offset(0U), size(&rows)
{
  int32_t       lIt0;
  const int32_t lD = aMatrix.dims;

  *this = aMatrix;
  /* Iterate through dimensions and initialize all ranges */
  for(lIt0 = 0; lIt0 < lD; lIt0++)
  {
    /* Init ranges through private function */
    initRange(apRanges, lIt0);
  }

  /* Check and update the continuity flag */
  updateContinuityFlagUMat(*this);
}

/*******************************************************************************
 * Overload
 ******************************************************************************/
vsdk::SUMat::SUMat(const vsdk::SUMat& aMatrix, const vsdk::Rect& aRoi)
    : flags(aMatrix.flags), dims(2), rows(aRoi.height), cols(aRoi.width), allocator(aMatrix.allocator),
      usageFlags(aMatrix.usageFlags), u(aMatrix.u),
      offset(aMatrix.offset + (static_cast< uint64_t >(aRoi.y) * aMatrix.step[0])), size(&rows)
{
  /* Set continuous flag, if specific conditions are met. Otherwise remove it. */
  flags &= (aRoi.width < aMatrix.cols) ? ~CONTINUOUS_FLAG : 0xFFFFFFFFU;
  flags |= (aRoi.height == 1) ? CONTINUOUS_FLAG : 0U;

  /* Determine element size amd increase reference counter */
  const size_t lEsz = VSDK_CV_ELEM_SIZE(flags);
  offset += static_cast< uint64_t >(aRoi.x) * lEsz;

  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    VSDK_CV_XADD_VOID(&(u->urefcount), 1);
  }

  /* Set submatrix flag, if this was created */
  if((aRoi.width < aMatrix.cols) || (aRoi.height < aMatrix.rows))
  {
    flags |= SUBMATRIX_FLAG;
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
 * Internal use function; properly re-allocates _size, _step arrays
 ******************************************************************************/
void vsdk::SUMat::copySize(const vsdk::SUMat& aMatrix)
{
  /* Use internal function to copy the sizes to this instance */
  setSizeUMat(*this, aMatrix.dims, NULL, NULL);

  /* Secondly, iterate through dims */
  for(int32_t lIt0 = 0; lIt0 < dims; lIt0++)
  {
    size[lIt0] = aMatrix.size[lIt0];
    step[lIt0] = aMatrix.step[lIt0];
  }
}

/*******************************************************************************
 * Destructor - calls release()
 ******************************************************************************/
vsdk::SUMat::~SUMat()
{
  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMat: Destructor, refcount " << ((u != static_cast< vsdk::UMatData* >(NULL)) ? u->urefcount : 0)
              << std::endl;
  }

  /* Call release function */
  release();

  /* If step buffer was not allocated here, delete it */
  if(step.p != step.buf)
  {
    delete[] step.p;
  }
}

/*******************************************************************************
 * Deallocates the matrix data
 ******************************************************************************/
void vsdk::SUMat::deallocate()
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMat: Deallocate" << std::endl;
  }
  u->currAllocator->deallocate(u);
  u = static_cast< vsdk::UMatData* >(NULL);
}

/*******************************************************************************
 * helper function for Function creator
 * The function is used in the Function creators, because of lowering CCM metric
 ******************************************************************************/
inline bool isUMatDataOK(const vsdk::SUMat&   aUMat,
                         const int32_t        aNDims,
                         const int32_t* const apSizes,
                         const uint32_t       aType)
{
  bool lRetVal = true;

  /* Check the umatdata existance and valid dims/type */
  if((aUMat.u != static_cast< vsdk::UMatData* >(NULL)) &&
     (((aNDims == aUMat.dims) || ((aNDims == 1) && (aUMat.dims <= 2)))) && (aType == aUMat.type()))
  {
    /* Check of correct compatible sizes */
    if((aNDims == 2) && (aUMat.rows == apSizes[0]) && (aUMat.cols == apSizes[1]))
    {
      lRetVal = false;
    }
    else
    {
      int32_t lIt0;
      /* Check each dimension */
      for(lIt0 = 0; lIt0 < aNDims; lIt0++)
      {
        if(aUMat.size[lIt0] != apSizes[lIt0])
        {
          break;
        }
      }

      if((lIt0 == aNDims) && ((aNDims > 1) || (aUMat.size[1] == 1)))
      {
        lRetVal = false;
      }
    }
  }

  return lRetVal;
}

/*******************************************************************************
 * Helper class for creation of allocator.
 ******************************************************************************/
void vsdk::SUMat::createAllocatorFinalize()
{
  /* Add a reference and finalize header if everything is ok */
  if((size != NULL) && (u != static_cast< vsdk::UMatData* >(NULL)) && (u->handle == static_cast< uint8_t >(NULL)))
  {
    *this = vsdk::SUMat();
  }
  /* Add a reference and finalize header */
  else
  {
    finalizeHdrUMat(*this);
    addref();
  }
}
/*******************************************************************************
 * Helper class for creation of allocator.
 ******************************************************************************/
void vsdk::SUMat::createAllocator(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType)
{
  /* Set size for this matrix */
  flags = (aType & VSDK_CV_MAT_TYPE_MASK) | MAGIC_VAL;
  setSizeUMat(*this, aNDims, apSizes, NULL, true);
  offset = 0U;

  /* if the size was set correctly, we allocate the memory */
  if(total() > 0U)
  {
    SUMatAllocator* lpA = allocator;
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    SUMatAllocator* a0 = &gAllocator;
#endif
    if(lpA == static_cast< SUMatAllocator* >(NULL))
    {
      lpA = &gAllocator;
#if !defined __STANDALONE__ && !defined __INTEGRITY__
      a0 = &gAllocator;
#endif
    }
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    try
    {
      u = lpA->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, usageFlags);
    }
    catch(...)
    {
      if(lpA != a0)
      {
        u = a0->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, usageFlags);
      }
    }
#else
    u = lpA->allocate(dims, size, static_cast< int32_t >(aType), NULL, step.p, 0, USAGE_DEFAULT);
#endif
  }

  /* Add a reference and finalize header if everything is ok */
  createAllocatorFinalize();
}

/*******************************************************************************
 * Creates the matrix - allocates if necessary
 ******************************************************************************/
void vsdk::SUMat::create(const int32_t              aNDims,
                         const int32_t* const       apSizes,
                         uint32_t                   aType,
                         const vsdk::UMatUsageFlags aUsageFlags)
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMat: Create" << std::endl;
  }
  this->usageFlags = aUsageFlags;

  aType = VSDK_CV_MAT_TYPE(aType);

  /* var to satisfy one exit point */
  bool lAllOK = isUMatDataOK(*this, aNDims, apSizes, aType);

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
 * Returns N if the matrix is 1-channel (N x ptdim) or ptdim-channel (1 x N)
 * or (N x 1); negative number otherwise
 ******************************************************************************/
int32_t vsdk::SUMat::checkVector(const int32_t aElemChannels, const int32_t aDepth, const bool aRequireContinuous) const
{
  return (((depth() == static_cast< uint32_t >(aDepth)) || (aDepth <= 0)) &&
          (isContinuous() || (!aRequireContinuous)) &&
          (((dims == 2) && ((((rows == 1) || (cols == 1)) && (channels() == static_cast< uint32_t >(aElemChannels))) ||
                            ((cols == aElemChannels) && (channels() == 1U)))) ||
           ((dims == 3) && (channels() == 1U) && (size[2] == aElemChannels) && ((size[0] == 1) || (size[1] == 1)) &&
            (isContinuous() || (step[1] == (step[2] * static_cast< uint64_t >(size[2])))))))
             ? static_cast< int32_t >((total() * channels()) / static_cast< uint64_t >(aElemChannels))
             : -1;
}

/*******************************************************************************
 * Returns SUMat handle for allocated structure
 ******************************************************************************/
void* vsdk::SUMat::handle(const uint32_t aAccessFlags) const
{
  void* lRetVal;
  /* Data integrity check */
  if(u == static_cast< vsdk::UMatData* >(NULL))
  {
    lRetVal = NULL;
  }
  /* If all ok, continue */
  else
  {
    /* If the device data changed, we need to unmap */
    if(u->deviceCopyObsolete())
    {
      u->currAllocator->unmap(u);
    }

    /* If access write is specific, the host copy will be marked obsolete */
    if((aAccessFlags & static_cast< uint32_t >(ACCESS_WRITE)) != 0U)
    {
      u->markHostCopyObsolete(true);
    }
    lRetVal = u->handle;
  }
  return lRetVal;
}

/*******************************************************************************
 * Fills the ofs with offsets for each dimension
 ******************************************************************************/
void vsdk::SUMat::ndoffset(int64_t* const aOfs) const
{
  int64_t lVal = static_cast< int64_t >(offset);
  for(int32_t lIt0 = 0; lIt0 < dims; lIt0++)
  {
    int64_t lS = static_cast< int64_t >(step.p[lIt0]);
    aOfs[lIt0] = static_cast< int64_t >(lVal / lS);
    lVal -= aOfs[lIt0] * lS;
  }
}

/*******************************************************************************
 * Returns a new matrix header for the specified diagonal
 * (aD=0 - the main diagonal,
 *  >0 - a diagonal from the lower half,
 *  <0 - a diagonal from the upper half)
 ******************************************************************************/
vsdk::SUMat vsdk::SUMat::diag(const int32_t aDiagonal) const
{
  vsdk::SUMat  lMatrix = *this;
  const size_t lEsz    = elemSize();
  int32_t      len;

  /* Main diagonal or lower half */
  if(aDiagonal >= 0)
  {
    len = std::min(cols - aDiagonal, rows);
    lMatrix.offset += lEsz * static_cast< uint32_t >(aDiagonal);
  }
  /* Upper half */
  else
  {
    len = std::min(rows + aDiagonal, cols);
    lMatrix.offset += step[0] * static_cast< uint32_t >(aDiagonal * (-1));
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
    lMatrix.flags &= ~CONTINUOUS_FLAG;
  }
  else
  {
    lMatrix.flags |= CONTINUOUS_FLAG;
  }

  /* Set submatrix flag */
  if((size[0] != 1) || (size[1] != 1))
  {
    lMatrix.flags |= SUBMATRIX_FLAG;
  }

  return lMatrix;
}

/*******************************************************************************
 * Get SMat function
 ******************************************************************************/
vsdk::SMat vsdk::SUMat::doGetMat(const uint32_t aFlags) const
{
  vsdk::SMat lRetVal;

  if(VSDK_CV_XADD(&u->refcount, 1) == 0)
  {
    /* Use allocator to access the data via mapping */
    u->currAllocator->map(u, aFlags);
  }

  /* If everything went well, create the header and return it */
  if((u->data != static_cast< uint8_t* >(NULL)) &&
     (((u->flags & (OAL_USAGE_CACHED | OAL_USAGE_NONCACHED)) == 0U) ||
      (((aFlags & OAL_USAGE_CACHED) != 0U) && ((u->flags & OAL_USAGE_CACHED) != 0U)) ||
      (((aFlags & OAL_USAGE_NONCACHED) != 0U) && ((u->flags & OAL_USAGE_NONCACHED) != 0U))))
  {
    SMat lHdr(dims, size.p, type(), &(u->data[offset]), step.p);
    lHdr.flags     = flags;
    lHdr.u         = u;
    lHdr.datastart = u->data;
    lHdr.data      = &(u->data[offset]);
    lHdr.dataend   = &(u->data[u->size]);
    lHdr.datalimit = lHdr.dataend;

    lRetVal = lHdr;
  }
  /* Otherwise return empty SMat */
  else
  {
    VSDK_CV_XADD_VOID(&u->refcount, -1);
    lRetVal = SMat();
  }
  return lRetVal;
}

/*******************************************************************************
 * Get SMat function
 ******************************************************************************/
vsdk::SMat vsdk::SUMat::getMat(uint32_t aFlags) const
{
  vsdk::SMat lRetVal;

  /* Check for UMatData existance */
  if(u == static_cast< vsdk::UMatData* >(NULL))
  {
    lRetVal = vsdk::SMat();
  }
  else
  {
    // TODO Support ACCESS_READ (ACCESS_WRITE) without unnecessary data transfers
    aFlags |= static_cast< uint32_t >(ACCESS_RW);

    /* Lock the structure and increase refcount */
    vsdk::UMatDataAutoLock lAutoLock(u);

    lRetVal = doGetMat(aFlags);
  }
  return lRetVal;
}

/*******************************************************************************
 * CustomPointerSUMat default constructor
 ******************************************************************************/
vsdk::CustomPointerSUMat::CustomPointerSUMat(const int32_t  aRows,
                                             const int32_t  aCols,
                                             uint32_t       aType,
                                             uint8_t* const apHWAddr,
                                             uint8_t* const apVirtAddr)
    : SUMat(USAGE_DONT_ALLOCATE)
{
  /* Basic constructor, set the type of the matrix */
  /* Set all basic data */
  flags      = vsdk::SUMat::MAGIC_VAL;
  dims       = 0;
  rows       = 0;
  cols       = 0;
  allocator  = static_cast< vsdk::SUMatAllocator* >(NULL);
  usageFlags = USAGE_DONT_ALLOCATE;
  u          = static_cast< vsdk::UMatData* >(NULL);
  offset     = 0U;
  size       = vsdk::MatSize(&rows);

  aType &= vsdk::SUMat::TYPE_MASK;

  bool lAllOK = true;

  /* Check for data integrity, if bad, just return */
  if((dims <= 2) && (rows == aRows) && (cols == aCols) && (type() == aType) &&
     (u != static_cast< vsdk::UMatData* >(NULL)))
  {
    lAllOK = false;
  }

  /* If data are correct, we call createCustom function */
  if(lAllOK)
  {
    int32_t lSz[] = {aRows, aCols};
    createCustom(2, lSz, aType, apHWAddr, usageFlags);

    u->data = apVirtAddr;
  }
}

/*******************************************************************************
 * Helper class for creation of allocator.
 ******************************************************************************/
void vsdk::CustomPointerSUMat::createAllocator(const int32_t        aNDims,
                                               const int32_t* const apSizes,
                                               const uint32_t       aType,
                                               void* const          apHWAddr)
{
  flags = (aType & VSDK_CV_MAT_TYPE_MASK) | MAGIC_VAL;

  /* Set size for this matrix */
  setSizeUMat(*this, aNDims, apSizes, NULL, true);
  offset = 0U;

  /* if the size was set correctly, we allocate the memory */
  if(total() > 0U)
  {
    SUMatAllocator* lpA = allocator;
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    SUMatAllocator* a0 = &gAllocator;
#endif
    if(lpA != static_cast< SUMatAllocator* >(NULL))
    {
      lpA = &gAllocator;
#if !defined __STANDALONE__ && !defined __INTEGRITY__
      a0 = &gAllocator;
#endif
    }
#if !defined __STANDALONE__ && !defined __INTEGRITY__
    try
    {
      u = lpA->allocate(dims, size, static_cast< int32_t >(aType), apHWAddr, step.p, 0, usageFlags);
    }
    catch(...)
    {
      if(lpA != a0)
      {
        u = a0->allocate(dims, size, static_cast< int32_t >(aType), apHWAddr, step.p, 0, usageFlags);
      }
    }
#else
    u = lpA->allocate(dims, size, static_cast< int32_t >(aType), apHWAddr, step.p, 0, USAGE_DEFAULT);
#endif
  }

  /* Add a reference and finalize header */
  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    u->flags |= static_cast< uint32_t >(VSDK_USER_ALLOCATED);
    finalizeHdrUMat(*this);
    addref();
  }
}

/*******************************************************************************
 * Creates the matrix - allocates if necessary
 ******************************************************************************/
void vsdk::CustomPointerSUMat::createCustom(const int32_t              aNDims,
                                            const int32_t* const       apSizes,
                                            uint32_t                   aType,
                                            void* const                apHWAddr,
                                            const vsdk::UMatUsageFlags aUsageFlags)
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMat: Create" << std::endl;
  }
  this->usageFlags = aUsageFlags;

  aType = VSDK_CV_MAT_TYPE(aType);

  /* var to satisfy one exit point */
  bool lAllOK = isUMatDataOK(*this, aNDims, apSizes, aType);

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
    createAllocator(aNDims, apSizes, aType, apHWAddr);
  }
}

/*******************************************************************************
 * Get SMat function
 ******************************************************************************/
vsdk::SMat vsdk::CustomPointerSUMat::getMat(uint32_t) const
{
  vsdk::SMat lRetVal;

  /* Check for UMatData object */
  if(u == static_cast< vsdk::UMatData* >(NULL))
  {
    lRetVal = vsdk::SMat();
  }
  else
  {
    // TODO Support ACCESS_READ (ACCESS_WRITE) without unnecessary data transfers
    /* Auto lock the structure */
    vsdk::UMatDataAutoLock lAutoLock(u);

    /* Increase the refcount */
    VSDK_CV_XADD_VOID(&u->refcount, 1);

    /* If the data are present, create the header object */
    if(u->data != static_cast< uint8_t >(NULL))
    {
      SMat lHdr(dims, size.p, type(), &(u->data[offset]), step.p);
      lHdr.flags     = flags;
      lHdr.u         = u;
      lHdr.datastart = u->data;
      lHdr.data      = &(u->data[offset]);
      lHdr.dataend   = &(u->data[u->size]);
      lHdr.datalimit = lHdr.dataend;
      lRetVal        = lHdr;
    }
    /* Otherwise create empty matrix */
    else
    {
      VSDK_CV_XADD_VOID(&u->refcount, -1);
      lRetVal = SMat();
    }
  }
  return lRetVal;
}

vsdk::CustomPointerSUMat::CustomPointerSUMat(const vsdk::UMatUsageFlags aUsageFlags) : SUMat(aUsageFlags)
{
  /* Default constructor creates a CustomPointerSUMat based on SUMat*/
}

/**
 * Default constructor
 */
vsdk::SUMat::SUMat(const vsdk::UMatUsageFlags aUsageFlags)
    : flags(vsdk::SUMat::MAGIC_VAL), dims(0), rows(0), cols(0), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      usageFlags(aUsageFlags), u(static_cast< vsdk::UMatData* >(NULL)), offset(0U), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* All the members are initialized to NULL except usage flags */
}

/**
 * Constructs 2D matrix of the specified size and type
 */
vsdk::SUMat::SUMat(const int32_t              aRows,
                   const int32_t              aCols,
                   const uint32_t             aType,
                   const vsdk::UMatUsageFlags aUsageFlags)
    : flags(vsdk::SUMat::MAGIC_VAL), dims(0), rows(0), cols(0), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      usageFlags(aUsageFlags), u(static_cast< vsdk::UMatData* >(NULL)), offset(0U), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* Afterwards, create is called. */
  create(aRows, aCols, aType, aUsageFlags);
}

/**
 * Constructs n-dimensional matrix
 */
vsdk::SUMat::SUMat(const int32_t              aNDims,
                   const int32_t* const       apSizes,
                   const uint32_t             aType,
                   const vsdk::UMatUsageFlags aUsageFlags)
    : flags(vsdk::SUMat::MAGIC_VAL), dims(0), rows(0), cols(0), allocator(static_cast< vsdk::SUMatAllocator* >(NULL)),
      usageFlags(aUsageFlags), u(static_cast< vsdk::UMatData* >(NULL)), offset(0UL), size(&rows)
{
  /* Function initializes all member functions in constructor */
  /* Afterwards, create is called. */
  create(aNDims, apSizes, aType, aUsageFlags);
}

/**
 * Copy constructor
 */
vsdk::SUMat::SUMat(const vsdk::SUMat& aMatrix)
    : flags(aMatrix.flags), dims(aMatrix.dims), rows(aMatrix.rows), cols(aMatrix.cols), allocator(aMatrix.allocator),
      usageFlags(aMatrix.usageFlags), u(aMatrix.u), offset(aMatrix.offset), size(&rows)
{
  /* Add a reference */
  addref();

  /* init the steps, which are done after constructor */
  if(aMatrix.dims <= 2)
  {
    step[0] = aMatrix.step[0];
    step[1] = aMatrix.step[1];
  }
  /* If dims are bigger, we use internal copySize function. */
  else
  {
    dims = 0;
    copySize(aMatrix);
  }
}

/**
 * Increases the reference counter; use with care to avoid memleaks
 */
void vsdk::SUMat::addref() const
{
  if(u != static_cast< vsdk::UMatData* >(NULL))
  {
    VSDK_CV_XADD_VOID(&(u->urefcount), 1);
  }

  if((VSDK_CV_USE_DEBUG_OUT != 0U) && (u != static_cast< vsdk::UMatData* >(NULL)))
  {
    std::cout << "SUMat: Add ref " << u->urefcount << std::endl;
  }
}

/**
 * decreases reference counter;
 * deallocates the data when reference counter reaches 0.
 */
void vsdk::SUMat::release()
{
  /* call deallocate function and decrease ref count */
  if((u != static_cast< vsdk::UMatData* >(NULL)) && (VSDK_CV_XADD(&(u->urefcount), -1) == 1))
  {
    deallocate();
  }
  for(int32_t i = 0; i < dims; i++)
  {
    size[i] = 0;
  }
  /* zero everything */
  u = static_cast< vsdk::UMatData* >(NULL);
}

/**
 * returns element type, similar to VSDK_CV_MAT_TYPE(cvmat->type)
 */
uint32_t vsdk::SUMat::type() const
{
  return VSDK_CV_MAT_TYPE(flags);
}

/**
 * returns the total number of matrix elements
 */
uint64_t vsdk::SUMat::total() const
{
  uint64_t lRetVal;
  /* in case there is a classic vector or matrix */
  if(dims <= 2)
  {
    lRetVal = static_cast< uint64_t >(rows) * static_cast< uint64_t >(cols);
  }
  /* arbitrary dimension size */
  else
  {
    uint64_t lP = 1U;
    for(int32_t lI = 0; lI < dims; lI++)
    {
      lP *= static_cast< uint64_t >(size[lI]);
    }
    lRetVal = lP;
  }
  return lRetVal;
}

/**
 * allocates new matrix data unless the matrix already has specified size and
 * type. previous data is unreferenced if needed.
 */
void vsdk::SUMat::create(const int32_t              aRows,
                         const int32_t              aCols,
                         uint32_t                   aType,
                         const vsdk::UMatUsageFlags aUsageFlags)
{
  aType &= vsdk::SUMat::TYPE_MASK;
  if((dims <= 2) && (rows == aRows) && (cols == aCols) && (type() == aType) &&
     (u != static_cast< vsdk::UMatData* >(NULL)))
  {
  }
  else
  {
    int32_t  sz[] = {aRows, aCols};
    int32_t* pSz  = static_cast< int32_t* >(sz);
    create(2, pSz, aType, aUsageFlags);
  }
}

/**
 * Returns a SUMat from specified ROI
 */
vsdk::SUMat vsdk::SUMat::operator()(const vsdk::Range* const apRanges) const
{
  return vsdk::SUMat(*this, apRanges);
}

/**
 * Returns element size in bytes
 */
uint64_t vsdk::SUMat::elemSize() const
{
  return ((dims > 0) ? (step[(dims - 1)]) : (0U));
}

/**
 * Assigned operator
 */
vsdk::SUMat& vsdk::SUMat::operator=(const vsdk::SUMat& aMatrix)
{
  /* If the matrix is "this", we return directly this SUMat */
  if(this != &aMatrix)
  {
    /* Add a reference and check the release on this matrix*/
    static_cast< vsdk::SUMat >(aMatrix).addref();
    release();

    /* Assign flags and dimensions */
    flags = aMatrix.flags;
    if((dims <= 2) && (aMatrix.dims <= 2))
    {
      dims    = aMatrix.dims;
      rows    = aMatrix.rows;
      cols    = aMatrix.cols;
      step[0] = aMatrix.step[0];
      step[1] = aMatrix.step[1];
    }
    /* For dim >=2 we use internal copySize*/
    else
    {
      copySize(aMatrix);
    }

    /* Copy the allocator and use flags*/
    allocator = aMatrix.allocator;
    if(usageFlags == vsdk::USAGE_DEFAULT)
    {
      usageFlags = aMatrix.usageFlags;
    }

    /* And of course the data */
    u      = aMatrix.u;
    offset = aMatrix.offset;
  }
  return *this;
}

/**
 * Returns true iff the matrix data is continuous
 * (i.e. when there are no gaps between successive rows).
 * similar to VSDK_CV_IS_MAT_CONT(cvmat->type)
 */
bool vsdk::SUMat::isContinuous() const
{
  return ((flags & vsdk::SUMat::CONTINUOUS_FLAG) != 0U);
}

/**
 * Returns true if the matrix is a submatrix of another matrix
 */
bool vsdk::SUMat::isSubmatrix() const
{
  return ((flags & vsdk::SUMat::SUBMATRIX_FLAG) != 0U);
}

/**
 * Returns the size of element channel in bytes.
 */
uint32_t vsdk::SUMat::elemSize1() const
{
  return VSDK_CV_ELEM_SIZE1(flags);
}

/**
 * Returns element type, similar to CV_MAT_DEPTH(cvmat->type)
 */
uint32_t vsdk::SUMat::depth() const
{
  return VSDK_CV_MAT_DEPTH(flags);
}

/**
 * Returns element type, similar to VSDK_CV_MAT_CN(cvmat->type)
 */
uint32_t vsdk::SUMat::channels() const
{
  return VSDK_CV_MAT_CN(flags);
}

/**
 * Returns step/elemSize1()
 */
uint64_t vsdk::SUMat::step1(const int32_t aI) const
{
  return (step[aI]) / elemSize1();
}

/**
 * Returns true if matrix data is NULL
 */
bool vsdk::SUMat::empty() const
{
  return ((u == static_cast< vsdk::UMatData* >(NULL)) || (total() == 0UL));
}

/**
 * Returns a SUMat from specified ROI
 */
vsdk::SUMat vsdk::SUMat::operator()(const vsdk::Range aRowRange, const vsdk::Range aColRange) const
{
  return vsdk::SUMat(*this, aRowRange, aColRange);
}

/**
 * Assign ROI operator
 */
vsdk::SUMat vsdk::SUMat::operator()(const vsdk::Rect& aRoi) const
{
  return vsdk::SUMat(*this, aRoi);
}

/**
 * Returns a new matrix header for the specified row
 */
vsdk::SUMat vsdk::SUMat::row(const int32_t aY) const
{
  return vsdk::SUMat(*this, vsdk::Range(aY, aY + 1), vsdk::Range::all());
}

/**
 * Returns a new matrix header for the specified column
 */
vsdk::SUMat vsdk::SUMat::col(const int32_t aX) const
{
  return vsdk::SUMat(*this, vsdk::Range::all(), vsdk::Range(aX, aX + 1));
}

/**
 * Returns a new matrix header for the specified row span
 */
vsdk::SUMat vsdk::SUMat::rowRange(const int32_t aStartRow, const int32_t aEndRow) const
{
  return vsdk::SUMat(*this, vsdk::Range(aStartRow, aEndRow), vsdk::Range::all());
}

/**
 * Returns a new matrix header for the specified row span
 */
vsdk::SUMat vsdk::SUMat::rowRange(const vsdk::Range& aRange) const
{
  return vsdk::SUMat(*this, aRange, vsdk::Range::all());
}

/**
 * Returns a new matrix header for the specified column span
 */
vsdk::SUMat vsdk::SUMat::colRange(const int32_t aStartCol, const int32_t aEndCol) const
{
  return vsdk::SUMat(*this, vsdk::Range::all(), vsdk::Range(aStartCol, aEndCol));
}

/**
 * Returns a new matrix header for the specified column span
 */
vsdk::SUMat vsdk::SUMat::colRange(const vsdk::Range& aRange) const
{
  return vsdk::SUMat(*this, vsdk::Range::all(), aRange);
}
