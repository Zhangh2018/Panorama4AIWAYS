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

#include "umat_mat.hpp"
#include "umat_allocator.hpp"

#include "oal.h"
static vsdk::SUMatAllocator gAllocator;
static vsdk::UMatAllocator  gAllocatorCVCompatible;

vsdk::Mat::Mat() : vsdk::SMat()
{
}

/*******************************************************************************
 * OpenCV Compatibility
 ******************************************************************************/
vsdk::Mat::Mat(const vsdk::Mat& aMatrix) : vsdk::SMat(aMatrix)

{
}

/*******************************************************************************
 * OpenCV Compatibility
 ******************************************************************************/
vsdk::Mat::Mat(const vsdk::SMat& aMatrix) : vsdk::SMat(aMatrix)

{
}

vsdk::Mat::Mat(const int32_t aRows, const int32_t aCols, const uint32_t aType) : vsdk::SMat(aRows, aCols, aType)
{
}

vsdk::Mat::Mat(const int32_t aNDims, const int32_t* const apSizes, const uint32_t aType)
    : vsdk::SMat(aNDims, apSizes, aType)
{
}

vsdk::Mat::Mat(const int32_t aRows, const int32_t aCols, const uint32_t aType, void* const apData, uint64_t aStep)
    : vsdk::SMat(aRows, aCols, aType, apData, aStep)
{
}

vsdk::Mat::Mat(const int32_t         aNDims,
               const int32_t* const  apSizes,
               const uint32_t        aType,
               void* const           apData,
               const uint64_t* const aSteps)
    : vsdk::SMat(aNDims, apSizes, aType, apData, aSteps)
{
}

vsdk::Mat::Mat(const vsdk::Mat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
    : vsdk::SMat(aMatrix, aRowRange, aColRange)
{
}

vsdk::Mat::Mat(const vsdk::Mat& aMatrix, const vsdk::Rect& aRoi) : vsdk::SMat(aMatrix, aRoi)
{
}

vsdk::Mat::Mat(const vsdk::Mat& aMatrix, const vsdk::Range* const aRanges) : vsdk::SMat(aMatrix, aRanges)
{
}

vsdk::Mat::~Mat()
{
  vsdk::Mat::release();
  if(step.p != step.buf)
  {
    free(step.p);
    step.p = NULL;
  }
}

void vsdk::Mat::deallocate()
{
  if(u != static_cast< vsdk::UMatData* >(NULL) && u->currAllocator != NULL)
  {
    if(OAL_MemoryReturnAddress(u->handle, ACCESS_PHY) != NULL)
    {
      u->currAllocator->unmap(u);
    }
    else
    {
      cv::Mat a;
      a.getDefaultAllocator()->unmap((cv::UMatData*)u);
    }
  }
  u = static_cast< vsdk::UMatData* >(NULL);
}

void vsdk::Mat::release()
{
  /* call deallocate function and decrease ref count */
  if((u != static_cast< vsdk::UMatData* >(NULL)) && (VSDK_CV_XADD(&u->refcount, -1) == 1))
  {
    vsdk::Mat::deallocate();
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
 * OpenCV Compatibility
 ******************************************************************************/
vsdk::Mat& vsdk::Mat::operator=(const vsdk::SMat& aMatrix)
{
  if(aMatrix.u)
    VSDK_CV_XADD(&aMatrix.u->refcount, 1);

  release();
  flags = aMatrix.flags;
  if(dims <= 2 && aMatrix.dims <= 2)
  {
    dims    = aMatrix.dims;
    rows    = aMatrix.rows;
    cols    = aMatrix.cols;
    step[0] = aMatrix.step[0];
    step[1] = aMatrix.step[1];
  }
  else
    copySize(aMatrix);
  data      = aMatrix.data;
  datastart = aMatrix.datastart;
  dataend   = aMatrix.dataend;
  datalimit = aMatrix.datalimit;
  allocator = aMatrix.allocator;
  u         = (vsdk::UMatData*)aMatrix.u;
  return *this;
}

vsdk::Mat& vsdk::Mat::operator=(const cv::Mat& aMatrix)
{
  if(aMatrix.u)
    VSDK_CV_XADD(&aMatrix.u->refcount, 1);

  release();
  flags = aMatrix.flags;
  if(dims <= 2 && aMatrix.dims <= 2)
  {
    dims    = aMatrix.dims;
    rows    = aMatrix.rows;
    cols    = aMatrix.cols;
    step[0] = aMatrix.step[0];
    step[1] = aMatrix.step[1];
  }
  else
    vsdk::Mat::copySize((vsdk::Mat&)aMatrix);
  data      = aMatrix.data;
  datastart = aMatrix.datastart;
  dataend   = aMatrix.dataend;
  datalimit = aMatrix.datalimit;
  //allocator = &gAllocatorCVCompatible;
  u = (vsdk::UMatData*)aMatrix.u;
  //u->currAllocator = &gAllocatorCVCompatible;
  return *this;
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
vsdk::Mat::Mat(const cv::Mat& aMatrix)
{
  *this = aMatrix;
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
vsdk::Mat::operator cv::Mat() const
{
  cv::Mat cvmat;

  cvmat.flags = flags;
  cvmat.dims  = dims;
  cvmat.rows  = rows;
  cvmat.cols  = cols;
  //cvmat.allocator = &gAllocatorCVCompatible;
  cvmat.u = (cv::UMatData*)u;

  if(cvmat.u != NULL)
  {
    cvmat.u->currAllocator = &gAllocatorCVCompatible;
  }

  cvmat.size      = cv::MatSize(&(cvmat.rows));
  cvmat.data      = data;
  cvmat.datastart = datastart;
  cvmat.dataend   = dataend;
  cvmat.datalimit = datalimit;

  if(u)
    VSDK_CV_XADD(&u->refcount, 1);
  if(dims <= 2)
  {
    cvmat.step[0] = step[0];
    cvmat.step[1] = step[1];
  }
  else
  {
    cvmat.dims = 0;
    cvmat.copySize(*(cv::Mat*)this);
  }

  return cvmat;
}
