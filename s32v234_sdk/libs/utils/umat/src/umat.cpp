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

#include "umat.hpp"
#include "umat_allocator.hpp"
#include "oal.h"

static vsdk::SUMatAllocator gAllocator;
static vsdk::UMatAllocator  gAllocatorCVCompatible;

/*******************************************************************************
 * OpenCV Compatibility
 ******************************************************************************/
vsdk::UMat::UMat(const vsdk::UMat& aMatrix) : vsdk::SUMat(aMatrix)

{
}

vsdk::UMat::UMat(const vsdk::SUMat& aMatrix) : vsdk::SUMat(aMatrix)

{
}

vsdk::UMat::UMat(const vsdk::UMatUsageFlags aUsageFlags) : vsdk::SUMat(aUsageFlags)
{
}
/*******************************************************************************
 * OpenCV Compatibility
 ******************************************************************************/
vsdk::UMat& vsdk::UMat::operator=(const vsdk::SUMat& aMatrix)
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

vsdk::UMat::UMat(const int32_t aRows, const int32_t aCols, const uint32_t aType, const vsdk::UMatUsageFlags aUsageFlags)
    : vsdk::SUMat(aRows, aCols, aType, aUsageFlags)
{
}

vsdk::UMat::UMat(const int32_t              aNDims,
                 const int32_t* const       apSizes,
                 const uint32_t             aType,
                 const vsdk::UMatUsageFlags aUsageFlags)
    : vsdk::SUMat(aNDims, apSizes, aType, aUsageFlags)
{
}

vsdk::UMat::UMat(const vsdk::UMat& aMatrix, const vsdk::Range& aRowRange, const vsdk::Range& aColRange)
    : vsdk::SUMat(aMatrix, aRowRange, aColRange)
{
}

vsdk::UMat::UMat(const vsdk::UMat& aMatrix, const vsdk::Rect& aRoi) : vsdk::SUMat(aMatrix, aRoi)
{
}

vsdk::UMat::UMat(const vsdk::UMat& aMatrix, const vsdk::Range* const apRanges) : vsdk::SUMat(aMatrix, apRanges)
{
}

vsdk::UMat::operator cv::UMat() const
{
  cv::UMat lCVUMat;
  lCVUMat.flags      = flags;
  lCVUMat.dims       = dims;
  lCVUMat.rows       = rows;
  lCVUMat.cols       = cols;
  lCVUMat.allocator  = &gAllocatorCVCompatible;
  lCVUMat.usageFlags = (cv::UMatUsageFlags)usageFlags;
  lCVUMat.u          = (cv::UMatData*)u;
  
  if(u != NULL)
  {
    lCVUMat.u->currAllocator = &gAllocatorCVCompatible;
  }
  lCVUMat.offset = offset;
  lCVUMat.size   = cv::MatSize(&(lCVUMat.rows));

  lCVUMat.addref();
  if(dims <= 2)
  {
    lCVUMat.step[0] = step[0];
    lCVUMat.step[1] = step[1];
  }
  else
  {
    lCVUMat.dims = 0;
    lCVUMat.copySize(*(cv::UMat*)this);
  }

  return lCVUMat;
}

vsdk::UMat& vsdk::UMat::operator=(const cv::UMat& aMatrix)
{
  if(!aMatrix.empty() && OAL_MemoryReturnAddress(aMatrix.u->handle, ACCESS_PHY) != 0)
  {
    const_cast< cv::UMat& >(aMatrix).addref();
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
      copySize(*(vsdk::UMat*)&aMatrix);
    //allocator = &gAllocatorCVCompatible;
    if(usageFlags == vsdk::USAGE_DEFAULT)
      usageFlags = (vsdk::UMatUsageFlags)aMatrix.usageFlags;
    u = (vsdk::UMatData*)aMatrix.u;
    //u->currAllocator = &gAllocatorCVCompatible;
    offset = aMatrix.offset;
  }
  else
  {
    if(aMatrix.empty())
    {
      *this = vsdk::UMat();
    }
    else
    {
      release();
      int32_t lSz[] = {aMatrix.rows, aMatrix.cols};
      create(2, lSz, aMatrix.type(), (vsdk::UMatUsageFlags)aMatrix.usageFlags);
      cv::Mat   lCVMat   = aMatrix.getMat(0);
      vsdk::Mat lVsdkMat = getMat(OAL_USAGE_CACHED);
      memcpy(lVsdkMat.data, lCVMat.data, total() * elemSize());
    }
  }
  return *this;
}

vsdk::UMat::UMat(const cv::UMat& aMatrix)
{
  *this = aMatrix;
}

/*******************************************************************************
 * Destructor - calls release()
 ******************************************************************************/
vsdk::UMat::~UMat()
{
  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMat: Destructor, refcount " << ((u != static_cast< vsdk::UMatData* >(NULL)) ? u->urefcount : 0)
              << std::endl;
  }

  /* Call release function */
  release();

  /* If step buffer was not allocated here, delete it */
  if(step.p != step.buf)
  {
    delete[] step.p;
    step.p = NULL;
  }
}

/*******************************************************************************
 * Deallocates the matrix data
 ******************************************************************************/
void vsdk::UMat::deallocate()
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMat: Deallocate" << std::endl;
  }
  if(OAL_MemoryReturnAddress(u->handle, ACCESS_PHY) != NULL)
  {
    u->currAllocator->deallocate(u);
  }
  else
  {
    cv::Mat a;
    a.getDefaultAllocator()->deallocate((cv::UMatData*)u);
  }
  u = static_cast< vsdk::UMatData* >(NULL);
}

void vsdk::UMat::release()
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

/*******************************************************************************
 * Get SMat function
 ******************************************************************************/
cv::Mat vsdk::UMat::doGetMat(const uint32_t aFlags) const
{
  cv::Mat lRetVal;

  if(VSDK_CV_XADD(&u->refcount, 1) == 0)
  {
    /* Use allocator to access the data via mapping */
    if (u && u->currAllocator)
      u->currAllocator->map(u, aFlags);
  }

  /* If everything went well, create the header and return it */
  if((u->data != static_cast< uint8_t* >(NULL)) &&
     (((u->flags & (OAL_USAGE_CACHED | OAL_USAGE_NONCACHED)) == 0U) ||
      (((aFlags & OAL_USAGE_CACHED) != 0U) && ((u->flags & OAL_USAGE_CACHED) != 0U)) ||
      (((aFlags & OAL_USAGE_NONCACHED) != 0U) && ((u->flags & OAL_USAGE_NONCACHED) != 0U))))
  {
    cv::Mat lHdr((int) dims, (int32_t*) size.p, (int) type(), (void*) &(u->data[offset]), (size_t*) step.p);
    lHdr.flags     = flags;
    lHdr.u          = (cv::UMatData*)u;
    lHdr.u = (cv::UMatData*)u;

    if(lHdr.u != NULL)
    {
      lHdr.u->currAllocator = &gAllocatorCVCompatible;
    }
  
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
    lRetVal = cv::Mat();
  }
  return lRetVal;
}

/*******************************************************************************
 * Get SMat function
 ******************************************************************************/
cv::Mat vsdk::UMat::getMat(uint32_t aFlags) const
{
  cv::Mat lRetVal;
  /* Check for UMatData existance */
  if(u == static_cast< vsdk::UMatData* >(NULL))
  {
    lRetVal = cv::Mat();
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
