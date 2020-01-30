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

#include "oal.h"

#include "sumat_mat.hpp"

#include <iostream>

/*******************************************************************************
 * Constructor...
 ******************************************************************************/
vsdk::SUMatAllocator::SUMatAllocator()
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Init" << std::endl;
  }
}

/*******************************************************************************
 * Descructor...
 ******************************************************************************/
vsdk::SUMatAllocator::~SUMatAllocator()
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Deinit" << std::endl;
  }
}

/*******************************************************************************
 * Function is called with specific information and should allocate the
 * internal buffer for a new SUMat
 **************************************************std::cout****************************/
vsdk::UMatData* vsdk::SUMatAllocator::allocate(const int32_t        aDims,
                                               const int32_t* const apSizes,
                                               const int32_t        aType,
                                               void* const          apData,
                                               uint64_t* const      apStep,
                                               const int32_t /*aFlags*/,
                                               const vsdk::UMatUsageFlags aUsageFlags) const
{
  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Allocate" << std::endl;
  }

  /* First, we need to compute the size of data to be allocated (according to ndims and sizes) */
  uint64_t lTotal = VSDK_CV_ELEM_SIZE(static_cast< uint32_t >(aType));
  for(int32_t lIt = aDims - 1; lIt >= 0; --lIt)
  {
    if(apStep != static_cast< uint64_t* >(NULL))
    {
      if((apData != NULL) && (apStep[lIt] != VSDK_CV_AUTOSTEP))
      {
        lTotal = apStep[lIt];
      }
      else
      {
        apStep[lIt] = lTotal;
      }
    }
    lTotal *= static_cast< uint64_t >(apSizes[lIt]);
  }

  /* Debug out with computed sizes */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "    size: " << lTotal << " bytes; dims: " << aDims << "; type: " << aType << ";" << std::endl;
  }

  uint32_t lAllocFlags;

  /* Big if/else to set the correct flags (OAL specific) */

  lAllocFlags = static_cast< uint32_t >(OAL_ALLOC_AUTO);

  /* If none, we set alloc auto */
  if(static_cast< uint32_t >(aUsageFlags) != 0U)
  {
    /* Allocate from DDR0 */
    if((static_cast< uint32_t >(aUsageFlags) & static_cast< uint32_t >(USAGE_DDR0)) != 0U)
    {
      lAllocFlags = static_cast< uint32_t >(OAL_ALLOC_DDR0);
    }
    /* Allocate from DDR1 */
    if((static_cast< uint32_t >(aUsageFlags) & static_cast< uint32_t >(USAGE_DDR1)) != 0U)
    {
      lAllocFlags = static_cast< uint32_t >(OAL_ALLOC_DDR1);
    }
    /* Allocate from single SRAM */
    if((static_cast< uint32_t >(aUsageFlags) & static_cast< uint32_t >(USAGE_SSRAM)) != 0U)
    {
      lAllocFlags = static_cast< uint32_t >(OAL_ALLOC_SRAM_SINGLE);
    }
    /* Allocate from vector SRAM */
    if((static_cast< uint32_t >(aUsageFlags) & static_cast< uint32_t >(USAGE_MSRAM)) != 0U)
    {
      lAllocFlags = static_cast< uint32_t >(OAL_ALLOC_SRAM_MULTI);
    }
  }

  /* Now, we can call OAL to allocate right memory chunk */
  uint8_t* lpData = (apData != NULL)
                        ? static_cast< uint8_t* >(apData)
                        : static_cast< uint8_t* >(OAL_MemoryAllocFlag(static_cast< uint32_t >(lTotal), lAllocFlags));

  /* Create UMatData structure */
  /* Init the allocated data handle */
  vsdk::UMatData* const lpUMatData = new vsdk::UMatData(this);
  lpUMatData->handle               = lpData;
  lpUMatData->origdata             = lpData;
  lpUMatData->data                 = static_cast< uint8_t* >(NULL);
  lpUMatData->size                 = lTotal;

  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "    Allocated handle: " << lpUMatData->handle << std::endl;
  }

  /* If there's Don't allocate flag (buffer is non OAL), we should copy the data when mapped to OAL memory */
  if((apData != NULL) && ((static_cast< uint32_t >(aUsageFlags) & static_cast< uint32_t >(USAGE_DONT_ALLOCATE)) == 0U))
  {
    lpUMatData->flags |= static_cast< uint32_t >(VSDK_USER_ALLOCATED);
    if(OAL_MemoryReturnAddress(apData, ACCESS_PHY) == NULL)
    {
      lpUMatData->flags |= static_cast< uint32_t >(VSDK_COPY_ON_MAP);
    }
  }
  return lpUMatData;
}

/*******************************************************************************
 * The function returns the info when the allocation will be performed for umat
 ******************************************************************************/
bool vsdk::SUMatAllocator::allocate(const vsdk::UMatData* const apData,
                                    const int32_t /*aAccessFlags*/,
                                    const vsdk::UMatUsageFlags /*lUsageFlags*/) const
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Allocate wrapper" << std::endl;
  }

  bool lRetVal;
  if(apData == static_cast< vsdk::UMatData* >(NULL))
  {
    lRetVal = false;
  }
  else
  {
    lRetVal = true;
  }
  return lRetVal;
}

/*******************************************************************************
 * Function frees the UMatData contents (via OAL)
 ******************************************************************************/
void vsdk::SUMatAllocator::deallocate(vsdk::UMatData* const apData) const
{
  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Deallocate" << std::endl;
  }

  if(apData != static_cast< vsdk::UMatData* >(NULL))
  {
    /* If we have OAL allocated memory, we should deallocate via OAL */
    if((apData->flags & static_cast< uint32_t >(VSDK_USER_ALLOCATED)) == 0U)
    {
      if(VSDK_CV_USE_DEBUG_OUT != 0U)
      {
        std::cout << "    Freeing handle: " << apData->handle << std::endl;
      }

      /* Use OAL to deallocate */
      if(OAL_MemoryFree(apData->handle) == OAL_FAILURE)
      {
        std::cout << "ERROR in deallocating OAL memory." << std::endl;
      }

      /* Set handle to 0 */
      apData->handle = static_cast< void* >(NULL);
    }
    delete apData;
  }
}

/*******************************************************************************
 * Function maps the UMatData to the virtual space with specific access flags
 ******************************************************************************/
void vsdk::SUMatAllocator::map(vsdk::UMatData* const apData, uint32_t aAccessFlags) const
{
  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: Map " << apData->handle << std::endl;
  }

  /* If there's no flag about cached or uncached, set the default */
  if(((aAccessFlags & OAL_USAGE_CACHED) == 0U) && ((aAccessFlags & OAL_USAGE_NONCACHED) == 0U))
  {
    aAccessFlags |= OAL_USAGE_CACHED;
  }

  if(apData != static_cast< vsdk::UMatData* >(NULL))
  {
    /* If data are not null means the data are mapped. They need to be unmapped if different mapping flag is requested */
    /* This can be done only if refcount is 1, otherwise we would unmap something else too */
    if((apData->refcount == 1) &&
       ((((apData->flags & OAL_USAGE_CACHED) != 0U) && ((aAccessFlags & OAL_USAGE_NONCACHED) != 0U)) ||
        (((apData->flags & OAL_USAGE_NONCACHED) != 0U) && ((aAccessFlags & OAL_USAGE_CACHED) != 0U))))
    {
      apData->flags &= ~(OAL_USAGE_CACHED | OAL_USAGE_NONCACHED);

      /* Call OAL unmap */
      OAL_MemoryUnmap(apData->handle);
    }

    /* We are mapping with non cached policy */
    if((OAL_USAGE_NONCACHED & aAccessFlags) != 0U)
    {
      /* Use OAL to do it */
      apData->data = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apData->handle, ACCESS_NCH_NB));

      /* If success, set correct flag */
      if(apData->data != static_cast< uint8_t* >(NULL))
      {
        apData->flags |= OAL_USAGE_NONCACHED;
      }
    }
    else
    {
      /* We are mapping with cached policy */
      if((OAL_USAGE_CACHED & aAccessFlags) != 0U)
      {
        /* Use OAL to do it */
        apData->data = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apData->handle, ACCESS_CH_WB));

        /* If success, set correct flag */
        if(apData->data != static_cast< uint8_t* >(NULL))
        {
          apData->flags |= OAL_USAGE_CACHED;
        }
      }
    }

    /* If there's forced non flushing, we need to set correct flag */
    if((aAccessFlags & static_cast< uint32_t >(OAL_USAGE_FORCE_NOTFLUSH)) != 0U)
    {
      apData->flags |= OAL_USAGE_FORCE_NOTFLUSH;
    }

    /* If the data are correctly mapped, we need to set correct flags */
    if(apData->data != static_cast< uint8_t* >(NULL))
    {
      apData->markDeviceMemMapped(true);
      if((aAccessFlags & static_cast< uint32_t >(ACCESS_WRITE)) != 0U)
      {
        apData->markDeviceCopyObsolete(true);
      }

      if(((aAccessFlags & static_cast< uint32_t >(ACCESS_READ)) != 0U) && apData->hostCopyObsolete())
      {
        apData->markHostCopyObsolete(false);
      }
    }
  }
}

/*******************************************************************************
 * Function unmaps the existing virtual space mapping (helper)
 ******************************************************************************/
void vsdk::SUMatAllocator::doUnmap(vsdk::UMatData* const apData) const
{
  /* Are cached and NOT non cached */
  if(((apData->flags & OAL_USAGE_CACHED) != 0U) && ((OAL_USAGE_FORCE_NOTFLUSH & apData->flags) == 0U))
  {
    if(VSDK_CV_USE_DEBUG_OUT != 0U)
    {
      std::cout << "    Flushing..." << std::endl;
    }

    /* Perform the flush */
    OAL_MemoryFlushAndInvalidate(OAL_MemoryReturnAddress(apData->handle, ACCESS_CH_WB),
                                 static_cast< uint32_t >(apData->size));
  }

  /* Set correctly all flags after unmap */
  apData->flags &= ~OAL_USAGE_FORCE_NOTFLUSH;
  apData->markDeviceMemMapped(false);
  apData->data = static_cast< uint8_t* >(NULL);
  apData->markDeviceCopyObsolete(false);
  apData->markHostCopyObsolete(true);
}

/*******************************************************************************
 * Function unmaps the existing virtual space mapping
 ******************************************************************************/
void vsdk::SUMatAllocator::unmap(vsdk::UMatData* apData) const
{
  /* Lock the buffer */
  vsdk::UMatDataAutoLock lock(apData);

  /* Debug out */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "SUMatAllocator: unmap" << std::endl;
  }

  /* If the ref counts are zero, we can deallocate data */
  if((apData->urefcount == 0) && (apData->refcount == 0))
  {
    deallocate(apData);
    apData = static_cast< vsdk::UMatData* >(NULL);
  }
  /* Otherwise, the data are kept, unmapped and flushed */
  else if (apData->refcount == 0)
  {
    /* In case the data are allocated in OAL */
    if((apData->flags & static_cast< uint32_t >(VSDK_USER_ALLOCATED)) == 0U)
    {
      doUnmap(apData);
    }
  }
}

/*******************************************************************************
 * Allocator download function (helper)
 ******************************************************************************/
void vsdk::SUMatAllocator::doDownload(const vsdk::SMat&           aSrc,
                                      const vsdk::SMat&           aDst,
                                      const vsdk::UMatData* const apData) const
{
  const vsdk::SMat*     lpArrays[] = {&aSrc, &aDst};
  uint8_t*              lpPtrs[2];
  vsdk::NAryMatIterator lIterMat(lpArrays, lpPtrs, 2);
  size_t                lIt;

  /* Use iterator to go through matrix planes and memcpy them */
  for(lIt = 0U; lIt < lIterMat.nplanes; ++lIt)
  {
    void* lDest = memcpy(lpPtrs[1], lpPtrs[0], lIterMat.size);

    if(lDest != lpPtrs[1])
    {
      std::cout << "Error in memcpy" << std::endl;
    }

    ++lIterMat;
  }

  /* Unmap the pointers, if they were mapped here */
  if(apData->data == static_cast< uint8_t* >(NULL))
  {
    OAL_MemoryUnmap(apData->handle);
  }
}
/*******************************************************************************
 * Allocator download function
 ******************************************************************************/
void vsdk::SUMatAllocator::download(const vsdk::UMatData* const apData,
                                    void* const                 aDst,
                                    const int32_t               aDims,
                                    const uint64_t* const       aSz,
                                    const uint64_t* const       aSrcOfs,
                                    const uint64_t* const       aSrcStep,
                                    const uint64_t* const       aDstStep) const
{
  /* Main check if the data are not null */
  if(apData != static_cast< vsdk::UMatData* >(NULL))
  {
    int32_t* const lISz     = new int32_t[VSDK_CV_MAX_DIM];
    uint8_t*       lpSrcPtr = apData->data;

    /* If data are null, we need to map them (SUMat case) */
    if(lpSrcPtr == static_cast< uint8_t* >(NULL))
    {
      lpSrcPtr = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apData->handle, ACCESS_CH_WB));
    }

    bool lOk = true;
    /* Iterate through all dimensions */
    for(int32_t lIt = 0; lIt < aDims; lIt++)
    {
      if(aSz[lIt] == 0U)
      {
        lOk = false;
        break;
      }

      /* shift the pointer for specific offset as parameter */
      if(aSrcOfs != static_cast< uint64_t* >(NULL))
      {
        lpSrcPtr += aSrcOfs[lIt] * ((lIt <= (aDims - 2)) ? aSrcStep[lIt] : 1U);
      }
      lISz[lIt] = static_cast< int32_t >(aSz[lIt]);
    }

    /* If still ok, we can copy */
    if(lOk)
    {
      /* Create helper SMat with computed pointers */
      vsdk::SMat lSrc(aDims, lISz, VSDK_CV_8U, lpSrcPtr, aSrcStep);
      vsdk::SMat lDst(aDims, lISz, VSDK_CV_8U, aDst, aDstStep);

      doDownload(lSrc, lDst, apData);
    }
    delete[] lISz;
  }
}

/*******************************************************************************
 * Allocator upload function (helper)
 ******************************************************************************/
void vsdk::SUMatAllocator::doUpload(const vsdk::SMat&           aSrc,
                                    const vsdk::SMat&           aDst,
                                    const vsdk::UMatData* const apData) const
{
  const vsdk::SMat*     lpArrays[] = {&aSrc, &aDst};
  uint8_t*              lpPtrs[2];
  vsdk::NAryMatIterator lIterMat(lpArrays, lpPtrs, 2);
  size_t                lIt;

  /* Use iterator to go through matrix planes and memcpy them */
  for(lIt = 0U; lIt < lIterMat.nplanes; ++lIt)
  {
    void* lDest = memcpy(lpPtrs[1], lpPtrs[0], lIterMat.size);

    if(lDest != lpPtrs[1])
    {
      std::cout << "Error in memcpy" << std::endl;
    }

    /* Flush the OAL memory */
    OAL_MemoryFlushAndInvalidate(lpPtrs[1], static_cast< uint32_t >(lIterMat.size));

    ++lIterMat;
  }

  /* Unmap the pointers, if they were mapped here */
  if(apData->data == static_cast< uint8_t* >(NULL))
  {
    OAL_MemoryUnmap(apData->handle);
  }
}
/*******************************************************************************
 * Allocator upload function
 ******************************************************************************/
void vsdk::SUMatAllocator::upload(const vsdk::UMatData* const apData,
                                  void* const                 aSrc,
                                  const int32_t               aDims,
                                  const uint64_t* const       aSz,
                                  const uint64_t* const       aDstOfs,
                                  const uint64_t* const       aDstStep,
                                  const uint64_t* const       aSrcStep) const
{
  /* Main check if the data are not null */
  if(apData != static_cast< vsdk::UMatData* >(NULL))
  {
    int32_t* const lISz     = new int32_t[VSDK_CV_MAX_DIM];
    uint8_t*       lpDstPtr = apData->data;

    /* If data are null, we need to map them (SUMat case) */
    if(lpDstPtr == static_cast< uint8_t* >(NULL))
    {
      lpDstPtr = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apData->handle, ACCESS_CH_WB));
    }

    bool lOk = true;

    /* Iterate through all dimensions */
    for(int32_t lIt = 0; lIt < aDims; lIt++)
    {
      if(aSz[lIt] == 0U)
      {
        lOk = false;
        break;
      }

      /* shift the pointer for specific offset as parameter */
      if(aDstOfs != static_cast< uint64_t* >(NULL))
      {
        lpDstPtr += aDstOfs[lIt] * ((lIt <= (aDims - 2)) ? aDstStep[lIt] : 1U);
      }
      lISz[lIt] = static_cast< int32_t >(aSz[lIt]);
    }

    /* If still ok, we can copy */
    if(lOk)
    {
      /* Create helper SMat with computed pointers */
      vsdk::SMat lSrc(aDims, lISz, VSDK_CV_8U, aSrc, aSrcStep);
      vsdk::SMat lDst(aDims, lISz, VSDK_CV_8U, lpDstPtr, aDstStep);

      doUpload(lSrc, lDst, apData);
    }
    delete[] lISz;
  }
}

/*******************************************************************************
 * Allocator copy function (helper)
 * Copies the matrix contents to another SUMat
 ******************************************************************************/
void vsdk::SUMatAllocator::doCopy(const vsdk::SMat&           aSrc,
                                  const vsdk::SMat&           aDst,
                                  const vsdk::UMatData* const apSrcData,
                                  const vsdk::UMatData* const apDstData) const
{
  /* Create helper SMat with computed pointers */

  const vsdk::SMat*     lpArrays[] = {&aSrc, &aDst};
  uint8_t*              lpPtrs[2];
  vsdk::NAryMatIterator lIterMat(lpArrays, lpPtrs, 2);
  size_t                lIt;

  /* Use iterator to go through matrix planes and memcpy them */
  for(lIt = 0U; lIt < lIterMat.nplanes; ++lIt)
  {
    void* lDest = memcpy(lpPtrs[1], lpPtrs[0], lIterMat.size);

    if(lDest != lpPtrs[1])
    {
      std::cout << "Error in memcpy" << std::endl;
    }

    /* Flush the OAL memory */
    OAL_MemoryFlushAndInvalidate(lpPtrs[1], static_cast< uint32_t >(lIterMat.size));

    ++lIterMat;
  }

  /* Unmap the pointers, if they were mapped here */
  if(apSrcData->data == static_cast< uint8_t* >(NULL))
  {
    OAL_MemoryUnmap(apSrcData->handle);
  }

  /* Unmap the pointers, if they were mapped here */
  if(apDstData->data == static_cast< uint8_t* >(NULL))
  {
    OAL_MemoryUnmap(apDstData->handle);
  }
}
/*******************************************************************************
 * Allocator copy function
 * Copies the matrix contents to another SUMat
 ******************************************************************************/
void vsdk::SUMatAllocator::copy(const vsdk::UMatData* const apSrcData,
                                const vsdk::UMatData* const apDstData,
                                const int32_t               aDims,
                                const uint64_t* const       aSz,
                                const uint64_t* const       aSrcOfs,
                                const uint64_t* const       aSrcStep,
                                const uint64_t* const       aDstOfs,
                                const uint64_t* const       aDstStep,
                                const int8_t /*aSync*/) const
{
  /* Main check if the both data are not null */
  if((apSrcData != static_cast< vsdk::UMatData* >(NULL)) && (apDstData != static_cast< vsdk::UMatData* >(NULL)))
  {
    /* Init local data pointers */
    int32_t* const lISz     = new int32_t[VSDK_CV_MAX_DIM];
    uint8_t*       lpSrcPtr = apSrcData->data;
    uint8_t*       lpDstPtr = apDstData->data;

    /* If data are null, we need to map them (SUMat case) */
    if(lpSrcPtr == static_cast< uint8_t* >(NULL))
    {
      lpSrcPtr = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apSrcData->handle, ACCESS_CH_WB));
    }

    /* If data are null, we need to map them (SUMat case) */
    if(lpDstPtr == static_cast< uint8_t* >(NULL))
    {
      lpDstPtr = static_cast< uint8_t* >(OAL_MemoryReturnAddress(apDstData->handle, ACCESS_CH_WB));
    }

    bool lOk = true;

    /* Iterate through all dimensions */
    for(int32_t lIt = 0; lIt < aDims; lIt++)
    {
      if(aSz[lIt] == 0U)
      {
        lOk = false;
        break;
      }

      /* shift the pointer for specific offset as parameter */
      if(aSrcOfs != static_cast< uint64_t* >(NULL))
      {
        lpSrcPtr += aSrcOfs[lIt] * ((lIt <= (aDims - 2)) ? aSrcStep[lIt] : 1U);
      }

      /* shift the pointer for specific offset as parameter */
      if(aDstOfs != static_cast< uint64_t* >(NULL))
      {
        lpDstPtr += aDstOfs[lIt] * ((lIt <= (aDims - 2)) ? aDstStep[lIt] : 1U);
      }
      lISz[lIt] = static_cast< int32_t >(aSz[lIt]);
    }

    /* If still ok, we can copy */
    if(lOk)
    {
      /* Create helper SMat with computed pointers */
      vsdk::SMat lSrc(aDims, lISz, VSDK_CV_8U, lpSrcPtr, aSrcStep);
      vsdk::SMat lDst(aDims, lISz, VSDK_CV_8U, lpDstPtr, aDstStep);
      doCopy(lSrc, lDst, apSrcData, apDstData);
    }
    delete[] lISz;
  }
}
