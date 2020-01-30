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

#include "sumat_data.hpp"
#include "sumat_mutex.hpp"

/*******************************************************************************
 * Locks for umats
 ******************************************************************************/
const uint64_t UMAT_NLOCKS = 31U;

static vsdk::Mutex gUmatLocks[UMAT_NLOCKS];

const uint8_t vsdk::VSDK_COPY_ON_MAP          = 0x01U;
const uint8_t vsdk::VSDK_HOST_COPY_OBSOLETE   = 0x02U;
const uint8_t vsdk::VSDK_DEVICE_COPY_OBSOLETE = 0x04U;
const uint8_t vsdk::VSDK_TEMP_UMAT            = 0x08U;
const uint8_t vsdk::VSDK_TEMP_COPIED_UMAT     = 0x18U;
const uint8_t vsdk::VSDK_USER_ALLOCATED       = 0x20U;
const uint8_t vsdk::VSDK_DEVICE_MEM_MAPPED    = 0x40U;

/*******************************************************************************
 * Creates a UMatData object with specified allocator, which will accompany
 * the object whole its life
 ******************************************************************************/
vsdk::UMatData::UMatData(const vsdk::SUMatAllocator* const aAllocator)
{
  /* Debug output */
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMatData: Constructor" << std::endl;
  }
  /* Initialize allocator to default one */
  prevAllocator = aAllocator;
  currAllocator = aAllocator;

  /* Initialize all members to 0 or NULL */
  urefcount        = 0;
  refcount         = 0;
  mapcount         = 0;
  data             = static_cast< uint8_t* >(NULL);
  origdata         = static_cast< uint8_t* >(NULL);
  size             = 0U;
  flags            = 0U;
  handle           = static_cast< void* >(NULL);
  userdata         = static_cast< void* >(NULL);
  allocatorFlags_  = 0;
  originalUMatData = static_cast< vsdk::UMatData* >(NULL);
}

/*******************************************************************************
 * Destroys a UMatData object. Checks whenever the data should be unmapped
 * or freed
 ******************************************************************************/
vsdk::UMatData::~UMatData()
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMatData: Destructor" << std::endl;
  }

  /* Set all data to 0 */
  prevAllocator   = static_cast< vsdk::SUMatAllocator* >(NULL);
  currAllocator   = static_cast< vsdk::SUMatAllocator* >(NULL);
  urefcount       = 0;
  refcount        = 0;
  data            = static_cast< uint8_t* >(NULL);
  origdata        = static_cast< uint8_t* >(NULL);
  size            = 0U;
  flags           = 0U;
  handle          = static_cast< void* >(NULL);
  userdata        = static_cast< void* >(NULL);
  allocatorFlags_ = 0;

  /* Time for deallocating the buffer */
  if(originalUMatData != static_cast< vsdk::UMatData* >(NULL))
  {
    /* decrease both uref and ref counts */
    vsdk::UMatData* lpU = originalUMatData;
    VSDK_CV_XADD_VOID(&(lpU->urefcount), -1);
    VSDK_CV_XADD_VOID(&(lpU->refcount), -1);
    bool lShowWarn = false;

    if(lpU->refcount == 0)
    {
      /* We shouldn't get to this point, problem in getMat/getUMat chain */
      if(lpU->urefcount > 0)
      {
        lShowWarn = true;
      }

      /* if there's any map, we need to unmap it first */
      if(lpU->mapcount != 0)
      {
        if(VSDK_CV_USE_DEBUG_OUT != 0U)
        {
          std::cout << "    Map count is not equal to zero, need to unmap." << std::endl;
        }
        lpU->currAllocator->unmap(lpU);
      }
      /* Else just for debug purposes */
      else
      {
        if(VSDK_CV_USE_DEBUG_OUT != 0U)
        {
          std::cout << "    Map count is equal to zero, doing nothing." << std::endl;
        }
        /* we don't do "map", so we can't do "unmap" */
      }
    }

    /* All refcounts are null, we need to deallocate the memory */
    if((lpU->refcount == 0) && (lpU->urefcount == 0)) /* oops, we need to free resources */
    {
      if(VSDK_CV_USE_DEBUG_OUT != 0U)
      {
        std::cout << "    WARNING, see below:" << std::endl;
      }
      lShowWarn = true;
      /* simulate SUMat::deallocate */
      lpU->currAllocator->deallocate(lpU);
    }
      /* Debug output when error happens */
#ifdef DEBUG_OUTPUT
    if(lShowWarn)
    {
      static int lWarnMessageShown = 0;
      if(lWarnMessageShown++ < 100)
      {
        if(VSDK_CV_USE_DEBUG_OUT != 0U)
        {
          std::cout << std::endl;
          std::cout << "    ! OPENCV warning: getUMat()/getMat() call chain possible problem.";
          std::cout << std::endl;
          std::cout
              << "    !                 Base object is dead, while nested/derived object is still alive or processed.";
          std::cout << std::endl;
          std::cout << "    !                 Please check lifetime of SUMat/SMat objects!";
          std::cout << std::endl;
        }
      }
    }
#else
    (void)lShowWarn;
#endif

    /* Unitialize the umatdata pointer */
    originalUMatData = static_cast< vsdk::UMatData* >(NULL);
  }
}

/*******************************************************************************
 * Tries to obtain a mutex for this structure
 ******************************************************************************/
void vsdk::UMatData::lock() const
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMatData: Lock" << std::endl;
  }
  uint64_t lHash = (uint64_t)((void*)(this));
  lHash          = lHash % UMAT_NLOCKS;
  gUmatLocks[lHash].lock();
}

/*******************************************************************************
 * Frees the mutex for this structure
 ******************************************************************************/
void vsdk::UMatData::unlock() const
{
  if(VSDK_CV_USE_DEBUG_OUT != 0U)
  {
    std::cout << "UMatData: Unlock" << std::endl;
  }
  uint64_t lHash = (uint64_t)((void*)(this));
  lHash          = lHash % UMAT_NLOCKS;
  gUmatLocks[lHash].unlock();
}

/*******************************************************************************
 * Query for host copy obsolete (mapped memory) flag
 ******************************************************************************/
bool vsdk::UMatData::hostCopyObsolete() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_HOST_COPY_OBSOLETE)) != 0U;
}

/*******************************************************************************
 * Query for device copy obsolete (physical address) flag
 ******************************************************************************/
bool vsdk::UMatData::deviceCopyObsolete() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_DEVICE_COPY_OBSOLETE)) != 0U;
}

/*******************************************************************************
 * Query for "is mapped" flag
 ******************************************************************************/
bool vsdk::UMatData::deviceMemMapped() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_DEVICE_MEM_MAPPED)) != 0U;
}

/*******************************************************************************
 * Query for "copy on map" flag
 ******************************************************************************/
bool vsdk::UMatData::copyOnMap() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_COPY_ON_MAP)) != 0U;
}

/*******************************************************************************
 * Query for "is temporary SUMat" flag
 ******************************************************************************/
bool vsdk::UMatData::tempUMat() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_TEMP_UMAT)) != 0U;
}

/*******************************************************************************
 * Query for "is copied SUMat" flag
 ******************************************************************************/
bool vsdk::UMatData::tempCopiedUMat() const
{
  return (flags & static_cast< uint32_t >(vsdk::VSDK_TEMP_COPIED_UMAT)) ==
         static_cast< uint32_t >(vsdk::VSDK_TEMP_COPIED_UMAT);
}

/*******************************************************************************
 * Flags the host copy (mapped memory) as obsolete
 ******************************************************************************/
void vsdk::UMatData::markDeviceMemMapped(const int8_t aFlag)
{
  if(aFlag != 0)
  {
    flags |= static_cast< uint32_t >(vsdk::VSDK_DEVICE_MEM_MAPPED);
  }
  else
  {
    flags &= ~static_cast< uint32_t >(vsdk::VSDK_DEVICE_MEM_MAPPED);
  }
}

/*******************************************************************************
 * Flags the device copy (physical address) as obsolete
 ******************************************************************************/
void vsdk::UMatData::markHostCopyObsolete(const int8_t aFlag)
{
  if(aFlag != 0)
  {
    flags |= static_cast< uint32_t >(vsdk::VSDK_HOST_COPY_OBSOLETE);
  }
  else
  {
    flags &= ~static_cast< uint32_t >(vsdk::VSDK_HOST_COPY_OBSOLETE);
  }
}

/*******************************************************************************
 * Flags the "is mapped"
 ******************************************************************************/
void vsdk::UMatData::markDeviceCopyObsolete(const int8_t aFlag)
{
  if(aFlag != 0)
  {
    flags |= static_cast< uint32_t >(vsdk::VSDK_DEVICE_COPY_OBSOLETE);
  }
  else
  {
    flags &= ~static_cast< uint32_t >(vsdk::VSDK_DEVICE_COPY_OBSOLETE);
  }
}

/*******************************************************************************
 * Function tries to obtain the lock for UMatData
 ******************************************************************************/
vsdk::UMatDataAutoLock::UMatDataAutoLock(vsdk::UMatData* const apU) : u(apU)
{
  u->lock();
}

/*******************************************************************************
 * Function releases the lock for assigned UMatData
 ******************************************************************************/
vsdk::UMatDataAutoLock::~UMatDataAutoLock()
{
  u->unlock();
}
