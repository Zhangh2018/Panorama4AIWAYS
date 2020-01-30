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
#ifndef SUMATDATA_HPP
#define SUMATDATA_HPP

#include "sumat_allocator.hpp"

#include <cstdint>

namespace vsdk
{
extern const uint8_t VSDK_COPY_ON_MAP;
extern const uint8_t VSDK_HOST_COPY_OBSOLETE;
extern const uint8_t VSDK_DEVICE_COPY_OBSOLETE;
extern const uint8_t VSDK_TEMP_UMAT;
extern const uint8_t VSDK_TEMP_COPIED_UMAT;
extern const uint8_t VSDK_USER_ALLOCATED;
extern const uint8_t VSDK_DEVICE_MEM_MAPPED;
/**
 * Structure UMatData keeps records about common data structure passed
 * between multiple SUMat/SMat instances. The structure keeps number of
 * references and decides whenever it's needed to deallocate/map/unmap data
 */
struct UMatData
{
  /**
   * @brief          Creates a UMatData object with specified aAllocator, which
   *                 will accompany the object whole its life
   *
   * @param[in]      aAllocator   Allocator used for creation of the buffer
   */
  UMatData(const vsdk::SUMatAllocator* const aAllocator);

  /**
   * @brief          Destroys a UMatData object. Checks whenever the data
   *                 should be unmapped
   *  or freed
   */
  ~UMatData();

  /**
   * @brief          Tries to obtain a mutex for this structure
   */
  void lock() const;

  /**
   * @brief          Frees the mutex for this structure
   */
  void unlock() const;

  /**
   * @brief          Query for host copy obsolete (mapped memory) flag
   *
   * @return         true if host copy obsolete flag is set.
   */
  bool hostCopyObsolete() const;

  /**
   * @brief          Query for device copy obsolete (physical address) flag
   *
   * @return         true if device copy obsolete flag is set.
   */
  bool deviceCopyObsolete() const;

  /**
   * @brief          Query for "is mapped" flag
   *
   * @return         true if "is mapped" flag is set.
   */
  bool deviceMemMapped() const;

  /**
   * @brief          Query for "copy on map" flag
   *
   * @return         true if "copy on map" flag is set.
   */
  bool copyOnMap() const;

  /**
   * @brief          Query for "is temporary SUMat" flag
   *
   * @return         true if "is temporary SUMat" flag is set.
   */
  bool tempUMat() const;

  /**
   * @brief          Query for "is copied SUMat" flag
   *
   * @return         true if "is copied SUMat" flag is set.
   */
  bool tempCopiedUMat() const;

  /**
   * @brief          Flags the host copy (mapped memory) as obsolete
   *
   * @param[in]      aFlag       If True, the host copy obsolete flag is set.
   *                             The HCO flag is released otherwise
   */
  void markHostCopyObsolete(const int8_t aFlag);

  /**
   * @brief          Flags the device copy (physical address) as obsolete
   *
   * @param[in]      aFlag       If True, the device copy obsolete flag is set.
   *                             The DCO flag is released otherwise
   */
  void markDeviceCopyObsolete(const int8_t aFlag);

  /**
   * @brief          Flags the "is mapped"
   *
   * @param[in]      aFlag       If True, the "is mapped" flag is set.
   *                             The "is mapped" flag is released otherwise
   */
  void markDeviceMemMapped(const int8_t aFlag);

  /**
   * @brief          Previously used allocator
   */
  const vsdk::SUMatAllocator* prevAllocator;

  /**
   * @brief          Currently used allocator
   */
  const vsdk::SUMatAllocator* currAllocator;

  /**
   * @brief          SUMat reference count (atomic add/sub)
   */
  int32_t urefcount;

  /**
   * @brief          All reference count (atomic add/sub)
   */
  int32_t refcount;

  /**
   * @brief          Data pointer (mapped value)
   */
  uint8_t* data;

  /**
   * @brief          Original data pointer (used in OAL). Without any ROI nor
   *                 offset
   */
  uint8_t* origdata;

  /**
   * @brief          Allocated size
   */
  uint64_t size;

  /**
   * @brief          Structure for flags
   */
  uint32_t flags;

  /**
   * @brief          OAL Handle
   */
  void* handle;

  /**
   * @brief          Arbitrary user data pointer added to the structure
   */
  void* userdata;

  /**
   * @brief          Used allocator flags
   */
  int32_t allocatorFlags_;

  /**
   * @brief          Number of mappings
   */
  int32_t mapcount;

  /**
   * @brief          Original UMatData which was this created from
   */
  vsdk::UMatData* originalUMatData;
}; /* class UMatData */

} /* namespace vsdk */

#endif /* UMATDATA_HPP */
