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

#ifndef SUMATALLOCATOR_HPP
#define SUMATALLOCATOR_HPP

#include "sumat_defines.hpp"

#include <cstdint>

namespace vsdk
{
class SMat;
struct UMatData;

/**
 * @brief          SUMat usage flags specify the usage of a specific SUMat.
 */
enum UMatUsageFlags
{
  USAGE_DEFAULT                = 0,
  USAGE_ALLOCATE_HOST_MEMORY   = 1 << 0,
  USAGE_ALLOCATE_DEVICE_MEMORY = 1 << 1,
  USAGE_ALLOCATE_SHARED_MEMORY = 1 << 2,
  USAGE_DONT_ALLOCATE          = 1 << 3,

  USAGE_DDR0               = 1 << 4,
  USAGE_DDR1               = 1 << 5,
  USAGE_SSRAM              = 1 << 6,
  USAGE_MSRAM              = 1 << 7,
  __UMAT_USAGE_FLAGS_32BIT = 0x7fffffff // Binary compatibility hint
};

/**
 * @brief          Access flags for the mapping
 */
enum UMatAccessFlags
{
  ACCESS_READ  = 1 << 24,
  ACCESS_WRITE = 1 << 25,
  ACCESS_RW    = 3 << 24,
  ACCESS_MASK  = ACCESS_RW,
  ACCESS_FAST  = 1 << 26
};

/**
 * @brief          SUMat Auto lock - obtains a loc on constructor, releases it
 *                 when dies
 */
struct UMatDataAutoLock
{
  /**
   * @brief          Function tries to obtain the lock for UMatData
   *
   * @param[in]      apU           UMatData object which is supposed to be
   * locked
   */
  explicit UMatDataAutoLock(vsdk::UMatData* const apU);

  /**
   * @brief          Function releases the lock for assigned UMatData
   */
  ~UMatDataAutoLock();

  /**
   * @brief          Associated UMatData object
   */
  UMatData* u;
};

/**
 * It's derived from OpenCV's SUMatAllocator class - manages all the allocations
 * inside the SUMat. It's called internally and can be also inherited further
 * in order to implement specific functionality.
 */
class SUMatAllocator
{
public:
  /**
   * @brief          Constructor, creates allocator...
   */
  SUMatAllocator();

  /**
   * @brief          Descructor, destroys allocator...
   *
   */
  ~SUMatAllocator();

  /**
   * @brief          Function is called with specific information and should
   *                 allocate the internal buffer for a new SUMat
   *
   * @param[in]      aDims            Number of dimensions
   * @param[in]      apSizes           Size for each dimension
   * @param[in]      aType            Data type
   * @param[in]      apData            Existing pointer to data, if available
   * @param[in]      apStep            Step for each dimension
   * @param[in]      aFlags           Access and other defined flags
   * @param[in]      aUsageFlags      Usage flags
   *
   * @return         Allocated UMatData structure
   */
  vsdk::UMatData* allocate(const int32_t              aDims,
                           const int32_t* const       apSizes,
                           const int32_t              aType,
                           void* const                apData,
                           uint64_t* const            apStep,
                           const int32_t              aFlags,
                           const vsdk::UMatUsageFlags aUsageFlags) const;

  /**
   * @brief          The function returns the info when the allocation will
   *                 be performed for umat
   *
   * @param[in]      apData           Existing pointer to data, if available
   * @param[in]      aAccessFlags     Access and other defined flags
   * @param[in]      aUsageFlags      Usage flags
   *
   * @return
   */
  bool allocate(const vsdk::UMatData* const apData,
                const int32_t               aAccessFlags,
                const vsdk::UMatUsageFlags  aUsageFlags) const;

  /**
   * @brief          Function frees the UMatData contents (via OAL)
   *
   * @param[in]      apData           Existing pointer to data, if available
   *
   */
  void deallocate(vsdk::UMatData* const apData) const;

  /**
   * @brief          Function maps the UMatData to the virtual space with
   *                 specific access flags
   *
   * @param[in]      apData           Existing pointer to data, if available
   * @param[in]      aAccessFlags     Access and other defined flags
   */
  void map(vsdk::UMatData* const apData, uint32_t aAccessFlags) const;

  /**
   * @brief          Function unmaps the existing virtual space mapping
   *
   * @param[in]      apData           Existing pointer to data, if available
   */
  void unmap(vsdk::UMatData* apData) const;

  /**
   * @brief          Function downloads the matrix contents into dst pointer
   *
   * @param[in]      apData           UMatData structure of the matrix
   * @param[in]      aDst             Destination pointer
   * @param[in]      aDims            Number of dimensions
   * @param[in]      aSz              Sizes for each dimension
   * @param[in]      aSrcOfs          Source offset for each dimenstion
   * @param[in]      aSrcStep         Source step for each dimension
   * @param[in]      aDstStep         Destination step
   *
   */
  void download(const vsdk::UMatData* const apData,
                void* const                 aDst,
                const int32_t               aDims,
                const uint64_t* const       aSz,
                const uint64_t* const       aSrcOfs,
                const uint64_t* const       aSrcStep,
                const uint64_t* const       aDstStep) const;

  /**
   * @brief          Function uploads the matrix contents from src pointer
   *
   * @param[in]      apData           UMatData structure of the matrix
   * @param[in]      aSrc             Source pointer
   * @param[in]      aDims            Number of dimensions
   * @param[in]      aSz              Sizes for each dimension
   * @param[in]      aDstOfs          Destination offset for each dimenstion
   * @param[in]      aDstStep         Destination step for each dimension
   * @param[in]      aSrcStep         Source step
   *
   */
  void upload(const vsdk::UMatData* const apData,
              void* const                 aSrc,
              const int32_t               aDims,
              const uint64_t* const       aSz,
              const uint64_t* const       aDstOfs,
              const uint64_t* const       aDstStep,
              const uint64_t* const       aSrcStep) const;

  /**
   * @brief          Copies the matrix contents to another SUMat
   *
   * @param[in]      apSrcData        Source UMatData structure of the matrix
   * @param[in]      apDstData        Destination UMatData structure of the
   *                                  matrix
   * @param[in]      aDims            Number of dimensions
   * @param[in]      aSz              Sizes for each dimension
   * @param[in]      aSrcOfs          Source offset for each dimenstion
   * @param[in]      aSrcStep         Source step for each dimension
   * @param[in]      aDstOfs          Destination offset for each dimenstion
   * @param[in]      aDstStep         Destination step
   * @param[in]      aSync            Not used now
   *
   */
  void copy(const vsdk::UMatData* const apSrcData,
            const vsdk::UMatData* const apDstData,
            const int32_t               aDims,
            const uint64_t* const       aSz,
            const uint64_t* const       aSrcOfs,
            const uint64_t* const       aSrcStep,
            const uint64_t* const       aDstOfs,
            const uint64_t* const       aDstStep,
            const int8_t                aSync) const;

private:
  /**
   * Allocator copy function (helper)
   * Copies the matrix contents to another SUMat
   */
  void doCopy(const vsdk::SMat&           aSrc,
              const vsdk::SMat&           aDst,
              const vsdk::UMatData* const apSrcData,
              const vsdk::UMatData* const apDstData) const;
  /**
   * Allocator upload function (helper)
   */
  void doUpload(const vsdk::SMat& aSrc, const vsdk::SMat& aDst, const vsdk::UMatData* const apData) const;
  /**
   * Allocator download function (helper)
   */
  void doDownload(const vsdk::SMat& aSrc, const vsdk::SMat& aDst, const vsdk::UMatData* const apData) const;

  /*
 * Function unmaps the existing virtual space mapping (helper)
 */
  void doUnmap(vsdk::UMatData* const apData) const;
}; /* class SUMatAllocator */
} /* namespace vsdk */

#endif /* UMATALLOCATOR_HPP */
