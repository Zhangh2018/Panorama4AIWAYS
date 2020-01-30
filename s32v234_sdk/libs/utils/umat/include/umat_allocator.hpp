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

#ifndef UMATALLOCATOR_HPP
#define UMATALLOCATOR_HPP

#include "sumat_defines.hpp"
#include "sumat_allocator.hpp"

#include <opencv2/opencv.hpp>

#include <cstdint>

namespace vsdk
{
class UMatAllocator : public vsdk::SUMatAllocator, public cv::MatAllocator
{
public:
  UMatAllocator();

  cv::UMatData* allocate(int                aDims,
                         const int*         apSizes,
                         int                aType,
                         void*              apData,
                         size_t*            apStep,
                         int                aFlags,
                         cv::UMatUsageFlags aUsageFlags) const;
  bool          allocate(cv::UMatData* apData, int32_t aAccessFlags, cv::UMatUsageFlags aUsageFlags) const;
  void          deallocate(cv::UMatData* apData) const;
  void          map(cv::UMatData* apData, int32_t aAccessFlags) const;
  void          unmap(cv::UMatData* apData) const;
  void          download(cv::UMatData* apData,
                         void*         apDst,
                         int           aDims,
                         const size_t  aSz[],
                         const size_t  aSrcOfs[],
                         const size_t  aSrcStep[],
                         const size_t  aDstStep[]) const;
  void          upload(cv::UMatData* apData,
                       const void*   apSrc,
                       int           aDims,
                       const size_t  aSz[],
                       const size_t  aDstOfs[],
                       const size_t  aDstStep[],
                       const size_t  aSrcStep[]) const;
  void          copy(cv::UMatData* apSrcData,
                     UMatData*     apDstData,
                     int           aDims,
                     const size_t  aSz[],
                     const size_t  aSrcOfs[],
                     const size_t  aSrcStep[],
                     const size_t  aDstOfs[],
                     const size_t  aDstStep[],
                     bool          aSync) const;

}; /* class UMatAllocator */
} /* namespace vsdk */

#endif /* UMATALLOCATOR_HPP */
