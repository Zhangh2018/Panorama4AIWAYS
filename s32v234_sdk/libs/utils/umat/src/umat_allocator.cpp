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

#include "umat_mat.hpp"

#include <iostream>
#include "umat_allocator.hpp"

static vsdk::SUMatAllocator gAllocator;

/*******************************************************************************
 * Constructor...
 ******************************************************************************/
vsdk::UMatAllocator::UMatAllocator() : vsdk::SUMatAllocator(), cv::MatAllocator()
{
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
cv::UMatData* vsdk::UMatAllocator::allocate(int                aDims,
                                            const int*         apSizes,
                                            int                aType,
                                            void*              apData,
                                            size_t*            apStep,
                                            int                aFlags,
                                            cv::UMatUsageFlags aUsageFlags) const
{
  printf("AAAAA\n");
  return (cv::UMatData*)gAllocator.allocate((int32_t)aDims, (const int32_t*)apSizes, (int32_t)aType, apData,
                                            (uint64_t*)apStep, (int32_t)aFlags, (vsdk::UMatUsageFlags)aUsageFlags);
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
bool vsdk::UMatAllocator::allocate(cv::UMatData* apData, int32_t aAccessFlags, cv::UMatUsageFlags aUsageFlags) const
{
  return gAllocator.allocate((vsdk::UMatData*)apData, aAccessFlags, (vsdk::UMatUsageFlags)aUsageFlags);
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
void vsdk::UMatAllocator::deallocate(cv::UMatData* apData) const
{
  gAllocator.deallocate((vsdk::UMatData*)apData);
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
void vsdk::UMatAllocator::map(cv::UMatData* apData, int32_t aAccessFlags) const
{
  gAllocator.map((vsdk::UMatData*)apData, aAccessFlags);
}

/*******************************************************************************
 * OpenCV compatibility overload
 ******************************************************************************/
void vsdk::UMatAllocator::unmap(cv::UMatData* apData) const
{
  gAllocator.unmap((vsdk::UMatData*)apData);
}

/*******************************************************************************
 * Allocator download function
 ******************************************************************************/
void vsdk::UMatAllocator::download(cv::UMatData* apData,
                                   void*         apDst,
                                   int           aDims,
                                   const size_t  aSz[],
                                   const size_t  aSrcOfs[],
                                   const size_t  aSrcStep[],
                                   const size_t  aDstStep[]) const
{
#if defined(APEX2_EMULATE)
  uint64_t* lpSz64 = new uint64_t[aDims];
  uint64_t* lpSrcOfs64      = new uint64_t[aDims];
  uint64_t* lpSrcStep64     = new uint64_t[aDims];
  uint64_t* lpDstStep64     = new uint64_t[aDims];
  for(int lIt = 0; lIt < aDims; ++lIt)
  {
    lpSz64[lIt]      = (uint32_t)aSz[lIt];
    lpSrcOfs64[lIt]  = (uint32_t)aSrcOfs[lIt];
    lpSrcStep64[lIt] = (uint32_t)aSrcStep[lIt];
    lpDstStep64[lIt] = (uint32_t)aDstStep[lIt];
  }

  gAllocator.download((vsdk::UMatData*)apData, apDst, aDims, lpSz64, lpSrcOfs64, lpSrcStep64, lpDstStep64);

  delete[] lpSz64;
  delete[] lpSrcOfs64;
  delete[] lpSrcStep64;
  delete[] lpDstStep64;
#else
  gAllocator.download((vsdk::UMatData*)apData, apDst, aDims, aSz, aSrcOfs, aSrcStep, aDstStep);
#endif
}

/*******************************************************************************
 * Allocator upload function
 ******************************************************************************/
void vsdk::UMatAllocator::upload(cv::UMatData* apData,
                                 const void*   apSrc,
                                 int           aDims,
                                 const size_t  aSz[],
                                 const size_t  aDstOfs[],
                                 const size_t  aDstStep[],
                                 const size_t  aSrcStep[]) const
{
#if defined(APEX2_EMULATE)
  uint64_t* lpSz64      = new uint64_t[aDims];
  uint64_t* lpDstOfs64  = new uint64_t[aDims];
  uint64_t* lpSrcStep64 = new uint64_t[aDims];
  uint64_t* lpDstStep64 = new uint64_t[aDims];
  for(int lIt = 0; lIt < aDims; ++lIt)
  {
    lpSz64[lIt]      = (uint32_t)aSz[lIt];
    lpDstOfs64[lIt]  = (uint32_t)aDstOfs[lIt];
    lpSrcStep64[lIt] = (uint32_t)aSrcStep[lIt];
    lpDstStep64[lIt] = (uint32_t)aDstStep[lIt];
  }

  gAllocator.upload((vsdk::UMatData*)apData, (void*)apSrc, aDims, lpSz64, lpDstOfs64, lpDstStep64, lpSrcStep64);

  delete[] lpSz64;
  delete[] lpDstOfs64;
  delete[] lpSrcStep64;
  delete[] lpDstStep64;
#else
  gAllocator.upload((vsdk::UMatData*)apData, (void*)apSrc, aDims, aSz, aDstOfs, aDstStep, aSrcStep);
#endif
}

/*******************************************************************************
 * Allocator copy function
 ******************************************************************************/
void vsdk::UMatAllocator::copy(cv::UMatData* apSrcData,
                               UMatData*     apDstData,
                               int           aDims,
                               const size_t  aSz[],
                               const size_t  aSrcOfs[],
                               const size_t  aSrcStep[],
                               const size_t  aDstOfs[],
                               const size_t  aDstStep[],
                               bool          aSync) const
{
#if defined(APEX2_EMULATE)
  uint64_t* lpSz64      = new uint64_t[aDims];
  uint64_t* lpSrcOfs64  = new uint64_t[aDims];
  uint64_t* lpDstOfs64  = new uint64_t[aDims];
  uint64_t* lpSrcStep64 = new uint64_t[aDims];
  uint64_t* lpDstStep64 = new uint64_t[aDims];
  for(int lIt = 0; lIt < aDims; ++lIt)
  {
    lpSz64[lIt]      = (uint32_t)aSz[lIt];
    lpSrcOfs64[lIt]  = (uint32_t)aSrcOfs[lIt];
    lpDstOfs64[lIt]  = (uint32_t)aDstOfs[lIt];
    lpSrcStep64[lIt] = (uint32_t)aSrcStep[lIt];
    lpDstStep64[lIt] = (uint32_t)aDstStep[lIt];
  }

  gAllocator.copy((vsdk::UMatData*)apSrcData, (vsdk::UMatData*)apDstData, aDims, lpSz64, lpSrcOfs64, lpSrcStep64,
                  lpDstOfs64, lpDstStep64, aSync);

  delete[] lpSz64;
  delete[] lpSrcOfs64;
  delete[] lpDstOfs64;
  delete[] lpSrcStep64;
  delete[] lpDstStep64;
#else
  gAllocator.copy((vsdk::UMatData*)apSrcData, (vsdk::UMatData*)apDstData, aDims, aSz, aSrcOfs, aSrcStep, aDstOfs,
                  aDstStep, aSync);
#endif
}
