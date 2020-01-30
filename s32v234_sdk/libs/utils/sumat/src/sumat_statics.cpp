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

const uint32_t VSDK_CV_USE_DEBUG_OUT    = 0U;
const uint32_t OAL_USAGE_CACHED         = 0x10000000U;
const uint32_t OAL_USAGE_NONCACHED      = 0x20000000U;
const uint32_t OAL_USAGE_FORCE_NOTFLUSH = 0x40000000U;
const uint32_t VSDK_CV_AUTOSTEP         = 0x7FFFFFFFU;
const uint32_t VSDK_CV_MAT_CONT_FLAG    = 16384U;
const uint32_t VSDK_CV_SUBMAT_FLAG      = 32768U;
const uint32_t VSDK_CV_MAX_DIM          = 32U;
const uint32_t VSDK_CV_CN_MAX           = 512U;
const uint32_t VSDK_CV_CN_SHIFT         = 3U;
const uint32_t VSDK_CV_MAT_CN_MASK      = 4088U;
const uint32_t VSDK_CV_MAT_TYPE_MASK    = 4095U;


#if defined _MSC_VER && !defined RC_INVOKED
#include <intrin.h>
int32_t VSDK_CV_XADD(int32_t* const aAddr, const int32_t aDelta)
{
  return (int32_t)_InterlockedExchangeAdd((long volatile*)aAddr, aDelta);
}

void VSDK_CV_XADD_VOID(int32_t* const aAddr, const int32_t aDelta)
{
   (int32_t)_InterlockedExchangeAdd((long volatile*)aAddr, aDelta);
}

#elif defined __INTEGRITY__

/*
 * Atomic fetch and accumulate with result return type
 */
int32_t VSDK_CV_XADD(int32_t* const aAddr, const int32_t aDelta)
{
  int32_t tmp = *aAddr;
  *(uint32_t*)aAddr += aDelta;
  return tmp;
}

/*
 * Atomic fetch and accumulate with void return type
 */
void VSDK_CV_XADD_VOID(int32_t* const aAddr, const int32_t aDelta)
{
  *(uint32_t*)aAddr += aDelta;
}

#else

/*
 * Atomic fetch and accumulate with result return type
 */
int32_t VSDK_CV_XADD(int32_t* const aAddr, const int32_t aDelta)
{
  return (int32_t)__sync_fetch_and_add((uint32_t*)(aAddr), static_cast< int32_t >(aDelta));
}

/*
 * Atomic fetch and accumulate with void return type
 */
void VSDK_CV_XADD_VOID(int32_t* const aAddr, const int32_t aDelta)
{
  __sync_fetch_and_add((uint32_t*)(aAddr), static_cast< int32_t >(aDelta));
}
#endif

/*
 * Function returning matrix depth based on its flags
 */
uint32_t VSDK_CV_MAT_DEPTH(const uint32_t aFlags)
{
  return aFlags & 0x7U;
}

/*
 * Function returning number of channels based on its flags
 */
uint32_t VSDK_CV_MAT_CN(const uint32_t aFlags)
{
  return (((aFlags)&VSDK_CV_MAT_CN_MASK) >> VSDK_CV_CN_SHIFT) + 1U;
}

/*
 * Function returning element size based on data type
 */
uint32_t VSDK_CV_ELEM_SIZE(const uint32_t aType)
{
  return VSDK_CV_MAT_CN(aType) << (((((sizeof(size_t) / 4U + 1U) * 16384U) | 0x3a50U) >> (((aType)&0x7U) * (2U))) & 3U);
}

/*
 * Function returning element size based on data type
 */
uint32_t VSDK_CV_ELEM_SIZE1(const uint32_t aType)
{
  return static_cast< uint32_t >((((sizeof(size_t) << 28U) | 0x8442211U) >> ((((aType)&0x7U)) * 4U)) & 15U);
}

/*
 * Function returning CV data type from mat flags
 */
uint32_t VSDK_CV_MAT_TYPE(const uint32_t aFlags)
{
  return (aFlags) & (VSDK_CV_MAT_TYPE_MASK);
}

/*
 * Function returning CV data type based on dept and num of channels
 */
uint32_t VSDK_CV_MAKETYPE(const uint32_t aDepth, const uint32_t aCn)
{
  return ((aDepth)&0x7U) + (((aCn)-1U) << VSDK_CV_CN_SHIFT);
}

/*
 * Function returning CV data type equal to 8 uint with aN channels
 */
uint32_t VSDK_CV_8UC(const uint32_t aN)
{
  return 0U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 8 int with aN channels
 */
uint32_t VSDK_CV_8SC(const uint32_t aN)
{
  return 1U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 16 uint with aN channels
 */
uint32_t VSDK_CV_16UC(const uint32_t aN)
{
  return 2U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 16 int with aN channels
 */
uint32_t VSDK_CV_16SC(const uint32_t aN)
{
  return 3U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 32 int with aN channels
 */
uint32_t VSDK_CV_32SC(const uint32_t aN)
{
  return 4U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 32 FP with aN channels
 */
uint32_t VSDK_CV_32FC(const uint32_t aN)
{
  return 5U + (((aN)-1U) << 3U);
}

/*
 * Function returning CV data type equal to 64 FP with aN channels
 */
uint32_t VSDK_CV_64FC(const uint32_t aN)
{
  return 6U + (((aN)-1U) << 3U);
}
