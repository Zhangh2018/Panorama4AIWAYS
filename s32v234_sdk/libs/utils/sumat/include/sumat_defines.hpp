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
#ifndef SUMATDEFINES_HPP
#define SUMATDEFINES_HPP

#include <cstdint>
#include <cstring>

extern const uint32_t VSDK_CV_USE_DEBUG_OUT;
extern const uint32_t OAL_USAGE_CACHED;
extern const uint32_t OAL_USAGE_NONCACHED;
extern const uint32_t OAL_USAGE_FORCE_NOTFLUSH;

extern const uint32_t VSDK_CV_MAT_CONT_FLAG;
extern const uint32_t VSDK_CV_SUBMAT_FLAG;
extern const uint32_t VSDK_CV_AUTOSTEP;

extern const uint32_t VSDK_CV_MAX_DIM;
extern const uint32_t VSDK_CV_CN_MAX;
extern const uint32_t VSDK_CV_CN_SHIFT;
extern const uint32_t VSDK_CV_MAT_CN_MASK;
extern const uint32_t VSDK_CV_MAT_TYPE_MASK;

static const uint32_t VSDK_CV_8U               = 0U;
static const uint32_t VSDK_CV_8S               = 1U;
static const uint32_t VSDK_CV_16U              = 2U;
static const uint32_t VSDK_CV_16S              = 3U;
static const uint32_t VSDK_CV_32S              = 4U;
static const uint32_t VSDK_CV_32F              = 5U;
static const uint32_t VSDK_CV_64F              = 6U;
static const uint32_t VSDK_CV_USRTYPE1         = 7U;
static const uint32_t VSDK_CV_8UC1             = 0U;
static const uint32_t VSDK_CV_8UC2             = 8U;
static const uint32_t VSDK_CV_8UC3             = 16U;
static const uint32_t VSDK_CV_8UC4             = 24U;
static const uint32_t VSDK_CV_8SC1             = 1U;
static const uint32_t VSDK_CV_8SC2             = 9U;
static const uint32_t VSDK_CV_8SC3             = 17U;
static const uint32_t VSDK_CV_8SC4             = 25U;
static const uint32_t VSDK_CV_16UC1            = 2U;
static const uint32_t VSDK_CV_16UC2            = 10U;
static const uint32_t VSDK_CV_16UC3            = 18U;
static const uint32_t VSDK_CV_16UC4            = 26U;
static const uint32_t VSDK_CV_16SC1            = 3U;
static const uint32_t VSDK_CV_16SC2            = 11U;
static const uint32_t VSDK_CV_16SC3            = 19U;
static const uint32_t VSDK_CV_16SC4            = 27U;
static const uint32_t VSDK_CV_32SC1            = 4U;
static const uint32_t VSDK_CV_32SC2            = 12U;
static const uint32_t VSDK_CV_32SC3            = 20U;
static const uint32_t VSDK_CV_32SC4            = 28U;
static const uint32_t VSDK_CV_32FC1            = 5U;
static const uint32_t VSDK_CV_32FC2            = 13U;
static const uint32_t VSDK_CV_32FC3            = 21U;
static const uint32_t VSDK_CV_32FC4            = 29U;
static const uint32_t VSDK_CV_64FC1            = 6U;
static const uint32_t VSDK_CV_64FC2            = 14U;
static const uint32_t VSDK_CV_64FC3            = 22U;
static const uint32_t VSDK_CV_64FC4            = 30U;

int32_t  VSDK_CV_XADD(int32_t* const aAddr, const int32_t aDelta);
void     VSDK_CV_XADD_VOID(int32_t* const aAddr, const int32_t aDelta);
uint32_t VSDK_CV_MAT_DEPTH(const uint32_t aFlags);
uint32_t VSDK_CV_MAT_CN(const uint32_t aFlags);
uint32_t VSDK_CV_ELEM_SIZE(const uint32_t aType);
uint32_t VSDK_CV_ELEM_SIZE1(const uint32_t aType);
uint32_t VSDK_CV_MAT_TYPE(const uint32_t aFlags);
uint32_t VSDK_CV_MAKETYPE(const uint32_t aDepth, const uint32_t aCn);
uint32_t VSDK_CV_8UC(const uint32_t aN);
uint32_t VSDK_CV_8SC(const uint32_t aN);
uint32_t VSDK_CV_16UC(const uint32_t aN);
uint32_t VSDK_CV_16SC(const uint32_t aN);
uint32_t VSDK_CV_32SC(const uint32_t aN);
uint32_t VSDK_CV_32FC(const uint32_t aN);
uint32_t VSDK_CV_64FC(const uint32_t aN);

#endif /* UMATDEFINES_HPP */
