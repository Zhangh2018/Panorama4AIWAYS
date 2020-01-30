/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP
* All Rights Reserved
*
******************************************************************************
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
****************************************************************************/

#ifndef REMAPTRANSFORMS_H
#define REMAPTRANSFORMS_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <icp_data.h>
#include "cimg_refdewarp.h"

int remap_bilin_rgb_execute_ref(
  uint8_t* srcImagePtr, uint16_t srcImageWidth, uint16_t srcImageHeight, uint16_t srcStride,
  uint8_t* dstImagePtr, uint16_t dstImageWidth, uint16_t dstImageHeight, uint16_t dstStride,
  int32_t dstBlockWidth, int32_t dstBlockHeight,
  uint16_t deltaWidth, uint16_t deltaHeight,
  float* floatMap);

int remap_bilin_grayscale_execute_ref(
  uint8_t* srcImagePtr, uint16_t srcImageWidth, uint16_t srcImageHeight, uint16_t srcStride,
  uint8_t* dstImagePtr, uint16_t dstImageWidth, uint16_t dstImageHeight, uint16_t dstStride,
  int32_t dstBlockWidth, int32_t dstBlockHeight,
  uint16_t deltaWidth, uint16_t deltaHeight,
  float* floatMap);

int remap_bilin_RGB_reference(
  vsdk::SMat   lSrcImage,
  vsdk::SMat   lDestImage,
  float*               floatMap,
  int32_t              dstBlockWidth,
  int32_t              dstBlockHeight);

int remap_bilin_grayscale_reference(
  vsdk::SMat   lSrcImage,
  vsdk::SMat   lDestImage,
  float*               floatMap,
  int32_t              dstBlockWidth,
  int32_t              dstBlockHeight);

int32_t floatMapTo1DOffset_ref(
  const float *floatMap, // stride assumed to be width?
  const int32_t imageWidth,
  const int32_t imageHeight,
  uint32_t* oneDimOffset,
  uint8_t* delta,
  const int32_t srcWidth,
  const int32_t srcHeight,
  const int32_t srcStride);

int32_t getSourceBlockSize_ref(
  uint32_t* oneDimOffset,
  uint32_t dstImageWidth,
  uint32_t dstImageHeight,
  const int32_t dstBW,
  const int32_t dstBH,
  const int32_t srcStride,
  int32_t &srcBW,
  int32_t &srcBH);

int32_t getSourceBlockOffsets_ref(
  uint32_t* oneDimOffset,
  uint32_t dstImageWidth,
  uint32_t dstImageHeight,
  const int32_t dstBW,
  const int32_t dstBH,
  const int32_t srcStride,
  uint32_t* blockOffset);

int32_t OneDOffsetToLocalOffset_ref(
  uint32_t* oneDimOffset,
  int dstImageWidth,
  int dstImageHeight,
  uint32_t* blockOffset,
  int dstBlockWidth,
  int dstBlockHeight,
  int srcBlockWidth,
  int srcStride,
  uint16_t* localOffset);

#endif /* REMAPTRANSFORMS_H */
