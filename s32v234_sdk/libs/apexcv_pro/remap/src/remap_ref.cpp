/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2017 NXP
* All Rights Reserved
*
*****************************************************************************
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

// Interface to CIMG_RefDewarp_c.cpp, which contains reference implementations for remap.

// for LIB_RESULT
#include "apexcv_pro_remap.h"
#include "oal.h"

#include "cimg_refdewarp.h"
#include "remap_transforms.h"

// For now, this value is only used for reference code!
#ifndef DEFAULT_SLICE_HEIGHT
#define DEFAULT_SLICE_HEIGHT 4
#endif

int remap_bilin_RGB_reference(
  vsdk::SMat   lSrcImage,
  vsdk::SMat   lDestImage,
  float*               floatMap,
  int32_t               dstBlockWidth,
  int32_t               dstBlockHeight)
{
  int32_t rval = 0;

  uint8_t* srcImagePtr = (uint8_t*)lSrcImage.data;
  uint16_t srcImageWidth = lSrcImage.cols;
  uint16_t srcImageHeight = lSrcImage.rows;
  uint16_t srcStride = srcImageWidth;

  uint8_t* dstImagePtr = (uint8_t*)lDestImage.data;
  uint16_t dstImageWidth = lDestImage.cols;
  uint16_t dstImageHeight = lDestImage.rows;
  uint16_t dstStride = dstImageWidth;

  uint16_t deltaWidth = lDestImage.cols * 2; //2 delta values (x,y) per destination value
  uint16_t deltaHeight = lDestImage.rows;

  rval |= remap_bilin_rgb_execute_ref(
    srcImagePtr, srcImageWidth, srcImageHeight, srcStride,
    dstImagePtr, dstImageWidth, dstImageHeight, dstStride,
    dstBlockWidth, dstBlockHeight,
    deltaWidth, deltaHeight,
    floatMap);

  return rval;

}

int remap_bilin_grayscale_reference(
  vsdk::SMat   lSrcImage,
  vsdk::SMat   lDestImage,
  float*               floatMap,
  int32_t              dstBlockWidth,
  int32_t              dstBlockHeight)
{
  int32_t rval;

  uint8_t* srcImagePtr = (uint8_t*)lSrcImage.data;
  uint16_t srcImageWidth = lSrcImage.cols;
  uint16_t srcImageHeight = lSrcImage.rows;
  uint16_t srcStride = lSrcImage.step[0];

  uint8_t* dstImagePtr = (uint8_t*)lDestImage.data;
  uint16_t dstImageWidth = lDestImage.cols;
  uint16_t dstImageHeight = lDestImage.rows;
  uint16_t dstStride = lDestImage.step[0];

  uint16_t deltaWidth = lDestImage.cols * 2; //2 delta values (x,y) per destination value
  uint16_t deltaHeight = lDestImage.rows;

  rval = remap_bilin_grayscale_execute_ref(
    srcImagePtr, srcImageWidth, srcImageHeight, srcStride,
    dstImagePtr, dstImageWidth, dstImageHeight, dstStride,
    dstBlockWidth, dstBlockHeight,
    deltaWidth, deltaHeight,
    floatMap);


  return rval;
}

int remap_bilin_rgb_execute_ref(
  uint8_t* srcImagePtr, uint16_t srcImageWidth, uint16_t srcImageHeight, uint16_t srcStride,
  uint8_t* dstImagePtr, uint16_t dstImageWidth, uint16_t dstImageHeight, uint16_t dstStride,
  int32_t /*dstBlockWidth*/, int32_t dstBlockHeight,
  uint16_t deltaWidth, uint16_t deltaHeight,
  float* floatMap)
{
  int32_t rval = 0;

  // Allocate memory for offset and delta buffers
  uint32_t* oneDimOffset = new uint32_t[dstImageWidth * dstImageHeight];
  uint8_t*  delta = new uint8_t[deltaWidth * deltaHeight];

  rval |= floatMapTo1DOffset_ref(
    floatMap,
    dstImageWidth,
    dstImageHeight,
    oneDimOffset,
    delta,
    srcImageWidth,
    srcImageHeight,
    srcStride);

  rval |= simulateRGBDewarp(
    dstImagePtr,
    srcImagePtr,
    oneDimOffset,
    delta,
    apexcv::Remap::INTER_LINEAR,
    srcStride, srcImageHeight,
    dstImageWidth, dstImageHeight,
    dstStride,
    dstBlockHeight);

  delete[] oneDimOffset;
  delete[] delta;

  return rval;
}

int remap_bilin_grayscale_execute_ref(
  uint8_t* srcImagePtr, uint16_t srcImageWidth, uint16_t srcImageHeight, uint16_t srcStride,
  uint8_t* dstImagePtr, uint16_t dstImageWidth, uint16_t dstImageHeight, uint16_t dstStride,
  int32_t /*dstBlockWidth*/, int32_t dstBlockHeight,
  uint16_t deltaWidth, uint16_t deltaHeight,
  float* floatMap)
{
  int32_t rval = 0;

  // Allocate memory for offset and delta buffers
  uint32_t* oneDimOffset = new uint32_t[dstImageWidth * dstImageHeight];
  uint8_t*  delta = new uint8_t[deltaWidth * deltaHeight];

  rval |= floatMapTo1DOffset_ref(
    floatMap,
    dstImageWidth,
    dstImageHeight,
    oneDimOffset,
    delta,
    srcImageWidth,
    srcImageHeight,
    srcStride);

  simulateGrayDewarp(
    dstImagePtr,
    srcImagePtr,
    oneDimOffset,
    delta,
    apexcv::Remap::INTER_LINEAR,
    srcStride, srcImageHeight,
    dstImageWidth, dstImageHeight,
    dstStride,
    dstBlockHeight);

  delete[] oneDimOffset;
  delete[] delta;

  return rval;
}
