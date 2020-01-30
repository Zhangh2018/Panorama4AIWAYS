/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2018 NXP
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

// for LIB_RESULT
#include "global_errors.h"
#include "oal.h"

#include "cimg_refdewarp.h"
#include "remap_transforms.h"
#include "apexcv_pro_remap.h"
#include "algorithm"
#include <t32term.h>
#include "icp_fixed.hpp"
#include "icp_remap_protected.hpp"
#include "mapper_type_defines.h"
#include "param_loader.h"

// GENERATED ACF PROCESS
#include "REMAP_BLOCK_SIZE.hpp"
#include "REMAP_GRAY.hpp"

//y and x offset divider for Fixed_offset;
static const int32_t offsetShift = 16;
static const int32_t offsetShiftValue = 0x0000ffff;
static const uint8_t chunkSizeCandidate[32] = { 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36, 40, 44, 48, 52, 56, 60, 64, 72, 80, 88, 96, 104, 112, 120, 128 };

using namespace apexcv;

Remap::Remap()
  :
  mTopViewPixelToMeterRatioX(0.0),
  mTopViewPixelToMeterRatioY(0.0),
  mSrcWidth(0),
  mSrcHeight(0),
  mSrcStride(0),
  mDstWidth(0),
  mDstHeight(0),
  mpRemapBlockProc(nullptr),
  mpRemapProc(nullptr),
  mInitState(0)

{
  mpParamLoader = new ParamLoader;
  mpGroundToDewarp = new Homography;
  mpGroundToWarp = new PerspectiveMapper;
}

Remap::~Remap()
{
  Release();
}

void Remap::Release()
{
  if (nullptr != mpRemapBlockProc)
  {
    delete (REMAP_BLOCK_SIZE*)mpRemapBlockProc;
    mpRemapBlockProc = nullptr;
  }
  if (nullptr != mpRemapProc)
  {
    delete (REMAP_GRAY*)mpRemapProc;
    mpRemapProc = nullptr;
  }
  if (nullptr != mpParamLoader)
  {
    delete (ParamLoader*)mpParamLoader;
    mpParamLoader = nullptr;
  }
  if (nullptr != mpGroundToDewarp)
  {
    delete (Homography*)mpGroundToDewarp;
    mpGroundToDewarp = nullptr;
  }
  if (nullptr != mpGroundToWarp)
  {
    delete (PerspectiveMapper*)mpGroundToWarp;
    mpGroundToWarp = nullptr;
  }
  mInitState = 0;
  mSrcWidth = 0;
  mSrcHeight = 0;
  mSrcStride = 0;
  mDstWidth = 0;
  mDstHeight = 0;
  return;
}

int32_t Remap::FloatToFixedOffsets(const float *apcMapRGB, vsdk::SUMat &aFixedOffset, const int32_t acISrcWidth, const int32_t acISrcHeight)
{
  vsdk::SMat aFixedOffsetMat = aFixedOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat deltaMat = mDelta.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

  // NB: dest, map, 1DOffset and delta all have same width/height
  int32_t imageHeight = aFixedOffset.rows;
  int32_t imageWidth = aFixedOffset.cols;
  uint32_t *offsetPtr = (uint32_t*)aFixedOffsetMat.data;
  uint8_t *deltaPtr = (uint8_t*)deltaMat.data;

  // ---BUILD offset and delta images---
  // NB: offset is ACF1-style offset, not local offset.  /TODO revisit this
  // Based on 8-bit delta encoding.  Unused for Nearest Neighbour
  const float scl = 256.0; // used to compute 8-bit delta value
  const float rnd = (0.5 / scl);
  for (int y = 0; y < imageHeight; ++y) {
    for (int x = 0; x < imageWidth; ++x) {
      // If fractional part is > delta value 0xff, round up offset count
      float sx = *apcMapRGB++ + rnd;
      float sy = *apcMapRGB++ + rnd;

      // Padding rules...
      if (sx < 0.0)
        sx = 0.0;
      if (sx > (acISrcWidth - 1))
        sx = (acISrcWidth - 1);
      if (sy < 0.0)
        sy = 0.0;
      // source image height is not known
      if (sy > (acISrcHeight - 1))
        sy = (acISrcHeight - 1);

      // bilinear interpolation
      int32_t xi = (int32_t)floor(sx);
      int32_t xf = (int32_t)floor(scl * (sx - (float)xi));
      int32_t yi = (int32_t)floor(sy);
      int32_t yf = (int32_t)floor(scl * (sy - (float)yi));

      const uint32_t offsetVal = (yi << offsetShift) + xi;
      *offsetPtr++ = offsetVal;
      // Resize delta values
      *deltaPtr++ = xf;// x first
      *deltaPtr++ = yf;// y second
    }
  }

  return 0;
}

int32_t Remap::InitMoOffsets(vsdk::SUMat &aFixedOffset, const int32_t acSrcStride)
{
  unsigned int rVal = 0;

  // Offset of the top left corner of the block
  uint32_t offsetTL = 0;

  // RetIcpDataTypeSizeInBytes(lSrc.GetContigDataDesc(0).GetElementDataType())
  // TODO fix this
  //unsigned int sourceDataSize = 4;
  vsdk::SMat moOffsetMat = moOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat fixedOffsetMat = aFixedOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint32_t* pSourceOffsetList = (uint32_t*)moOffsetMat.data;
  uint32_t* pOffsetMap = (uint32_t*)fixedOffsetMat.data;
  // Offset and Destination images are the same size
  uint32_t destImageWidth = aFixedOffset.cols;
  uint32_t destImageHeight = aFixedOffset.rows;

  // used to parse and populate sourceOffsetList;
  int srcOffsetIndex = 0;

  // Areas not processed in the "last" block - right or bottom
  unsigned int excessPixelsRight = destImageWidth % validConfig.dstWidth;
  unsigned int excessPixelsBottom = destImageHeight % validConfig.dstHeight;

  // For each block
  for (unsigned int i = 0; i < destImageHeight; i += validConfig.dstHeight) {
    unsigned int currentBlockHeight = validConfig.dstHeight;
    if ((i > (destImageHeight - validConfig.dstHeight)) && (excessPixelsBottom > 0)) {
      currentBlockHeight = excessPixelsBottom;
    }
    for (unsigned int j = 0; j < destImageWidth; j += validConfig.dstWidth) {
      unsigned int currentBlockWidth = validConfig.dstWidth;
      if ((j > (destImageWidth - validConfig.dstWidth)) && (excessPixelsRight > 0)) {
        currentBlockWidth = excessPixelsRight;
      }

      // Min and max X and Y values observed, in source image offsets
      unsigned int offsetMinX = 0xffffffff;		// set to very large integer
      unsigned int offsetMinY = 0xffffffff;		// set to very large integer

      // We are parsing each offset in the destination block
      int blkIndex = i * destImageWidth + j;

      // For each pixel in the block
      for (unsigned int k = 0; k < currentBlockHeight; k++) {
        // Offset of first pixel in block row
        int offsetIndex = blkIndex + k * destImageWidth;

        for (unsigned int l = 0; l < currentBlockWidth; l++) {
          unsigned int xMap = (pOffsetMap[offsetIndex]) & offsetShiftValue;
          unsigned int yMap = (pOffsetMap[offsetIndex]) >> offsetShift;

          if ((xMap < offsetMinX) || (yMap < offsetMinY)) {
            if (xMap < offsetMinX) { offsetMinX = xMap; }
            if (yMap < offsetMinY) { offsetMinY = yMap; }

            // NB: this might not even be one of the offsets in the block
            offsetTL = offsetMinY * acSrcStride + offsetMinX;
          }
          offsetIndex++;
        } // next pixel in row
      } // next pixel row in block

      // Determine results of block processing
      pSourceOffsetList[srcOffsetIndex++] = offsetTL;

    } // next block in row
  } // next row of blocks

  return rVal;
}

int32_t Remap::FixedToLocalOffsets(vsdk::SUMat &aFixedOffset, const int32_t acSrcStride)
{
  vsdk::SMat moOffsetMat = moOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat localOffsetMat = mLocalOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat fixedOffsetMat = aFixedOffset.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

  uint32_t *srcBlockOffsets = (uint32_t*)moOffsetMat.data;
  uint32_t *offsetPtr = (uint32_t*)fixedOffsetMat.data;
  uint16_t *optr = (uint16_t*)localOffsetMat.data;

  // Takes advantage of the fact that 1DOffset and Dest images are the same size
  const unsigned int numBlocksX = (aFixedOffset.cols + validConfig.dstWidth - 1) / validConfig.dstWidth;

  // for each pixel, probably single loop only needed?  but need to know my block number
  for (int y = 0; y < aFixedOffset.rows; ++y) {
    int by = y / validConfig.dstHeight;

    for (int x = 0; x < aFixedOffset.cols; ++x) {
      // Convert offset to 1-D block offset by:
      // - determine which source block to use
      // - subtract the offset of the top left of the current source block
      // - calucalte x, y offsets to top left of current source block by:
      //    offX = (offset - TLoffset) % stride   offY = (offset - TLoffset) / stride
      // - blockoffset = offY * blockstride + offX

      unsigned int bx = x / validConfig.dstWidth;

      // offset of the top left pixel of the source block
      unsigned int offsetTL = srcBlockOffsets[by * numBlocksX + bx];

      // Because offsetVal = (yi * acSrcStride) + xi;
      unsigned int offBlockX = (*offsetPtr & offsetShiftValue) - (offsetTL % acSrcStride);
      unsigned int offBlockY = (*offsetPtr >> offsetShift) - (offsetTL / acSrcStride);

      *optr = offBlockY * validConfig.srcWidth + offBlockX;

      offsetPtr++;
      optr++;
    }
  }

  return 0;
}

void Remap::PopulateBlockDesc(vsdk::SUMat aData, blockDesc *apDesc)
{
  vsdk::SMat dataMat = aData.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  uint16_t *t = (uint16_t*)dataMat.data;
  for (int32_t cu = 0; cu < dataMat.cols; ++cu) {
    if (t[0] > apDesc->srcWidth) {
      apDesc->srcWidth = t[0];
    }

    if (t[1] > apDesc->srcHeight) {
      apDesc->srcHeight = t[1];
    }

    t += 2;
  }

  apDesc->srcWidth += 1;
  apDesc->srcWidth = (apDesc->srcWidth + 3) & (~3);
  apDesc->srcHeight += 1;

  apDesc->memSize = apDesc->srcWidth * apDesc->srcHeight * 2 + apDesc->dstWidth * apDesc->dstHeight * 10;
  apDesc->ratio = (float)(apDesc->dstWidth * apDesc->dstHeight) / (float)(apDesc->srcWidth * apDesc->srcHeight);
}

APEXCV_LIB_RESULT Remap::Initialize(float* apMap,
  vsdk::SUMat aSrc,
  vsdk::SUMat aDst,
  INTER_TYPE aInterp,
  BORDER_TYPE aBorder,
  uint32_t aBorderValue)
{

  APEXCV_LIB_RESULT rval = APEXCV_ERROR_INTERNAL_ERROR;

  blockDesc candidates[4] = { { 0,0,0,0,0.0,0 } };

  vsdk::SUMat two;
  vsdk::SUMat four;
  vsdk::SUMat eight;
  vsdk::SUMat sixteen;
  vsdk::SUMat fixedOffset;

  uint16_t chunkWidth = 4;
  uint16_t chunkHeight = 4;
  int32_t numCU;
  int32_t candidateNumCU = 0;

  float maxRatio = 0;
  bool validCandidate = false;

  uint16_t blocksX;
  uint16_t blocksY;

  REMAP_BLOCK_SIZE *pBlockProcess;
  REMAP_GRAY *pGrayProcess;
  REMAP_BLOCK_SIZE *pRemapBlockProcPtr;
  REMAP_GRAY *pRemapProcPtr;


  if (0 != mInitState)
  {
    rval = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    goto exit;
  }

  if (aSrc.empty() || aDst.empty())
  {
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  if ((aSrc.type() != VSDK_CV_8UC1) ||
    (aDst.type() != VSDK_CV_8UC1))
  {
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  pBlockProcess = new REMAP_BLOCK_SIZE;
  if (nullptr == pBlockProcess)
  {
    rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto exit;
  }
  mpRemapBlockProc = pBlockProcess;

  pGrayProcess = new REMAP_GRAY;
  if (nullptr == pGrayProcess)
  {
    rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto exit;
  }
  mpRemapProc = pGrayProcess;

  pRemapBlockProcPtr = (REMAP_BLOCK_SIZE *)mpRemapBlockProc;
  pRemapProcPtr = (REMAP_GRAY *)mpRemapProc;

  if (aInterp != INTER_LINEAR) {
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  if (aBorder != BORDER_CONSTANT || aBorderValue != 0) {
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  if (pRemapProcPtr->Initialize() || pRemapBlockProcPtr->Initialize())
  {
    rval = APEXCV_ERROR_ACF_PROC_INITIALIZE;
    goto exit;
  }
  {
    ApexcvHostBaseBaseClass::InfoCluster lInfo;
    lInfo.set_ACF("REMAP_GRAY", pRemapProcPtr);
    lInfo.push_PortName("SRC_IMG");
    mvInfoClusters.push_back(lInfo);
  }
  fixedOffset = vsdk::SUMat(aDst.rows, aDst.cols, VSDK_CV_32SC1);
  mDelta = vsdk::SUMat(aDst.rows, aDst.cols, VSDK_CV_8UC2);
  mLocalOffset = vsdk::SUMat(aDst.rows, aDst.cols, VSDK_CV_16UC1);

  if (fixedOffset.empty() || mDelta.empty() || mLocalOffset.empty())
  {
    rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto exit;
  }


  FloatToFixedOffsets(apMap, fixedOffset, aSrc.cols, aSrc.rows);

  numCU = pRemapProcPtr->RetCuArrayWidth();
  // adujst number of cu for remap
  candidateNumCU = AdjustNumberOfCU(aDst.cols);
  if (candidateNumCU == 0)
  {
    rval = APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    goto exit;
  }
  numCU = candidateNumCU;

  two = vsdk::SUMat(1, numCU, VSDK_CV_16UC2);
  four = vsdk::SUMat(1, numCU, VSDK_CV_16UC2);
  eight = vsdk::SUMat(1, numCU, VSDK_CV_16UC2);
  sixteen = vsdk::SUMat(1, numCU, VSDK_CV_16UC2);

  if (two.empty() || four.empty() || eight.empty() || sixteen.empty())
  {
    rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto exit;
  }

  if (pRemapBlockProcPtr->ConnectIO("MAP", fixedOffset) ||
    pRemapBlockProcPtr->ConnectIO("TWO_HEIGHT", two) ||
    pRemapBlockProcPtr->ConnectIO("FOUR_HEIGHT", four) ||
    pRemapBlockProcPtr->ConnectIO("EIGHT_HEIGHT", eight) ||
    pRemapBlockProcPtr->ConnectIO("SIXTEEN_HEIGHT", sixteen))
  {
    rval = APEXCV_ERROR_ACF_PROC_CONNECTIO;
    goto exit;
  }
  {
    ApexcvHostBaseBaseClass::InfoCluster lInfo;
    lInfo.set_ACF("REMAP_BLOCK_SIZE", pRemapBlockProcPtr);
    lInfo.push_PortName("MAP");
    mvInfoClusters.push_back(lInfo);
  }
  chunkWidth = (aDst.cols + numCU - 1) / numCU;
  chunkHeight = 16;
  if (pRemapBlockProcPtr->SelectScenario("MAP", chunkWidth, chunkHeight) != 0) {
    chunkHeight = 8;
    if (pRemapBlockProcPtr->SelectScenario("MAP", chunkWidth, chunkHeight) != 0) {
      rval = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      goto exit;
    }
  }

  rval |= pRemapBlockProcPtr->Start();
  {
    pRemapBlockProcPtr->Wait();
    candidates[0].dstWidth = chunkWidth;
    candidates[0].dstHeight = 2;
    PopulateBlockDesc(two, &candidates[0]);

    candidates[1].dstWidth = chunkWidth;
    candidates[1].dstHeight = 4;
    PopulateBlockDesc(four, &candidates[1]);

    candidates[2].dstWidth = chunkWidth;
    candidates[2].dstHeight = 8;
    PopulateBlockDesc(eight, &candidates[2]);
    if (chunkHeight == 16) {
      candidates[3].dstWidth = chunkWidth;
      candidates[3].dstHeight = 16;
      PopulateBlockDesc(sixteen, &candidates[3]);
    }

    for (uint32_t i = 0; i < sizeof(candidates) / sizeof(struct blockDesc); ++i)
    {
      blockDesc *desc = &candidates[i];

      if (desc->memSize > pRemapProcPtr->RetCmemHeapSizeInBytes()) {
        continue;
      }

      if (desc->srcWidth * desc->srcHeight * numCU >= (1 << 16)) {
        continue;
      }

      if (desc->ratio > maxRatio) {
        maxRatio = desc->ratio;
        validConfig = *desc;
        validCandidate = true;
      }
    }

  }

  if (!validCandidate)
  {
    rval = APEXCV_ERROR_ACF_PROC_INITIALIZE;
    goto exit;
  }

  blocksX = (aDst.cols + validConfig.dstWidth - 1) / validConfig.dstWidth;
  blocksY = (aDst.rows + validConfig.dstHeight - 1) / validConfig.dstHeight;

  moOffset = vsdk::SUMat(blocksY, blocksX, VSDK_CV_32SC1);
  if (moOffset.empty()) {
    rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto exit;
  }

  InitMoOffsets(fixedOffset, aSrc.step[0]);
  FixedToLocalOffsets(fixedOffset, aSrc.step[0]);

  {
    if (pRemapProcPtr->ConnectIO("DST_IMG", aDst) ||
      pRemapProcPtr->ConnectIndirectInput("SRC_IMG", aSrc, moOffset) ||
      pRemapProcPtr->ConnectIO("LOCAL_OFFSET", mLocalOffset) ||
      pRemapProcPtr->ConnectIO("DELTA", mDelta))
    {
      rval = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      goto exit;
    }
    if (pRemapProcPtr->SelectScenario("", 0, 0) != 0) {
      rval = APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
      goto exit;
    }

    {
      acf_scenario *s;
      s = pRemapProcPtr->RetSelectedScenario();
      s->pBufferData[0].chunkWidth = validConfig.srcWidth;
      s->pBufferData[0].chunkHeight = validConfig.srcHeight;

      s->pBufferData[1].chunkWidth = validConfig.dstWidth;
      s->pBufferData[1].chunkHeight = validConfig.dstHeight;

      s->pBufferData[2].chunkWidth = validConfig.dstWidth;
      s->pBufferData[2].chunkHeight = validConfig.dstHeight;

      s->pBufferData[3].chunkWidth = validConfig.dstWidth;
      s->pBufferData[3].chunkHeight = validConfig.dstHeight;
    }
  }
  mSrcWidth = aSrc.cols;
  mSrcHeight = aSrc.rows;
  mSrcStride = aSrc.step[0];
  mDstWidth = aDst.cols;
  mDstHeight = aDst.rows;

  mInitState = 1;
  rval = APEXCV_SUCCESS;

exit:
  if (APEXCV_SUCCESS != rval)
  {
    Release();
  }

  return rval;
}

APEXCV_LIB_RESULT Remap::ReconnectIO(vsdk::SUMat &aSrc, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT rval = APEXCV_SUCCESS;

  if (1 != mInitState)
  {
    rval = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    goto exit;
  }

  if ((aSrc.type() != VSDK_CV_8UC1 ) ||
    (aDst.type() != VSDK_CV_8UC1))
  {
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  // src and dst width and height should stay the same
  // src stride should stay the same
  if (
    (aSrc.cols != mSrcWidth) ||
    (aSrc.rows != mSrcHeight) ||
    (aSrc.step[0] != mSrcStride) ||
    (aDst.cols != mDstWidth) ||
    (aDst.rows != mDstHeight))
  {
    // changes in these parameters not supported, as it needs a mapping parameter change
    rval = APEXCV_ERROR_INVALID_ARGUMENT;
    goto exit;
  }

  {
    REMAP_GRAY *pRemapProcPtr = (REMAP_GRAY *)mpRemapProc;

    if (pRemapProcPtr->ConnectIO("DST_IMG", aDst) ||
      pRemapProcPtr->ConnectIndirectInput("SRC_IMG", aSrc, moOffset))
    {
      rval = APEXCV_ERROR_ACF_PROC_CONNECTIO;
      goto exit;
    }
  }

exit:
  return rval;
}

APEXCV_LIB_RESULT Remap::Process()
{
  APEXCV_LIB_RESULT rval = APEXCV_SUCCESS;

  if (1 != mInitState)
  {
    return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }

  int status = 0;
  REMAP_GRAY *pRemapProcPtr = (REMAP_GRAY *)mpRemapProc;

  status |= pRemapProcPtr->Start();
  status |= pRemapProcPtr->Wait();

  if (status)
  {
    rval = APEXCV_ERROR_ACF_PROC_EXEC;
    goto exit;
  }

exit:
  return rval;
}

APEXCV_LIB_RESULT Remap::GenerateLUTFromCalibLoader(const char* apcFilename,
  uint32_t aDstWidth, uint32_t aDstHeight,
  uint32_t aSrcWidth, uint32_t aSrcHeight,
  uint32_t /*destBlockWidth*/, uint32_t /*destBlockHeight*/,
  uint32_t /*refSrcBlockWidth*/, uint32_t /*refSrcBlockHeight*/)
{
  APEXCV_LIB_RESULT rval = APEXCV_SUCCESS;

  if (mpParamLoader != 0 &&
    mpGroundToWarp != 0)
  {
    int status = 0;
    ParamLoader& rParamLoader = *((ParamLoader*)mpParamLoader);
    PerspectiveMapper& rPerspectiveMapper = *((PerspectiveMapper*)mpGroundToWarp);

    status |= rParamLoader.load(apcFilename, 0);
    DewarpMapper dewarpMapper(rParamLoader.cameraParams,
      rParamLoader.distortParams,
      rParamLoader.dewarpCameraParams);

    PerspectiveMapper topMapper;
    PerspectiveMapper leftMapper;
    PerspectiveMapper rightMapper;
    Homography        groundToDewarp;
    PerspectiveMapper groundToWarp;
    MatQ1616 map;

    if (0 == status)
    {
      rParamLoader.configureEBCMappers(dewarpMapper,
        topMapper,
        leftMapper,
        rightMapper,
        groundToDewarp,
        rPerspectiveMapper);

      status |= createMapQ1616(map,
        topMapper,
        rParamLoader.topImageSize);
    }
    else
    {
      rval = APEXCV_ERROR_INVALID_ARGUMENT;
      goto EXIT;
    }
    if (0 == status)
    {

      // Delta + Offset combined file
      //-------------------------------

      // Convert Image types to ContigDataDesc types, for portability
      vsdk::SUMat lSrcDesc(aSrcHeight, aSrcWidth, VSDK_CV_8UC1);
      vsdk::SUMat lDestACFDesc(aDstHeight, aDstWidth, VSDK_CV_8UC1);
      if (lSrcDesc.empty() || lDestACFDesc.empty())
      {
        rval = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
      }
      else
      {
        // Perform remap
        // ACF Remap
        // apexcv interface
        rval = Remap::Initialize((float *)map.ReturnPtr(),
          lSrcDesc,
          lDestACFDesc,
          INTER_LINEAR,
          BORDER_CONSTANT,
          0);
      }
    }
    else
    {
      rval = APEXCV_ERROR_INTERNAL_ERROR;
      goto EXIT;
    }
    //Set up mapping parameters
    mTopViewPixelToMeterRatioX = rParamLoader.topGroundROI.width / aDstWidth;
    mTopViewPixelToMeterRatioY = rParamLoader.topGroundROI.height / aDstHeight;
  }
  else
  {

    rval = APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
  }
  EXIT:
  return rval;
}

int Remap::RetLUTs(vsdk::SUMat &aDeltaDesc,
  vsdk::SUMat &aLocalOffsetDesc,
  vsdk::SUMat &aBlockOffsetDesc)
{
  aDeltaDesc = mDelta;
  aLocalOffsetDesc = mLocalOffset;
  aBlockOffsetDesc = moOffset;

  return 0;
}

void Remap::GenerateFloatMap(vsdk::SUMat& aInput,
  vsdk::SUMat& aOutput,
  float* apMap,
  float aMax_offset_per_dim,
  int aSeed)
{
  srand(aSeed);

  for (int32_t y = 0; y < aOutput.rows; ++y)
  {
    float iy = ((float)y * aInput.rows) / aOutput.rows;
    for (int32_t x = 0; x < aOutput.cols; ++x)
    {
      float ix = ((float)x * aInput.cols) / aOutput.cols;
      float rx = ix + ((float)rand() / (float)(RAND_MAX)) * aMax_offset_per_dim - (aMax_offset_per_dim / 2.0f);
      //float rx = ix + 2.0f;
      rx = rx < 0 ? 0 : rx;
      rx = rx > aInput.cols ? aInput.cols : rx;

      float ry = iy + ((float)rand() / (float)(RAND_MAX)) * aMax_offset_per_dim - (aMax_offset_per_dim / 2.0f);
      //float ry = iy;
      ry = ry < 0 ? 0 : ry;
      ry = ry > aInput.rows ? aInput.rows : ry;

      *apMap++ = rx;
      *apMap++ = ry;
    }
  }
}

// return number of multiply by 64
int32_t Remap::AdjustImageWidth(int32_t aWidth)
{
  int32_t numCU;
  int32_t remain = 0;
  int32_t candidateWidth = 0;
  int32_t chunkSize = 0;

  if (nullptr == mpRemapProc)
  {
    return aWidth;
  }

  REMAP_GRAY *pRemapProcPtr = (REMAP_GRAY *)mpRemapProc;
  numCU = pRemapProcPtr->RetCuArrayWidth();
  if ((remain = (aWidth%numCU)) == 0)
  {
    candidateWidth = aWidth;
  }
  else
  {
    candidateWidth = aWidth - remain + numCU;
  }

  // check the chunk size
  chunkSize = candidateWidth / numCU;
  //int multiply = 0;
  if (chunkSize > 72)
  {
    chunkSize = (chunkSize + 0x7) & ~0x7;
  }
  else if (chunkSize > 32)
  {
    chunkSize = (chunkSize + 0x3) & ~0x3;
  }
  else
  {
    chunkSize = (chunkSize + 0x1) & ~0x1;
  }
  return chunkSize * numCU;
}

// return number of cu that fits for multiply by 64
int32_t Remap::AdjustNumberOfCU(int32_t aWidth)
{
  int32_t numCU = 0;
  int32_t remain = 0;
  int32_t candidateCU = 0;
  int32_t cNumCU = 0;

  if (nullptr == mpRemapProc)
  {
    return 0;
  }

  REMAP_GRAY *pRemapProcPtr = (REMAP_GRAY *)mpRemapProc;
  numCU = pRemapProcPtr->RetCuArrayWidth();
  for (uint32_t i = 0; i < sizeof(chunkSizeCandidate); i++)
  {
    remain = aWidth%chunkSizeCandidate[i];
    cNumCU = aWidth / chunkSizeCandidate[i];
    if ((remain == 0) && (cNumCU <= numCU))
    {
      // with of chunk offset array must be multiple of 4
      // # of pointers in the horizontal dimension must be a multiple of 4
      // this is a limitation of motion com
      if (((cNumCU % 4) == 0) &&
        (cNumCU > candidateCU))
      {
        candidateCU = cNumCU;
      }
      else
      {
        if (candidateCU != 0)
          break;
      }
    }
  }
  if (candidateCU != 0)
  {
    return candidateCU;
  }
  return 0; // there is no suitable number of cu with chunk size
}
