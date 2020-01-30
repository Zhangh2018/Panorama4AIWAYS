/*******************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2018 NXP
* All Rights Reserved
*
********************************************************************************
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
*******************************************************************************/

#include <apexcv_pro_hog.h>
#include <hog_config.h>
#include <umat.hpp>

// GENERATED ACF PROCESS
#include <HOG_SCORE.hpp>
#include <HOG_DESCRIPTOR.hpp>
#include <HOG_SVM.hpp>

namespace
{
inline size_t GetHogDescriptorSize(uint32_t winBlocksWidth, uint32_t winBlocksHeight)
{
  return winBlocksWidth * winBlocksHeight * gc_hogFixedParams.nbins;
}

APEXCV_LIB_RESULT ConvertSVM(const vsdk::SUMat &aDoubleSVM, vsdk::SUMat &aDstSVM, uint32_t detWinWidth, uint32_t detWinHeight);

APEXCV_LIB_RESULT FormatConfig(apexcv::Hog::Config &aConfig)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_ERROR_INTERNAL_ERROR;

  // validate fixed parameters
  if ((gc_hogFixedParams.block.height != aConfig.mBlockHeight) ||
    (gc_hogFixedParams.block.width != aConfig.mBlockWidth) ||
    (gc_hogFixedParams.nbins != aConfig.mHistogramBins))
  {
    lStatus = APEXCV_ERROR_INVALID_ARGUMENT;
    goto EXIT;
  }

  if (((aConfig.mDetWinWidth / gc_hogFixedParams.block.width) > gc_maxWinBlocks.width) ||
    ((aConfig.mDetWinHeight / gc_hogFixedParams.block.height) > gc_maxWinBlocks.height))
  {
    lStatus = APEXCV_ERROR_INVALID_ARGUMENT;
    goto EXIT;
  }

  //round down window size per number of blocks
  aConfig.mDetWinWidth = (aConfig.mDetWinWidth / gc_hogFixedParams.block.width)*gc_hogFixedParams.block.width;
  aConfig.mDetWinHeight = (aConfig.mDetWinHeight / gc_hogFixedParams.block.height)*gc_hogFixedParams.block.height;
  lStatus = APEXCV_SUCCESS;

EXIT:
  return lStatus;
}

APEXCV_LIB_RESULT AllocateBuffers(const apexcv::Hog::Config &aConfig, vsdk::SUMat &aWinBlocks, vsdk::SUMat &aSVMTransMode);
}

namespace apexcv
{
// Default values for the structure:
Hog::Config::Config() :
  mDetWinWidth(64U),
  mDetWinHeight(128U),
  mBlockWidth(gc_hogFixedParams.block.width),// synchronize APU implementation with interface
  mBlockHeight(gc_hogFixedParams.block.height),// synchronize APU implementation with interface
  mStrideWidth(gc_hogFixedParams.winStride.width),// synchronize APU implementation with interface
  mStrideHeight(gc_hogFixedParams.winStride.height),// synchronize APU implementation with interface
  mHistogramBins(gc_hogFixedParams.nbins),// synchronize APU implementation with interface
  mSVMTransformMode(SVMTransformMode::NONE)
{}

/**************
* Destructor
*************/
Hog::~Hog()
{
  if (nullptr != mpProcess)
  {
    switch ((TYPE)mType)
    {
    case TYPE::DETECT:
      delete (HOG_SCORE*)mpProcess;
      break;
    case TYPE::BLOCKHIST:
      delete (HOG_DESCRIPTOR*)mpProcess;
      break;
    default:
      // would be an error, should not happen
      break;
    }
  }
}

/*********************************************
* Copy the configuration to the data members
*********************************************/
APEXCV_LIB_RESULT Hog::SetConfig(const Config &aConfig)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_ERROR_INTERNAL_ERROR;

  if(nullptr != mpProcess)
  {
    lStatus = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  else
  {
    Config lConfig = aConfig;
    lStatus = FormatConfig(lConfig);
    if (APEXCV_SUCCESS == lStatus)
    {
      mConfig = lConfig;
    }
  }

  return lStatus;
}

/**********************************************
 * Return a copy of the internal configuration
 **********************************************/
void Hog::GetConfig(Config &aConfig) const
{
  aConfig = mConfig;
}

/*!***************************************************************************
*  \brief Create the APU process
*  \return Error code for ACF process initialization, APEXCV_SUCCESS on
*  success.
*
*  Allocates and initializes ACF process.
*****************************************************************************/
APEXCV_LIB_RESULT Hog::Create(TYPE aHogType)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_ERROR_INTERNAL_ERROR;
  std::string lProcessName;

  if(nullptr != mpProcess)
  {
    lStatus = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
    goto EXIT;
  }

  switch (aHogType)
  {
  case TYPE::DETECT:
  {
    HOG_SCORE *pProcess = new HOG_SCORE;
    mpProcess = pProcess;
    lProcessName = "HOG_SCORE";
  }
  break;
  case TYPE::BLOCKHIST:
  {
    HOG_DESCRIPTOR *pProcess = new HOG_DESCRIPTOR;
    mpProcess = pProcess;
    lProcessName = "HOG_DESCRIPTOR";
  }
  break;
  default:
    //mpProcess is not assigned so following test will fail
    break;
  }

  if (nullptr == mpProcess)
  {
    lStatus = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }

  if (((ACF_Process_APU*)mpProcess)->Initialize() != 0)
  {
    // on failure cleanup
    switch (aHogType)
    {
    case TYPE::DETECT:
      delete (HOG_SCORE*)mpProcess;
      break;
    case TYPE::BLOCKHIST:
      delete (HOG_DESCRIPTOR*)mpProcess;
      break;
    default:
      break;
    }
    mpProcess = nullptr;
    lStatus = APEXCV_ERROR_ACF_PROC_INITIALIZE;
    goto EXIT;
  }

  mType = aHogType;
  {
    ApexcvHostBaseBaseClass::InfoCluster lInfo;
    lInfo.set_ACF(lProcessName, mpProcess);
    lInfo.push_PortName("IMAGE");
    mvInfoClusters.push_back(lInfo);
  }
  lStatus = APEXCV_SUCCESS;

EXIT:

  return lStatus;
}

/*********************************
 * Initialize and connect input and output ports
 ********************************/
APEXCV_LIB_RESULT Hog::Initialize(const vsdk::SUMat &aSrc, const vsdk::SUMat &aSVM, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT lStatus = ConvertSVM(aSVM, mSVMUMat, mConfig.mDetWinWidth, mConfig.mDetWinHeight);
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = Create(TYPE::DETECT);
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = AllocateBuffers(mConfig, mWinBlocksUMat, mSVMTransformModeUMat);
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = ReconnectIO(aSrc, aDst);

  EXIT:
  return lStatus;
}

APEXCV_LIB_RESULT Hog::Initialize(const vsdk::SUMat &aSrc, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT lStatus = Create(TYPE::BLOCKHIST);

  if (APEXCV_SUCCESS == lStatus)
  {
    lStatus = ReconnectIO(aSrc, aDst);
  }

  return lStatus;
}

/*********************************
 * Connect input and output ports
 ********************************/
APEXCV_LIB_RESULT Hog::ReconnectIO(const vsdk::SUMat &aSrc, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_SUCCESS;

  if(nullptr == mpProcess)
  {
    return APEXCV_ERROR_ACF_PROC_INITIALIZE;
  }

  if (mType == (int)TYPE::DETECT)
  {
    HOG_SCORE* pProcess = (HOG_SCORE*)mpProcess;

    int32_t numCU = pProcess->RetCuArrayWidth();
    /*
     *  The chunk width must be a multiple of the minimum unit size which is (winStrideX, winStrideY).
     *  Here we determine the smallest chunk width that can still contain the image in the full CU array and
     *  inform ACF of this chunk size through SelectScenario.
     */
    int32_t chunkWidth = (aSrc.cols + numCU - 1) / numCU;
    chunkWidth = (chunkWidth + gc_hogFixedParams.winStride.width - 1) & ~(gc_hogFixedParams.winStride.width - 1);
    chunkWidth = chunkWidth > 8 ? chunkWidth : 8;

    while(chunkWidth <= 32)
    {
      if((aSrc.cols % chunkWidth) == 0)
      {
        if(pProcess->SelectScenario("IMAGE", chunkWidth, gc_hogFixedParams.winStride.height) == 0)
        {
          break;
        }
      }

      chunkWidth += gc_hogFixedParams.winStride.width;
    }

    // Verify that selected chunkWidth is valid
    if(pProcess->SelectScenario("IMAGE", chunkWidth, gc_hogFixedParams.winStride.height))
    {
      return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
    }

    const int outWidth = aSrc.cols / gc_hogFixedParams.winStride.width;
    const int outHeight = aSrc.rows / gc_hogFixedParams.winStride.height;

    bool allocateNew = true;
    if(!aDst.empty())
    {
      // Remove ROI for the existing buffer
      cv::UMat cvDst =  (vsdk::UMat)aDst;
      cv::Size outBufferSize;
      cv::Point outBufferOffset;
      cvDst.locateROI(outBufferSize, outBufferOffset);
      if((outBufferSize.width >= outWidth) && (outBufferSize.height >= outHeight))
      {
        aDst = vsdk::SUMat(aDst, vsdk::Rect(-outBufferOffset.x, -outBufferOffset.y, outWidth, outHeight));
        allocateNew = false;
      }
    }
    if(allocateNew)
    {
      // Allocate new buffer
      aDst = vsdk::SUMat(outHeight, outWidth, VSDK_CV_16SC1);
    }

    if(nullptr == aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    vsdk::SUMat imageUMat(aSrc);
    if( pProcess->ConnectIO("IMAGE", imageUMat)
        || pProcess->ConnectIO("SVM_FUNCTION", mSVMUMat)
        || pProcess->ConnectIO("SVM_TRANSFORM_MODE", mSVMTransformModeUMat)
        || pProcess->ConnectIO("WIN_BLOCKS", mWinBlocksUMat)
        || pProcess->ConnectIO("SCORES", aDst))
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    // Trim the score's extra buffer.
    // Note: first offsetY score rows are skipped, because the APEX computation waits untill the whole buffer
    // of blocks histograms is prepared.
    int offsetY = (mConfig.mDetWinHeight/gc_hogFixedParams.block.height - 1)*gc_hogFixedParams.winStridePerBlock.height;
    if (offsetY < 0)
    {
      offsetY = 0;
    }
    int slidingWinsCols = (aSrc.cols - (mConfig.mDetWinWidth - gc_hogFixedParams.winStride.width)) / gc_hogFixedParams.winStride.width;
    int slidingWinsRows = (aSrc.rows - (mConfig.mDetWinHeight - gc_hogFixedParams.winStride.height)) / gc_hogFixedParams.winStride.height;
    if (slidingWinsCols > outWidth)
    {
      slidingWinsCols = outWidth;
    }
    if (slidingWinsRows > outHeight)
    {
      slidingWinsRows = outHeight;
    }
    aDst = vsdk::SUMat(aDst, vsdk::Rect(0, offsetY, slidingWinsCols, slidingWinsRows));
  }
  else
  {
    HOG_DESCRIPTOR* pProcess = static_cast<HOG_DESCRIPTOR*>(mpProcess);

    int32_t numCU = pProcess->RetCuArrayWidth();
    /*
     *  The chunk width must be a multiple of the minimum unit size which is (winStrideX, winStrideY).
     *  Here we determine the smallest chunk width that can still contain the image in the full CU array and
     *  inform ACF of this chunk size through SelectScenario.
     */
    int32_t chunkWidth = (aSrc.cols + numCU - 1) / numCU;
    chunkWidth = (chunkWidth + gc_hogFixedParams.winStride.width - 1) & ~(gc_hogFixedParams.winStride.width - 1);
    chunkWidth = chunkWidth > 8 ? chunkWidth : 8;

    while(chunkWidth <= 32)
    {
      if((aSrc.cols % chunkWidth) == 0)
      {
        if(pProcess->SelectScenario("IMAGE", chunkWidth, gc_hogFixedParams.winStride.height) == 0)
        {
          break;
        }
      }

      chunkWidth += gc_hogFixedParams.winStride.width;
    }

    // Verify that selected chunkWidth is valid
    if(pProcess->SelectScenario("IMAGE", chunkWidth, gc_hogFixedParams.winStride.height))
    {
      return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
    }

    // Compute all blocks in steps of (winStride.width, winStride.height)
    const unsigned int outWidth = aSrc.cols / gc_hogFixedParams.winStride.width;
    const unsigned int outHeight = aSrc.rows / gc_hogFixedParams.winStride.height;

    bool allocateNew = true;
    if(!aDst.empty())
    {
      // Remove ROI for the existing buffer
      cv::UMat cvDst =  (vsdk::UMat)aDst;
      cv::Size outBufferSize;
      cv::Point outBufferOffset;
      cvDst.locateROI(outBufferSize, outBufferOffset);
      if((static_cast<unsigned int>(outBufferSize.width) >= (outWidth*gc_hogFixedParams.nbins)) &&
         (static_cast<unsigned int>(outBufferSize.height) >= outHeight))
      {
        aDst = vsdk::SUMat(aDst, vsdk::Rect(-outBufferOffset.x, -outBufferOffset.y, outWidth*gc_hogFixedParams.nbins, outHeight));
        allocateNew = false;
      }
    }
    if(allocateNew)
    {
      // Output is an image with gc_hogFixedParams.nbins channels. It is bigger in order to
      // simplify the APEX implementation
      aDst = vsdk::SUMat(outHeight, outWidth*gc_hogFixedParams.nbins, VSDK_CV_8UC1);
    }

    if(nullptr == aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)
    {
      return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }

    vsdk::SUMat imageUMat = vsdk::SUMat(aSrc);
    if(pProcess->ConnectIO("IMAGE", imageUMat) ||
       pProcess->ConnectIO("BLOCK_HISTOGRAMS", aDst))
    {
      return APEXCV_ERROR_ACF_PROC_CONNECTIO;
    }

    // Set ROI for the required number of blocks
    const unsigned int trimOutWidth =  ((aSrc.cols - (gc_hogFixedParams.block.width - gc_hogFixedParams.winStride.width)) / gc_hogFixedParams.winStride.width) * gc_hogFixedParams.nbins;
    const unsigned int trimOutHeight =  (aSrc.rows - (gc_hogFixedParams.block.height - gc_hogFixedParams.winStride.height)) / gc_hogFixedParams.winStride.height;
    aDst = vsdk::SUMat(aDst, vsdk::Rect(0, 0, trimOutWidth, trimOutHeight));
  }

  return lStatus;
}

/**********************************
 * APU HOG Descriptors Computation
 *********************************/
APEXCV_LIB_RESULT Hog::GetDescriptors(const Config &aConfig, const vsdk::SUMat &aBlocksHist, vsdk::SUMat &aDescriptor)
{
  const int inRows = (aBlocksHist.rows + 1) * gc_hogFixedParams.winStride.width;
  const int inCols = (aBlocksHist.cols / gc_hogFixedParams.nbins + 1) * gc_hogFixedParams.winStride.height;
  const int outRows = (inRows - (aConfig.mDetWinHeight - gc_hogFixedParams.winStride.height)) / gc_hogFixedParams.winStride.height;
  const int outCols = (inCols - (aConfig.mDetWinWidth - gc_hogFixedParams.winStride.width)) / gc_hogFixedParams.winStride.width;

  const uint32_t winBlocksWidth = aConfig.mDetWinWidth/gc_hogFixedParams.block.width;
  const uint32_t winBlocksHeight = aConfig.mDetWinHeight/gc_hogFixedParams.block.height;
  const int descriptorSize = GetHogDescriptorSize(winBlocksWidth, winBlocksHeight);

  // Allocate output
  aDescriptor = vsdk::SUMat(outRows, outCols * descriptorSize, VSDK_CV_8UC1);

  vsdk::SMat outMat = aDescriptor.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  vsdk::SMat blockMat = aBlocksHist.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  if(nullptr == blockMat.data)
  {
    return APEXCV_ERROR_INVALID_ARGUMENT;
  }
  if(nullptr == outMat.data)
  {
    return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
  }

  // Build descriptor by concatenating block histograms column-wise
  for(int row = 0; row < outRows; ++row)
  {
    for(int col = 0; col < outCols; ++col)
    {
      // Pointer to output descriptor
      uint8_t* pOut = outMat.data + row * outMat.step[0] + col * descriptorSize;

      // Pointer to block histogram in upper left corner of detection window
      uint8_t* pBlock = blockMat.data + row * blockMat.step[0] + col * gc_hogFixedParams.nbins;

      // Iterate over histograms in the detection window,
      // Assume block stride == block size
      for(unsigned int x = 0; x < winBlocksWidth; ++x)
      {
        int colOffset = x * gc_hogFixedParams.winStridePerBlock.width * gc_hogFixedParams.nbins;

        for(unsigned int y = 0; y < winBlocksHeight; ++y)
        {
          int rowOffset = y * gc_hogFixedParams.winStridePerBlock.height * blockMat.step[0];

          memcpy(pOut, pBlock + colOffset + rowOffset, gc_hogFixedParams.nbins);
          pOut += gc_hogFixedParams.nbins;
        }
      }
    }
  }

  return APEXCV_SUCCESS;
}

Hog::SVM::~SVM()
{
  delete (HOG_SVM*)mpProcess;
}

/*********************************************
 * Copy the configuration to the data members
 *********************************************/
APEXCV_LIB_RESULT Hog::SVM::SetConfig(const Config &aConfig)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_ERROR_INTERNAL_ERROR;

  if(nullptr != mpProcess)
  {
    lStatus = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  else
  {
    Config lConfig = aConfig;
    lStatus = FormatConfig(lConfig);
    if (APEXCV_SUCCESS == lStatus)
    {
      mConfig = lConfig;
    }
  }

  return lStatus;
}

/**********************************************
 * Return a copy of the internal configuration
 **********************************************/
void Hog::SVM::GetConfig(Config &aConfig) const
{
  aConfig = mConfig;
}

/*!***************************************************************************
*  \brief Create the APU process
*  \return Error code for ACF process initialization, APEXCV_SUCCESS on
*  success.
*
*  Allocates and initializes ACF process.
*****************************************************************************/
APEXCV_LIB_RESULT Hog::SVM::Create()
{
  APEXCV_LIB_RESULT lStatus = APEXCV_SUCCESS;

  if(nullptr != mpProcess)
  {
    lStatus = APEXCV_ERROR_OBJECT_ALREADY_INITIALIZED;
  }
  else
  {
    HOG_SVM *pProcess = new HOG_SVM;

    if(nullptr == pProcess)
    {
      lStatus = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    else if(pProcess->Initialize() != 0)
    {
      delete pProcess;
      lStatus = APEXCV_ERROR_ACF_PROC_INITIALIZE;
    }
    else
    {
      ApexcvHostBaseBaseClass::InfoCluster lInfo;
      lInfo.set_ACF("HOG_SVM", pProcess);
      lInfo.push_PortName("IMAGE_BLOCK_HISTOGRAMS");
      mvInfoClusters.push_back(lInfo);

      mpProcess = pProcess;
    }
  }

  return lStatus;
}

/************************************************
 * Initialize and connect input and output ports
 ************************************************/
APEXCV_LIB_RESULT Hog::SVM::Initialize(const vsdk::SUMat &aSrc, const vsdk::SUMat &aSVM, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT lStatus = ConvertSVM(aSVM, mSVMUMat, mConfig.mDetWinWidth, mConfig.mDetWinHeight);
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = Create();
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = AllocateBuffers(mConfig, mWinBlocksUMat, mSVMTransformModeUMat);
  if (APEXCV_SUCCESS != lStatus)
  {
    goto EXIT;
  }

  lStatus = ReconnectIO(aSrc, aDst);
 EXIT:
  return lStatus;
}

/*********************************
 * Connect input and output ports
 ********************************/
APEXCV_LIB_RESULT Hog::SVM::ReconnectIO(const vsdk::SUMat &aSrc, vsdk::SUMat &aDst)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_SUCCESS;

  if(nullptr == mpProcess)
  {
    return APEXCV_ERROR_ACF_PROC_INITIALIZE;
  }

  HOG_SVM* pProcess = (HOG_SVM*)mpProcess;

  // Find the full size of aBlocksHist, ignoring ROI
  cv::UMat cvSrc =  (vsdk::UMat)aSrc;
  cv::Size outBufferSize;
  cv::Point outBufferOffset;
  cvSrc.locateROI(outBufferSize, outBufferOffset);

  int32_t numCU = pProcess->RetCuArrayWidth();

  // Required chunk width is gc_hogFixedParams.nbins, but it should be scaled in order to avoid
  // problems with too many chunks per CU
  int32_t chunkWidthScale = (outBufferSize.width + numCU*gc_hogFixedParams.nbins - 1) / (numCU*gc_hogFixedParams.nbins);
  int32_t chunkWidth = chunkWidthScale * gc_hogFixedParams.nbins;

  // Verify that selected chunkWidth is valid
  if(pProcess->SelectScenario("IMAGE_BLOCK_HISTOGRAMS", chunkWidth, 1))
  {
    return APEXCV_ERROR_ACF_PROC_SELECTSCENARIO;
  }

  const int outWidth = outBufferSize.width / gc_hogFixedParams.nbins;
  const int outHeight = outBufferSize.height;

  bool allocateNew = true;
  if(!aDst.empty())
  {
    // Remove ROI for the existing buffer
    cv::UMat cvDst =  (vsdk::UMat)aDst;
    cvDst.locateROI(outBufferSize, outBufferOffset);
    if((outBufferSize.width >= outWidth) && (outBufferSize.height >= outHeight))
    {
      aDst = vsdk::SUMat(aDst, vsdk::Rect(-outBufferOffset.x, -outBufferOffset.y, outWidth, outHeight));
      allocateNew = false;
    }
  }
  if(allocateNew)
  {
    // Allocate new buffer
    aDst = vsdk::SUMat(outHeight, outWidth, VSDK_CV_16SC1);
  }

  if(nullptr == aDst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data)
  {
    return APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
  }

  // Remove the ROI of the blocks histograms
  vsdk::SUMat histUMat(aSrc);
  histUMat.cols = histUMat.size[1] = outWidth*gc_hogFixedParams.nbins;
  histUMat.rows = outHeight;

  if( pProcess->ConnectIO("IMAGE_BLOCK_HISTOGRAMS", histUMat)
      || pProcess->ConnectIO("SVM_FUNCTION", mSVMUMat)
      || pProcess->ConnectIO("SVM_TRANSFORM_MODE", mSVMTransformModeUMat)
      || pProcess->ConnectIO("WIN_BLOCKS", mWinBlocksUMat)
      || pProcess->ConnectIO("SCORES", aDst))
  {
    return APEXCV_ERROR_ACF_PROC_CONNECTIO;
  }

  // Trim the score's extra buffer.
  // Note: first offsetY score rows are skipped, because the APEX computation waits untill the whole buffer
  // of blocks histograms is prepared.
  int offsetY = (mConfig.mDetWinHeight/gc_hogFixedParams.block.height - 1)*gc_hogFixedParams.winStridePerBlock.height;
  if (offsetY < 0)
  {
    offsetY = 0;
  }
  int slidingWinsCols = aSrc.cols /gc_hogFixedParams.nbins  - (mConfig.mDetWinWidth - gc_hogFixedParams.block.width) / gc_hogFixedParams.winStride.width;
  int slidingWinsRows = aSrc.rows - (mConfig.mDetWinHeight - gc_hogFixedParams.block.height) / gc_hogFixedParams.winStride.height;
  if (slidingWinsCols > outWidth)
  {
    slidingWinsCols = outWidth;
  }
  if (slidingWinsRows > outHeight)
  {
    slidingWinsRows = outHeight;
  }

  aDst = vsdk::SUMat(aDst, vsdk::Rect(0, offsetY, slidingWinsCols, slidingWinsRows));

  return lStatus;
}

}  /* namespace apex */

namespace
{
APEXCV_LIB_RESULT ConvertSVM(const vsdk::SUMat &aDoubleSVM, vsdk::SUMat &aDstSVM, uint32_t detWinWidth, uint32_t detWinHeight)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_SUCCESS;

  const int cSvmSize = GetHogDescriptorSize(detWinWidth/gc_hogFixedParams.block.width,
                                            detWinHeight/gc_hogFixedParams.block.height);

  vsdk::SMat doubleSVM_Mat = aDoubleSVM.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
  double *pDoubleSVM = (double*)doubleSVM_Mat.data;

  if(nullptr == pDoubleSVM)
  {
    lStatus = APEXCV_ERROR_INTERNAL_ERROR;
  }
  else if (aDoubleSVM.rows != 1 || aDoubleSVM.cols < cSvmSize + 1 || aDoubleSVM.type() != VSDK_CV_64FC1)
  {
    lStatus = APEXCV_ERROR_INVALID_ARGUMENT;
  }
  else
  {
    // Allocate svm.width for 8-bit weights + 2 for 16-bit offset
    aDstSVM = vsdk::SUMat(1, GetHogDescriptorSize(gc_maxWinBlocks.width, gc_maxWinBlocks.height) + 2, VSDK_CV_8SC1);

    vsdk::SMat fixSVM_Mat = aDstSVM.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

    if (nullptr == fixSVM_Mat.data)
    {
      lStatus = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    }
    else
    {
      int8_t *pFixSVM = (int8_t*)fixSVM_Mat.data;

      for(int i = 0; i < cSvmSize; ++i)
      {
        // Convert weights from double to Q0.7
        pFixSVM[i] = (int8_t)(pDoubleSVM[i] * 128.0 - 0.5);
      }

      // Convert offset from double to Q4.11
      const int16_t cOffset = (int16_t)(pDoubleSVM[cSvmSize] * 2048.0 - 0.5);
      memcpy(&pFixSVM[cSvmSize], &cOffset, 2);
    }
  }

  return lStatus;
}

APEXCV_LIB_RESULT AllocateBuffers(const apexcv::Hog::Config &aConfig, vsdk::SUMat &aWinBlocks, vsdk::SUMat &aSVMTransMode)
{
  APEXCV_LIB_RESULT lStatus = APEXCV_ERROR_INTERNAL_ERROR;

  // Create UMat for the input graph ports of Hog config
  aWinBlocks = vsdk::SUMat(1, 2, VSDK_CV_8UC1);
  if (aWinBlocks.empty())
  {
    lStatus = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }

  {
    vsdk::SMat lWinBlocksMat = aWinBlocks.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED);
    uint32_t lDetWinWidthInBlocks = aConfig.mDetWinWidth / gc_hogFixedParams.block.width;
    uint32_t lDetWinHeightInBlocks = aConfig.mDetWinHeight / gc_hogFixedParams.block.height;
    lWinBlocksMat.at<uint8_t>((uint32_t)0) = static_cast<uint8_t>(lDetWinWidthInBlocks);
    lWinBlocksMat.at<uint8_t>(1) = static_cast<uint8_t>(lDetWinHeightInBlocks);
  }

  aSVMTransMode = vsdk::SUMat(1, 1, VSDK_CV_8UC1);
  if (aSVMTransMode.empty())
  {
    lStatus = APEXCV_ERROR_MEMORY_ALLOCATION_FAILED;
    goto EXIT;
  }

  {
    vsdk::SMat lSVMTransformModeMat = aSVMTransMode.getMat(vsdk::ACCESS_RW | OAL_USAGE_NONCACHED);
    lSVMTransformModeMat.at<uint8_t>((uint32_t)0) = static_cast<uint8_t>(aConfig.mSVMTransformMode);
  }
  lStatus = APEXCV_SUCCESS;
EXIT:
  return lStatus;
}
}
