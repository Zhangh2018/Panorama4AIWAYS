/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP
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

/*!*****************************************************************************
*  \file
*  \brief Apex HOG Object Detector.
*******************************************************************************/

#ifndef APEXCVHOGREF_H
#define APEXCVHOGREF_H

#include <utility>

// ACF HOST LIB
#include <sumat.hpp>

#include "apexcv_error_codes.hpp"
#include "apexcv_pro_hog.h"

namespace ref
{

/*!*****************************************************************************
*  \brief HOG class.
*******************************************************************************/
  class Hog
  {
public:


  /*!***************************************************************************
  *  \brief Default constructor.
  *****************************************************************************/
  Hog();

  /*!***************************************************************************
  *  Destructor.
  *****************************************************************************/
  ~Hog();

  /*!***************************************************************************
  *  Set Config.
  *****************************************************************************/
  APEXCV_LIB_RESULT SetConfig(const apexcv::Hog::Config& aConfig);

  /*!***************************************************************************
  *  \brief Convert and initialize the SVM port.
  *  \return Error code for memory allocation (APEXCV_SUCCESS on success).
  *
  *  Specify the decision function used by the linear SVM classifier.  The
  *  decision function comes from training the detector on a class of objects
  *  (e.g. pedestrians), so by specifying the decision function you specify
  *  what class of objects you wish to detect.  The size of the decision 
  *  function is descriptorSize + 1 for the SVM weights + offset.
  *****************************************************************************/
  APEXCV_LIB_RESULT SetSVM(const vsdk::SUMat& aSVM  /*!< Vector containing descriptorSize + 1 doubles*/);

  /*!***************************************************************************
  *  \brief Get fixed point SVM decision function 
  *  \return Error code for memory allocation (APEXCV_SUCCESS on success).
  *
  *  This function has to be called after Initialize().
  *****************************************************************************/
  APEXCV_LIB_RESULT GetSVM(vsdk::SUMat& aSVM  /*!< Vector to be populated with descriptorSize linear SVM weights in Q0.7 format */,
                           int16_t& offset    /*!< Linear SVM offset in Q4.11 */ );

  /*!***************************************************************************
  *  \brief Get size of HOG descriptor
  *****************************************************************************/
  size_t GetDescriptorSize() const;

  std::pair<int, int> GetWinBlocks() const;

  /*!***************************************************************************
  *  \brief Run the reference hog algorithm.
  *  \return Zero on success.
  *
  *  The detector takes an 8-bit grayscale image and detects objects every 4x4
  *  pixels.  That is, for each pixel on a 4x4 lattice of the image, the HOG
  *  descriptor is computed.  The trained linear SVM classifier is applied to
  *  each descriptor to produce a 16-bit \em SVM \em score. The output buffer
  *  is allocated within this function and ownership of the memory returns to
  *  the caller with output.
  *
  *****************************************************************************/
  APEXCV_LIB_RESULT Detect(const vsdk::SUMat& aInput  /*!< Populated input image. */,
                                 vsdk::SUMat& aOutput /*!< Output, populated on successful return. */);

  /*!***************************************************************************
  *  \brief Compute the HOG normalized block histograms
  *  \return Error code for the ACF execution (APEXCV_SUCCESS on success).
  *
  *  The detector takes an 8-bit grayscale image and computes the 8-bin
  *  normalized block histogram every 4x4 pixels.
  *  For a (WinxHin) sized image and wxh sized block window the output tensor 
  *  size is: (Wout, Hout, Dout) = (floor((Win-w)/4), floor((Hin-h)/4), 8)
  *
  *****************************************************************************/
  APEXCV_LIB_RESULT GetBlockHistograms(const vsdk::SUMat& aInput  /*!< Populated input image. */,
                                             vsdk::SUMat& aOutput /*!< Output, populated on successful return. */);

  /*!***************************************************************************
  *  \brief Compute the HOG descriptor
  *  \return Error code for the ACF execution (APEXCV_SUCCESS on success).
  *
  *  The detector takes an 8-bit grayscale image and computes the HOG descriptor 
  *  every 4x4 pixels. The HOG descriptor is a column-wise concatenation of the 
  *  block histograms in a detection window.   
  *  For a (WinxHin) sized image and wxh sized block window the output tensor 
  *  size is: (Wout, Hout, Dout) = (floor((Win-w)/4), floor((Hin-h)/4), Nd)
  *
  *****************************************************************************/
  APEXCV_LIB_RESULT GetDescriptors(const vsdk::SUMat& aInput  /*!< Populated input image. */,
                                         vsdk::SUMat& aOutput /*!< Output, populated on successful return. */);

private:
  vsdk::SUMat mSVM;               /*!< SVM decision function memory buffer. */
  apexcv::Hog::Config mHogConfig; /*!< HOG parameters */
  };

  inline std::pair<int,int> Hog::GetWinBlocks() const
  {
    std::pair<int, int> lRetVal;
    lRetVal.first = mHogConfig.mDetWinWidth / mHogConfig.mBlockWidth;
    lRetVal.second = mHogConfig.mDetWinHeight / mHogConfig.mBlockHeight;
    return lRetVal;
  }

  /**************************************************
   * Get the number of elements in the HOG descriptor
   **************************************************/
  inline size_t Hog::GetDescriptorSize() const
  {
    const std::pair<int, int> lWinBlock = GetWinBlocks();
    return lWinBlock.first * lWinBlock.second * mHogConfig.mHistogramBins;
  }

}  /* namespace apex */

#endif /* APEXCVHOGREF_H */
