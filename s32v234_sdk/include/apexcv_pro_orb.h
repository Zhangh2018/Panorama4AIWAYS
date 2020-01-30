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
*  \file apexcv_pro_orb.h
*  \brief Orb library API
*******************************************************************************/

#ifndef APEXCV_PRO_ORB_H
#define APEXCV_PRO_ORB_H

#include <stdio.h>
#include <stdint.h>
#include <acf_process_apu.h>
#include "apexcv_pro_fast.h"
#include "apexcv_error_codes.hpp"
#include <apexcv_host_base_class.h>

namespace apexcv
{
/*!*****************************************************************************
  *  \brief          Apex Orb class
  *  \details        apexcv::Orb is the host-ACF interface for creating, initializing,
  *                  executing and releasing the resources for running the Orb algorithm
  *******************************************************************************/
class Orb: public ApexcvHostBaseBaseClass
{
public:
  /*!***************************************************************************
  *  \brief          ORB::Corner
  *  \details        The class is used to classify the keypoints found by FAST9
  *                  The member - strength - is the metric for discriminating
  *                  between different corners. The higher the value, the higher
  *                  the probabily to be a corner.
  *****************************************************************************/
  class Corner
  {
    public:
      unsigned int x; // X position in the image
      unsigned int y; // Y position in the image
      float strength; // Corner strength based on the Harris Score
    };
 
/*******************************************************************************
*  Orb default constructor, releases the resources
*******************************************************************************/
  Orb();

/*******************************************************************************
*  Orb default desconstructor, releases the resources
*******************************************************************************/
  ~Orb();

/*!*****************************************************************************
*  \brief  Initializes the Orb class. The function begins by allocating all the
*  necessary buffers then initializes the FAST keypoint detector, Harris Corner
*  Score and Orb orientation.
*  \param[in] aApexId Discriminates between the two APEX accelerators
*  \param[in] aNrOfThreads Indicates how many APUs the algorithm will use
*  \param[in] aBorderSize Image border that will be trimmed from aInImg
*  \param[in] aPatchSize A square region that contains the center feature
*  \param[in] aRadius  Hypothetical circle with its center inside the
*  center of the patch
*  \param[in] aDescriptorSizeInBytes The size of the descriptor in bytes
*  for each patch
*  \param[in] aFastCircumference The circumference of the circle where
*  the FAST9 algorithm is applied
*  \param[in] aFastThreshold Fast algorithm detection threshold
*  \param[in] aHarrisK Harris Corner Coefficent
*  \param[in] aNrOfKeypoints maximum number of keypoints that the algo
*  should look for
*  \return Check apexcv_error_codes.h to see to possible outcomes
*  \note Maximum resolution is <b> 1920 x 1080 pixels </b> with <b> 64 </b> CUs.
*******************************************************************************/
  APEXCV_LIB_RESULT Create(unsigned char aApexId = 0,
                           unsigned char aNrOfThreads = 2,                                                
                           unsigned int aBorderSize = 32,
                           unsigned int aPatchSize = 32,
                           unsigned int aRadius = 16,
                           unsigned int aDescriptorSizeInBytes = 32,
                           unsigned int aFastCircumference = 16,
                           unsigned int aFastThreshold = 20,
                           float aHarrisK = 0.04f,
                           unsigned int aNrOfKeypoints = 512);

/*!*****************************************************************************
*  \brief  Starts the processing of the Orb keypoint detector.
*  Three processes will be run on APEX: FAST, Harris and Orb orientation.
*  After APEX finishes, the best <b> nrOfKeypoints </b> are selected based on
*  Harris Score.
*  that the algo should look for. The number of detected keypoints could be
*  less than <b> nrOfKeypoints </b> if so the value is updated.
*  \param[in] aInImg 8-bit grayscale source image
*  \return Error code for initialization (zero on success).
*  \note This method has the highest latency and the processing is split between
*  APEX and ARM
*  \return Check apexcv_error_codes.h to see to possible outcomes
*******************************************************************************/
  APEXCV_LIB_RESULT Detect(vsdk::SUMat &aInImg);

  /*!*****************************************************************************
*  \brief  Starts the processing of the Orb rBrief
*  \param[in] aInSmoothedImg 8-bit grayscale source image
*  \param[out] aOutDescriptors- [Output] - 8-bit unsigned, will containt out descr.\n
*  Expected size: descriptorSizeBytes * nrOfKeypoints \n
*  \note Maximum resolution is <b> 1920 x 1080 pixels </b> with <b> 64 </b> CUs.
*  \return Check apexcv_error_codes.h to see to possible outcomes
*******************************************************************************/
  APEXCV_LIB_RESULT Compute(vsdk::SUMat &aInSmoothedImg,                           
                            vsdk::SUMat &aOutDescriptors);

  /*!*****************************************************************************
*  \brief Returns the chunk offsets \n
*  \return Reference to an internal pointer that holds the chunk offets.
*          Each chunk is described by a 32 bit value, that represents the offset
*          in bytes from the start of the input image
*******************************************************************************/
  vsdk::SUMat &GetChunkOffsets();

  /*!*****************************************************************************
*  \brief Returns the patch size of rBRIEF/IC \n
*  \return Patch size that rBrief will process, 8 bits, signed
*******************************************************************************/
  vsdk::SUMat &GetPatchSize();

  /*!*****************************************************************************
*  \brief Returns the radius of the IC calculus \n
*  \return Radius of the orientation algorithm, 8 bit, signed
*******************************************************************************/
  vsdk::SUMat &GetRadius();

  /*!*****************************************************************************
*  \brief Returns the FAST9 score image \n
*  \return Pairs of coordianats <y,x> for every keypoint, each pairs has 32 bits
*******************************************************************************/
  vsdk::SUMat &GetFastOut();

  /*!*****************************************************************************
*  \brief Returns the angles for each patch calculated by the ICO \n
*  \return An angles for each keypoint, [0-360] degrees are mapped to [0-255]
*******************************************************************************/
  vsdk::SUMat &GetIcoAngles();

  /*!*****************************************************************************
*  \brief Returns the packed coordinates and metric for the FAST9 Img \n
*******************************************************************************/
  vsdk::SUMat &GetSerializerOut();

  /*!*****************************************************************************
*  \brief  Counter for the keypoints that were detected by FAST \n
*******************************************************************************/
  int GetNrOfDetectedKeypoints();

  /*!*****************************************************************************
*  \brief Counter for the keypoints that are inside the border \n
*******************************************************************************/
  int GetNrOfValidKeypoints();

  /*!*****************************************************************************
*  \brief Orb keypoints will be processed \n
*******************************************************************************/
  bool DataIsValid();

  /*!*****************************************************************************
*  \brief Sets the sampling pattern for BRIEF \n
*  \return Check apexcv_error_codes.h to see to possible outcomes
*******************************************************************************/
  APEXCV_LIB_RESULT SetBriefSamplingPattern(vsdk::SUMat &aInBitPattern);

  /*!*****************************************************************************
* \brief Dumps the keypoints in a standard format \n
* Usage:
* std::vector<Orb::Corner> &kpntVec = orb.GetKeypoints();
* if(kpntVec.size() > 0) {// Only now the buffer is valid};
* \return Array of keypoints, see Orb::Corner type
*******************************************************************************/
  std::vector<Orb::Corner> &GetKeypoints();

private:
  std::vector<Orb::Corner> mKeypoints; /*!< Data container for all the keypoints */
  vsdk::SUMat mInGrayScale;        /*!< keep track of the input data image */
  vsdk::SUMat mPatchSizeInPixels;  /*!< Keep track of the patch size */
  vsdk::SUMat mRadiusInPixels;     /*!< Keep track of the radius size */
  vsdk::SUMat mFastCornerList;     /*!< Fast corner score image */
  vsdk::SUMat mDescrSizeBytes;     /*!< The size of the descriptor in bytes for each patch. */

  vsdk::SUMat *mpChunkOffsets;    /*!< Keeps track of the image offsets for all the valid keypoints */
  vsdk::SUMat *mpAngles;          /*!< 8-bit unsigned char. For each keypoint the algorithm outputs a single value.*/
  vsdk::SUMat *mpDescriptorCount; /*!< 8-bit unsigned char. For each keypoint the algorithm outputs a single value.*/
  vsdk::SUMat *mpBinPatternOut;   /*!< 8-bit unsigned char. This buffer will keep the descriptors but it will be split in nrOfThreads parts*/
  vsdk::SUMat msmplBitPattern;    /*!< 8-bit signed char. Will hold the sampling pattern for the default configuration */

  bool mInit;                       /*!< Indicates if it has been initialized or not*/  
  int mApexId;                      /*!< Discriminates between APEX accelerators */
  int mNrOfThreads;                 /*!< Indicates the number of APUs that the algorithm will use */  
  int mDetectedKeypoints;           /*!< Detected keypoints by the FAST9 algorithm */  
  unsigned int mImgSpan;            /*!< Input image span */
  unsigned int mImgW;               /*!< Input image width */
  unsigned int mImgH;               /*!< Input image height */
  unsigned int mNrOfFeatures;       /*!< Number of desired keypoints */
  unsigned int mDistanceFromBorder; /*!< All the pixels that are inside x,y = borderSize:Width-BorderSize will be processed by the algorithm. */
  unsigned int mFastCirc;           /*!< Fast circumference */
  unsigned int mFastTh;             /*!< Fast threshold */  
  float mHarrisK;                   /*!< Harris scaling constant */
  
  vsdk::SUMat mAllChunkOffsets; /*!< Contains the chunk offsets from both APU */
  vsdk::SUMat mAllAngles;       /*!< Contains estimated intensicy centroid angles from both APU */

  Fast mFast9; /*!< Fast class */

  void *mpProcessCalcOrientation; /*!< Orb orientation ACF process */
  void *mpProcessCalcRbrief;      /*!< Orb rBRIEF ACF process */
};
}
#endif /* APEXCVPROORB_H */
