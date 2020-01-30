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

/*!*********************************************************************************
*  \file
*  \brief C reference HOG detector execution.
*
*  Functions for executing the the C reference fixed-point implementation
*  of the HOG detector.
***********************************************************************************/

#ifndef HOGREFEXEC_H
#define HOGREFEXEC_H

#include "hog_types_ref.h"
#include "hog_gradient_ref.h"

#ifdef __cplusplus
/*********************** Cache Allocation/Deletion **************************/

/*!*********************************************************************************
*  Allocate memory for the image cache.
***********************************************************************************/
void HOG_allocateImageCache(HOG_ImageCache& imageCache   /*!< The image cache. */,
                            const HOG_Params& params     /*!< Basic HOG parameters. */,
                            const Rect& imgROI           /*!< The image region in which detection windows will be evaluated. */);

/*!*********************************************************************************
*  Release memory allocated for the image cache.
***********************************************************************************/
void HOG_deleteImageCache(HOG_ImageCache& imageCache /*!< The image cache. */);

/*!*********************************************************************************
*  Allocate memory for the scan cache.
***********************************************************************************/
void HOG_allocateScanCache(HOG_ScanCache& scanCache   /*!< The scam cache. */,
                           const HOG_Params& params   /*!< Basic HOG parameters. */,
                           const Rect& imgROI         /*!< The image region in which detection windows will be evaluated. */,
                           const Size& cacheStride    /*!< The horizontal and vertical intervals of successive blocks in the image. */);

/*!*********************************************************************************
*  Release memory allocated for the scan cache.
***********************************************************************************/
void HOG_deleteScanCache(HOG_ScanCache& scanCache);

/*!*********************************************************************************
*  Allocate memory for the image cache and scan cache.
***********************************************************************************/
void HOG_allocateCache(HOG_Detector& detector      /*!< The HOG detector. */,
                       const HOG_Params& params    /*!< Basic HOG parameters. */,
                       const Rect& imageROI        /*!< The image region in which detection windows will be evaluated. */);

/*!*********************************************************************************
*  Release memory allocated for the image cache and scan cache.
***********************************************************************************/
void HOG_deleteCache(HOG_Detector& detector);
#endif   // __cplusplus

/************************ Cache Configuration ********************************/

/*!*********************************************************************************
*  Get the detection window size from the basic HOG parameters.
***********************************************************************************/
void HOG_getDetectionWindowSize(Size& detectionWindowSize   /*!< The detection window size. */,
                                const HOG_Params& params    /*!< Basic HOG parameters. */);

/*!*********************************************************************************
*  Configure the image cache from basic HOG parameters and the input image.
***********************************************************************************/
void HOG_configureImageCache(HOG_ImageCache& imageCache     /*!< The image cache. */,
                             const HOG_Params& params       /*!< Basic HOG parameters. */,
                             const Mat8u& image             /*!< The entire input image. */,
                             const Rect& imageROI           /*!< The image region in which detection windows will be evaluated. */);

/*!*********************************************************************************
*  Configure the image cache and scan cache from basic HOG parameters,
*  the input image and the detection window stride.
***********************************************************************************/
void HOG_configureCache(HOG_ImageCache& imageCache    /*!< The image cache. */,
                        HOG_ScanCache& scanCache      /*!< The scan cache. */,
                        const HOG_Params& params      /*!< Basic HOG parameters. */,
                        const Mat8u& image            /*!< The entire input image. */,
                        const Rect& imageROI          /*!< The image region in which detection windows will be evaluated. */);


/************************ HOG Descriptor Computation **************************/

/*!*********************************************************************************
*  Compute the HOG descriptors for an image cache based on a specified detection
*  window locations.
***********************************************************************************/
void HOG_compute(VecFix8u& descriptors          /*!< The set of returned HOG descriptors.
                                                     The entire vector is a concatenation of HOG descriptors
                                                     in the order of the specified locations*/,
                 HOG_ImageCache& imageCache     /*!< The HOG image cache. */,
                 const VecPoint& locations      /*!< Specified detection window locations. */);


/*!*********************************************************************************
*  Compute the HOG descriptors for an image cache based on a regular detection
*  window stride.
***********************************************************************************/
void HOG_compute(VecFix8u& descriptors          /*!< The set of returned HOG descriptors. */,
                 HOG_ImageCache& imageCache     /*!< The HOG image cache. */,
                 HOG_ScanCache& scanCache       /*!< The HOG scan cache, which encodes the detection window stride. */);


/*!*********************************************************************************
*  Compute the HOG descriptors for an image cache based on a regular detection
*  window stride.
***********************************************************************************/
void HOG_compute(VecFix8u& descriptors       /*!< The set of returned HOG descriptors. */,
                 HOG_Detector& detector      /*!< The HOG detector. */,
                 const Mat8u& image          /*!< The entire input image. */,
                 const Rect& imageROI        /*!< The image region in which detection windows will be evaluated. */);

/*!*********************************************************************************
*  Compute the block histograms for an image cache based on a regular detection
*  window stride.
***********************************************************************************/
void HOG_compute2(VecFix8u& blockHistograms,
                  HOG_Detector& detector,
                  const Mat8u& image,
                  const Rect& imageROI);


/************************ HOG Detector **************************/

/*!*********************************************************************************
*  Detect objects in a specified set of locations.
***********************************************************************************/
void HOG_detect(VecPoint& foundLocations     /*!< The set of locations where an object is detected. */,
                VecFix16& scores             /*!< The scores associated with each specified search location. */,
                HOG_ImageCache& imageCache   /*!< The image cache. */,
                const VecPoint& searchLocations    /*!< The search locations. */,
                VecFix8u& descriptorBuffer         /*!< A memory buffer for storing the HOG descriptor. */,
                const HOG_DecisionFunction& decisionFunction   /*!< The SVM decision function. */,
                fix16s threshold = 0      /*!< The SVM threshold.  Any detection window evaluating above the threshold is a positive detection. */);

/*!*********************************************************************************
*  Detect objects for a regular detection window stride.
***********************************************************************************/
void HOG_detect(VecPoint& foundLocations     /*!< The set of locations where an object is detected. */,
                VecFix16& scores             /*!< The scores associated with the regularly spaced detection windows. */,
                HOG_ImageCache& imageCache   /*!< The image cache. */,
                HOG_ScanCache& scanCache     /*!< The scan cache. */,
                const HOG_DecisionFunction& decisionFunction      /*!< The SVM decision function. */,
                fix16s threshold = 0      /*!< The SVM threshold.  Any detection window evaluating above the threshold is a positive detection. */);


/*!*********************************************************************************
*  Configure the HOG detector given the basic parameters, the input image and the
*  detection window stride.
***********************************************************************************/
void HOG_configureDetector(HOG_Detector& detector     /*!< The HOG detector. */,
                           const HOG_Params& params   /*!< The basic HOG parameters. */,
                           const Mat8u& image         /*!< The entire input image. */,
                           const Rect& imageROI       /*!< The image region in which detection windows will be evaluated. */);


/*!*********************************************************************************
*  Detect objects for a regular detection window stride.
***********************************************************************************/
void HOG_detect(VecPoint& foundLocations     /*!< The set of locations where an object is detected. */,
                VecFix16& scores             /*!< The scores associated with the regularly spaced detection windows. */,
                HOG_Detector& detector       /*!< The HOG detector. */,
                const Mat8u& image           /*!< The entire input image. */,
                const Rect& imageROI         /*!< The image region in which detection windows will be evaluated. */,
                fix16s threshold = 0         /*!< The SVM threshold.  Any detection window evaluating above the threshold is a positive detection. */);

void hog_descriptor_ref_exec_padded(uint8_t* blockHistograms,
                                    const uint8_t* paddedImage, int paddedImageSpan,
                                    int paddedRows, int paddedCols,
                                    const HOG_Params& params);

void hog_descriptor_ref_exec(uint8_t* blockHistograms,
                             const uint8_t* image, int imageSpan,
                             int rows, int cols,
                             const HOG_Params& params);

void hog_ref_exec_padded(int16_t* dst,
                         const uint8_t* paddedImage, int paddedImageSpan,
                         int paddedRows, int paddedCols,
                         const HOG_Params& params,
                         const int8_t* svmFunction);

void hog_ref_exec(int16_t* dst,
                  const uint8_t* image, int imageSpan,
                  int rows, int cols,
                  const HOG_Params& params,
                  const int8_t* svmFunction);

#endif   /* HOGREFEXEC_H */
