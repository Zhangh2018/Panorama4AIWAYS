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
*  \brief Basic HOG data types.
*
*  Contains the basic data types used in the C reference code.
***********************************************************************************/

#ifndef HOGTYPESREF_H
#define HOGTYPESREF_H
#include <utility>

#include "hog_types_common.h"
#include "fixed.h"
#include "hog_config.h"

#define HIST_Q 10    /*!< Number of fractional bits for the block histogram. */
#define SCOR_Q 11    /*!< Number of fractional bits for the SVM score. */
#define DESC_Q 8     /*!< Number of fractional bits for the HOG descriptor. */
#define FUNC_Q 7     /*!< Number of fractional bits for the SVM decision function. */


/************************************************************************************
*                                   Vectors
************************************************************************************/

/*!*********************************************************************************
*  Vector of unsigned 8-bit fixed point numbers.
***********************************************************************************/
struct VecFix8u
{
  int size;          /*!< Current number of elements. */
  int capacity;      /*!< Total number of elements allocated in memory.*/
  fix8u* data;       /*!< Pointer to the first element. */
  fix8u* data_oal;       /*!< Pointer to the first element. */
};

/*!*********************************************************************************
*  Vector of signed 8-bit fixed point numbers.
***********************************************************************************/
struct VecFix8
{
  int size;          /*!< Current number of elements. */
  int capacity;      /*!< Total number of elements allocated in memory.*/
  fix8s* data;       /*!< Pointer to the first element. */
  fix8s* data_oal;       /*!< Pointer to the first element. */
};

/*!*********************************************************************************
*  Vector of signed 16-bit fixed point numbers.
***********************************************************************************/
struct VecFix16
{
  int size;          /*!< Current number of elements. */
  int capacity;      /*!< Total number of elements allocated in memory.*/
  fix16s* data;      /*!< Pointer to the first element. */
  fix16s* data_oal;       /*!< Pointer to the first element. */
};


/************************************************************************************
*                                   Matrices
************************************************************************************/

/*!*********************************************************************************
*  Matrix of unsigned 8-bit fixed point numbers.
***********************************************************************************/
struct MatFix8u
{
  int rows;          /*!< Number of rows. */
  int cols;          /*!< Number of columns. */
  int channels;      /*!< Number of channels.  E.g. an RGB image has three channels. */
  int step;          /*!< Number of elements that must be skipped to go to the next row. */
  int capacity;      /*!< Total number of elements allocated in memory.*/
  fix8u* data;       /*!< Pointer to the first element. */
  fix8u* data_oal;       /*!< Pointer to the first element. */
};




/************************************************************************************
*                              C reference HOG types
************************************************************************************/

/*!*********************************************************************************
*  The basic parameters that define the HOG descriptor.
***********************************************************************************/
struct HOG_Params
{
  std::pair<int, int> winBlocks;               /*!< number of blocks in detection window */
};

/*!*********************************************************************************
*  Cache required for storing the gradient magnitudes and angle bins.
***********************************************************************************/
struct HOG_GradientCache
{
  Mat8u magnitude;      /*!< Gradient magnitude image. */
  Mat8u qangle;         /*!< Gradient angle bin image. */
};

/*!*********************************************************************************
*  Cache required for storing intermediate values when computing the HOG descriptor.
***********************************************************************************/
struct HOG_ImageCache
{
  int blockHistogramSize;        /*!< Number of elements in the block histogram. */
  int descriptorSize;            /*!< Number of elements in the HOG descriptor. */
  Size blockSize;                /*!< The width and height of a block in pixels. */
  Size windowSize;               /*!< The width and height of a detection window in pixels. */

  HOG_Params params;             /*!< The basic HOG parameters. */
  HOG_GradientCache grad;        /*!< The HOG gradient cache. */

  VecBlockData blockData;        /*!< The block data for each block in the detection window. */
  Vec16s imgOffsets;             /*!< The upper-left pixel location of each block with respect to each detection window. */
};

/*!*********************************************************************************
*  Cache for storing and reusing block histograms when computing detections
*  windows at regular intervals.
***********************************************************************************/
struct HOG_ScanCache
{
  Size windowStride;             /*!< Horizontal and vertical distance, in pixels, between neighbouring detection windows. */
  Size cacheStride;              /*!< The smallest horizontal and vertical distance in pixels between blocks, given the detection window stride. */
  Size nWindows;                 /*!< Total number of detection windows in the entire image. */

  Vec32s ymaxCached;             /*!< Flags for setting a given row of blockCacheFlags to zero. */
  Mat8u histogramComputed;       /*!< Flags indicating whether or not a block histogram has been computed. */
  MatFix8u histograms;           /*!< Matrix of block histograms, one for each block on the image. */
};

/*!*********************************************************************************
*  The SVM decision function.
***********************************************************************************/
struct HOG_DecisionFunction
{
  VecFix8 normal;       /*!< The learned normal to the hyperplane that best separates the training data. */
  fix16s offset;        /*!< The learned offset of the hyperplane. */
};


/*!*********************************************************************************
*  The HOG object detector.
***********************************************************************************/
struct HOG_Detector
{
  HOG_DecisionFunction decisionFunction;     /*!< The SVM decision function learned from training data. */
  HOG_ImageCache imageCache;                 /*!< The image cache. */
  HOG_ScanCache scanCache;                   /*!< The scan cache. */
};

#endif   /* HOGTYPESREF_H */
