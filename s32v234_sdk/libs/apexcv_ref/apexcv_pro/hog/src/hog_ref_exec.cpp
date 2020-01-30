/*******************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2014-2017 NXP
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
*******************************************************************************/

#include "hog_ref_exec.h"
#include "hog_types_ref.h"
#include "apexcv_pro_util.h"
#include "string.h"

int allocateDetector(HOG_Detector& detector, const HOG_Params& params, const Size& maxImageSize)
{
  // Allocate buffers
  Rect hogRect;
  hogRect.x = hogRect.y = 0;
  hogRect.width = maxImageSize.width;
  hogRect.height = maxImageSize.height;
  HOG_allocateCache(detector, params, hogRect);
  detector.imageCache.params = params;

  return 0;
}

void hog_descriptor_ref_exec_padded(uint8_t* blockHistograms,
                                    const uint8_t* paddedImage, int paddedImageSpan,
                                    int paddedRows, int paddedCols,
                                    const HOG_Params& params)
{
  // Use padImageReplicate
  Mat8u image;
  image.data = (uint8_t*)paddedImage;
  image.channels = 1;
  image.cols = paddedCols;
  image.rows = paddedRows;
  image.step = paddedCols;
  image.capacity = image.rows * image.step;

  Rect imageROI;
  imageROI.x = 1;
  imageROI.y = 1;
  imageROI.width = paddedCols - 2;
  imageROI.height = paddedRows - 2;

  Size imageSize;
  imageSize.width = imageROI.width;
  imageSize.height = imageROI.height;


  // C Reference HOG descriptor
  HOG_Detector detector;
  allocateDetector(detector, params, imageSize);

  const int histogramCols = (imageROI.width - (gc_hogFixedParams.block.width - gc_hogFixedParams.winStride.width)) / gc_hogFixedParams.winStride.width;
  const int histogramRows = (imageROI.height - (gc_hogFixedParams.block.height - gc_hogFixedParams.winStride.height)) / gc_hogFixedParams.winStride.height;

  VecFix8u histograms;
  histograms.data = blockHistograms;
  histograms.capacity = histograms.size = histogramCols * histogramRows * gc_hogFixedParams.nbins;

  HOG_compute2(histograms, detector, image, imageROI);

  // clean up
  HOG_deleteCache(detector);
}

void hog_descriptor_ref_exec(uint8_t* blockHistograms,
                             const uint8_t* image, int imageSpan,
                             int rows, int cols,
                             const HOG_Params& params)
{
  // Generate the padded image
  const int paddedRows = rows + 2;
  const int paddedCols = cols + 2;
  const int paddedSize = paddedRows * paddedCols;
  Vec8u paddedImage;
  VECTOR_RESERVE(paddedImage, paddedSize, uint8_t);

  apexcv::padReplicate(paddedImage.data, paddedCols,
                       image, imageSpan,
                       cols, rows,
                       (size_t)1, 1,
                       (size_t)1, 1,
                       1, 1,
                       1, 1);

  // Run the reference code
  hog_descriptor_ref_exec_padded(blockHistograms, paddedImage.data, paddedCols, paddedRows, paddedCols, params);

  VECTOR_DELETE(paddedImage);
}

void hog_ref_exec_padded(int16_t* dst,
                         const uint8_t* paddedImage, int paddedImageSpan,
                         int paddedRows, int paddedCols,
                         const HOG_Params& params,
                         const int8_t* svmFunction)
{
  // Use padImageReplicate
  Mat8u image;
  image.data = (uint8_t*)paddedImage;
  image.channels = 1;
  image.cols = paddedCols;
  image.rows = paddedRows;
  image.step = paddedCols;
  image.capacity = image.rows * image.step;

  Rect imageROI;
  imageROI.x = 1;
  imageROI.y = 1;
  imageROI.width = paddedCols - 2;
  imageROI.height = paddedRows - 2;

  Size imageSize;
  imageSize.width = imageROI.width;
  imageSize.height = imageROI.height;


  // C Reference HOG descriptor
  HOG_Detector detector;
  allocateDetector(detector, params, imageSize);
  // Check the decision function file
  const int descriptorSize = gc_hogFixedParams.nbins * gc_hogFixedParams.blockCells.width * gc_hogFixedParams.blockCells.height * params.winBlocks.first * params.winBlocks.second;

  VecFix8& vec = detector.decisionFunction.normal;
  VECTOR_RESERVE(vec, descriptorSize, fix8s);
  VECTOR_RESIZE(vec, descriptorSize);
  memcpy(vec.data, svmFunction, descriptorSize);
  detector.decisionFunction.offset = *((const int16_t*)(svmFunction + descriptorSize));

  // Compute the SVM scores
  VecPoint locations;
  VecFix16 scores;
  const int N_WINDOWS_X = (imageSize.width - (params.winBlocks.first * gc_hogFixedParams.block.width - gc_hogFixedParams.winStride.width)) / gc_hogFixedParams.winStride.width;    /*!< Number of horizontal detection windows per image. */
  const int N_WINDOWS_Y = (imageSize.height - (params.winBlocks.second * gc_hogFixedParams.block.height - gc_hogFixedParams.winStride.height)) / gc_hogFixedParams.winStride.height;    /*!< Number of vertical   detection windows per image. */

  const int N_windows = N_WINDOWS_X * N_WINDOWS_Y;
  VECTOR_RESERVE(locations, N_windows, Point);
  scores.data = dst;
  scores.capacity = scores.size = N_windows;

  HOG_detect(locations, scores, detector, image, imageROI, (fix16s) - 32768);

  // clean up
  VECTOR_DELETE(locations);
  VECTOR_DELETE(vec);
  HOG_deleteCache(detector);
}

void hog_ref_exec(int16_t* dst,
                  const uint8_t* image, int imageSpan,
                  int rows, int cols,
                  const HOG_Params& params,
                  const int8_t* svmFunction)
{
  // Generate the padded image
  const int paddedRows = rows + 2;
  const int paddedCols = cols + 2;
  const int paddedSize = paddedRows * paddedCols;
  Vec8u paddedImage;
  VECTOR_RESERVE(paddedImage, paddedSize, uint8_t);

  apexcv::padReplicate(paddedImage.data, paddedCols,
                       image, imageSpan,
                       cols, rows,
                       (size_t)1, 1,
                       (size_t)1, 1,
                       1, 1,
                       1, 1);

  // Run the reference code
  hog_ref_exec_padded(dst, paddedImage.data, paddedCols, paddedRows, paddedCols, params, svmFunction);

  VECTOR_DELETE(paddedImage);
}
