/**************************************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2015-2018 NXP
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
**************************************************************************************************/

 #include "hog_apu.h"
 #include "hog_config.h"

#ifdef ACF_KERNEL_METADATA
#include "hog_acf.h"

KERNEL_INFO  kernelInfoConcat(HOG_GET_GRADIENT_K)
(
  HOG_GET_GRADIENT_KN,
  3,
  __port(__index(0),
         __identifier(IMAGE),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(1, 1, 1, 1),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(GRADIENT_MAGNITUDE),
         __attributes(ACF_ATTR_VEC_OUT),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(2),
         __identifier(GRADIENT_ANGLE_BIN),
         __attributes(ACF_ATTR_VEC_OUT),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO  kernelInfoConcat(HOG_GET_CELL_HISTOGRAMS_K)
(
  HOG_GET_CELL_HISTOGRAMS_KN,
  3,
  __port(__index(0),
         __identifier(CELL_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_OUT),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d16u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(GRADIENT_MAGNITUDE),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_hogFixedParams.winStride.width, gc_hogFixedParams.winStride.height)),
  __port(__index(2),
         __identifier(GRADIENT_ANGLE_BIN),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_hogFixedParams.winStride.width, gc_hogFixedParams.winStride.height))
);

KERNEL_INFO  kernelInfoConcat(HOG_COMPUTE_BLOCK_HISTOGRAMS_K)
(
  HOG_COMPUTE_BLOCK_HISTOGRAMS_KN,
  3,
  __port(__index(0),
         __identifier(BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_OUT_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height)),
  __port(__index(1),
         __identifier(CELL_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(0, 1, 0, 1),
         __e0_data_type(d16u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, 1)),
  __port(__index(2),
         __identifier(ROW),
         __attributes(ACF_ATTR_SCL_OUT_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO  kernelInfoConcat(HOG_GET_BLOCK_HISTOGRAMS_K)
(
  HOG_GET_BLOCK_HISTOGRAMS_KN,
  2,
  __port(__index(0),
         __identifier(BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_OUT),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_hogFixedParams.nbins, 1)),
  __port(__index(1),
         __identifier(CELL_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(0, 1, 0, 1),
         __e0_data_type(d16u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, 1))
);

KERNEL_INFO  kernelInfoConcat(HOG_GET_SCORES_K)
(
  HOG_GET_SCORES_KN,
  11,
  __port(__index(0),
         __identifier(SCORES),
         __attributes(ACF_ATTR_VEC_OUT),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d16s),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_IN_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height)),
  __port(__index(2),
         __identifier(SVM_FUNCTION),
         __attributes(ACF_ATTR_SCL_IN_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08s),
         __e0_size(1, 1),
         __ek_size(gc_maxWinBlocks.width*gc_maxWinBlocks.height * gc_hogFixedParams.nbins + 2, 1)),
  __port(__index(3),
         __identifier(SVM_TRANSFORM_MODE),
         __attributes(ACF_ATTR_SCL_IN_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(4),
         __identifier(WIN_BLOCKS),
         __attributes(ACF_ATTR_SCL_IN_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(2, 1)),
  __port(__index(5),
         __identifier(X_LUT),
         __attributes(ACF_ATTR_SCL_OUT_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_maxWinBlocks.width, 1)),
  __port(__index(6),
         __identifier(R_LUT),
         __attributes(ACF_ATTR_SCL_OUT_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(7),
         __identifier(Y_LUT),
         __attributes(ACF_ATTR_SCL_OUT_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height, 1)),
  __port(__index(8),
         __identifier(ROW),
         __attributes(ACF_ATTR_SCL_OUT_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(9),
         __identifier(READY_BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_SCL_OUT_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(10),
         __identifier(SCORES_ROW_BUFFER),
         __attributes(ACF_ATTR_VEC_OUT_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d16s),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO  kernelInfoConcat(HOG_CONVERT_BLOCK_HISTOGRAMS_K)
(
  HOG_CONVERT_BLOCK_HISTOGRAMS_KN,
  3,
  __port(__index(0),
         __identifier(IMAGE_BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_IN),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(gc_hogFixedParams.nbins, 1)),
  __port(__index(1),
         __identifier(BLOCK_HISTOGRAMS),
         __attributes(ACF_ATTR_VEC_OUT_STATIC),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(gc_hogFixedParams.nbins, 1),
         __ek_size(1, gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height)),
  __port(__index(2),
         __identifier(ROW),
         __attributes(ACF_ATTR_SCL_OUT_STATIC_FIXED),
         __spatial_dep(0, 0, 0, 0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);
#endif   // #ifdef ACF_KERNEL_METADATA

#ifdef ACF_KERNEL_IMPLEMENTATION

void hog_get_gradient(kernel_io_desc image,
                      kernel_io_desc magnitude,
                      kernel_io_desc angleBin)
{
  const vec08u* iPtr = (const vec08u*)image.pMem;
  vec08u* mPtr = (vec08u*)magnitude.pMem;
  vec08u* aPtr = (vec08u*)angleBin.pMem;

  HOG_getGradient(mPtr, magnitude.chunkSpan,
                  aPtr, angleBin.chunkSpan,
                  iPtr, image.chunkSpan,
                  magnitude.chunkHeight,
                  magnitude.chunkWidth);
}

void hog_get_cell_histograms(kernel_io_desc cellHistograms,
                             kernel_io_desc magnitude,
                             kernel_io_desc angleBin)
{
  vec16u* c = (vec16u*)cellHistograms.pMem;
  const vec08u* m = (const vec08u*)magnitude.pMem;
  const vec08u* a = (const vec08u*)angleBin.pMem;

  HOG_getCellHistograms(c, cellHistograms.chunkSpan>>1,
                        m, magnitude.chunkSpan,
                        a, angleBin.chunkSpan,
                        magnitude.chunkWidth, magnitude.chunkHeight,
                        gc_hogFixedParams.winStride.width, gc_hogFixedParams.winStride.height);
}

void hog_compute_block_histograms(kernel_io_desc blockHistograms,
                                  kernel_io_desc cellHistograms,
                                  kernel_io_desc rowBuf)
{
  vec08u*  b = (vec08u*)blockHistograms.pMem;
  vec16u*  c = (vec16u*)cellHistograms.pMem;
  int08u row = *((int08u*)rowBuf.pMem);

  if(ACF_RET_VAR(ACF_VAR_FIRST_TILE_FLAG))
  {
    row = 0;
  }

  HOG_getBlockHistograms(b + row * blockHistograms.chunkWidth * gc_hogFixedParams.nbins,
                         blockHistograms.chunkSpan,
                         c, cellHistograms.chunkSpan>>1,
                         cellHistograms.chunkWidth,
                         cellHistograms.chunkHeight,
                         gc_hogFixedParams.winStridePerBlock.width,
                         gc_hogFixedParams.winStridePerBlock.height);

  row++;
  if(row == gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height)
  {
    row = 0;
  }
  *(int08u*)(rowBuf.pMem) = row;
}

void hog_get_block_histograms(kernel_io_desc blockHistograms,
                              kernel_io_desc cellHistograms)
{
  vec08u*  b = (vec08u*)blockHistograms.pMem;
  vec16u*  c = (vec16u*)cellHistograms.pMem;

  HOG_getBlockHistograms(b, blockHistograms.chunkSpan,
                         c, cellHistograms.chunkSpan>>1,
                         cellHistograms.chunkWidth,
                         cellHistograms.chunkHeight,
                         gc_hogFixedParams.winStridePerBlock.width,
                         gc_hogFixedParams.winStridePerBlock.height);
}

void hog_get_scores(kernel_io_desc scores,
                    kernel_io_desc blockHistograms,
                    kernel_io_desc svmFunction,
                    kernel_io_desc svmTransformMode,
                    kernel_io_desc winBlocksBuff,
                    kernel_io_desc x_lut,
                    kernel_io_desc r_lut,
                    kernel_io_desc y_lut,
                    kernel_io_desc rowBuf,
                    kernel_io_desc readyBlockHistogramsBuf,
                    kernel_io_desc scoresRowBuf)
{
  int08u* winBlocks = (int08u*)winBlocksBuff.pMem;
  int08u* x = (int08u*)x_lut.pMem;
  int08u* r = (int08u*)r_lut.pMem;
  int08u* y = (int08u*)y_lut.pMem;
  int08u row = *((int08u*)rowBuf.pMem);
  int08u readyBlockHistograms = *((int08u*)readyBlockHistogramsBuf.pMem);

  if(ACF_RET_VAR(ACF_VAR_FIRST_TILE_FLAG))
  {
    row = 0;
    readyBlockHistograms = 0;

    // Speed up: precompute x and y indices for the loops in order to avoid mod operations
    HOG_createLUTs(x, r, y,
        blockHistograms.chunkWidth,
        gc_hogFixedParams.winStridePerBlock.width,
        gc_hogFixedParams.winStridePerBlock.height,
        winBlocks[0],
        gc_maxWinBlocks.height);
  }

  // Speed up: skip score's computation for the first gc_hogFixedParams.winStridePerBlock.height * (winBlocks[1] - 1) rows,
  // for which not all block histograms has been computed yet. These skipped scores should be ignored in the output data.
  if (readyBlockHistograms)
  {
    vec16s* s = (vec16s*)scores.pMem;
    const vec08u* b = (const vec08u*)blockHistograms.pMem;
    const int08s* f = (const int08s*)svmFunction.pMem;
    vec16s* scoresRow = (vec16s*)scoresRowBuf.pMem;
    int08u svmMirrored = *(int08u*)svmTransformMode.pMem;

    if (svmMirrored == 2)
    {
      // Mirror the SVM function and save the scores to a temporary score buffer
      svmMirrored = 1;
      HOG_getScores(scoresRow,
                    b,
                    blockHistograms.chunkWidth,
                    blockHistograms.chunkHeight,
                    blockHistograms.chunkSpan,
                    f,
                    svmMirrored,
                    x,
                    y,
                    r,
                    row,
                    gc_hogFixedParams.winStridePerBlock.height,
                    winBlocks[0],
                    winBlocks[1]);

      // Continue with SVM as it is
      svmMirrored = 0;
    }

    // Compute the scores
    HOG_getScores(s,
                  b,
                  blockHistograms.chunkWidth,
                  blockHistograms.chunkHeight,
                  blockHistograms.chunkSpan,
                  f,
                  svmMirrored,
                  x,
                  y,
                  r,
                  row,
                  gc_hogFixedParams.winStridePerBlock.height,
                  winBlocks[0],
                  winBlocks[1]);

    if (*(int08u*)svmTransformMode.pMem == 2)
    {
      // Keep the higher score
      HOG_mergeScores(s, scoresRow, blockHistograms.chunkWidth);
    }
  }

  row++;
  if(row == gc_hogFixedParams.winStridePerBlock.height * (winBlocks[1] - 1))
  {
    // All block histograms for the first search window will be available after
    // gc_hogFixedParams.winStridePerBlock.height * winBlocks[1] iterations, i.e. ready in the next call of HOG_getScores()
    readyBlockHistograms = 1;
  }
  else if(row == blockHistograms.chunkHeight)
  {
    // Move to the first row of the chunk
    row = 0;
  }

  *(int08u*)(rowBuf.pMem) = row;
  *(int08u*)(readyBlockHistogramsBuf.pMem) = readyBlockHistograms;
}

void hog_convert_block_histograms(kernel_io_desc imageBlockHistograms,
                                  kernel_io_desc blockHistograms,
                                  kernel_io_desc rowBuf)
{
  const vec08u*  bi = (vec08u*)imageBlockHistograms.pMem;
  vec08u*  bo = (vec08u*)blockHistograms.pMem;
  int08u row = *((int08u*)rowBuf.pMem);

  if(ACF_RET_VAR(ACF_VAR_FIRST_TILE_FLAG))
  {
    row = 0;
  }

  HOG_convertBlockHistograms(bi,
                             bo + row * blockHistograms.chunkWidth * gc_hogFixedParams.nbins,
                             imageBlockHistograms.chunkWidth,
                             imageBlockHistograms.chunkHeight,
                             imageBlockHistograms.chunkSpan);

  row++;
  if(row == gc_hogFixedParams.winStridePerBlock.height * gc_maxWinBlocks.height)
  {
    row = 0;
  }
  *(int08u*)(rowBuf.pMem) = row;
}

#endif //#ifdef ACF_KERNEL_IMPLEMENTATION
