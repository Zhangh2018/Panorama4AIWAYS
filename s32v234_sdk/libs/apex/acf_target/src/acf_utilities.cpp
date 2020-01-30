/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include <acf_utilities.h>
#include <acf_common.h>
#include <util.h>
#include <stdint.h>
#include <acf_autokernel_inc.h>
#include <acf_memmgmt.h>
#include <apu_test_util.h> //padding
//#include "../../common/include/driver/apu_stdio.h"

#if (1 == APEX_SIM_NATIVE)
#include <native_cmem.h>
#endif

#if (1 == APEX_SIM_ISS || 1 == APEX_SIM_NATIVE)
#include <seq_func_sim.h>
#endif

//global values that can be used by advanced kernels to perform init/reduction operations
int16_t gsACF_FIRST_TILE_FLAG;
int16_t gsACF_LAST_TILE_FLAG;
int16_t gsACF_TILE_WIDTH_IN_CHUNKS;
int16_t gsACF_FIRST_CUID;
int16_t gsACF_NUM_INPUT_ITERATIONS;
int16_t gsACF_CU_ARRAY_WIDTH;
int16_t gsACF_VEC_WRAP_IDX;

int16_t ACF_RET_VAR(ACF_VAR lVar)
{
   int16_t lRetVal = 0;

   switch(lVar)
   {
   case ACF_VAR_FIRST_TILE_FLAG:
      lRetVal = gsACF_FIRST_TILE_FLAG;
      break;
   case ACF_VAR_LAST_TILE_FLAG:
      lRetVal = gsACF_LAST_TILE_FLAG;
      break;
   case ACF_VAR_TILE_WIDTH_IN_CHUNKS:
      lRetVal = gsACF_TILE_WIDTH_IN_CHUNKS;
      break;
   case ACF_VAR_FIRST_CUID:
      lRetVal = gsACF_FIRST_CUID;
      break;
   case ACF_VAR_NUM_INPUT_ITERATIONS:
      lRetVal = gsACF_NUM_INPUT_ITERATIONS;
      break;
   case ACF_VAR_CU_ARRAY_WIDTH:
      lRetVal = gsACF_CU_ARRAY_WIDTH;
      break;
   case ACF_VAR_VEC_WRAP_IDX:
      lRetVal = gsACF_VEC_WRAP_IDX;
      break;
   }

   return lRetVal;
}

//these point to counterparts allocated statically in the main pipeline file (needs to be cleaned up)
vec16u* gpvBytesPushedIntoFifoArray;
uint32_t** gpPtrToFinalFifoSizeArray;
void ACF_PUSH_VEC_BUFFER_TO_FIFO(kernel_io_desc lIoDescVecBuffer, vec16u lvVecBufferSize, kernel_io_desc lIoDescFifoSize)
{
   int16_t lIndex = lIoDescVecBuffer._framework_lmindex;
   gpvBytesPushedIntoFifoArray[lIndex] = lvVecBufferSize;
   gpPtrToFinalFifoSizeArray[lIndex] = (uint32_t*)lIoDescFifoSize.pMem;
}

void memcpy_int(int* lpSrc, int* lpDst, int lSize)
{
   for (int i=0; i<lSize; i++)
      lpDst[i] = lpSrc[i];
}

void memset_vec16s(vec16s* lpDst, int16_t lVal, int lSize)
{
   for (int i = 0; i < lSize; i++)
   {
      lpDst[i] = lVal;
   }
}


#if 0
//this version is only called by custom resize right now ...
kernel_io_desc RetIoDesc(ACF_LocalMemDesc& lLmDesc, int lIter, int lLmIndex)
{
   kernel_io_desc lIoDesc;

   lIoDesc.pMem = RetAddr_iter(&lLmDesc, lIter);
   lIoDesc.chunkWidth  = lLmDesc.chunkWidth;
   lIoDesc.chunkHeight = lLmDesc.chunkHeight;
   lIoDesc.chunkSpan   = lLmDesc.chunkSpan;
   lIoDesc._framework_lmindex = lLmIndex;

   return lIoDesc;
}
#endif

// NB:  it is assumed that 16-bit vector copies/shifts will be twice as efficient as 8-bit copies/shifts.
//      however, no assumption can be made for 8-bit e0 types that the source and destination buffers in a copy
//      will be 16-bit aligned.  As a first pass, 8-bit e0 types will use 8-bit vector copies exclusively,
//      and 16- and 32-bit e0 types will use 16-bit vector copies exclusively.  At a later optimization step,
//      it might be desirable to review this to better optimize 8-bit e0 padding performance.
//      Also note that this requires the calling tree for padding to be identical down to the copy level,
//      at which point the code will branch.
//      This also means that everything before the split must work in 8-bit pointer/vector types.

// NB:  Padding is e0 replication on internal edges, fill of edge-most e0 value on external edges
//  i.e.  -------- --------
//        --ABCD-- --TUVW--
//  becomes
//        AAABCDTU CDTUVWWW
//        AAABCDTU CDTUVWWW

void PadTopBottomHelper (ACF_LocalMemDesc* lpLM,
                         vec08u* lpSrcTopLeft,
                         int lE0Width,
                         int lBlockHeight,
                         int lIter,
                         int lNumIterations,
                         int lPadTop,
                         int lPadBot,
                         int lPadLeft,
                         int lPadChOffsetTop,
                         int lPadChOffsetBot)
{
   vec08u *src, *dst;

   // North padding - First iteration only.  Includes NW and NE corners
   if ((lPadTop>0) && (lIter == 0))
   {
      src = lpSrcTopLeft - lE0Width*lPadLeft + lpLM->chunkE0Span*lPadChOffsetTop;
      dst = src - lPadTop*lpLM->chunkE0Span;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy(dst, lpLM->chunkSpan, src, lpLM->chunkSpan, lpLM->e0.y, lpLM->chunkSpan, lPadTop*lpLM->e0.y);
   }

   // South padding - last iteration only.  Includes SW and SE corners.
   if ((lPadBot>0) && (lIter == lNumIterations-1))
   {
      dst = lpSrcTopLeft + lBlockHeight*lpLM->chunkSpan - lE0Width*lPadLeft - lpLM->chunkE0Span*lPadChOffsetBot;
      src = dst - lpLM->chunkE0Span;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy(dst, lpLM->chunkSpan, src, lpLM->chunkSpan, lpLM->e0.y, lpLM->chunkSpan, lPadBot*lpLM->e0.y);
   }
}

void PadTopBottomHelper8 (ACF_LocalMemDesc* lpLM,
                          vec08u* lpSrcTopLeft,
                          int lE0Width,
                          int lBlockHeight,
                          int lIter,
                          int lNumIterations,
                          int lPadTop,
                          int lPadBot,
                          int lPadLeft,
                          int lPadChOffsetTop,
                          int lPadChOffsetBot)
{
   vec08u *src, *dst;
   int chunkE0Stride = lpLM->chunkE0Span>>0;
   int chunkStride = lpLM->chunkSpan>>0;

   // North padding - First iteration only.  Includes NW and NE corners
   if ((lPadTop>0) && (lIter == 0))
   {
      src = lpSrcTopLeft - lE0Width*lPadLeft + chunkE0Stride*lPadChOffsetTop;
      dst = src - lPadTop*chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_08(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadTop*lpLM->e0.y);
   }

   // South padding - last iteration only.  Includes SW and SE corners.
   if ((lPadBot>0) && (lIter == lNumIterations-1))
   {
      dst = lpSrcTopLeft + lBlockHeight*chunkStride - lE0Width*lPadLeft - chunkE0Stride*lPadChOffsetBot;
      src = dst - chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_08(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadBot*lpLM->e0.y);
   }
}

void PadTopBottomHelper16 (ACF_LocalMemDesc* lpLM,
                           vec16u* lpSrcTopLeft,
                           int lE0Width,
                           int lBlockHeight,
                           int lIter,
                           int lNumIterations,
                           int lPadTop,
                           int lPadBot,
                           int lPadLeft,
                           int lPadChOffsetTop,
                           int lPadChOffsetBot)
{
   vec16u *src, *dst;
   int chunkE0Stride = lpLM->chunkE0Span>>1;
   int chunkStride = lpLM->chunkSpan>>1;

   // North padding - First iteration only.  Includes NW and NE corners
   if ((lPadTop>0) && (lIter == 0))
   {
      src = lpSrcTopLeft - lE0Width*lPadLeft + chunkE0Stride*lPadChOffsetTop;
      dst = src - lPadTop*chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_16(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadTop*lpLM->e0.y);
   }

   // South padding - last iteration only.  Includes SW and SE corners.
   if ((lPadBot>0) && (lIter == lNumIterations-1))
   {
      dst = lpSrcTopLeft + lBlockHeight*chunkStride - lE0Width*lPadLeft - chunkE0Stride*lPadChOffsetBot;
      src = dst - chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_16(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadBot*lpLM->e0.y);
   }
}

void PadTopBottomHelper32 (ACF_LocalMemDesc* lpLM,
                           vec32u* lpSrcTopLeft,
                           int lE0Width,
                           int lBlockHeight,
                           int lIter,
                           int lNumIterations,
                           int lPadTop,
                           int lPadBot,
                           int lPadLeft,
                           int lPadChOffsetTop,
                           int lPadChOffsetBot)
{
   vec32u *src, *dst;
   int chunkE0Stride = lpLM->chunkE0Span>>2;
   int chunkStride = lpLM->chunkSpan>>2;

   // North padding - First iteration only.  Includes NW and NE corners
   if ((lPadTop>0) && (lIter == 0))
   {
      src = lpSrcTopLeft - lE0Width*lPadLeft + chunkE0Stride*lPadChOffsetTop;
      dst = src - lPadTop*chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_32(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadTop*lpLM->e0.y);
   }

   // South padding - last iteration only.  Includes SW and SE corners.
   if ((lPadBot>0) && (lIter == lNumIterations-1))
   {
      dst = lpSrcTopLeft + lBlockHeight*chunkStride - lE0Width*lPadLeft - chunkE0Stride*lPadChOffsetBot;
      src = dst - chunkE0Stride;

      // At this point we should have enough data to copy without having to care
      // if the padding is greater than block height
      apu_til_copy_32(dst, chunkStride, src, chunkStride, lpLM->e0.y, chunkStride, lPadBot*lpLM->e0.y);
   }
}

// Edge cases where not all CUs are used or if the full block is not used
void PadEdgeCaseHelper(ACF_LocalMemDesc* lpLM,
                       vec08u* lpSrcTopLeft,
                       int lE0Width,
                       int lTileWidthInChunks,
                       int lPadRight,
                       int lPadChOffsetRight,
                       int lCuArrayWidth)
{
   vec08u *src, *dst;

   if ((lTileWidthInChunks != lCuArrayWidth) || (lPadChOffsetRight != 0))
   {
      vec16u mask = 0;
      cmem_write((vec16u __cmem*) &mask, lTileWidthInChunks-1, 1);
      dst = lpSrcTopLeft + lE0Width*(lpLM->chunkWidth-lPadChOffsetRight);
      src = dst - lE0Width;
      apu_til_copy_right(dst, lpLM->chunkSpan, src, lpLM->chunkSpan, lE0Width, lE0Width*lPadRight, lpLM->bufferHeight*lpLM->e0.y, mask);
   }
}

void PadEdgeCaseHelper8(ACF_LocalMemDesc* lpLM,
                        vec08u* lpSrcTopLeft,
                        int lE0Width,
                        int lTileWidthInChunks,
                        int lPadRight,
                        int lPadChOffsetRight,
                        int lCuArrayWidth)
{
   vec08u *src, *dst;

   if ((lTileWidthInChunks != lCuArrayWidth) || (lPadChOffsetRight != 0))
   {
      vec16u mask = 0;
      cmem_write((vec16u __cmem*) &mask, lTileWidthInChunks-1, 1);
      dst = lpSrcTopLeft + lE0Width*(lpLM->chunkWidth-lPadChOffsetRight);
      src = dst - lE0Width;

      if (vany(mask==1))
         apu_til_copy_right_08(dst, (lpLM->chunkSpan>>0), src, (lpLM->chunkSpan>>0), lE0Width, lE0Width*lPadRight, lpLM->bufferHeight*lpLM->e0.y, mask);
   }
}

void PadEdgeCaseHelper16(ACF_LocalMemDesc* lpLM,
                         vec16u* lpSrcTopLeft,
                         int lE0Width,
                         int lTileWidthInChunks,
                         int lPadRight,
                         int lPadChOffsetRight,
                         int lCuArrayWidth)
{
   vec16u *src, *dst;

   if ((lTileWidthInChunks != lCuArrayWidth) || (lPadChOffsetRight != 0))
   {
      vec16u mask = 0;
      cmem_write((vec16u __cmem*) &mask, lTileWidthInChunks-1, 1);
      dst = lpSrcTopLeft + lE0Width*(lpLM->chunkWidth-lPadChOffsetRight);
      src = dst - lE0Width;

      if (vany(mask==1))
         apu_til_copy_right_16(dst, (lpLM->chunkSpan>>1), src, (lpLM->chunkSpan>>1), lE0Width, lE0Width*lPadRight, lpLM->bufferHeight*lpLM->e0.y, mask);
   }
}

void PadEdgeCaseHelper32(ACF_LocalMemDesc* lpLM,
                         vec32u* lpSrcTopLeft,
                         int lE0Width,
                         int lTileWidthInChunks,
                         int lPadRight,
                         int lPadChOffsetRight,
                         int lCuArrayWidth)
{
   vec32u *src, *dst;

   if ((lTileWidthInChunks != lCuArrayWidth) || (lPadChOffsetRight != 0))
   {
      vec16u mask = 0;
      cmem_write((vec16u __cmem*) &mask, lTileWidthInChunks-1, 1);
      dst = lpSrcTopLeft + lE0Width*(lpLM->chunkWidth-lPadChOffsetRight);
      src = dst - lE0Width;

      if (vany(mask==1))
         apu_til_copy_right_32(dst, (lpLM->chunkSpan>>2), src, (lpLM->chunkSpan>>2), lE0Width, lE0Width*lPadRight, lpLM->bufferHeight*lpLM->e0.y, mask);
   }
}

//NOTE: the reason the padding function was broken down into 12 different methods was to to reduce run-time branching and prevent the inclusion
//      of unnecessary functions/code in the build

void ACF_PAD_08_e0xis1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                 int lCbIndex,
                                 int lIter,
                                 int lNumIterations,
                                 int lTileWidthInChunks,
                                 int lPadTop,
                                 int lPadBot,
                                 int lPadLeft,
                                 int lPadRight,
                                 int lPadChOffsetTop,
                                 int lPadChOffsetBot,
                                 int lPadChOffsetLeft,
                                 int lPadChOffsetRight,
                                 int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_08_h2(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lPadLeft);
      else
         apu_til_padhor_08(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_08_e0xis1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                    int lCbIndex,
                                    int lIter,
                                    int lNumIterations,
                                    int lTileWidthInChunks,
                                    int lPadTop,
                                    int lPadBot,
                                    int lPadLeft,
                                    int lPadRight,
                                    int lPadChOffsetTop,
                                    int lPadChOffsetBot,
                                    int lPadChOffsetLeft,
                                    int lPadChOffsetRight,
                                    int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_08_h2(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);
      else
         apu_til_padhor_08(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_08_e0xnot1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                  int lCbIndex,
                                  int lIter,
                                  int lNumIterations,
                                  int lTileWidthInChunks,
                                  int lPadTop,
                                  int lPadBot,
                                  int lPadLeft,
                                  int lPadRight,
                                  int lPadChOffsetTop,
                                  int lPadChOffsetBot,
                                  int lPadChOffsetLeft,
                                  int lPadChOffsetRight,
                                  int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_08_h2_e0x(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft);
      else
         apu_til_padhor_08_e0x(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_08_e0xnot1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                     int lCbIndex,
                                     int lIter,
                                     int lNumIterations,
                                     int lTileWidthInChunks,
                                     int lPadTop,
                                     int lPadBot,
                                     int lPadLeft,
                                     int lPadRight,
                                     int lPadChOffsetTop,
                                     int lPadChOffsetBot,
                                     int lPadChOffsetLeft,
                                     int lPadChOffsetRight,
                                     int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_08_h2_e0x(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);
      else
         apu_til_padhor_08_e0x(srcTopLeft, lpLM->chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_16_e0xis1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                 int lCbIndex,
                                 int lIter,
                                 int lNumIterations,
                                 int lTileWidthInChunks,
                                 int lPadTop,
                                 int lPadBot,
                                 int lPadLeft,
                                 int lPadRight,
                                 int lPadChOffsetTop,
                                 int lPadChOffsetBot,
                                 int lPadChOffsetLeft,
                                 int lPadChOffsetRight,
                                 int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lPadLeft);
      else
         apu_til_padhor_16((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_16_e0xis1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                    int lCbIndex,
                                    int lIter,
                                    int lNumIterations,
                                    int lTileWidthInChunks,
                                    int lPadTop,
                                    int lPadBot,
                                    int lPadLeft,
                                    int lPadRight,
                                    int lPadChOffsetTop,
                                    int lPadChOffsetBot,
                                    int lPadChOffsetLeft,
                                    int lPadChOffsetRight,
                                    int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);
      else
         apu_til_padhor_16((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_16_e0xnot1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                  int lCbIndex,
                                  int lIter,
                                  int lNumIterations,
                                  int lTileWidthInChunks,
                                  int lPadTop,
                                  int lPadBot,
                                  int lPadLeft,
                                  int lPadRight,
                                  int lPadChOffsetTop,
                                  int lPadChOffsetBot,
                                  int lPadChOffsetLeft,
                                  int lPadChOffsetRight,
                                  int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft);
      else
         apu_til_padhor_16_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_16_e0xnot1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                     int lCbIndex,
                                     int lIter,
                                     int lNumIterations,
                                     int lTileWidthInChunks,
                                     int lPadTop,
                                     int lPadBot,
                                     int lPadLeft,
                                     int lPadRight,
                                     int lPadChOffsetTop,
                                     int lPadChOffsetBot,
                                     int lPadChOffsetLeft,
                                     int lPadChOffsetRight,
                                     int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);
      else
         apu_til_padhor_16_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_32_e0xis1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                 int lCbIndex,
                                 int lIter,
                                 int lNumIterations,
                                 int lTileWidthInChunks,
                                 int lPadTop,
                                 int lPadBot,
                                 int lPadLeft,
                                 int lPadRight,
                                 int lPadChOffsetTop,
                                 int lPadChOffsetBot,
                                 int lPadChOffsetLeft,
                                 int lPadChOffsetRight,
                                 int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
        apu_til_padhor_32((vec32u*)srcTopLeft, lpLM->chunkSpan>>2, lpLM->chunkWidth, blockHeight, lPadLeft); //This optimization doesn't work for 32-bit variant
      else
         apu_til_padhor_32((vec32u*)srcTopLeft, lpLM->chunkSpan>>2, lpLM->chunkWidth, blockHeight, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_32_e0xis1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                    int lCbIndex,
                                    int lIter,
                                    int lNumIterations,
                                    int lTileWidthInChunks,
                                    int lPadTop,
                                    int lPadBot,
                                    int lPadLeft,
                                    int lPadRight,
                                    int lPadChOffsetTop,
                                    int lPadChOffsetBot,
                                    int lPadChOffsetLeft,
                                    int lPadChOffsetRight,
                                    int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_32_h2((vec32u*)srcTopLeft, lpLM->chunkSpan>>2, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);
      else
         apu_til_padhor_32((vec32u*)srcTopLeft, lpLM->chunkSpan>>2, lpLM->chunkWidth, blockHeight, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_32_e0xnot1_LequaltoR(ACF_LocalMemDesc* lpLM,
                                  int lCbIndex,
                                  int lIter,
                                  int lNumIterations,
                                  int lTileWidthInChunks,
                                  int lPadTop,
                                  int lPadBot,
                                  int lPadLeft,
                                  int lPadRight,
                                  int lPadChOffsetTop,
                                  int lPadChOffsetBot,
                                  int lPadChOffsetLeft,
                                  int lPadChOffsetRight,
                                  int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x<<1, lPadLeft);
      else
         apu_til_padhor_16_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x<<1, lPadLeft);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_32_e0xnot1_LnotequaltoR(ACF_LocalMemDesc* lpLM,
                                     int lCbIndex,
                                     int lIter,
                                     int lNumIterations,
                                     int lTileWidthInChunks,
                                     int lPadTop,
                                     int lPadBot,
                                     int lPadLeft,
                                     int lPadRight,
                                     int lPadChOffsetTop,
                                     int lPadChOffsetBot,
                                     int lPadChOffsetLeft,
                                     int lPadChOffsetRight,
                                     int lCuArrayWidth)
{
   int e0Width = RetE0WidthInBytes(&lpLM->e0);
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (lpLM->chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>1, lpLM->chunkWidth, blockHeight, lpLM->e0.x<<1, lPadLeft, lPadRight);
      else
         apu_til_padhor_16_e0x((vec16u*)srcTopLeft, lpLM->chunkSpan>>2, lpLM->chunkWidth, blockHeight, lpLM->e0.x<<1, lPadLeft, lPadRight);

      PadEdgeCaseHelper(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_08(ACF_LocalMemDesc* lpLM,
                int lCbIndex,
                int lIter,
                int lNumIterations,
                int lTileWidthInChunks,
                int lPadTop,
                int lPadBot,
                int lPadLeft,
                int lPadRight,
                int lPadChOffsetTop,
                int lPadChOffsetBot,
                int lPadChOffsetLeft,
                int lPadChOffsetRight,
                int lCuArrayWidth)
{
   int chunkSpan = lpLM->chunkSpan>>0;
   int e0Width = (RetE0WidthInBytes(&lpLM->e0))>>0;
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec08u* srcTopLeft = (vec08u*)(lpLM->pBuf0) + (chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_08_h2_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);
      else
         apu_til_padhor_08_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);

      PadEdgeCaseHelper8(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper8(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_16(ACF_LocalMemDesc* lpLM,
                int lCbIndex,
                int lIter,
                int lNumIterations,
                int lTileWidthInChunks,
                int lPadTop,
                int lPadBot,
                int lPadLeft,
                int lPadRight,
                int lPadChOffsetTop,
                int lPadChOffsetBot,
                int lPadChOffsetLeft,
                int lPadChOffsetRight,
                int lCuArrayWidth)
{
   int chunkSpan = lpLM->chunkSpan>>1;
   int e0Width = (RetE0WidthInBytes(&lpLM->e0))>>1;
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec16u* srcTopLeft = (vec16u*)(lpLM->pBuf0) + (chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_16_h2_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);
      else
         apu_til_padhor_16_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);

      PadEdgeCaseHelper16(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper16(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

void ACF_PAD_32(ACF_LocalMemDesc* lpLM,
                int lCbIndex,
                int lIter,
                int lNumIterations,
                int lTileWidthInChunks,
                int lPadTop,
                int lPadBot,
                int lPadLeft,
                int lPadRight,
                int lPadChOffsetTop,
                int lPadChOffsetBot,
                int lPadChOffsetLeft,
                int lPadChOffsetRight,
                int lCuArrayWidth)
{
   int chunkSpan = lpLM->chunkSpan>>2;
   int e0Width = (RetE0WidthInBytes(&lpLM->e0))>>2;
   int blockHeight = lpLM->bufferHeight*lpLM->e0.y;
   vec32u* srcTopLeft = (vec32u*)(lpLM->pBuf0) + (chunkSpan*blockHeight*lCbIndex) + lPadChOffsetLeft*e0Width;

   if (lPadLeft > 0 || lPadRight > 0)
   {
      if (!(blockHeight&0x1))
         apu_til_padhor_32_h2_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);
      else
         apu_til_padhor_32_e0x(srcTopLeft, chunkSpan, lpLM->chunkWidth, blockHeight, lpLM->e0.x, lPadLeft, lPadRight);

      PadEdgeCaseHelper32(lpLM, srcTopLeft, e0Width, lTileWidthInChunks, lPadRight, lPadChOffsetRight, lCuArrayWidth);
   }

   PadTopBottomHelper32(lpLM, srcTopLeft, e0Width, blockHeight, lIter, lNumIterations, lPadTop, lPadBot, lPadLeft, lPadChOffsetTop, lPadChOffsetBot);
}

// NB:  Padding is e0 replication on internal edges, fill of edgemost e0 value on external edges
//      This method autofills the top and bottom pads of a multi-buffer region, when necessary
//  i.e.  -------- --------
//        --ABCD-- --TUVW--
//
//        --JKFL-- --MNOP--
//
//        --abcd-- --tuvw--
//        -------- --------
// either the abcd/tuvw will be filled at teh top, or ABCD/TUVW will be filled at the bottom.
// Left/Right padding is not addressed
//
// NB:  This method is guaranteed to be called after the relevant ACF_PAD call.  This means that
//      a single copy of left/middle/centre can be made.
void ACF_COPYBUF(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft)
{
   // Copy or fill dimensions
   int copyHeight;
   // Determine TL corner of source, destination blocks
   vec08u* src;
   vec08u* dst;

   int e0Width = RetE0WidthInBytes(&(lpLM->e0));

   // We're guaranteed to never have to do anything on the first iteration
   if (lIter > 0) {
      int currentBuffer = lCbIndex;
      // Top to bottom case
      // - Do this for enough buffers (starting at 0) such that lPadBot is satisfied (if the chunkHeight < lPadBot it requires multiple copies!!!)
      int lLinesCopied = currentBuffer*lpLM->chunkHeight;
      if (lLinesCopied<lPadBot && lIter >= lpLM->numBuffers) {
         // Copy Top row to bottom padding region
         int lNumLinesToBeCopiedDown = lPadBot-lLinesCopied;
         if (lIter == lNumIterations-1) {
            // padding has been completed so can do a final copy since all data is available
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         } else if (lNumLinesToBeCopiedDown >= lpLM->chunkHeight) {
            copyHeight = lpLM->chunkHeight * lpLM->e0.y;
         } else {
            // lNumLinesToBeCopiedDown < lpLM->chunkHeight
            copyHeight = (lNumLinesToBeCopiedDown) * lpLM->e0.y;
         }

         // Determine TL corner of source, destination blocks
         src = (vec08u*)(lpLM->pBuf0) + (currentBuffer*lpLM->chunkHeight*lpLM->chunkE0Span) - (lPadLeft*e0Width);
         dst = src + lpLM->chunkE0Span * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy(dst, lpLM->chunkSpan, src, lpLM->chunkSpan, copyHeight, lpLM->chunkSpan, copyHeight);

      // Bottom to top case
      // - since we don't know where the bottom of the image is, always do this
      } else if ((currentBuffer == (lpLM->numBuffers-1)) && (lPadTop > 0)) {
         // Copy Bottom to top padding region
         copyHeight = lPadTop * lpLM->e0.y;

         // Determine TL corner of source, destination blocks
         src = (vec08u*)(lpLM->pBuf0)
               + (lpLM->chunkHeight * lpLM->numBuffers - lPadTop) * lpLM->chunkE0Span
               - lPadLeft * e0Width;
         dst = src - lpLM->chunkE0Span * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy(dst, lpLM->chunkSpan, src, lpLM->chunkSpan, copyHeight, lpLM->chunkSpan, copyHeight);
      }
   } // endif first iteration
}

void ACF_COPYBUF_08(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft)
{
   int copyHeight;
   vec08u* src;
   vec08u* dst;
   int chunkE0Stride = lpLM->chunkE0Span>>0;
   int chunkStride = lpLM->chunkSpan>>0;
   int e0Width = (RetE0WidthInBytes(&(lpLM->e0)))>>0;

   // We're guaranteed to never have to do anything on the first iteration
   if (lIter > 0) {
      int currentBuffer = lCbIndex;
      // Top to bottom case
      // - Do this for enough buffers (starting at 0) such that lPadBot is satisfied (if the chunkHeight < lPadBot it requires multiple copies!!!)
      int lLinesCopied = currentBuffer*lpLM->chunkHeight;
      if (lLinesCopied<lPadBot && lIter >= lpLM->numBuffers) {
         // Copy Top row to bottom padding region
         int lNumLinesToBeCopiedDown = lPadBot-lLinesCopied;
         if (lIter == lNumIterations-1)
         {
            //padding has been completed so can do a final copy since all data is available
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }
         else if(lNumLinesToBeCopiedDown >= lpLM->chunkHeight)
         {
            copyHeight = lpLM->chunkHeight * lpLM->e0.y;
         }
         else //lNumLinesToBeCopiedDown < lpLM->chunkHeight
         {
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }

         // Determine TL corner of source, destination blocks
         src = (vec08u*)(lpLM->pBuf0) + (currentBuffer*lpLM->chunkHeight*chunkE0Stride) - (lPadLeft*e0Width);
         dst = src + chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_08(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);

      // Bottom to top case
      // - since we don't know where the bottom of the image is, always do this
      } else if ((currentBuffer == (lpLM->numBuffers-1)) && (lPadTop > 0)) {
         // Copy Bottom to top padding region
         copyHeight = lPadTop * lpLM->e0.y;

         // Determine TL corner of source, destination blocks
         src = (vec08u*)(lpLM->pBuf0)
               + (lpLM->chunkHeight * lpLM->numBuffers - lPadTop) * chunkE0Stride
               - lPadLeft * e0Width;
         dst = src - chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_08(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);
      }
   } // endif first iteration
}

void ACF_COPYBUF_16(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft)
{
   int copyHeight;
   vec16u* src;
   vec16u* dst;
   int chunkE0Stride = lpLM->chunkE0Span>>1;
   int chunkStride = lpLM->chunkSpan>>1;
   int e0Width = (RetE0WidthInBytes(&(lpLM->e0)))>>1;

   // We're guaranteed to never have to do anything on the first iteration
   if (lIter > 0) {
      int currentBuffer = lCbIndex;
      // Top to bottom case
      // - Do this for enough buffers (starting at 0) such that lPadBot is satisfied (if the chunkHeight < lPadBot it requires multiple copies!!!)
      int lLinesCopied = currentBuffer*lpLM->chunkHeight;
      if (lLinesCopied<lPadBot && lIter >= lpLM->numBuffers) {
         // Copy Top row to bottom padding region
         int lNumLinesToBeCopiedDown = lPadBot-lLinesCopied;
         if (lIter == lNumIterations-1)
         {
            //padding has been completed so can do a final copy since all data is available
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }
         else if (lNumLinesToBeCopiedDown >= lpLM->chunkHeight)
         {
            copyHeight = lpLM->chunkHeight * lpLM->e0.y;
         }
         else //lNumLinesToBeCopiedDown < lpLM->chunkHeight
         {
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }

         // Determine TL corner of source, destination blocks
         src = (vec16u*)(lpLM->pBuf0) + (currentBuffer*lpLM->chunkHeight*chunkE0Stride) - (lPadLeft*e0Width);
         dst = src + chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_16(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);

      // Bottom to top case
      // - since we don't know where the bottom of the image is, always do this
      } else if ((currentBuffer == (lpLM->numBuffers-1)) && (lPadTop > 0)) {
         // Copy Bottom to top padding region
         copyHeight = lPadTop * lpLM->e0.y;

         // Determine TL corner of source, destination blocks
         src = (vec16u*)(lpLM->pBuf0)
               + (lpLM->chunkHeight * lpLM->numBuffers - lPadTop) * chunkE0Stride
               - lPadLeft * e0Width;
         dst = src - chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_16(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);
      }
   } // endif first iteration
}

void ACF_COPYBUF_32(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft)
{
   int copyHeight;
   vec32u* src;
   vec32u* dst;
   int chunkE0Stride = lpLM->chunkE0Span>>2;
   int chunkStride = lpLM->chunkSpan>>2;
   int e0Width = (RetE0WidthInBytes(&(lpLM->e0)))>>2;

   // We're guaranteed to never have to do anything on the first iteration
   if (lIter > 0) {
      int currentBuffer = lCbIndex;
      // Top to bottom case
      // - Do this for enough buffers (starting at 0) such that lPadBot is satisfied (if the chunkHeight < lPadBot it requires multiple copies!!!)
      int lLinesCopied = currentBuffer*lpLM->chunkHeight;
      if (lLinesCopied<lPadBot && lIter >= lpLM->numBuffers) {
         // Copy Top row to bottom padding region
         int lNumLinesToBeCopiedDown = lPadBot-lLinesCopied;
         if (lIter == lNumIterations-1)
         {
            //padding has been completed so can do a final copy since all data is available
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }
         else if (lNumLinesToBeCopiedDown >= lpLM->chunkHeight)
         {
            copyHeight = lpLM->chunkHeight * lpLM->e0.y;
         }
         else //lNumLinesToBeCopiedDown < lpLM->chunkHeight
         {
            copyHeight = lNumLinesToBeCopiedDown * lpLM->e0.y;
         }

         // Determine TL corner of source, destination blocks
         src = (vec32u*)(lpLM->pBuf0) + (currentBuffer*lpLM->chunkHeight*chunkE0Stride) - (lPadLeft*e0Width);
         dst = src + chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_32(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);

      // Bottom to top case
      // - since we don't know where the bottom of the image is, always do this
      } else if ((currentBuffer == (lpLM->numBuffers-1)) && (lPadTop > 0)) {
         // Copy Bottom to top padding region
         copyHeight = lPadTop * lpLM->e0.y;

         // Determine TL corner of source, destination blocks
         src = (vec32u*)(lpLM->pBuf0)
               + (lpLM->chunkHeight * lpLM->numBuffers - lPadTop) * chunkE0Stride
               - lPadLeft * e0Width;
         dst = src - chunkE0Stride * lpLM->chunkHeight * lpLM->numBuffers;

         apu_til_copy_32(dst, chunkStride, src, chunkStride, copyHeight, chunkStride, copyHeight);
      }
   } // endif first iteration
}

int LmInit(ACF_LocalMemDesc* pLmDesc,
           acf_scenario_buffer_data* pSenarioBufferData,
           ICP_DATATYPE dataType, int e0x, int e0y,
           int pairedLmIndex,
           int padOffsetR, int padOffsetRefChunkWidth, int padOffsetB, int padOffsetRetChunkHeight,
           int memTypeFlag, //0=SCL, 1=VEC
           int verDepOverlapFlag)
{
   int lRetVal = 0;

   e0_desc e0;
   e0.dataType = dataType;
   e0.x = e0x;
   e0.y = e0y;
   int e0WidthInBytes = RetE0WidthInBytes(&e0);

   int16_t padN = pSenarioBufferData->padding.n;
   int16_t padS = pSenarioBufferData->padding.s;
   int16_t padE = pSenarioBufferData->padding.e;
   int16_t padW = pSenarioBufferData->padding.w;

   //these variables are introduced to handle the 'verDepOverlapFlag == 1' case
   int lBufferHeight = pSenarioBufferData->chunkHeight;
   int localPadN = padN;
   int localPadS = padS;
   if (1 == verDepOverlapFlag)
   {
      //from a memory allocation and buffering point of view, if (1 == verDepOverlapFlag) then the N and S
      //spatial dependencies become part of the buffer itself
      lBufferHeight = pSenarioBufferData->chunkHeight + padN + padS;
      localPadN = 0;
      localPadS = 0;
   }

   pLmDesc->e0.dataType = dataType;
   pLmDesc->e0.x        = e0x;
   pLmDesc->e0.y        = e0y;
   pLmDesc->padding.n   = padN;
   pLmDesc->padding.s   = padS;
   pLmDesc->padding.e   = padE;
   pLmDesc->padding.w   = padW;
   pLmDesc->pairedLmIndex = pairedLmIndex;
   pLmDesc->chunkWidth  = pSenarioBufferData->chunkWidth;
   pLmDesc->chunkHeight = pSenarioBufferData->chunkHeight;
   pLmDesc->numBuffers  = pSenarioBufferData->numBuffers;
   pLmDesc->chunkSpan   = (pLmDesc->chunkWidth + padE + padW)*e0WidthInBytes;
   pLmDesc->chunkE0Span = pLmDesc->chunkSpan*e0y;
   pLmDesc->bufferOffset = lBufferHeight*pLmDesc->chunkE0Span;
   pLmDesc->verDepOverlapFlag = verDepOverlapFlag;
   pLmDesc->verDepOverlapOffset = ((1 == verDepOverlapFlag) ? (padN*pLmDesc->chunkE0Span) : 0);
   pLmDesc->bufferHeight = lBufferHeight;
   pLmDesc->memTypeFlag = memTypeFlag;

   if (0 == memTypeFlag)
      pLmDesc->pTop = (void*)AllocFromSclHeap(pLmDesc->chunkE0Span*(lBufferHeight*pLmDesc->numBuffers + localPadN + localPadS));
   else
      pLmDesc->pTop = (void*)AllocFromVecHeap(pLmDesc->chunkE0Span*(lBufferHeight*pLmDesc->numBuffers + localPadN + localPadS));

   if (pLmDesc->pTop) //if memory allocation was successful
   {
      if (0 == memTypeFlag)
      {
         pLmDesc->pBuf0 = (void *)((int8_t*)pLmDesc->pTop + (pLmDesc->chunkE0Span*localPadN + e0WidthInBytes*padW));
         pLmDesc->pBot  = (void *)((int8_t*)pLmDesc->pTop + (pLmDesc->chunkE0Span*(localPadN + lBufferHeight*pLmDesc->numBuffers)));
      }
      else
      {
         pLmDesc->pBuf0 = (void *)((vec08s*)pLmDesc->pTop + (pLmDesc->chunkE0Span*localPadN + e0WidthInBytes*padW));
         pLmDesc->pBot  = (void *)((vec08s*)pLmDesc->pTop + (pLmDesc->chunkE0Span*(localPadN + lBufferHeight*pLmDesc->numBuffers)));
#if (1 == APEX_SIM_NATIVE)
         pLmDesc->logicalVecByteAddrBuf0 = ((intptr_t)pLmDesc->pBuf0 - ((intptr_t)RetVecHeapBaseAddr()))/sizeof(vec08s);
#else
         pLmDesc->logicalVecByteAddrBuf0 = (intptr_t)pLmDesc->pBuf0;
#endif
      }

      if (padE)
      {
         pLmDesc->padOffsetR = (pLmDesc->chunkWidth == padOffsetRefChunkWidth) ? (padOffsetR) : (pLmDesc->chunkWidth*padOffsetR/padOffsetRefChunkWidth);
      }

      if (padS)
      {
         pLmDesc->padOffsetB = (pLmDesc->chunkHeight == padOffsetRetChunkHeight) ? (padOffsetB) : (pLmDesc->chunkHeight*padOffsetB/padOffsetRetChunkHeight);
      }
   }
   else //memory allocation failure
   {
      //apex_printf("DEBUG ---> MEM ALLOC FAILED for %d bytes on %d stack\n", pLmDesc->chunkE0Span*(lBufferHeight*pLmDesc->numBuffers + localPadN + localPadS), memTypeFlag);
      lRetVal = 1;
   }

   return lRetVal;
}

void IoDescInit(kernel_io_desc* pIoDesc,
                ACF_LocalMemDesc* pLmDesc,
                int lmIndex)
{
   pIoDesc->chunkWidth = pLmDesc->chunkWidth;
   pIoDesc->chunkHeight = pLmDesc->chunkHeight;
   pIoDesc->chunkSpan = pLmDesc->chunkSpan;
   pIoDesc->_framework_lmindex = lmIndex;
   pIoDesc->_framework_cbindex = -1;
#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
   pIoDesc->pMem = (void*)((intptr_t)pLmDesc->pBuf0 + pLmDesc->verDepOverlapOffset - pLmDesc->bufferOffset);
#else
   //the following code would work for both paths, but is currently disabled in the non-sim path due to code size concerns
   if (0 == pLmDesc->memTypeFlag)
   {
      pIoDesc->pMem = (void*)((int8_t*)pLmDesc->pBuf0 + pLmDesc->verDepOverlapOffset - pLmDesc->bufferOffset);
   }
   else
   {
      if (icp::DATATYPE_08U == pLmDesc->e0.dataType || icp::DATATYPE_08S == pLmDesc->e0.dataType) //if 8-bit
         pIoDesc->pMem = (void*)((vec08s*)pLmDesc->pBuf0 + pLmDesc->verDepOverlapOffset - pLmDesc->bufferOffset);
      else if (icp::DATATYPE_16U == pLmDesc->e0.dataType || icp::DATATYPE_16S == pLmDesc->e0.dataType) //if 16-bit
         pIoDesc->pMem = (void*)((vec16s*)pLmDesc->pBuf0 + (pLmDesc->verDepOverlapOffset>>1) - (pLmDesc->bufferOffset>>1));
      else if (icp::DATATYPE_32U == pLmDesc->e0.dataType || icp::DATATYPE_32S == pLmDesc->e0.dataType) //if 32-bit
         pIoDesc->pMem = (void*)((vec32s*)pLmDesc->pBuf0 + (pLmDesc->verDepOverlapOffset>>2) - (pLmDesc->bufferOffset>>2));
   }
#endif
}

kernel_io_desc* RetIoDescScl_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   return RetIoDescScl(lpLmDesc, lpKernelIoDesc);
}

kernel_io_desc* RetIoDescVec08_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   return RetIoDescVec08(lpLmDesc, lpKernelIoDesc);
}

kernel_io_desc* RetIoDescVec16_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   return RetIoDescVec16(lpLmDesc, lpKernelIoDesc);
}

kernel_io_desc* RetIoDescVec32_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   return RetIoDescVec32(lpLmDesc, lpKernelIoDesc);
}

void LlineArrayForward(AcfCircularLineManager* lMan)
{
   for (int lIter = 0; lIter < lMan->lNumOfLines; lIter++)
   {
      lMan->pLldata[lIter].lineIndex++;
      if (lMan->pLldata[lIter].lineIndex == lMan->pLmDesc->chunkHeight)
      {
         lMan->pLldata[lIter].lineIndex = 0;
      }
      lMan->pLldata[lIter].linePointer = (int32_t)((intptr_t)lMan->pLmDesc->pBuf0) + lMan->pLmDesc->chunkSpan * lMan->pLldata[lIter].lineIndex;
   }
}

void LlineArrayForwardBuf(AcfCircularLineManager* lMan)
{
   for (int lIter = 0; lIter < lMan->lNumOfLines; lIter++)
   {
      lMan->pLldata[lIter].lineIndex++;
      if (lMan->pLldata[lIter].lineIndex == lMan->pLmDesc->numBuffers)
      {
         lMan->pLldata[lIter].lineIndex = 0;
      }
      lMan->pLldata[lIter].linePointer = (int32_t)((intptr_t)lMan->pLmDesc->pBuf0) + lMan->pLmDesc->bufferOffset * lMan->pLldata[lIter].lineIndex;
   }
}

#if (1 == APEX_SIM_NATIVE)
void SimLmInitAdjustment(ACF_LocalMemDesc* lpLmDescArray, int lNumLm, uint32_t lCmemHeapBaseAddr)
{
   //adjust pointers so they are to the right type of memory (on target all vector pointers refer to a single heap that can be cast to the various types,
   //but when using the native library this is not possible; vector buffers must be 'typed' properly since each is actually a separate memory space

   vec16s* lpVec16Heap = ((NativeCmem*)lCmemHeapBaseAddr)->mCmem16;
   vec32s* lpVec32Heap = ((NativeCmem*)lCmemHeapBaseAddr)->mCmem32;

   for (int i=0; i<lNumLm; i++)
   {
      ACF_LocalMemDesc* lpLmDesc = &lpLmDescArray[i];
      int e0WidthInBytes = RetE0WidthInBytes(&lpLmDesc->e0);
      int lBufferHeight = lpLmDesc->bufferHeight;
      int localPadN = (1 == lpLmDesc->verDepOverlapFlag) ? 0 : lpLmDesc->padding.n;
      int lPadW = lpLmDesc->padding.w;
      int lLogicalByteAddrBuf0 = lpLmDesc->logicalVecByteAddrBuf0;
      int lLogicalByteAddrTop = lLogicalByteAddrBuf0 - (lpLmDesc->chunkE0Span*localPadN + e0WidthInBytes*lPadW);

      if (1 == lpLmDesc->memTypeFlag) //if vector
      {
         if (icp::DATATYPE_16U == lpLmDesc->e0.dataType || icp::DATATYPE_16S == lpLmDesc->e0.dataType) //if 16-bit
         {
            lpLmDesc->pTop  = (void*)(lpVec16Heap + lLogicalByteAddrTop/2);
            lpLmDesc->pBuf0 = (void*)((vec16s*)lpLmDesc->pTop + ((lpLmDesc->chunkE0Span*localPadN + e0WidthInBytes*lPadW)/2));
            lpLmDesc->pBot  = (void*)((vec16s*)lpLmDesc->pTop + ((lpLmDesc->chunkE0Span*(localPadN + lBufferHeight*lpLmDesc->numBuffers))/2));
         }
         else if (icp::DATATYPE_32U == lpLmDesc->e0.dataType || icp::DATATYPE_32S == lpLmDesc->e0.dataType) //if 32-bit
         {
            lpLmDesc->pTop  = (void*)(lpVec32Heap + lLogicalByteAddrTop/4);
            lpLmDesc->pBuf0 = (void*)((vec32s*)lpLmDesc->pTop + ((lpLmDesc->chunkE0Span*localPadN + e0WidthInBytes*lPadW)/4));
            lpLmDesc->pBot  = (void*)((vec32s*)lpLmDesc->pTop + ((lpLmDesc->chunkE0Span*(localPadN + lBufferHeight*lpLmDesc->numBuffers))/4));
         }
      }
   }
}
#endif

#if (1 == APEX_SIM_ISS || 1 == APEX_SIM_NATIVE)
void AdvanceSimulation(int lApexId) {SequencerStep(lApexId);}
#endif

#ifndef CGV_FORCE_INLINE
#include "acf_utilities_inline.h"
#endif











#if 0

//Started work on intercu padding stuff but it is being shelved for now.
//Should work resume at some point, the code below was quickly written and not tested so
//review prior to even starting to work with it!!!

void
acf_intercu_pad_lr (int lChunkWidth,
                    int lChunkHeight,
                    int lE0WidthInBytes,
                    int lPadLeftSize,
                    int lPadRightSize,
                    vec08u* lvpTopLeft,
                    vec08u* lvpTopRight,
                    vec08u* lvpMaskLeft,
                    vec08u* lvpMaskRight)
{
   int lChunkSpan = lChunkWidth*lE0WidthInBytes;

   //left padding
   for (int i=0; i<lChunkSpan*lChunkHeight; i+=lChunkSpan)
   {
      for (int j=0; j<lPadLeftSize; j++)
      {
         for (int k=0; k<lE0WidthInBytes; k++)
         {
            vec08u vTmp = vmrl(lvpTopLeft[i+k]);
            vif (lvpMaskLeft[0] == 1)
               lvpTopRight[i - j*lE0WidthInBytes + k] = vTmp;
         }
      }
   }

   //right padding
   for (int i=0; i<llChunkSpan*lChunkHeight; i+=lChunkSpan)
   {
      for (int j=0; j<lPadRightSize; j++)
      {
         for (int k=0; k<lE0WidthInBytes; k++)
         {
            vec08u vTmp = vmrr(lvpTopRight[i+k]);
            vif (lvpMaskRight[0] == 1)
               lvpTopLeft[i+j*lE0WidthInBytes+k] = vTmp;
         }
      }
   }
}

void
acf_intercu_pad_tb(int lChunkWidth,
                   int lE0WidthInBytes,
                   int lPadSize,
                   vec08u* lvpPad,
                   vec08u* lvpPadSrc)
{
   int lChunkSpan = lChunkWidth*lE0WidthInBytes;

   for (int i=0; i<lPadSize; i++)
   {
      for (int j=0; j<lChunkSpan; j++)
      {
         lvpPad[i*lChunkSpan+j] = lvpPadSrc[j];
      }
   }
}


void ACF_PAD_INTERCU(ACF_LocalMemDesc& lLM,
                     int lIter,
                     int lNumIterations,
                     int lTileWidthInChunks,
                     int lPadTop,
                     int lPadBot,
                     int lPadLeft,
                     int lPadRight)
{
   const int lCuArrayWidth = 64; //TODO: need to get this from CMEM IF registers

   //generate masks
   vec08u lvMaskLeft, lvMaskRight;
   lvMaskLeft  = 0;
   lvMaskRight = 0;
   cmem_write(&lvMaskLeft,  lCuArrayWidth-1,    1); //'enabled' CU is last CU in array (wrap around)
   cmem_write(&lvMaskRight, lTileWidthInChunks, 1); //'enabled' CU is the CU to the right of the tile

   int lCurrentBuffer = RetBufferIndex(&lLM, lIter);
   int lE0WidthInBytes = RetE0WidthInBytes(&(lLM.e0));

   //top border
   if ((lPadTop>0) && (lIter == 0))
   {
      //pad top
      vec08u* lpvSrc = (vec08s*)(lLM.pBuf0) + lLM.chunkSpan*lLM.chunkHeight*lCurrentBuffer;
      vec08u* lpvDst = lpvSrc - lLM.chunkSpan*lPadTop;
      acf_intercu_pad_tb(lLM.chunkWidth, lE0WidthInBytes, lPadTop, lpvDst, lpvSrc);

      //pad top left and right
      if (lPadLeft > 0 || lPadRight > 0)
      {
         vec08u* lvpTopLeft = lpvDst;
         vec08u* lvpTopRight = lvpTopLeft + lLM.chunkSpan - lE0WidthInBytes;
         acf_intercu_pad_lr(lLM.chunkWidth, lPadTop, lE0WidthInBytes, lPadLeft, lPadRight,
                            lvpTopLeft, lvpTopRight, &lvMaskLeft, &lvMaskRight);
      }
   }

   //left and right border
   if (lPadLeft > 0 || lPadRight > 0)
   {
      vec08u* lvpTopLeft = (vec08s*)(lLM.pBuf0) + lLM.chunkSpan*lLM.chunkHeight*lCurrentBuffer;
      vec08u* lvpTopRight = lvpTopLeft + lLM.chunkSpan - lE0WidthInBytes;
      acf_intercu_pad_lr(lLM.chunkWidth, lLM.chunkHeight, lE0WidthInBytes, lPadLeft, lPadRight,
                         lvpTopLeft, lvpTopRight, &lvMaskLeft, &lvMaskRight);
   }

   //bottom border
   if ((lPadBot>0) && (lIter == lNumIterations-1))
   {
      //pad bottom
      vec08u* lpvSrc = (vec08s*)(lLM.pBuf0) + lLM.chunkSpan*(lLM.chunkHeight*(lCurrentBuffer+1)-1);
      vec08u* lpvDst = lpvSrc + lLM.chunkSpan;
      acf_intercu_pad_tb(lLM.chunkWidth, lE0WidthInBytes, lPadTop, lpvDst, lpvSrc);

      //pad bottom left and right
      if (lPadLeft > 0 || lPadRight > 0)
      {
         vec08u* lvpTopLeft = lpvDst;
         vec08u* lvpTopRight = lvpTopLeft + lLM.chunkSpan - lE0WidthInBytes;
         acf_intercu_pad_lr(lLM.chunkWidth, lPadBot, lE0WidthInBytes, lPadLeft, lPadRight,
                            lvpTopLeft, lvpTopRight, &lvMaskLeft, &lvMaskRight);
      }
   }
}
#endif
