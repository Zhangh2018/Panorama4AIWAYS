/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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

#include <stdio.h>
#include <acf_cbmanager.hpp>
#include <acf_common.h>

#define ENABLE_PADDING_AND_COPIES 1

#if (ENABLE_PADDING_AND_COPIES == 1)
#include <apu_test_util.h> //padding
void cmem_write (vec16u __cmem* dst, int cu, int16u val);
#endif

void CbManager::Init(ACF_LocalMemDesc* lpLmDesc, int16_t llWorstCaseSpatialDepBot, int16_t lTileWidthInChunks, int16_t lCuArrayWidth)
{
   mpLmDesc           = lpLmDesc;
   mCoreRegLines      = lpLmDesc->chunkHeight*lpLmDesc->numBuffers;
   mExtraLinesTop     = lpLmDesc->padding.n;
   mExtraLinesBot     = lpLmDesc->padding.s;
   mSpatialDepBot     = llWorstCaseSpatialDepBot;
   mProdMarker        = 0;
   mNextProdMarker    = 0;
   mWrapIdx           = 0;
   mCopyDownDeficit   = 0;
   mCopyDownDstIdx    = 0;
   mFirstProdFlag     = 1;
   mPadTopFlag        = 1;
   mTileWidthInChunks = lTileWidthInChunks;
   mCuArrayWidth      = lCuArrayWidth;
}

void CbManager::WrapOverflow(int16_t lOverflow)
{
   //assume that all the necessary lines will be available in one shot for the copy up
   mNumLinesToBeCopiedUp = mExtraLinesTop + lOverflow;

   if (mNumLinesToBeCopiedUp)
   {
      mNumLinesToBeCopiedUp = mNumLinesToBeCopiedUp;
      mCopyUpSrcIdx         = lOverflow + mCoreRegLines - mNumLinesToBeCopiedUp;
      mCopyUpDstIdx         = -mExtraLinesTop;
    
      //printf("CB_UP(%d lines from %d to %d)\n", mNumLinesToBeCopiedUp, mCopyUpSrcIdx, mCopyUpDstIdx);
#if (ENABLE_PADDING_AND_COPIES == 1)
      //CB_UP
      int16_t lE0WidthInBytes = RetE0WidthInBytes(&mpLmDesc->e0);
      vec08u* lpSrc = (vec08u*)(mpLmDesc->pBuf0) - (lE0WidthInBytes*mpLmDesc->padding.w) + mpLmDesc->chunkE0Span*mCopyUpSrcIdx;
      vec08u* lpDst = (vec08u*)(mpLmDesc->pBuf0) - (lE0WidthInBytes*mpLmDesc->padding.w) + mpLmDesc->chunkE0Span*mCopyUpDstIdx;
      apu_til_copy (lpDst, mpLmDesc->chunkSpan,
                    lpSrc, mpLmDesc->chunkSpan,
                    mNumLinesToBeCopiedUp*mpLmDesc->e0.y,
                    mpLmDesc->chunkSpan,
                    mNumLinesToBeCopiedUp*mpLmDesc->e0.y);
#endif
   }

   //set the copy down deficit (deficit will be 'paid' by LinesProduced)
   mCopyDownDeficit = mSpatialDepBot - lOverflow;
   mCopyDownDstIdx = mCoreRegLines + lOverflow;
}

int16_t CbManager::RetNextIdxForProd()
{
   return mProdMarker; //return the 'current' producer marker
}

void CbManager::LinesProducedOd(int16_t lNumLinesProduced, int16_t lLastFlag)
{
   //In the OD case, the producer will always write 'chunkheight' lines worth of data to the buffer it receives.
   //However, the first time lNumLinesProduced is non-zero, only the last 'lNumLinesProduced' lines will be considered 'valid'.
   //To realign, copy the valid lNumLinesProduced lines to the top of the buffer so they are aligned with first core
   //buffer starting point.  This adjustment/copy only needs to take place once.
   if (lNumLinesProduced > 0)
   {
      if (mFirstProdFlag && (lNumLinesProduced < mpLmDesc->chunkHeight))
      {
#if (ENABLE_PADDING_AND_COPIES == 1)
         //perform copy up to realign
         int16_t lE0WidthInBytes = RetE0WidthInBytes(&mpLmDesc->e0);
         vec08u* lpSrc = (vec08u*)(mpLmDesc->pBuf0) + (mpLmDesc->chunkHeight - lNumLinesProduced)*mpLmDesc->chunkE0Span;
         vec08u* lpDst = (vec08u*)(mpLmDesc->pBuf0);
         apu_til_copy (lpDst, mpLmDesc->chunkSpan,
                       lpSrc, mpLmDesc->chunkSpan,
                       lNumLinesProduced*mpLmDesc->e0.y,
                       mpLmDesc->chunkWidth*lE0WidthInBytes,
                       lNumLinesProduced*mpLmDesc->e0.y);
#endif
      }

      mFirstProdFlag = 0;
      
      LinesProduced(lNumLinesProduced, lLastFlag);  
   } 
}

void CbManager::LinesProduced(int16_t lNumLinesProduced, int16_t lLastFlag)
{
   //printf("prod %d @ buf idx = %d\n", lNumLinesProduced, mProdMarker);
   
#if (ENABLE_PADDING_AND_COPIES == 1)
   //-----------------------------------------------------
   //PADDING (perform padding on the newly produced lines)
   //-----------------------------------------------------
   
   //TODO: enable 'edge case helpers' below (need lTileWidthInChunks, lPadChOffsetRight, lPadChOffsetBot, lCuArrayWidth)
   
   vec08u* lpSrcTopLeft = (vec08u*)(mpLmDesc->pBuf0) + mpLmDesc->chunkE0Span*mProdMarker;
   int16_t lE0WidthInBytes = RetE0WidthInBytes(&mpLmDesc->e0);
   
   //LEFT & RIGHT:------------------------------------------
   //NOTE: currently assuming worst case all the time to avoid pulling in too much code (i.e. assume 08u, e0xnot1, lnotequaltoR)
   
   if (mpLmDesc->padding.e || mpLmDesc->padding.w)
   {
      apu_til_padhor_08_e0x(lpSrcTopLeft,
                            mpLmDesc->chunkSpan,
                            mpLmDesc->chunkWidth,
                            lNumLinesProduced,
                            lE0WidthInBytes,
                            mpLmDesc->padding.w,
                            mpLmDesc->padding.e);

      //Edge cases where not all CUs are used or if the full block is not used (from PadEdgeCaseHelper)
      if (mTileWidthInChunks != mCuArrayWidth)
      {
         vec16u lvMask = 0;
         cmem_write((vec16u __cmem*) &lvMask, mTileWidthInChunks-1, 1);
         vec08u* dst = lpSrcTopLeft + lE0WidthInBytes*mpLmDesc->chunkWidth;
         vec08u* src = dst - lE0WidthInBytes;
         apu_til_copy_right(dst, mpLmDesc->chunkSpan, src, mpLmDesc->chunkSpan, lE0WidthInBytes, lE0WidthInBytes*mpLmDesc->padding.e, lNumLinesProduced*mpLmDesc->e0.y, lvMask);
      }
   }
   
   //TOP--------------------------------------------------
   
   if (mPadTopFlag && mpLmDesc->padding.n && lNumLinesProduced)
   {
      vec08u* lpSrc = lpSrcTopLeft - (lE0WidthInBytes*mpLmDesc->padding.w);
      vec08u* lpDst = lpSrc - mpLmDesc->padding.n*mpLmDesc->chunkE0Span;
      apu_til_copy(lpDst, mpLmDesc->chunkSpan, lpSrc, mpLmDesc->chunkSpan, mpLmDesc->e0.y, mpLmDesc->chunkSpan, mpLmDesc->padding.n*mpLmDesc->e0.y);
      mPadTopFlag = 0; //set to 0 so top padding is only done once (i.e. the first time lines are produced)
   }
   
   //BOTTOM-----------------------------------------------
   
   if (lLastFlag && mpLmDesc->padding.s)
   {
      int16_t lPadChOffsetBot = 0; //TODO: fill this in!!!
      vec08u* lpDst = lpSrcTopLeft + (lNumLinesProduced - lPadChOffsetBot)*mpLmDesc->chunkE0Span - (lE0WidthInBytes*mpLmDesc->padding.w);
      vec08u* lpSrc = lpDst - mpLmDesc->chunkE0Span;
      apu_til_copy(lpDst, mpLmDesc->chunkSpan, lpSrc, mpLmDesc->chunkSpan, mpLmDesc->e0.y, mpLmDesc->chunkSpan,  mpLmDesc->padding.s*mpLmDesc->e0.y); 
   }
#endif

   //--------------------------------------------
   //HANDLE WRAP AROUND AND CIRCULAR BUFFERING UP
   //--------------------------------------------

   //HANDLE CORE REGION OVERFLOW: if producer fills the core region exactly (overflow of 0) or if it overflows into the padding region but still fits
   int16_t lTmpIdx = mProdMarker + lNumLinesProduced;
   if ((lTmpIdx == mCoreRegLines) ||
      (lTmpIdx > mCoreRegLines && lTmpIdx <= (mCoreRegLines + mExtraLinesBot)))
   {
      int16_t lOverflow = (mProdMarker + lNumLinesProduced) - (mCoreRegLines);
      mNextProdMarker = lOverflow; //next one wraps around
      WrapOverflow(lOverflow);
   }
   //NO CORE REGION OVERFLOW: just increment by the number of lines produced
   else
   {
      mNextProdMarker = mProdMarker + lNumLinesProduced;
   }

   //--------------------------------------------
   //HANDLE COPY DOWN DEFICIT
   //--------------------------------------------

   //if there is a copy down deficit and we are producing AFTER having wrapped around
   if ((mCopyDownDeficit > 0) && (mProdMarker < mNextProdMarker))
   {
      mCopyDownSrcIdx = mProdMarker;
      
      if (lNumLinesProduced >= mCopyDownDeficit) //number of lines produced fully satisfies (or exceeds) outstanding deficit 
      {
         mNumLinesToBeCopiedDown = mCopyDownDeficit; 
      }
      else //number of lines produced does not fully satisfy outstanding deficit
      {  
         mNumLinesToBeCopiedDown = lNumLinesProduced;
      }
      
      //printf("CB_DOWN(%d<%d> lines from %d to %d)\n", mNumLinesToBeCopiedDown, mCopyDownDeficit, mCopyDownSrcIdx, mCopyDownDstIdx);
 #if (ENABLE_PADDING_AND_COPIES == 1)
      //CB_DOWN
      int16_t lE0WidthInBytes = RetE0WidthInBytes(&mpLmDesc->e0);
      vec08u* lpSrc = (vec08u*)(mpLmDesc->pBuf0) - (lE0WidthInBytes*mpLmDesc->padding.w) + mpLmDesc->chunkE0Span*mCopyDownSrcIdx;
      vec08u* lpDst = (vec08u*)(mpLmDesc->pBuf0) - (lE0WidthInBytes*mpLmDesc->padding.w) + mpLmDesc->chunkE0Span*mCopyDownDstIdx;
      apu_til_copy (lpDst, mpLmDesc->chunkSpan,
                    lpSrc, mpLmDesc->chunkSpan,
                    mNumLinesToBeCopiedDown*mpLmDesc->e0.y,
                    mpLmDesc->chunkSpan,
                    mNumLinesToBeCopiedDown*mpLmDesc->e0.y);
#endif

      mCopyDownDeficit -= mNumLinesToBeCopiedDown;
      mCopyDownDstIdx += mNumLinesToBeCopiedDown;
   }

   mProdMarker = mNextProdMarker;
}
