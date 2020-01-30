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

#ifndef ACFCBMANAGER_H
#define ACFCBMANAGER_H

#include <stdint.h>
#include <acf_common.h>

class CbManager
{
public:

   //NOTES: - in all cases below, lines refers to 'lines of e0'
   //       - REQUIREMENT: mpLmDesc is allocated with enough bottom padding to handle worst case overflow (i.e. underflow will be impossible);
   //         this means the number of extra lines at the bottom must be equal to MAX(worst case bottom spatial dependency, chunkheight-1)

   void Init(ACF_LocalMemDesc* lpLmDesc, int16_t llWorstCaseSpatialDepBot, int16_t lTileWidthInChunks, int16_t lCuArrayWidth);
   void WrapOverflow(int16_t lOverflow);
   int16_t RetNextIdxForProd();
   void LinesProduced(int16_t lNumLinesProduced, int16_t lLastFlag);
   void LinesProducedOd(int16_t lNumLinesProduced, int16_t lLastFlag);

   ACF_LocalMemDesc* mpLmDesc; //pointer to the memory region this will be managing
   int16_t mCoreRegLines;      //number of 'core' region lines
   int16_t mExtraLinesTop;     //number of lines present to accommodate worst case top padding
   int16_t mExtraLinesBot;     //number of lines present to accommodate worst case bottom padding and/or worst case overflow (whichever is bigger)
   int16_t mSpatialDepBot;     //the actual bottom spatial dependency requirements for copy down purposes; mSpatialDepBot must be <= mExtraLinesBot
   int16_t mProdMarker;
   int16_t mNextProdMarker;
   int16_t mWrapIdx;
   int16_t mFirstProdFlag;     //initialized to 1; will be set to 0 after first non-zero production has occurred
   int16_t mPadTopFlag;        //initialized to 1; used to determine if top padding needs to be performed
   int16_t mTileWidthInChunks;
   int16_t mCuArrayWidth;

   //information relevant to performing the circular buffering
   //up
   int16_t mNumLinesToBeCopiedUp; //this will always take place in one shot
   int16_t mCopyUpSrcIdx;
   int16_t mCopyUpDstIdx;

   //down
   int16_t mCopyDownDeficit;
   int16_t mNumLinesToBeCopiedDown; //***may not be satisfied in one shot (depends on the size of the deficit and the production details)
   int16_t mCopyDownSrcIdx;
   int16_t mCopyDownDstIdx;
};

#endif /* ACFCBMANAGER_H */
