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

#ifndef ACFODPRODPATTERN_H
#define ACFODPRODPATTERN_H

#include <stdint.h>

//*********************************************
//production pattern for output delay
//*********************************************

class OdProdPattern
{
public:

   void Init(int16_t lOutputDelayInLines, int16_t lChunkHeight)
   {
      mChunkHeight = lChunkHeight;
      mOutputDelayModChunkHeight = lOutputDelayInLines%lChunkHeight;
      mOutputDelayDivChunkHeight = lOutputDelayInLines/lChunkHeight;
   }

   int16_t CalcNumLinesProduced(int16_t lIter, int16_t lLastIterFlag)
   {
      int16_t lLinesProd = 0;

      if (lIter < mOutputDelayDivChunkHeight)
      {
         lLinesProd = 0;
      }
      else if (lIter == mOutputDelayDivChunkHeight)
      {
         lLinesProd = mChunkHeight - mOutputDelayModChunkHeight;
      }
      else if (lLastIterFlag) //final invocation
      {
         if (0 == mOutputDelayModChunkHeight)
         {
            lLinesProd = mChunkHeight;
         }
         else
         {
            lLinesProd = mOutputDelayModChunkHeight;
         }
      }
      else //((lIter > OD/TILE_HEIGHT) && (lIter != (LAST_ITER)))
      {
         lLinesProd = mChunkHeight;
      }

      return lLinesProd;
   }

   int16_t mOutputDelayModChunkHeight;
   int16_t mOutputDelayDivChunkHeight;
   int16_t mChunkHeight;
};

#endif /* ACFODPRODPATTERN_H */