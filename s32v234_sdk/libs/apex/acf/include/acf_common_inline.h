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

#ifndef ACFCOMMONINLINE_H
#define ACFCOMMONINLINE_H

CGV_INLINE_PREFIX int RetLocalMemDescSizeInBytes(ACF_LocalMemDesc* lpLM)
{
   int32_t lBufSizeInBytes = 0;
   if (1 == lpLM->verDepOverlapFlag)
      lBufSizeInBytes = lpLM->chunkE0Span*((lpLM->chunkHeight + lpLM->padding.n + lpLM->padding.s)*lpLM->numBuffers);
   else
      lBufSizeInBytes = lpLM->chunkE0Span*(lpLM->chunkHeight*lpLM->numBuffers + lpLM->padding.n + lpLM->padding.s);

   return lBufSizeInBytes;
}

//the use of the function has been reduced and is on its way to being eliminated
//due to the use of modulus in some cases (currently only called by RetAddr_iter(), which
//is called by ACF_CollectVarLengthScatteredLists() and various linked list generation functions)
CGV_INLINE_PREFIX int RetBufferIndex(ACF_LocalMemDesc* lpLM, int lIter)
{
   int lNumBuffers = lpLM->numBuffers;
   int lMask = lNumBuffers-1; //lNumBuffers must be >= 1
   if((lNumBuffers&lMask) == 0) //if lNumBuffers is a power of 2
      return lIter&lMask;
   else
      return lIter%lNumBuffers; //SLOW - need an alternative for this!!!
}

CGV_INLINE_PREFIX void CbIndexIncrement(int16_t* lpIndex, uint16_t lNumBuffers)
{
   if (++(*lpIndex) == lNumBuffers)
      *lpIndex = 0;
}

#endif /* ACFCOMMONINLINE_H */
