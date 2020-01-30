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

//NOTE: these functions assume:
// - lpKernelIoDesc->pMem has been initialized to (lpLmDesc->pBuf0 - lpLmDesc->bufferOffset)
// - lpKernelIoDesc->_framework_cbindex has been initialized to -1
//so there is no difference between the first time this is called and subsequent times

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescScl(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   lpKernelIoDesc->pMem = (void*)((int8_t*)lpKernelIoDesc->pMem + lpLmDesc->bufferOffset);
   lpKernelIoDesc->_framework_cbindex ++;
   if (lpKernelIoDesc->_framework_cbindex >= lpLmDesc->numBuffers)
   {
      lpKernelIoDesc->pMem = (void*)((int8_t*)lpLmDesc->pBuf0 + lpLmDesc->verDepOverlapOffset);
      lpKernelIoDesc->_framework_cbindex = 0;
   }
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec08(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   lpKernelIoDesc->pMem = (void*)((vec08s*)lpKernelIoDesc->pMem + lpLmDesc->bufferOffset);
   lpKernelIoDesc->_framework_cbindex ++;
   if (lpKernelIoDesc->_framework_cbindex >= lpLmDesc->numBuffers)
   {
      lpKernelIoDesc->pMem = (void*)((vec08s*)lpLmDesc->pBuf0 + lpLmDesc->verDepOverlapOffset);
      lpKernelIoDesc->_framework_cbindex = 0;
   }
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec16(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   lpKernelIoDesc->pMem = (void*)((vec16s*)lpKernelIoDesc->pMem + (lpLmDesc->bufferOffset>>1));
   lpKernelIoDesc->_framework_cbindex ++;
   if (lpKernelIoDesc->_framework_cbindex >= lpLmDesc->numBuffers)
   {
      lpKernelIoDesc->pMem = (void*)((vec16s*)lpLmDesc->pBuf0 + (lpLmDesc->verDepOverlapOffset>>1));
      lpKernelIoDesc->_framework_cbindex = 0;
   }
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec32(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc)
{
   lpKernelIoDesc->pMem = (void*)((vec32s*)lpKernelIoDesc->pMem + (lpLmDesc->bufferOffset>>2));
   lpKernelIoDesc->_framework_cbindex ++;
   if (lpKernelIoDesc->_framework_cbindex >= lpLmDesc->numBuffers)
   {
      lpKernelIoDesc->pMem = (void*)((vec32s*)lpLmDesc->pBuf0 + (lpLmDesc->verDepOverlapOffset>>2));
      lpKernelIoDesc->_framework_cbindex = 0;
   }
   return lpKernelIoDesc;
}

//producer buffer access pattern for buffers managed with the CbManager (currently only used in the producer output delay case)
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescScl(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc)
{
   int lBufLineIdx = lpCbManager->RetNextIdxForProd(); //this may be negative!
   ACF_LocalMemDesc* lpLmDesc = lpCbManager->mpLmDesc;
   lpKernelIoDesc->pMem = (void*)((int8_t*)lpLmDesc->pBuf0 + lBufLineIdx*lpLmDesc->chunkE0Span);
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec08(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc)
{
   int lBufLineIdx = lpCbManager->RetNextIdxForProd(); //this may be negative!
   ACF_LocalMemDesc* lpLmDesc = lpCbManager->mpLmDesc;
   lpKernelIoDesc->pMem = (void*)((vec08s*)lpLmDesc->pBuf0 + lBufLineIdx*lpLmDesc->chunkE0Span);
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec16(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc)
{
   int lBufLineIdx = lpCbManager->RetNextIdxForProd(); //this may be negative!
   ACF_LocalMemDesc* lpLmDesc = lpCbManager->mpLmDesc;
   lpKernelIoDesc->pMem = (void*)((vec16s*)lpLmDesc->pBuf0 + lBufLineIdx*(lpLmDesc->chunkE0Span>>1));
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec32(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc)
{
   int lBufLineIdx = lpCbManager->RetNextIdxForProd(); //this may be negative!
   ACF_LocalMemDesc* lpLmDesc = lpCbManager->mpLmDesc;
   lpKernelIoDesc->pMem = (void*)((vec32s*)lpLmDesc->pBuf0 + lBufLineIdx*(lpLmDesc->chunkE0Span>>2));
   return lpKernelIoDesc;
}

CGV_INLINE_PREFIX void ACF_UpdateTileFlags(int lIter, int lNumIter)
{
   gsACF_FIRST_TILE_FLAG = 0;
   gsACF_LAST_TILE_FLAG = 0;
   if (lIter == 0)
      gsACF_FIRST_TILE_FLAG = 1;
   if (lIter == lNumIter-1)
      gsACF_LAST_TILE_FLAG = 1;
}

CGV_INLINE_PREFIX void* RetAddrScl_iter(ACF_LocalMemDesc* lpLM, int lIter)
{
   return (void*)((int8_t*)lpLM->pBuf0 + RetBufferIndex(lpLM, lIter)*(lpLM->bufferOffset));
}

CGV_INLINE_PREFIX void* RetAddrVec08_iter(ACF_LocalMemDesc* lpLM, int lIter)
{
   return (void*)((vec08s*)lpLM->pBuf0 + RetBufferIndex(lpLM, lIter)*(lpLM->bufferOffset));
}

CGV_INLINE_PREFIX void* RetAddrVec16_iter(ACF_LocalMemDesc* lpLM, int lIter)
{
   return (void*)((vec16s*)lpLM->pBuf0 + RetBufferIndex(lpLM, lIter)*(lpLM->bufferOffset>>1));
}

CGV_INLINE_PREFIX void* RetAddrVec32_iter(ACF_LocalMemDesc* lpLM, int lIter)
{
   return (void*)((vec32s*)lpLM->pBuf0 + RetBufferIndex(lpLM, lIter)*(lpLM->bufferOffset>>2));
}
