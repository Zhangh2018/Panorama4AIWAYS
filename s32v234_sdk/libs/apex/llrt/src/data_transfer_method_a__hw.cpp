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

#include <stdint.h>
#include <stdio.h>
#include <dma2d_hal.h>
#include <apu_mem_helper.hpp>
#include <circ_tile_buf.hpp> //for MEM_TYPE
#include <vector_map_desc.hpp>

//current dummy implementation uses an intermediate DMEM buffer plus APU reorg to
//enable flexible stream dma like data transfers (this is for testing/enablement purposes
//and is extremely inefficient).  for example, for the XMEM->CMEM case, the DMA will be used
//to transfer the input data from XMEM to DMEM, then the APU will copy the data from DMEM to CMEM.
   
//intermediate DMEM buffer
static int8_t gsDmemTmpBuf[256];

void CU_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
#if 1
   uintptr_t lSrcAddrPhys = (uintptr_t)lpSrc;
   uintptr_t lDstAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)gsDmemTmpBuf);
   if (MEM_TYPE_DMEM_SCL == lSrcMemType)
      lSrcAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)lpSrc);

   volatile APEXD_DMA2D_CH lDma2dCh;
   
   lDma2dCh.id = 0;
   lDma2dCh.wrCmdQueueId = 0;

   lDma2dCh.context.Cfg.asBitField.SrcType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.DstType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.SrcWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.SrcBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DstWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.DstBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DontWait      = 0;

   lDma2dCh.context.SrcDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.SrcDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.SrcAddr.as32BitValue           = lSrcAddrPhys;
   lDma2dCh.context.SrcSpan.asBitField.Span        = 0; //don't care (1D for now)

   lDma2dCh.context.DstDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.DstDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.DstAddr.as32BitValue           = lDstAddrPhys;
   lDma2dCh.context.DstSpan.asBitField.Span        = 0;

   dma2d_hal_ch_push(lDma2dCh);
   dma2d_hal_start(lDma2dCh);
   dma2d_hal_wait(lDma2dCh);
   
   for (int32_t i = 0; i < lSize; i++)
   {
      lpDst[i] = vput(lpDst[i], gsDmemTmpBuf[i], lDstVecIdx);
   }
#endif
}

void CU_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   CU_VEC_WRITE_8((int8_t*)lpSrc, (vec08s*)lpDst, lSrcMemType, lDstVecIdx, lSize * 2);
}

void CU_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   CU_VEC_WRITE_8((int8_t*)lpSrc, (vec08s*)lpDst, lSrcMemType, lDstVecIdx, lSize * 4);
}

void CU_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize)
{
#if 1
   for (int32_t i = 0; i < lSize; i++)
   {
      gsDmemTmpBuf[i] = vget(lpSrc[i], lSrcVecIdx);
   }
   
   uintptr_t lSrcAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)gsDmemTmpBuf);
   uintptr_t lDstAddrPhys = (uintptr_t)lpDst;
   if (MEM_TYPE_DMEM_SCL == lDstMemType)
      lDstAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)lpDst);

   volatile APEXD_DMA2D_CH lDma2dCh;
   
   lDma2dCh.id = 0;
   lDma2dCh.wrCmdQueueId = 0;

   lDma2dCh.context.Cfg.asBitField.SrcType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.DstType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.SrcWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.SrcBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DstWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.DstBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DontWait      = 0;

   lDma2dCh.context.SrcDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.SrcDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.SrcAddr.as32BitValue           = lSrcAddrPhys;
   lDma2dCh.context.SrcSpan.asBitField.Span        = 0; //don't care (1D for now)

   lDma2dCh.context.DstDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.DstDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.DstAddr.as32BitValue           = lDstAddrPhys;
   lDma2dCh.context.DstSpan.asBitField.Span        = 0;

   dma2d_hal_ch_push(lDma2dCh);
   dma2d_hal_start(lDma2dCh);
   dma2d_hal_wait(lDma2dCh);
#endif
}

void CU_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize)
{
   CU_VEC_READ_8((vec08s*)lpSrc, (int8_t*)lpDst, lSrcVecIdx, lDstMemType, lSize * 2);
}

void CU_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx,  MEM_TYPE lDstMemType, int32_t lSize)
{
   CU_VEC_READ_8((vec08s*)lpSrc, (int8_t*)lpDst, lSrcVecIdx, lDstMemType, lSize * 4);
}

void CU_VEC_READ_WRITE_8(vec08s* lpSrc, vec08s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
   {
      int8_t lTmp = vget(lpSrc[i], lVecIdx);
      lpDst[i] = vput(lpDst[i], lTmp, lVecIdx);
   }
}

void CU_VEC_READ_WRITE_16(vec16s* lpSrc, vec16s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   CU_VEC_READ_WRITE_8((vec08s*)lpSrc, (vec08s*)lpDst, lVecIdx, lSize * 2);
}

void CU_VEC_READ_WRITE_32(vec32s* lpSrc, vec32s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   CU_VEC_READ_WRITE_8((vec08s*)lpSrc, (vec08s*)lpDst, lVecIdx, lSize * 4);
}

void SCL_XFER_8(int8_t* lpSrc, int8_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
#if 1
   uintptr_t lSrcAddrPhys = (uintptr_t)lpSrc;
   uintptr_t lDstAddrPhys = (uintptr_t)lpDst;
   if (MEM_TYPE_DMEM_SCL == lSrcMemType)
      lSrcAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)lpSrc);
   if (MEM_TYPE_DMEM_SCL == lDstMemType)
      lDstAddrPhys = ApuMemHelper_RetDmemPhys((uintptr_t)lpDst);

   volatile APEXD_DMA2D_CH lDma2dCh;

   lDma2dCh.id = 0;
   lDma2dCh.wrCmdQueueId = 0;

   lDma2dCh.context.Cfg.asBitField.SrcType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.DstType       = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_TYPE_ID_MEMORY;
   lDma2dCh.context.Cfg.asBitField.SrcWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_SRC_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.SrcBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DstWordSz     = APEX_DMA2D_WR_CMD_QUEUE_0_CHANNEL_CFG_BIT_DST_WORD_SZ_ID_BYTE;
   lDma2dCh.context.Cfg.asBitField.DstBeatLength = INCR1;
   lDma2dCh.context.Cfg.asBitField.DontWait      = 0;

   lDma2dCh.context.SrcDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.SrcDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.SrcAddr.as32BitValue           = lSrcAddrPhys;
   lDma2dCh.context.SrcSpan.asBitField.Span        = 0; //don't care (1D for now)

   lDma2dCh.context.DstDimension.asBitField.Width  = lSize-1;
   lDma2dCh.context.DstDimension.asBitField.Height = 0; // if Height is zero, 1D transfer otherwise 2D transfer  
   lDma2dCh.context.DstAddr.as32BitValue           = lDstAddrPhys;
   lDma2dCh.context.DstSpan.asBitField.Span        = 0;

   dma2d_hal_ch_push(lDma2dCh);
   dma2d_hal_start(lDma2dCh);
   dma2d_hal_wait(lDma2dCh);
#endif
}

void SCL_XFER_16(int16_t* lpSrc, int16_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   SCL_XFER_8((int8_t*)lpSrc, (int8_t*)lpDst, lSrcMemType, lDstMemType, lSize * 2);
}

void SCL_XFER_32(int32_t* lpSrc, int32_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   SCL_XFER_8((int8_t*)lpSrc, (int8_t*)lpDst, lSrcMemType, lDstMemType, lSize * 4);
}

void GOC_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}

void GOC_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}

void GOC_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}

void GOC_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}

void GOC_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}

void GOC_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented yet - probably same as iss version due to the layering
}
