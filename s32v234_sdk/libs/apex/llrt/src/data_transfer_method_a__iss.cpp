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
#include <circ_tile_buf.hpp> //for MEM_TYPE
#include <vector_map_desc.hpp>
#include <apex_transfer_api.h> //for xim

//for test/debug use case where IO are in APEX mem instead of 'XMEM'
MEM_TYPE RetSclMemTypeBasedOnAddrMangling(uintptr_t lSclAddr)
{
   return ((lSclAddr & 0xC0000000) ? MEM_TYPE_XMEM_SCL : MEM_TYPE_DMEM_SCL);
}

void CalcCuMask(uint32_t* lMask00, uint32_t* lMask32, int lStartCu, int lNumCus)
{
   *lMask00 = 0;
   *lMask32 = 0;
   for (int i = lStartCu; i<(lStartCu + lNumCus); i++)
   {
      if (i<32) *lMask00 |= (1 << (i));
      else      *lMask32 |= (1 << (i - 32));
   }
}

void CU_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   //override mem type for debug/test case where SRC is in DMEM
   lSrcMemType = RetSclMemTypeBasedOnAddrMangling((uintptr_t)lpSrc);

   if ((MEM_TYPE_XMEM_SCL == lSrcMemType)) //if src is in 'xmem'
   {
      uint32_t lMask00, lMask32;
      CalcCuMask(&lMask00, &lMask32, lDstVecIdx, 1);
      int32_t lCmemAddr = (intptr_t)lpDst;
      int32_t lChunkSpan = 0; //not used (1D)
      uint32_t lSrcAddr = (uintptr_t)lpSrc;
      int32_t lSrcSpan = 0; //not used (1D)
      int32_t lBytesPerCu = lSize;
      int32_t lHeight = 1; //1D

      //need to pull back src address due to the way this function works
      lSrcAddr = (lSrcAddr & (~0xC0000000)) - (lDstVecIdx*lSize);

      int32_t lRet = apex_transfer_xmem_to_cmem(lCmemAddr,
         lChunkSpan,
         lSrcAddr,
         lSrcSpan,
         lBytesPerCu,
         lHeight,
         lMask00,
         lMask32);

      if (lRet != 0)
      {
         printf("CU_VEC_WRITE_8: Failed xmem->cmem (%d)\n", lRet);
      }
   }
   else //src is in local scalar mem (e.g. dmem) so no xim required
   {
      //TODO: this path is untested
      for (int32_t i = 0; i < lSize; i++)
         lpDst[i] = vput(lpDst[i], lpSrc[i], lDstVecIdx);
   }
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
   //override mem type for debug/test case where DST is in DMEM
   lDstMemType = RetSclMemTypeBasedOnAddrMangling((uintptr_t)lpDst);

   if ((MEM_TYPE_XMEM_SCL == lDstMemType)) //if dst is in 'xmem'
   {
      uint32_t lMask00, lMask32;
      CalcCuMask(&lMask00, &lMask32, lSrcVecIdx, 1);
      int32_t lDstAddr = (intptr_t)lpDst;
      int32_t lDstSpan = 0; //not used (1D)
      int32_t lCmemAddr = (intptr_t)lpSrc;
      int32_t lChunkSpan = 0; //not used (1D)
      int32_t lBytesPerCu = lSize;
      int32_t lHeight = 1; //1D

      //need to pull back dst address due to the way this function works
      lDstAddr = (lDstAddr & (~0xC0000000)) - (lSrcVecIdx*lSize);

      int lRet = apex_transfer_cmem_to_xmem(lDstAddr,
         lDstSpan,
         lCmemAddr,
         lChunkSpan,
         lBytesPerCu,
         lHeight,
         lMask00,
         lMask32);

      if (lRet != 0)
      {
         printf("CU_VEC_READ_8: Failed cmem->xmem (%d)\n", lRet);
      }
   }
   else //dst is in local scalar mem (e.g. dmem) so no xim required
   {
      //TODO: this path is untested
      for (int32_t i = 0; i < lSize; i++)
         lpDst[i] = vget(lpSrc[i], lSrcVecIdx);
   }
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

void CU_VEC_READ_WRITE_32(vec32s* lpSrc, vec32s* lpDst, int32_t lVecIdx,int32_t lSize)
{
   CU_VEC_READ_WRITE_8((vec08s*)lpSrc, (vec08s*)lpDst, lVecIdx, lSize * 4);
}

void SCL_XFER_8(int8_t* lpSrc, int8_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   int lRet = 0;

   //override mem type for debug/test case where SRC/DST is in DMEM
   lSrcMemType = RetSclMemTypeBasedOnAddrMangling((uintptr_t)lpSrc);
   lDstMemType = RetSclMemTypeBasedOnAddrMangling((uintptr_t)lpDst);

   int32_t lDstSpan = 0; //not used (1D)
   int32_t lSrcSpan = 0; //not used (1D)
   int32_t lHeight = 1;  //1D
   int32_t lWidthInBytes = lSize;

   if ((MEM_TYPE_XMEM_SCL == lSrcMemType) && (MEM_TYPE_DMEM_SCL == lDstMemType))
   {
      //src=xmem dst=dmem
      lRet = apex_transfer_xmem_to_dmem((uintptr_t)lpDst,
         lDstSpan,
         (((uintptr_t)lpSrc) & (~0xC0000000)),
         lSrcSpan,
         lWidthInBytes,
         lHeight);
   }
   else if ((MEM_TYPE_DMEM_SCL == lSrcMemType) && (MEM_TYPE_XMEM_SCL == lDstMemType))
   {
      //src=dmem dst=xmem
      lRet = apex_transfer_dmem_to_xmem((((uintptr_t)lpDst) & (~0xC0000000)),
         lDstSpan,
         (uintptr_t)lpSrc,
         lSrcSpan,
         lWidthInBytes,
         lHeight);
   }
   else if ((MEM_TYPE_DMEM_SCL == lSrcMemType) && (MEM_TYPE_DMEM_SCL == lDstMemType))
   {
      //src=dmem dst=dmem

      for (int y = 0; y<lHeight; y++)
      {
         for (int x = 0; x<lWidthInBytes; x++)
         {
            lpDst[y*lDstSpan + x] = lpSrc[y*lSrcSpan + x];
         }
      }
   }

   if (lRet != 0)
   {
      printf("TRANSFER_MemToMem: Failed mem->mem (%d)\n", lRet);
   }
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
   const int32_t lShift = 4;
   int8_t  lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lDstVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lDstVecIdx*lNumCusInGoc);

   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      SCL_XFER_8(lpSrc, lItermediate, lSrcMemType, MEM_TYPE_DMEM_SCL, lXferSizeInDataType);

      if (lCuMask)
      {
         for (int32_t j=0; j<lSizeInDataType; j++)
         {
            vstore(&lpDst[j], (vec16s)0, (vec08s)lItermediate[j]);
         }
      }

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}

void GOC_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   const int32_t lShift = 3;
   int16_t lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lDstVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lDstVecIdx*lNumCusInGoc);

   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      SCL_XFER_16(lpSrc, lItermediate, lSrcMemType, MEM_TYPE_DMEM_SCL, lXferSizeInDataType);

      if (lCuMask)
      {
         for (int32_t j=0; j<lXferSizeInDataType; j++)
         {
            vstore(&lpDst[j], (vec16s)0, (vec16s)lItermediate[j]);
         }
      }

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}

void GOC_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   const int32_t lShift = 2;
   int32_t lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lDstVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lDstVecIdx*lNumCusInGoc);
   
   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      SCL_XFER_32(lpSrc, lItermediate, lSrcMemType, MEM_TYPE_DMEM_SCL, lXferSizeInDataType);

      if (lCuMask)
      {
         for (int32_t j=0; j<lXferSizeInDataType; j++)
         {
            vstore(&lpDst[j], (vec16s)0, (vec32s)lItermediate[j]);
         }
      }

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}

void GOC_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   const int32_t lShift = 4;
   int8_t  lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lSrcVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lSrcVecIdx*lNumCusInGoc);

   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      if (lCuMask)
      {
         for (int32_t j=0; j<lXferSizeInDataType; j++)
         {
            lItermediate[j] = vget(vload(&lpSrc[j], (vec16s)0), lSrcVecIdx*lNumCusInGoc);
         }
      }

      SCL_XFER_8(lItermediate, lpDst, MEM_TYPE_DMEM_SCL, lDstMemType, lXferSizeInDataType);

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}

void GOC_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   const int32_t lShift = 3;
   int16_t  lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lSrcVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lSrcVecIdx*lNumCusInGoc);

   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      if (lCuMask)
      {
         for (int32_t j=0; j<lXferSizeInDataType; j++)
         {
            lItermediate[j] = vget(vload(&lpSrc[j], (vec16s)0), lSrcVecIdx*lNumCusInGoc);
         }
      }

      SCL_XFER_16(lItermediate, lpDst, MEM_TYPE_DMEM_SCL, lDstMemType, lXferSizeInDataType);

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}

void GOC_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   const int32_t lShift = 2;
   int32_t lItermediate[1<<lShift]; //intermediate DMEM buffer
   int32_t lXferSizeInDataType = 1<<lShift;
   int32_t lXferRem = lSizeInDataType - ((lSizeInDataType>>lShift)<<lShift);
   int32_t lNumXfer = (lSizeInDataType + ((1<<lShift) - 1)) >> lShift;
   int32_t lNumCusInGoc = RetNumCusInGoc(lSrcVecMode);
   vbool lCuMask = 0;
   lCuMask = vput(lCuMask, true, lSrcVecIdx*lNumCusInGoc);

   for (int32_t i=0; i<lNumXfer; i++)
   {
      if ((i == (lNumXfer-1)) && (lXferRem != 0))
         lXferSizeInDataType = lXferRem;

      if (lCuMask)
      {
         for (int32_t j=0; j<lXferSizeInDataType; j++)
         {
            lItermediate[j] = vget(vload(&lpSrc[j], (vec16s)0), lSrcVecIdx*lNumCusInGoc);
         }
      }

      SCL_XFER_32(lItermediate, lpDst, MEM_TYPE_DMEM_SCL, lDstMemType, lXferSizeInDataType);

      lpSrc += lXferSizeInDataType;
      lpDst += lXferSizeInDataType;
   }
}
