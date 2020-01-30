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
#include <circ_tile_buf.hpp> //for MEM_TYPE
#include <vector_map_desc.hpp> //for VEC_MODE

void CU_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i].putElement(lpSrc[i], lDstVecIdx);
}

void CU_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i].putElement(lpSrc[i], lDstVecIdx);
}

void CU_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i].putElement(lpSrc[i], lDstVecIdx);
}

void CU_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i].getElement(lSrcVecIdx);
}

void CU_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i].getElement(lSrcVecIdx);
}

void CU_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx,  MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i].getElement(lSrcVecIdx);
}

void CU_VEC_READ_WRITE_8(vec08s* lpSrc, vec08s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
   {
      int8_t lTmp = lpSrc[i].getElement(lVecIdx);
      lpDst[i].putElement(lTmp, lVecIdx);
   }
}

void CU_VEC_READ_WRITE_16(vec16s* lpSrc, vec16s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
   {
      int16_t lTmp = lpSrc[i].getElement(lVecIdx);
      lpDst[i].putElement(lTmp, lVecIdx);
   }
}

void CU_VEC_READ_WRITE_32(vec32s* lpSrc, vec32s* lpDst, int32_t lVecIdx, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
   {
      int32_t lTmp = lpSrc[i].getElement(lVecIdx);
      lpDst[i].putElement(lTmp, lVecIdx);
   }
}

void SCL_XFER_8(int8_t* lpSrc, int8_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i];
}

void SCL_XFER_16(int16_t* lpSrc, int16_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i];
}

void SCL_XFER_32(int32_t* lpSrc, int32_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   for (int32_t i = 0; i < lSize; i++)
      lpDst[i] = lpSrc[i];
}

void GOC_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented for native
}

void GOC_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented for native
}

void GOC_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType)
{
   //not implemented for native
}

void GOC_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented for native
}

void GOC_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented for native
}

void GOC_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType)
{
   //not implemented for native
}
