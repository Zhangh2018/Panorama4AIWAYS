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
#include <icp_datatype.h> 
#include <data_transfer.hpp>
#include <transfer_desc.hpp>
#include <circ_tile_smart_buf.hpp>
#include <vector_map_desc.hpp>
#include <buf_desc.hpp>
#include <data_desc.hpp>

//-------------------------------------------------------------------------------------------
//Generic and slow (but flexible) data transfer implementation - this is placeholder to get all
//targets up and running (e.g. ISS, Native, HW).
//-------------------------------------------------------------------------------------------

/*
NOTES:
- In this methodology there is no pipelining, so the data transfer will be handled in-line.  In the
case of real HW, a data transfer task will likely be broken into two logical tasks - one for scheduling and one that checks for completion
- for data movement, i am assuming scalar pointers refer to physically contiguous memory that can be cast between types; i
am not making the same assumptions about vector memory based on my past experience with native/iss/hw
- there is no evil pointer casting arithmetic going on below; i tried to keep pointer and offset separate throughout
and only bring them together once the pointer is properly typed, especially for vector stuff
*/

void CU_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize);
void CU_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize);
void CU_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSize);
void CU_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize);
void CU_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, MEM_TYPE lDstMemType, int32_t lSize);
void CU_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx,  MEM_TYPE lDstMemType, int32_t lSize);
void CU_VEC_READ_WRITE_8(vec08s* lpSrc, vec08s* lpDst, int32_t lVecIdx,  int32_t lSize);
void CU_VEC_READ_WRITE_16(vec16s* lpSrc, vec16s* lpDst, int32_t lVecIdx,  int32_t lSize);
void CU_VEC_READ_WRITE_32(vec32s* lpSrc, vec32s* lpDst, int32_t lVecIdx, int32_t lSize);
void SCL_XFER_8(int8_t* lpSrc, int8_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize);
void SCL_XFER_16(int16_t* lpSrc, int16_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize);
void SCL_XFER_32(int32_t* lpSrc, int32_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize);
void GOC_VEC_WRITE_8(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType);
void GOC_VEC_WRITE_16(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType);
void GOC_VEC_WRITE_32(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSizeInDataType);
void GOC_VEC_READ_8(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType);
void GOC_VEC_READ_16(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType);
void GOC_VEC_READ_32(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSizeInDataType);

void WRITE_vec08s(int8_t* lpSrc, vec08s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSize)
{
   if (IsGocMode(lDstVecMode))
      GOC_VEC_WRITE_8(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode, lSize);
   else
      CU_VEC_WRITE_8(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("WRITE_vec08s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lDstVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void WRITE_vec16s(int16_t* lpSrc, vec16s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSize)
{
   if (IsGocMode(lDstVecMode))
      GOC_VEC_WRITE_16(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode, lSize);
   else
      CU_VEC_WRITE_16(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("WRITE_vec16s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lDstVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void WRITE_vec32s(int32_t* lpSrc, vec32s* lpDst, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, VEC_MODE lDstVecMode, int32_t lSize)
{
   if (IsGocMode(lDstVecMode))
      GOC_VEC_WRITE_32(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode, lSize);
   else
      CU_VEC_WRITE_32(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("WRITE_vec32s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lDstVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_vec08s(vec08s* lpSrc, int8_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSize)
{
   if (IsGocMode(lSrcVecMode))
      GOC_VEC_READ_8(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize);
   else
      CU_VEC_READ_8(lpSrc, lpDst, lSrcVecIdx, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_vec08s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lSrcVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_vec16s(vec16s* lpSrc, int16_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSize)
{
   if (IsGocMode(lSrcVecMode))
      GOC_VEC_READ_16(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize);
   else
      CU_VEC_READ_16(lpSrc, lpDst, lSrcVecIdx, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_vec16s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lSrcVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_vec32s(vec32s* lpSrc, int32_t* lpDst, int32_t lSrcVecIdx, VEC_MODE lSrcVecMode, MEM_TYPE lDstMemType, int32_t lSize)
{
   if (IsGocMode(lSrcVecMode))
      GOC_VEC_READ_32(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize);
   else
      CU_VEC_READ_32(lpSrc, lpDst, lSrcVecIdx, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_vec32s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lSrcVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_WRITE_vec08s(vec08s* lpSrc, vec08s* lpDst, int32_t lVecIdx, VEC_MODE lVecMode, int32_t lSize)
{
   CU_VEC_READ_WRITE_8(lpSrc, lpDst, lVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_WRITE_vec08s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_WRITE_vec16s(vec16s* lpSrc, vec16s* lpDst, int32_t lVecIdx, VEC_MODE lVecMode, int32_t lSize)
{
   CU_VEC_READ_WRITE_16(lpSrc, lpDst, lVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_WRITE_vec16s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void READ_WRITE_vec32s(vec32s* lpSrc, vec32s* lpDst, int32_t lVecIdx, VEC_MODE lVecMode, int32_t lSize)
{
   CU_VEC_READ_WRITE_32(lpSrc, lpDst, lVecIdx, lSize);

#if (DEBUG_LVL > 1)
   printf("READ_WRITE_vec32s(vecIdx = %d, %d values 0x%p -> 0x%p)\n", lVecIdx, lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void XFER_scl_08(int8_t* lpSrc, int8_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   SCL_XFER_8(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("XFER_scl_08(%d values 0x%p -> 0x%p)\n", lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void XFER_scl_16(int16_t* lpSrc, int16_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   SCL_XFER_16(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("XFER_scl_16(%d values 0x%p -> 0x%p)\n", lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void XFER_scl_32(int32_t* lpSrc, int32_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize)
{
   SCL_XFER_32(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize);

#if (DEBUG_LVL > 1)
   printf("XFER_scl_32(%d values 0x%p -> 0x%p)\n", lSize, (void*)lpSrc, (void*)lpDst);
#endif
}

void TransferChunkRecursive(
   int32_t lNumDim,
   icp::DATATYPE lDataType,
   int32_t lSize_DataType[],

   void* lpSrcAddr,
   MEM_TYPE lSrcMemType,
   int32_t lSrcOffset_DataType,
   int32_t lSrcStride_DataType[],
   VEC_MODE lSrcVecMode,
   int32_t lSrcVecIdx,

   void* lpDstAddr,
   MEM_TYPE lDstMemType,
   int32_t lDstOffset_DataType,
   int32_t lDstStride_DataType[],
   VEC_MODE lDstVecMode,
   int32_t lDstVecIdx,

   int32_t lCurDimIdx, //recursive state variable
   int32_t lIdx[])   //recursive state variable
{
   if (0 == lCurDimIdx)
   {
      //perform transfer associated with lSize[0] dimension (i.e. lowest dimension)

      //calc offsets
      for (int32_t i = 0; i < lNumDim; i++)
      {
         lSrcOffset_DataType += lSrcStride_DataType[i] * lIdx[i];
         lDstOffset_DataType += lDstStride_DataType[i] * lIdx[i];
      }

      if (VEC_MODE_NONE == lSrcVecMode)
      {
         if (VEC_MODE_NONE == lDstVecMode)
         {
            //---------------------------
            //src = scalar, dst = scalar
            //---------------------------

            switch (lDataType)
            {
            case icp::DATATYPE_08S:
            case icp::DATATYPE_08U:
            {
               int8_t* lpSrc = (int8_t*)lpSrcAddr + lSrcOffset_DataType;
               int8_t* lpDst = (int8_t*)lpDstAddr + lDstOffset_DataType;
               XFER_scl_08(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_16S:
            case icp::DATATYPE_16U:
            {
               int16_t* lpSrc = (int16_t*)lpSrcAddr + lSrcOffset_DataType;
               int16_t* lpDst = (int16_t*)lpDstAddr + lDstOffset_DataType;
               XFER_scl_16(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_32S:
            case icp::DATATYPE_32U:
            {
               int32_t* lpSrc = (int32_t*)lpSrcAddr + lSrcOffset_DataType;
               int32_t* lpDst = (int32_t*)lpDstAddr + lDstOffset_DataType;
               XFER_scl_32(lpSrc, lpDst, lSrcMemType, lDstMemType, lSize_DataType[0]);
               break;
            }
            }
         }
         else
         {
            //---------------------------
            //src = scalar, dst = vector
            //---------------------------
            switch (lDataType)
            {
            case icp::DATATYPE_08S:
            case icp::DATATYPE_08U:
            {
               int8_t* lpSrc = (int8_t*)lpSrcAddr + lSrcOffset_DataType;
               vec08s* lpDst = (vec08s*)lpDstAddr + lDstOffset_DataType;
               WRITE_vec08s(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_16S:
            case icp::DATATYPE_16U:
            {
               int16_t* lpSrc = (int16_t*)lpSrcAddr + lSrcOffset_DataType;
               vec16s* lpDst = (vec16s*)lpDstAddr + lDstOffset_DataType;
               WRITE_vec16s(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_32S:
            case icp::DATATYPE_32U:
            {
               int32_t* lpSrc = (int32_t*)lpSrcAddr + lSrcOffset_DataType;
               vec32s* lpDst = (vec32s*)lpDstAddr + lDstOffset_DataType;
               WRITE_vec32s(lpSrc, lpDst, lSrcMemType, lDstVecIdx, lDstVecMode,lSize_DataType[0]);
               break;
            }
            }
         }
      }
      else
      {
         if (VEC_MODE_NONE == lDstVecMode)
         {
            //---------------------------
            //src = vector, dst = scalar
            //---------------------------

            switch (lDataType)
            {
            case icp::DATATYPE_08S:
            case icp::DATATYPE_08U:
            {
               vec08s* lpSrc = (vec08s*)lpSrcAddr + lSrcOffset_DataType;
               int8_t* lpDst = (int8_t*)lpDstAddr + lDstOffset_DataType;
               READ_vec08s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_16S:
            case icp::DATATYPE_16U:
            {
               vec16s* lpSrc = (vec16s*)lpSrcAddr + lSrcOffset_DataType;
               int16_t* lpDst = (int16_t*)lpDstAddr + lDstOffset_DataType;
               READ_vec16s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_32S:
            case icp::DATATYPE_32U:
            {
               vec32s* lpSrc = (vec32s*)lpSrcAddr + lSrcOffset_DataType;
               int32_t* lpDst = (int32_t*)lpDstAddr + lDstOffset_DataType;
               READ_vec32s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lDstMemType, lSize_DataType[0]);
               break;
            }
            }
         }
         else
         {
            //---------------------------
            //src = vector, dst = vector
            //---------------------------

            switch (lDataType)
            {
            case icp::DATATYPE_08S:
            case icp::DATATYPE_08U:
            {
               vec08s* lpSrc = (vec08s*)lpSrcAddr + lSrcOffset_DataType;
               vec08s* lpDst = (vec08s*)lpDstAddr + lDstOffset_DataType;
               READ_WRITE_vec08s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_16S:
            case icp::DATATYPE_16U:
            {
               vec16s* lpSrc = (vec16s*)lpSrcAddr + lSrcOffset_DataType;
               vec16s* lpDst = (vec16s*)lpDstAddr + lDstOffset_DataType;
               READ_WRITE_vec16s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lSize_DataType[0]);
               break;
            }

            case icp::DATATYPE_32S:
            case icp::DATATYPE_32U:
            {
               vec32s* lpSrc = (vec32s*)lpSrcAddr + lSrcOffset_DataType;
               vec32s* lpDst = (vec32s*)lpDstAddr + lDstOffset_DataType;
               READ_WRITE_vec32s(lpSrc, lpDst, lSrcVecIdx, lSrcVecMode, lSize_DataType[0]);
               break;
            }
            }
         }
      }
#if 0
      printf("transfer %d (%d bytes) @ [", lSize_DataType[0], lDataSizeInBytes);
      for (int32_t i = 0; i < lNumDim; i++)
      {
         printf("%d ", lIdx[i]);
      }
      printf("] (src = 0x%p dst = 0x%p\n", lSrcAddr, lDstAddr);
#endif
   }
   else
   {
      for (int32_t i = 0; i < lSize_DataType[lCurDimIdx]; i++)
      {
         lIdx[lCurDimIdx] = i;
         TransferChunkRecursive(
            lNumDim, lDataType, lSize_DataType,
            lpSrcAddr, lSrcMemType, lSrcOffset_DataType, lSrcStride_DataType, lSrcVecMode, lSrcVecIdx,
            lpDstAddr, lDstMemType, lDstOffset_DataType, lDstStride_DataType, lDstVecMode, lDstVecIdx,
            lCurDimIdx - 1, lIdx);
      }
   }
}

void TransferChunk(
   int32_t lNumDim,
   icp::DATATYPE lDataType,
   int32_t lSize_DataType[],

   void* lpSrcAddr,
   MEM_TYPE lSrcMemType,
   int32_t lSrcOffset_DataType,
   int32_t lSrcStride_DataType[],
   VEC_MODE lSrcVecMode,
   int32_t lSrcVecIdx,

   void* lpDstAddr,
   MEM_TYPE lDstMemType,
   int32_t lDstOffset_DataType,
   int32_t lDstStride_DataType[],
   VEC_MODE lDstVecMode,
   int32_t lDstVecIdx)
{
   int32_t lIdx[DESC_MAX_NUM_DIM] = { 0 }; //state var required for TransferChunkRecursive
   TransferChunkRecursive(
      lNumDim, lDataType, lSize_DataType,
      lpSrcAddr, lSrcMemType, lSrcOffset_DataType, lSrcStride_DataType, lSrcVecMode, lSrcVecIdx,
      lpDstAddr, lDstMemType, lDstOffset_DataType, lDstStride_DataType, lDstVecMode, lDstVecIdx,
      lNumDim - 1, lIdx);
}

int32_t DataTransfer::SetupEnvSpecific()
{
   //nothing to do here
   return 0;
}

int32_t DataTransfer::Iterate_ToApexCbScl(int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   if (mNumTilesTransfered < mNumTilesTotal)
   {
      if (mpCircTileSmartBufDst->QueryTilesAvailForProd(lNumTilesToTransfer))
      {
         mpCircTileSmartBufDst->GetTilePtrArrayForProd(lNumTilesToTransfer, mpTilePtrArrayDst);

         for (int32_t i = 0; i < lNumTilesToTransfer; i++)
         {
            //this is here because of the way mTileLoc is used (after call to CalcNextTileOffsetInDataType it contains
            //next tile loc, not current)
            TileInfo lTileInfo;
            for (int32_t j = 0; j < mNumDim; j++)
               lTileInfo.mTileLoc[j] = mTileLoc[j];
            mpCircTileSmartBufDst->SetTileInfo(&lTileInfo, i);

            int32_t lTileOffsetInDataType = CalcNextTileOffsetInDataType();

            TransferChunk(
               mNumDim,
               mpBufDesc->mpDataDesc->mDataType,
               mTransferSize_DataType,

               (void*)mpBufDesc->mDataAddress,
               MEM_TYPE_XMEM_SCL,
               lTileOffsetInDataType,
               mpBufDesc->mpDataDesc->mStrideInDataType,
               VEC_MODE_NONE, 0,

               mpTilePtrArrayDst[i],
               mpCircTileSmartBufDst->mCircTileBuf.mBufType,
               0,
               mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc->mStrideInDataType,
               VEC_MODE_NONE, 0);

#if (DEBUG_LVL > 0)
            printf("DataTransfer::Iterate_ToApexCbScl -> ");
            printf("[");
            for (int32_t i = 0; i < mNumDim; i++)
            {
               if (i == (mNumDim - 1)) printf("%d]", mTransferSize_DataType[i]);
               else                    printf("%d,", mTransferSize_DataType[i]);
            }
            printf("*%d values from SRC=0x%p(+0x%X) to DST=0x%p\n", mNumVchunksTotal, (void*)mpBufDesc->mDataAddress, lTileOffsetInDataType, mpTilePtrArrayDst[i]);
#endif

            mNumTilesTransfered++;
         }

         int32_t lLastFlag = (mNumTilesTransfered == mNumTilesTotal) ? 1 : 0;
         mpCircTileSmartBufDst->TilesProd(lNumTilesToTransfer, lLastFlag);
      }
   }

   return lRetVal;
}

int32_t DataTransfer::Iterate_ToApexCbVec(int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   if (mNumTilesTransfered < mNumTilesTotal)
   {
      if (mpCircTileSmartBufDst->QueryTilesAvailForProd(lNumTilesToTransfer))
      {
         mpCircTileSmartBufDst->GetTilePtrArrayForProd(lNumTilesToTransfer, mpTilePtrArrayDst);

         for (int32_t i = 0; i < lNumTilesToTransfer; i++)
         {
            //this is 'before' because of the way mTileLoc is used (after call to CalcNextTileOffsetInDataType it contains
            //next tile loc, not current)
            TileInfo lTileInfo;
            for (int32_t j = 0; j < mNumDim; j++)
               lTileInfo.mTileLoc[j] = mTileLoc[j];
            mpCircTileSmartBufDst->SetTileInfo(&lTileInfo, i);

            int32_t lTileOffsetInDataType = CalcNextTileOffsetInDataType();
            //printf("DataTransfer::Iterate_ToApexCbVec -> lTileOffsetInDataType = %d\n", lTileOffsetInDataType);
            int32_t lVecIdx = mpVectorMapDesc->mVecPad[0];
            int32_t lWrapOffsetInDataType = 0;

            for (int32_t j = 0; j < mNumVchunksTotal; j++)
            {
               int32_t lVchunkOffsetInDataType = CalcNextVchunkOffsetInDataType(); //this automatically resets!

               //deal with vector wrapping
               if (lVecIdx == mpVectorMapDesc->mPhysVecSize)
               {
                  lVecIdx = 0;
                  lWrapOffsetInDataType += mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc->mTotalSizeInDataType;
               }

               TransferChunk(
                  mNumDim,
                  mpBufDesc->mpDataDesc->mDataType,
                  mTransferSize_DataType,

                  (void*)mpBufDesc->mDataAddress,
                  MEM_TYPE_XMEM_SCL,
                  lTileOffsetInDataType + lVchunkOffsetInDataType,
                  mpBufDesc->mpDataDesc->mStrideInDataType,
                  VEC_MODE_NONE, 0,

                  mpTilePtrArrayDst[i],
                  mpCircTileSmartBufDst->mCircTileBuf.mBufType,
                  lWrapOffsetInDataType,
                  mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc->mStrideInDataType,
                  mpVectorMapDesc->mVecMode,
                  lVecIdx);

               lVecIdx++;
            }

#if (DEBUG_LVL > 0)
            printf("DataTransfer::Iterate_ToApexCbVec -> ");
            printf("[");
            for (int32_t i = 0; i < mNumDim; i++)
            {
               if (i == (mNumDim - 1)) printf("%d]", mTransferSize_DataType[i]);
               else                  printf("%d,", mTransferSize_DataType[i]);
            }
            printf("*%d values from SRC=0x%p(+0x%X) to DST=0x%p\n", mNumVchunksTotal, (void*)mpBufDesc->mDataAddress, lTileOffsetInDataType, mpTilePtrArrayDst[i]);
#endif

            mNumTilesTransfered++;
         }

         int32_t lLastFlag = (mNumTilesTransfered == mNumTilesTotal) ? 1 : 0;
         mpCircTileSmartBufDst->TilesProd(lNumTilesToTransfer, lLastFlag);
      }
   }

   return lRetVal;
}

int32_t DataTransfer::Iterate_FromApexCbScl(int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   if (mNumTilesTransfered < mNumTilesTotal)
   {
      if (mpCircTileSmartBufSrc->QueryTilesAvailForCons(lNumTilesToTransfer, mConsId))
      {
         mpCircTileSmartBufSrc->GetTilePtrArrayForCons(lNumTilesToTransfer, mpTilePtrArraySrc, mConsId);

         for (int32_t i = 0; i < lNumTilesToTransfer; i++)
         {
            int32_t lTileOffsetInDataType = CalcNextTileOffsetInDataType();

            TransferChunk(
               mNumDim,
               mpBufDesc->mpDataDesc->mDataType,
               mTransferSize_DataType,

               mpTilePtrArraySrc[i],
               mpCircTileSmartBufSrc->mCircTileBuf.mBufType,
               0,
               mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mStrideInDataType,
               VEC_MODE_NONE, 0,

               (void*)(mpBufDesc->mDataAddress),
               MEM_TYPE_XMEM_SCL,
               lTileOffsetInDataType,
               mpBufDesc->mpDataDesc->mStrideInDataType,
               VEC_MODE_NONE, 0);

#if (DEBUG_LVL > 0)
            printf("DataTransfer::Iterate_FromApexCbScl -> ");
            printf("[");
            for (int i = 0; i<mNumDim; i++)
            {
               if (i == (mNumDim - 1)) printf("%d]", mTransferSize_DataType[i]);
               else                  printf("%d,", mTransferSize_DataType[i]);
            }
            printf("*%d values from SRC=0x%p to DST=0x%p(+0x%X)\n", mNumVchunksTotal, mpTilePtrArraySrc[i], (void*)mpBufDesc->mDataAddress, lTileOffsetInDataType);
#endif

            mNumTilesTransfered++;
         }

         mpCircTileSmartBufSrc->TilesCons(lNumTilesToTransfer, mConsId);
      }
   }

   return lRetVal;
}

int32_t DataTransfer::Iterate_FromApexCbVec(int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   if (mNumTilesTransfered < mNumTilesTotal)
   {
      if (mpCircTileSmartBufSrc->QueryTilesAvailForCons(lNumTilesToTransfer, mConsId))
      {
         mpCircTileSmartBufSrc->GetTilePtrArrayForCons(lNumTilesToTransfer, mpTilePtrArraySrc, mConsId);

         for (int32_t i = 0; i < lNumTilesToTransfer; i++)
         {
            int32_t lTileOffsetInDataType = CalcNextTileOffsetInDataType();
            int32_t lVecIdx = mpVectorMapDesc->mVecPad[0];
            int32_t lWrapOffsetInDataType = 0;

            for (int32_t j = 0; j < mNumVchunksTotal; j++)
            {
               int32_t lVchunkOffsetInDataType = CalcNextVchunkOffsetInDataType(); //this automatically resets!

               //deal with vector wrapping
               if (lVecIdx == mpVectorMapDesc->mPhysVecSize)
               {
                  lVecIdx = 0;
                  lWrapOffsetInDataType += mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mTotalSizeInDataType;
               }

               TransferChunk(
                  mNumDim,
                  mpBufDesc->mpDataDesc->mDataType,
                  mTransferSize_DataType,

                  mpTilePtrArraySrc[i],
                  mpCircTileSmartBufSrc->mCircTileBuf.mBufType,
                  lWrapOffsetInDataType,
                  mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mStrideInDataType,
                  mpVectorMapDesc->mVecMode,
                  lVecIdx,

                  (void*)mpBufDesc->mDataAddress,
                  MEM_TYPE_XMEM_SCL,
                  lTileOffsetInDataType + lVchunkOffsetInDataType,
                  mpBufDesc->mpDataDesc->mStrideInDataType,
                  VEC_MODE_NONE, 0);

               lVecIdx++;
            }

#if (DEBUG_LVL > 0)
            printf("DataTransfer::Iterate_FromApexCbVec -> ");
            printf("[");
            for (int i = 0; i < mNumDim; i++)
            {
               if (i == (mNumDim - 1)) printf("%d]", mTransferSize_DataType[i]);
               else                  printf("%d,", mTransferSize_DataType[i]);
            }
            printf("*%d values from SRC=0x%p to DST=0x%p(+0x%X)\n", mNumVchunksTotal, mpTilePtrArraySrc[i], (void*)mpBufDesc->mDataAddress, lTileOffsetInDataType);
#endif

            mNumTilesTransfered++;
         }

         mpCircTileSmartBufSrc->TilesCons(lNumTilesToTransfer, mConsId);
      }
   }

   return lRetVal;
}

int32_t DataTransfer::Iterate_ApexCbToApexCb(int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   if (mpCircTileSmartBufSrc->QueryTilesAvailForCons(lNumTilesToTransfer, mConsId) &&
      mpCircTileSmartBufDst->QueryTilesAvailForProd(lNumTilesToTransfer))
   {
      mpCircTileSmartBufSrc->GetTilePtrArrayForCons(lNumTilesToTransfer, mpTilePtrArraySrc, mConsId);
      mpCircTileSmartBufDst->GetTilePtrArrayForProd(lNumTilesToTransfer, mpTilePtrArrayDst);

      DataDesc* lpSrcTileDesc = mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc;
      DataDesc* lpDstTileDesc = mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc;
      int32_t lVecIdx, lWrapOffsetInDataTypeSrc, lWrapOffsetInDataTypeDst;
      MEM_TYPE lSrcMemType = mpCircTileSmartBufSrc->mCircTileBuf.mBufType;
      MEM_TYPE lDstMemType = mpCircTileSmartBufDst->mCircTileBuf.mBufType;

      for (int32_t i = 0; i < lNumTilesToTransfer; i++)
      {
         //propagate tile info
         TileInfo* lpTileInfo = mpCircTileSmartBufSrc->GetTileInfo(i, mConsId);
         mpCircTileSmartBufDst->SetTileInfo(lpTileInfo, i);

         switch (mpTransferDesc->mTransferType)
         {
         case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC:

            lVecIdx = mpVectorMapDesc->mVecPad[0];
            lWrapOffsetInDataTypeSrc = 0;
            lWrapOffsetInDataTypeDst = 0;

            for (int32_t j = 0; j < mNumVchunksTotal; j++)
            {
               //deal with vector wrapping
               if (lVecIdx == mpVectorMapDesc->mPhysVecSize)
               {
                  lVecIdx = 0;
                  lWrapOffsetInDataTypeSrc += lpSrcTileDesc->mTotalSizeInDataType;
                  lWrapOffsetInDataTypeDst += lpDstTileDesc->mTotalSizeInDataType;
               }

               TransferChunk(
                  mNumDim, lpSrcTileDesc->mDataType, mTransferSize_DataType,
                  mpTilePtrArraySrc[i], lSrcMemType, lWrapOffsetInDataTypeSrc, lpSrcTileDesc->mStrideInDataType, mpVectorMapDesc->mVecMode, lVecIdx,
                  mpTilePtrArrayDst[i], lDstMemType, lWrapOffsetInDataTypeDst, lpDstTileDesc->mStrideInDataType, mpVectorMapDesc->mVecMode, lVecIdx);

               lVecIdx++;
            }

            break;

         case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL:

            TransferChunk(
               mNumDim, lpSrcTileDesc->mDataType, mTransferSize_DataType,
               mpTilePtrArraySrc[i], lSrcMemType, 0, lpSrcTileDesc->mStrideInDataType, VEC_MODE_NONE, 0,
               mpTilePtrArrayDst[i], lDstMemType, 0, lpDstTileDesc->mStrideInDataType, VEC_MODE_NONE, 0);

            break;

         case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL:

            lVecIdx = mpVectorMapDesc->mVecPad[0];
            lWrapOffsetInDataTypeSrc = 0;

            for (int32_t j = 0; j < mNumVchunksTotal; j++)
            {
               int32_t lVchunkOffsetInDataType = CalcNextVchunkOffsetInDataType(mpTileVectorizationDesc, lpDstTileDesc);

               //deal with vector wrapping
               if (lVecIdx == mpVectorMapDesc->mPhysVecSize)
               {
                  lVecIdx = 0;
                  lWrapOffsetInDataTypeSrc += lpSrcTileDesc->mTotalSizeInDataType;
               }

               TransferChunk(
                  mNumDim, lpSrcTileDesc->mDataType, mTransferSize_DataType,
                  mpTilePtrArraySrc[i], lSrcMemType, lWrapOffsetInDataTypeSrc, lpSrcTileDesc->mStrideInDataType, mpVectorMapDesc->mVecMode, lVecIdx,
                  mpTilePtrArrayDst[i], lDstMemType, lVchunkOffsetInDataType, lpDstTileDesc->mStrideInDataType, VEC_MODE_NONE, 0);

               lVecIdx++;
            }

            break;

         case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC:

            lVecIdx = mpVectorMapDesc->mVecPad[0];
            lWrapOffsetInDataTypeDst = 0;

            for (int32_t j = 0; j < mNumVchunksTotal; j++)
            {
               int32_t lVchunkOffsetInDataType = CalcNextVchunkOffsetInDataType(mpTileVectorizationDesc, lpSrcTileDesc);

               //deal with vector wrapping
               if (lVecIdx == mpVectorMapDesc->mPhysVecSize)
               {
                  lVecIdx = 0;
                  lWrapOffsetInDataTypeDst += lpDstTileDesc->mTotalSizeInDataType;
               }

               TransferChunk(
                  mNumDim, lpSrcTileDesc->mDataType, mTransferSize_DataType,
                  mpTilePtrArraySrc[i], lSrcMemType, lVchunkOffsetInDataType, lpSrcTileDesc->mStrideInDataType, VEC_MODE_NONE, 0,
                  mpTilePtrArrayDst[i], lDstMemType, lWrapOffsetInDataTypeDst, lpDstTileDesc->mStrideInDataType, mpVectorMapDesc->mVecMode, lVecIdx);

               lVecIdx++;
            }

            break;
         }

#if (DEBUG_LVL > 0)
         printf("DataTransfer::Iterate_ApexCbToApexCb -> ");
         printf("[");
         for (int32_t i = 0; i < mNumDim; i++)
         {
            if (i == (mNumDim - 1)) printf("%d]", mTransferSize_DataType[i]);
            else                  printf("%d,", mTransferSize_DataType[i]);
         }
         printf("*%d values from SRC=0x%p to DST=0x%p\n", mNumVchunksTotal, mpTilePtrArraySrc[i], mpTilePtrArrayDst[i]);
#endif

         mNumTilesTransfered++;
      }

      mpCircTileSmartBufSrc->TilesCons(lNumTilesToTransfer, mConsId);
      mpCircTileSmartBufDst->TilesProd(lNumTilesToTransfer, mpCircTileSmartBufSrc->QueryDone(mConsId));
   }

   return lRetVal;
}











//*****************************************************************************************************
//*****************************************************************************************************

/*GOC data movement notes:
- write 4 bytes per CU across the GOC and wrap back to first CU in GOC if the size exceeds (lNumCusInGoc*4)
- this
-******DO I NEED TO TRANSLATE THE VECTOR ADDRESSESS??? I think so :(  These are addresses calculated assuming CU access, not GOC access.
(it's good i kept the tile start and the offset separate since i'll have to use this to perform the translation!)
-***this is uglier than i thought - core offset also needs to be 'adjusted' properly if it is non-zero.  plus, i don't want to assume alignment at the start
so the first thing written may NOT be 4 bytes, it may be less
*/

/*
does the lpDst type make sense here??  Is the vector address always analogous to the GOC address??  there may be
conceptual issues here ... need to think more.
basically for GOC mode i'll be reinterpreting the vector address; how exactly does this re-interpretation work and what
assumptions are being made?
e.g. if lpDst above = 64 ... what does that mean?  am i actually assuming this means address 64 from the CU perspective, then I reinterpret it for
the GOC accordingly?
- am I just going to use the tile ptrs to contain the CU addr, then reinterpret?  Depending on if there are 2 or 4 bytes per CU, this would change though,
which is wierd.  if I say CU X addr A, this is absolute and always refers to the same physical memory location.  if I say GOC8_0 addr A, I would like the same
thing to be true to maintain consistency.  I probably need to use something else here that isn't necessarily physical, then adjust to physical based on
how the data was written to CMEM (e.g. 2b/CU vs 4b/CU).
*/

/*
Conceptually for a CU, i take an address between 0 and 8K, plus a vec idx (i.e CU idx) to determine the physical location in CMEM.
I should do the same for GOC, except take an address between 0 and (lNumCusInGoc * 8K) plus vec idx (i.e. GOC idx).
The GOC address could be combined with the vector idx to determine exact physical translation, and would be consistent
with the CU-based methodology.

Maybe it makes sense to keep everything in CU and have another 'sub-idx' value to capture where in the GOC we are?
In CU case we have vecXX* + vecidx(cu) to know how to get physical.
In GOC case we have vecXX* + vecidx(goc) + vecidx(cu) to know how to get physical


 */
//this function takes vector offset in CU mode and returns the vector offset for GOC interpretation
/*
int32_t TranslateOffset(int32_t lOffsetInDataType, int32_t lDataTypeBitCnt, VEC_MODE lVecMode)
{
   int32_t lOffset = lOffsetInDataType;

   if (IsGocMode(lVecMode))
   {
      int32_t lValPerCu = 4>>(lDataTypeBitCnt>>4); //assuming 4 bytes per CU physical mapping
      int32_t lNumCusInGoc = RetNumCusInGoc(lVecMode);
      int32_t lGocWrapCnt = lOffsetInDataType/(lNumCusInGoc*lValPerCu);
      int32_t lRem = lOffsetInDataType - (lGocWrapCnt*lNumCusInGoc*lValPerCu);
      lOffset = lValPerCu*lGocWrapCnt;
      lOffset += lRem%lValPerCu;
   }

   return lOffset;
}
*/

/*
void GOC_VEC_WRITE_8(int8_t* lpSrc, uintptr_t lDstAddr, MEM_TYPE lSrcMemType, int32_t lDstVecIdx, int32_t lSizeInDataType, VEC_MODE lDstVecMode)
{
   const int32_t lDataTypeShift = 0; //this must change depending on data type (0 for 8b, 1 for 16b, 2 for 32b)
   const int32_t lBytesPerCu = 4;    //this is always 4 since it is based on physical memory
   const int32_t lDataTypePerCu = lBytesPerCu >> lDataTypeShift;
   int32_t lSizeInBytes = lSizeInDataType << lDataTypeShift;

   int32_t lGocShift = 2; //GOC4
   if      (VEC_MODE_GOC8  == lDstVecMode) lGocShift = 3;

   int32_t lNumCusInGoc = RetNumCusInGoc(lDstVecMode);
   int32_t lCuIdx = (lDstVecIdx * lNumCusInGoc) + ((lDstAddr - ((lDstAddr>>(lGocShift+2))<<((lGocShift+2))))>>2); //CU idx for first CU_VEC_WRITE_8

   vec08s* lpDstFirst = (vec08s*)((lDstAddr >> lGocShift) + (lDstAddr & 0x3)); //vector address of first 4 byte (or <) segment
   vec08s* lpDstIter = (vec08s*)(lDstAddr >> lGocShift);                       //vector address for remaining 4 byte segments (size of last may be < 4)
   int32_t lBytesPerCuFirst = lBytesPerCu - (lDstAddr & 0x3);                  //number of bytes to write the first time to lpDstFirst
   if (lSizeInBytes < lBytesPerCuFirst)
      lBytesPerCuFirst = lSizeInBytes;
   int32_t lBytesPerCuLast = (lSizeInBytes - lBytesPerCuFirst) & 0x3;          //number of bytes to write the last time to lpDstIter
   if (0 == lBytesPerCuLast)
      lBytesPerCuLast = lBytesPerCu;
   int32_t lNumIter = 1 + ((lSizeInBytes - lBytesPerCuFirst + (lBytesPerCu - 1))>>2);

   int32_t lWriteSizeInDataType = 0;
   vec08s* lpDst = 0;

   for (int32_t i=0; i<lNumIter; i++)
   {
      if (0 == i) //first
      {
         lWriteSizeInDataType = (lBytesPerCuFirst >> lDataTypeShift);
         lpDst = lpDstFirst;
      }
      else
      {
         lpDst = lpDstIter;

         if ((lNumIter-1) == i) //last
            lWriteSizeInDataType = (lBytesPerCuLast >> lDataTypeShift);
         else //middle
            lWriteSizeInDataType = (lBytesPerCu >> lDataTypeShift);
      }

      CU_VEC_WRITE_8(lpSrc, lpDst, lSrcMemType, lCuIdx, lWriteSizeInDataType);

      lpSrc += lWriteSizeInDataType;

      if (lCuIdx == (lNumCusInGoc - 1))
      {
         lCuIdx = 0;
         lpDstIter += lDataTypePerCu;
      }
      else
      {
         lCuIdx ++;
      }
   }
}*/
