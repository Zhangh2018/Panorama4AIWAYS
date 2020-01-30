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
#include <data_desc.hpp>
#include <data_transfer.hpp>
#include <transfer_desc.hpp>
#include <utilities_desc.hpp>
#include <buf_desc.hpp>
#include <tiling_desc.hpp>
#include <tile_vectorization_desc.hpp>
#include <circ_tile_smart_buf.hpp>
#include <apu_mem_mgmt.hpp>

void SCL_XFER_32(int32_t* lpSrc, int32_t* lpDst, MEM_TYPE lSrcMemType, MEM_TYPE lDstMemType, int32_t lSize);

int32_t DataTransfer::Setup(TransferDesc* lpTransferDesc, int32_t lNumTilesToTransfer)
{
   int32_t lRetVal = 0;

   Init();

   mpTransferDesc = lpTransferDesc;
   mNumTilesToTransfer = lNumTilesToTransfer;

   //-------------------------
   //initialize local members
   //-------------------------

   //init some locals for easier access
   mpBufDesc = mpTransferDesc->mpBufDesc;
   mpCircTileSmartBufSrc = mpTransferDesc->mpCircTileBufSmartSrc;
   mpCircTileSmartBufDst = mpTransferDesc->mpCircTileBufSmartDst;
   mpTilingDesc = mpTransferDesc->mpTilingDesc;
   mpTileVectorizationDesc = mpTransferDesc->mpTileVectorizationDesc;
   mpVectorMapDesc = mpTransferDesc->mpVectorMapDesc;

   //extract number of dimensions (should be same for all so get it from the first available)
   if (mpBufDesc != 0)
      mNumDim = mpBufDesc->mpDataDesc->mNumDim; 
   else if (mpCircTileSmartBufSrc != 0)
      mNumDim = mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mNumDim;
   else if (mpCircTileSmartBufDst != 0)
      mNumDim = mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc->mNumDim;

   if (0 != mpTransferDesc->mpPatchOffsetArray) //if 2D patch processing transfer
   {
      //need to understand if a patch is defined such that patch height != tile height managed by the CB
      //(e.g. patch is 8x8, but the destination CB manages lines of 8x1 - in must be possible to perform
      //address calculation at the line level, not just the patch level, and all interaction with the CB
      //is wrt to the tile unit it is managing (e.g. in the 2D case this will almost always be lines)
      int32_t lCbHeight = mpCircTileSmartBufDst->mCircTileBuf.mpTileDesc->mSize[1];
      mNumTilesPerPatch = mpTransferDesc->mPatchHeight / lCbHeight;
      mPatchTileOffsetInDataType = mpBufDesc->mpDataDesc->mStrideInDataType[1] * mpBufDesc->mpDataDesc->mElementSize[1] * lCbHeight;
      mNumTilesTotal = mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[1] * mNumTilesPerPatch;
      mNumVchunksTotal = mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]; //tile 'width' in vchunks is same as offset array width
      mTransferSize_DataType[0] = mpTransferDesc->mPatchWidth * mpBufDesc->mpDataDesc->mElementSize[0];
      mTransferSize_DataType[1] = lCbHeight * mpBufDesc->mpDataDesc->mElementSize[1];

      //deal with patch offsets (they reside in external memory, and must be copied into APEX memory)
      mPatchOffsetArray = (int32_t*)ApuMemAllocSclDmem(mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]*4); //allocate space for single row of offsets
      //mPatchOffsetArray = (int32_t*)ApuMemAllocSclDmem(mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mTotalSizeInBytes); //allocate space for all offsets

      if (0 != mPatchOffsetArray)
      {
         SCL_XFER_32((int32_t*)mpTransferDesc->mpPatchOffsetArray->mDataAddress, mPatchOffsetArray, MEM_TYPE_XMEM_SCL, MEM_TYPE_DMEM_SCL, mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]); //copy first row of offsets
         //SCL_XFER_32((int32_t*)mpTransferDesc->mpPatchOffsetArray->mDataAddress, mPatchOffsetArray, MEM_TYPE_XMEM_SCL, MEM_TYPE_DMEM_SCL, mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mTotalSizeInDataType); //copy all offsets
         //for (int32_t i = 0; i < mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]; i++)
         //   printf("OFFSET=%d\n", mPatchOffsetArray[i]);
      }
      else //memory allocation failure
      {
         printf("DataTransfer::Setup -> failed to allocate memory (%d bytes) for indirect offset array!\n", mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]*4);
         lRetVal = 1;
      }
   }
   else //tiling described by mpTilingDesc is being performed on a region of data defined by mpBufDesc
   {
      if (mpTilingDesc && mpBufDesc)
      {
         for (int32_t i = 0; i < mNumDim; i++)
            mNumTileInDim[i] = CalcNumTileInDim(mpBufDesc->mpDataDesc, mpTilingDesc, i);

         mNumTilesTotal = CalcNumTileTotal(mpBufDesc->mpDataDesc, mpTilingDesc);
      }

      if (TRANSFER_TYPE__TO_APEX_CB_VEC_ITER == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER == mpTransferDesc->mTransferType)
      {
         for (int32_t i = 0; i < mNumDim; i++)
         {
            mNumVchunkInDim[i] = CalcNumVchunkInDim(mpTilingDesc->mSize[i], mpTileVectorizationDesc, i);
            mTransferSize_DataType[i] = mpTileVectorizationDesc->mSize[i] * mpBufDesc->mpDataDesc->mElementSize[i];
         }

         mNumVchunksTotal = CalcNumVchunkTotal(mpTilingDesc, mpTileVectorizationDesc);
      }
      else if (TRANSFER_TYPE__TO_APEX_CB_SCL_ITER == mpTransferDesc->mTransferType ||
               TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER == mpTransferDesc->mTransferType)
      {
         for (int32_t i = 0; i < mNumDim; i++)
         {
            mNumVchunkInDim[i] = 1;
            mTransferSize_DataType[i] = mpTilingDesc->mSize[i] * mpBufDesc->mpDataDesc->mElementSize[i];
         }

         mNumVchunksTotal = 1;
      }
      else if (TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC == mpTransferDesc->mTransferType ||
               TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL == mpTransferDesc->mTransferType)
      {
         //- in the CB->CB cases the user is not providing a tiling descriptor since CBs are already working in units of tiles
         //- in the vec->scl and scl->vec cases, a vectorization descriptor will be required
         //- in the vec->vec and scl->scl cases, a blind copy will be performed based on CB tile desc (it is assumed
         //  that the src and dst tile desc be the same size)

         for (int32_t i = 0; i < mNumDim; i++)
            mTransferSize_DataType[i] = mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mSize[i] * mpCircTileSmartBufSrc->mCircTileBuf.mpTileDesc->mElementSize[i];

         mNumVchunksTotal = mpCircTileSmartBufSrc->mCircTileBuf.mNumVchunksInTile;
      }
      else if (TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL == mpTransferDesc->mTransferType ||
               TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC == mpTransferDesc->mTransferType)
      {
         CircTileSmartBuf* lpSclCSB = mpCircTileSmartBufSrc;
         if (TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL == mpTransferDesc->mTransferType)
            lpSclCSB = mpCircTileSmartBufDst;

         for (int32_t i = 0; i < mNumDim; i++)
         {
            mNumVchunkInDim[i] = CalcNumVchunkInDim(lpSclCSB->mCircTileBuf.mpTileDesc->mSize[i], mpTileVectorizationDesc, i);
            mTransferSize_DataType[i] = mpTileVectorizationDesc->mSize[i] * lpSclCSB->mCircTileBuf.mpTileDesc->mElementSize[i];
         }

         mNumVchunksTotal = CalcNumVchunkTotal(lpSclCSB->mCircTileBuf.mpTileDesc->mSize, mpTileVectorizationDesc);
      }
   }
      
#if (DEBUG_LVL > 0)
   switch (mpTransferDesc->mTransferType)
   {
   case TRANSFER_TYPE__TO_APEX_CB_SCL_ITER:
   case TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER:
      printf("DataTransfer::Setup (EXT<->CB_SCL) -> %d tiles\n", mNumTilesTotal);
      break;
   case TRANSFER_TYPE__TO_APEX_CB_VEC_ITER:
   case TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER:
      printf("DataTransfer::Setup (EXT<->CB_VEC) -> %d tiles, %d vchunks/tile (excluding padding)\n", mNumTilesTotal, mNumVchunksTotal);
      break;
   case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC:
   case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL:
   case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL:
   case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC:
      printf("DataTransfer::Setup (CB->CB) -> %d vchunks/tile (excluding padding)\n", mNumVchunksTotal);
      break;
   }
#endif

   //allocate memory for tile pointer array
   int32_t lTilePtrArrayAllocOk = 1;
   if (mpCircTileSmartBufSrc)
   {
      mpTilePtrArraySrc = (void**)ApuMemAllocSclDmem(sizeof(void*)*mNumTilesToTransfer);
      if (0 == mpTilePtrArraySrc)
         lTilePtrArrayAllocOk = 0;
   }

   if (mpCircTileSmartBufDst)
   {
      mpTilePtrArrayDst = (void**)ApuMemAllocSclDmem(sizeof(void*)*mNumTilesToTransfer);
      if (0 == mpTilePtrArrayDst)
         lTilePtrArrayAllocOk = 0;
   }

   if (lTilePtrArrayAllocOk)
   {
      //in the vector case, based on transfer description, determine which transfer pattern is optimal
      if (TRANSFER_TYPE__TO_APEX_CB_VEC_ITER == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER == mpTransferDesc->mTransferType)
      {
         if (0 != DetermineOptimalVecTransferPattern())
         {
            printf("DataTransfer::Setup -> optimal vector transfer pattern detection failed!\n");
            lRetVal = 1;
         }
      }

      //perform setup specific to the environment (iss, native, hw)
      if (0 != SetupEnvSpecific())
      {
         printf("DataTransfer::Setup -> environment specific setup failed!\n");
         lRetVal = 1;
      }

      //if data transfer is behaving as a consumer, get a consumer id
      if (TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL == mpTransferDesc->mTransferType ||
          TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC == mpTransferDesc->mTransferType)
      {
         mpCircTileSmartBufSrc->ReqConsId(&mConsId);
      }
   }
   else //memory allocation failure
   {
      printf("DataTransfer::Setup -> tile pointer array memory allocation failure!\n");
      lRetVal = 1;
   }

   return lRetVal;
}

int32_t DataTransfer::Iterate()
{
   int32_t lRetVal = 0;

   switch (mpTransferDesc->mTransferType)
   {
   case TRANSFER_TYPE__TO_APEX_CB_SCL_ITER:
      Iterate_ToApexCbScl(mNumTilesToTransfer);
      break;
   case TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER:
      Iterate_FromApexCbScl(mNumTilesToTransfer);
      break;

   case TRANSFER_TYPE__TO_APEX_CB_VEC_ITER:
      Iterate_ToApexCbVec(mNumTilesToTransfer);
      break;
   case TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER:
      Iterate_FromApexCbVec(mNumTilesToTransfer);
      break;

   case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC:
   case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL:
   case TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL:
   case TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC:
      Iterate_ApexCbToApexCb(mNumTilesToTransfer);
      break;

   default:
      lRetVal = 1;
      break;
   }

   return lRetVal;
}

int32_t DataTransfer::Teardown()
{
   ApuMemFree((int8_t*)mpTilePtrArraySrc);
   ApuMemFree((int8_t*)mpTilePtrArrayDst);
   ApuMemFree((int8_t*)mPatchOffsetArray);

   return 0;
}

int32_t DataTransfer::Finished()
{
   return (mNumTilesTransfered == mNumTilesTotal) ? 1 : 0;
}

void DataTransfer::Reset()
{
   for (int32_t i = 0; i < mNumDim; i++)
   {
      mTileLoc[i] = 0;
      mVchunkLoc[i] = 0;
   }

   mNumTilesTransfered = 0;
   mPatchOffsetRowTileCnt = 0;
   mPatchOffsetRowCnt = 0;
   mPatchOffsetColCnt = 0;

   //no explicit reset of mpCircTileSmartBuf is being performed here ... user can do it separately
   //if required
}

void DataTransfer::Init()
{
   for (int32_t i = 0; i < DESC_MAX_NUM_DIM; i++)
   {
      mTileLoc[i] = 0;
      mVchunkLoc[i] = 0;
      mNumTileInDim[i] = 0;
      mNumVchunkInDim[i] = 0;
   }

   mpTransferDesc = 0;
   mNumDim = 0;
   mNumTilesTotal = 0;
   mNumVchunksTotal = 0;
   mNumTilesTransfered = 0;
   mpBufDesc = 0;
   mpCircTileSmartBufSrc = 0;
   mpCircTileSmartBufDst = 0;
   mpTilingDesc = 0;
   mpTileVectorizationDesc = 0;
   mpVectorMapDesc = 0;
   mNumTilesToTransfer = 1;
   mpTilePtrArraySrc = 0;
   mpTilePtrArrayDst = 0;
   mVecTransferPattern = VEC_TRANSFER_PATTERN_UNDEF;
   mConsId = 0;

   mPatchOffsetRowTileCnt = 0;
   mPatchOffsetRowCnt = 0;
   mPatchOffsetColCnt = 0;
   mPatchOffsetArray = 0;
}

//NOTE: this is a general implementation for use with tiles or vchunks; can implement more optimized versions for known use-cases as required
int32_t DataTransfer::CalcNextChunkOffsetInDataType(int32_t lStepDim[], int32_t lStepSize[], DataDesc* lDataDesc, int32_t lChunkLoc[], int32_t lNumChunksInDim[])
{
   int32_t lChunkLocRet[DESC_MAX_NUM_DIM] = { 0 }; //this is what will be returned (update applies to next iteration!)
   for (int32_t i = 0; i < mNumDim; i++)
      lChunkLocRet[i] = lChunkLoc[i];

   int32_t lAllDimMaxFlag = 1;

   for (int32_t i = 0; i < mNumDim; i++)
   {
      int32_t lDim = lStepDim[i];

      //if current dimension is not at MAX
      if (lChunkLoc[lDim] < (lNumChunksInDim[lDim] - 1))
      {
         lAllDimMaxFlag = 0;

         //increment current dimension
         lChunkLoc[lDim]++;

         //reset all previous dimensions
         for (int32_t j = 0; j < i; j++)
            lChunkLoc[lStepDim[j]] = 0;

         break;
      }
   }

   //based on lTileLocRet calculate offset
   int32_t lTileOffsetInDataType = 0;
   for (int32_t i = 0; i < mNumDim; i++)
   {
      lTileOffsetInDataType += lChunkLocRet[i] * lStepSize[i] * lDataDesc->mElementStrideInDataType[i];

      //printf("### i=%d: lChunkLocRet[]=%d lStepSize[]=%d mElementStrideInDataType[]=%d mElementSize[]=%d mStrideInDataType[]=%d\n",
      //      i, lChunkLocRet[i], lStepSize[i], lDataDesc->mElementStrideInDataType[i], lDataDesc->mElementSize[i], lDataDesc->mStrideInDataType[i]);
   }

#if (DEBUG_LVL > 1)
   for (int32_t i = 0; i < mNumDim; i++)
      printf("%d ", lChunkLocRet[i]);
   printf("]\n");
#endif

   //auto reset the state when all dimensions are maxed
   if (lAllDimMaxFlag)
   {
      for (int32_t i = 0; i < mNumDim; i++)
         lChunkLoc[i] = 0;
   }

   return lTileOffsetInDataType;
}

int32_t DataTransfer::CalcNextTileOffsetInDataType()
{
   int32_t lOffset = 0;

   if (0 != mpTransferDesc->mpPatchOffsetArray) //if 2D patch processing transfer
   {
      lOffset = 0; //tile offset is constant at 0 since patch offsets are relative to a common start point (i.e. not relative to the start of a tile)
   }
   else
   {
      /*if (mpTilingDesc->mNumTileUserDef != 0)
      {     
         //use mTileLoc[0] to track running index
         lOffset = mpTilingDesc->mTileOffsetArrayUserDef[mTileLoc[0]];
         mTileLoc[0]++;
         if (mTileLoc[0] == mpTilingDesc->mNumTileUserDef)
            mTileLoc[0] = 0;
      }
      else*/
      {
#if (DEBUG_LVL > 1)
         printf("DataTransfer::CalcNextTileOffsetInDataType -> lTileLoc=[ ");
#endif
         lOffset = CalcNextChunkOffsetInDataType(
            mpTilingDesc->mStepDim,
            mpTilingDesc->mStepSize,
            mpBufDesc->mpDataDesc,
            mTileLoc,
            mNumTileInDim);
      }
   }

   return lOffset;
}

int32_t DataTransfer::CalcNextVchunkOffsetInDataType()
{
   int32_t lOffset = 0;

   if (0 != mpTransferDesc->mpPatchOffsetArray) //if 2D patch processing transfer
   {
      int32_t lIdx = mPatchOffsetColCnt; //single offset row version
      //int32_t lIdx = (mPatchOffsetRowCnt*mNumVchunksTotal) + mPatchOffsetColCnt; //full offset version
      
      lOffset = mPatchOffsetArray[lIdx];
      if (1 == mpTransferDesc->mPatchOffsetInBytesFlag) //for compatibility with ACF...
         lOffset = lOffset / (RetDataTypeSizeInBytes(mpTransferDesc->mpBufDesc->mpDataDesc->mDataType));
      lOffset += mPatchTileOffsetInDataType*mPatchOffsetRowTileCnt;

      mPatchOffsetColCnt++;

      if (mPatchOffsetColCnt == mNumVchunksTotal)
      {
         mPatchOffsetColCnt = 0;
         mPatchOffsetRowTileCnt++;

         if (mPatchOffsetRowTileCnt == mNumTilesPerPatch)
         {
            mPatchOffsetRowCnt++;
            mPatchOffsetRowTileCnt = 0;

            //read in offsets for the next row
            SCL_XFER_32(
               (int32_t*)mpTransferDesc->mpPatchOffsetArray->mDataAddress + mPatchOffsetRowCnt*mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mStrideInDataType[1],
               mPatchOffsetArray,
               MEM_TYPE_XMEM_SCL, MEM_TYPE_DMEM_SCL,
               mpTransferDesc->mpPatchOffsetArray->mpDataDesc->mSize[0]);
         }
      }
   }
   else
   {
      /*if (mpTileVectorizationDesc->mNumVchunkPerTileUserDef != 0)
      {
         //use mVchunkLoc[0] to track running index
         lOffset = mpTileVectorizationDesc->mVchunkOffsetArrayUserDef[mVchunkLoc[0]];
         mVchunkLoc[0]++;
         if (mVchunkLoc[0] == mpTileVectorizationDesc->mNumVchunkUserDef)
            mVchunkLoc[0] = 0;
      }
      else*/
      {
#if (DEBUG_LVL > 1)
         printf("DataTransfer::CalcNextVchunkOffsetInDataType -> lTileLoc=[ ");
#endif
         lOffset = CalcNextChunkOffsetInDataType(
            mpTileVectorizationDesc->mStepDim,
            mpTileVectorizationDesc->mStepSize,
            mpBufDesc->mpDataDesc,
            mVchunkLoc,
            mNumVchunkInDim);
      }
   }

   return lOffset;
}

int32_t DataTransfer::CalcNextVchunkOffsetInDataType(TileVectorizationDesc* lpTileVectorizationDesc, DataDesc* lTargetRegionDesc)
{
   int32_t lOffset = CalcNextChunkOffsetInDataType(
      lpTileVectorizationDesc->mStepDim,
      lpTileVectorizationDesc->mStepSize,
      lTargetRegionDesc,
      mVchunkLoc,
      mNumVchunkInDim);

   return lOffset;
}

int32_t DataTransfer::DetermineOptimalVecTransferPattern()
{
   int32_t lRetVal = 0;

   mVecTransferPattern = VEC_TRANSFER_PATTERN_UNDEF;

   return lRetVal;
}
