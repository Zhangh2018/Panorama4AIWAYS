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
#include <circ_tile_buf.hpp>
#include <data_desc.hpp>
#include <vector_map_desc.hpp>
#include <apu_mem_mgmt.hpp>

//----------------------------
//power of 2 helper functions
//----------------------------

int32_t IsPowerOfTwo(int32_t n)
{
   return (n != 0) && ((n & (n - 1)) == 0);
}

int32_t RetNextPowerOfTwo(int32_t n)
{
   if (0 == n)
      n += 1;

   --n;

   n |= n >> 1;
   n |= n >> 2;
   n |= n >> 4;
   n |= n >> 8;
   n |= n >> 16;

   return n + 1;
}

int32_t IsVec(MEM_TYPE lBufType)
{
   return lBufType & 0x10;
}

//----------------------------
//CircTileBuf
//----------------------------

int32_t CircTileBuf::CreateDmem(int32_t lNumTiles, DataDesc* lpTileDesc)
{
   int lRetVal = 0;

   Init();

   mBufType = MEM_TYPE_DMEM_SCL;
   mNumTiles = lNumTiles;
   mpTileDesc = lpTileDesc;
   mNumVchunksInTile = 1;

   mTileBuffer = (void*)ApuMemAllocSclDmem(lpTileDesc->mTotalSizeInBytes*lNumTiles);

   if (0 != mTileBuffer)
   {
      lRetVal = DerivedInit();
   }
   else //memory allocation failure
   {
      printf("CircTileBuf::CreateDmem -> memory allocation failure!\n");
      lRetVal = 1;
   }

   return lRetVal;
}

int32_t CircTileBuf::CreateSmem(int32_t lNumTiles, DataDesc* lpTileDesc)
{
   int lRetVal = 0;

   Init();

   mBufType = MEM_TYPE_SMEM_SCL;
   mNumTiles = lNumTiles;
   mpTileDesc = lpTileDesc;
   mNumVchunksInTile = 1;

   mTileBuffer = (void*)ApuMemAllocSclSmem(lpTileDesc->mTotalSizeInBytes*lNumTiles);

   if (0 != mTileBuffer)
   {
      lRetVal = DerivedInit();
   }
   else //memory allocation failure
   {
      printf("CircTileBuf::CreateSmem -> memory allocation failure!\n");
      lRetVal = 1;
   }

   return lRetVal;
}

int32_t CircTileBuf::CreateCmem(int32_t lNumTiles, DataDesc* lpVchunkDesc, int32_t lNumVchunksInTile, VectorMapDesc* lpVecMapDesc)
{
   int lRetVal = 0;

   Init();

   mBufType = MEM_TYPE_CMEM_VEC;
   mNumTiles = lNumTiles;
   mpTileDesc = lpVchunkDesc;
   mNumVchunksInTile = lNumVchunksInTile;
   mpVectorMapDesc = lpVecMapDesc;

   //calculate tile wrap
   int lReqVchunkInclVecPad = mNumVchunksInTile + mpVectorMapDesc->mVecPad[0] + mpVectorMapDesc->mVecPad[1];
   mTileWrapCnt = (lReqVchunkInclVecPad - 1) / mpVectorMapDesc->mPhysVecSize;

   //allocate tile buffer
   //- vector allocation with c_malloc is per-CU (e.g. if i allocate 64 bytes, i'm allocating 64 bytes per CU)
   //- for GOC modes convert GOC size to per-CU size for allocation purposes (assuming 4 bytes/CU)

   const int lNumBytesPerCu = 4;
   int32_t lNumFourByteSeg = (lpVchunkDesc->mTotalSizeInBytes + (lNumBytesPerCu-1)) / lNumBytesPerCu;
   int32_t lTileSizeInBytesPerCu = 0;

   switch (lpVecMapDesc->mVecMode)
   {
   case VEC_MODE_CU:    lTileSizeInBytesPerCu = (lpVchunkDesc->mTotalSizeInBytes);            break;
   case VEC_MODE_GOC4:  lTileSizeInBytesPerCu = lNumBytesPerCu*((lNumFourByteSeg + 3) / 4);   break;
   case VEC_MODE_GOC8:  lTileSizeInBytesPerCu = lNumBytesPerCu*((lNumFourByteSeg + 7) / 8);   break;
   default:
      printf("CircTileBuf::CreateCmem -> unrecognized vector mode!\n");
      lRetVal = 1;
      break;
   }

   int32_t lAllocSizeInBytesPerCu = lTileSizeInBytesPerCu*lNumTiles*(mTileWrapCnt + 1);

   if (0 == lRetVal)
   {
      switch (lpVchunkDesc->mDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
         mTileBuffer = (void*)ApuMemAllocVec08(lAllocSizeInBytesPerCu);
         break;

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
         mTileBuffer = (void*)ApuMemAllocVec16(lAllocSizeInBytesPerCu);
         break;

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
         mTileBuffer = (void*)ApuMemAllocVec32(lAllocSizeInBytesPerCu);
         break;

      default:
         printf("CircTileBuf::CreateCmem -> unrecognized data type!\n");
         lRetVal = 1;
         break;
      }
   }

   //allocate mpTileSegmentVecIdxStart and mpTileSegmentVecWidth (help to deal with vector wrapping)
   mpTileSegmentVecIdxStart = (int32_t*)ApuMemAllocSclDmem(sizeof(int32_t)*(mTileWrapCnt+1));
   mpTileSegmentVecWidth = (int32_t*)ApuMemAllocSclDmem(sizeof(int32_t)*(mTileWrapCnt+1));

   if (mTileBuffer != 0 && mpTileSegmentVecIdxStart != 0 && mpTileSegmentVecWidth != 0)
   {
      int32_t lVchunkCnt = 0; //keep a running count to make the following calculation easier

      for (int32_t i=0; i<(mTileWrapCnt+1); i++)
      {
         if (0 == i) //first
         {
            mpTileSegmentVecIdxStart[i] = mpVectorMapDesc->mVecPad[0];

            if (0 == mTileWrapCnt) mpTileSegmentVecWidth[i] = mNumVchunksInTile;
            else                   mpTileSegmentVecWidth[i] = mpVectorMapDesc->mPhysVecSize - mpVectorMapDesc->mVecPad[0];
         }
         else //middle and last
         {
            mpTileSegmentVecIdxStart[i] = 0;

            if (mTileWrapCnt == i) //last
            {
               mpTileSegmentVecWidth[i] = mNumVchunksInTile - lVchunkCnt;
            }
            else //middle
            {
               mpTileSegmentVecWidth[i] = mpVectorMapDesc->mPhysVecSize;
            }
         }

         lVchunkCnt += mpTileSegmentVecWidth[i];
      }

      lRetVal = DerivedInit();
   }
   else //memory allocation failure
   {
      printf("CircTileBuf::CreateCmem -> memory allocation failure!\n");

      switch (mpTileDesc->mDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
         ApuMemFree((vec08s*)mTileBuffer);
         break;

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
         ApuMemFree((vec16s*)mTileBuffer);
         break;

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
         ApuMemFree((vec32s*)mTileBuffer);
         break;
      }

      ApuMemFree((int8_t*)mpTileSegmentVecIdxStart);
      ApuMemFree((int8_t*)mpTileSegmentVecWidth);

      lRetVal = 1;
   }

   return lRetVal;
}

int32_t CircTileBuf::Destroy()
{
   switch (mpTileDesc->mDataType)
   {
   case icp::DATATYPE_08S:
   case icp::DATATYPE_08U:
      ApuMemFree((vec08s*)mTileBuffer);
      break;

   case icp::DATATYPE_16S:
   case icp::DATATYPE_16U:
      ApuMemFree((vec16s*)mTileBuffer);
      break;

   case icp::DATATYPE_32S:
   case icp::DATATYPE_32U:
      ApuMemFree((vec32s*)mTileBuffer);
      break;
   }

   ApuMemFree((int8_t*)mTilePtrArray);
   ApuMemFree((int8_t*)mpTileSegmentVecIdxStart);
   ApuMemFree((int8_t*)mpTileSegmentVecWidth);
   ApuMemFree((int8_t*)mTileInfoArray);

   return 0;
}

void* CircTileBuf::RetTilePtr(int32_t lTileIdx)
{
   return mTilePtrArray[((*mpRetTileIdx)(this, lTileIdx))];
}

void CircTileBuf::GetTilePtrArray(int32_t lTileIdx, int32_t lNumTiles, void* lTilePtrArray[])
{
   for (int32_t i = 0; i < lNumTiles; i++)
   {
      lTilePtrArray[i] = RetTilePtr(lTileIdx + i);
   }
}

int32_t CircTileBuf::QueryTileSegmentVecIdxStart(int32_t lWrapIdx)
{
   return mpTileSegmentVecIdxStart[lWrapIdx];
}

int32_t CircTileBuf::QueryTileSegmentVecWidth(int32_t lWrapIdx)
{
   return mpTileSegmentVecWidth[lWrapIdx];
}

int32_t CircTileBuf::RetTileWrapCnt()
{
   return mTileWrapCnt;
}

void CircTileBuf::SetTileInfo(TileInfo* lpTileInfo, int32_t lIdx)
{
   for (int32_t i = 0; i < mpTileDesc->mNumDim; i++)
      mTileInfoArray[((*mpRetTileIdx)(this, lIdx))].mTileLoc[i] = lpTileInfo->mTileLoc[i];
}

TileInfo* CircTileBuf::GetTileInfo(int32_t lIdx)
{
   return &mTileInfoArray[((*mpRetTileIdx)(this, lIdx))];
}

void CircTileBuf::Init()
{
   mNumTiles = 0;
   mpTileDesc = 0;
   mTileBuffer = 0;
   mPow2Mask = 0;
   mpVectorMapDesc = 0;
   mTileWrapCnt = 0;
   mNumVchunksInTile = 0;
   mTilePtrArray = 0;
   mpTileSegmentVecIdxStart = 0;
   mpTileSegmentVecWidth = 0;
   mTileInfoArray = 0;
   mpRetTileIdx = 0;
}

int32_t RetTileIdx_Pow2(CircTileBuf* lpBuf, int32_t lTileIdx)
{
   return (lTileIdx & lpBuf->mPow2Mask);
}
int32_t RetTileIdx_Flex(CircTileBuf* lpBuf, int32_t lTileIdx)
{
   //TODO: need to improve this version; don't want to use div or mod!!
   return (lTileIdx % lpBuf->mNumTiles);
}

uintptr_t RetGocAddr(uintptr_t lCuAddr, VEC_MODE lVecMode)
{
   uintptr_t lGocAddr = 0;
   switch(lVecMode)
   {
   case VEC_MODE_GOC4: lGocAddr = (lCuAddr<<2) | (1<<19) | (1<<18); break;
   case VEC_MODE_GOC8: lGocAddr = (lCuAddr<<3) | (1<<19);           break;
   default:            printf("RetGocAddr: Invalid vector mode!\n");
   }

   return lGocAddr;
}

int32_t CircTileBuf::DerivedInit()
{
   int32_t lRetVal = 0;

   //use optimal index wrapping method if number of tiles is a power of 2
   if (IsPowerOfTwo(mNumTiles))
   {
      //printf("Selected pow2 version for %d tiles!\n", mNumTiles);
      mPow2Mask = mNumTiles - 1; //calculate power of 2 mask
      mpRetTileIdx = &RetTileIdx_Pow2;
   }
   else //not a power of 2
   {
      //printf("Selected flex version for %d tiles!\n", mNumTiles);
      mpRetTileIdx = &RetTileIdx_Flex;
   }

   //allocate and fill in mTilePtrArray and mTileInfoPtrArray
   mTilePtrArray = (void**)ApuMemAllocSclDmem(sizeof(void*)*mNumTiles);
   mTileInfoArray = (TileInfo*)ApuMemAllocSclDmem(sizeof(TileInfo)*mNumTiles);

   if ((0 != mTilePtrArray) && (0 != mTileInfoArray))
   {
      for (int32_t i = 0; i < mNumTiles; i++)
      {
         if (0 == IsVec(mBufType)) //if scalar (using byte arithmetic and assuming contiguous scalar memory)
         {
            mTilePtrArray[i] = 
               (void*)(((intptr_t)mTileBuffer) + 
               (mpTileDesc->mTotalSizeInBytes*(mTileWrapCnt + 1)*i) + 
               (mpTileDesc->mCoreOffsetInDataType*RetDataTypeSizeInBytes(mpTileDesc->mDataType)));
         }
         else //vector (using typed pointer arithmetic)
         {
            int32_t lTileOffsetInDataType = (mpTileDesc->mTotalSizeInDataType*(mTileWrapCnt + 1)*i) + mpTileDesc->mCoreOffsetInDataType;

            switch (mpTileDesc->mDataType)
            {
            case icp::DATATYPE_08S:
            case icp::DATATYPE_08U:

               if (IsGocMode(mpVectorMapDesc->mVecMode))
                  mTilePtrArray[i] = (void*)(((vec08s*)RetGocAddr((uintptr_t)mTileBuffer, mpVectorMapDesc->mVecMode)) + lTileOffsetInDataType);
               else
                  mTilePtrArray[i] = (void*)((vec08s*)mTileBuffer + lTileOffsetInDataType);
               break;
            case icp::DATATYPE_16S:
            case icp::DATATYPE_16U:

               if (IsGocMode(mpVectorMapDesc->mVecMode))
                  mTilePtrArray[i] = (((vec16s*)RetGocAddr((uintptr_t)mTileBuffer, mpVectorMapDesc->mVecMode)) + lTileOffsetInDataType);
               else
                  mTilePtrArray[i] = (void*)((vec16s*)mTileBuffer + lTileOffsetInDataType);
               break;
            case icp::DATATYPE_32S:
            case icp::DATATYPE_32U:
               if (IsGocMode(mpVectorMapDesc->mVecMode))
                  mTilePtrArray[i] = (((vec32s*)RetGocAddr((uintptr_t)mTileBuffer, mpVectorMapDesc->mVecMode)) + lTileOffsetInDataType);
               else
                  mTilePtrArray[i] = (void*)((vec32s*)mTileBuffer + lTileOffsetInDataType);
               break;
            default:
               break;
            }
         }
      }
   }
   else //memory allocation failure
   {
      ApuMemFree((int8_t*)mTilePtrArray);
      ApuMemFree((int8_t*)mTileInfoArray);

      printf("CircTileBuf::DerivedInit -> memory allocation failure!\n");
      lRetVal = 1;
   }

   return lRetVal;
}
