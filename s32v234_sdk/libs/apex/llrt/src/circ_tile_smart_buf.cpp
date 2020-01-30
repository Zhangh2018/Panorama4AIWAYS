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
#include <circ_tile_smart_buf.hpp>
#include <vector_map_desc.hpp>
#include <data_desc.hpp>
#include <apu_test_util.h> //optimized pad and copy kernels (apu2 implementations)

#define LOCAL_VEC_PADDING 1

void CircTileSmartBuf::Init()
{
   mNumCons = 0;

   for (int32_t i = 0; i < MAX_NUM_CONS; i++)
   {
      mConsCounter[i] = 0;
      mNumTilesAvailForCons[i] = 0;
      mFirstFlag[i] = 1;
      mNumTilesContEdgePadTConsSpec[i] = 0;
      mNumTilesContEdgePadBConsSpec[i] = 0;
      mConsOrder[i] = i;
      mConsGateFlag[i] = 0;
   }

   mProdCounter = 0;
   mNumTilesAvailForProd = 0;
   mProdDoneFlag = 0;
   mTileEdgePadT = 0;
   mTileEdgePadB = 0;
   mTileEdgePadL = 0;
   mTileEdgePadR = 0;
   mPadDimTB = 0;
   mPadDimLR = 0;
   mPadEnabledFlag = 0;
   mTileEdgePadType = EDGE_PAD_TYPE__UNDEFINED;
   mPadTopCompleteFlag = 0;
   mPadBotCompleteFlag = 0;
   mNumTilesContEdgePadT = 0;
   mNumTilesContEdgePadB = 0;
}

void CircTileSmartBuf::DerivedInit()
{
   mNumTilesAvailForProd = mCircTileBuf.mNumTiles;

   //initializing like this so if we need to go backwards for top padding, the number is still positive!
   for (int32_t i = 0; i < MAX_NUM_CONS; i++)
      mConsCounter[i] = mCircTileBuf.mNumTiles;
   mProdCounter = mCircTileBuf.mNumTiles;
}

int32_t CircTileSmartBuf::CreateDmem(int32_t lNumTiles, DataDesc* lpTileDesc)
{
   int lRetVal = 0;

   Init();

   if (0 != mCircTileBuf.CreateDmem(lNumTiles, lpTileDesc))
   {
      lRetVal = 1;
   }

   DerivedInit();

   return lRetVal;
}

int32_t CircTileSmartBuf::CreateSmem(int32_t lNumTiles, DataDesc* lpTileDesc)
{
   int lRetVal = 0;

   Init();

   if (0 != mCircTileBuf.CreateSmem(lNumTiles, lpTileDesc))
   {
      lRetVal = 1;
   }

   DerivedInit();

   return lRetVal;
}

int32_t CircTileSmartBuf::CreateCmem(int32_t lNumTiles, DataDesc* lpVchunkDesc, int32_t lNumVchunksInTile, VectorMapDesc* lpVecMapDesc)
{
   int lRetVal = 0;

   Init();

   if (0 != mCircTileBuf.CreateCmem(lNumTiles, lpVchunkDesc, lNumVchunksInTile, lpVecMapDesc))
   {
      lRetVal = 1;
   }

   DerivedInit();

   return lRetVal;
}

int32_t CircTileSmartBuf::Destroy()
{
   return mCircTileBuf.Destroy();
}

int32_t CircTileSmartBuf::ReqConsId(int32_t* lpConsId)
{
   int32_t lRetVal = 0;
   if ((mNumCons + 1) <= MAX_NUM_CONS)
   {
      *lpConsId = mNumCons++;
   }
   else
   {
      printf("CircTileSmartBuf::ReqConsId -> exceeded max num concurrent consumers!\n");
      lRetVal = 1;
   }
   return lRetVal;
}

void CircTileSmartBuf::TilesProd(int32_t lNumTilesProd, int32_t lLastFlag)
{
   //TODO: if there is bottom padding, last should not really be set here since since the real last producer will be the padding function

   mProdCounter += lNumTilesProd;
   for (int32_t i = 0; i<mNumCons; i++)
      mNumTilesAvailForCons[i] += lNumTilesProd;
   mNumTilesAvailForProd -= lNumTilesProd;
   mProdDoneFlag = lLastFlag;

   if (mPadEnabledFlag)
   {
      if (mTileEdgePadL || mTileEdgePadR)
         PerformTilePaddingLR(lNumTilesProd);
   }
}

void CircTileSmartBuf::TilesCons(int32_t lNumTilesCons, int32_t lConsId)
{
   mFirstFlag[lConsId] = (lNumTilesCons > 0) ? 0 : 1;
   mNumTilesAvailForCons[lConsId] -= lNumTilesCons;

   if (1 == mNumCons) //single consumer fast track
   {
      mConsCounter[lConsId] += lNumTilesCons;
      mNumTilesAvailForProd += lNumTilesCons;
   }
   else //multiple consumers on this buffer (can only increment mNumTilesAvailForProd if ALL consumers have consumed)
   {
      //if consumer lConsId is the slowest
      if (mConsOrder[0] == lConsId)
      {
         int32_t lDiff = mConsCounter[mConsOrder[1]] - mConsCounter[lConsId];
         if (lNumTilesCons >= lDiff)
            mNumTilesAvailForProd += lDiff;
         else
            mNumTilesAvailForProd += lNumTilesCons;
      }

      mConsCounter[lConsId] += lNumTilesCons;

      //adjust position of lConsId in the 'mConsOrder' list so it stays in order
      for (int32_t i = 0; i < mNumCons - 1; i++)
      {
         if (lConsId == mConsOrder[i])
         {
            if (mConsCounter[mConsOrder[i]] >  mConsCounter[mConsOrder[i + 1]])
            {
               int32_t lTmp = mConsOrder[i + 1];
               mConsOrder[i + 1] = mConsOrder[i];
               mConsOrder[i] = lTmp;
            }
         }
      }
   }
}

int32_t CircTileSmartBuf::QueryTilesAvailForProd(int32_t lNumTiles)
{
   return (lNumTiles <= mNumTilesAvailForProd) ? (1) : (0);
}

int32_t CircTileSmartBuf::QueryTilesAvailForCons(int32_t lNumTiles, int32_t lConsId)
{
   int32_t lAvailFlag = 0;

   if (mPadEnabledFlag)
   {
      if (mTileEdgePadT && (0 == mPadTopCompleteFlag))
         PerformTilePaddingTop();

      //this is here because it may not be possible to fully complete bottom padding during the producers time slot; it will only be done
      //once there is enough space to accommodate it (may need to wait for additional tiles to be consumed to make room)
      if (mTileEdgePadB && (0 == mPadBotCompleteFlag) && mProdDoneFlag)
         PerformTilePaddingBot();      
   }

   //if consumption is gated (e.g. top padding is required), this function must return that
   //nothing is available until it is ungated (e.g. padding is actually performed)
   if (1 == mConsGateFlag[lConsId])
      lAvailFlag = 0;
   else
      lAvailFlag = (lNumTiles <= mNumTilesAvailForCons[lConsId]) ? (1) : (0);

   return lAvailFlag;
}

void* CircTileSmartBuf::RetTilePtrForProd()
{
   return mCircTileBuf.RetTilePtr(mProdCounter);
}

void* CircTileSmartBuf::RetTilePtrForCons(int32_t lConsId)
{
   return mCircTileBuf.RetTilePtr(mConsCounter[lConsId]);
}

void CircTileSmartBuf::GetTilePtrArrayForProd(int32_t lNumTiles, void* lTilePtrArray[])
{
   mCircTileBuf.GetTilePtrArray(mProdCounter, lNumTiles, lTilePtrArray);
}

int32_t CircTileSmartBuf::RetTileWrapCnt()
{
   return mCircTileBuf.RetTileWrapCnt();
}

void CircTileSmartBuf::GetTilePtrArrayForCons(int32_t lNumTiles, void* lTilePtrArray[], int32_t lConsId)
{
   mCircTileBuf.GetTilePtrArray(mConsCounter[lConsId], lNumTiles, lTilePtrArray);
}

int32_t CircTileSmartBuf::QueryNumTilesAvailForProd()
{
   return mNumTilesAvailForProd;
}

int32_t CircTileSmartBuf::QueryNumTilesAvailForProdContiguous()
{
   return mCircTileBuf.mNumTiles - (mProdCounter & mCircTileBuf.mPow2Mask);
}

int32_t CircTileSmartBuf::QueryDone(int32_t lConsId)
{
   return (mProdDoneFlag && (0 == (mNumTilesAvailForCons[lConsId] - mNumTilesContEdgePadTConsSpec[lConsId] - mNumTilesContEdgePadBConsSpec[lConsId])));
}

int32_t CircTileSmartBuf::QueryLastCons(int32_t lNumTiles, int32_t lConsId)
{
   return (mProdDoneFlag && (lNumTiles >= (mNumTilesAvailForCons[lConsId] - mNumTilesContEdgePadTConsSpec[lConsId] - mNumTilesContEdgePadBConsSpec[lConsId])));
}

int32_t CircTileSmartBuf::QueryFirstCons(int32_t lConsId)
{
   return (1 == mFirstFlag[lConsId]);
}

void CircTileSmartBuf::SetTileInfo(TileInfo* lpTileInfo, int32_t lIdx)
{
   mCircTileBuf.SetTileInfo(lpTileInfo, mProdCounter + lIdx);
}

TileInfo* CircTileSmartBuf::GetTileInfo(int32_t lIdx, int32_t lConsId)
{
   return mCircTileBuf.GetTileInfo(mConsCounter[lConsId] + lIdx);
}

void CircTileSmartBuf::Reset()
{
   for (int32_t i = 0; i < mNumCons; i++)
   {
      mNumTilesAvailForCons[i] = 0;
      mFirstFlag[i] = 1;
      mConsOrder[i] = i;

      if (mNumTilesContEdgePadTConsSpec[i])
         mConsGateFlag[i] = 1;
   }

   DerivedInit(); //takes care of mNumTilesAvailForProd, mConsCounter, mProdCounter

   mProdDoneFlag = 0;
   mPadTopCompleteFlag = 0;
   mPadBotCompleteFlag = 0;
   mNumTilesAvailForProd -= mNumTilesContEdgePadT;
}

int32_t CircTileSmartBuf::EnableTileEdgePadding2D(int32_t lTop, int32_t lBottom, int32_t lLeft, int32_t lRight, EDGE_PAD_TYPE lEdgePadType, int32_t lConsId)
{
   int32_t lRetVal = 0;

   mPadEnabledFlag = 1;

   //only configure padding if it has never been configured before
   if (EDGE_PAD_TYPE__UNDEFINED == mTileEdgePadType)
      mTileEdgePadType = lEdgePadType;

   //since this function can be called multiple times by multiple consumers, make sure that the padding type of subsequent calls matches with previous calls
   if (lEdgePadType != mTileEdgePadType)
   {
      printf("CircTileSmartBuf::EnableTileEdgePadding2D -> multiple padding types have been requested on this buffer; only a single padding type is supported\n");
      lRetVal = 1;
   }

   //if this is called multiple times, take the worst case size
   //e.g. consider case where two consumers with different padding sizes consume off the same buffer
   mTileEdgePadT = (lTop > mTileEdgePadT) ? lTop : mTileEdgePadT;
   mTileEdgePadB = (lBottom > mTileEdgePadB) ? lBottom : mTileEdgePadB;
   mTileEdgePadL = (lLeft > mTileEdgePadL) ? lLeft : mTileEdgePadL;
   mTileEdgePadR = (lRight > mTileEdgePadR) ? lRight : mTileEdgePadR;
   mPadDimTB = 1; //assuming [x,y]
   mPadDimLR = 0; //assuming [x,y]

   int32_t lTileSizeInPadDim = mCircTileBuf.mpTileDesc->mSize[mPadDimTB];
   int32_t mNumTilesContEdgePadTPrev = mNumTilesContEdgePadT;
   mNumTilesContEdgePadT = (mTileEdgePadT + lTileSizeInPadDim - 1) / lTileSizeInPadDim;
   mNumTilesContEdgePadB = (mTileEdgePadB + lTileSizeInPadDim - 1) / lTileSizeInPadDim;

   //need to keep track of top and bottom per consumer!!!
   mNumTilesContEdgePadTConsSpec[lConsId] = (lTop + lTileSizeInPadDim - 1) / lTileSizeInPadDim;
   mNumTilesContEdgePadBConsSpec[lConsId] = (lBottom + lTileSizeInPadDim - 1) / lTileSizeInPadDim;

   //if there is top padding, decrease the tiles avail to the producer by the worst case size (it's important
   //that the adjustment be done NOW (i.e. at initialization) to prevent the producer from filling in the tiles
   //reserved for top padding)
   if (mNumTilesContEdgePadT > mNumTilesContEdgePadTPrev)
      mNumTilesAvailForProd -= (mNumTilesContEdgePadT - mNumTilesContEdgePadTPrev);

   //if there is top padding, gate consumption until top padding has been completed
   if (lTop)
   {
      mConsGateFlag[lConsId] = 1;
   }

   return lRetVal;
}

void CircTileSmartBuf::TopPaddingComplete()
{
   mPadTopCompleteFlag = 1;

   //it is assumed padding is shared for all consumers: ungate them all and adjust consumer state info
   for (int i = 0; i < mNumCons; i++)
   {
      //TODO: still may want to bring first two back to the other padding config function!!!
      mConsCounter[i] -= mNumTilesContEdgePadTConsSpec[i]; //pull back consumer - this should never be negative (initialized to the number of tiles in the CB)
      mNumTilesAvailForCons[i] += mNumTilesContEdgePadTConsSpec[i];
      mConsGateFlag[i] = 0;

      //adjust mConsOrder based on top padding consumer pull back (always ordered from low to high so full sort is not required)
      for (int32_t k = mNumCons - 1; k > 0; k--)
      {
         if (i == mConsOrder[k])
         {
            if (mConsCounter[mConsOrder[k]] < mConsCounter[mConsOrder[k - 1]])
            {
               int32_t lTmp = mConsOrder[k - 1];
               mConsOrder[k - 1] = mConsOrder[k];
               mConsOrder[k] = lTmp;
            }
         }
      }
   }
}

void CircTileSmartBuf::GetTilePtrArrayBehindProd(int32_t lNumTilesBehindProd, void* lTilePtrArray[])
{
   //(mProdCounter - lNumTilesBehindProd) should never be negative (initialized to the number of tiles in the CB)
   mCircTileBuf.GetTilePtrArray(mProdCounter - lNumTilesBehindProd, lNumTilesBehindProd, lTilePtrArray);
}

void CircTileSmartBuf::GetTilePtrArrayBehindCons(int32_t lNumTilesBehindCons, void* lTilePtrArray[], int32_t lConsId)
{
   //(mConsCounter[lConsId] - lNumTilesBehindCons) should never be negative (initialized to the number of tiles in the CB)
   mCircTileBuf.GetTilePtrArray(mConsCounter[lConsId] - lNumTilesBehindCons, lNumTilesBehindCons, lTilePtrArray);
}

void CircTileSmartBuf::GetTilePtrArrayAtCoreStart(int32_t lNumTiles, void* lTilePtrArray[])
{
   mCircTileBuf.GetTilePtrArray(0, lNumTiles, lTilePtrArray);
}

void CircTileSmartBuf::GetTilePtrArrayBehindCoreStart(int32_t lNumTiles, void* lTilePtrArray[])
{
   mCircTileBuf.GetTilePtrArray(mCircTileBuf.mNumTiles - lNumTiles, lNumTiles, lTilePtrArray);
}

int32_t CircTileSmartBuf::QueryTilesAvailForTopPaddingSrc(int32_t lNumTiles)
{
   //using consumer 0 for the query - mNumTilesAvailForCons should be the same for all consumers before top padding anyways
   return (lNumTiles <= mNumTilesAvailForCons[0]) ? (1) : (0);
}

void CircTileSmartBuf::PerformTilePaddingTop()
{
   //NOTE: assumes L/R padding has already been performed!!

   if (EDGE_PAD_TYPE__REPLICATE == mTileEdgePadType)
   {
      //need a single source tile for replication
      if (QueryTilesAvailForTopPaddingSrc(1))
      {
         PadTop_Replicate();
         TopPaddingComplete();
      }
   }
   else if (EDGE_PAD_TYPE__ZERO == mTileEdgePadType)
   {
      //no source tiles are needed for zero padding
      PadTop_Zero();
      TopPaddingComplete();
   }
}

void CircTileSmartBuf::PerformTilePaddingLR(int32_t lNumTiles)
{

#if (DEBUG_LVL > 0)
   printf("[0x%03X]CircTileSmartBuf::PerformTileEdgePaddingLR\n", (unsigned int)(((uintptr_t)this) & 0xFFF));
#endif

   if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
      PadLR_InterCu(lNumTiles);

   if (EDGE_PAD_TYPE__REPLICATE == mTileEdgePadType)
   {
      PadLR_Replicate(lNumTiles);
   }
   else if (EDGE_PAD_TYPE__ZERO == mTileEdgePadType)
   {
      PadLR_Zero(lNumTiles);
   }
}

void CircTileSmartBuf::PerformTilePaddingBot()
{
   //NOTE: assumes L/R padding has already been performed!!
   //NOTE: bottom padding is done all at once (an inremental option can be considered if it reduces memory footprint)

   if (QueryTilesAvailForProd(mNumTilesContEdgePadB))
   {
      if (EDGE_PAD_TYPE__REPLICATE == mTileEdgePadType)
      {
         PadBot_Replicate();
      }
      else if (EDGE_PAD_TYPE__ZERO == mTileEdgePadType)
      {
         PadBot_Zero();
      }

      //since we are generating padding (which will not be processed), cannot call TilesProd because it will blindly increment mNumTilesAvailForCons for all
      //consumers without consideration for bottom padding.  Use the following variation instead:
      mProdCounter += mNumTilesContEdgePadB;
      mNumTilesAvailForProd -= mNumTilesContEdgePadB;
      for (int i = 0; i < mNumCons; i++)
         mNumTilesAvailForCons[i] += mNumTilesContEdgePadBConsSpec[i];

      mPadBotCompleteFlag = 1;
   }
}

//-------------------------------------------------------------------------------------------------------------
//NOTE: All padding functions below are 2D implementations assuming dim0=x and dim1=y (with L/R on x, T/B on y)
//-------------------------------------------------------------------------------------------------------------

void CircTileSmartBuf::PadTop_Replicate()
{
   //get a single tile for padding src
   void* lpTileSrc[1];
   GetTilePtrArrayAtCoreStart(1, lpTileSrc);

   //get 'mNumTilesContEdgePadT' tiles into which padding will occur
   void* lpTileDst[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayBehindCoreStart(mNumTilesContEdgePadT, lpTileDst);

#if (DEBUG_LVL > 0)
   printf("[0x%03X]CircTileSmartBuf::PadTop_Replicate -> SRC=0x%p DST=", (unsigned int)(((uintptr_t)this) & 0xFFF), lpTileSrc[0]);
   for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
      printf("0x%p ", lpTileDst[i]);
   printf("\n");
#endif

   int32_t lXdim = 0;
   int32_t lYdim = 1;
   int32_t lStride = mCircTileBuf.mpTileDesc->mStrideInDataType[lYdim];
   int32_t lElementSizeY = mCircTileBuf.mpTileDesc->mElementSize[lYdim];
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeY = mCircTileBuf.mpTileDesc->mSize[lYdim];
   int32_t lSizeX = (mCircTileBuf.mpTileDesc->mSize[lXdim] + mTileEdgePadL + mTileEdgePadR)*lElementSizeX;
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
   {
      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         vec08u* lpSrc = (vec08u*)lpTileSrc[0] - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec08u* lpDst = (vec08u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec08u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec08u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_08(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         vec16u* lpSrc = (vec16u*)lpTileSrc[0] - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec16u* lpDst = (vec16u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec16u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec16u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_16(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         vec32u* lpSrc = (vec32u*)lpTileSrc[0] - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec32u* lpDst = (vec32u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec32u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec32u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_32(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }
      } //switch
   }
   else //scalar padding
   {
      printf("CircTileSmartBuf::PadTop_Replicate -> Scalar padding (replication) not implemented yet!!!\n");
   }
}

void CircTileSmartBuf::PadTop_Zero()
{
   //get 'mNumTilesContEdgePadT' tiles into which padding will occur
   void* lpTileDst[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayBehindCoreStart(mNumTilesContEdgePadT, lpTileDst);

#if (DEBUG_LVL > 0)
   printf("[0x%03X]CircTileSmartBuf::PadTop_Zero -> SRC=N/A DST=", (unsigned int)(((uintptr_t)this) & 0xFFF));
   for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
      printf("0x%p ", lpTileDst[i]);
   printf("\n");
#endif

   int32_t lXdim = 0;
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
   {
      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec08u* lpDst = (vec08u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec08u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_08(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec16u* lpDst = (vec16u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec16u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_16(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadT; i++)
         {
            vec32u* lpDst = (vec32u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec32u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_32(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }
      } //switch
   }
   else //scalar padding
   {
      printf("CircTileSmartBuf::PadTop_Replicate -> Scalar padding (replication) not implemented yet!!!\n");
   }
}

void CircTileSmartBuf::PadLR_Replicate(int32_t lNumTiles)
{
   void* lpTilePtr[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayBehindProd(lNumTiles, lpTilePtr);

   int32_t lXdim = 0;
   int32_t lYdim = 1;
   int32_t lStride = mCircTileBuf.mpTileDesc->mStrideInDataType[lYdim];
   int32_t lElementSizeY = mCircTileBuf.mpTileDesc->mElementSize[lYdim];
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeX = mCircTileBuf.mpTileDesc->mSize[lXdim];
   int32_t lSizeY = mCircTileBuf.mpTileDesc->mSize[lYdim];
   int32_t lTotalSizeInDataType = mCircTileBuf.mpTileDesc->mTotalSizeInDataType;
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   for (int32_t i = 0; i<lNumTiles; i++)
   {
      if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
      {

#if (1 == LOCAL_VEC_PADDING)

         //calculate right edge mask
         int32_t lCuIdx = mCircTileBuf.mpTileSegmentVecWidth[RetTileWrapCnt()] + mCircTileBuf.mpTileSegmentVecIdxStart[RetTileWrapCnt()] - 1;
         vec16u lMaskRight = 0;
         lMaskRight = vput(lMaskRight, 1, lCuIdx);

         //calculate left edge mask
         lCuIdx = mCircTileBuf.mpTileSegmentVecIdxStart[0];
         vec16u lMaskLeft = 0;
         lMaskLeft = vput(lMaskLeft, 1, lCuIdx);

         switch (lDataType)
         {
         case icp::DATATYPE_08S:
         case icp::DATATYPE_08U:
         {
            vec08u *lpSrc, *lpDst;

            //right edge
            lpDst = ((vec08u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            lpSrc = lpDst - lElementSizeX;
            apu_til_copy_right_08(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadR*lElementSizeX, lSizeY*lElementSizeY, lMaskRight);

            //left edge
            lpDst = (vec08u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            lpSrc = (vec08u*)lpTilePtr[i];
            apu_til_copy_right_08(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadL*lElementSizeX, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }

         case icp::DATATYPE_16S:
         case icp::DATATYPE_16U:
         {
            vec16u *lpSrc, *lpDst;

            //right edge
            lpDst = ((vec16u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            lpSrc = lpDst - lElementSizeX;
            apu_til_copy_right_16(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadR*lElementSizeX, lSizeY*lElementSizeY, lMaskRight);

            //left edge
            lpDst = (vec16u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            lpSrc = (vec16u*)lpTilePtr[i];
            apu_til_copy_right_16(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadL*lElementSizeX, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }

         case icp::DATATYPE_32S:
         case icp::DATATYPE_32U:
         {
            vec32u *lpSrc, *lpDst;

            //right edge
            lpDst = ((vec32u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            lpSrc = lpDst - lElementSizeX;
            apu_til_copy_right_32(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadR*lElementSizeX, lSizeY*lElementSizeY, lMaskRight);

            //left edge
            lpDst = (vec32u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            lpSrc = (vec32u*)lpTilePtr[i];
            apu_til_copy_right_32(lpDst, lStride, lpSrc, lStride, lElementSizeX, mTileEdgePadL*lElementSizeX, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }
         }

#else

         //TODO: GON implementation of L/R edge replication
         printf("CircTileSmartBuf::PadLR_Replicate -> GON padding implementation not done yet!!\n");

#endif

      }
      else //scalar padding
      {
         printf("CircTileSmartBuf::PadLR_Replicate -> Scalar padding (replication) not implemented yet!!!\n");
      }
   }
}

void CircTileSmartBuf::PadLR_Zero(int32_t lNumTiles)
{
   void* lpTilePtr[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayBehindProd(lNumTiles, lpTilePtr);

   int32_t lXdim = 0;
   int32_t lYdim = 1;
   int32_t lStride = mCircTileBuf.mpTileDesc->mStrideInDataType[lYdim];
   int32_t lElementSizeY = mCircTileBuf.mpTileDesc->mElementSize[lYdim];
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeX = mCircTileBuf.mpTileDesc->mSize[lXdim];
   int32_t lSizeY = mCircTileBuf.mpTileDesc->mSize[lYdim];
   int32_t lTotalSizeInDataType = mCircTileBuf.mpTileDesc->mTotalSizeInDataType;
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   for (int32_t i = 0; i<lNumTiles; i++)
   {
      if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
      {

#if (1 == LOCAL_VEC_PADDING)

         //calculate right edge mask
         int32_t lCuIdx = mCircTileBuf.mpTileSegmentVecWidth[RetTileWrapCnt()] + mCircTileBuf.mpTileSegmentVecIdxStart[RetTileWrapCnt()] - 1;
         vec16u lMaskRight = 0;
         lMaskRight = vput(lMaskRight, 1, lCuIdx);

         //calculate left edge mask
         lCuIdx = mCircTileBuf.mpTileSegmentVecIdxStart[0];
         vec16u lMaskLeft = 0;
         lMaskLeft = vput(lMaskLeft, 1, lCuIdx);

         switch (lDataType)
         {
         case icp::DATATYPE_08S:
         case icp::DATATYPE_08U:
         {
            vec08u *lpDst;

            //right edge
            lpDst = ((vec08u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            apu_blk_zero_08(lpDst, lStride, lElementSizeX*mTileEdgePadR, lSizeY*lElementSizeY, lMaskRight);


            //left edge
            lpDst = (vec08u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            apu_blk_zero_08(lpDst, lStride, lElementSizeX*mTileEdgePadL, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }

         case icp::DATATYPE_16S:
         case icp::DATATYPE_16U:
         {
            vec16u *lpDst;

            //right edge
            lpDst = ((vec16u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            apu_blk_zero_16(lpDst, lStride, lElementSizeX*mTileEdgePadR, lSizeY*lElementSizeY, lMaskRight);

            //left edge
            lpDst = (vec16u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            apu_blk_zero_16(lpDst, lStride, lElementSizeX*mTileEdgePadL, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }

         case icp::DATATYPE_32S:
         case icp::DATATYPE_32U:
         {
            vec32u *lpDst;

            //right edge
            lpDst = ((vec32u*)lpTilePtr[i]) + (RetTileWrapCnt()*lTotalSizeInDataType) + lElementSizeX*lSizeX;
            apu_blk_zero_32(lpDst, lStride, lElementSizeX*mTileEdgePadR, lSizeY*lElementSizeY, lMaskRight);

            //left edge
            lpDst = (vec32u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX);
            apu_blk_zero_32(lpDst, lStride, lElementSizeX*mTileEdgePadL, lSizeY*lElementSizeY, lMaskLeft);

            break;
         }
         }

#else

         //TODO: GON implementation of L/R edge zero
         printf("CircTileSmartBuf::PadLR_Zero -> GON padding implementation not done yet!!\n");

#endif

      }
      else //scalar padding
      {
         printf("CircTileSmartBuf::PadLR_Zero -> Scalar padding (replication) not implemented yet!!!\n");
      }
   }
}

void CircTileSmartBuf::PadLR_InterCu(int32_t lNumTiles)
{
   void* lpTilePtr[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayBehindProd(lNumTiles, lpTilePtr);

   int32_t lXdim = 0;
   int32_t lYdim = 1;
   int32_t lStride = mCircTileBuf.mpTileDesc->mStrideInDataType[lYdim];
   int32_t lElementSizeY = mCircTileBuf.mpTileDesc->mElementSize[lYdim];
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeX = mCircTileBuf.mpTileDesc->mSize[lXdim];
   int32_t lSizeY = mCircTileBuf.mpTileDesc->mSize[lYdim];
   int32_t lTotalSizeInDataType = mCircTileBuf.mpTileDesc->mTotalSizeInDataType;
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   for (int32_t i = 0; i<lNumTiles; i++)
   {

#if (1 == LOCAL_VEC_PADDING) //NOTE: This 'local' implementation only functions correctly assuming padding is fully satisfied by 1 neighbor CU!!

      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         vec08u *lpBlk, *lpWrapLeft, *lpWrapRight;

         for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
         {
            lpBlk = ((vec08u*)lpTilePtr[i]) + (j*lTotalSizeInDataType);
            lpWrapLeft = lpBlk;
            lpWrapRight = lpBlk;

            if (RetTileWrapCnt() > 0)
            {
               lpWrapRight = lpBlk + lTotalSizeInDataType;

               if (j > 0)
               {
                  lpWrapLeft = lpBlk - lTotalSizeInDataType + (lSizeX*lElementSizeX) - lElementSizeX;
               }
            }

            apu_blk_padhor_08(lpBlk, lpWrapLeft, lpWrapRight, lSizeX, lSizeY*lElementSizeY, lStride, lElementSizeX, mTileEdgePadL, mTileEdgePadR);
         }

         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         vec16u *lpBlk, *lpWrapLeft, *lpWrapRight;

         for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
         {
            lpBlk = ((vec16u*)lpTilePtr[i]) + (j*lTotalSizeInDataType);
            lpWrapLeft = lpBlk;
            lpWrapRight = lpBlk;

            if (RetTileWrapCnt() > 0)
            {
               lpWrapRight = lpBlk + lTotalSizeInDataType;

               if (j > 0)
               {
                  lpWrapLeft = lpBlk - lTotalSizeInDataType + (lSizeX*lElementSizeX) - lElementSizeX;
               }
            }

            apu_blk_padhor_16(lpBlk, lpWrapLeft, lpWrapRight, lSizeX, lSizeY*lElementSizeY, lStride, lElementSizeX, mTileEdgePadL, mTileEdgePadR);
         }

         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         vec32u *lpBlk, *lpWrapLeft, *lpWrapRight;

         for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
         {
            lpBlk = ((vec32u*)lpTilePtr[i]) + (j*lTotalSizeInDataType);
            lpWrapLeft = lpBlk;
            lpWrapRight = lpBlk;

            if (RetTileWrapCnt() > 0)
            {
               lpWrapRight = lpBlk + lTotalSizeInDataType;

               if (j > 0)
               {
                  lpWrapLeft = lpBlk - lTotalSizeInDataType + (lSizeX*lElementSizeX) - lElementSizeX;
               }
            }

            apu_blk_padhor_32(lpBlk, lpWrapLeft, lpWrapRight, lSizeX, lSizeY*lElementSizeY, lStride, lElementSizeX, mTileEdgePadL, mTileEdgePadR);
         }

         break;
      }
      }

#else

      //GON implementation should magically handle this ... nothing to do here???

#endif

   }
}

void CircTileSmartBuf::PadBot_Replicate()
{
   //get a single tile for padding src
   void* lpTileSrc[1];
   GetTilePtrArrayBehindProd(1, lpTileSrc);

   //get 'mNumTilesContEdgePadB' tiles into which padding will occur
   void* lpTileDst[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayForProd(mNumTilesContEdgePadB, lpTileDst);

#if (DEBUG_LVL > 0)
   printf("[0x%03X]CircTileSmartBuf::PadBot_Replicate -> SRC=0x%p DST=", (unsigned int)(((uintptr_t)this) & 0xFFF), lpTileSrc[0]);
   for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
      printf("0x%p ", lpTileDst[i]);
   printf("\n");
#endif

   int32_t lXdim = 0;
   int32_t lYdim = 1;
   int32_t lStride = mCircTileBuf.mpTileDesc->mStrideInDataType[lYdim];
   int32_t lElementSizeY = mCircTileBuf.mpTileDesc->mElementSize[lYdim];
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeY = mCircTileBuf.mpTileDesc->mSize[lYdim];
   int32_t lSizeX = (mCircTileBuf.mpTileDesc->mSize[lXdim] + mTileEdgePadL + mTileEdgePadR)*lElementSizeX;
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
   {
      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         vec08u* lpSrc = (vec08u*)lpTileSrc[0] + ((lSizeY - 1)*lElementSizeY*lStride) - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec08u* lpDst = (vec08u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec08u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec08u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_08(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         vec16u* lpSrc = (vec16u*)lpTileSrc[0] + ((lSizeY - 1)*lElementSizeY*lStride) - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec16u* lpDst = (vec16u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec16u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec16u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_16(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         vec32u* lpSrc = (vec32u*)lpTileSrc[0] + ((lSizeY - 1)*lElementSizeY*lStride) - mTileEdgePadL*lElementSizeX;

         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec32u* lpDst = (vec32u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec32u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               vec32u* lpSrcLocal = lpSrc + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_til_copy_32(lpDstLocal, lStride, lpSrcLocal, lStride, lElementSizeY, lSizeX, lSizeY);
            }
         }

         break;
      }
      }
   }
   else //scalar padding
   {
      printf("CircTileSmartBuf::PadBot_Replicate -> Scalar padding (replication) not implemented yet!!!\n");
   }
}

void CircTileSmartBuf::PadBot_Zero()
{
   //get 'mNumTilesContEdgePadB' tiles into which padding will occur
   void* lpTileDst[8]; //TODO: FIX THIS ... hardcoded max num padding tiles to 8 to avoid dynamic allocation
   GetTilePtrArrayForProd(mNumTilesContEdgePadB, lpTileDst);

#if (DEBUG_LVL > 0)
   printf("[0x%03X]CircTileSmartBuf::PadBot_Zero -> SRC=N/A DST=", (unsigned int)(((uintptr_t)this) & 0xFFF));
   for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
      printf("0x%p ", lpTileDst[i]);
   printf("\n");
#endif

   int32_t lXdim = 0;
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   icp::DATATYPE lDataType = mCircTileBuf.mpTileDesc->mDataType;

   if (MEM_TYPE_CMEM_VEC == mCircTileBuf.mBufType)
   {
      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec08u* lpDst = (vec08u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec08u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_08(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec16u* lpDst = (vec16u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec16u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_16(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         for (int32_t i = 0; i < mNumTilesContEdgePadB; i++)
         {
            vec32u* lpDst = (vec32u*)lpTileDst[i] - mTileEdgePadL*lElementSizeX;

            for (int32_t j = 0; j < RetTileWrapCnt() + 1; j++) //loop layer for wrap stuff
            {
               vec32u* lpDstLocal = lpDst + (j*mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
               apu_blk_zero_32(lpDstLocal, 0, mCircTileBuf.mpTileDesc->mTotalSizeInDataType, 1, (vec16u)1);
            }
         }

         break;
      }
      }
   }
   else //scalar padding
   {
      printf("CircTileSmartBuf::PadBot_Zero -> Scalar padding (zero) not implemented yet!!!\n");
   }
}

























//===============================================================
//MISC FRAGMENTS/SCRATCH (will clean up later)
//===============================================================
#if 0
void PadFunction(int32_t lLeftPadCuN,
                 int32_t lLeftPadCuNplus1,
                 int32_t lRightPadCuN,
                 int32_t lRightPadCuNplus1,
                 int32_t lCuIdx,
                 int32_t lLeftPadCuNIdx,
                 int32_t lLeftPadCuNplus1Idx,
                 int32_t lRightPadCuNIdx,
                 int32_t lRightPadCuNplus1Idx,
                 vec16u* lpSrc_LeftPadCuN,
                 vec16u* lpSrc_LeftPadCuNplus1,
                 vec16u* lpSrc_RightPadCuN,
                 vec16u* lpSrc_RightPadCuNplus1,
                 vec16u* lpDst_Left,
                 vec16u* lpDst_Right,
                 int32_t lTileHeightInDataType,
                 int32_t lElementSizeXInDataType)
{
   for (int32_t i=0; i<lTileHeightInDataType; i++)
   {
      int32_t lIdx = 0;
      int16_t lTmp = 0;

      for (int32_t j=0; j<lLeftPadCuNplus1*lElementSizeXInDataType; j++)
      {
         lTmp = vget(lpSrc_LeftPadCuNplus1[j], lLeftPadCuNplus1Idx);
         lpDst_Left[lIdx] = vput(lpDst_Left[lIdx], lTmp,  lCuIdx);
         lIdx++;
      }

      for (int32_t j=0; j<lLeftPadCuN*lElementSizeXInDataType; j++)
      {
         lTmp = vget(lpSrc_LeftPadCuN[j], lLeftPadCuNIdx);
         lpDst_Left[lIdx] = vput(lpDst_Left[lIdx], lTmp,  lCuIdx);
         lIdx++;
      }

      for (int32_t j=0; j<lRightPadCuN*lElementSizeXInDataType; j++)
      {
         lTmp = vget(lpSrc_RightPadCuN[j], lRightPadCuNIdx);
         lpDst_Right[lIdx] = vput(lpDst_Right[lIdx], lTmp,  lCuIdx);
         lIdx++;
      }

      for (int32_t j=0; j<lRightPadCuNplus1*lElementSizeXInDataType; j++)
      {
         lTmp = vget(lpSrc_RightPadCuNplus1[j], lRightPadCuNplus1Idx);
         lpDst_Right[lIdx] = vput(lpDst_Right[lIdx], lTmp,  lCuIdx);
         lIdx++;
      }
   }
}

{
   //my slow but semi-comprehensible vput/vget implementation

   int32_t lLeftPadCuN, lLeftPadCuNplus1, lRightPadCuN, lRightPadCuNplus1;
   int32_t lLeftPadCuN_offset, lLeftPadCuNplus1_offset, lRightPadCuN_offset, lRightPadCuNplus1_offset;
   int32_t lElementSizeX = mCircTileBuf.mpTileDesc->mElementSize[lXdim];
   int32_t lSizeX = mCircTileBuf.mpTileDesc->mSize[lXdim];

   if (mTileEdgePadL >= lSizeX)
   {
      lLeftPadCuN = lSizeX;
      lLeftPadCuNplus1 = mTileEdgePadL - lSizeX;
      lLeftPadCuN_offset = 0;
      lLeftPadCuNplus1_offset = (lSizeX - mTileEdgePadL)*lElementSizeX;
   }
   else
   {
      lLeftPadCuN = mTileEdgePadL;
      lLeftPadCuNplus1 = 0;
      lLeftPadCuN_offset = (lSizeX - mTileEdgePadL)*lElementSizeX;
      lLeftPadCuNplus1_offset = 0; //don't care
   }

   if (mTileEdgePadR >= lSizeX)
   {
      lRightPadCuN = lSizeX;
      lRightPadCuNplus1 = mTileEdgePadR - lSizeX;
      lRightPadCuN_offset = 0;
      lRightPadCuNplus1_offset = (lSizeX - mTileEdgePadR)*lElementSizeX;
   }
   else
   {
      lRightPadCuN = mTileEdgePadR;
      lRightPadCuNplus1 = 0;
      lRightPadCuN_offset = (lSizeX - mTileEdgePadR)*lElementSizeX;
      lRightPadCuNplus1_offset = 0; //don't care
   }

   int32_t lVchunkCnt = 0;

   for (int32_t j=0; j<RetTileWrapCnt()+1; j++) //loop layer for wrap stuff
   {
      int32_t lLeftEdgeCuIdx = mCircTileBuf.mpTileSegmentVecIdxStart[j];

      //for each CU in the tile segment
      for (int32_t lCuIdx = mCircTileBuf.mpTileSegmentVecIdxStart[j]; lCuIdx < mCircTileBuf.mpTileSegmentVecWidth[j]; lCuIdx++)
      {
         if ((j == 0) && (lCuIdx == lLeftEdgeCuIdx)) //left edge
         {
            //replicate locally
         }
         else if (lVchunkCnt == mCircTileBuf.mNumVchunksInTile) //right edge
         {
            //replicate locally
         }
         else if ((j == 0) && (lCuIdx == (lLeftEdgeCuIdx+1))) //left edge + 1
         {
            //get all padding from left CU (combo of local + rep)
         }
         else if (lVchunkCnt == (mCircTileBuf.mNumVchunksInTile-1)) //right edge - 1
         {
            //get all padding from right CU (combo of local + rep)
         }
         else //middle as in not a logical 2D edge
         {
            if (0 == lCuIdx) //CU0
            {
               //left pad comes from CU_last_prev and CU_last-1_prev
               //right pad comes from CU1 and CU2

            }
            else if (1 == lCuIdx) //CU0+1
            {
               //left pad comes from CU0 and CU_last_prev
               //right pad comes from CU2 and CU3
            }
            else if ((mCircTileBuf.mpVectorMapDesc->mPhysVecSize-1) == lCuIdx) //CU_LAST
            {
               //left pad comes from CU_last-1 and CU_last-2
               //right pad comes from CU0_next and CU1_next
            }
            else if ((mCircTileBuf.mpVectorMapDesc->mPhysVecSize-2) == lCuIdx) //CU_LAST-1
            {
               //left pad comes from CU_last-3 and CU_last-2
               //right pad comes from CU_last and CU0_next
            }
            else
            {
               //left pad comes from CU-2 and CU-1
               //right pad comes from CU+1 and CU+2
               PadFunction(lLeftPadCuN, lLeftPadCuNplus1, lRightPadCuN, lRightPadCuNplus1,
                           lCuIdx, lCuIdx-1, lCuIdx-2, lCuIdx+1, lCuIdx+2,
                           (vec16u*)lpTilePtr[i] + lLeftPadCuN_offset,
                           (vec16u*)lpTilePtr[i] + lLeftPadCuNplus1_offset,
                           (vec16u*)lpTilePtr[i] + lRightPadCuN_offset,
                           (vec16u*)lpTilePtr[i] + lRightPadCuNplus1_offset,
                           (vec16u*)lpTilePtr[i] - (mTileEdgePadL*lElementSizeX),
                           (vec16u*)lpTilePtr[i] + (lSizeX*lElementSizeX),
                           mCircTileBuf.mpTileDesc->mSize[lYdim]*mCircTileBuf.mpTileDesc->mElementSize[lYdim],
                           lElementSizeX);
            }
         }

         lVchunkCnt ++;
      }
   }
}

//Code fragment for managing sloweset consumer
{
   //get the id of the slowest consumer (only the slowest consumer can 'free' anything for the producer)
   int32_t lConsCountMin = mConsCounter[0];
   int32_t lConsCountMax = mConsCounter[0];
   int32_t lConsIdSlowest = 0;
   int32_t lConsIdFastest = 0;
   for (int32_t i = 1; i < mNumCons; i++)
   {
      if (mConsCounter[i] < lConsCountMin)
      {
         lConsCountMin = mConsCounter[i];
         lConsIdSlowest = i;
      }

      if (mConsCounter[i] > lConsCountMax)
      {
         lConsCountMax = mConsCounter[i];
         lConsIdFastest = i;
      }
   }

   if (lConsId == lConsIdSlowest)
   {
      //get the id of the 2nd slowest consumer
      lConsCountMin = mConsCounter[lConsIdFastest];
      int32_t lConsId2ndSlowest = 0;
      for (int32_t i = 0; i < mNumCons; i++)
      {
         if ((mConsCounter[i] < lConsCountMin) && (i != lConsIdSlowest))
         {
            lConsCountMin = mConsCounter[i];
            lConsId2ndSlowest = i;
         }
      }

      int32_t lDiff = mConsCounter[lConsId2ndSlowest] - mConsCounter[lConsIdSlowest];
      if (lNumTilesCons >= lDiff)
         mNumTilesAvailForProd += lDiff;
      else
         mNumTilesAvailForProd += lNumTilesCons;
   }
}

int32_t CircTileSmartBuf::TopPaddingHelper(int32_t lNumTilesSrc, void* lTilePtrArraySrc[], int32_t lNumTilesDst, void* lTilePtrArrayDst[])
{
   int lRetVal = 0;

   if (0 == mPadTopCompleteFlag) //if top padding has not already been done
   {
      //since top padding has not been done, mNumTilesAvailForCons should be the same for all consumers; use lConsId 0
      if (lNumTilesSrc <= mNumTilesAvailForCons[0]) //if the required source tiles are avaialble
      {
         GetTilePtrArrayAtCoreStart(lNumTilesSrc, lTilePtrArraySrc);
         GetTilePtrArrayBehindCoreStart(lNumTilesDst, lTilePtrArrayDst);
      }
      else //required number of source tiles not available yet
      {
         lRetVal = 1;
      }
   }
   else
   {
      //printf("CircTileSmartBuf::TopPaddingHelper -> top padding has already been completed!");
      lRetVal = 2;
   }

   return lRetVal;
}

#endif
