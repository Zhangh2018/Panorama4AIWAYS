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

#ifndef _DATA_TRANSFER_HPP
#define _DATA_TRANSFER_HPP

#include <stdint.h>
#include <common.hpp>

//forward declarations
class TransferDesc;
class BufDesc;
class CircTileBuf;
class CircTileSmartBuf;
class TilingDesc;
class TileVectorizationDesc;
class VectorMapDesc;
class DataDesc;

typedef enum _VEC_TRANSFER_PATTERN
{
   VEC_TRANSFER_PATTERN_A, //2D, 4 bytes per CU, wrapping line transfers
   VEC_TRANSFER_PATTERN_B, //TBD
   VEC_TRANSFER_PATTERN_C, //TBD
   VEC_TRANSFER_PATTERN_UNDEF
} VEC_TRANSFER_PATTERN;

class DataTransfer
{
public: 
   
   //---------
   //methods
   //---------
   
   int32_t Setup(TransferDesc* lpTransferDesc, int32_t lNumTilesToTransfer);
   int32_t Iterate();
   int32_t Teardown();
   int32_t Finished();
   void Reset();
   
   //---------
   //members
   //---------
   
   TransferDesc* mpTransferDesc;
   
   //vector size (e.g. size=8 refers to a vector spanning 8 CUs if gocMode=0, and 8 GOCs if gocMode is not 0); this value may exceed mpVectorMapDesc.mPhysVecSize
   //and if it does, it can be interpreted as a 'virtual' vector that is realized via wrapping at vector element resolution. it is calculated based on
   //mpVectorMapDesc.mPhysVecSize, mpVectorMapDesc.mGocMode, and mpTileVectorizationDesc.
   int32_t mVecSize;
   
private:

   void Init();
   int32_t SetupEnvSpecific(); //perform setup specific to the environment (iss, native, hw)
   int32_t CalcNextChunkOffsetInDataType(int32_t lStepDim[], int32_t lStepSize[], DataDesc* lDataDesc, int32_t lChunkLoc[], int32_t mNumChunksInDim[]);
   int32_t CalcNextTileOffsetInDataType();
   int32_t CalcNextVchunkOffsetInDataType();
   int32_t CalcNextVchunkOffsetInDataType(TileVectorizationDesc* lpTileVectorizationDesc, DataDesc* lTargetRegionDesc);
   int32_t Iterate_ToApexCbScl(int32_t lNumTilesToTransfer);
   int32_t Iterate_ToApexCbVec(int32_t lNumTilesToTransfer);
   int32_t Iterate_FromApexCbScl(int32_t lNumTilesToTransfer);
   int32_t Iterate_FromApexCbVec(int32_t lNumTilesToTransfer);
   int32_t Iterate_ApexCbToApexCb(int32_t lNumTilesToTransfer);
   int32_t DetermineOptimalVecTransferPattern();

   int32_t mTileLoc[DESC_MAX_NUM_DIM]; //keep track of running tile location (absolute recalculation is costly)
   int32_t mVchunkLoc[DESC_MAX_NUM_DIM]; //keep track of running vchunk location (absolute recalculation is costly)
   int32_t mNumDim;
   int32_t mNumTileInDim[DESC_MAX_NUM_DIM]; //number of tiles in each dimension
   int32_t mNumVchunkInDim[DESC_MAX_NUM_DIM]; //number of vchunks in each dimension
   int32_t mNumTilesTotal;   //total number of tiles
   int32_t mNumVchunksTotal; //total number of vchunks per tile
   int32_t mNumTilesTransfered;
   int32_t mNumTilesToTransfer; //number of tiles to transfer each iteration
   void** mpTilePtrArraySrc;
   void** mpTilePtrArrayDst;
   int32_t mConsId; //store consumer id if data transfer task consuming from a CircTileSmartBuf
   int32_t mTransferSize_DataType[DESC_MAX_NUM_DIM];

   BufDesc* mpBufDesc; //buffer that will be src/dst of data transfer
   CircTileSmartBuf* mpCircTileSmartBufSrc;
   CircTileSmartBuf* mpCircTileSmartBufDst;
   TilingDesc* mpTilingDesc;
   TileVectorizationDesc* mpTileVectorizationDesc;
   VectorMapDesc* mpVectorMapDesc;

   VEC_TRANSFER_PATTERN mVecTransferPattern;

   //members to deal with 2D patch data transfer
   int32_t mNumTilesPerPatch; //typically number of lines per patch since base CB tile unit in 2D will always be lines
   int32_t mPatchTileOffsetInDataType;
   int32_t* mPatchOffsetArray;
   int32_t mPatchOffsetRowTileCnt;
   int32_t mPatchOffsetRowCnt;
   int32_t mPatchOffsetColCnt;

   //<variables required to encapsulate actual HW transfer cfg/desc>;
   
   //FUTURE
   //look into DMEM FIFO as end-point ... not sure if this is actually needed
};

#endif //_DATA_TRANSFER_HPP
