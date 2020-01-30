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

#ifndef _CIRC_TILE_BUF_HPP
#define _CIRC_TILE_BUF_HPP

#include <stdint.h>
#include <common.hpp>

//forward declarations
class DataDesc;
class VectorMapDesc;
class TileInfo;

//this enum exists for backwards compatibility purposes
enum
{
   NUM_TILES_1  = 1,
   NUM_TILES_2  = 2,
   NUM_TILES_4  = 4,
   NUM_TILES_8  = 8,
   NUM_TILES_16 = 16,
   NUM_TILES_32 = 32,
   NUM_TILES_64 = 64
};

//power of 2 helper functions
int32_t IsPowerOfTwo(int32_t n);
int32_t RetNextPowerOfTwo(int32_t n);

//use bit 5 to know if the type is vector (1) or scalar (0)
typedef enum _MEM_TYPE
{
   MEM_TYPE_CMEM_VEC = 0x10, //cmem as vector memory (CU or GOC)
   MEM_TYPE_CMEM_SCL = 0x00, //cmem as scalar memory (currently not used, but it may be worth distinguishing this from the typical vector case)
   MEM_TYPE_DMEM_SCL = 0x01,
   MEM_TYPE_SMEM_SCL = 0x02,
   MEM_TYPE_XMEM_SCL = 0x03
} MEM_TYPE;

int32_t IsVec(MEM_TYPE lBufType);

class CircTileBuf
{
public:

   //---------
   //methods
   //---------
   
   //these assume no vectorization
   int32_t CreateDmem(int32_t lNumTiles, DataDesc* lpTileDesc);
   int32_t CreateSmem(int32_t lNumTiles, DataDesc* lpTileDesc);
   
   //this one is for typical vectorized tile buffer in CMEM (vectorization could be on CU level or GOC level)
   int32_t CreateCmem(int32_t lNumTiles, DataDesc* lpVchunkDesc, int32_t lNumVchunksInTile, VectorMapDesc* lpVecMapDesc);
   
   int32_t Destroy();

   //these assume that 'lTileIdx' >=0
   void* RetTilePtr(int32_t lTileIdx);
   void GetTilePtrArray(int32_t lTileIdx, int32_t lNumTiles, void* lTilePtrArray[]);
   
   //these can be used to determine start CU/GOC and tile segment size (for example) depending on mWrapIdx
   int32_t QueryTileSegmentVecIdxStart(int32_t lWrapIdx);
   int32_t QueryTileSegmentVecWidth(int32_t lWrapIdx);

   int32_t RetTileWrapCnt();
   void SetTileInfo(TileInfo* lpTileInfo, int32_t lIdx);
   TileInfo* GetTileInfo(int32_t lIdx);

   //---------
   //members
   //---------

   int32_t mNumTiles;
   DataDesc* mpTileDesc; //description of tile (e.g. memory per CU, memory per GOC, or just memory if scalar)
   void* mTileBuffer; //starting point of tile buffer
   void** mTilePtrArray; //array containing pre-calculated tile addresses
   int32_t mPow2Mask; //power of 2 mask for tile idx calculation
   //vector specific members
   VectorMapDesc* mpVectorMapDesc; //e.g. primarily for start CU/GOC, and GOC mode
   int32_t mNumVchunksInTile;
   MEM_TYPE mBufType;

   TileInfo* mTileInfoArray; //array of TileInfo ptrs (one for each tile)

   //these are for use in the vector case only
   int32_t* mpTileSegmentVecIdxStart;
   int32_t* mpTileSegmentVecWidth;

private:

   void Init();
   int32_t DerivedInit();

   int32_t (*mpRetTileIdx)(CircTileBuf*, int32_t);
   int32_t mTileWrapCnt; //number of times a tile described by mpTileDesc will wrap
};

class TileInfo
{
public:
   int32_t mTileLoc[DESC_MAX_NUM_DIM];
};

#endif //_CIRC_TILE_BUF_HPP
