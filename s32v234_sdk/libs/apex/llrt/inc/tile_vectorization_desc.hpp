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

#ifndef _TILE_VECTORIZATION_DESC_HPP
#define _TILE_VECTORIZATION_DESC_HPP

#include <common.hpp>

//////////////////////////////////////////////////////////////////////////
/// TileVectorizationDesc describes how a tile should be vectorized.  This descriptor is fundamentally
/// the same as TilingDesc and simply allows for the expression of another layer of partitioning (i.e. a region
/// of data is first partitioned into tiles, and then each tile is further partitioned into vchunks).
//////////////////////////////////////////////////////////////////////////
class TileVectorizationDesc
{
public:
   
   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize multi-dimensional tile vectorization descriptor.  Refer to the
   /// description of TilingDesc::InitND for more information.
   /////////////////////////////////////////////////////////////////////////
   int32_t InitND(int32_t lNumDim, int32_t lSize[], int32_t lStepSize[], int32_t lStepDim[]);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize 2D tile vectorization descriptor.  This is a 2D version of InitND that is
   /// provided for convenience.  See the description of InitND for more information.
   /////////////////////////////////////////////////////////////////////////
   int32_t Init2D(int32_t lSize0, int32_t lSize1,
                  int32_t lStepSize0, int32_t lStepSize1,
                  int32_t lStepDim0, int32_t lStepDim1);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize N-dimensional user-defined tile vectorization descriptor.  This can be used to describe
   /// irregular offset-based vectorization patterns that cannot be expressed using InitND/Init2D.
   ///
   /// \param[in] lNumDim
   /// Number of dimensions.
   ///
   /// \param[in] lSize
   /// Array containing vchunk size in each dimension.  This describes the vchunk referenced by each offset value.
   /// It is assumed that vchunk size is the same for all offset values.
   ///
   /// \param[in] lNumVchunkPerTile
   /// The number of vchunks/offsets per tile.
   ///
   /// \param[in] lNumTile
   /// The number of tiles of user-defined offsets.  If the same user-defined vchunk offset pattern is applicable
   /// to each tile ,"lNumTile" should  be 1.  If each tile has a unique offset pattern, "lNumTile"
   /// should equal total number of tiles and the offset array should contain "lNumTile" rows of "lNumVchunkPerTile" offsets.
   ///
   /// \param[in] lOffsetArray
   /// Array of offsets (in units of data type). The offset array size must be equal to (lNumVchunkPerTile*lNumTile).
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   //int32_t InitUserDefND(int32_t lNumDim, int32_t lSize[], int32_t lNumVchunkPerTile, int32_t lNumTile, int32_t lOffsetArray[]);
   
   //---------
   //members
   //---------

   int32_t mNumDim;
   int32_t mSize[DESC_MAX_NUM_DIM];     //vchunk size (in elements)
   int32_t mStepSize[DESC_MAX_NUM_DIM]; //amt to step in each dimension to get to the next vchunk (in elements)
   int32_t mStepDim[DESC_MAX_NUM_DIM];  //describe stepping direction priority based on dimension

   //for cases where user wants to define a custom vectorization pattern  
   int32_t mNumVchunkPerTileUserDef;
   int32_t mNumTileUserDef;
   int32_t* mOffsetArrayUserDef; //(in data type)
};

#endif //_TILE_VECTORIZATION_DESC_HPP
