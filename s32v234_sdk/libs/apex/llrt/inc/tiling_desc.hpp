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

#ifndef _TILING_DESC_HPP
#define _TILING_DESC_HPP

#include <common.hpp>

//////////////////////////////////////////////////////////////////////////
/// TilingDesc describes how a contiguous multi-dimensional region of data
/// should be partitioned into tiles.
//////////////////////////////////////////////////////////////////////////
class TilingDesc
{
public:
   
   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize N-dimensional tiling descriptor.
   ///
   /// For example, consider defining a tiling descriptor to partition a 256x128 2D region of memory residing at address A.
   ///
   /// If we want tiles to be entire rows (i.e. lines):
   /// \code{.cpp}
   /// int32_t lSize[2] = {256, 1}; //lSize[0] = 256 = full width of the 2D region
   /// int32_t lStepSize[2] = {256, 1}; //step size equals the tile size because no overlap is desired
   /// int32_t lStepDim[2] = {0, 1}; //step on dimension 0, then dimension 1 (doesn't matter in this case since lSize[0] is max)
   /// \endcode
   /// Tile addr order: A+(256*0 + 0), A+(256*1 + 0), A+(256*2+ 0), ..., A+(256*127 + 0) (128 tiles in total)
   ///
   /// If we want tiles to be entire columns:
   /// \code{.cpp}
   /// int32_t lSize[2] = {1, 128}; //lSize[1] = 128 = full height of the 2D region
   /// int32_t lStepSize[2] = {1, 128}; //step size equals the tile size because no overlap is desired
   /// int32_t lStepDim[2] = {0, 1}; //step on dimension 0, then dimension 1 (doesn't matter in this case since lSize[1] is max)
   /// \endcode
   /// Tile addr order: A+(256*0 + 0), A+(256*0 + 1), A+(256*0 + 2), ..., A+(256*0 + 255) (256 tiles in total)
   ///
   /// If we want tiles to be 128x4:
   /// \code{.cpp}
   /// int32_t lSize[2] = {128, 4};
   /// int32_t lStepSize[2] = {128, 4}; //step size equals the tile size because no overlap is desired
   /// \endcode
   /// If lStepDim={0, 1}: Tile addr order: A+(256*0 + 0), A+(256*0 + 128), A+(256*4 + 0), A+(256*4 + 128), ..., A+(256*124 + 0), A+(256*124 + 128) (64 tiles in total)
   /// If lStepDim={1, 0}: Tile addr order: A+(256*0 + 0), A+(256*4 + 0), ..., A+(256*124 + 0), A+(256*0 + 128), A+(256*4 + 128), ..., A+(256*124 + 128) (64 tiles in total)
   ///
   /// \param[in] lNumDim
   /// Number of dimensions.
   ///
   /// \param[in] lSize
   /// Array containing the tile size in each dimension.
   ///
   /// \param[in] lStepSize
   /// Array containing the step size in each dimension when traversing the parent data region.  Step size will typically be
   /// the same as tile size (i.e. lStepSize == lSize), unless you are trying to express overlapping tiles.
   ///
   /// \param[in] lStepDim
   /// Array containing the step dimension priority.
   /// e.g. lStepDim=[0,2,1] means to step first on dimension 0, then 2, then 1.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t InitND(int32_t lNumDim, int32_t lSize[], int32_t lStepSize[], int32_t lStepDim[]);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize 2D tiling descriptor.  This is a 2D version of InitND that is
   /// provided for convenience.  See the description of InitND for more information.
   /////////////////////////////////////////////////////////////////////////
   int32_t Init2D(int32_t lSize0, int32_t lSize1,
                  int32_t lStepSize0, int32_t lStepSize1,
                  int32_t lStepDim0, int32_t lStepDim1);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize N-dimensional user-defined tiling descriptor.  This can be used to describe
   /// irregular offset-based tiling patterns that cannot be expressed using InitND/Init2D.
   ///
   /// \param[in] lNumDim
   /// Number of dimensions.
   ///
   /// \param[in] lSize
   /// Array containing the tile size in each dimension.
   ///
   /// \param[in] lNumTile
   /// The number of tiles (i.e. the number of tile offsets present in "lTileOffsetArray").
   ///
   /// \param[in] lTileOffsetArray
   /// Array of tile offsets (in units of data type).
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   //int32_t InitUserDefND(int32_t lNumDim, int32_t lSize[], int32_t lNumTile, int32_t lTileOffsetArray[]);

   //---------
   //members
   //---------

   int32_t mNumDim;
   int32_t mSize[DESC_MAX_NUM_DIM];     //tile size (in elements)
   int32_t mStepSize[DESC_MAX_NUM_DIM]; //amt to step in each dimension to get to the next tile (in elements)
   int32_t mStepDim[DESC_MAX_NUM_DIM];  //describe stepping direction priority based on dimension
   
   //for cases where user wants to define a custom tiling pattern
   int32_t mNumTileUserDef;
   int32_t* mTileOffsetArrayUserDef; //(in data type)
};

#endif //_TILING_DESC_HPP
