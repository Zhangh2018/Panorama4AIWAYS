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
#include <common.hpp>
#include <data_desc.hpp>
#include <tiling_desc.hpp>
#include <tile_vectorization_desc.hpp>
#include <vector_map_desc.hpp>

int32_t CalcNumTileInDim(DataDesc* lpDataDesc, TilingDesc* lpTilingDesc, int32_t lDim)
{
   return ((lpDataDesc->mSize[lDim] - lpTilingDesc->mSize[lDim]) / lpTilingDesc->mStepSize[lDim]) + 1;
}

int32_t CalcNumVchunkInDim(int32_t lTileSizeInDim, TileVectorizationDesc* lpTileVectorizationDesc, int32_t lDim)
{
   return ((lTileSizeInDim - lpTileVectorizationDesc->mSize[lDim]) / lpTileVectorizationDesc->mStepSize[lDim]) + 1;
}

int32_t CalcNumTileTotal(DataDesc* lpDataDesc, TilingDesc* lpTilingDesc)
{
   int32_t lNumTile = 0;

   lNumTile = 1;
   for (int32_t i = 0; i < lpDataDesc->mNumDim; i++)
   {
      lNumTile *= CalcNumTileInDim(lpDataDesc, lpTilingDesc, i);
   }

   return lNumTile;
}

int32_t CalcNumVchunkTotal(int32_t lTileSize[], TileVectorizationDesc* lpTileVectorizationDesc)
{
   int32_t lNumVchunk = 0;

   lNumVchunk = 1;
   for (int32_t i = 0; i < lpTileVectorizationDesc->mNumDim; i++)
   {
      lNumVchunk *= CalcNumVchunkInDim(lTileSize[i], lpTileVectorizationDesc, i);
   }

   return lNumVchunk;
}

int32_t CalcNumVchunkTotal(TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizationDesc)
{
   return (CalcNumVchunkTotal(lpTilingDesc->mSize, lpTileVectorizationDesc));
}

/*
int32_t CalcTileWrapCnt(TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizationDesc, VectorMapDesc* lpVectorMapDesc)
{
   int lWrapCnt = 0;
   int lNumVchunkInTile = CalcNumVchunkTotal(lpTilingDesc, lpTileVectorizationDesc);
   int lReqVchunkInclVecPad = lNumVchunkInTile + lpVectorMapDesc->mVecPad[0] + lpVectorMapDesc->mVecPad[1];
   lWrapCnt = (lReqVchunkInclVecPad - 1) / lpVectorMapDesc->mPhysVecSize;
   return lWrapCnt;
}
*/
