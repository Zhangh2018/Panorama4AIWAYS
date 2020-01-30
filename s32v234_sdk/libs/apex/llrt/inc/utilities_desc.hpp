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

#ifndef _UTILITIES_HPP
#define _UTILITIES_HPP

//forward declarations
class DataDesc;
class TilingDesc;
class TileVectorizationDesc;
class VectorMapDesc;

//some general helper functions for deriving required values from various descriptors or combinations of descriptors
int32_t CalcNumTileInDim(DataDesc* lpDataDesc, TilingDesc* lpTilingDesc, int32_t lDim);
int32_t CalcNumVchunkInDim(int32_t lTileSizeInDim, TileVectorizationDesc* lpTileVectorizationDesc, int32_t lDim);
int32_t CalcNumTileTotal(DataDesc* lpDataDesc, TilingDesc* lpTilingDesc);
int32_t CalcNumVchunkTotal(int32_t lTileSize[], TileVectorizationDesc* lpTileVectorizationDesc);
int32_t CalcNumVchunkTotal(TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizationDesc);
//int32_t CalcTileWrapCnt(TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizationDesc, VectorMapDesc* lpVectorMapDesc);

#endif //_UTILITIES_HPP
