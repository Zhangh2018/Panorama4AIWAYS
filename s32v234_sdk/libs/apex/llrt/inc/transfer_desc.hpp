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

#ifndef _TRANSFER_DESC_HPP
#define _TRANSFER_DESC_HPP

#include <stdint.h>

//forward declarations
class BufDesc;
class CircTileBuf;
class CircTileSmartBuf;
class TilingDesc;
class TileVectorizationDesc;
class VectorMapDesc;

typedef enum _TRANSFER_TYPE
{
   TRANSFER_TYPE__UNDEFINED = 0,
   TRANSFER_TYPE__TO_APEX_CB_SCL_ITER,
   TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER,
   TRANSFER_TYPE__TO_APEX_CB_VEC_ITER,
   TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER,
   TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC,
   TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL,
   TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL,
   TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC
} TRANSFER_TYPE;

//////////////////////////////////////////////////////////////////////////
/// TransferDesc collects all information required to fully define a data transfer
/// to or from vector or scalar memory, including details about tiling, 
/// vectorization, and vector mapping.
//////////////////////////////////////////////////////////////////////////
class TransferDesc
{
public:
   
   //---------
   //methods
   //---------

   //--------------------------------------------------------------------------------
   //for transfers between external mem and APEX scalar mem (SMEM, DMEM, scalar CMEM)
   //--------------------------------------------------------------------------------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize a transfer descriptor that transfers tiles from source region described
   /// by "lpSrc" to the scalar circular tile buffer "lpDst" based on the tiling descriptor "lpTilingDesc".
   /// 
   /// \param[in] lpSrc
   /// Pointer to BufDesc that describes the source region.
   ///
   /// \param[in] lpDst
   /// Pointer to destination CircTileSmartBuf.
   ///
   /// \param[in] lpTilingDesc
   /// Tiling descriptor.
   /////////////////////////////////////////////////////////////////////////
   void Init(BufDesc* lpSrc, CircTileSmartBuf* lpDst, TilingDesc* lpTilingDesc);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize a transfer descriptor that transfers tiles from the scalar circular tile buffer "lpSrc"
   /// to the destination region described by "lpDst" based on the tiling descriptor "lpTilingDesc".
   ///
   /// \param[in] lpSrc
   /// Pointer to source CircTileSmartBuf.
   ///
   /// \param[in] lpDst
   /// Pointer to BufDesc that describes the destination region.
   ///
   /// \param[in] lpTilingDesc
   /// Tiling descriptor.
   /////////////////////////////////////////////////////////////////////////
   void Init(CircTileSmartBuf* lpSrc, BufDesc* lpDst, TilingDesc* lpTilingDesc);

   //------------------------------------------------------------------------------------
   //for transfers between external mem and APEX vector mem (CMEM @ CU, CMEM @ GOC, etc.)
   //------------------------------------------------------------------------------------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize a transfer descriptor that transfers tiles from source region described
   /// by "lpSrc" to the vector circular tile buffer "lpDst" based on the tiling descriptor "lpTilingDesc", 
   /// the tile vectorization descriptor "lpTileVectorizaitonDesc", and the vector map descriptor
   /// "lpVectorMapDesc".
   /// 
   /// \param[in] lpSrc
   /// Pointer to BufDesc that describes the source region.
   ///
   /// \param[in] lpDst
   /// Pointer to destination CircTileSmartBuf.
   ///
   /// \param[in] lpTilingDesc
   /// Tiling descriptor.
   ///
   /// \param[in] lpTileVectorizaitonDesc
   /// Tile vectorization descriptor.
   ///
   /// \param[in] lpVectorMapDesc
   /// Vector map descriptor.
   /////////////////////////////////////////////////////////////////////////
   void Init(BufDesc* lpSrc, CircTileSmartBuf* lpDst, TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizaitonDesc, VectorMapDesc* lpVectorMapDesc);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize a transfer descriptor that transfers tiles from the vector circular tile buffer "lpSrc"
   /// to the destination region described by "lpDst" based on the tiling descriptor "lpTilingDesc", 
   /// the tile vectorization descriptor "lpTileVectorizaitonDesc", and the vector map descriptor
   /// "lpVectorMapDesc".
   ///
   /// \param[in] lpSrc
   /// Pointer to source CircTileSmartBuf.
   ///
   /// \param[in] lpDst
   /// Pointer to BufDesc that describes the destination region.
   ///
   /// \param[in] lpTilingDesc
   /// Tiling descriptor.
   ///
   /// \param[in] lpTileVectorizaitonDesc
   /// Tile vectorization descriptor.
   ///
   /// \param[in] lpVectorMapDesc
   /// Vector map descriptor.
   /////////////////////////////////////////////////////////////////////////
   void Init(CircTileSmartBuf* lpSrc, BufDesc* lpDst, TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizaitonDesc, VectorMapDesc* lpVectorMapDesc);

   /////////////////////////////////////////////////////////////////////////
   /// Initialize a transfer descriptor that transfers tiles from the source circular tile buffer "lpSrc"
   /// to the destination circular tile buffer "lpDst".  VEC->SCL, SCL->VEC, SCL->SCL, and VEC->VEC
   /// are supported.  If VEC->SCL or SCL->VEC, tile vectorization descriptor "lpTileVectorizaitonDesc" needs
   /// to be provided (otherwise it can be set to 0).
   ///
   /// \param[in] lpSrc
   /// Pointer to source CircTileSmartBuf.
   ///
   /// \param[in] lpDst
   /// Pointer to destination CircTileSmartBuf.
   ///
   /// \param[in] lpTileVectorizaitonDesc
   /// Tile vectorization descriptor.  This is required in the VEC->SCL and SCL->VEC cases.
   /////////////////////////////////////////////////////////////////////////
   void Init(CircTileSmartBuf* lpSrc, CircTileSmartBuf* lpDst, TileVectorizationDesc* lpTileVectorizaitonDesc);

   //TODO: document these
   void Init2DPatch(BufDesc* lpSrc, CircTileSmartBuf* lpDst, int32_t lPatchWidth, int32_t lPatchHeight, BufDesc* lpPatchOffsetArray, VectorMapDesc* lpVectorMapDesc);
   void Init2DPatch(BufDesc* lpSrc, CircTileSmartBuf* lpDst, int32_t lPatchWidth, int32_t lPatchHeight, BufDesc* lpPatchOffsetArray);
   void FlagPatchOffsetInBytes(); //this is for backwards compatibility with ACF - ACF'indirect' mode required users to provide offsets in units of bytes so they could be used directly with the DMA HW

   //FUTURE:
   //- partially parameterized interfaces based on known patterns (e.g. A = 2D IO line transfer, B = 2D IO chunk transfer, etc.)
   //  e.g. Init_patternA(DataDesc*, <subset of params req for pattern A>);
   //  e.g. Init_patternB(DataDesc*, <subset of params req for pattern B>);
   
   //---------
   //members
   //---------
  
   TilingDesc* mpTilingDesc;
   TileVectorizationDesc* mpTileVectorizationDesc;
   VectorMapDesc* mpVectorMapDesc;
   BufDesc* mpBufDesc; //buffer that will be src/dst of data transfer
   CircTileSmartBuf* mpCircTileBufSmartSrc; //local APEX circular buffer (source)
   CircTileSmartBuf* mpCircTileBufSmartDst; //local APEX circular buffer (destination)
   TRANSFER_TYPE mTransferType; //allow for quick determination of conceptual data transfer (e.g. offchip->APEX vs APEX->offchip)

   //members related to 2D patch processing
   BufDesc* mpPatchOffsetArray;
   int32_t mPatchWidth; //in unit of elements
   int32_t mPatchHeight; //in unit of elements
   int32_t mPatchOffsetInBytesFlag;
   
private:

   void Init();  
};

#endif //_TRANSFER_DESC_HPP
