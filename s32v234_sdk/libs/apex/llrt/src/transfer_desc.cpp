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
#include <transfer_desc.hpp>
#include <circ_tile_buf.hpp>
#include <circ_tile_smart_buf.hpp>

void TransferDesc::Init(BufDesc* lpSrc, CircTileSmartBuf* lpDst, TilingDesc* lpTilingDesc)
{
   Init();

   mpBufDesc = lpSrc;
   mpCircTileBufSmartDst = lpDst;
   mpTilingDesc = lpTilingDesc;

   mTransferType = TRANSFER_TYPE__TO_APEX_CB_SCL_ITER;
}

void TransferDesc::Init(CircTileSmartBuf* lpSrc, BufDesc* lpDst, TilingDesc* lpTilingDesc)
{

   Init();

   mpCircTileBufSmartSrc = lpSrc;
   mpBufDesc = lpDst;
   mpTilingDesc = lpTilingDesc;

   mTransferType = TRANSFER_TYPE__FROM_APEX_CB_SCL_ITER;
}

void TransferDesc::Init(BufDesc* lpSrc, CircTileSmartBuf* lpDst, TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizaitonDesc, VectorMapDesc* lpVectorMapDesc)
{
   Init();

   mpBufDesc = lpSrc;
   mpCircTileBufSmartDst = lpDst;
   mpTilingDesc = lpTilingDesc;
   mpTileVectorizationDesc = lpTileVectorizaitonDesc;
   mpVectorMapDesc = lpVectorMapDesc;

   mTransferType = TRANSFER_TYPE__TO_APEX_CB_VEC_ITER;
}

void TransferDesc::Init(CircTileSmartBuf* lpSrc, BufDesc* lpDst, TilingDesc* lpTilingDesc, TileVectorizationDesc* lpTileVectorizaitonDesc, VectorMapDesc* lpVectorMapDesc)
{
   Init();

   mpCircTileBufSmartSrc = lpSrc;
   mpBufDesc = lpDst;
   mpTilingDesc = lpTilingDesc;
   mpTileVectorizationDesc = lpTileVectorizaitonDesc;
   mpVectorMapDesc = lpVectorMapDesc;

   mTransferType = TRANSFER_TYPE__FROM_APEX_CB_VEC_ITER;
}

void TransferDesc::Init(CircTileSmartBuf* lpSrc, CircTileSmartBuf* lpDst, TileVectorizationDesc* lpTileVectorizaitonDesc)
{
   Init();

   mpCircTileBufSmartSrc = lpSrc;
   mpCircTileBufSmartDst = lpDst;
   mpTileVectorizationDesc = lpTileVectorizaitonDesc;

   if (IsVec(lpSrc->mCircTileBuf.mBufType) && IsVec(lpDst->mCircTileBuf.mBufType))
   {
      //vec->vec
      mTransferType = TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_VEC;
      mpVectorMapDesc = lpSrc->mCircTileBuf.mpVectorMapDesc; //take from src; assumed to be the same
   }
   else if (!IsVec(lpSrc->mCircTileBuf.mBufType) && !IsVec(lpDst->mCircTileBuf.mBufType))
   {
      //scl->scl
      mTransferType = TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_SCL;
   }
   else
   {
      //mix of vector and scalar CBs
      if (IsVec(lpSrc->mCircTileBuf.mBufType))
      {
         mTransferType = TRANSFER_TYPE__APEX_CB_VEC_TO_APEX_CB_SCL;
         mpVectorMapDesc = lpSrc->mCircTileBuf.mpVectorMapDesc;
      }
      else
      {
         mTransferType = TRANSFER_TYPE__APEX_CB_SCL_TO_APEX_CB_VEC;
         mpVectorMapDesc = lpDst->mCircTileBuf.mpVectorMapDesc;
      }
   }
}

void TransferDesc::Init2DPatch(BufDesc* lpSrc, CircTileSmartBuf* lpDst, int32_t lPatchWidth, int32_t lPatchHeight, BufDesc* lpPatchOffsetArray, VectorMapDesc* lpVectorMapDesc)
{
   Init();

   mpBufDesc = lpSrc;
   mpCircTileBufSmartDst = lpDst;
   mPatchWidth = lPatchWidth;
   mPatchHeight = lPatchHeight;
   mpPatchOffsetArray = lpPatchOffsetArray;
   mpVectorMapDesc = lpVectorMapDesc;

   mTransferType = TRANSFER_TYPE__TO_APEX_CB_VEC_ITER;
}

void TransferDesc::Init2DPatch(BufDesc* lpSrc, CircTileSmartBuf* lpDst, int32_t lPatchWidth, int32_t lPatchHeight, BufDesc* lpPatchOffsetArray)
{
   Init();

   mpBufDesc = lpSrc;
   mpCircTileBufSmartDst = lpDst;
   mPatchWidth = lPatchWidth;
   mPatchHeight = lPatchHeight;
   mpPatchOffsetArray = lpPatchOffsetArray;

   mTransferType = TRANSFER_TYPE__TO_APEX_CB_SCL_ITER;
}

void TransferDesc::FlagPatchOffsetInBytes()
{
   mPatchOffsetInBytesFlag = 1;
}

void TransferDesc::Init()
{
   mpTilingDesc = 0;
   mpTileVectorizationDesc = 0;
   mpVectorMapDesc = 0;
   mpBufDesc = 0;
   mpCircTileBufSmartSrc = 0;
   mpCircTileBufSmartDst = 0;
   mTransferType = TRANSFER_TYPE__UNDEFINED;

   mpPatchOffsetArray = 0;
   mPatchWidth = 0;
   mPatchHeight = 0;
   mPatchOffsetInBytesFlag = 0;
}
