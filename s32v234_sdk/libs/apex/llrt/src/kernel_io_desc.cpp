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

#include <stdio.h>
#include <kernel_io_desc.hpp>
#include <apu_mem_mgmt.hpp>
#include <circ_tile_smart_buf.hpp>
#include <data_desc.hpp>

int32_t KernelIoDesc::CreateProd(int32_t lNumTilePtrs, CircTileSmartBuf* lpCircTileSmartBuf)
{
   int32_t lRetVal = 0;

   mNumTilePtrs = lNumTilePtrs;
   mpCircTileSmartBuf = lpCircTileSmartBuf;
   mpDataDesc = lpCircTileSmartBuf->mCircTileBuf.mpTileDesc;
   mConsId = 0;

   mpTilePtrArray = (void**)ApuMemAllocSclDmem(sizeof(vec08s*)*lNumTilePtrs);
   
   if (0 == mpTilePtrArray)
   {
      printf("KernelIoDesc::CreateProd -> memory allocation failure (%d bytes)!\n", sizeof(vec08s*)*lNumTilePtrs);
      lRetVal = 1;
   }
   
   return lRetVal;
}

int32_t KernelIoDesc::CreateCons(int32_t lNumTilePtrs, CircTileSmartBuf* lpCircTileSmartBuf, int32_t lConsId)
{
   int32_t lRetVal = CreateProd(lNumTilePtrs, lpCircTileSmartBuf);

   mConsId = lConsId;

   return lRetVal;
}

int32_t KernelIoDesc::Destroy()
{
   int32_t lRetVal = 0;

   ApuMemFree((int8_t*)mpTilePtrArray);
   
   return lRetVal;
}

void** KernelIoDesc::RetTilePtrArray()
{
   return mpTilePtrArray;
}

int32_t KernelIoDesc::RetNumTilePtrs()
{
   return mNumTilePtrs;
}

int32_t KernelIoDesc::RetSize(int32_t lDim)
{
   return mpDataDesc->mSize[lDim];
}

int32_t KernelIoDesc::RetStride(int32_t lDim)
{
   return mpDataDesc->mStrideInDataType[lDim];
}

int32_t KernelIoDesc::RetElementSize(int32_t lDim)
{
   return mpDataDesc->mElementSize[lDim];
}

int32_t KernelIoDesc::RetNumDim()
{
   return mpDataDesc->mNumDim;
}

void KernelIoDesc::UpdateIoDescCons_SCL08()
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
}

void KernelIoDesc::UpdateIoDescCons_SCL16()
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
}

void KernelIoDesc::UpdateIoDescCons_SCL32()
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
}

void KernelIoDesc::UpdateIoDescCons_VEC08(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec08s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}

void KernelIoDesc::UpdateIoDescCons_VEC16(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec16s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}

void KernelIoDesc::UpdateIoDescCons_VEC32(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForCons(mNumTilePtrs, mpTilePtrArray, mConsId);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec32s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}

void KernelIoDesc::UpdateIoDescProd_SCL08()
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
}

void KernelIoDesc::UpdateIoDescProd_SCL16()
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
}

void KernelIoDesc::UpdateIoDescProd_SCL32()
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
}

void KernelIoDesc::UpdateIoDescProd_VEC08(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec08s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}

void KernelIoDesc::UpdateIoDescProd_VEC16(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec16s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}

void KernelIoDesc::UpdateIoDescProd_VEC32(int32_t lWrapIdx)
{
   mpCircTileSmartBuf->GetTilePtrArrayForProd(mNumTilePtrs, mpTilePtrArray);
   for (int32_t i=0; i<mNumTilePtrs; i++)
      mpTilePtrArray[i] = (void*)((vec32s*)mpTilePtrArray[i] + lWrapIdx*mpCircTileSmartBuf->mCircTileBuf.mpTileDesc->mTotalSizeInDataType);
}
