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
#include <common.hpp>
#include <data_desc.hpp>

int32_t DataDesc::InitND(int32_t lNumDim, icp::DATATYPE lDataType, int32_t lSize[])
{
   int lRetVal = 0;
   
   if (lNumDim <= DESC_MAX_NUM_DIM)
   {
      mNumDim = lNumDim;
      mDataType = lDataType;
      for (int32_t i=0; i<mNumDim; i++)
      {
         mElementSize[i] = 1;
         mSize[i] = lSize[i];
         mPad[2*i] = 0;
         mPad[2*i+1] = 0;
      }
      
      DerivedInit();
   }
   else
   {
      //lNumDim is out of range
      lRetVal = 1;
   }
   
   return lRetVal;
}

int32_t DataDesc::InitND(int32_t lNumDim, icp::DATATYPE lDataType, int32_t lSize[], int32_t lElemSize[], int32_t lPad[])
{
   int lRetVal = 0;
   
   if (lNumDim <= DESC_MAX_NUM_DIM)
   {
      mNumDim = lNumDim;
      mDataType = lDataType;
      for (int32_t i=0; i<mNumDim; i++)
      {
         mElementSize[i] = lElemSize[i];
         mSize[i] = lSize[i];
         mPad[2*i] = lPad[2*i];
         mPad[2*i+1] = lPad[2*i+1];
      }
      
      DerivedInit();
   }
   else
   {
      //lNumDim is out of range
      lRetVal = 1;
   }
   
   return lRetVal;
}

void DataDesc::Init2D(icp::DATATYPE lDataType,
                      int32_t lSize0, int32_t lSize1)
{
   mNumDim = 2;
   mDataType = lDataType;
   mElementSize[0] = 1;
   mElementSize[1] = 1;
   mSize[0] = lSize0;
   mSize[1] = lSize1;
   mPad[0] = 0;
   mPad[1] = 0;
   mPad[2] = 0;
   mPad[3] = 0;
   
   DerivedInit();
}

void DataDesc::Init2D(icp::DATATYPE lDataType,
                      int32_t lSize0, int32_t lSize1,
                      int32_t lElemSize0, int32_t lElemSize1,
                      int32_t lPad0Low, int32_t lPad0High, int32_t lPad1Low, int32_t lPad1High)
{
   mNumDim = 2;
   mDataType = lDataType;
   mElementSize[0] = lElemSize0;
   mElementSize[1] = lElemSize1;
   mSize[0] = lSize0;
   mSize[1] = lSize1;
   mPad[0] = lPad0Low;
   mPad[1] = lPad0High;
   mPad[2] = lPad1Low;
   mPad[3] = lPad1High;
   
   DerivedInit();
}

void DataDesc::DerivedInit()
{  
   //calculate various strides and core offset assuming the stride of the
   //lowest dimension is 1 element (i.e. no 'gaps' between elements)
   mElementStrideInElement[0] = 1;
   mElementStrideInDataType[0] = 1*mElementSize[0];
   mStrideInDataType[0] = 1;
   mCoreOffsetInDataType = mPad[0] * mElementStrideInDataType[0];
   for (int32_t i=1; i<mNumDim; i++)
   {
      mElementStrideInElement[i] = mElementStrideInElement[i - 1]*(mSize[i - 1] + mPad[2 * (i - 1)] + mPad[(2 * (i - 1)) + 1]);

      mElementStrideInDataType[i] = mElementStrideInElement[i];
      for (int32_t j=0; j<=i; j++)
         mElementStrideInDataType[i] *= mElementSize[j];

      mStrideInDataType[i] = mElementStrideInElement[i];
      for (int32_t j=0; j<i; j++)
         mStrideInDataType[i] *= mElementSize[j];

      mCoreOffsetInDataType += mPad[2 * i] * mElementStrideInDataType[i];
   }

   //calculate total size in data type
   mTotalSizeInDataType = 1;
   for (int32_t i = 0; i < mNumDim; i++)
      mTotalSizeInDataType *= mElementSize[i] * (mSize[i] + mPad[2 * i] + mPad[((2 * i)) + 1]);
   mTotalSizeInBytes = mTotalSizeInDataType*RetDataTypeSizeInBytes(mDataType);
}

//-----------------------------------------------
//Some helper methods (padding, comparison, etc.)
//-----------------------------------------------

void DataDesc_PadCorner(
   uint8_t* lpSrcElem,
   uint8_t* lpDstElem,
   int32_t  lPadWidthInElem,
   int32_t  lPadHeightInElem,
   int32_t  lElemWidthInBytes,
   int32_t  lElemHeightInLines,
   int32_t  lSpan)
{
   for (int32_t i = 0; i<lPadWidthInElem; i++) //padding width (in elem)
   {
      for (int32_t j = 0; j<lPadHeightInElem; j++) //padding height (in elem)
      {
         for (int32_t k = 0; k<lElemWidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l = 0; l<lElemHeightInLines; l++) //height of e0 (in lines)
            {
               lpDstElem[i*lElemWidthInBytes + j*lElemHeightInLines*lSpan + k + l*lSpan] = lpSrcElem[k + l*lSpan];
            }
         }
      }
   }
}

void DataDesc_PadHorEdge(
   uint8_t* lpSrcElem,
   uint8_t* lpDstElem,
   int32_t  lEdgeWidthInElem,
   int32_t  lPadHeightInElem,
   int32_t  lElemWidthInBytes,
   int32_t  lElemHeightInLines,
   int32_t  lSpan)
{
   for (int32_t i = 0; i<lEdgeWidthInElem; i++) //padding width (in elem)
   {
      for (int32_t j = 0; j<lPadHeightInElem; j++) //padding height (in elem)
      {
         for (int32_t k = 0; k<lElemWidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l = 0; l<lElemHeightInLines; l++) //height of e0 (in lines)
            {
               lpDstElem[i*lElemWidthInBytes + j*lElemHeightInLines*lSpan + k + l*lSpan] = lpSrcElem[i*lElemWidthInBytes + k + l*lSpan];
            }
         }
      }
   }
}

void DataDesc_PadVerEdge(
   uint8_t* lpSrcElem,
   uint8_t* lpDstElem,
   int32_t  lPadWidthInElem,
   int32_t  lEdgeHeightInElem,
   int32_t  lElemWidthInBytes,
   int32_t  lElemHeightInLines,
   int32_t  lSpan)
{
   for (int32_t i = 0; i<lPadWidthInElem; i++) //padding width (in elem)
   {
      for (int32_t j = 0; j<lEdgeHeightInElem; j++) //padding height (in elem)
      {
         for (int32_t k = 0; k<lElemWidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l = 0; l<lElemHeightInLines; l++) //height of e0 (in lines)
            {
               lpDstElem[i*lElemWidthInBytes + j*lElemHeightInLines*lSpan + k + l*lSpan] = lpSrcElem[k + (l + (j*lElemHeightInLines))*lSpan];
            }
         }
      }
   }
}

void DataDescElemReplicate2D(DataDesc* lpDatdaDesc, void* lpCoreDataRegion)
{
   int32_t lPadSizeInElemL = lpDatdaDesc->mPad[0];
   int32_t lPadSizeInElemR = lpDatdaDesc->mPad[1];
   int32_t lPadSizeInElemT = lpDatdaDesc->mPad[2];
   int32_t lPadSizeInElemB = lpDatdaDesc->mPad[3];

   uint8_t* lpDataBase = (unsigned char*)lpCoreDataRegion; //pointer to start of region on which elem replication should occur
   int32_t lElementDimX = lpDatdaDesc->mElementSize[0];
   int32_t lElementDimY = lpDatdaDesc->mElementSize[1];
   int32_t lElemWidthInBytes = lElementDimX*RetDataTypeSizeInBytes(lpDatdaDesc->mDataType);
   int32_t lElemHeightInLines = lElementDimY;
   int32_t lWidth = lpDatdaDesc->mSize[0];
   int32_t lHeight = lpDatdaDesc->mSize[1];
   int32_t lSpan = lpDatdaDesc->mStrideInDataType[1] * RetDataTypeSizeInBytes(lpDatdaDesc->mDataType);
   int32_t lWidthInBytes = lWidth*lElementDimX*RetDataTypeSizeInBytes(lpDatdaDesc->mDataType);
   int32_t lHeightInLines = lHeight*lElementDimY;

   uint8_t* lpDataTopLeftCorner = lpDataBase;
   uint8_t* lpDataBotLeftCorner = lpDataBase + lSpan*(lHeightInLines - lElemHeightInLines);
   uint8_t* lpDataTopRightCorner = lpDataTopLeftCorner + lWidthInBytes - lElemWidthInBytes;
   uint8_t* lpDataBotRightCorner = lpDataBotLeftCorner + lWidthInBytes - lElemWidthInBytes;

   uint8_t* lpDst = 0;

   //top left corner
   lpDst = lpDataTopLeftCorner - lPadSizeInElemT*lElemHeightInLines*lSpan - lPadSizeInElemL*lElemWidthInBytes;
   DataDesc_PadCorner(lpDataTopLeftCorner, lpDst, lPadSizeInElemL, lPadSizeInElemT, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //top right corner
   lpDst = lpDataTopRightCorner - lPadSizeInElemT*lElemHeightInLines*lSpan + lElemWidthInBytes;
   DataDesc_PadCorner(lpDataTopRightCorner, lpDst, lPadSizeInElemR, lPadSizeInElemT, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //bottom left corner
   lpDst = lpDataBotLeftCorner + lElemHeightInLines*lSpan - lPadSizeInElemL*lElemWidthInBytes;
   DataDesc_PadCorner(lpDataBotLeftCorner, lpDst, lPadSizeInElemL, lPadSizeInElemB, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //bottom right corner
   lpDst = lpDataBotRightCorner + lElemHeightInLines*lSpan + lElemWidthInBytes;
   DataDesc_PadCorner(lpDataBotRightCorner, lpDst, lPadSizeInElemR, lPadSizeInElemB, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //top edge
   lpDst = lpDataTopLeftCorner - lPadSizeInElemT*lElemHeightInLines*lSpan;
   DataDesc_PadHorEdge(lpDataTopLeftCorner, lpDst, lWidth, lPadSizeInElemT, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //bottom edge
   lpDst = lpDataBotLeftCorner + lElemHeightInLines*lSpan;
   DataDesc_PadHorEdge(lpDataBotLeftCorner, lpDst, lWidth, lPadSizeInElemB, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //left edge
   lpDst = lpDataTopLeftCorner - lPadSizeInElemL*lElemWidthInBytes;
   DataDesc_PadVerEdge(lpDataTopLeftCorner, lpDst, lPadSizeInElemL, lHeight, lElemWidthInBytes, lElemHeightInLines, lSpan);

   //right edge
   lpDst = lpDataTopRightCorner + lElemWidthInBytes;
   DataDesc_PadVerEdge(lpDataTopRightCorner, lpDst, lPadSizeInElemR, lHeight, lElemWidthInBytes, lElemHeightInLines, lSpan);
}

int32_t EqualityCheckRecursive(
   int32_t lNumDim,
   icp::DATATYPE lDataType,
   int32_t lSizeInDataType[],

   void* lpCoreDataRegionA,
   int32_t lStrideInDataTypeA[],
   int32_t lOffsetInDataTypeA,

   void* lpCoreDataRegionB,
   int32_t lStrideInDataTypeB[],
   int32_t lOffsetInDataTypeB,

   int32_t lCurDimIdx, //recursive state variable
   int32_t lIdx[])     //recursive state variable
{
   int32_t lEqualityFlag = 1;

   if (0 == lCurDimIdx)
   {
      //calc offsets
      for (int32_t i = 0; i < lNumDim; i++)
      {
         lOffsetInDataTypeA += lStrideInDataTypeA[i] * lIdx[i];
         lOffsetInDataTypeB += lStrideInDataTypeB[i] * lIdx[i];
      }

      switch (lDataType)
      {
      case icp::DATATYPE_08S:
      case icp::DATATYPE_08U:
      {
         int8_t* lpSrcA = (int8_t*)lpCoreDataRegionA + lOffsetInDataTypeA;
         int8_t* lpSrcB= (int8_t*)lpCoreDataRegionB + lOffsetInDataTypeB;

         for (int32_t i = 0; i < lSizeInDataType[0]; i++)
         {
            if (lpSrcA[i] != lpSrcB[i])
            {
               printf("Mismatch at value [");
               printf(" %d ", i);
               for (int32_t i = 1; i < lNumDim; i++)
                  printf("%d ", lIdx[i]);
               printf("] A = %d B = %d\n", lpSrcA[i], lpSrcB[i]);

               lEqualityFlag = 0;
               break;
            }
         }
         break;
      }

      case icp::DATATYPE_16S:
      case icp::DATATYPE_16U:
      {
         int16_t* lpSrcA = (int16_t*)lpCoreDataRegionA + lOffsetInDataTypeA;
         int16_t* lpSrcB = (int16_t*)lpCoreDataRegionB + lOffsetInDataTypeB;
         for (int32_t i = 0; i < lSizeInDataType[0]; i++)
         {
            if (lpSrcA[i] != lpSrcB[i])
            {
               printf("Mismatch at value [");
               printf(" %d ", i);
               for (int32_t i = 1; i < lNumDim; i++)
                  printf("%d ", lIdx[i]);
               printf("] A = %d B = %d\n", lpSrcA[i], lpSrcB[i]);

               lEqualityFlag = 0;
               break;
            }
         }
         break;
      }

      case icp::DATATYPE_32S:
      case icp::DATATYPE_32U:
      {
         int32_t* lpSrcA = (int32_t*)lpCoreDataRegionA + lOffsetInDataTypeA;
         int32_t* lpSrcB = (int32_t*)lpCoreDataRegionB + lOffsetInDataTypeB;
         for (int32_t i = 0; i < lSizeInDataType[0]; i++)
         {
            if (lpSrcA[i] != lpSrcB[i])
            {
               printf("Mismatch at value [");
               printf(" %d ", i);
               for (int32_t i = 1; i < lNumDim; i++)
                  printf("%d ", lIdx[i]);
               printf("] A = %d B = %d\n", lpSrcA[i], lpSrcB[i]);

               lEqualityFlag = 0;
               break;
            }
         }
         break;
      }
      } 
   }
   else
   {
      for (int32_t i = 0; i < lSizeInDataType[lCurDimIdx]; i++)
      {
         lIdx[lCurDimIdx] = i;
         int32_t lTmp = EqualityCheckRecursive(lNumDim, lDataType, lSizeInDataType,
            lpCoreDataRegionA, lStrideInDataTypeA, lOffsetInDataTypeA,
            lpCoreDataRegionB, lStrideInDataTypeB, lOffsetInDataTypeB,
            lCurDimIdx - 1, lIdx);
         if (0 == lTmp)
         {
            lEqualityFlag = 0;
            break;
         }
      }
   }

   return lEqualityFlag;
}

int32_t DataDescEqualityCheck(DataDesc* lpDataDescA, void* lpCoreDataRegionA, DataDesc* lpDataDescB, void* lpCoreDataRegionB)
{
   int32_t lRetVal = 1;

   //if the comparison makes sense (i.e. dimensions, data types, sizes, etc. match)
   if (lpDataDescA->mNumDim == lpDataDescB->mNumDim &&
       lpDataDescA->mDataType == lpDataDescB->mDataType)
   {
      int32_t lSizeMatchFlag = 1;
      for (int32_t i = 0; i < lpDataDescA->mNumDim; i++)
      {
         if (lpDataDescA->mSize[i] != lpDataDescB->mSize[i] ||
             lpDataDescA->mElementSize[i] != lpDataDescB->mElementSize[i])
         {
            lSizeMatchFlag = 0;
            break;
         }
      }

      if (1 == lSizeMatchFlag)
      {
         int32_t lOffsetInDataTypeA = 0;
         int32_t lOffsetInDataTypeB = 0;
         int32_t lIdx[DESC_MAX_NUM_DIM] = { 0 }; //state var required for TransferChunkRecursive
         int32_t lSizeInDataType[DESC_MAX_NUM_DIM];

         for (int32_t i = 0; i < lpDataDescA->mNumDim; i++)
            lSizeInDataType[i] = lpDataDescA->mSize[i] * lpDataDescA->mElementSize[i];

         return EqualityCheckRecursive(lpDataDescA->mNumDim, lpDataDescA->mDataType, lSizeInDataType,
            lpCoreDataRegionA, lpDataDescA->mStrideInDataType, lOffsetInDataTypeA,
            lpCoreDataRegionB, lpDataDescB->mStrideInDataType, lOffsetInDataTypeB,
            lpDataDescA->mNumDim - 1, lIdx);
      }
      else //size descriptions don't match
      {
         lRetVal = 0;
      }
   }
   else //num dimensions don't match
   {
      lRetVal = 0;
   }

   return lRetVal;
}
