/******************************************************************************
 *  (C) Copyright CogniVue Corporation. 2015 All right reserved.
 *  
 *  Confidential Information
 *
 *  All parts of the CogniVue Program Source are protected by copyright law
 *  and all rights are reserved.
 *  This documentation may not, in whole or in part, be copied, photocopied,
 *  reproduced, translated, or reduced to any electronic medium or machine
 *  readable form without prior consent, in writing, from CogniVue.
 ******************************************************************************/

#include <acf_datadesc.hpp>
#include <stdint.h>
#include <stdlib.h> //for srand
#include <string.h> //memset
#include <stdio.h>
#include <sstream>
#include <iomanip>

using namespace std;

ACF_DataDesc::ACF_DataDesc()
{
   mpData       = 0;
   mpDataPhys   = 0;
   mNdSize.channel = 0;
   mNdSize.width   = 0;
   mNdSize.height  = 0;
   mNdSize.batch   = 0;
   mNdSpan.channel = 0;
   mNdSpan.width   = 0;
   mNdSpan.height  = 0;
   mNdSpan.batch   = 0;
   mElementDataType = icp::DATATYPE_08U;
   mElementDimX = 1;
   mElementDimY = 1;

   for (int32_t i=0; i<4; i++)
      mRoiInfo[i] = 0;
}


ACF_DataDesc::ACF_DataDesc(const ACF_DataDesc &original)
{
    Init4D(original.mpData, original.mpDataPhys,
        original.mNdSize.channel, original.mNdSize.width, original.mNdSize.height, original.mNdSize.batch,
        original.mNdSpan.channel, original.mNdSpan.width, original.mNdSpan.height, original.mNdSpan.batch,
        original.mElementDataType, original.mElementDimX, original.mElementDimY,
        original.mRoiInfo[0], original.mRoiInfo[1], original.mRoiInfo[2], original.mRoiInfo[3]);
}

ACF_DataDesc::ACF_DataDesc(void* const lpData,
                           void* const lpDataPhys,
                           int32_t lWidth,
                           int32_t lHeight,
                           int32_t lSpan,
                           icp::DATATYPE lElementDataType,
                           int32_t lElementDimX,
                           int32_t lElementDimY,
                           int32_t lRoiInfoL,
                           int32_t lRoiInfoR,
                           int32_t lRoiInfoT,
                           int32_t lRoiInfoB)
{
   Init2D(lpData, lpDataPhys,
          lWidth, lHeight, lSpan,
          lElementDataType, lElementDimX, lElementDimY,
          lRoiInfoL, lRoiInfoR, lRoiInfoT, lRoiInfoB);
}

ACF_DataDesc::ACF_DataDesc(void* const lpData,
                           void* const lpDataPhys,
                           int32_t lSize,
                           icp::DATATYPE lElementDataType,
                           int32_t lElementDimX)
{
   Init1D(lpData, lpDataPhys,
         lSize,
         lElementDataType, lElementDimX);
}

ACF_DataDesc::ACF_DataDesc(void* const lpData,
                           void* const lpDataPhys,
                           int32_t lChannel,
                           int32_t lWidth,
                           int32_t lHeight,
                           int32_t lBatch,
                           int32_t lChannelSpan,
                           int32_t lWidthSpan,
                           int32_t lHeightSpan,
                           int32_t lBatchSpan,
                           icp::DATATYPE lElementDataType,
                           int32_t lElementDimX,
                           int32_t lElementDimY,
                           int32_t lRoiInfoL,
                           int32_t lRoiInfoR,
                           int32_t lRoiInfoT,
                           int32_t lRoiInfoB)
{
   Init4D(lpData, lpDataPhys,
          lChannel, lWidth, lHeight, lBatch,
          lChannelSpan, lWidthSpan, lHeightSpan, lBatchSpan,
          lElementDataType,
          lElementDimX, lElementDimY,
          lRoiInfoL, lRoiInfoR, lRoiInfoT, lRoiInfoB);
}


void ACF_DataDesc::Init1D (void* const lpData,
                           void* const lpDataPhys,
                           int32_t lSize,
                           icp::DATATYPE lElementDataType,
                           int32_t lElementDimX)
{
   mpData           = lpData;
   mpDataPhys       = lpDataPhys;
   mNdSize.channel  = 1;
   mNdSize.width    = lSize;
   mNdSize.height   = 1;
   mNdSize.batch    = 1;
   mNdSpan.channel  = RetDataTypeSizeInBytes(lElementDataType)*lElementDimX;
   mNdSpan.width    = mNdSize.channel*mNdSpan.channel;
   mNdSpan.height   = lSize*RetDataTypeSizeInBytes(lElementDataType)*lElementDimX;
   mNdSpan.batch    = mNdSize.height*mNdSpan.height;
   mElementDataType = lElementDataType;
   mElementDimX     = lElementDimX;
   mElementDimY     = 1;

   for (int32_t i=0; i<4; i++)
      mRoiInfo[i] = 0;

   VerifyPhysPtrAlignment();
}

void ACF_DataDesc::Init2D(void* const lpData,
                          void* const lpDataPhys,
                          int32_t lWidth,
                          int32_t lHeight,
                          int32_t lSpan,
                          icp::DATATYPE lElementDataType,
                          int32_t lElementDimX,
                          int32_t lElementDimY,
                          int32_t lRoiInfoL,
                          int32_t lRoiInfoR,
                          int32_t lRoiInfoT,
                          int32_t lRoiInfoB)
{
   mpData           = lpData;
   mpDataPhys       = lpDataPhys;
   mNdSize.channel  = 1;
   mNdSize.width    = lWidth;
   mNdSize.height   = lHeight;
   mNdSize.batch    = 1;
   mNdSpan.channel  = RetDataTypeSizeInBytes(lElementDataType)*lElementDimX;
   mNdSpan.width    = mNdSize.channel*mNdSpan.channel;
   mNdSpan.height   = lSpan;
   mNdSpan.batch    = mNdSize.height*mNdSpan.height;
   mElementDataType = lElementDataType;
   mElementDimX     = lElementDimX;
   mElementDimY     = lElementDimY;

   mRoiInfo[0] = lRoiInfoL;
   mRoiInfo[1] = lRoiInfoR;
   mRoiInfo[2] = lRoiInfoT;
   mRoiInfo[3] = lRoiInfoB;

//    printf("mpData:           %d\n", mpData);
//    printf("mpDataPhys:       %d\n", mpDataPhys);
//    printf("mNdSize.width:    %d\n", mNdSize.width);
//    printf("mNdSize.height:   %d\n", mNdSize.height);
//    printf("mNdSpan.height:   %d\n", mNdSpan.height);
//    printf("mElementDataType: %d\n", mElementDataType);
//    printf("mElementDimX:     %d\n", mElementDimX);
//    printf("mElementDimY:     %d\n", mElementDimY);
//    printf("--------------------\n", mElementDimY);
   
   VerifyPhysPtrAlignment();
}

void ACF_DataDesc::Init4D(void* const lpData,
                          void* const lpDataPhys,
                          int32_t lChannel,
                          int32_t lWidth,
                          int32_t lHeight,
                          int32_t lBatch,
                          int32_t lChannelSpan,
                          int32_t lWidthSpan,
                          int32_t lHeightSpan,
                          int32_t lBatchSpan,
                          icp::DATATYPE lElementDataType,
                          int32_t lElementDimX,
                          int32_t lElementDimY,
                          int32_t lRoiInfoL,
                          int32_t lRoiInfoR,
                          int32_t lRoiInfoT,
                          int32_t lRoiInfoB)
{
   mpData           = lpData;
   mpDataPhys       = lpDataPhys;
   mNdSize.channel  = lChannel;
   mNdSize.width    = lWidth;
   mNdSize.height   = lHeight;
   mNdSize.batch    = lBatch;
   mNdSpan.channel  = lChannelSpan;
   mNdSpan.width    = lWidthSpan;
   mNdSpan.height   = lHeightSpan;
   mNdSpan.batch    = lBatchSpan;
   mElementDataType = lElementDataType;
   mElementDimX     = lElementDimX;
   mElementDimY     = lElementDimY;

   mRoiInfo[0] = lRoiInfoL;
   mRoiInfo[1] = lRoiInfoR;
   mRoiInfo[2] = lRoiInfoT;
   mRoiInfo[3] = lRoiInfoB;

//    printf("mpData:           %d\n", mpData);
//    printf("mpDataPhys:       %d\n", mpDataPhys);
//    printf("mNdSize.width:    %d\n", mNdSize.width);
//    printf("mNdSize.height:   %d\n", mNdSize.height);
//    printf("mNdSpan.height:   %d\n", mNdSpan.height);
//    printf("mElementDataType: %d\n", mElementDataType);
//    printf("mElementDimX:     %d\n", mElementDimX);
//    printf("mElementDimY:     %d\n", mElementDimY);
//    printf("--------------------\n", mElementDimY);
   
   VerifyPhysPtrAlignment();
}

int32_t ACF_DataDesc::Init2D_ROI(const ACF_DataDesc& lContigDataDesc,
                                 int32_t lROI_XOffset,
                                 int32_t lROI_YOffset,
                                 int32_t lROI_Width,
                                 int32_t lROI_Height)
{
   int32_t lRetVal = 0;

   icp::DATATYPE lElementDataType = lContigDataDesc.RetElementDataType();
   int32_t lElementDimX = lContigDataDesc.RetElementDimX();
   int32_t lElementDimY = lContigDataDesc.RetElementDimY();

   int32_t lE0WidthInBytes   = RetDataTypeSizeInBytes(lElementDataType);
   int32_t lSpan             = lContigDataDesc.RetSpan();
   int32_t lE0Span           = lSpan*lElementDimY;
   int32_t lRoiOffsetInBytes = lE0WidthInBytes*lROI_XOffset + lROI_YOffset*lE0Span;

   //verify that specified ROI falls within the boundaries of lContigDataDesc
   int32_t lRoiInfoL = lROI_XOffset;
   int32_t lRoiInfoR = lContigDataDesc.RetWidth() - lROI_XOffset - lROI_Width;
   int32_t lRoiInfoT = lROI_YOffset;
   int32_t lRoiInfoB = lContigDataDesc.RetHeight() - lROI_YOffset - lROI_Height;

   if (lROI_XOffset >= 0 && lROI_XOffset < lContigDataDesc.RetWidth()  &&
       lROI_YOffset >= 0 && lROI_YOffset < lContigDataDesc.RetHeight() &&
       lRoiInfoR >= 0 &&
       lRoiInfoB >= 0)
   {
      Init2D((void*)(((intptr_t)lContigDataDesc.RetDataPtr()) + lRoiOffsetInBytes),
             (void*)(((intptr_t)lContigDataDesc.RetDataPtrPhys()) + lRoiOffsetInBytes),
             lROI_Width, lROI_Height, lSpan,
             lElementDataType, lElementDimX, lElementDimY,
             lRoiInfoL, lRoiInfoR, lRoiInfoT, lRoiInfoB);			 
   }
   else
   {
      printf ("ACF_DataDesc::Init2D_ROI( ... ) -> the ROI does not fall within the specified parent data region\n");
      lRetVal = 1;
   }

   return lRetVal;
}

void ACF_DataDesc::UpdatePtrs(void* const lpData,
                              void* const lpDataPhys)
{
   mpData     = lpData;
   mpDataPhys = lpDataPhys;

   VerifyPhysPtrAlignment();
}

void* ACF_DataDesc::RetDataPtr() const
{
   return mpData;
}

void* ACF_DataDesc::RetDataPtrPhys() const
{
   return mpDataPhys;
}

icp::DATATYPE ACF_DataDesc::RetElementDataType() const
{
   return mElementDataType;
}

int32_t ACF_DataDesc::RetElementDimX() const
{
   return mElementDimX;
}

int32_t ACF_DataDesc::RetElementDimY() const
{
   return mElementDimY;
}

int32_t ACF_DataDesc::RetChannel() const
{
   return mNdSize.channel;
}

int32_t ACF_DataDesc::RetWidth() const
{
   return mNdSize.width;
}

int32_t ACF_DataDesc::RetHeight() const
{
   return mNdSize.height;
}

int32_t ACF_DataDesc::RetBatch() const
{
   return mNdSize.batch;
}

int32_t ACF_DataDesc::RetSpan() const
{
   return mNdSpan.height;
}

int32_t ACF_DataDesc::RetChannelSpan() const
{
   return mNdSpan.channel;
}

int32_t ACF_DataDesc::RetWidthSpan() const
{
   return mNdSpan.width;
}

int32_t ACF_DataDesc::RetHeightSpan() const
{
   return mNdSpan.height;
}

int32_t ACF_DataDesc::RetBatchSpan() const
{
   return mNdSpan.batch;
}

int32_t ACF_DataDesc::RetStride_ElementDataType() const
{
   return (mNdSpan.height/RetDataTypeSizeInBytes(mElementDataType));
}

int32_t ACF_DataDesc::RetStride_ElementX() const
{
   return (mNdSpan.height/(RetDataTypeSizeInBytes(mElementDataType)*mElementDimX));
}

int32_t ACF_DataDesc::RetRoiInfoL() const
{
   return mRoiInfo[0];
}

int32_t ACF_DataDesc::RetRoiInfoR() const
{
   return mRoiInfo[1];
}

int32_t ACF_DataDesc::RetRoiInfoT() const
{
   return mRoiInfo[2];
}

int32_t ACF_DataDesc::RetRoiInfoB() const
{
   return mRoiInfo[3];
}

int32_t ACF_DataDesc::VerifyPhysPtrAlignment()
{
   int32_t lRetVal = 0;

   intptr_t lMask = (~ACF_DATADESC_MIN_BYTE_ALIGNMENT)&(ACF_DATADESC_MIN_BYTE_ALIGNMENT-1);

   //verify if the starting point is aligned
   if (((intptr_t)mpDataPhys & lMask) != 0)
   {
      //printf("ACF_DataDesc::VerifyPtrAlignment(0x%08X) -> data region starting point is not %d byte aligned\n", (uintptr_t)mpDataPhys, ACF_DATADESC_MIN_BYTE_ALIGNMENT);
      lRetVal = 1;
   }

   //verify that if mHeight > 1, that the starting point + span is also aligned
   if (mNdSize.height > 1)
   {
      if ((((intptr_t)mpDataPhys + mNdSpan.height) & lMask) != 0)
      {
         //printf("ACF_DataDesc::VerifyPtrAlignment(0x%08X) -> data region starting point + span (%d) is not %d byte aligned\n", (uintptr_t)mpDataPhys, mNdSpan.height, ACF_DATADESC_MIN_BYTE_ALIGNMENT);
         lRetVal = 1;
      }
   }

   return lRetVal;
}

#define EQUALITY_DEBUG 0

bool ACF_DataDesc::operator==(ACF_DataDesc const& lCdd)
{
   bool lRetVal = true;

   //if the comparison makes sense
   if (mNdSize.width == lCdd.mNdSize.width &&
       mNdSize.height == lCdd.mNdSize.height &&
       mElementDataType == lCdd.mElementDataType &&
       mElementDimX == lCdd.mElementDimX &&
       mElementDimY == lCdd.mElementDimY)
   {
      int32_t lWidth    = mNdSize.width*mElementDimX*RetDataTypeSizeInBytes(mElementDataType);
      int32_t lHeight   = mNdSize.height*mElementDimY;
      uint8_t* lpDataA = (uint8_t*)mpData;
      uint8_t* lpDataB = (uint8_t*)lCdd.mpData;
      int32_t lSpanA    = mNdSpan.height;
      int32_t lSpanB    = lCdd.mNdSpan.height;

      for (int32_t i=0; i<lHeight; i++)
      {
         for (int32_t j=0; j<lWidth; j++)
         {
            if (lpDataA[j] != lpDataB[j])
            {
               //printf("%d|%d(%d,%d) ",lpDataA[j], lpDataB[j], j, i);
               lRetVal = false;

               #if (EQUALITY_DEBUG == 1)
               printf("X");
               #endif
            }
            else
            {
               #if (EQUALITY_DEBUG == 1)
               printf("-");
               #endif
            }
         }

         #if (EQUALITY_DEBUG == 1)
         printf("\n");
         #endif

         lpDataA += lSpanA;
         lpDataB += lSpanB;
      }
   }
   else
   {
      lRetVal = false;
   }

   return lRetVal;
}

int32_t gCDDRandomGeneratorSeeded = 0;
int32_t gCDDRandomSeedValue = 0x12345678;

void ACF_DataDesc::FillRandom(int32_t lSeedVal)
{
   //seed random number generator only if it has not already been seeded
   if (lSeedVal != 0)
   {
      srand(lSeedVal);
      gCDDRandomGeneratorSeeded = 1;
   }
   else //user didn't specify a seed value
   {
      if (!gCDDRandomGeneratorSeeded)
      {
         srand(gCDDRandomSeedValue); //use the global seed value
         gCDDRandomGeneratorSeeded = 1;
      }
   }

   int lWidthInBytes = mNdSize.width*mElementDimX*RetDataTypeSizeInBytes(mElementDataType);

   unsigned char* lpData = (unsigned char*)mpData;
   for (int32_t j=0; j<mNdSize.height*mElementDimY; j++)
   {
      for (int32_t i=0; i<lWidthInBytes; i++)
      {
         *(lpData + (j*mNdSpan.height + i)) = rand() % 256;
      }
   }
}

void ACF_DataDesc::FillZero()
{
   int lWidthInBytes = mNdSize.width*mElementDimX*RetDataTypeSizeInBytes(mElementDataType);

   for (int32_t j=0; j<mNdSize.height*mElementDimY; j++)
   {
      memset((void*)((char*)mpData + (j*mNdSpan.height)), 0, lWidthInBytes);
   }
}

int32_t ACF_DataDesc::CopyTo(ACF_DataDesc& lDst)
{
   int32_t lRetVal = 0;

   //if all the relevant parameters match
   if (lDst.mNdSize.width == mNdSize.width &&
       lDst.mNdSize.height == mNdSize.height &&
       lDst.mElementDataType == mElementDataType &&
       lDst.mElementDimX == mElementDimX &&
       lDst.mElementDimY == mElementDimY)
   {
      int32_t lWidthInBytes = mNdSize.width*mElementDimX*RetDataTypeSizeInBytes(mElementDataType);
      int32_t lHeightInLines = mNdSize.height*mElementDimY;
      int32_t lSrcSpan = mNdSpan.height;
      int32_t lDstSpan = lDst.RetSpan();
      uint8_t* lpSrc = (uint8_t*)mpData;
      uint8_t* lpDst = (uint8_t*)lDst.RetDataPtr();

      for (int32_t y = 0; y<lHeightInLines; y++)
      {
         memcpy((void*)lpDst, (void*)lpSrc, lWidthInBytes);
         lpSrc += lSrcSpan;
         lpDst += lDstSpan;
      }
   }
   else
   {
      //src and dst are not the same size
      lRetVal = 1;
   }

   return lRetVal;
}

void PadCorner(uint8_t* lpSrcE0,
               uint8_t* lpDstE0,
               int32_t  lPadWidthInE0,
               int32_t  lPadHeightInE0,
               int32_t  lE0WidthInBytes,
               int32_t  lE0HeightInLines,
               int32_t  lSpan)
{
   for (int32_t i=0; i<lPadWidthInE0; i++) //padding width (in e0)
   {
      for (int32_t j=0; j<lPadHeightInE0; j++) //padding height (in e0)
      {
         for (int32_t k=0; k<lE0WidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l=0; l<lE0HeightInLines; l++) //height of e0 (in lines)
            {
               lpDstE0[i*lE0WidthInBytes + j*lE0HeightInLines*lSpan + k + l*lSpan] = lpSrcE0[k + l*lSpan];
            }
         }
      }
   }
}

void PadHorEdge(uint8_t* lpSrcE0,
                uint8_t* lpDstE0,
                int32_t  lEdgeWidthInE0,
                int32_t  lPadHeightInE0,
                int32_t  lE0WidthInBytes,
                int32_t  lE0HeightInLines,
                int32_t  lSpan)
{
   for (int32_t i=0; i<lEdgeWidthInE0; i++) //padding width (in e0)
   {
      for (int32_t j=0; j<lPadHeightInE0; j++) //padding height (in e0)
      {
         for (int32_t k=0; k<lE0WidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l=0; l<lE0HeightInLines; l++) //height of e0 (in lines)
            {
               lpDstE0[i*lE0WidthInBytes + j*lE0HeightInLines*lSpan + k + l*lSpan] = lpSrcE0[i*lE0WidthInBytes + k + l*lSpan];
            }
         }
      }
   }
}

void PadVerEdge(uint8_t* lpSrcE0,
                uint8_t* lpDstE0,
                int32_t  lPadWidthInE0,
                int32_t  lEdgeHeightInE0,
                int32_t  lE0WidthInBytes,
                int32_t  lE0HeightInLines,
                int32_t  lSpan)
{
   for (int32_t i=0; i<lPadWidthInE0; i++) //padding width (in e0)
   {
      for (int32_t j=0; j<lEdgeHeightInE0; j++) //padding height (in e0)
      {
         for (int32_t k=0; k<lE0WidthInBytes; k++) //width of e0 (in bytes)
         {
            for (int32_t l=0; l<lE0HeightInLines; l++) //height of e0 (in lines)
            {
               lpDstE0[i*lE0WidthInBytes + j*lE0HeightInLines*lSpan + k + l*lSpan] = lpSrcE0[k + (l+(j*lE0HeightInLines))*lSpan];
            }
         }
      }
   }
}

void ACF_DataDesc::E0Replicate(int32_t lPadSizeInE0L,
                               int32_t lPadSizeInE0R,
                               int32_t lPadSizeInE0T,
                               int32_t lPadSizeInE0B)
{
   uint8_t* lpDataBase = (unsigned char*)mpData; //pointer to start of region on which e0 replication should occur
   int32_t lE0WidthInBytes   = mElementDimX*RetDataTypeSizeInBytes(mElementDataType);
   int32_t lE0HeightInLines = mElementDimY;
   int32_t lWidth            = mNdSize.width;
   int32_t lHeight           = mNdSize.height;
   int32_t lSpan             = mNdSpan.height;
   int32_t lWidthInBytes     = mNdSize.width*mElementDimX*RetDataTypeSizeInBytes(mElementDataType);
   int32_t lHeightInLines    = mNdSize.height*mElementDimY;

   uint8_t* lpDataTopLeftCorner  = lpDataBase;
   uint8_t* lpDataBotLeftCorner  = lpDataBase + lSpan*(lHeightInLines-lE0HeightInLines);
   uint8_t* lpDataTopRightCorner = lpDataTopLeftCorner + lWidthInBytes - lE0WidthInBytes;
   uint8_t* lpDataBotRightCorner = lpDataBotLeftCorner + lWidthInBytes - lE0WidthInBytes;

   uint8_t* lpDst = 0;

   //top left corner
   lpDst = lpDataTopLeftCorner - lPadSizeInE0T*lE0HeightInLines*lSpan - lPadSizeInE0L*lE0WidthInBytes;
   PadCorner(lpDataTopLeftCorner, lpDst, lPadSizeInE0L, lPadSizeInE0T, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //top right corner
   lpDst = lpDataTopRightCorner - lPadSizeInE0T*lE0HeightInLines*lSpan + lE0WidthInBytes;
   PadCorner(lpDataTopRightCorner, lpDst, lPadSizeInE0R, lPadSizeInE0T, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //bottom left corner
   lpDst = lpDataBotLeftCorner + lE0HeightInLines*lSpan - lPadSizeInE0L*lE0WidthInBytes;
   PadCorner(lpDataBotLeftCorner, lpDst, lPadSizeInE0L, lPadSizeInE0B, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //bottom right corner
   lpDst = lpDataBotRightCorner + lE0HeightInLines*lSpan + lE0WidthInBytes;
   PadCorner(lpDataBotRightCorner, lpDst, lPadSizeInE0R, lPadSizeInE0B, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //top edge
   lpDst = lpDataTopLeftCorner - lPadSizeInE0T*lE0HeightInLines*lSpan;
   PadHorEdge(lpDataTopLeftCorner, lpDst, lWidth, lPadSizeInE0T, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //bottom edge
   lpDst = lpDataBotLeftCorner + lE0HeightInLines*lSpan;
   PadHorEdge(lpDataBotLeftCorner, lpDst, lWidth, lPadSizeInE0B, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //left edge
   lpDst = lpDataTopLeftCorner - lPadSizeInE0L*lE0WidthInBytes;
   PadVerEdge(lpDataTopLeftCorner, lpDst, lPadSizeInE0L, lHeight, lE0WidthInBytes, lE0HeightInLines, lSpan);

   //right edge
   lpDst = lpDataTopRightCorner + lE0WidthInBytes;
   PadVerEdge(lpDataTopRightCorner, lpDst, lPadSizeInE0R, lHeight, lE0WidthInBytes, lE0HeightInLines, lSpan);
}

int32_t ACF_DataDesc::DumpToFile(const char* lFilenameRoot)
{
  int32_t lRetVal = 0;

#ifndef APEX2_EMULATE
   stringstream lUpdatedFileName;
   lUpdatedFileName << lFilenameRoot << "__0x" << setbase(16) << setw(8) << setfill('0') << (intptr_t)mpDataPhys << ".bin";
   
   FILE* lpFile = fopen(lUpdatedFileName.str().c_str(), "wb");
   if (lpFile)
   {
      void* lpLineOfBytes = mpData;
      int32_t lNumBytesWritten = 0;
      int32_t lNumBytesPerLine = mNdSpan.height;
      for (int32_t y = 0; y<(mNdSize.height*mElementDimY); y++)
      {
         lNumBytesWritten = fwrite(lpLineOfBytes, 1, lNumBytesPerLine, lpFile);
         if (lNumBytesWritten != lNumBytesPerLine)
         {
            lRetVal = 1;
            break;
         }
         lpLineOfBytes = (void*)((intptr_t)(lpLineOfBytes) + lNumBytesPerLine);
      }

      fclose(lpFile);
   }
#endif

   return lRetVal;
}

template<typename T>
void DumpToStdout_helper(T* lpData, int32_t lWidth, int32_t lHeight, int32_t lStride)
{
   for (int32_t i=0; i<lHeight; i++)
   {
      for (int32_t j=0; j<lWidth; j++)
      {
         printf("%d ", lpData[j]);
      }
      printf("\n");
      lpData += lStride;
   }
}

void ACF_DataDesc::DumpToStdout()
{
   int32_t lWidth  = mNdSize.width*mElementDimX;
   int32_t lHeight = mNdSize.height*mElementDimY;
   int32_t lStride;

   switch (mElementDataType)
   {
   case icp::DATATYPE_08U:
      lStride = mNdSpan.height;
      DumpToStdout_helper((uint8_t*) mpData, lWidth, lHeight, lStride);
      break;
   case icp::DATATYPE_08S:
      lStride = mNdSpan.height;
      DumpToStdout_helper((int8_t*) mpData, lWidth, lHeight, lStride);
      break;
   case icp::DATATYPE_16U:
      lStride = mNdSpan.height >> 1;
      DumpToStdout_helper((uint16_t*) mpData, lWidth, lHeight, lStride);
      break;
   case icp::DATATYPE_16S:
      lStride = mNdSpan.height >> 1;
      DumpToStdout_helper((int16_t*) mpData, lWidth, lHeight, lStride);
      break;
   case icp::DATATYPE_32U:
      lStride = mNdSpan.height >> 2;
      DumpToStdout_helper((uint32_t*) mpData, lWidth, lHeight, lStride);
      break;
   case icp::DATATYPE_32S:
      lStride = mNdSpan.height >> 2;
      DumpToStdout_helper((int32_t*) mpData, lWidth, lHeight, lStride);
      break;
   }
}

