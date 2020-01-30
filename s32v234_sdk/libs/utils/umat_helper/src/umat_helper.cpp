/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2017 NXP Semiconductor;
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

#include <sumat.hpp>
#include <umat_helper.h>
#include <icp_datatype.h>
#include <string.h> //memset

namespace vsdk
{

  //-----------------------------------------------------------------------------------------------------------
  //function for calculating buffer size information (i.e. buffer with space around it for padding or overflow)
  //-----------------------------------------------------------------------------------------------------------
  void UMatHelper::CalcBufferInfo(int32_t lWidth,
    int32_t lHeight,
    icp::DATATYPE lElementDataType,
    int32_t lElementDimX,
    int32_t lElementDimY,
    int32_t lPadL,
    int32_t lPadR,
    int32_t lPadT,
    int32_t lPadB,
    int32_t lMinAlignmentInBytes,
    int32_t lWorstCaseOverflowRightInBytes,
    int32_t lWorstCaseOverflowBottomInElements,
    int32_t& lBufferWidthInElem,
    int32_t& lBufferHeightInElem,
    int32_t& lBufferSizeInBytes,
    int32_t& lBufferSpan,
    int32_t& lBufferCoreDataOffsetInBytes,
    int32_t& lBufferCoreDataOffsetXInElem,
    int32_t& lBufferCoreDataOffsetYInElem)
  {
    //IO buffers will be allocated so they can satisfy the specified requirements (e.g. alignment, overflow space on
    //right and bottom, etc.)

    //Notes:
    //- span must always be divisible by lElementWidthInBytes (i.e. user must be be able to navigate the buffer
    //  via element and stride)
    //- there are two important alignment points: the start of buffer and the start of the core data region (and all subsequent
    //  lines assuming a 2D structure, so the span must also respect minimum alignment)

    int32_t lElementWidthInBytes = RetDataTypeSizeInBytes(lElementDataType)*lElementDimX;

    //ensure that the user-specified left padding region width is itself divisible by both lElementWidthInBytes and MIN_ALIGNMENT_IN_BYTES
    //so it does not misalign the start of the core data region
    int32_t lBufferPadLeftInBytes = lPadL*lElementWidthInBytes; //minimum number of bytes required to satisfy user-defined left padding
    while (lBufferPadLeftInBytes%lMinAlignmentInBytes != 0)
      lBufferPadLeftInBytes += lElementWidthInBytes;

    int32_t lCoreDataWidthInBytes = lWidth*lElementWidthInBytes;
    int32_t lBufferOverflowPadRightInBytes = (((lWorstCaseOverflowRightInBytes - 1) + (lElementWidthInBytes - 1)) / lElementWidthInBytes)*lElementWidthInBytes;
    int32_t lBufferPadRightInBytes = lPadR*lElementWidthInBytes; //minimum number of bytes required to satisfy user-defined right padding
    lBufferPadRightInBytes += lBufferOverflowPadRightInBytes;
    while ((lCoreDataWidthInBytes + lBufferPadRightInBytes) % lMinAlignmentInBytes != 0)
      lBufferPadRightInBytes += lElementWidthInBytes;

    int32_t lCoreDataHeightInLines = lHeight*lElementDimY;
    int32_t lPadTopInLines = lPadT*lElementDimY;
    int32_t lPadBotInLines = lPadB*lElementDimY;
    int32_t lBufferOverflowPadBotInLines = (lWorstCaseOverflowBottomInElements - 1)*lElementDimY;

    int32_t lBufferHeightInLines = lPadTopInLines + lCoreDataHeightInLines + lPadBotInLines + lBufferOverflowPadBotInLines;

    lBufferWidthInElem = (lBufferPadLeftInBytes / lElementWidthInBytes) + lWidth + (lBufferPadRightInBytes / lElementWidthInBytes);
    lBufferHeightInElem = (lBufferHeightInLines / lElementDimY);
    lBufferSpan = lBufferPadLeftInBytes + lCoreDataWidthInBytes + lBufferPadRightInBytes;
    lBufferSizeInBytes = lBufferSpan*lBufferHeightInLines;

    lBufferCoreDataOffsetInBytes = lPadTopInLines*lBufferSpan + lBufferPadLeftInBytes;
    lBufferCoreDataOffsetXInElem = (lBufferPadLeftInBytes / lElementWidthInBytes);
    lBufferCoreDataOffsetYInElem = lPadTopInLines;
  }

  vsdk::SUMat UMatHelper::CreateAcfFriendlyUMat(int32_t aRows, int32_t aCols, int32_t aType,
    int32_t aPadL, int32_t aPadR, int32_t aPadT, int32_t aPadB)
  {
    //values for these parameters are selected based on the needs of ACF/APEX
    const int32_t lMinAlignmentInBytes = 8;
    const int32_t lWorstCaseOverflowRightInBytes = 128; //currently based on ACF worst case chunk width in bytes
    const int32_t lWorstCaseOverflowBottomInElements = 32;  //currently based on ACF worst case chunk height in elements

    int32_t lBufferWidthInElem = 0;
    int32_t lBufferHeightInElem = 0;
    int32_t lBufferSizeInBytes = 0;
    int32_t lBufferSpan = 0;
    int32_t lBufferCoreDataOffsetInBytes = 0;
    int32_t lBufferCoreDataOffsetXInElem = 0;
    int32_t lBufferCoreDataOffsetYInElem = 0;

    //convert from 'cv' datatype to 'icp' datatype
    uint32_t lCvDataType = aType & 0x00000007; //TODO: is there a macro for this?
    icp::DATATYPE lIcpDataType = icp::DATATYPE_08U;
    if (lCvDataType == VSDK_CV_8U)
    {
      lIcpDataType = icp::DATATYPE_08U;
    }
    else
    {
      if (lCvDataType == VSDK_CV_8S)
      {
        lIcpDataType = icp::DATATYPE_08S;
      }
      else
      {
        if (lCvDataType == VSDK_CV_16U)
        {
          lIcpDataType = icp::DATATYPE_16U;
        }
        else
        {
          if (lCvDataType == VSDK_CV_16S)
          {
            lIcpDataType = icp::DATATYPE_16S;
          }
          else
          {
            if (lCvDataType == VSDK_CV_32S)
            {
              lIcpDataType = icp::DATATYPE_32S;
            }
            else
            {
              lIcpDataType = icp::DATATYPE_32U;
            }
          }
        }
      }
    }
    

    int32_t lElementDimX = VSDK_CV_ELEM_SIZE(aType);
    int32_t lElementDimY = 1; //uMat offers no support for specifying y dimension of element

    CalcBufferInfo(aCols, aRows, lIcpDataType, lElementDimX, lElementDimY, aPadL, aPadR, aPadT, aPadB,
      lMinAlignmentInBytes, lWorstCaseOverflowRightInBytes, lWorstCaseOverflowBottomInElements,
      lBufferWidthInElem, lBufferHeightInElem, lBufferSizeInBytes, lBufferSpan, lBufferCoreDataOffsetInBytes, lBufferCoreDataOffsetXInElem, lBufferCoreDataOffsetYInElem);

    //create 'base' SUMat from which the ROI will be derived (it is assumed this allocation is at least 8-byte aligned!)
    vsdk::SUMat lBaseUMat = vsdk::SUMat(lBufferHeightInElem, lBufferWidthInElem, aType);

    //create 'ROI' SUMat on top of 'base' SUMat
    vsdk::SUMat lRoiUMat = vsdk::SUMat(lBaseUMat, vsdk::Rect(lBufferCoreDataOffsetXInElem, lBufferCoreDataOffsetYInElem, aCols, aRows));

    return lRoiUMat;
  }

  vsdk::SUMat UMatHelper::CreateAcfFriendlyUMat(int32_t lRows, int32_t lCols, int32_t lType)
  {
    return CreateAcfFriendlyUMat(lRows, lCols, lType, 0, 0, 0, 0);
  }

  void UMatHelper::FillZero(vsdk::SUMat& aUMat)
  {
    vsdk::SMat lMat = aUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpData = lMat.data;
    int32_t lWidthInBytes = lMat.cols*lMat.elemSize();
    int32_t lSpan = aUMat.step[0];
    int32_t lHeightInLines = lMat.rows;

    for (int y = 0; y < lHeightInLines; y++)
    {
      memset((void*)((char*)lpData + (y*lSpan)), 0, lWidthInBytes);
    }
  }

  void UMatHelper::FillRandom(vsdk::SUMat& aUMat, uint32_t* apSeedVal)
  {
    vsdk::SMat lMat = aUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpData = lMat.data;
    int32_t lWidthInBytes = lMat.cols*lMat.elemSize();
    int32_t lSpan = aUMat.step[0];
    int32_t lHeightInLines = lMat.rows;

    for (int32_t y = 0; y < lHeightInLines; y++)
    {
      for (int32_t x = 0; x < lWidthInBytes; x++)
      {
#ifdef APEX2_EMULATE
        *(lpData + (y*lSpan + x)) = rand() % 256;
#else
        *(lpData + (y*lSpan + x)) = rand_r(apSeedVal) % 256;
#endif
      }
    }
  }

  void UMatHelper::FillValue(vsdk::SUMat& aUMat, uint32_t* apVal)
  {
    vsdk::SMat lMat = aUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpData = lMat.data;
    int32_t lWidthInBytes = lMat.cols*lMat.elemSize();
    int32_t lSpan = aUMat.step[0];
    int32_t lHeightInLines = lMat.rows;

    for (int32_t y = 0; y < lHeightInLines; y++)
    {
      for (int32_t x = 0; x < lWidthInBytes; x++)
      {
        *(lpData + (y*lSpan + x)) = *(apVal + (y*lSpan + x));
      }
    }
  }

  void UMatHelper::FillValue(vsdk::SUMat& aUMat, uint8_t aVal)
  {
    vsdk::SMat lMat = aUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpData = lMat.data;
    int32_t lWidthInBytes = lMat.cols*lMat.elemSize();
    int32_t lSpan = aUMat.step[0];
    int32_t lHeightInLines = lMat.rows;

    for (int32_t y = 0; y < lHeightInLines; y++)
    {
      for (int32_t x = 0; x < lWidthInBytes; x++)
      {
        *(lpData + (y*lSpan + x)) = aVal;
      }
    }
  }

  // returns the count of diff between images
  // returns -1 is different image sizes
#define DO_DEBUG_PRINT 0
  int32_t UMatHelper::Compare(vsdk::SUMat& aUMatA, vsdk::SUMat& aUMatB)
  {
    int lRetVal = 0;

    vsdk::SMat lMatA = aUMatA.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpDataA = lMatA.data;
    int32_t lWidthInBytesA = lMatA.cols*lMatA.elemSize();
    int32_t lHeightInLinesA = lMatA.rows;
    int32_t lSpanA = aUMatA.step[0];

    vsdk::SMat lMatB = aUMatB.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
    uint8_t* lpDataB = lMatB.data;
    int32_t lWidthInBytesB = lMatB.cols*lMatB.elemSize();
    int32_t lHeightInLinesB = lMatB.rows;
    int32_t lSpanB = aUMatB.step[0];

    if ((lWidthInBytesA == lWidthInBytesB) && (lHeightInLinesA == lHeightInLinesB))
    {
      for (int32_t y = 0; y < lHeightInLinesA; y++, lpDataA += lSpanA, lpDataB += lSpanB)
      {
        for (int32_t x = 0; x < lWidthInBytesA; x++)
        {
          if (lpDataA[x] != lpDataB[x])
          {
#if DO_DEBUG_PRINT
            printf("%d|%d(%d,%d) ",lpDataA[x], lpDataB[x], x, y);
            break;
#else
            lRetVal++;
#endif
            //break;
          }
        }

#if DO_DEBUG_PRINT
        if (lRetVal)
          break;
#endif
      }
    }
    else
    {
      lRetVal = -1;
    }

    return lRetVal;
  }

}
