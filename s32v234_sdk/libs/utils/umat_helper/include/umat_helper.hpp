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

#ifndef UMAT_HELPER_HPP
#define UMAT_HELPER_HPP

#include <sumat.hpp>
#include <icp_datatype.h>

namespace vsdk
{
  class UMatHelper
  {
  public:
  static vsdk::SUMat CreateAcfFriendlyUMat(int32_t aRows, int32_t aCols, int32_t aType);
  static vsdk::SUMat CreateAcfFriendlyUMat(int32_t aRows, int32_t aCols, int32_t aType,
                                   int32_t aPadL, int32_t aPadR, int32_t aPadT, int32_t aPadB);
  static void FillZero(vsdk::SUMat& aUMat);
  static void FillRandom(vsdk::SUMat& aUMat, uint32_t* apSeedVal);
  static void FillValue(vsdk::SUMat& aUMat, uint32_t* apVal);
  static void FillValue(vsdk::SUMat& aUMat, uint8_t aVal);
  static int32_t Compare(vsdk::SUMat& aUMatA, vsdk::SUMat& aUMatB);
  private:
    static void CalcBufferInfo(int32_t lWidth,
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
      int32_t& lBufferCoreDataOffsetYInElem);

  };
}
#endif //UMAT_HELPER_HPP
