/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2017 NXP
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

#include <apexcv_pro_util.h>
#include <oal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmem_if_hal.h>
#include <global_hal.h>

namespace apexcv{

   int32_t RetNumCU(int aApexID, int32_t& aArrayWidth)
   {
      int32_t lRetVal = 0;
      int32_t lHwVersion = global_hal_RetHwVersion(aApexID);
      switch (lHwVersion)
      {
      case 0x321:
         aArrayWidth = 32;
         cmem_if_hal_set_apu_cfg(aApexID, CFG_321__APU_0_CU_0_31_SMEM_0_1);
         break;
      case 0x642:
         cmem_if_hal_set_apu_cfg(aApexID, CFG_642__APU_0_CU_0_63_SMEM_0_3);
         aArrayWidth = 64;
         break;
      default:
         printf("QueryApuArraySize -> unknown APU configuration detected (%d)", lHwVersion);
         lRetVal = 1;
      }

      return lRetVal;
   }


   /********************************
   **
   **	MEMORY OPERATIONS (copy, compare)
   **
   ********************************/

   /***** MEMORY BLOCK FUNCTIONS ******************************************************/

   inline void copyBlock_inline(char* apDst, size_t aDstSpan,
      const char* apcSrc, size_t aSrcSpan,
      size_t aWidth, int aHeight)
   {
      // Copy a block of width <width> bytes and height <height> from a 2D source
      // memory with span <srcSpan> to a 2D destination memory with span <dstSpan>
      for (char* dEnd = apDst + (aHeight*aDstSpan); apDst < dEnd; apDst += aDstSpan, apcSrc += aSrcSpan)
      {
         memcpy(apDst, apcSrc, aWidth);
      }
   }

   void copyBlock(void* apDst, size_t aDstSpan,
      const void* apcSrc, size_t aSrcSpan,
      size_t aWidth, int aHeight)
   {
      copyBlock_inline((char*)apDst, aDstSpan,
         (const char*)apcSrc, aSrcSpan,
         aWidth, aHeight);
   }

   int compBlock_inline(const char* apcBuf1, size_t aSpan1,
      const char* apcBuf2, size_t aSpan2,
      size_t aWidth, int aHeight)
   {
      // Compare a block of memory from two buffers
      int res = 0;
      for (const char* bufEnd1 = apcBuf1 + (aHeight*aSpan1); apcBuf1 < bufEnd1; apcBuf1 += aSpan1, apcBuf2 += aSpan2)
      {
         res = memcmp(apcBuf1, apcBuf2, aWidth);  // compare this row
         if (res) break;                   // break if a difference is found in the row
      }
      return res;
   }

   int compBlock(const void* apcBuf1, size_t aSpan1,
      const void* apcBuf2, size_t aSpan2,
      size_t aWidth, int aHeight)
   {
      // Compare a block of memory from two buffers
      return compBlock_inline((const char*)apcBuf1, aSpan1,
         (const char*)apcBuf2, aSpan2,
         aWidth, aHeight);
   }

   void* setBlock(void * apDst, size_t aSpan,
      int aValue,
      size_t aWidth, int aHeight)
   {
      char* d = (char*)apDst;
      for (int y = 0; y < aHeight; ++y, d += aSpan)
      {
         memset(d, aValue, aWidth);
      }

      return apDst;
   }

   /***** MEMORY LATTICE FUNCTIONS ******************************************************/

   void copyLattice(void* apDst         /*!< Destination pointer. */,
      size_t aDstSpan    /*!< Destination span. */,
      size_t aDstStepX   /*!< Number of bytes between horizontal lattice points in the destination buffer. */,
      size_t aDstStepY   /*!< Number of bytes between vertical   lattice points in the destination buffer. */,

      const void* apcSrc   /*!< Source pointer. */,
      size_t aSrcSpan    /*!< Source span. */,
      size_t aSrcStepX   /*!< Number of bytes between horizontal lattice points in the source      buffer. */,
      size_t aSrcStepY   /*!< Number of bytes between vertical   lattice points in the source      buffer. */,

      size_t aWidth      /*!< Block width in bytes. */,
      int aHeight        /*!< Block height. */,

      int aNx            /*!< Number of horizontal lattice points. */,
      int aNy            /*!< Number of vertical   lattice points. */)
   {
      for (int y = 0; y < aNy; ++y)   // For each lattice row
      {
         // Get destination and source address of lattice point (0, y)
         char* d = (char*)apDst + y * aDstStepY;
         const char* s = (const char*)apcSrc + y * aSrcStepY;

         for (int x = 0; x < aNx; ++x)   // For each lattice column
         {
            // Copy the block of width <width> and height <height>
            copyBlock_inline(d + x*aDstStepX, aDstSpan,
               s + x*aSrcStepX, aSrcSpan,
               aWidth, aHeight);
         }
      }
   }

   void compLattice(const void* apcBuf1    /*!< Address of buffer 1. */,
      size_t aSpan1        /*!< Span of buffer 1. */,
      size_t aStepX1       /*!< Number of bytes between horizontal lattice points in buffer 1. */,
      size_t aStepY1       /*!< Number of bytes between vertical   lattice points in buffer 1. */,

      const void* apcBuf2    /*!< Address of buffer 2. */,
      size_t aSpan2        /*!< Span of buffer 2. */,
      size_t aStepX2       /*!< Number of bytes between horizontal lattice points in buffer 2. */,
      size_t aStepY2       /*!< Number of bytes between vertical   lattice points in buffer 2. */,

      size_t aWidth        /*!< Block width in bytes. */,
      int aHeight          /*!< Block height. */,

      int aNx              /*!< Number of horizontal lattice points. */,
      int aNy              /*!< Number of vertical   lattice points. */)
   {
      for (int y = 0; y < aNy; ++y)   // For each lattice row
      {
         // Get destination and source address of lattice point (0, y)
         const char* b1 = (const char*)apcBuf1 + y * aStepY1;
         const char* b2 = (const char*)apcBuf2 + y * aStepY2;

         for (int x = 0; x < aNx; ++x)   // For each lattice column
         {
            // Compare the block of width <width> and height <height>
            compBlock_inline(b1 + x*aStepX1, aSpan1,
               b2 + x*aStepX2, aSpan2,
               aWidth, aHeight);
         }
      }
   }


   /***** PADDING FUNCTIONS *************************************************************/

   void padSet(void *apDst      /*!< Address of destination buffer. */,
      size_t aSpan    /*!< Span of destination buffer. */,
      int aValue      /*!< Value assigned to each padded byte. */,
      size_t aWidth   /*!< Unpadded width in bytes. */,
      int aHeight     /*!< Unpadded height (rows). */,
      size_t aLeft    /*!< Number of bytes padded on the left. */,
      size_t aRight   /*!< Number of bytes padded on the right. */,
      int aTop        /*!< Number of rows padded on the top. */,
      int aBottom     /*!< Number of rows padded on the bottom. */)
   {
      char* d = (char*)apDst;
      const size_t paddedWidth = aLeft + aWidth + aRight;

      // Pad top
      setBlock(d, aSpan, aValue, paddedWidth, aTop);

      // Pad left
      d += aTop * aSpan;
      setBlock(d, aSpan, aValue, aLeft, aHeight);

      // Pad right
      setBlock(d + aLeft + aWidth, aSpan, aValue, aRight, aHeight);

      // Pad bottom
      d += aHeight * aSpan;
      setBlock(d, aSpan, aValue, paddedWidth, aBottom);
   }

   void padSet(void* apDst         /*!< Address of destination buffer. */,
      size_t aDstSpan    /*!< Span of destination buffer. */,
      const void* apcSrc   /*!< Address of source buffer. */,
      size_t aSrcSpan    /*!< Span of source buffer. */,
      int aValue         /*!< Value assigned to each padded byte. */,
      size_t aWidth      /*!< Unpadded width in bytes. */,
      int aHeight        /*!< Unpadded height (rows). */,
      size_t aLeft       /*!< Number of bytes padded on the left. */,
      size_t aRight      /*!< Number of bytes padded on the right. */,
      int aTop           /*!< Number of rows padded on the top. */,
      int aBottom        /*!< Number of rows padded on the bottom. */)
   {
      // Copy source data
      char* d = (char*)apDst;
      copyBlock(d + aTop*aDstSpan + aLeft, aDstSpan,
         apcSrc, aSrcSpan,
         aWidth, aHeight);

      // Pad the destination with <value>
      padSet(apDst, aDstSpan, aValue, aWidth, aHeight,
         aLeft, aRight, aTop, aBottom);
   }

   void padReplicate(void* apBuf, size_t aSpan,
      size_t aWidth, int aHeight,
      size_t aLeftWidth, int aLeftCols,
      size_t aRightWidth, int aRightCols,
      int aTopHeight, int aTopRows,
      int aBottomHeight, int aBottomRows)
   {
      const size_t leftSize = aLeftCols * aLeftWidth;
      const size_t lastCol = aWidth - aRightWidth;
      const size_t topSpan = aTopHeight*aSpan;
      const size_t bottomSpan = aBottomHeight*aSpan;

      char* d = (char*)apBuf;
      const char* s = d + aTopRows*topSpan + leftSize;

      // Pad-replicate top left
      copyLattice(d, aSpan, aLeftWidth, topSpan,
         s, aSpan, 0, 0,
         aLeftWidth, aTopHeight,
         aLeftCols, aTopRows);

      // Pad-replicate top
      copyLattice(d + leftSize, aSpan, 0, topSpan,
         s, aSpan, 0, 0,
         aWidth, aTopHeight,
         1, aTopRows);

      // Pad-replicate top right
      copyLattice(d + leftSize + aWidth, aSpan, aRightWidth, topSpan,
         s + lastCol, aSpan, 0, 0,
         aRightWidth, aTopHeight,
         aRightCols, aTopRows);

      // Pad-replicate left
      d += aTopRows * topSpan;
      copyLattice(d, aSpan, aLeftWidth, 0,
         s, aSpan, 0, 0,
         aLeftWidth, aHeight,
         aLeftCols, 1);

      // Pad-replicate right
      copyLattice(d + leftSize + aWidth, aSpan, aRightWidth, 0,
         s + lastCol, aSpan, 0, 0,
         aRightWidth, aHeight,
         aRightCols, 1);

      // Pad-replicate bottom left
      d += aSpan * aHeight;
      s += aSpan * (aHeight - aBottomHeight);
      copyLattice(d, aSpan, aLeftWidth, bottomSpan,
         s, aSpan, 0, 0,
         aLeftWidth, aBottomHeight,
         aLeftCols, aBottomRows);

      // Pad-replicate bottom
      copyLattice(d + leftSize, aSpan, 0, bottomSpan,
         s, aSpan, 0, 0,
         aWidth, aBottomHeight,
         1, aBottomRows);

      // Pad-replicate bottom right
      copyLattice(d + leftSize + aWidth, aSpan, aRightWidth, bottomSpan,
         s + lastCol, aSpan, 0, 0,
         aRightWidth, aBottomHeight,
         aRightCols, aBottomRows);
   }

   void padReplicate(void* apDst, size_t aDstSpan,
      const void* apcSrc, size_t aSrcSpan,
      size_t aWidth, int aHeight,
      size_t aLeftWidth, int aLeftCols,
      size_t aRightWidth, int aRightCols,
      int aTopHeight, int aTopRows,
      int aBottomHeight, int aBottomRows)
   {
      // Copy source data
      copyBlock((char*)apDst + aTopRows*aTopHeight*aDstSpan + aLeftWidth*aLeftCols, aDstSpan,
         apcSrc, aSrcSpan,
         aWidth, aHeight);

      // Pad by replication
      padReplicate(apDst, aDstSpan,
         aWidth, aHeight,
         aLeftWidth, aLeftCols,
         aRightWidth, aRightCols,
         aTopHeight, aTopRows,
         aBottomHeight, aBottomRows);
   }

   // Compare \ref data with another.
   template <typename D>
   int compare_results(D* aTest, D* aRef, int aW, int aH, int aRange)
   {
      int lRetVal = 0;

      for (int y = 0; y < aH; ++y) {
         for (int x = 0; x < aW; ++x, ++aTest, ++aRef) {
            if (abs((long)(*(aTest)-*(aRef))) > aRange) {
               //printf("%d: (test): %d != (ref): %d\n", y*w + x, *(test), *(ref));
               lRetVal |= 1;
            }
         }
      }

      return lRetVal;
   }

   template int compare_results(uint8_t*  aTest, uint8_t*  aRef, int aW, int aH, int aRange);
   template int compare_results(uint16_t* aTest, uint16_t* aRef, int aW, int aH, int aRange);
   template int compare_results(uint32_t* aTest, uint32_t* aRef, int aW, int aH, int aRange);
   template int compare_results(int8_t*   aTest, int8_t*   aRef, int aW, int aH, int aRange);
   template int compare_results(int16_t*  aTest, int16_t*  aRef, int aW, int aH, int aRange);
   template int compare_results(int32_t*  aTest, int32_t*  aRef, int aW, int aH, int aRange);

} /* namespace apex */
