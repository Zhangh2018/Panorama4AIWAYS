/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
* All Rights Reserved
*
******************************************************************************
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

/*!*********************************************************************************
*  \file
*  \brief APEX-CV Utility functions
***********************************************************************************/

#ifndef APEXCVUTIL_H
#define APEXCVUTIL_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>


namespace apexcv{

   int32_t RetNumCU(int apexID, int32_t& lArrayWidth);
   /********************************
   **
   **	MEMORY OPERATIONS (copy, compare)
   **
   ********************************/

   /*!*********************************************************************************
   *  \brief 2D extension of memcpy.
   *
   *  The standard C function memcpy copies a contiguous string of memory from one
   *  location to another.  The 2D extension of this is copyBlock, which
   *  copies a \ref pagBlock "2D memory block" from one location to another.
   *
   *  Consider a 2D memory source with location \c src and span \c srcSpan, and a 2D
   *  memory destination with location \c dst and span \c dstSpan.  Beginning at address
   *  \c src is a block of \c width (bytes) by \c height rows.  These are shown in
   *  the following figure.
   *
   *  \image html  copyBlock.svg "copyBlock copies a contiguous area of memory from one location to another."
   *  \image latex copyBlock.pdf "copyBlock copies a contiguous area of memory from one location to another." width = 8cm
   *
   *  Since each row is contiguous in memory, we use memcpy to copy each source row to
   *  its corresponding destination row.
   ***********************************************************************************/
   void copyBlock(void* apDst         /*!< Destination pointer. */,
      size_t aDstSpan    /*!< Destination span. */,
      const void* apcSrc   /*!< Source pointer. */,
      size_t aSrcSpan    /*!< Source span. */,
      size_t aWidth      /*!< Number of bytes to copy per row. */,
      int aHeight        /*!< Number of rows to copy. */);

   /*!*********************************************************************************
   *  \brief 2D extension of memcmp.
   *
   *  The standard C function memcmp compares two contiguous strings of memory.
   *  The 2D extension of this is compBlock, which compares two
   *  \ref pagBlock "2D memory blocks".
   *
   *  Consider two 2D memory buffers with addresses \c buf1 and \c buf2 and span
   *  \c span1 and \c span2, respectively.  Each buffer contains a memory block of
   *  \c width (bytes) by \c height rows.These are shown in the following figure.
   *
   *  \image html  compBlock.svg "compBlock compares two blocks of memory."
   *  \image latex compBlock.pdf "compBlock compares two blocks of memory." width = 8cm
   *
   *  Since each row is contiguous in memory, we use memcmp to compare corresponding
   *  rows in each buffer.
   ***********************************************************************************/
   int compBlock(const void* apcBuf1   /*!< Address of buffer 1. */,
      size_t aSpan1       /*!< Span of buffer 1. */,
      const void* apcBuf2   /*!< Address of buffer 2. */,
      size_t aSpan2       /*!< Span of buffer 2. */,
      size_t aWidth       /*!< Number of bytes to compare per row. */,
      int aHeight         /*!< Number of rows to compare. */);

   /*!*********************************************************************************
   *  \brief Set each byte of a \ref pagBlock "2D memory block" to a certain value.
   *
   *  The standard C function memset sets each byte of a string to a certain value.
   *  The 2D extension of this is blockSet.
   ***********************************************************************************/
   void* setBlock(void * apDst     /*!< Destination pointer. */,
      size_t aSpan    /*!< Destination span. */,
      int aValue      /*!< Value (cast to unsigned 8-bit) to fill the block. */,
      size_t aWidth   /*!< Number of bytes to set per row. */,
      int aHeight     /*!< Number of rows to set. */);

   /*!*********************************************************************************
   *  \brief Copy a \ref pagLattice "2D memory lattice" to another.
   ***********************************************************************************/
   void copyLattice(void* apDst         /*!< Destination pointer. */,
      size_t aDstSpan    /*!< Destination span. */,
      size_t aDstStepX   /*!< Number of bytes between horizontal lattice points in the destination buffer. */,
      size_t aDstStepY   /*!< Number of bytes between vertical   lattice points in the destination buffer. */,

      const void* apcSrc   /*!< Source pointer. */,
      size_t aSrcSpan    /*!< Source span. */,
      size_t aSrcStepX   /*!< Number of bytes between horizontal lattice points in the source buffer. */,
      size_t aSrcStepY   /*!< Number of bytes between vertical   lattice points in the source buffer. */,

      size_t aWidth      /*!< Block width in bytes. */,
      int aHeight        /*!< Block height. */,

      int aNx            /*!< Number of horizontal lattice points. */,
      int aNy            /*!< Number of vertical   lattice points. */);

   /*!*********************************************************************************
   *  \brief Compare a \ref pagLattice "2D memory lattice" with another.
   ***********************************************************************************/
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
      int aNy              /*!< Number of vertical   lattice points. */);

   /*!*********************************************************************************
   *  \brief Pad a \ref pagBlock "2D memory block" with a single value.
   ***********************************************************************************/
   void padSet(void *apDst      /*!< Address of destination buffer. */,
      size_t aSpan    /*!< Span of destination buffer. */,
      int aValue      /*!< Value assigned to each padded byte. */,
      size_t aWidth   /*!< Unpadded width in bytes. */,
      int aHeight     /*!< Unpadded height (rows). */,
      size_t aLeft    /*!< Number of bytes padded on the left. */,
      size_t aRight   /*!< Number of bytes padded on the right. */,
      int aTop        /*!< Number of rows padded on the top. */,
      int aBottom     /*!< Number of rows padded on the bottom. */);

   /*!*********************************************************************************
   *  \brief Copy a \ref pagBlock "2D memory block" and pad with a single value.
   ***********************************************************************************/
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
      int aBottom        /*!< Number of rows padded on the bottom. */);

   /*!*********************************************************************************
   *  \brief Pad a \ref pagBlock "2D memory block" by replication.
   ***********************************************************************************/
   void padReplicate(void* apBuf         /*!< Address of destination buffer. */,
      size_t aSpan        /*!< Span of destination buffer. */,
      size_t aWidth       /*!< Unpadded width in bytes. */,
      int aHeight         /*!< Unpadded height (rows). */,
      size_t aleftWidth   /*!< Width in bytes of each padded column on the left. */,
      int aLeftCols       /*!< Number of padded columns on the left. */,
      size_t aRightWidth  /*!< Width in bytes of each padded column on the right. */,
      int aRightCols      /*!< Number of padded columns on the right. */,
      int aTopHeight      /*!< Height in of each padded row on the top. */,
      int aTopRows        /*!< Number of padded rows on the top. */,
      int aBottomHeight   /*!< Height in of each padded row on the bottom. */,
      int aBottomRows     /*!< Number of padded rows on the bottom. */);

   /*!*********************************************************************************
   *  \brief Copy a \ref pagBlock "2D memory block" and pad by replication.
   ***********************************************************************************/
   void padReplicate(void* apDst         /*!< Address of destination buffer. */,
      size_t aDstSpan    /*!< Span of destination buffer. */,
      const void* apcSrc   /*!< Address of source buffer. */,
      size_t aSrcSpan    /*!< Span of source buffer. */,
      size_t aWidth      /*!< Unpadded width in bytes. */,
      int aHeight        /*!< Unpadded height (rows). */,
      size_t aLeftWidth  /*!< Width in bytes of each padded column on the left. */,
      int aLeftCols      /*!< Number of padded columns on the left. */,
      size_t aRightWidth /*!< Width in bytes of each padded column on the right. */,
      int aRightCols     /*!< Number of padded columns on the right. */,
      int aTopHeight     /*!< Height in of each padded row on the top. */,
      int aTopRows       /*!< Number of padded rows on the top. */,
      int aBottomHeight  /*!< Height in of each padded row on the bottom. */,
      int aBottomRows    /*!< Number of padded rows on the bottom. */);

   /*!*********************************************************************************
   *  \brief Compare \ref data with another.
   ***********************************************************************************/
   template <typename D>
   int compare_results(D* apTest   /*!< Address of data 1. */,
      D* apRef   /*!< Address of data 2. */,
      int aW    /*!< Data width. */,
      int aH    /*!< Data height. */,
      int aRange = 0  /*!< Range of error. */);

}  /* namespace apex */

#endif /* APEXCVUTIL_H */
