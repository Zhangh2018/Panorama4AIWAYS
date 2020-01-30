/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
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

CGV_INLINE_PREFIX int TransferReq(int* lTransferArray, int  lTransferArraySize)
{
   int lRetVal = 0;
   for (int i=0; i<lTransferArraySize; i++)
   {
      if (lTransferArray[i] != -1)
      {
         lRetVal = 1;
         break;
      }
   }

   return lRetVal;
}

CGV_INLINE_PREFIX void ACF_FinalizeLL(int* lpLinkedListEntry, int lLastFlag, int lLLBufIndex)
{
   //go back and update ENTRY_2

   int* lpEntry = lpLinkedListEntry;

   lpEntry[2] &= 0x0FFFFFFF;

   if (0 == lLastFlag) //not the last transfer
   {
      if (lLLBufIndex == 0) lpEntry[2] |= 0xC0000000; //pause & interrupt enable
      else                  lpEntry[2] |= 0xF0000000; //pause & interrupt enable & loop on last & last
   }
   else //last transfer
   {
      lpEntry[2] |= 0xD0000000; //pause & interrupt enable & last
   }
}

CGV_INLINE_PREFIX void ACF_LleSetLast(int* lpLinkedListEntry, int lLastFlag)
{
   if (lLastFlag)
   {
      lpLinkedListEntry[2] |= 0x10000000;
   }
}

CGV_INLINE_PREFIX void ACF_LleSetLastMce(int* lpLinkedListEntry, int lLastFlag)
{
   if (lLastFlag)
   {
      lpLinkedListEntry[0] |= 0x10000000;
   }
}

/*
Regarding ACF_AppendLLE_XMEM_CMEM_IN, ACF_AppendLLE_XMEM_CMEM_OUT:

lTransferArray contains the iteration lIter of the transfer normalized to the start of the buffer in question
(i.e. if lIter=0 it indicates buffer 0.  if lIter=1 it indicates buffer 1.  if lIter=2 it indicates buffer 2
if there are at least 3 buffers or it indicates buffer 0 [wrap around] if there are only 2 buffers, etc.)

The index associated with the calculation of the tile address in physical memory (call it physTileIndex)
is a combination of the iteration and the var 'lIterAdjust';  physTileIndex = iteration + lIterAdjust.
This crops up dealing with output in the ROI case since lIter and physTileIndex become desynchronized.
In such a case the first lIter may be > 0, but the first physTileIndex must always be 0 so the tiles
go to the right place in external memory.
*/

CGV_INLINE_PREFIX void ACF_AppendLLE_XMEM_CMEM_IN(int* lpLinkedListBuf,
                                int* lTransferArray,
                                int  lTransferArraySize,
                                acf_llentry_desc* lpLleDb,
                                int lNumIterations,
                                int lLastTileWidthInChunks,
                                int lLlEntrySizeInWords)
{
   for (int i=0; i<lTransferArraySize; i++)
   {
      int lIter = lTransferArray[i];
      acf_llentry_desc* lpDbEntry = &lpLleDb[i];
      int* lpEntry = lpLinkedListBuf+i*lLlEntrySizeInWords;
      ACF_LocalMemDesc* lpLmDesc = lpDbEntry->pLmDesc;

      if (lIter != -1)
      {
         //if transfer size is zero, it means this transfer was previously configured as a
         //'dummy' transfer; restore necessary fields before continuing with regular setup
         if (lpEntry[3] == 0)
         {
            lpEntry[1] |= 0x80000000; //set bit 31 (enable stream)

            //restore 'transfer size' to correct value (tricky ... 'backed up' transfer
            //size value was stored in unused linked list entry word [6])
            lpEntry[3] = lpEntry[6];
         }

         lpEntry[0] = lpDbEntry->pIoDesc->physAddr + lIter*lpDbEntry->pIoDesc->tileOffset;

         //retrieve 'logical' cmem address for STREAM purposes
         int lCmemAddr = lpLmDesc->logicalVecByteAddrBuf0 + lpDbEntry->_framework_cbindex*lpLmDesc->bufferOffset;

         CbIndexIncrement(&lpDbEntry->_framework_cbindex, lpLmDesc->numBuffers); //must be incremented in all cases
         int lStartCu = lpEntry[8] & 0x7F; //keep preconfigured value for lStartCu
         lpEntry[8] = (lCmemAddr>>1) << 20 | (lCmemAddr>>1) << 8  | lStartCu << 0;
         
         //if lLastTileWidthInChunks is -1, it means no adjustment is required (i.e. tile width is unchanged from previous widths)
         if (lIter == (lNumIterations-1) && (lLastTileWidthInChunks != -1)) //this is the last tile for this IO
         {
            //in the 1D case the tile width (in chunks) of the last tile may be different
            int lChunkWidthInBytes = lpEntry[17] + 1;
            int lBytesPerLine      = lChunkWidthInBytes*lLastTileWidthInChunks;
            int lNumLines          = ((lpEntry[9]>>6) & 0x3FF) + 1;
            int lWordSize          = CalcOptimalWordSize(lpEntry[0], (lpEntry[4] >> 16), lBytesPerLine); //addr, span, bytes per line

            //update dma word size
            lpEntry[2] &= 0xFFFFF8FF;
            lpEntry[2] |= (lWordSize << 8);
            //update dma transfer size
            lpEntry[3] = lBytesPerLine*lNumLines;
            //update dma tile width
            lpEntry[5] = lBytesPerLine;
            //update stream bytes per line
            lpEntry[9] &= 0x0000FFFF;
            lpEntry[9] |= (lBytesPerLine << 16);
#if 0
            //adjust height of last tile transfer if height is not nicely divisible by chunk height
            int lHeightModChunkHeight = lpDbEntry->pIoDesc->height % lpLmDesc->chunkHeight;
            if (lHeightModChunkHeight > 0)
            {
               lNumLines = lHeightModChunkHeight * lpLmDesc->e0.y;
            }

            lpEntry[3] = lBytesPerLine*lNumLines;
            lpEntry[5] = 0 << 16 | lBytesPerLine << 0;
            lpEntry[9] &= 0x0000003F;
            lpEntry[9] |= (lBytesPerLine << 16);
            lpEntry[9] |= ((lNumLines-1) << 6);
#endif
         }
      }
      else if (lpEntry[3] != 0) //make entry a 'dummy' transfer (if it wasn't already done in a prev iter)
      {
         lpEntry[1] &= 0x7FFFFFFF; //clear bit 31 (disable stream)

         //set transfer size to 0 (tricky ... need to set transfer size to 0
         //but may need to restore it later so 'save it' in unused lpEntry[6]
         lpEntry[6] = lpEntry[3];
         lpEntry[3] = 0;
      }
   }
}

CGV_INLINE_PREFIX void ACF_AppendLLE_XMEM_CMEM_OUT(int* lpLinkedListBuf,
                                 int* lTransferArray,
                                 int  lTransferArraySize,
                                 acf_llentry_desc* lpLleDb,
                                 int lIterAdjust,
                                 int lNumIterations,
                                 int lLastTileWidthInChunks,
                                 int lLlEntrySizeInWords)
{
   for (int i=0; i<lTransferArraySize; i++)
   {
      int lIter = lTransferArray[i];
      acf_llentry_desc* lpDbEntry = &lpLleDb[i];
      int* lpEntry = lpLinkedListBuf+i*lLlEntrySizeInWords;
      ACF_LocalMemDesc* lpLmDesc = lpDbEntry->pLmDesc;

      if (lIter != -1)
      {
         //if transfer size is zero, it means this transfer was previously configured as a
         //'dummy' transfer; restore necessary fields before continuing with regular setup
         if (lpEntry[3] == 0)
         {
            lpEntry[0] |= 0x80000000; //set bit 31 (enable stream)

            //restore 'transfer size' to correct value (tricky ... 'backed up' transfer
            //size value was stored in unused linked list entry word [6])
            lpEntry[3] = lpEntry[6];
         }

         lpEntry[1] = lpDbEntry->pIoDesc->physAddr + (lIter+lIterAdjust)*lpDbEntry->pIoDesc->tileOffset;

         //retrieve 'logical' cmem address for STREAM purposes
         int lCmemAddr = lpLmDesc->logicalVecByteAddrBuf0 + lpDbEntry->_framework_cbindex*lpLmDesc->bufferOffset;

         CbIndexIncrement(&lpDbEntry->_framework_cbindex, lpLmDesc->numBuffers); //must be incremented in all cases
         int lStartCu = lpEntry[8] & 0x7F; //keep preconfigured value for lStartCu
         lpEntry[8] = (lCmemAddr>>1) << 20 | (lCmemAddr>>1) << 8  | lStartCu << 0;

         //if lLastTileWidthInChunks is -1, it means no adjustment is required (i.e. tile width is unchanged from previous widths)
         if (lIter == (lNumIterations-1) && (lLastTileWidthInChunks != -1)) //this is the last tile for this IO
         {
            //in the 1D case the tile width (in chunks) of the last tile may be different
            int lChunkWidthInBytes = lpEntry[17] + 1;
            int lBytesPerLine      = lChunkWidthInBytes*lLastTileWidthInChunks;
            int lNumLines          = ((lpEntry[9]>>6) & 0x3FF) + 1;
            int lWordSize          = CalcOptimalWordSize(lpEntry[1], (lpEntry[4]), lBytesPerLine); //addr, span, bytes per line

            //update dma word size
            lpEntry[2] &= 0xFFFFF8FF;
            lpEntry[2] |= (lWordSize << 8);
            //update dma transfer size
            lpEntry[3] = lBytesPerLine*lNumLines;
            //update dma tile width
            lpEntry[5] = lBytesPerLine;
            //update stream bytes per line
            lpEntry[9] &= 0x0000FFFF;
            lpEntry[9] |= (lBytesPerLine << 16);
#if 0
            //adjust height of last tile transfer if height is not nicely divisible by chunk height
            int lHeightModChunkHeight = lpDbEntry->pIoDesc->height % lpLmDesc->chunkHeight;
            if (lHeightModChunkHeight > 0)
            {
               lNumLines = lHeightModChunkHeight * lpLmDesc->e0.y;
            }

            lpEntry[3] = lBytesPerLine*lNumLines;
            lpEntry[5] = 0 << 16 | lBytesPerLine << 0;
            lpEntry[9] &= 0x0000003F;
            lpEntry[9] |= (lBytesPerLine << 16);
            lpEntry[9] |= ((lNumLines-1) << 6);
#endif
         }
      }
      else if (lpEntry[3] != 0) //make entry a 'dummy' transfer (if it wasn't already done in a prev iter)
      {
         lpEntry[0] &= 0x7FFFFFFF; //clear bit 31 (disable stream)

         //set transfer size to 0 (tricky ... need to set transfer size to 0
         //but may need to restore it later so 'save it' in unused lpEntry[6]
         lpEntry[6] = lpEntry[3];
         lpEntry[3] = 0;
      }
   }
}

//local implementaitons of a few sequencer hal calls for improved speed and inlining
CGV_INLINE_PREFIX void ACF_seq_hal_proc_inc_rscmask(int lProcId, int lRscMask)
{
#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
   gspSeqRegisters->ProcInc[lProcId].as32BitValue = lRscMask;
#else
   seq_hal_proc_inc_rscmask(gsApexId, lProcId, lRscMask);
#endif
}

CGV_INLINE_PREFIX void ACF_seq_hal_proc_dec_rscmask(int lProcId, int lRscMask)
{
   gspSeqRegisters->ProcDec[lProcId].as32BitValue = lRscMask;
}

CGV_INLINE_PREFIX int ACF_seq_hal_ret_rsc_counter(int lRsc)
{
   volatile REG32* lCntArray = (volatile REG32*)&(gspSeqRegisters->ResourceFix[0]);
   return lCntArray[lRsc];
}

CGV_INLINE_PREFIX void ACF_seq_hal_clear_interrupts(int lMask)
{
#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
   gspSeqRegisters->Icr.as32BitValue = lMask;
#else
   seq_hal_clear_interrupts(gsApexId, lMask);
#endif
}

CGV_INLINE_PREFIX void ACF_seq_hal_clear_interrupt(int lProcId)
{
#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
   gspSeqRegisters->Icr.as32BitValue = (1<<lProcId);
#else
   seq_hal_clear_interrupt(gsApexId, lProcId);
#endif
}

CGV_INLINE_PREFIX int ACF_seq_hal_query_interrupts()
{
   return gspSeqRegisters->Isr.as32BitValue;
}
