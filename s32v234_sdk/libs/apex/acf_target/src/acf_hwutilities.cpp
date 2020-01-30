/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#include <acf_memmgmt.h>
#include <acf_common.h>
#include <seq_hal.h>
#include <mcdma_hal.h>
#include <mce_hal.h>
#include <stio_hal.h>
#include <linked_list_utilities.h>
#include <stdint.h>
#include <hw_reg_user.h>
#include <util.h>
#include <acf_utilities.h>
#include <acf_hwutilities.h>

int gsDmemBaseAddrForDma = 0; //initialized by the APU program ...
volatile APEX_642CL_SEQ_DCR* gspSeqRegisters = 0; //for use by ACF 'optimized' versions of sequencer hal calls
int gsApexId = 0;

int CalcTransferWidth(e0_desc &e0, int width, int chunkWidth, int tileWidthInChunks)
{
   int transferWidth = tileWidthInChunks*chunkWidth; //default value

   //Now, decide if 'width' should be used instead of 'tileWidthInChunks*chunkWidth'
   //Calculating two candidates for 'bytes per line':
   //1) bytesPerLine_A = assuming bytes per line matches width exactly
   //2) bytesPerLine_B = assuming bytes per line is rounded up to chunk width resolution
   //These will differ when dealing with the partial CU case (i.e. chunk width doesn't divide evenly into width).
   //When this happens, A or B will be selected based on DMA friendliness.  If equally friendly, A will be selected.
   //NOTE: in the true 1D case, it is possible that (width > tileWidthInChunks*chunkWidth).  If this is detected,
   //      the default transferWidth (i.e. tileWidthInChunks*chunkWidth) must be used!
   int elementWidthInBytes = RetDataTypeSizeInBytes(e0.dataType)*e0.x;
   int bytesPerLine_A = width*elementWidthInBytes;
   int bytesPerLine_B = tileWidthInChunks*chunkWidth*elementWidthInBytes;

   if ((CalcOptimalWordSize(0,0,bytesPerLine_A) >= CalcOptimalWordSize(0,0,bytesPerLine_B)) &&
       (width <= tileWidthInChunks*chunkWidth))
   {
      transferWidth = width;
   }

   return transferWidth;
}

/*
In order to configure a 1D transfer, call with chunkHeight = 1 and set chunkWidth = to the full size of the chunk!
In the 1D case the 'span' and 'chunkSpan' parameters will have no meaning and will not be used.
*/

void ACF_LLE_2DTransferConfig_XMEM_CMEM (int* llBuffer,
                                         e0_desc &e0,
                                         int srcAddr,
                                         int dstAddr,
                                         int span,
                                         int width,
                                         int tileWidthInChunks,
                                         int chunkWidth,
                                         int chunkHeight,
                                         int chunkSpan,
                                         int cmemAddr,
                                         int startCu)
{
   LLE_2DDMATransferConfig_XMEM_CMEM ((uint32_t*)llBuffer,
                                      e0.dataType,
                                      e0.x,
                                      e0.y,
                                      srcAddr,
                                      dstAddr,
                                      span,
                                      CalcTransferWidth(e0, width, chunkWidth, tileWidthInChunks),
                                      chunkWidth,
                                      chunkHeight,
                                      chunkSpan,
                                      cmemAddr,
                                      startCu);
}

void ACF_LLE_2DTransferConfig_MEM_MEM (int* llBuffer,
                                       e0_desc &e0,
                                       int srcAddr,
                                       int dstAddr,
                                       int srcSpan,
                                       int dstSpan,
                                       int chunkWidth,
                                       int chunkHeight)
{
   LLE_2DDMATransferConfig_MEM_MEM ((uint32_t*)llBuffer,
                                    e0.dataType,
                                    e0.x,
                                    e0.y,
                                    srcAddr,
                                    dstAddr,
                                    srcSpan,
                                    dstSpan,
                                    chunkWidth,
                                    chunkHeight);
}

void ACF_AppendLLE_DMEM_XMEM(int* lpLinkedListBuf,
                             int* lTransferArray,
                             int  lTransferArraySize,
                             acf_llentry_desc* lpLleDb,
                             int lTransferType,
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
            //restore 'transfer size' to correct value (tricky ... 'backed up' transfer
            //size value was stored in unused linked list entry word [6])
            lpEntry[3] = lpEntry[6];
         }

         //if advanced transfer info is present it overrides preconfigured frame size, tile width, and offset
         int lOffsetInBytes = (lIter)*lpDbEntry->pIoDesc->tileOffset;
         if (lpDbEntry->pAdvTransferInfo != 0)
         {
            int lTransferSizeInBytes = lpDbEntry->pAdvTransferInfo->lTransferSizeInBytes;
            lpEntry[3] = lTransferSizeInBytes; //frame size
            lpEntry[5] = lTransferSizeInBytes; //tile width
            lOffsetInBytes = lpDbEntry->pAdvTransferInfo->lTotalBytesTransferred;

            //update the total bytes transferred so we don't lose our place
            lpDbEntry->pAdvTransferInfo->lTotalBytesTransferred += lTransferSizeInBytes;
         }

         if (lTransferType == INPUT_TRANSFER)
         {
            lpEntry[0] = lpDbEntry->pIoDesc->physAddr + lOffsetInBytes;
            lpEntry[1] = gsDmemBaseAddrForDma + (intptr_t)lpLmDesc->pBuf0 + lpDbEntry->_framework_cbindex*lpLmDesc->bufferOffset;
         }
         else //OUTPUT_TRANSFER
         {
            lpEntry[0] = gsDmemBaseAddrForDma + (intptr_t)lpLmDesc->pBuf0 + lpDbEntry->_framework_cbindex*lpLmDesc->bufferOffset;
            lpEntry[1] = lpDbEntry->pIoDesc->physAddr + lOffsetInBytes;
         }
         CbIndexIncrement(&lpDbEntry->_framework_cbindex, lpLmDesc->numBuffers); //must be incremented in all cases

         //in the FIFO output case, need to adjust 'word size' every iteration since the transfer size is changing
         if (lpDbEntry->pAdvTransferInfo != 0)
         {
            int32_t lWordSize = CalcOptimalWordSize(lpEntry[0]|lpEntry[1], lpEntry[3], lpEntry[3]);
            lpEntry[2] &= 0xFFFFF8FF;
            lpEntry[2] |= (lWordSize<<8);
         }

#if 0
         //need to add 'lNumIterations' var to the fn signature to use this
         if (lIter == (lNumIterations-1)) //this is the last tile for this IO
         {
            //adjust height of last tile transfer if height is not nicely divisible by chunk height
            int lNumLines = lpLmDesc->chunkHeight * lpLmDesc->e0.y;
            int lHeightModChunkHeight = lpDbEntry->pIoDesc->height % lpLmDesc->chunkHeight;
            if (lHeightModChunkHeight > 0)
            {
               lNumLines = lHeightModChunkHeight * lpLmDesc->e0.y;
            }
            int lBytesPerLine = lpLmDesc->chunkWidth*RetDataTypeSizeInBytes(lpLmDesc->e0.dataType)*lpLmDesc->e0.x;
            lpEntry[3] = lBytesPerLine*lNumLines;
         }
#endif
      }
      else if (lpEntry[3] != 0) //make entry a 'dummy' transfer (if it wasn't already done in a prev iter)
      {
         //set transfer size to 0 (tricky ... need to set transfer size to 0
         //but may need to restore it later so 'save it' in unused lpEntry[6]
         lpEntry[6] = lpEntry[3];
         lpEntry[3] = 0;
      }
   }
}

void ACF_LLE_MCETransferConfig (int* llBuffer,
                                e0_desc &e0,
                                int baseAddr,
                                int ptrArrayAddr,
                                int dstAddr,
                                int srcDataSpan,
                                int tileWidthInChunks,
                                int chunkWidth,
                                int chunkHeight,
                                int chunkSpan,
                                int cmemAddr,
                                int startCu)
{
   LLE_MCETransferConfig ((uint32_t*)llBuffer,
                          e0.dataType,
                          e0.x,
                          e0.y,
                          baseAddr,
                          ptrArrayAddr,
                          dstAddr,
                          srcDataSpan,
                          tileWidthInChunks,
                          chunkWidth,
                          chunkHeight,
                          chunkSpan,
                          cmemAddr,
                          startCu, 0xFFFFFFFF, 0xFFFFFFFF);
}

DECLARE_BYTE_ALIGNED(static unsigned int, gsNullPtrArrayForDummyMceTransfer[4], 16) = {0x80000000, 0x80000000, 0x80000000, 0x80000000}; //force 128-bit alignment

void ACF_GenLL_MCE(int* lpLinkedListBuf,
                   int* lTransferArray,
                   int  lTransferArraySize,
                   acf_llentry_mce_desc* lpLleDb,
                   int lNumIterations,
                   int lLastTileWidthInChunks)
{
   int* lpEntry = 0;
   for (int i=0; i<lTransferArraySize; i++)
   {
      int lIter = lTransferArray[i];
      acf_llentry_mce_desc* lpDbEntry = &lpLleDb[i];
      lpEntry = lpLinkedListBuf+i*LLE_SIZE_MCE;
      ACF_LocalMemDesc* lpLmDesc = lpDbEntry->pLmDesc;
      
      if (lIter != -1)
      {
         //if the stream enable is 0, it means this transfer was previously configured as a 
         //'dummy' transfer; restore necessary fields before continuing with regular setup
         if ((lpEntry[4] & 0x80000000) == 0)
         {
            //restore number of GOCs (n-1) to proper value
            lpEntry[0] |= ((lpDbEntry->pIoDesc->offsetArrayWidth>>2)-1) << 1; 
            
            //ptr array address (lpEntry[2]) will be reinitialized below ...   

            //re-enable stream
            lpEntry[4] |= 0x80000000; //set bit 31    
         }                   
         
         lpEntry[2] = lpDbEntry->pIoDesc->physPtrArrayAddr + lIter*lpDbEntry->pIoDesc->tileOffset;


         //retrieve 'logical' cmem address for STREAM purposes
         int lCmemAddr = lpLmDesc->logicalVecByteAddrBuf0 + lpDbEntry->_framework_cbindex*lpLmDesc->bufferOffset;

         CbIndexIncrement(&lpDbEntry->_framework_cbindex, lpLmDesc->numBuffers); //must be incremented in all cases
         int lStartCu = lpEntry[8] & 0x7F; //keep preconfigured value for lStartCu
         lpEntry[8] = (lCmemAddr>>1) << 20 | (lCmemAddr>>1) << 8  | lStartCu << 0; //the DMA addresses CMEM in a 16-bit fashion, so divide 'lCmemAddr' by 2

         if (lIter == (lNumIterations-1)) //this is the last tile for this IO
         {
            //in the 1D case the tile width (in chunks) of the last tile may be different
            int lChunkWidthInBytes = lpEntry[17]+1;
            int lNumLines = ((lpEntry[0]>>11)&0xFFF)+1;
            int lBytesPerTile = lChunkWidthInBytes*lLastTileWidthInChunks*lNumLines;

            lpEntry[0] &= 0xFFFFF801;
            lpEntry[0] |= (((lLastTileWidthInChunks>>2)-1) << 1);
            lpEntry[9] &= 0x0000FFFF;
            lpEntry[9] |= (lBytesPerTile << 16);
         }
      }
      else //make entry a 'dummy' transfer
      {
         //set number of GOCs (n-1) to a single GOC (i.e. set it to 0)
         lpEntry[0] &= 0xFFFFF801;

         lpEntry[2] = (int)((unsigned int)(gsDmemBaseAddrForDma) | (uintptr_t)(&gsNullPtrArrayForDummyMceTransfer[0]));
         
         //disable stream
         lpEntry[4] &= 0x7FFFFFFF; //clear bit 31  
      }
   }
}

void ACF_XFER_DMEM_XMEM(int lApexId, acf_io_desc& lIoDesc, ACF_LocalMemDesc& lLmDesc, uint32_t lDmaCh, uint32_t lXMEMtoDMEM, uint32_t lClover389Flag)
{
   DECLARE_BYTE_ALIGNED(static uint32_t, lLinkedList[8], 16); //linked list addr must be 128-bit aligned

   uint32_t lSrcAddr = gsDmemBaseAddrForDma + (uintptr_t)lLmDesc.pBuf0;
   uint32_t lDstAddr = lIoDesc.physAddr;
   uint32_t lSrcSpan = lLmDesc.chunkSpan;
   uint32_t lDstSpan = lIoDesc.nd_span.height;
   if (lXMEMtoDMEM == 1) //if going to DMEM
   {
      lSrcAddr = lIoDesc.physAddr;
      lDstAddr = gsDmemBaseAddrForDma + (uintptr_t)lLmDesc.pBuf0;
      lSrcSpan = lIoDesc.nd_span.height;
      lDstSpan = lLmDesc.chunkSpan;
   }

   LLE_2DDMATransferConfig_MEM_MEM (lLinkedList,
                                    lLmDesc.e0.dataType,
                                    lLmDesc.e0.x,
                                    lLmDesc.e0.y,
                                    lSrcAddr,
                                    lDstAddr,
                                    lSrcSpan,
                                    lDstSpan,
                                    lLmDesc.chunkWidth,
                                    lLmDesc.chunkHeight);

   if (1 == lClover389Flag)
      LLE_DMA_SetBurstToINCR1(lLinkedList);
   LLE_DMA_Finalize(lLinkedList);

   mcdma_hal_program_seq_linkedlist(lApexId, lDmaCh, gsDmemBaseAddrForDma + (uintptr_t)lLinkedList);
   mcdma_hal_start(lApexId, lDmaCh);
   mcdma_hal_wait(lApexId, lDmaCh);
   mcdma_hal_interruptclear_ch_done(lApexId, lDmaCh);
}

void ACF_2DXFER_XMEM_CMEM(int lApexId,
                          acf_io_desc& lIoDesc,
                          ACF_LocalMemDesc& lLmDesc,
                          uint32_t lDmaCh,
                          uint32_t lStreamCh,
                          uint32_t lXMEMtoCMEM,
                          uint32_t lTileWidthInChunks,
                          uint32_t lStartCu)
{
   DECLARE_BYTE_ALIGNED(static uint32_t, lLinkedList[24], 16); //linked list addr must be 128-bit aligned

   uint32_t lSrcAddr = RetStreamOutPeriphId(lStreamCh);
   uint32_t lDstAddr = lIoDesc.physAddr;
   if (lXMEMtoCMEM == 1)
   {
      lSrcAddr = lIoDesc.physAddr;
      lDstAddr = RetStreamInPeriphId(lStreamCh);
   }

   uint32_t lCmemAddr = (uintptr_t)lLmDesc.pBuf0; //this is the 8-bit CMEM address

   LLE_2DDMATransferConfig_XMEM_CMEM (lLinkedList,
                                      lLmDesc.e0.dataType,
                                      lLmDesc.e0.x,
                                      lLmDesc.e0.y,
                                      lSrcAddr,
                                      lDstAddr,
                                      lIoDesc.nd_span.height,
                                      CalcTransferWidth(lLmDesc.e0, lIoDesc.nd_size.channel*lIoDesc.nd_size.width, lLmDesc.chunkWidth, lTileWidthInChunks),
                                      lLmDesc.chunkWidth,
                                      lLmDesc.chunkHeight,
                                      lLmDesc.chunkSpan,
                                      lCmemAddr,
                                      lStartCu);
   
   LLE_DMA_Finalize(lLinkedList);

   mcdma_hal_program_seq_linkedlist(lApexId, lDmaCh, gsDmemBaseAddrForDma + (uintptr_t)lLinkedList);
   mcdma_hal_start(lApexId, lDmaCh);
   mcdma_hal_wait(lApexId, lDmaCh);
   mcdma_hal_interruptclear_ch_done(lApexId, lDmaCh);
   if (lXMEMtoCMEM == 1) stio_hal_interruptclear_sti_done(lApexId, lStreamCh);
   else                  stio_hal_interruptclear_sto_done(lApexId, lStreamCh);
}

void SEQ_INCREMENT_RESOURCE(int lApexId, int lResource)
{
   seq_hal_proc_inc_rscmask(lApexId, 0, 1<<lResource);
}

void ACF_InitSeqRegPtr(int lApexId)
{
   gspSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(lApexId, HW_REG_USEQ);
   gsApexId = lApexId;
}

void Group(int8_t lNumIo,
           int8_t lNumHwPaths,
           int8_t* lpNumLogicalTransfers,
           struct transfer* lpLogicalTransfers)
{
   if (lNumIo > 0 && lNumHwPaths > 0)
   {
      *lpNumLogicalTransfers = lNumHwPaths;
      if (lNumHwPaths >= lNumIo)
      {
         *lpNumLogicalTransfers = lNumIo;
      }

      //group IOs into logical transfers (identify by index 0 to (NumIo-1))
      //(for now just try to keep the number of transfers per logical transfer as close as possible)
      int8_t lTmpQuo = lNumIo / (*lpNumLogicalTransfers);
      int8_t lTmpRem = lNumIo % (*lpNumLogicalTransfers);
      int8_t lIndex = 0;

      for (int i = 0; i<(*lpNumLogicalTransfers); i++)
      {
         int8_t lSize = lTmpQuo;
         if (i < lTmpRem)
            lSize = lTmpQuo + 1;

         lpLogicalTransfers[i].numIdx = lSize;
         for (int j=0; j<lSize; j++)
         {
            lpLogicalTransfers[i].idxArray[j] = lIndex;
            lIndex += 1;
         }
      }
   }
}

int ACF_OrgLogXfer_CmemOnly(int8_t lNumDmaCh,
                            int8_t lNumStreamCh,
                            int8_t lNumTransfers,
                            int8_t* lpNumLogicalTransfers,
                            struct transfer* lpLogicalTransfers)
{
   int lRetVal = 0;
   int8_t lNumHwPath = 0; //number of logical HW paths (a path consists of: DMA+STREAM)

   //number of valid HW pathways = whichever is smaller between lNumStreamCh and lNumDmaCh
   if (lNumDmaCh >= 1 && lNumStreamCh >= 1)
   {
      lNumHwPath = (lNumStreamCh > lNumDmaCh) ? lNumDmaCh : lNumStreamCh;
   }
   else //need at least one dma channel and one stream channel
   {
      lRetVal = 1;
   }

   //now determine number of logical transfers of each type and how to split up IOs into logical groups
   //NOTE: currently this decision is dumb; ideally it should group IOs into logical transfers such that
   //logical transfer sizes are equivalent(ish) to one another to maximize overlap between logical transfers
   Group (lNumTransfers, lNumHwPath, lpNumLogicalTransfers, lpLogicalTransfers);

   return lRetVal;
}

int ACF_OrgLogXfer_DmemOnly(int8_t lNumDmaCh,
                            int8_t lNumTransfers,
                            int8_t* lpNumLogicalTransfers,
                            struct transfer* lpLogicalTransfers)
{
   int lRetVal = 0;
   int8_t lNumHwPath = 0; //number of logical HW paths (a path consists of: DMA)

   //number of valid HW pathways = lNumDmaCh (stream not used)
   if (lNumDmaCh >= 1)
   {
      lNumHwPath = lNumDmaCh;
   }
   else //need at least one dma channel
   {
      lRetVal = 1;
   }

   //now determine number of logical transfers of each type and how to split up IOs into logical groups
   //NOTE: currently this decision is dumb; ideally it should group IOs into logical transfers such that
   //logical transfer sizes are equivalent(ish) to one another to maximize overlap between logical transfers
   Group (lNumTransfers, lNumHwPath, lpNumLogicalTransfers, lpLogicalTransfers);

   return lRetVal;
}

int ACF_OrgLogXfer_CmemAndDmem(int8_t lNumDmaCh,
                               int8_t lNumStreamCh,
                               int8_t lNumCmemTransfers,
                               int8_t lNumDmemTransfers,
                               int8_t* lpNumLogicalTransfersCmem,
                               struct transfer* lpLogicalTransfersCmem,
                               int8_t* lpNumLogicalTransfersDmem,
                               struct transfer* lpLogicalTransfersDmem)
{
   int lRetVal = 0;
   int8_t lNumHwPathCmem = 0; //number of input DMA+STREAM avail (ext mem to/from CMEM)
   int8_t lNumHwPathDmem = 0; //number of input DMA avail (ext mem to/from DMEM)

   //ideally we should not need 2 and both MEM-MEM and MEM-PERIPH could share a single HW path if necessary, but clover-387
   //prevents this; MEM-MEM and MEM-PERIPH must be separated by a pause or be on different channels altogether!
   //for now keep it simple; take one dmaCh for the dmem (MEM-MEM) input and use whatever is left for the cmem input (MEM-PERIPH)
   if (lNumDmaCh >= 2 && lNumStreamCh >= 1)
   {
      lNumHwPathDmem = 1;
      lNumHwPathCmem = (lNumStreamCh > (lNumDmaCh-1)) ? (lNumDmaCh-1) : lNumStreamCh;
   }
   else
   {
      lRetVal = 1;
   }

   //now determine number of logical transfers of each type and how to split up IOs into logical groups
   //NOTE: currently this decision is dumb; ideally it should group IOs into logical transfers such that
   //logical transfer sizes are equivalent(ish) to one another to maximize overlap between logical transfers
   Group (lNumCmemTransfers, lNumHwPathCmem, lpNumLogicalTransfersCmem, lpLogicalTransfersCmem);
   Group (lNumDmemTransfers, lNumHwPathDmem, lpNumLogicalTransfersDmem, lpLogicalTransfersDmem);

   return lRetVal;
}

void ConfigureLogicalTransfersDma(int lApexId,
                                  int8_t lNumLogicalTransfers,
                                  struct transfer* lpLogicalTransfers,
                                  acf_llentry_desc* lpLleDb,
                                  int8_t* lpDmaCh,
                                  int8_t* lpStreamCh,
                                  int8_t  lType) //0=streamin, 1=streamout, 2=memtomem
{
   for (int i=0; i<lNumLogicalTransfers; i++)
   {
      transfer* lpLogTransfer = &lpLogicalTransfers[i];

      for (int j=0; j<lpLogTransfer->numIdx; j++)
      {
         for (int k=0; k<2; k++) //double buffered LL
         {
            int* lpLleBuf = lpLleDb[lpLogTransfer->idxArray[j]].pEntry[k];
            int* lpNextLleBuf = 0;

            if (0 == lType) //streamin
            {
               //streamin peripheral id
               lpLleBuf[1] &= 0xFFFFFFF0;
               lpLleBuf[1] |= RetStreamInPeriphId(lpStreamCh[i]);
            }
            else if (1 == lType) //streamout
            {
               //streamout peripheral id
               lpLleBuf[0] &= 0xFFFFFFF0;
               lpLleBuf[0] |= RetStreamOutPeriphId(lpStreamCh[i]);
            }

            if (j == (lpLogTransfer->numIdx-1)) //last LLE in the logical transfer
            {
               lpLogTransfer->lastEntryPtr[k] = lpLleBuf;

               //pause+interrupt enable at the end of each logical transfer
               lpLleBuf[2] |= 0xC0000000;

               if (0 == k)
                  lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[0]].pEntry[1]; //last entry of buf0 points to 1st entry of buf1
               else //(1 == k)
                  lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[0]].pEntry[0]; //last entry of buf1 points back to 1st entry of buf0
            }
            else //not the last LLE in the transfer
            {
               lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[j+1]].pEntry[k];
            }

            lpLleBuf[7] = (gsDmemBaseAddrForDma + (uintptr_t)lpNextLleBuf) | 0x3; //0x3 = non-sequential + enable
         }
      }

      mcdma_hal_program_nonseq_linkedlist(lApexId, lpDmaCh[i], gsDmemBaseAddrForDma + (uintptr_t)(lpLleDb[lpLogTransfer->idxArray[0]].pEntry[0]));
   }
}

void ConfigureLogicalTransfersMce(int lApexId,
                                  int8_t lNumLogicalTransfers,
                                  struct transfer* lpLogicalTransfers,
                                  acf_llentry_mce_desc* lpLleDb,
                                  int8_t* lpMceCh,
                                  int8_t* lpStreamCh)
{
   for (int i=0; i<lNumLogicalTransfers; i++)
   {
      transfer* lpLogTransfer = &lpLogicalTransfers[i];

      for (int j=0; j<lpLogTransfer->numIdx; j++)
      {
         for (int k=0; k<2; k++) //double buffered LL
         {
            int* lpLleBuf = lpLleDb[lpLogTransfer->idxArray[j]].pEntry[k];
            int* lpNextLleBuf = 0;

            lpLleBuf[4] &= 0xFFFFFFF0;
            lpLleBuf[4] |= RetStreamInPeriphId(lpStreamCh[i]);

            if (j == (lpLogTransfer->numIdx-1)) //last LLE in the logical transfer
            {
               lpLogTransfer->lastEntryPtr[k] = lpLleBuf;

               //pause+interrupt enable at the end of each logical transfer
               lpLleBuf[0] |= 0xC0000000;

               if (0 == k)
                  lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[0]].pEntry[1]; //last entry of buf0 points to 1st entry of buf1
               else //(1 == k)
                  lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[0]].pEntry[0]; //last entry of buf1 points back to 1st entry of buf0
            }
            else //not the last LLE in the transfer
            {
               lpNextLleBuf = lpLleDb[lpLogTransfer->idxArray[j+1]].pEntry[k];
            }

            lpLleBuf[7] = (gsDmemBaseAddrForDma + (uintptr_t)lpNextLleBuf) | 0x3; //0x3 = non-sequential + enable
         }
      }

      mce_hal_program_nonseq_linkedlist(lApexId, lpMceCh[i], gsDmemBaseAddrForDma + (uintptr_t)(lpLleDb[lpLogTransfer->idxArray[0]].pEntry[0]));
   }
}

#define ACF_COLLECT_VAR_LENGTH_LISTS(tag, VECT, SCL, shift)                      \
void ACF_CollectVarLengthScatteredLists_##tag##vec(int lIter,                    \
                                        ACF_LocalMemDesc lLmDescCmem,            \
                                        ACF_LocalMemDesc lLmDescDmem,            \
                                        vec16u lvBytesPushedIntoFifo,            \
                                        ACF_AdvTransferInfo* lpAdvTransferInfo,  \
                                        uint32_t* lpTotalFifoSize,               \
                                        int lStartCu, int lTileWidthInChunks)    \
{                                                                                \
   VECT* lvpCmem = (VECT*)RetAddrVec ##tag ##_iter(&lLmDescCmem, lIter);         \
   SCL*  lpDmem  = (SCL* )RetAddrScl_iter(&lLmDescDmem, lIter);                  \
   int32_t lDmemOffset = 0;                                                      \
                                                                                 \
   for (int32_t i=lStartCu; i<lTileWidthInChunks; i++)                           \
   {                                                                             \
      int32_t lSize = vget(lvBytesPushedIntoFifo, i);                            \
      for (int32_t x=0; x<(lSize>>shift); x++)                                   \
      {                                                                          \
         lpDmem[lDmemOffset++] = vget(lvpCmem[x], i);                            \
      }                                                                          \
   }                                                                             \
                                                                                 \
   lpAdvTransferInfo->lTransferSizeInBytes = lDmemOffset*(1<<shift);             \
   *lpTotalFifoSize += lDmemOffset*(1<<shift);                                   \
}

ACF_COLLECT_VAR_LENGTH_LISTS(08, vec08s, int8_t,  0)
ACF_COLLECT_VAR_LENGTH_LISTS(16, vec16s, int16_t, 1)
ACF_COLLECT_VAR_LENGTH_LISTS(32, vec32s, int32_t, 2)

#ifndef CGV_FORCE_INLINE
#include "acf_hwutilities_inline.h"
#endif
