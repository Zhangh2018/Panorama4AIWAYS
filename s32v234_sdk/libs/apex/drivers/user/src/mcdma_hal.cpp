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

#include <mcdma_hal.h>
#include <hw_reg_user.h>
#include <stdio.h>
#include <stdint.h>
#include <time_helper.h>

typedef unsigned int REG32;
#include "apex_642cl_mcdma_map.h"

#if ((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE))
#include <mcdma_func_sim.h>
#endif

void mcdma_hal_program_seq_linkedlist(int apexID, unsigned int ChannelN, unsigned int LinkedListAddr)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);
   
   //if ((lpDmaRegisters->ChEnable.as32BitValue &  (1 << ChannelN)) == (1 << ChannelN))
   //   printf("mcdma_hal_program_seq_linkedlist:CH%d ISR:0x%08X EN:0x%08X\n", ChannelN, lpDmaRegisters->Isr.as32BitValue, lpDmaRegisters->ChEnable.as32BitValue);

   //configure linked list address and enable (mode will be 0 indicating sequential)
   lpDmaRegisters->Ch[ChannelN].LinkedList.as32BitValue = (LinkedListAddr | 0x01);

#if 0
#ifndef __chess__
   if (lpDmaRegisters->Ch[ChannelN].LinkedList.as32BitValue != (LinkedListAddr | 0x01))
   {
      printf("mcdma_hal_program_seq_linkedlist: Ch[%d].LinkedList.as32BitValue = 0x%08X (should have been 0x%08X)\n", ChannelN, lpDmaRegisters->Ch[ChannelN].LinkedList.as32BitValue, (LinkedListAddr | 0x01));
      printf("                                  Ch[%d].StatusFsm.as32BitValue = 0x%08X\n", ChannelN, lpDmaRegisters->Ch[ChannelN].StatusFsm.as32BitValue);
      printf("                                  Ch[%d].Byte.as32BitValue = 0x%08X\n", ChannelN, lpDmaRegisters->Ch[ChannelN].Byte.as32BitValue);
      printf("                                  ChEnable.as32BitValue = 0x%08X\n", lpDmaRegisters->ChEnable.as32BitValue);
   }
#endif
#endif
}

void mcdma_hal_program_nonseq_linkedlist(int apexID, unsigned int ChannelN, unsigned int LinkedListAddr)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);

   //configure linked list address and enable (mode will be 1 indicating non-sequential)
   lpDmaRegisters->Ch[ChannelN].LinkedList.as32BitValue = (LinkedListAddr | 0x03);
}

void mcdma_hal_interruptclear_ch_done(int apexID, unsigned int ChannelN)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpDmaRegisters->Icr.as32BitValue = (1 << ChannelN);

#else

   lpDmaRegisters->Isr.as32BitValue &= (~(1 << ChannelN));

#endif
}

void mcdma_hal_start(int apexID, unsigned int ChannelN)
{
   //int lCount = 0;
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);
   mcdma_hal_interruptclear_ch_done(apexID, ChannelN);

#if 0
   while ((lpDmaRegisters->Isr.as32BitValue & (1 << ChannelN)) == (1 << ChannelN))
   {
      lCount++;
   }
#ifndef __chess__
   if (lCount >0)
      printf("mcdma_hal_start: lCount=%d, ch=%d\n", lCount, ChannelN);
#endif
#endif

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpDmaRegisters->ChEnable.as32BitValue = (1 << ChannelN);

#else

   McdmaChStep(apexID, ChannelN);

#endif
}

void mcdma_hal_wait(int apexID, unsigned int ChannelN)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);

   while (lpDmaRegisters->ChEnable.as32BitValue & (1 << ChannelN))
   {
   }

   mcdma_hal_interruptclear_ch_done(apexID, ChannelN);
}

#if !defined(__chess__) && !defined(__clang__) //THESE FUNCTIONS ARE ONLY FOR HOST USE (i.e. do not use on the APU)!!!!

//a DMA channel should never be in an enabled state after its ISR has been received; the 'gDmaChEnableErrCnt' variable
//was added to help verify that this is true
static int gDmaChEnableErrCnt = 0;
int mcdma_hal_wait_timeout(int apexID, unsigned int ChannelN, int timeoutInUs)
{
   int retVal = 0;

   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);

   volatile uint32_t reg_isr;
   //volatile uint32_t reg_en[2];

   uint64_t timeoutProbe = RET_TICKS;
   int timeoutDetected = 0;

   while (1)
   {
      if (((reg_isr = lpDmaRegisters->Isr.as32BitValue) & (1u << ChannelN)) == (1u << ChannelN))
      {
         mcdma_hal_interruptclear_ch_done(apexID, ChannelN); //clear irq
         break;
      }

      if (1 == timeoutDetected)
      {
         ++gDmaChEnableErrCnt;
   #if 0
         printf(" **** mcdma_hal_wait_timeout(): [%1i][%2i] %6i I: %08X E: %08X %08X\n", apexID, ChannelN, gDmaChEnableErrCnt, reg_isr, reg_en[0], reg_en[1]);
         volatile uint32_t* reg_ch_sts = &lpDmaRegisters->Ch[ChannelN].Src.as32BitValue;
         const int nreg = &lpDmaRegisters->Ch[ChannelN].Priority.as32BitValue - &lpDmaRegisters->Ch[ChannelN].Src.as32BitValue;
         for (int r=0; r<nreg; ++r)
            printf("    [%1i][%2i][%2i] %08X\n", apexID, ChannelN, r, reg_ch_sts[r]);
   #endif
         // hesitant to reset channel ... timeout need to dump information about the channel information
         mcdma_hal_ch_clear(apexID, ChannelN);
         mcdma_hal_interruptclear_ch_done(apexID, ChannelN); //clear irq
         retVal = -1;
         break;
      }

      //timeout detection
      if (CONVERT_TICKS_TO_US(RET_TICKS - timeoutProbe) >= timeoutInUs)
         timeoutDetected = 1;

      _wait_cc(10);
   }

   //printf("CH%d ISR:0x%08X EN:0x%08X\n", ChannelN, lpDmaRegisters->Isr.as32BitValue, lpDmaRegisters->ChEnable.as32BitValue);

   return retVal;
}

int mcdma_hal_wait_ch_enable_timeout(int apexID, unsigned int ChannelN, int timeoutInUs)
{
   int retVal = 0;

   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);

   uint64_t timeoutProbe = RET_TICKS;

   while (lpDmaRegisters->ChEnable.as32BitValue & (1 << ChannelN))
   {
      if (CONVERT_TICKS_TO_US(RET_TICKS - timeoutProbe) >= timeoutInUs)
      {
         lpDmaRegisters->ChClear.as32BitValue = (1u << ChannelN);
         lpDmaRegisters->ChFlush.as32BitValue = (1u << ChannelN);
         retVal = 1;
         break;
      }

      _wait_cc(10); //prevent overly aggressive polling
   }

   lpDmaRegisters->Icr.as32BitValue = (1 << ChannelN);

   return retVal;
}

#endif //!defined(__chess__) && !defined(__clang__)

void mcdma_hal_set_memory_fill_words(int apexID, 
									          unsigned int FillWord0,
                                     unsigned int FillWord1,
                                     unsigned int FillWord2,
                                     unsigned int FillWord3)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);
   lpDmaRegisters->MemoryFill[0].Word[0].as32BitValue = FillWord0;
   lpDmaRegisters->MemoryFill[0].Word[1].as32BitValue = FillWord1;
   lpDmaRegisters->MemoryFill[0].Word[2].as32BitValue = FillWord2;
   lpDmaRegisters->MemoryFill[0].Word[3].as32BitValue = FillWord3;
}

int RetStreamInPeriphId(int lIdx)
{
   int lPeriphId = 0;
   switch(lIdx)
   {
   case 0: lPeriphId = DST_CMEM_DMA_STREAM_IN0; break;
   case 1: lPeriphId = DST_CMEM_DMA_STREAM_IN1; break;
   case 2: lPeriphId = DST_CMEM_DMA_STREAM_IN2; break;
   case 3: lPeriphId = DST_CMEM_DMA_STREAM_IN3; break;
   }
   return lPeriphId;
}

int RetStreamOutPeriphId(int lIdx)
{
   int lPeriphId = 0;
   switch(lIdx)
   {
   case 0: lPeriphId = SRC_CMEM_DMA_STREAM_OUT0; break;
   case 1: lPeriphId = SRC_CMEM_DMA_STREAM_OUT1; break;
   }
   return lPeriphId;
}

void mcdma_hal_ch_clear(int apexID, unsigned int ChannelN)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);
   lpDmaRegisters->ChClear.as32BitValue = (1u << ChannelN);
}

void mcdma_hal_ch_flush(int apexID, unsigned int ChannelN)
{
   volatile APEX_642CL_MCDMA_DCR *const lpDmaRegisters = (APEX_642CL_MCDMA_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_MCDMA);
   lpDmaRegisters->ChFlush.as32BitValue = (1u << ChannelN);
}
