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

#include <seq_hal.h>
#include <hw_reg_user.h>

typedef unsigned int REG32;
#include "apex_642cl_seq_map.h"

#if ((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE))
#include <seq_func_sim.h>
#endif

void seq_hal_set_rsc_counter(int apexID, int lRsc, int lVal)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   volatile REG32* lCntArray = (volatile REG32*)&(lpSeqRegisters->ResourceFix[0]);
   lCntArray[lRsc] = lVal;
}

int seq_hal_ret_rsc_counter(int apexID, int lRsc)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   volatile REG32* lCntArray = (volatile REG32*)&(lpSeqRegisters->ResourceFix[0]);
   return lCntArray[lRsc];
}

void seq_hal_proc_inc_rscmask(int apexID, int lProcId, int lRscMask)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpSeqRegisters->ProcInc[lProcId].as32BitValue = lRscMask;

#else

   SequencerRscInc(apexID, lRscMask);

#endif
}

void seq_hal_proc_dec_rscmask(int apexID, int lProcId, int lRscMask)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->ProcDec[lProcId].as32BitValue = lRscMask;
}

void seq_hal_enable_seq(int apexID, int lProcId)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->Enable.as32BitValue = 1<<lProcId;
}

void seq_hal_disable_seq(int apexID, int lProcId)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpSeqRegisters->SeqDisable.as32BitValue = 1<<lProcId;

#else

   lpSeqRegisters->Enable.as32BitValue &= ~(1<<lProcId);

#endif
}

int seq_hal_query_interrupt(int apexID, int lProcId)
{
   int lRetVal = 0;
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   if (lpSeqRegisters->Isr.as32BitValue & (1<<lProcId))
      lRetVal = 1;
   return lRetVal;
}

void seq_hal_clear_interrupt(int apexID, int lProcId)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpSeqRegisters->Icr.as32BitValue = (1<<lProcId);

#else

   lpSeqRegisters->Isr.as32BitValue &= (~(1<<lProcId));

#endif
}

void seq_hal_scb_enable(int apexID, int lScbWord)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->ScbEnable.as32BitValue = lScbWord;
}

void seq_hal_scb_disable(int apexID, int lScbWord)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpSeqRegisters->ScbDisable.as32BitValue = lScbWord;

#else

   lpSeqRegisters->ScbEnable.as32BitValue &= ~lScbWord;

#endif
}

void seq_hal_set_dma_inc(int apexID, int lCh, int lVal)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->DmaInc[lCh].as32BitValue = lVal;
}

void seq_hal_set_dma_dec(int apexID, int lCh, int lVal)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->DmaDec[lCh].as32BitValue = lVal;
}

void seq_hal_set_motioncomp_inc(int apexID, int lCh, int lVal)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->MotionCompInc[lCh].as32BitValue = lVal;
}

void seq_hal_set_motioncomp_dec(int apexID, int lCh, int lVal)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->MotionCompDec[lCh].as32BitValue = lVal;
}

void seq_hal_mask_interrupts(int apexID, int lMask)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   lpSeqRegisters->Imr.as32BitValue = lMask;
}

void seq_hal_mask_interrupts_apu(int apexID, int lApuId, int lMask)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   if (0 == lApuId)
   {
      lpSeqRegisters->Imr2.as32BitValue = lMask;
   }
   else //1 == lApuId
   {
      lpSeqRegisters->Imr3.as32BitValue = lMask;
   }
}

void seq_hal_clear_interrupts(int apexID, int lMask)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpSeqRegisters->Icr.as32BitValue = lMask;

#else

   lpSeqRegisters->Isr.as32BitValue &= ~lMask;

#endif
}

int seq_hal_query_interrupts(int apexID)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);
   return lpSeqRegisters->Isr.as32BitValue;
}

int RetMceScbId(int lIdx)
{
   int lScbId = 0;
   switch(lIdx)
   {
   case 0: lScbId = 24; break;
   case 1: lScbId = 25; break;
   }
   return lScbId;
}

//----------------------------------------------------------------------------------------------------
//APU specific methods to work around HW issue CLOVER-403
//----------------------------------------------------------------------------------------------------

void seq_hal_enable_seq_apu(int apexID, int apuID, int lProcId)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

   //enable SEQ (NOTE: multiple writes based on priority is a workaround for CLOVER-403)
   if (0 == apuID)
   {
      lpSeqRegisters->Enable.as32BitValue = 1<<lProcId;
      lpSeqRegisters->Enable.as32BitValue = 1<<lProcId;
   }
   else //(1 == apuID)
   {
      lpSeqRegisters->Enable.as32BitValue = 1<<lProcId;
   }
}

void seq_hal_disable_seq_apu(int apexID, int apuID, int lProcId)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   //disable SEQ (NOTE: multiple writes based on priority is a workaround for CLOVER-403)
   if (0 == apuID)
   {
      lpSeqRegisters->SeqDisable.as32BitValue = 1<<lProcId;
      lpSeqRegisters->SeqDisable.as32BitValue = 1<<lProcId;
   }
   else //(1 == apuID)
   {
      lpSeqRegisters->SeqDisable.as32BitValue = 1<<lProcId;
   }

#else

   lpSeqRegisters->Enable.as32BitValue &= ~(1<<lProcId);

#endif
}

void seq_hal_scb_enable_apu(int apexID, int apuID, int lScbWord)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

   //enable SCBs (NOTE: multiple writes based on priority is a workaround for CLOVER-403)
   if (0 == apuID)
   {
      lpSeqRegisters->ScbEnable.as32BitValue = lScbWord;
      lpSeqRegisters->ScbEnable.as32BitValue = lScbWord;
   }
   else //(1 == apuID)
   {
      lpSeqRegisters->ScbEnable.as32BitValue = lScbWord;
   }
}

void seq_hal_scb_disable_apu(int apexID, int apuID, int lScbWord)
{
   volatile APEX_642CL_SEQ_DCR *const lpSeqRegisters = (APEX_642CL_SEQ_DCR*)ReturnVirtualHwPtr(apexID, HW_REG_USEQ);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   //disable SCBs (NOTE: multiple writes based on priority is a workaround for CLOVER-403)
   if (0 == apuID)
   {
      lpSeqRegisters->ScbDisable.as32BitValue = lScbWord;
      lpSeqRegisters->ScbDisable.as32BitValue = lScbWord;
   }
   else //(1 == apuID)
   {
      lpSeqRegisters->ScbDisable.as32BitValue = lScbWord;
   }

#else

   lpSeqRegisters->ScbEnable.as32BitValue &= ~lScbWord;

#endif
}
