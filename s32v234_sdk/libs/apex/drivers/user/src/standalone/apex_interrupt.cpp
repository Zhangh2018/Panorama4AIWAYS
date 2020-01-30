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
#include <apex_interrupt.h>
#include <oal.h>
#include <oal_irq.h>
#include <seq_hal.h>

extern APEX_INTERRUPTS gApexImrState;

void __attribute__((optimize("no-tree-slp-vectorize"))) APEX_InterruptSignalHandler(int32_t lApexId)
{
   //printf("APEX_InterruptSignalHandler() called!\n");

   //handle all relevant APEX interrupts here!!!
   //1) capture state
   //2) clear interrupts
   //3) 'handle' interrupts (i.e. release applicable semaphores!)
   
   APEX_INTERRUPTS lIsrState = {0, 0, 0};

   //SEQ
   lIsrState.seq = seq_hal_query_interrupts(lApexId) & ~gApexImrState.seq; //capture
   seq_hal_clear_interrupts(lApexId, lIsrState.seq); //clear

   APEX_InterruptSignalHandlerCommon(lApexId, lIsrState);
}

//Interrupt signal handler for APEX 0
void APEX_InterruptSignalHandlerApex0()
{
  //printf("APEX interrupt 0\n");
  APEX_InterruptSignalHandler(0);
}

//Interrupt signal handler for APEX 1
void APEX_InterruptSignalHandlerApex1()
{
  //printf("APEX interrupt 1\n");
  APEX_InterruptSignalHandler(1);
}

//Interrupt setup
#define APEX0_INT_ID 141
#define APEX1_INT_ID 142
int32_t APEX_InterruptSetup()
{
   int32_t lRetVal = OAL_SUCCESS;

   //setup for each APEX
   lRetVal |= OAL_IRQInit(APEX_InterruptSignalHandlerApex0, APEX0_INT_ID, 0xA0, OAL_IRQ_LEVEL);
   lRetVal |= OAL_IRQInit(APEX_InterruptSignalHandlerApex1, APEX1_INT_ID, 0xA0, OAL_IRQ_LEVEL);

   if (OAL_SUCCESS == lRetVal)
   {
      //enable for each APEX
      OAL_IRQEnable(APEX0_INT_ID);
      OAL_IRQEnable(APEX1_INT_ID);
   }

   return lRetVal;
}

//Interrupt cleanup
int32_t APEX_InterruptCleanup()
{
   int32_t lRetVal = OAL_SUCCESS;

   //cleanup for APEX 0
   OAL_IRQDisable(APEX0_INT_ID);
   lRetVal |= OAL_IRQDelete(APEX0_INT_ID);

   //cleanup for APEX 1
   OAL_IRQDisable(APEX1_INT_ID);
   lRetVal |= OAL_IRQDelete(APEX1_INT_ID);

   return lRetVal;
}
