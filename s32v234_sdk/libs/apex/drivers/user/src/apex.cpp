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

#include <apex.h>
#include <hw_reg_user.h>
#include <global_hal.h>
#include <resource_manager_apex.h>
#include <apu_hal.h>
#include <apex_interrupt.h>
#include <seq_hal.h>

static int gsApexInitFlag = 0;

int APEX_Init()
{
   int lRetVal = 0;

   lRetVal |= InitVirtualHwPtrArray();

   if (0 == lRetVal)
   {
      global_hal_AllReset(0);
      global_hal_AllReset(1);

      lRetVal |= ResourceManagerInit_APEX(global_hal_RetHwVersion(0));

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
      lRetVal |= apu_hal_Init(); //required for semaphore protection related to apu enable/disable
      lRetVal |= APEX_InterruptInit();
#endif
      
      if (0 == lRetVal)
      {
         gsApexInitFlag = 1;
      }
   }

   return lRetVal;
}

int APEX_Deinit()
{
   int lRetVal = 0;

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))
   lRetVal |= APEX_InterruptDeinit();
   lRetVal |= apu_hal_Deinit();
#endif

   lRetVal |= ResourceManagerDeinit_APEX(global_hal_RetHwVersion(0));
   lRetVal |= DeinitVirtualHwPtrArray();
   
   if (0 == lRetVal)
   {
      gsApexInitFlag = 0;
   }

   return lRetVal;
}

int RetApexInitFlag()
{
   return gsApexInitFlag;
}

void APEX_Reset(int lApexId)
{
   global_hal_AllReset(lApexId);

   //restore sequencer interrupt masks initially setup in APEX_InterruptInit()
   //(these are wiped in the sequencer reset)
   seq_hal_mask_interrupts(lApexId,  ~(1<<30 | 1<<31));
}
