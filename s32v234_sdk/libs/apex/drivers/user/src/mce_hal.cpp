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

#include <mce_hal.h>
#include <hw_reg_user.h>
#include <time_helper.h>

typedef unsigned int REG32;
#include "apex_642cl_motion_comp_map.h"

#if ((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE))
#include <mce_func_sim.h>
#endif

void mce_hal_program_linkedlist(int lApexID, unsigned int chNo, unsigned int linkedListAddr)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);
   lpMceRegisters->Ch[chNo].LinkedList.as32BitValue = (linkedListAddr | 0x01);
}

void mce_hal_program_nonseq_linkedlist(int lApexID, unsigned int chNo, unsigned int linkedListAddr)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);
   lpMceRegisters->Ch[chNo].LinkedList.as32BitValue = (linkedListAddr | 0x03);
}

void mce_hal_enable_ch(int lApexID, unsigned int chNo)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpMceRegisters->ChEnable.as32BitValue = (1 << chNo);

#else

   MceChStep(lApexID, chNo);

#endif
}

void mce_hal_wait_polling(int lApexID, unsigned int chNo)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);
   while(lpMceRegisters->ChEnable.as32BitValue & (1 << chNo)) ;
}

#if !defined(__chess__) && !defined(__clang__) //THIS FUNCTION IS ONLY FOR HOST USE (i.e. do not use on the APU)!!!!

int mce_hal_wait_ch_enable_timeout(int lApexID, unsigned int chNo, int timeoutInUs)
{
   int retVal = 0;

   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);

   uint64_t timeoutProbe = RET_TICKS;

   while(lpMceRegisters->ChEnable.as32BitValue & (1 << chNo))
   {
      if (CONVERT_TICKS_TO_US(RET_TICKS - timeoutProbe) >= timeoutInUs)
      {
         lpMceRegisters->ChClear.as32BitValue = (1 << chNo);
         retVal = 1;
         break;
      }

      _wait_cc(10); //prevent overly aggressive polling
   }

   lpMceRegisters->Icr.as32BitValue = (1 << chNo);

   return retVal;
}

#endif //!defined(__chess__) && !defined(__clang__)

void mce_hal_clear_int(int lApexID, unsigned int chNo)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);

#if (!((1 == APEX_SIM_ISS) || (1 == APEX_SIM_NATIVE)))

   lpMceRegisters->Icr.as32BitValue = (1 << chNo);

#else

   lpMceRegisters->Isr.as32BitValue &= (~(1 << chNo));

#endif
}

void mce_hal_clear_ch(int lApexID, unsigned int chNo)
{
   volatile APEX_642CL_MOTION_COMP_DCR *const lpMceRegisters = (APEX_642CL_MOTION_COMP_DCR*)ReturnVirtualHwPtr(lApexID, HW_REG_MODMA);
   lpMceRegisters->ChClear.as32BitValue = (1 << chNo);
}