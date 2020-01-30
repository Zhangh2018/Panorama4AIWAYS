/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* Copyright (c) 2017 NXP;

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
/**
* \file oal_interrupt.cpp
* \brief OAL interrupt handling functionality
* \author Rostislav Hulik
* \version
* \date
****************************************************************************/

#include "oal.h"
#include "os_oal_check.h"
#include <gic400.h>

////////////////////////////////////////////////////////////////////////
/// Initialize an interrupt and add a service routine to it.
////////////////////////////////////////////////////////////////////////
int32_t OAL_IRQInit(void (*pServiceRoutineFunc)(void), unsigned int irq, unsigned int priority, IRQ_MODE mode)
{
  int32_t retval = OAL_SUCCESS;

  CHECK_INIT(retval)
  if (retval == OAL_SUCCESS)
  {
      if (irq < INT_NUM_SOURCES) 
      {
        g_INT_vectors[irq] = (int_function_t)pServiceRoutineFunc;
        gic400_setIntPriority(irq, priority);
        gic400_setIntTarget(irq, GIC400_TARGET_CPU0);
        if (mode == OAL_IRQ_LEVEL) {
          gic400_configureSPI(irq, GIC400_CONFIG_LEVEL);
        }
        else if (mode == OAL_IRQ_EDGE) {
          gic400_configureSPI(irq, GIC400_CONFIG_EDGE);
        }
        else {
            retval = OAL_FAILURE;
        }
      }
      else
      {
        retval = OAL_FAILURE;
      }
  }
  return retval;
}

////////////////////////////////////////////////////////////////////////
/// Enables the specified interrupt.
////////////////////////////////////////////////////////////////////////
void OAL_IRQEnable(unsigned int irq)
{
  int32_t retval = OAL_SUCCESS;

  CHECK_INIT(retval)
  if (retval == OAL_SUCCESS)
  {
    if (irq < INT_NUM_SOURCES)
    {
        gic400_enableIntID(irq);
    }
  }
}

////////////////////////////////////////////////////////////////////////
/// Deinitialize an interrupt and removes the service routine link.
////////////////////////////////////////////////////////////////////////
int32_t OAL_IRQDelete(unsigned int irq)
{
  int32_t retval = OAL_SUCCESS;

  CHECK_INIT(retval)
  if (retval == OAL_SUCCESS)
  {
    if (irq < INT_NUM_SOURCES)
    {
        g_INT_vectors[irq] = (int_function_t)default_interrupt_routine;
        retval = OAL_SUCCESS;
    }
    else
    {
        retval = OAL_FAILURE;
    }
  }
  return retval;
}

////////////////////////////////////////////////////////////////////////
/// Disables the specified interrupt.
////////////////////////////////////////////////////////////////////////
void OAL_IRQDisable(unsigned int irq)
{
  int32_t retval = OAL_SUCCESS;

  CHECK_INIT(retval)
  if (retval == OAL_SUCCESS)
  {
    if (irq < INT_NUM_SOURCES)
    {
        gic400_disableIntID(irq);
    }
  }
}
