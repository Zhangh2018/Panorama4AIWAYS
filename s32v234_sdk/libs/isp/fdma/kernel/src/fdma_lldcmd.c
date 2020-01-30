/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file     fdma_lldcmd.c
 * \brief    LLD CMD functionality of fDMA driver
 * \author   Tomas Babinec
 * \version  0.1
 * \date     4. September 2015
 * \note
 ****************************************************************************/


/*=============================================================================
*                              INCLUDE FILES
==============================================================================*/

#include <linux/uaccess.h>  // for get_user and put_user
#include <linux/mutex.h>  // for semaphore/mutex
#include <linux/string.h> // for memset
#include <linux/sched.h>  // for current->pid

#include "fdma_func.h"
#include "os_oal.h"

#include "vdb_log.h"

#include "fdma_lldcmd.h"

/*==============================================================================
*                            DEFINES AND MACROS
==============================================================================*/
// fDMA locking mutex
static DEFINE_MUTEX(sFdmaMutex);

/*==============================================================================
*                                  ENUMS
==============================================================================*/

/*=============================================================================
*                        STRUCTURES AND OTHER TYPEDEFS
* ============================================================================*/

/*==============================================================================
*                        LOCAL FUNCTION PROTOTYPES
==============================================================================*/

int64_t FdmaLldcmd(const OAL_LldCmd_t* apCmd);

/*==============================================================================
*                        LOCAL FUNCTION DECLARATIONS
==============================================================================*/

int64_t FdmaLldcmd(const OAL_LldCmd_t* apCmd)
{
  int32_t lRet = FDMA_DRV_SUCCESS;

  // only one LLDCMD can be processed at the time
  if(mutex_lock_interruptible(&sFdmaMutex) > 0)
  {
    VDB_LOG_WARNING("fDMA mutex down interrupted.\n");
    lRet = -ERESTARTSYS;
  } // if fDMA mutex interrupted
  else
  {
    VDB_LOG_NOTE("fDMA mutex locked.\n");

    // switch the command number
    switch (apCmd->mCommandId)
    {
      case LLDCMD_FDMA_TD_RESERVE:
      {
        FDMA_Tdm_t *lpFreeTdm = NULL;

        //try to reserve TD
        lpFreeTdm = FDMA_TdmFreePop();
        if( lpFreeTdm != NULL)
        {
          // set the TDM
          lpFreeTdm->mReserved = 1;
          lpFreeTdm->mOwnerProcId = current->pid;

          // copy TDM back to user
          if (copy_to_user((FDMA_Tdm_t *)apCmd->mArg,
                           lpFreeTdm,
                           sizeof(FDMA_Tdm_t)) != 0U)
          {
            VDB_LOG_ERROR("Copy to user failed.\n");
            lRet = -EIO;
          } // if copy to user failed
        } // if pop succeeded
        else
        {
          // return failure (leave TC reset to user space)
          lRet = FDMA_DRV_FAILURE;
        } // if pop failed
      } // case LLDCMD_FDMA_TD_RESERVE
      break;

      case LLDCMD_FDMA_TD_ISP_CONFIG:
      case LLDCMD_FDMA_TD_CONFIG:
      {
        FDMA_Tc_t lUsrTc;
        VDB_LOG_NOTE("FDM_TD_CONFIG.\n");
        // copy user TC to kernel
        if (copy_from_user(&lUsrTc,
                           (FDMA_Tc_t *)apCmd->mArg,
                           sizeof(FDMA_Tc_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        // set the new TD values
        if(apCmd->mCommandId == LLDCMD_FDMA_TD_CONFIG)
        {
          if(FDMA_TdConfig(&lUsrTc) != FDMA_DRV_SUCCESS)
          {
            VDB_LOG_ERROR("TD config failed.\n");
            lRet = FDMA_DRV_FAILURE;
            break;
          }
        } // if user TD config
        else
        {
          if(FDMA_TdIspConfig(&lUsrTc) != FDMA_DRV_SUCCESS)
          {
            VDB_LOG_ERROR("TD config failed.\n");
            lRet = FDMA_DRV_FAILURE;
            break;
          }
        }// else from if user TD config

      } // case LLDCMD_FDMA_TD_CONFIG
      break;

      case LLDCMD_FDMA_TD_SCHEDULE:
      {
        FDMA_Tc_t lUsrTc;
        // copy user TC to kernel
        if (copy_from_user(&lUsrTc,
                           (FDMA_Tc_t *)apCmd->mArg,
                           sizeof(FDMA_Tc_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        if(FDMA_TcSchedule(&lUsrTc) != FDMA_DRV_SUCCESS)
        {
          lRet = FDMA_DRV_FAILURE;
          VDB_LOG_WARNING("TC scheduling failed.\n");
        } // if TC cheduling failed
      } // case LLDCMD_FDMA_TD_SCHEDULE
      break;

      case LLDCMD_FDMA_TD_CONFIG_SCHEDULE:
      {
        FDMA_Tc_t lUsrTc;
        VDB_LOG_NOTE("FDM_TD_CONFIG_SCHEDULE.\n");
        // copy user TC to kernel
        if (copy_from_user(&lUsrTc,
                           (FDMA_Tc_t *)apCmd->mArg,
                           sizeof(FDMA_Tc_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        // set the new TD values
        if(FDMA_TdConfig(&lUsrTc) != FDMA_DRV_SUCCESS)
        {
          VDB_LOG_ERROR("TD config failed.\n");
          lRet = FDMA_DRV_FAILURE;
          break;
        }

        //schedule part
        if(FDMA_TcSchedule(&lUsrTc) != FDMA_DRV_SUCCESS)
        {
          lRet = FDMA_DRV_FAILURE;
        }
      } // case LLDCMD_FDMA_TD_CONFIG_SCHEDULE
      break;

      case LLDCMD_FDMA_TD_RELEASE:
      {
        FDMA_Tdm_t lUsrTdm;

        // copy user TC to kernel
        if (copy_from_user(&lUsrTdm,
                           (FDMA_Tdm_t *)apCmd->mArg,
                           sizeof(FDMA_Tdm_t)) != 0)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        //try to push free TDM
        if( FDMA_TdRelease(&lUsrTdm) != FDMA_DRV_SUCCESS )
        {
          VDB_LOG_ERROR("Td release failed.\n");
          lRet = FDMA_DRV_FAILURE;
        } // if release failed

        // leave TC reset to user space
      } //case LLDCMD_FDMA_TD_RELEASE
      break;

      case LLDCMD_FDMA_SEQ_MODE_GET:
      {
        int32_t lSeqMod = (int32_t)FDMA_SeqModeGet();
        // copy fdma registers back to user
        if (copy_to_user( (int32_t*)apCmd->mArg,
                          (int32_t*)&lSeqMod,
                          sizeof(int32_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy to user failed.\n");
          lRet = -EIO;
        } // if copy to user failed
      } //case LLDCMD_FDMA_SEQ_MODE_GET
      break;

      case LLDCMD_FDMA_STATUS_GET:
      {
        FDMA_Regs_t *lpFdmaRegs = FDMA_RegsGet();

        if(lpFdmaRegs != NULL)
        {
          // copy fdma registers back to user
          if (copy_to_user((void*)apCmd->mArg,
                           (void*)lpFdmaRegs,
                           sizeof(FDMA_Regs_t)) != 0U)
          {
            VDB_LOG_ERROR("Copy to user failed.\n");
            lRet = -EIO;
          } // if copy to user failed
        }else
        {
          VDB_LOG_ERROR("FDMA registers not mapped.\n");
          lRet = FDMA_DRV_FAILURE;
        }
      } //case LLDCMD_FDMA_STATUS_GET
      break;

      case LLDCMD_FDMA_EVENT_GET:
      {
        FDMA_TcEvent_t lEvent;
        VDB_LOG_NOTE("FDM_EVENT_GET.\n");
        // copy user values to kernel
        if (copy_from_user(&lEvent,
                           (FDMA_TcEvent_t *)apCmd->mArg,
                           sizeof(FDMA_TcEvent_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        if(FDMA_EventGet(&lEvent) != FDMA_DRV_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to get Event.\n");
          lRet = FDMA_DRV_FAILURE;
        } // if failed to get event

        // copy fdma registers back to user
        if (copy_to_user((void*)apCmd->mArg,
                          (void*)&lEvent,
                          sizeof(FDMA_TcEvent_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy to user failed.\n");
          lRet = -EIO;
        } // if copy to user failed
      } //case LLDCMD_FDMA_STATUS_GET
      break;

      case LLDCMD_FDMA_TC_GET:
      {
        FDMA_Tc_t lUsrTc;

        // copy user TC to kernel
        if (copy_from_user(&lUsrTc,
                           (FDMA_Tc_t *)apCmd->mArg,
                           sizeof(FDMA_Tc_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
          break;
        } // if copy from user failed

        // read the TD
        if(FDMA_TcGet(&lUsrTc) != FDMA_DRV_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to get the TC.\n");
          lRet = FDMA_DRV_FAILURE;
        }else
        {
          // copy TC back to user
          if (copy_to_user((FDMA_Tc_t *)apCmd->mArg,
                           &lUsrTc,
                           sizeof(FDMA_Tc_t)) != 0U)
          {
              VDB_LOG_ERROR("Copy to user failed.\n");
              lRet = -EIO;
          } // if copy to user failed
        }
      } //case LLDCMD_FDMA_TD_GET
        break;

      case LLDCMD_FDMA_TD_USED:
      {
        uint32_t td_free = FDMA_TdmFreeCntGet();
        uint32_t td_used = FDMA_TD_CNT - td_free;
        VDB_LOG_NOTE("TD_CNT: %u, Free: %u, Used: %u\n", td_used, td_free);
        // copy TC back to user
        if (copy_to_user((int32_t *)apCmd->mArg,
                         &td_used,
                         sizeof(int32_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy to user failed.\n");
          lRet = -EIO;
        } // if copy to user failed
      } //case LLDCMD_FDMA_TD_USED
        break;

      case LLDCMD_FDMA_TDT_GET:
      {
        uint64_t lTdtBasePhys = FDMA_TdtPhysAddrGet();
        // copy TDT address to user
        if (copy_to_user((uint64_t *)apCmd->mArg,
                         &lTdtBasePhys,
                         sizeof(uint64_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy to user failed.\n");
          lRet = -EIO;
        } // if copy to user failed
      } //case LLDCMD_FDMA_STATUS_GET
        break;

      default:
      {
        VDB_LOG_WARNING("Unknown lld command number.\n");
        lRet = -ENOTTY;
      } // default
        break;
    } // switch lld command

    // relase the fDMA mutex
    mutex_unlock(&sFdmaMutex);
    VDB_LOG_NOTE("fDMA mutex released.\n");
  }// else from if mutex down interrupted

  return lRet;
} // FdmaLldcmd()
