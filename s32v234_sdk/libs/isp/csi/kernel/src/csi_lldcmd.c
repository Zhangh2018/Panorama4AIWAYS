/*
 * Copyright (c) 2017-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*==============================================================================
Revision History:
                        Modification       Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Tomas Babinec           28-Apr-2016                   Init version
Nguyen Tri Hai(B51945)  1-Dec-2017        VSDK-1225   Resolve Misra violations
Tomas Babinec           03-Jan-2018       VSDK-1624   LLD CMD
==============================================================================*/
/*==============================================================================
*                                 INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==============================================================================*/
/**
* @file            csi_lldcmd.c
*/

#include <linux/uaccess.h>	// for get_user and put_user
#include <linux/mutex.h>	// for semaphore/mutex
#include <linux/string.h>	// for memset
#include <linux/sched.h>	// for current->pid

#include "csi.h"
#include "csi_lldcmd.h"
#include "csi_func.h"
#include "os_oal.h"

#include "vdb_log.h"


/*==============================================================================
*                                MACROS
==============================================================================*/
// CSI locking mutex
static DEFINE_MUTEX(sCsiMutex);

/*==============================================================================
*                                GLOBAL FUNCTIONS
==============================================================================*/

/**
* @brief          Gets current embedded status
* @details        Gets current embedded status
*
* @param[in]      aLldcmdArg the IOCT parameter.
* @return         int
* @api
* @isr
*
* @pre
*
* @post
*
*
* @note
*/
static int32_t CsiEmbdStatusGet(uint64_t aLldcmdArg);

/**
* @brief          Gets current error status
* @details        Gets current error status
*
* @param[in]      aLldcmdArg the IOCT parameter.
* @return         int
* @api
* @isr
*
* @pre
*
* @post
*
*
* @note
*/
static int32_t CsiErrStatusGet(uint64_t aLldcmdArg);

/*============================================================================*/
/**
* @brief          The function manipulates the underlying device parameters.
* @details        The function manipulates the underlying device parameters of
*                 special files. In particular, many operating characteristics
*                 of character special files (e.g., terminals) may be controlled
*                 with CsiLldcmd() requests.
*
* @param[in]      apCmd     pointer to command structure
*
* @return         long
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/

int32_t CsiLldcmd(const OAL_LldCmd_t* apCmd);

int32_t CsiLldcmd(const OAL_LldCmd_t* apCmd)
{
  int32_t lRet = CSI_DRV_SUCCESS;

  // only one LLDCMD can be processed at the time
  if(mutex_lock_interruptible(&sCsiMutex) > 0)
  {
    VDB_LOG_WARNING("CSI mutex down interrupted.\n");
    lRet = -ERESTARTSYS;
  } // if CSI mutex interrupted
  else
  {
    VDB_LOG_NOTE("CSI mutex locked.\n");

    // switch the lldcmd command
    switch (apCmd->mCommandId)
    {
      case LLDCMD_CSI_CONFIG:
      {
        CSI_Cfg_t lCfg;

        if (copy_from_user( &lCfg, (CSI_Cfg_t *)apCmd->mArg,
            sizeof(CSI_Cfg_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        {
          CSI_DRV_Config(&lCfg);
        } // else from if copy from user failed
      } // case LLDCMD_CSI_INIT
      break;

      case LLDCMD_CSI_EMBD_CONFIG:
      {
        CSI_EmbdCfg_t lCfg;

        if (copy_from_user( &lCfg, (CSI_EmbdCfg_t *)apCmd->mArg,
            sizeof(CSI_EmbdCfg_t)) != 0U)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        {
          CSI_DRV_EmbdConfig(&lCfg);
        } // else from if copy from user failed
      } // case LLDCMD_CSI_INIT
      break;

      case LLDCMD_CSI_EMBD_RECAP:
      {
        enum CSI_IDX lCsiIdx = (enum CSI_IDX)apCmd->mArg;
        CSI_DRV_EmbdRecap(lCsiIdx);
      } // case LLDCMD_CSI_EMBD_RECAP
      break;

      case LLDCMD_CSI_EMBD_STATUS_GET:
      {
        lRet = CsiEmbdStatusGet(apCmd->mArg);
      } // case LLDCMD_CSI_EMBD_STATUS_GET
      break;

      case LLDCMD_CSI_ERR_STATUS_GET:
      {
        lRet = CsiErrStatusGet(apCmd->mArg);
      } // LLDCMD_CSI_ERR_STATUS_GET
      break;

      case LLDCMD_CSI_ERR_CLEAR:
      {
        enum CSI_IDX lCsiIdx = (enum CSI_IDX)apCmd->mArg;
        CSI_DRV_ErrClear(lCsiIdx);
      } // case LLDCMD_CSI_ERR_CLEAR
      break;

      case LLDCMD_CSI_STOP:
      {
        CSI_IdKey_t lCsiIdKey;
        lCsiIdKey.mKey = (uint32_t)apCmd->mArg;

        CSI_DRV_Stop(lCsiIdKey);
      } // case LLDCMD_CSI_STOP
      break;

      case LLDCMD_CSI_START:
      {
        CSI_IdKey_t lCsiIdKey;
        lCsiIdKey.mKey = (uint32_t)apCmd->mArg;

        CSI_DRV_Start(lCsiIdKey);
      } // case LLDCMD_CSI_START
      break;

      case LLDCMD_CSI_SWRESET:
      {
        CSI_IDX_t lCsiIdx = (CSI_IDX_t)apCmd->mArg;

        CSI_DRV_SwReset(lCsiIdx);
      } // case LLDCMD_CSI_SWRESET
      break;

      default:
      {
        VDB_LOG_WARNING("Unknown lldcmd command.\n");

        lRet = -ENOTTY;
      } // default
        break;
    } // switch lldcmd command

    // relase the CSI mutex
    mutex_unlock(&sCsiMutex);
    VDB_LOG_NOTE("Csi mutex released.\n");
  }// else from if mutex down interrupted
  return lRet;
} // CsiLldcmd()

/*============================================================================*/
/**
* @details        The function provides CSI embedded status.
* @brief          The function provides CSI embedded status.
*                 *
* @param[in]      aLldcmdArg the IOCT parameter.
* @return         int
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/

static int32_t CsiEmbdStatusGet(uint64_t aLldcmdArg)
{
  int32_t lRet = 0;
  CSI_EmbdStatus_t lEmbdStat;

  if (copy_from_user( &lEmbdStat, (CSI_EmbdStatus_t *)aLldcmdArg,
      sizeof(CSI_EmbdStatus_t)) != 0U)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  {
    // for input mOthrEmbdId used as CSI_IDX
    enum CSI_IDX lCsiIdx = (enum CSI_IDX) lEmbdStat.mOthrEmbdId;

    if(lCsiIdx < CSI_IDX_INVALID)
    {
      lEmbdStat.mEmbdStatus = CSI_DRV_EmbdStatusGet(lCsiIdx);

      if (copy_to_user((CSI_EmbdStatus_t*)aLldcmdArg,
                      &lEmbdStat,
                      sizeof(CSI_EmbdStatus_t)) != 0U)
      {
        VDB_LOG_ERROR("Copy to user failed.\n");
        lRet = -EIO;
      } // if copy from user failed
    } // if idx in range
    else
    {
      lRet = CSI_DRV_FAILURE;
      VDB_LOG_ERROR("CSI idx out of range.\n");
    }// else from if idx in range
  } // else from if copy from user failed

  return lRet;
} // CsiEmbdStatusGet()

/*============================================================================*/
/**
* @brief          The function provides CSI error status.
* @details        The function provides CSI error status.
*                 *
* @param[in]      aLldcmdArg the IOCT parameter.
* @return         int
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/

static int32_t CsiErrStatusGet(uint64_t aLldcmdArg)
{
  int32_t lRet = 0;
  CSI_ErrStatus_t lErrStat;

  if (copy_from_user( &lErrStat, (CSI_ErrStatus_t *)aLldcmdArg,
      sizeof(CSI_ErrStatus_t)) != 0U)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  {
    // for input mOthrErrId used as CSI_IDX
    enum CSI_IDX lCsiIdx = (enum CSI_IDX) lErrStat.mOthrErrFrmId;

    if(lCsiIdx < CSI_IDX_INVALID)
    {
      const CSI_ErrStatus_t *lpErrStat = CSI_DRV_ErrStatusGet(lCsiIdx);

      if (copy_to_user((CSI_ErrStatus_t*)aLldcmdArg,
                      lpErrStat,
                      sizeof(CSI_ErrStatus_t)) != 0U)
      {
        VDB_LOG_ERROR("Copy to user failed.\n");
        lRet = -EIO;
      } // if copy from user failed
    } // if idx in range
    else
    {
      lRet = CSI_DRV_FAILURE;
      VDB_LOG_ERROR("CSI idx out of range.\n");
    }// else from if idx in range
  } // else from if copy from user failed

  return lRet;
} // CsiErrStatusGet()
