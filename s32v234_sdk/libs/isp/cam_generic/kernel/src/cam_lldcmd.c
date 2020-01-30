/*
 * Copyright (c) 2015-2016 Freescale Semiconductor
 * Copyright (c) 2016-2018 NXP
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
/**
 * \file     fdma_lldcmd.c
 * \brief    LLDCMD functionality of fDMA driver
 * \author   Tomas Babinec
 * \version  0.1
 * \date     21. September 2015
 * \note
 ****************************************************************************/

#include <linux/uaccess.h>	// for get_user and put_user 
#include <linux/mutex.h>	// for semaphore/mutex
#include <linux/string.h>	// for memset
#include <linux/sched.h>	// for current->pid
#include <linux/gpio.h>   // Required for the GPIO functions

#include "cam.h"
#include "cam_lldcmd.h"
#include "cam_func.h"
#include "sony_func.h"
#include "ov10640_func.h"
#include "os_oal.h"

#include "vdb_log.h"

#define I2C_CACHE_SIZE (uint32_t)256 // number of bytes for caching I2C addrs and vals

//*****************************************************************************

// CAM locking mutex
#if 0
extern mutex_t gCamMutex;
#endif // 0

//*****************************************************************************

static long ReadRegBatch(unsigned long aLldcmdArg);
static long WriteRegBatch(unsigned long aLldcmdArg);
static long ReadRegSingle(unsigned long aLldcmdArg);
static long WriteRegSingle(unsigned long aLldcmdArg);
static long ControlGpio(GpioControlCmd_t aGpioCtrlCmd);

//*****************************************************************************

long CamLldcmd(OAL_LldCmd_t* apCmd)
{
  long lRet = CAM_DRV_SUCCESS;
#if 0  
  // only one LLDCMD can be processed at the time
  if(mutex_lock_interruptible(&gCamMutex) > 0)
  {
    VDB_LOG_WARNING("CAM mutex down interrupted.\n");
    lRet = -ERESTARTSYS;
  } // if CAM mutex interrupted
  else
  {
    VDB_LOG_NOTE("CAM mutex locked.\n");
#endif //0
    
    CamI2cCfg_t lCfg;
    CamI2cClient_t lCamHandle;
    uint32_t lI2CRepeat;

    // switch the lldcmd command
    switch (apCmd->mCommandId) 
    {
      //***********************************************************************
      // *** Generic camera ***      

      case LLDCMD_CAM_CLI_ADD:
        if (copy_from_user(&lCfg, (CamI2cCfg_t *)apCmd->mArg, 
            sizeof(CamI2cCfg_t)) != 0)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        {  
          CamI2cClient_t lClientHandle;
          lClientHandle = CAM_DRV_ClientAdd(&lCfg);
          if(lClientHandle == 0)
          {
            VDB_LOG_ERROR("Failed to add a client.\n");
            lRet = CAM_DRV_FAILURE;
          } // if failed to add a client
          else
          {
            // *** write back the client handle to user space ***
            if (copy_to_user((CamI2cClient_t*)apCmd->mArg, 
                              &lClientHandle, 
                              sizeof(CamI2cClient_t)) != 0)
            {
              VDB_LOG_ERROR("Copy to user failed.\n");
              lRet = -EIO;
            } // if copy from user failed
          } // else from if failed to add a client
        } // else from if copy from user failed
        break;

      case LLDCMD_CAM_CLI_REM:
        lCamHandle = (CamI2cClient_t)apCmd->mArg;
        lRet = CAM_DRV_ClientRem(lCamHandle);
        break;

      case LLDCMD_CAM_REG_RDB:
        lRet = ReadRegBatch(apCmd->mArg);       
        break;

      case LLDCMD_CAM_REG_WRB:
        lRet = WriteRegBatch(apCmd->mArg);                
        break;

      case LLDCMD_CAM_REG_RDS:
        lRet = ReadRegSingle(apCmd->mArg);       
        break;
      
      case LLDCMD_CAM_REG_WRS:
        lRet = WriteRegSingle(apCmd->mArg);                
        break;

      case LLDCMD_CAM_I2C_REPEAT_SET:
        if (copy_from_user(&lI2CRepeat, (uint32_t *)apCmd->mArg, 
            sizeof(uint32_t)) != 0)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        CAM_DRV_I2C_RepeatSet(lI2CRepeat);
        break;

      case LLDCMD_CAM_I2C_REPEAT_GET:
        // *** write back the current number of I2C retransmission to user space ***
        CAM_DRV_I2C_RepeatGet(&lI2CRepeat);
        if (copy_to_user((uint32_t *)apCmd->mArg, &lI2CRepeat, 
                          sizeof(uint32_t)) != 0)
        {
          VDB_LOG_ERROR("Copy to user failed.\n");
          lRet = -EIO;
        } // if copy to user failed
        break;

      //***********************************************************************
      // *** Sony imx224 MipiCam ***
      case LLDCMD_SONY_CONNECT:
        if(SONY_DRV_Setup((CamI2cClient_t)apCmd->mArg))              
        {
          VDB_LOG_ERROR("Failed to connect Sony camera.\n");
          lRet = CAM_DRV_FAILURE;
        } // if failed to read the data
        break;
      
      case LLDCMD_SONY_DISCONNECT:
        if(SONY_DRV_Close((CamI2cClient_t)apCmd->mArg))              
        {
          VDB_LOG_ERROR("Failed to disconnect Sony camera.\n");
          lRet = CAM_DRV_FAILURE;
        } // if failed to read the data        
        break;
      
      case LLDCMD_SONY_EXP_STATUS_GET:
      {
        uint32_t lArg = 0;
        if (copy_from_user(&lArg, (uint32_t *)apCmd->mArg, 
            sizeof(uint32_t)) != 0)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        {
          if (0U == lArg)
          {
            lArg = SONY_DRV_ExpStateGet(CSI_IDX_0);
          }
          else if(1U == lArg)
          {
            lArg = SONY_DRV_ExpStateGet(CSI_IDX_1);
          }
          else
          {
            VDB_LOG_ERROR("Failed to get the Sony exposure status.\n");
            lRet = CAM_DRV_FAILURE;
          }
          if (copy_to_user((uint32_t *)apCmd->mArg, &lArg, 
                          sizeof(uint32_t)) != 0)
          {
            VDB_LOG_ERROR("Copy to user failed.\n");
            lRet = -EIO;
          } // if copy to user failed
        } //else from if copy from user failed
      }
        break;
      case LLDCMD_SONY_EXP_SENT:
      {       
        SONY_DRV_ExpSent((enum I2C_BUS_IDX)apCmd->mArg);
      } // LLDCMD_SONY_EXP_STATUS_SET
      break;
     
      //***********************************************************************
      // *** Maxim MipiCam Start/Stop ***
      
      //***********************************************************************
      // *** OV10635 Viu ***
      
      //***********************************************************************
      // *** Maxim OV10640 ***
      
      //***********************************************************************
      // *** OV10640 MipiCam ***
      case LLDCMD_OV10640_CONNECT:
        if(OV10640_DRV_Setup((CamI2cClient_t)apCmd->mArg))              
        {
          VDB_LOG_ERROR("Failed to connect Sony camera.\n");
          lRet = CAM_DRV_FAILURE;
        } // if failed to read the data
        break;
      
      case LLDCMD_OV10640_DISCONNECT:
        if(OV10640_DRV_Close((CamI2cClient_t)apCmd->mArg))              
        {
          VDB_LOG_ERROR("Failed to disconnect Sony camera.\n");
          lRet = CAM_DRV_FAILURE;
        } // if failed to read the data        
        break;
      //***********************************************************************

      case LLDCMD_CAM_GPIO_CONTROL:
      {
        GpioControlCmd_t lGpioCtrlCmd;

        if (copy_from_user(&lGpioCtrlCmd, (GpioControlCmd_t *)apCmd->mArg, sizeof(GpioControlCmd_t)) == 0)
        {
          ControlGpio(lGpioCtrlCmd);
        }
        else
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        }
      }
        break;

      case LLDCMD_CAM_RESET:
      {
        CamResetPinCmd_t lResetCtrlCmd;

        if (copy_from_user(&lResetCtrlCmd, (CamResetPinCmd_t *)apCmd->mArg,
            sizeof(CamResetPinCmd_t)) == 0)
        {
          CAM_DRV_Reset(lResetCtrlCmd);
        } // copy from user success
        else
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // copy from user fail
      } // case LLDCMD_CAM_RESET
      break;

      case LLDCMD_CAM_POWER_CONTROL:
      {
        CamPowerControlCmd_t lPowerCtrlCmd;

        if (copy_from_user(&lPowerCtrlCmd, (CamPowerControlCmd_t *)apCmd->mArg,
            sizeof(CamPowerControlCmd_t)) == 0)
        {
          CAM_DRV_Power_Control(lPowerCtrlCmd);
        } // copy from user success
        else
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // copy from user fail
      } // case LLDCMD_CAM_RESET
      break;

      default:
        VDB_LOG_WARNING("Unknown lldcmd command.\n");
        lRet = printk("Unknown lldcmd command.\n");
        
        lRet = -ENOTTY;
        break;            
      
    } // switch lldcmd command
#if 0  
    // relase the CAM mutex
    mutex_unlock(&gCamMutex);
    VDB_LOG_NOTE("Cam mutex released.\n");
  }// else from if mutex down interrupted
#endif // 0

  return lRet;
} // CamLldcmd()

//*****************************************************************************

static long ReadRegBatch(unsigned long aLldcmdArg)
{
  long lRet = CAM_DRV_SUCCESS;
  I2cXfrReadBatch_t lBatch;
  I2cXfrSingle_t lXfr;
  uint8_t lpI2cMsgData[CAM_I2C_SINGLE_DATA_MAX] = {0};

  // *** fetch batch info from user space ***
  if (copy_from_user( &lBatch, (I2cXfrReadBatch_t *)aLldcmdArg, 
      sizeof(I2cXfrReadBatch_t)) != 0)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  { 
    if( (lBatch.mDestDev  != 0) &&
        (lBatch.mpByteCnt != NULL) && 
        (lBatch.mpData   != NULL))
    {
      //const CamGeneric_t * const lpCam = lBatch.mDestDev;
      uint32_t      lPartSize = (uint32_t)(I2C_CACHE_SIZE/((uint32_t)3)); // 3 bytes per read command
      uint32_t      i;

      lXfr.mDestDev = lBatch.mDestDev;
      // *** divide batch into smaller parts ***
      for(i = 0; i < lBatch.mCmdCnt; i += lPartSize)
      {
        uint32_t lCmdCnt = min((lBatch.mCmdCnt - i), lPartSize);
        uint8_t lpByteCnt[I2C_CACHE_SIZE/3];
        uintptr_t  lpData[I2C_CACHE_SIZE/3];
        
        // *** fetch register addresses from user space ***
        if (copy_from_user( lpByteCnt, 
                            (uint8_t *)&(lBatch.mpByteCnt[i]), 
                            sizeof(uint8_t) * lCmdCnt) != 0
            || 
            copy_from_user( lpData, 
                            (uint8_t *)&(lBatch.mpData[i]), 
                            sizeof(uintptr_t) * lCmdCnt) != 0
          )
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        { 
          // *** read from the addresses ***
          uint32_t j;
          for( j = 0; j < lCmdCnt; j++)
          {
                  //Run with a command
                  lXfr.mByteCnt = lpByteCnt[j];
                  if(lXfr.mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
                  {
                    // check data not too big
                    if(lXfr.mByteCnt <= CAM_I2C_SINGLE_DATA_MAX)
                    {

                      //The value of lpData[j] is the user pointer to i2cmessage
                      // *** get the additional i2c message data ***
                      if (copy_from_user(lpI2cMsgData, 
                                         (void*)lpData[j], 
                                         sizeof(uint8_t)*lXfr.mByteCnt) != 0)
                      {
                        VDB_LOG_ERROR("Copy from user failed.\n");
                        lRet = -EIO;
                      } // if copy from user failed    

                      //assign the data pointer (kernel space)
                      lXfr.mData = (uintptr_t)lpI2cMsgData;
                      if(CAM_DRV_Read(&lXfr) == CAM_DRV_SUCCESS)
                      {
                        // *** write back the read data to user space ***
                        if (copy_to_user((void*)lpData[j], 
                                         lpI2cMsgData, 
                                         sizeof(uint8_t)*lXfr.mByteCnt) != 0)
                        {
                          VDB_LOG_ERROR("Copy to user failed.\n");
                          lRet = -EIO;
                        } // if copy from user failed        
                      } // if I2C read successfull  
                      else
                      {
                        lRet = -EIO;
                      } // else from if I2C read successfull  
                    } // if data size in range
                    else
                    {
                      VDB_LOG_ERROR("I2c data too long %u (allowed %u).\n",
                                    lpByteCnt[j],
                                    CAM_I2C_SINGLE_DATA_MAX
                                   );
                      lRet = -EIO;
                    } // else from if data size not in range
                  } // if data don't fit into structure
                  else
                  {

                    lXfr.mData = lpData[j];
                    if(CAM_DRV_Read(&lXfr) == CAM_DRV_SUCCESS)
                    {
                      // *** write back the read data to user space ***
                      //lBatch.mData contains the pointer to user space
                      if (copy_to_user(&(lBatch.mpData[i+j]), 
                                       &lXfr.mData, 
                                       sizeof(uintptr_t)) != 0)
                      {
                        VDB_LOG_ERROR("Copy to user failed.\n");
                        lRet = -EIO;
                      } // if copy from user failed        
                    } // if I2C read successfull  
                    else
                    {
                      lRet = -EIO;
                    } // else from if I2C read successfull  
                  } // else from if data don't fit into structure


          } // for all commands in this part of the batch
          
        } // else from if copy from user failed
      } // until all commands fetched and done
    } // if pointers not NULL
    else
    {
      VDB_LOG_ERROR("Batch contains NULL pointers.\n");
      lRet = -EIO;
    } // else from if pointers not NULL
  } // else from if copy from user failed   
  
  return lRet;
} // ReadRegBatch()

//*****************************************************************************

static long WriteRegBatch(unsigned long aLldcmdArg)
{
  long lRet = CAM_DRV_SUCCESS;
  I2cXfrWriteBatch_t lBatch;
  uint8_t lpI2cMsgData[CAM_I2C_SINGLE_DATA_MAX] = {0};
  uint8_t lpI2cMskData[CAM_I2C_SINGLE_DATA_MAX] = {0};

  // *** fetch batch info from user space ***
  if (copy_from_user( &lBatch, (I2cXfrWriteBatch_t *)aLldcmdArg, 
      sizeof(I2cXfrWriteBatch_t)) != 0)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  { 
    if((lBatch.mDestDev  != 0) &&
       (lBatch.mpCmdArr  != NULL))
    {
      const CamGeneric_t * const lpCam     = lBatch.mDestDev;
      
      uint32_t      lNumBytes = (uint32_t)sizeof(I2cXfrWriteCmd_t);
      uint32_t      lPartSize = (uint32_t)(I2C_CACHE_SIZE/lNumBytes);
      uint32_t      i, j;
      
      // *** divide batch into smaller parts ***
      for(i = 0; i < lBatch.mCmdCnt; i += lPartSize)
      {
        uint32_t      lCmdCnt = min((lBatch.mCmdCnt - i), lPartSize);
        I2cXfrWriteCmd_t lpCmd[lPartSize];
        
        // *** fetch commands from user space ***
        if (copy_from_user( lpCmd, 
                            (I2cXfrWriteCmd_t *)&(lBatch.mpCmdArr[i]), 
                            sizeof(I2cXfrWriteCmd_t) * lCmdCnt) != 0)
        {
          VDB_LOG_ERROR("Copy from user failed.\n");
          lRet = -EIO;
        } // if copy from user failed
        else
        { 
          // *** write the commands ***
          for( j = 0; j < lCmdCnt; j++)
          {
           
                  if(lpCmd[j].mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
                  {
                    // check data not too big
                    if(lpCmd[j].mByteCnt <= CAM_I2C_SINGLE_DATA_MAX)
                    {
                      // *** get the additional i2c message data ***
                      if (copy_from_user(lpI2cMsgData, 
                                         (void*)lpCmd[j].mData, 
                                         sizeof(uint8_t)*lpCmd[j].mByteCnt) != 0)
                      {
                        VDB_LOG_ERROR("Copy from user failed.\n");
                        lRet = -EIO;
                      } // if copy from user failed    
                      else
                      {

                        // Check if mask is applied or not
                        if (((uintptr_t)0) != lpCmd[j].mMask)
                        {
                            // *** get the additional i2c mask data ***
                            if (copy_from_user(lpI2cMskData, 
                                         (void*)lpCmd[j].mMask, 
                                         sizeof(uint8_t)*lpCmd[j].mByteCnt) != 0)
                            {
                              VDB_LOG_ERROR("Copy mask from user failed.\n"
                                      "Terminating batch command at index %u.\n", 
                                      j
                                     );
                              lRet = -EIO;
                              //break;
                            } // if copy from user failed  
                            else 
                            {
                              //Apply mask to a command
                              lpCmd[j].mMask = (uintptr_t)lpI2cMskData;
                            }
                            
                        }

                        //Apply data to a command
                        lpCmd[j].mData = (uintptr_t)lpI2cMsgData;          
                        if(CAM_DRV_CmdWrite(lpCam, 
                                    &(lpCmd[j])) == CAM_DRV_SUCCESS)
                        {
                          
                        } // if I2C write command successful  
                        else
                        {
                          lRet = -EIO;
                        } // else from if I2C write command successfull 
                      } // else from if copy from user failed    
                    } // if data size in range
                    else
                    {
                      VDB_LOG_ERROR("Failed to allocate required memory.\n");
                      lRet = -EIO;
                    } // else from if data size not in range
                  } // if data don't fit into structure
                  else
                  {
                      if(CAM_DRV_CmdWrite(lpCam, 
                                    &(lpCmd[j])) != CAM_DRV_SUCCESS)              
                      {
                        VDB_LOG_ERROR("Failed to write to I2C address\n"
                                      "Terminating batch command at index %u.\n", 
                                      j
                                     );
                        lRet = -EIO;
                        //break;
                      } // if failed to read the data 
                  } // else from if data don't fit into structure
          

          } // for all commands in this part of the batch
        } // else from if copy from user failed
      } // until all commands fetched and done
    } // if pointers not NULL
    else
    {
      VDB_LOG_ERROR("Batch contains NULL pointers.\n");
      lRet = -EIO;
    } // else from if pointers not NULL
  } // else from if copy from user failed  
  
  return lRet;
} // WriteRegBatch()

//*****************************************************************************

static long ReadRegSingle(unsigned long aLldcmdArg)
{
  long lRet = CAM_DRV_SUCCESS;
  I2cXfrSingle_t lXfr;
  uint8_t lpI2cMsgData[CAM_I2C_SINGLE_DATA_MAX] = {0};

  // *** fetch transfer info from user space ***
  if (copy_from_user( &lXfr, 
                      (I2cXfrSingle_t *)aLldcmdArg, 
                      sizeof(I2cXfrSingle_t)) != 0)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  { 
    if(lXfr.mDestDev != 0)
    { 
      if(lXfr.mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
      {
        // check data not too big
        if(lXfr.mByteCnt <= CAM_I2C_SINGLE_DATA_MAX)
        {
          //remember user data pointer
          void *lpData = (void*)lXfr.mData;

          // *** get the additional i2c message data ***
          if (copy_from_user(lpI2cMsgData, 
                             (void*)lpData, 
                             sizeof(uint8_t)*lXfr.mByteCnt) != 0)
          {
            VDB_LOG_ERROR("Copy from user failed.\n");
            lRet = -EIO;
          } // if copy from user failed    

          //assign the data pointer
          lXfr.mData = (uintptr_t)lpI2cMsgData;
          if(CAM_DRV_Read(&lXfr) == CAM_DRV_SUCCESS)
          {
            // *** write back the read data to user space ***
            if (copy_to_user(lpData, 
                             lpI2cMsgData, 
                             sizeof(uint8_t)*lXfr.mByteCnt) != 0)
            {
              VDB_LOG_ERROR("Copy to user failed.\n");
              lRet = -EIO;
            } // if copy from user failed        
          } // if I2C read successfull  
          else
          {
            lRet = -EIO;
          } // else from if I2C read successfull  
        } // if data size in range
        else
        {
          VDB_LOG_ERROR("I2c data too long %u (allowed %u).\n",
                        lXfr.mByteCnt,
                        CAM_I2C_SINGLE_DATA_MAX
                       );
          lRet = -EIO;
        } // else from if data size not in range
      } // if data don't fit into structure
      else
      {
        if(CAM_DRV_Read(&lXfr) == CAM_DRV_SUCCESS)
        {
          // *** write back the read data to user space ***
          if (copy_to_user((I2cXfrSingle_t*)aLldcmdArg, 
                           &lXfr, 
                           sizeof(I2cXfrSingle_t)) != 0)
          {
            VDB_LOG_ERROR("Copy to user failed.\n");
            lRet = -EIO;
          } // if copy from user failed        
        } // if I2C read successfull  
        else
        {
          lRet = -EIO;
        } // else from if I2C read successfull  
      } // else from if data don't fit into structure
    } // if device valid
    else
    {
      VDB_LOG_ERROR("Invalid camera device handle.\n");
      lRet = -EIO;
    } // else if device valid
  } // else from if copy from user failed   
  
  return lRet;
} // ReadRegSingle()

//*****************************************************************************

static long WriteRegSingle(unsigned long aLldcmdArg)
{
  long lRet = CAM_DRV_SUCCESS;
  I2cXfrSingle_t lXfr;
  uint8_t lpI2cMsgData[CAM_I2C_SINGLE_DATA_MAX] = {0};
  // *** fetch transfer info from user space ***
  if (copy_from_user( &lXfr, 
                      (I2cXfrSingle_t *)aLldcmdArg, 
                      sizeof(I2cXfrSingle_t)) != 0)
  {
    VDB_LOG_ERROR("Copy from user failed.\n");
    lRet = -EIO;
  } // if copy from user failed
  else
  { 
    if(lXfr.mDestDev != 0)
    { 
      if(lXfr.mByteCnt > CAM_I2C_SINGLE_DATA_SZ)
      {
        // check data not too big
        if(lXfr.mByteCnt <= CAM_I2C_SINGLE_DATA_MAX)
        {
          // *** get the additional i2c message data ***
          if (copy_from_user(lpI2cMsgData, 
                             (void*)lXfr.mData, 
                             sizeof(uint8_t)*lXfr.mByteCnt) != 0)
          {
            VDB_LOG_ERROR("Copy from user failed.\n");
            lRet = -EIO;
          } // if copy from user failed    
          else
          {
            lXfr.mData = (uintptr_t)lpI2cMsgData;          
            if(CAM_DRV_Write(&lXfr) == CAM_DRV_SUCCESS)
            {
              
            } // if I2C read successfull  
            else
            {
              lRet = -EIO;
            } // else from if I2C read successfull
          } // else from if copy from user failed    
        } // if data size in range
        else
        {
          VDB_LOG_ERROR("Failed to allocate required memory.\n");
          lRet = -EIO;
        } // else from if data size not in range
      } // if data don't fit into structure
      else
      {
        if(CAM_DRV_Write(&lXfr) != CAM_DRV_SUCCESS)
        {          
          lRet = -EIO;
        } // else from if I2C read successfull  
      } // else from if data don't fit into structure
    } // if device valid
    else
    {
      VDB_LOG_ERROR("Invalid camera device handle.\n");
      lRet = -EIO;
    } // else if device valid
  } // else from if copy from user failed  
  
  return lRet;
} // WriteRegSingle()

//*****************************************************************************

static long ControlGpio(GpioControlCmd_t aGpioCtrlCmd)
{
  long lRet = CAM_DRV_SUCCESS;
  gpio_set_value((unsigned int)aGpioCtrlCmd.mGpioPin, (unsigned int) aGpioCtrlCmd.mVal);

  return lRet;
} // ControlGpio()

//*****************************************************************************
