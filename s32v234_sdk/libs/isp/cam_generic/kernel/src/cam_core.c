/*
 * Copyright (c) 2016 Freescale Semiconductor
 * Copyright 2016-2017 NXP
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
 * \file     cam_core.c
 * \brief    init/exit functionality of camera driver
 * \author   Tomas Babinec
 * \version  0.1
 * \date     13.6.2016
 * \note
 ****************************************************************************/
#include <linux/uaccess.h>  // for get_user and put_user 
#include <linux/kernel.h>	// We're doing kernel work
#include <linux/module.h>	// Specifically, a module 
#include <linux/moduleparam.h>  // for cmd line arguments
#include <linux/device.h> // for class_create
#include <linux/slab.h>		// for kmalloc
#include <linux/interrupt.h>  // For interrupts

#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>		// Char device structure 
#include <linux/mutex.h>	// for semaphore/mutex
#include <linux/gpio.h>   // Required for the GPIO functions
#include <linux/string.h>

#include "config.h"
#include "cam.h"
#include "cam_func.h"
// #include "csi_types.h"    // To get csi index count
#include "os_oal.h"
#include "vdb_log.h"

//*****************************************************************************
// constants
//*****************************************************************************

//*****************************************************************************
// external function declarations
//*****************************************************************************

long CamLldcmd(OAL_LldCmd_t* apCmd);

//*****************************************************************************
// Local function declarations
//*****************************************************************************
static int CamOpen(struct inode *const apInode, struct file *const apFile);
static int CamRelease(struct inode *const apInode, struct file *const apFile);
static ssize_t CamWrite(struct file* apFile, 
                        const char*  apcData, 
                        size_t       aSize, 
                        loff_t*      apOffset);
static int __init CamInit(void);
static void __exit CamExit(void);

//*****************************************************************************
// Global variables
//*****************************************************************************

//*****************************************************************************
// Static variables
//*****************************************************************************

// internal device open counter
static int sDeviceOpenCnt = 0;

// char device structure for fDMA module
static struct cdev sCamCdev;

// device number
static unsigned int SCamDevno;
// major device number
//static int sMajor;

// Cam driver class
static struct class *sCamDriverClass = NULL;

// file operations
static struct file_operations sFops = {
  .write   = CamWrite,
  .open    = CamOpen,
  .release = CamRelease,  /* a.k.a. close */
}; // struct file_operations

//****************************************************************************
// 
// Essential kernel module behaviour definition 
//
static int CamOpen(struct inode *const apInode, struct file *const apFile)
{
  VDB_LOG_NOTE("Cam generic driver open call\n");

  sDeviceOpenCnt++;
  
  try_module_get(THIS_MODULE);
  return CAM_DRV_SUCCESS;
} // CamOpen()

//****************************************************************************

//
// called whenever a process attempts to close the device file 
//
static int CamRelease(struct inode *const apInode, struct file *const apFile)
{
  sDeviceOpenCnt--;

  module_put(THIS_MODULE);

  return CAM_DRV_SUCCESS;
} // CamRelease()

//*****************************************************************************

static ssize_t CamWrite(struct file* apFile, 
                        const char*  apcData, 
                        size_t       aSize, 
                        loff_t*      apOffset)
{
  ssize_t lRet = aSize;
  if(aSize == sizeof(OAL_LldCmd_t))
  {
    OAL_LldCmd_t lCmd = {1};
    if(copy_from_user(
         &lCmd, 
         apcData, 
         aSize) == 0)
    {
      int32_t lRes = 0;
      lRes = (ssize_t)CamLldcmd(&lCmd);
      if(lRes < 0)
      {
        lRet = lRes;
      } // if returned error
    } // if copy from user ok
    else
    {
      lRet = -EIO;
    } // else from if copy from user ok
  } // if command size fits
  else
  {
    lRet = (ssize_t)(-EOPNOTSUPP);
  } // else from if command size fits
  
  return lRet;
} // CamWrite()

//****************************************************************************

// 
// Initialize the module - Register the character device 
//
static int __init CamInit(void)
{
  int32_t lRetVal = CAM_DRV_SUCCESS;
  
  // enable logging
  lRetVal = vdb_LogInit();

    printk("Load evb board configuration\n");
    gCsiResetPin = gEvbCsiResetPin;
    gCsiPowerDownPin = gEvbCsiPowerDownPin;
    gViuResetPin = gEvbViuResetPin;
    gViuPowerDownPin = gEvbViuPowerDownPin;
#if 0
  if ((of_machine_is_compatible(CAM_EVB_29288_COMPATIBLE_STRING) > 0) ||
      (of_machine_is_compatible(CAM_EVB_28899_COMPATIBLE_STRING) > 0))
  {
    // evb board
    printk("Load evb board configuration\n");
    gCsiResetPin = gEvbCsiResetPin;
    gCsiPowerDownPin = gEvbCsiPowerDownPin;
    gViuResetPin = gEvbViuResetPin;
    gViuPowerDownPin = gEvbViuPowerDownPin;
  }
  else if (of_machine_is_compatible(CAM_SBC_COMPATIBLE_STRING) > 0)
  {
    // sbc board
    printk("Load sbc board configuration\n");
    gCsiResetPin = gSbcCsiResetPin;
    gCsiPowerDownPin = gSbcCsiPowerDownPin;
    gViuResetPin = gSbcViuResetPin;
    gViuPowerDownPin = gSbcViuPowerDownPin;
  }
  else
  {
    VDB_LOG_ERROR("CAM: Unsupported board.\n");
    return CAM_DRV_FAILURE;
  }
#endif
  // *** Setup the driver ***  
  if(CAM_DRV_Setup() != CAM_DRV_SUCCESS)
  {
    lRetVal = printk("CGD: setup failed.\n");
    lRetVal = CAM_DRV_FAILURE;
  } // if driver setup failed
  else
  {
    // *** initialize device file ***
  
    // get device number
    if (alloc_chrdev_region(&SCamDevno, 0, 1, CAM_DEVICE_NAME) < 0)
    {
      lRetVal = printk("CGD: Device number allocation failed.\n");
      lRetVal = CAM_DRV_Close();
      lRetVal = CAM_DRV_FAILURE;
    } // if alloc char dev number failed
    else
    {
      sCamDriverClass = class_create(THIS_MODULE, "cam_generic_drv");
      if(IS_ERR(sCamDriverClass))
      {
        lRetVal = printk("CGD: Driver class creation failed.\n");
        unregister_chrdev_region(SCamDevno, 1);	  
        lRetVal = CAM_DRV_Close();
        lRetVal = CAM_DRV_FAILURE;
      } // if cam driver class creation failed
      else
      {
        if (device_create(sCamDriverClass, 
                    NULL, 
                    SCamDevno, 
                    NULL, 
                    CAM_DEVICE_NAME) == NULL)
        {
          lRetVal = printk("CGD: Driver device creation failed.\n");
          class_destroy(sCamDriverClass);
          unregister_chrdev_region(SCamDevno, 1);	  
          lRetVal = CAM_DRV_Close();
          lRetVal = CAM_DRV_FAILURE;
        } // if device not created
        else
        {
          cdev_init(&sCamCdev, &sFops);
          sCamCdev.owner = THIS_MODULE;
          sCamCdev.ops = &sFops;

          if(0 != cdev_add(&sCamCdev, SCamDevno, 1))
          {
            lRetVal = printk("CGD: Device registration failed.\n");
            device_destroy(sCamDriverClass, SCamDevno);
            class_destroy(sCamDriverClass);
            unregister_chrdev_region(SCamDevno, 1);	  
            lRetVal = CAM_DRV_Close();
            lRetVal = CAM_DRV_FAILURE;
          } // if device registration failed
          else
          {
            uint8_t i;
            // set reset/power down pin as output
            for(i = 0; i < (uint32_t)CAM_CSI_DEV_CNT; i++)
            {
              // config pins
              if (gCsiResetPin[i] != CAM_GPIO_INVALID_PIN)
              {
                if (gpio_direction_output(gCsiResetPin[i], 1))
                {
                  lRetVal = CAM_DRV_FAILURE;
                  break;
                }
              } // if gpio pin is valid

              if (gCsiPowerDownPin[i] != CAM_GPIO_INVALID_PIN)
              {
                if (gpio_direction_output(gViuPowerDownPin[i], 1))
                {
                  lRetVal = CAM_DRV_FAILURE;
                  break;
                }
              } // if gpio pin is valid
            }

            for(i = 0; i < (uint32_t)CAM_VIU_DEV_CNT; i++)
            {
              // config pins
              if (gViuResetPin[i] != CAM_GPIO_INVALID_PIN)
              {
                if (gpio_direction_output(gCsiResetPin[i], 1))
                {
                  lRetVal = CAM_DRV_FAILURE;
                  break;
                }
              } // if gpio pin is valid

            if (gViuPowerDownPin[i] != CAM_GPIO_INVALID_PIN)
              {
                if (gpio_direction_output(gViuPowerDownPin[i], 1))
                {
                  lRetVal = CAM_DRV_FAILURE;
                  break;
                }
              } // if gpio pin is valid
            }

            if (lRetVal == CAM_DRV_SUCCESS)
            {
              // print success message
              printk("CGD: ready.\n");
            }
            else
            {
              printk("Set output for reset/power down pin fail\n");
            }
          }
        }
      }
    }
  }  
  
  // init error processing
  if( lRetVal != CAM_DRV_SUCCESS )
  {
      printk("CGD: Error during insmod. Terminating ...\n");
      vdb_LogStop();
  } // if some failure
  return lRetVal;
} // CamInit()

//****************************************************************************

// 
// Cleanup - unregister the appropriate file from /proc 
//
static void __exit CamExit(void)
{
  int32_t lRetVal = 0;
  // 
  // Unregister the device 
  //
  VDB_LOG_NOTE("Removing cam character device driver module.\n");
  cdev_del(&sCamCdev);
  device_destroy(sCamDriverClass, SCamDevno);
  class_destroy(sCamDriverClass);
  unregister_chrdev_region(SCamDevno, 1);

  // relase resources
  lRetVal = CAM_DRV_Close();

  vdb_LogStop();
  
  lRetVal = printk("CGD: exited.\n");
} // CamExit()

//****************************************************************************

module_init(CamInit);
module_exit(CamExit);

//****************************************************************************

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Babinec <tomas.babinec <at> nxp.com>");
MODULE_DESCRIPTION("Camera generic NXP driver.");
MODULE_ALIAS("cam-generic");
