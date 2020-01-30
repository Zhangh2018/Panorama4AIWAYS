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
 * INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file     fdma_core.c
 * \brief    init/exit functionality of fDMA driver
 * \author   Tomas Babinec
 * \version  0.1
 * \date     2.4.2014
 * \note
 ****************************************************************************/
#include <linux/uaccess.h>  // for get_user and put_user
#include <linux/signalfd.h> // for send_sig_info
#include <linux/kernel.h>	// We're doing kernel work
#include <linux/module.h>	// Specifically, a module
#include <linux/moduleparam.h>  // for cmd line arguments
#include <linux/device.h> // for class_create
#include <linux/init.h>
#include <linux/slab.h>		// for kmalloc

#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>		// Char device structure
#include <linux/mutex.h>	// for semaphore/mutex

#include "fdma_func.h"
#include "seq_types.h"
#include "os_oal.h"
#include "vdb_log.h"

/*****************************************************************************
*                              CONSTANTS
*****************************************************************************/

#define CLASS_NAME "fdmadrv" ///< name of the device class name
#define DONE_IRQ_IDX 0        ///< FDMA transfer done  IRQ index in DTB node
#define ERR_IRQ_IDX  1        ///< FDMA transfer error IRQ index in DTB node

/*****************************************************************************
*                           EXTERNAL FUNCTION
*****************************************************************************/

extern int64_t FdmaLldcmd(const OAL_LldCmd_t* apCmd);

/*****************************************************************************
*                             LOCAL FUNCTION
*****************************************************************************/

static int32_t FdmaOpen(struct inode *apInode, struct file *apFile);
static int32_t FdmaRelease(struct inode *apInode, struct file *apFile);
static int32_t FdmaDrvProbe(struct platform_device *apDev);
static ssize_t FdmaWrite(struct file* apFile,
                         const char*  apcData,
                         size_t       aSize,
                         loff_t*      apOffset);
static int32_t FdmaMap(struct platform_device *apDev);
static int32_t FdmaDrvRemove(struct platform_device *apDev);
static void SigSend(int32_t aProcId, int32_t aValue);
static int32_t __init FdmaInit(void);
static void __exit FdmaExit(void);

/*****************************************************************************
*                             GLOBAL VARIABLES
*****************************************************************************/

// Fdma driver info structure
static FDMA_DrvInfo_t sFdmaDrvInfo;

/*****************************************************************************
*                              LOCAL VARIABLES
*****************************************************************************/

// internal device open counter
static int32_t sDeviceOpenCnt = 0;

// platform device structure
static struct platform_device *spFdmaPdev;

// char device structure for fDMA module
static struct cdev sFdmaCdev;

// fDMA device number
static int32_t sFdmaDevno;
// major device number
static int32_t sMajor;

// Fdma driver class
static struct class *sFdmaDriverClass = NULL;

// file operations
static struct file_operations sFops =
{
  .write          = FdmaWrite,
  .open           = FdmaOpen,
  .release        = FdmaRelease,
}; // struct file_operations

static struct of_device_id sFdmaOfMatch[] =
{
  {
    .compatible = "fsl,s32v234-fdma",
  },
  {}
}; // module device table

MODULE_DEVICE_TABLE(of, sFdmaOfMatch);

static struct platform_driver sFdmaPlatformDriver =
{
  .probe  = FdmaDrvProbe,
  .remove = FdmaDrvRemove,
  .driver =
  {
    .name           = "fdma",
    .owner          = THIS_MODULE,
    .of_match_table = sFdmaOfMatch,
  },
}; // platform driver structure

static SEQ_FdmaComm_t gSEQ_DRV_FdmaComm;  ///< fdma communication structure

/*****************************************************************************/
/**
* Essential kernel module behaviour definition
*/
static int32_t FdmaOpen(struct inode *apInode, struct file *apFile)
{
  VDB_LOG_NOTE("FDMA open call\n");

  sDeviceOpenCnt++;

  try_module_get(THIS_MODULE);
  return FDMA_DRV_SUCCESS;
} // FdmaOpen()

/*****************************************************************************/
/**
* called whenever a process attempts to close the device file
*/
static int32_t FdmaRelease(struct inode *apInode, struct file *apFile)
{
  sDeviceOpenCnt--;

  module_put(THIS_MODULE);

  return FDMA_DRV_SUCCESS;
} // FdmaRelease()

/****************************************************************************/
/**
* Probe the driver
*/

static int32_t FdmaDrvProbe(struct platform_device *apDev)
{
  int32_t lRet = 0;

  spFdmaPdev = apDev;

  // *** Setup the driver ***
  sFdmaDrvInfo.mpRegs               = NULL;
  sFdmaDrvInfo.mSequencerOn         = SEQ_LESS_MODE;
  sFdmaDrvInfo.mpSEQ_FdmaTdSchedule = NULL;
  sFdmaDrvInfo.mpSigInvoke          = &SigSend;
  sFdmaDrvInfo.mpTdtBasePhyHndl     = NULL;
  sFdmaDrvInfo.mTdtBasePhy          = 0;
  sFdmaDrvInfo.mpSeqComm            = NULL;
  sFdmaDrvInfo.mIrqDone             = 0;
  sFdmaDrvInfo.mIrqErr              = 0;
  // rest of the sFdmaDrvInfo members initialized in FDMA_Setup()

  // attempt to connect to Sequencer module if available
  sFdmaDrvInfo.mpSeqComm = (SEQ_FdmaComm_t*)symbol_get(gSEQ_DRV_FdmaComm);

  lRet = FdmaMap(apDev);
  if(lRet != 0)
  {
    return lRet;
  } // if Fdmamap failed

  sFdmaDrvInfo.mIrqDone = irq_of_parse_and_map(
                                              apDev->dev.of_node,
                                              DONE_IRQ_IDX
                                            );

  sFdmaDrvInfo.mIrqErr  = irq_of_parse_and_map(
                                              apDev->dev.of_node,
                                              ERR_IRQ_IDX
                                            );

  dev_set_drvdata(&apDev->dev, &sFdmaDrvInfo);

  return lRet;
} // FdmaDrvProbe()

/*****************************************************************************/

static ssize_t FdmaWrite(struct file* apFile,
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
         aSize) == 0U)
    {
      int32_t lRes = 0;
      lRes = (ssize_t)FdmaLldcmd(&lCmd);
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
} // FdmaWrite()

/*****************************************************************************/

static int32_t FdmaMap(struct platform_device *apDev)
{
  struct resource *lpRes      = NULL;
  void            *lpFdmaRegs = NULL;

  /* map register space */
  lpRes = platform_get_resource(apDev, IORESOURCE_MEM, 0);

  if (NULL == lpRes)
  {
    dev_err(&apDev->dev, "could not get FDMA regs resource\n");
    return -ENODEV;
  }

  lpFdmaRegs = devm_ioremap_resource(&apDev->dev, lpRes);

  if (IS_ERR(lpFdmaRegs))
  {
    dev_err(&apDev->dev, "could not ioremap FDMA regs resource\n");
    return PTR_ERR(lpFdmaRegs);
  }
  sFdmaDrvInfo.mpRegs = (FDMA_Regs_t*) lpFdmaRegs;

  return 0;
} // FdmaMap(struct platform_device *apDev)

/****************************************************************************/

static int32_t FdmaDrvRemove(struct platform_device *apDev)
{
  if(sFdmaDrvInfo.mpSeqComm != NULL)
  {
    symbol_put(gSEQ_DRV_FdmaComm);
  }// if seq connected

  sFdmaDrvInfo.mpRegs               = NULL;
  sFdmaDrvInfo.mSequencerOn         = SEQ_LESS_MODE;
  sFdmaDrvInfo.mpSEQ_FdmaTdSchedule = NULL;
  sFdmaDrvInfo.mpSigInvoke          = NULL;
  sFdmaDrvInfo.mpTdtBasePhyHndl     = NULL;
  sFdmaDrvInfo.mTdtBasePhy          = 0;
  sFdmaDrvInfo.mpSeqComm            = NULL;
  sFdmaDrvInfo.mIrqDone             = 0;
  sFdmaDrvInfo.mIrqErr              = 0;

  return 0;
} // FdmaDrvRemove()

/****************************************************************************/

static void SigSend(int32_t aProcId, int32_t aValue)
{
  struct  siginfo info;
  struct  pid *pid_struct;
  struct  task_struct *t;

  // send the signal to user space
  (void)memset(&info, 0u, sizeof(struct siginfo));
  info.si_signo = FDMA_EVENT_SIGNAL;
  info.si_code  = SI_QUEUE;
  info.si_int   = aValue;
  rcu_read_lock();
  pid_struct    = find_get_pid(aProcId);
  t = pid_task(pid_struct,PIDTYPE_PID);

  if(t == NULL)
  {
    printk("FDMA Done IRQ handler - No such pid %d.\n", aProcId);
    rcu_read_unlock();
  } // if no such pid
  else
  {
    int32_t lRet;
    rcu_read_unlock();
    lRet = send_sig_info(FDMA_EVENT_SIGNAL, &info, t);    //send the signal

    if (lRet < (int32_t)0)
    {
      printk("FDMA IRQ handler - Error sending signal for DONE event.\n");
    } // if sending signal failed
  } // else if no such pid
} // SigSend()

/*****************************************************************************/
/*
* Initialize the module - Register the character device
*/
static int32_t __init FdmaInit(void)
{
  int32_t lRetVal = 0;

  // enable logging
  vdb_LogInit();

  if(platform_driver_register(&sFdmaPlatformDriver) != 0)
  {
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: platform driver registration failed.\n");
    goto fdma_pd_failed;
  } // if platform driver registration failed

  if(FDMA_Setup(&sFdmaDrvInfo) != FDMA_DRV_SUCCESS)
  {
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: driver setup failed.\n");
    goto fdma_setup_failed;
  } // if driver setup failed

  // *** initialize device file ***

  // get device number
  if (alloc_chrdev_region(&sFdmaDevno, 0, 1, FDMA_DEVICE_NAME) < 0)
  {
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: device number allocation failed.\n");
    goto device_number_allocation_failed;
  } // if alloc char dev number failed

  sFdmaDriverClass = class_create(THIS_MODULE, CLASS_NAME);
  if(IS_ERR(sFdmaDriverClass))
  {
    sFdmaDriverClass = NULL;
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: Driver class creation failed.\n");
    goto driver_class_cration_failed;
  } // if fdma driver class creation failed

  if (IS_ERR(device_create( sFdmaDriverClass,
                            NULL,
                            sFdmaDevno,
                            NULL,
                            FDMA_DEVICE_NAME)))
  {
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: Driver device creation failed.\n");
    goto device_creation_failed;
  } // if device creation failed

  cdev_init(&sFdmaCdev, &sFops);
  sFdmaCdev.owner = THIS_MODULE;
  sFdmaCdev.ops   = &sFops;

  if(cdev_add(&sFdmaCdev, sFdmaDevno, 1) < 0)
  {
    lRetVal = FDMA_DRV_FAILURE;
    printk("FDMA: Device registration failed.\n");
    goto registration_failed;
  } // if device registration failed

  sMajor = MAJOR(sFdmaDevno);

  // print success message
  printk("FDMA: driver ready in Sequencer-%s mode.\n",
         (sFdmaDrvInfo.mSequencerOn==SEQ_BASED_MODE)?"based":"less"
        );

  // init error processing
  if( lRetVal != FDMA_DRV_SUCCESS)
  {
    registration_failed:
      device_destroy(sFdmaDriverClass, sFdmaDevno);
    device_creation_failed:
      class_destroy(sFdmaDriverClass);
    driver_class_cration_failed:
      unregister_chrdev_region(sFdmaDevno, 1);
    device_number_allocation_failed:
      FDMA_Close();
    fdma_setup_failed:
      platform_driver_unregister(&sFdmaPlatformDriver);
    fdma_pd_failed:
      printk("FDMA: Error during insmod. Terminating ...\n");
      vdb_LogStop();
  } // if some failure

  return lRetVal;
} // FdmaInit()

/*****************************************************************************/
/*
* Cleanup - unregister the appropriate file from /proc
*/
static void __exit FdmaExit(void)
{
  //
  // Unregister the device
  //
  cdev_del(&sFdmaCdev);
  device_destroy(sFdmaDriverClass, sFdmaDevno);
  class_destroy(sFdmaDriverClass);
  unregister_chrdev_region(sFdmaDevno, 1);

  // relase resources
  FDMA_Close();

  // *** disconnect from Sequencer module ***
  if(sFdmaDrvInfo.mSequencerOn == SEQ_BASED_MODE)
  {
    sFdmaDrvInfo.mSequencerOn = SEQ_LESS_MODE;

    sFdmaDrvInfo.mpSeqComm = NULL;
    symbol_put(gSEQ_DRV_FdmaComm);
  } // if Sequencer on

  platform_driver_unregister(&sFdmaPlatformDriver);

  vdb_LogStop();

  printk("FDMA: driver exited.\n");
} // FdmaExit()

/*****************************************************************************/

#if 1
  module_init(FdmaInit);
  module_exit(FdmaExit);
#else
  module_platform_driver(sFdmaPlatformDriver);
#endif

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Babinec <tomas.babinec <at> nxp.com>");
MODULE_DESCRIPTION("fDMA NXP driver.");
MODULE_ALIAS("fdma");
