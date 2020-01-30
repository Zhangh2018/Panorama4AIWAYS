/*
 * Copyright (c) 2016 Freescale Semiconductor
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


/*==============================================================================
Revision History:
                        Modification      Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Cristian Tomescu        03-Jan-2016                   Init version
Xuyen Dao               05-Aug-2016        VSDK-251   Update template
Tomas Babinec           15-Sep-2016        VSDK-325   Modified include files
Cristian Tomescu        04-Nov-2016        VSDK-488   Splitting the core driver
Cristian Tomescu        15-Nov-2016        VSDK-495   Remove the 'fsl'
Cristian Tomescu        17-Jan-2017        VSDK-694   Clock control improvement
Cristian Tomescu        08-May-2017        VSDK-560   Fix MISRA violations
Tomas Babinec           04-Jan-2018        VSDK-1624  LLD CMD
==============================================================================*/

#ifdef __cplusplus
extern "C"{
#endif


/*==============================================================================
*                                 INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==============================================================================*/
/**
* @file            viulite_linux.c
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/pm_runtime.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>
#include <linux/cdev.h>
#include <asm/current.h>
#include <asm/segment.h>
#include <linux/uaccess.h>

#include "os_oal.h"
#include "viulite_linux.h"
#include "viulite_core.h"

#include "vdb_log.h"
/*==============================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==============================================================================*/


/*==============================================================================
*                               LOCAL MACROS
==============================================================================*/
#define DRIVER_NAME    "viulite"

/* number of VIULite units */
#define VIULITE_DEV_COUNT    2

/**********************************************************
 * Macros for tracing
 **********************************************************/
#define VIULite_INFO    KERN_INFO
#define LOG_TRACE__   0

#if LOG_TRACE__
    #define TRACE_VIU__() printk(VIULite_INFO "[ VIU ] %s\n", __func__)
#else
    #define TRACE_VIU__()
#endif


/*==============================================================================
*                       STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
/**
* @brief          Structure viulite data.
* @details        Structure viulite data.
* @pre
* @post
*
*/
struct viulite_data
{
  struct cdev   chdev;
  dev_t         dev_num;
  VIU_Registers_t    *reg_base;
  uint32_t      irq;
  struct clk    *viu_clk;
  struct clk    *per_clk;
  atomic_t      access;
}; /* viulite_data */

/*==============================================================================
*                                   LOCAL MACROS
==============================================================================*/


/*==============================================================================
*                                  LOCAL CONSTANTS
==============================================================================*/


/*==============================================================================
*                                  LOCAL VARIABLES
==============================================================================*/
static struct class    *viulite_class;
static dev_t           viulite_devn; /*__u32 */
static uint8_t         viu_devices;

/*==============================================================================
*                                 GLOBAL CONSTANTS
==============================================================================*/
static int8_t *device_name[VIULITE_DEV_COUNT] =
{
  "viulite0",
  "viulite1"
};

/*==============================================================================
*                                 GLOBAL VARIABLES
==============================================================================*/


/*==============================================================================
*                                LOCAL FUNCTION PROTOTYPES
==============================================================================*/
static int32_t viulite_remove(struct platform_device *const pdev);
static int32_t viulite_probe(struct platform_device *pdev);
static int32_t viulite_open(struct inode *const inod, struct file *const pfile);
static irqreturn_t viulite_intr(int32_t irq, void * dev_id);

/*==============================================================================
*                                LOCAL FUNCTIONS
==============================================================================*/
static dev_t viulite_dev_create(const uint8_t viu_id);

/*============================================================================*/
/**
* @brief          The function viulite runtime suspend
* @details        The function viulite runtime suspend
*
* @param[in]      pdev      struct platform device
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
static int32_t viulite_runtime_suspend(struct device *const dev)
{
  const struct viulite_data *const viu_data = dev_get_drvdata(dev);
  clk_disable_unprepare(viu_data->viu_clk);
  clk_disable_unprepare(viu_data->per_clk);

  return 0;
} /* viulite_runtime_suspend */

/*============================================================================*/
/**
* @brief          The function viulite close
* @details        The function viulite close
*
* @param[in]      inod      struct inode
* @param[in]      pfile     file close
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
static int32_t viulite_close(struct inode *const inod, struct file *const pfile)
{
  TRACE_VIU__();
  return 0;
} /* viulite_close */

/*============================================================================*/
/**
* @brief          The function viulite runtime resume
* @details        The function viulite runtime resume
*
* @param[in]      pdev      struct platform device
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
static int32_t viulite_runtime_resume(struct device *const dev)
{
  struct viulite_data *const viu_data = dev_get_drvdata(dev);
  clk_prepare_enable(viu_data->viu_clk);
  clk_prepare_enable(viu_data->per_clk);

  return 0;
} /* viulite_runtime_resume */

/*==============================================================================
*                                       GLOBAL FUNCTIONS
==============================================================================*/
/*============================================================================*/
/**
* @brief          The function manipulates the underlying device parameters.
* @details        The function manipulates the underlying device parameters of
*                 special files. In particular, many operating characteristics
*                 of character special files (e.g., terminals) may be controlled
*                 with viulite_lldcmd() requests.
*
* @param[in]      pcmd    pointer to command structure
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
static int64_t viulite_lldcmd(struct file *const pfile, OAL_LldCmd_t* pcmd)
{
  int32_t ret = 0;

  const struct inode *const pnode = pfile->f_inode;
  const struct viulite_data *const viu_data =
      container_of(pnode->i_cdev, struct viulite_data, chdev);
  VIU_Registers_t *const viulite_baseloc = viu_data->reg_base;
  uint32_t irq = (uint32_t)(viu_data->irq);

  TRACE_VIU__();

  switch (pcmd->mCommandId)
  {
    case VIULITE_LLDCMD_DMA_CONFIG:
    {
      DMA_CONFIG dma_configuration;

      /* copy data from user space */
      if(copy_from_user(&dma_configuration, (DMA_CONFIG *)pcmd->mArg,
            sizeof(dma_configuration)) != 0U)
      {
        ret = -EIO;
      }

      viulite_dma_config(viulite_baseloc, &dma_configuration);
    }
    break;

    case VIULITE_LLDCMD_DMA_START:
      disable_irq(irq);
      viulite_dma_start(viulite_baseloc);
      enable_irq(irq);
      break;

    case VIULITE_LLDCMD_DMA_STOP:
      viulite_dma_stop(viulite_baseloc);
      break;

    case VIULITE_LLDCMD_DMA_GET_STATUS:
    {
      VIU_BOOL dma_st;

      viulite_dma_getstatus(viulite_baseloc, &dma_st);

      /* copy data to user space */
      if( copy_to_user((VIU_BOOL *)pcmd->mArg, &dma_st, sizeof(dma_st)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_SW_RESET:
      viulite_sw_reset(viulite_baseloc);
      break;

    case VIULITE_LLDCMD_SET_VIDEOIN_FORMAT:
    {
      VIU_INPUT_FORMAT in_format;

      /* copy data from user space */
      if(copy_from_user(&in_format, (VIU_INPUT_FORMAT *)pcmd->mArg,
                sizeof(in_format)) != 0U)
      {
        ret = -EIO;
      }

      viulite_set_videoinputformat(viulite_baseloc, &in_format);
    }
    break;

    case VIULITE_LLDCMD_GET_VIDEOIN_FORMAT:
    {
      VIU_INPUT_FORMAT in_format;

      viulite_get_videoinputformat(viulite_baseloc, &in_format);

      /* copy data to user space */
      if(copy_to_user((VIU_INPUT_FORMAT *)pcmd->mArg, &in_format,
              sizeof(in_format)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_SET_DATA_INTERFACE:
    {
      VIU_DATA_INTERFACE data_interface;

      /* copy data from user space */
      if(copy_from_user(&data_interface, (VIU_DATA_INTERFACE *)pcmd->mArg,
              sizeof(data_interface)) != 0U)
      {
        ret = -EIO;
      }

      viulite_set_datainterface(viulite_baseloc, &data_interface);

    }
    break;

    case VIULITE_LLDCMD_GET_DATA_INTERFACE:
    {
      VIU_DATA_INTERFACE data_interface;

      viulite_get_datainterface(viulite_baseloc, &data_interface);

      /* copy data to user space */
      if(copy_to_user((VIU_DATA_INTERFACE *)pcmd->mArg, &data_interface,
              sizeof(data_interface)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_CONFIG_IRQS:
    {
      uint8_t irq_mask;

      /* copy data from user space */
      if(copy_from_user(&irq_mask, (uint8_t *)pcmd->mArg,
              sizeof(irq_mask)) != 0U)
      {
        ret = -EIO;
      }

      viulite_enable_irqs(viulite_baseloc, irq_mask);
    }
    break;

    case VIULITE_LLDCMD_EN_ITU_ERRCODE:
    {
      VIU_BOOL itu_errset;

      /* copy data from user space */
      if(copy_from_user(&itu_errset, (VIU_BOOL *)pcmd->mArg,
              sizeof(itu_errset)) != 0U)
      {
        ret = -EIO;
      }

      viulite_enable_ituerror(viulite_baseloc, itu_errset);
    }
    break;

    case VIULITE_LLDCMD_RESET_IRQSTATUS:
    {
      uint8_t irq_status;

      /* copy data from user space */
      if(copy_from_user(&irq_status, (uint8_t *)pcmd->mArg,
              sizeof(irq_status)) != 0U)
      {
        ret = -EIO;
      }

      viulite_reset_irqstatus(viulite_baseloc, irq_status);
    }
    break;

    case VIULITE_LLDCMD_GET_IRQSTATUS:
    {
      uint8_t irq_status;

      viulite_get_irqstatus(viulite_baseloc, &irq_status);

      /* copy data to user space */
      if(copy_to_user((uint8_t *)pcmd->mArg, &irq_status,
              sizeof(irq_status)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_GET_ITU_ERRCODE:
    {
      VIU_ITU_ERR itu_err;

      viulite_get_ituerror(viulite_baseloc, &itu_err);

      /* copy data to user space */
      if(copy_to_user((VIU_ITU_ERR *)pcmd->mArg, &itu_err,
              sizeof(itu_err)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_GET_SYNC:
    {
      VIU_SYNC_STATUS sync_st;

      viulite_get_syncsignals(viulite_baseloc, &sync_st);

      /* copy data to user space */
      if(copy_to_user((VIU_SYNC_STATUS *)pcmd->mArg, &sync_st,
              sizeof(sync_st)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_GET_FIELDNUM:
    {
      uint8_t field_num;

      viulite_get_fieldnum(viulite_baseloc, &field_num);

      /* copy data to user space */
      if(copy_to_user((uint8_t *)pcmd->mArg, &field_num,
              sizeof(field_num)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_GET_FRAME_SIZE:
    {
      VIU_FRAME_SIZE frame_size;

      viulite_get_framesize(viulite_baseloc, &frame_size);

      /* copy data to user space */
      if(copy_to_user((VIU_FRAME_SIZE *)pcmd->mArg, &frame_size,
              sizeof(frame_size)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    case VIULITE_LLDCMD_SET_CLIPPING:
    {
      VIU_IMAGE_PARAMS image_params;

      /* copy data from user space */
      if((uint32_t)copy_from_user(&image_params, (VIU_IMAGE_PARAMS *)pcmd->mArg,
              sizeof(image_params)) != 0U)
      {
        ret = -EIO;
      }

      viulite_set_clippingdata(viulite_baseloc, &image_params);
    }
    break;

    case VIULITE_LLDCMD_GET_CLIPPING:
    {
      VIU_IMAGE_PARAMS image_params;

      viulite_get_clippingdata(viulite_baseloc, &image_params);

      /* copy data to user space */
      if((uint32_t)copy_to_user((VIU_IMAGE_PARAMS *)pcmd->mArg, &image_params,
              sizeof(image_params)) != 0U)
      {
        ret = -EIO;
      }
    }
    break;

    default:
      // Nothing to do
      VDB_LOG_WARNING("Nothing to do! \n");
      break;
  } /* switch lldcmd id */

  return (int64_t)ret;
} /* viulite_lldcmd */

/*============================================================================*/
/**
* @brief          The function viulite interrupt
* @details        The function viulite interrupt
*
* @param[in]      irq        Interrupt
* @param[in]      dev_id     Id
* @return         irqreturn_t
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/
static irqreturn_t viulite_intr(int32_t irq, void *dev_id)
{
  const struct viulite_data *viu_data = (struct viulite_data *)dev_id;
  VIU_Registers_t *viulite_regbase = viu_data->reg_base;

  uint32_t  int_status, reg_value;
  int_status = viulite_regbase->INTR;

  if (0U != (int_status & (INTR_VSYNC_MASK | INTR_VSYNC_EN)))
  {
      /* start a new frame transfer */
      reg_value = viulite_regbase->SCR;
      reg_value |= SCR_DMA_ACT_MASK;
      viulite_regbase->SCR = reg_value;

      int_status |= INTR_VSYNC_MASK;
  }

  if (0U != (int_status & (INTR_DMA_END_MASK | INTR_DMA_END_EN)))
  {
      int_status |= INTR_DMA_END_MASK;
  }

  if (0U != (int_status & (INTR_ERR_MASK | INTR_ERR_EN)))
  {
      int_status |= INTR_ERR_MASK;
  }

  if (0U != (int_status & (INTR_HSYNC_MASK | INTR_HSYNC_EN)))
  {
      int_status |= INTR_HSYNC_MASK;
  }

  if (0U != (int_status & (INTR_LINE_END_MASK | INTR_LINE_END_EN)))
  {
      int_status |= INTR_LINE_END_MASK;
  }

  if (0U != (int_status & (INTR_FRAME_END_MASK | INTR_FRAME_END_EN)))
  {
      int_status |= INTR_FRAME_END_MASK;
  }

  if (0U != (int_status & (INTR_FIELD_MASK | INTR_FIELD_EN)))
  {
      int_status |= INTR_FIELD_MASK;
  }

  if (0U != (int_status & (INTR_VSTART_MASK | INTR_VSTART_EN)))
  {
      int_status |= INTR_VSTART_MASK;
  }

  /* clear all pending irqs */
  viulite_regbase->INTR = int_status;

  return IRQ_HANDLED;
} /* viulite_intr */

/*============================================================================*/
/**
* @brief          The function viulite open
* @details        The function viulite open
*
* @param[in]      inod      struct inode
* @param[in]      pfile     file open
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
static int32_t viulite_open(struct inode *const inod, struct file *const pfile)
{
  struct viulite_data *viu_data;

  TRACE_VIU__();

  viu_data = container_of(inod->i_cdev, struct viulite_data, chdev);
  pfile->private_data = viu_data;

  return 0;
} /* viulite_open */

/*============================================================================*/
/**
* @brief          The function viulite write
* @details        Write syscall handler for viulite driver
*
* @param[in]      pfile      file struct pointer
* @param[in]      pdata      pointer to data buffer
* @param[in]      size       number of bytes in data buffer
* @param[in]      poffset    offset inside the data buffer
* @return         number of bytes read
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/
static ssize_t viulite_write(struct file* pfile,
                             const char*  pdata,
                             size_t       size,
                             loff_t*      poffset)
{
  ssize_t lRet = size;
  if(size == sizeof(OAL_LldCmd_t))
  {
    OAL_LldCmd_t lCmd = {1};
    if(copy_from_user(
         &lCmd,
         pdata,
         size) == 0U)
    {
      int32_t lRes = 0;
      lRes = (ssize_t)viulite_lldcmd(pfile, &lCmd);
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
} // viulite_write()

/*============================================================================*/
// STRUCT operations
/*============================================================================*/
static struct file_operations viulite_fops =
{
  .owner   = THIS_MODULE,
  .open    = viulite_open,
  .release = viulite_close,
  .write   = viulite_write,
};

/*============================================================================*/
/**
* @brief          The function viulite dev create
* @details        The function viulite dev create
*
* @param[in]      viu_id    viulite id
* @return         dev_t
* @api
* @isr
*
* @pre
*
* @post
*
* @note
*/
static dev_t viulite_dev_create(const uint8_t viu_id)
{
  int32_t ret = 0;

  TRACE_VIU__();

  if (NULL == viulite_class)
  {
    /* Alloc MAJOR number for the character and
    * the 1st minor device in dev
    */
    if( alloc_chrdev_region(&viulite_devn, 0,
                  VIULITE_DEV_COUNT, DRIVER_NAME) < 0)
    {
      printk(VIULite_INFO "alloc_region failed: %d\n", ret);
    }

    viulite_class = class_create(THIS_MODULE, DRIVER_NAME);

    if (NULL == viulite_class)
    {
      printk(VIULite_INFO "class_create failed %d\n", ret);
      ret = -1;
    }
  }

  /* Device is created and registered in sysfs */

  if (NULL == (device_create(viulite_class, NULL,
          viulite_devn, NULL, device_name[viu_id])))
  {
    printk(VIULite_INFO "device_create 0 failed %d\n", ret);
    ret = -1;
  }
  else
  {
    ret = viulite_devn;
  }

  return ret;
} /* viulite_dev_create */

/*============================================================================*/
/**
* @brief          The function viulite probe
* @details        The function viulite probe
*
* @param[in]      pdev      struct platform device
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
static int32_t viulite_probe(struct platform_device *pdev)
{
  struct resource *res;
  struct device_node *const viu_n = pdev->dev.of_node;
  struct viulite_data *viudata;
  void __iomem *viulite_reg_base;
  struct clk *viulite_clk;
  uint32_t viulite_irq;
  dev_t devn_temp;

  uint8_t viu_num;
  int32_t ret = 0;

  TRACE_VIU__();

  ret = of_alias_get_id(viu_n, "viu");

  if (ret < 0)
  {
    dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
  }
  else
  {
    viu_num = (uint8_t)ret;

    /* create device and register it in /dev through sysfs */
    devn_temp = (dev_t)viulite_dev_create(viu_num);

    /* map register space */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if (NULL == res)
    {
      dev_err(&pdev->dev, "could not get memory IO resource\n");
      ret = -ENODEV;
    }
    else
    {
      viudata = devm_kzalloc(&pdev->dev,
            sizeof(struct viulite_data), GFP_KERNEL);

      /* setup file operations */
      cdev_init(&(viudata->chdev), &viulite_fops);
      ret = cdev_add(&(viudata->chdev), viulite_devn, 1);

      if (ret < 0)
      {
        printk(VIULite_INFO "cdev_add failed with %d\n", ret);
      }
      else
      {
        viulite_reg_base = devm_ioremap_resource(&pdev->dev, res);

        if (IS_ERR(viulite_reg_base))
        {
          dev_err(&pdev->dev, "could not ioremap resource\n");
          ret = -1;
        }
        else
        {
          viudata->reg_base = (VIU_Registers_t *)viulite_reg_base;
          viudata->dev_num = devn_temp;

          /* enable clocks for VIU */
          viulite_clk = devm_clk_get(&pdev->dev, device_name[viu_num]);

          if (IS_ERR(viulite_clk))
          {
            ret = -1;
            dev_err(&pdev->dev, "could not get viu clock\n");
          }
          else
          {
            clk_prepare_enable(viulite_clk);
            viudata->viu_clk = viulite_clk;
            viulite_clk = devm_clk_get(&pdev->dev, "per");

            if (IS_ERR(viulite_clk))
            {
              ret = -1;
              dev_err(&pdev->dev, "could not get per clock\n");
            }
            else
            {
              clk_prepare_enable(viulite_clk);
              viudata->per_clk = viulite_clk;
              ret = 0;
            }
          }

          if (0 == ret)
          {
            viulite_irq = (uint32_t)platform_get_irq(pdev, 0);

            if (viulite_irq == (uint32_t)0)
            {
                dev_err(&pdev->dev, "Error while mapping the irq\n");
                ret = -1;
            }
            else
            {
              viudata->irq = viulite_irq;

              /* install interrupt handler */

              if (0 > request_irq(viudata->irq, viulite_intr, 0,
                         device_name[viu_num], (void *)viudata)) {
                dev_err(&pdev->dev, "Request VIULite IRQ failed.\n");
                ret = -1;
              }
              else
              {
                dev_set_drvdata(&pdev->dev, viudata);
                viulite_devn++;
                viu_devices++;
                ret = 0;
              }
            }
          }
        }
      }
    }
  }
  TRACE_VIU__();
  return ret;
} /* viulite_probe */

/*============================================================================*/
/**
* @brief          The function viulite remove
* @details        The function viulite remove
*
* @param[in]      pdev      struct platform device
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
static int32_t viulite_remove(struct platform_device *const pdev)
{
  struct viulite_data *v_data = dev_get_drvdata(&pdev->dev);

  TRACE_VIU__();

  free_irq(v_data->irq, (void *)v_data);

  clk_disable_unprepare(v_data->viu_clk);
  clk_disable_unprepare(v_data->per_clk);

  cdev_del(&v_data->chdev);
  device_destroy(viulite_class, v_data->dev_num);

  viu_devices--;

  if ((uint8_t)0 == viu_devices)
  {
      class_destroy(viulite_class);
      unregister_chrdev_region(v_data->dev_num, 2);
  }

  return 0;
} /* viulite_remove */

/*==============================================================================
*                       LOCAL VARIABLES
==============================================================================*/
/**
* @brief          Structure of device id
* @details        DCU Linux DTB query related operations
* @pre
* @post
*
*/
static struct of_device_id viulite_dt_ids[] =
{
  {
    .compatible = "fsl,s32v234-viulite0",
  },
  {
    .compatible = "fsl,s32v234-viulite1",
  },
  {}
};

/**
* @brief          Struct dev_pm_ops
* @details        Struct dev_pm_ops
* @pre
* @post
*
*/
static struct dev_pm_ops viulite_pm_ops =
{
    SET_RUNTIME_PM_OPS(viulite_runtime_suspend,
            viulite_runtime_resume, NULL)
}; /* dev_pm_ops viulite_pm_ops */

/**
* @brief          Struct platform_driver
* @details        Struct platform_driver
* @pre
* @post
*
*/
static struct platform_driver fsl_viulite_driver =
{
  .driver =
  {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = viulite_dt_ids,
    .pm = &viulite_pm_ops,
  },
  .probe = viulite_probe,
  .remove = viulite_remove,
};

module_platform_driver(fsl_viulite_driver);

MODULE_AUTHOR("Tomescu Cristian Corneliu");
MODULE_DESCRIPTION("VIULite driver");
MODULE_LICENSE("GPL");

#ifdef __cplusplus
}
#endif


/*******************************************************************************
 * EOF
 ******************************************************************************/

/** @} */
