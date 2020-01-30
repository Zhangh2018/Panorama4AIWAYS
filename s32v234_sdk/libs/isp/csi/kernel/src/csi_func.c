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

/*==============================================================================
Revision History:
                        Modification       Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Tomas Babinec           28-Apr-2016                   Init version
Cuong Nguyen (nxa14430) 03-Nov-2017        VSDK-1494  Add check sppCsiDrvInfo is
                                                        different than NULL
Nguyen Tri Hai(B51945)  1-Dec-2017        VSDK-1225   Resolve Misra violations
==============================================================================*/

/*==============================================================================
*                                 INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==============================================================================*/
/**
* @file            csi_func.c
*/

#if !defined(__STANDALONE__)
  #include <linux/kernel.h>     // We're doing kernel work
  #include <linux/module.h>     // Specifically, a module
  #include <linux/interrupt.h>  // For interrupts
  #include <asm/siginfo.h>      // siginfo
  #include <linux/sched.h>      // send_sig_info
  #include <linux/types.h>
  #include <linux/delay.h>      // for msleep
  #include <linux/i2c.h>
  #include <asm/io.h>           // for ioremap
  #include <linux/of_platform.h>
  #include <linux/signalfd.h> // for send_sig_info
  #include <linux/platform_device.h>
#else
  #include <stdio.h>
  #include <unistd.h>
  #include <stdint.h>
  #include <stddef.h>
  #include <string.h>
  #include <math.h>
  #include "oal_irq.h"
  #include "i2c.h"

  #include "seq_sa_kernel_api.h"

  unsigned long get_uptime_microS(void);

  #define msleep(_val) \
        {\
          unsigned long _lStart = get_uptime_microS();\
          while((get_uptime_microS() - _lStart) < _val*1000);\
        }
#endif // else from if !defined(__STANDALONE__)

#include "config.h"
#include "csi.h"
#include "csi_func.h"
#include "vdb_log.h"

/*==============================================================================
*                            LOCAL CONSTANTS
==============================================================================*/
#ifdef CAM_MAXIM
  #define CAM_CNT CAM_MAXIM
  #define CSI_DATA_FREQ 369U // data frequency (all lanes) [MHz] - OV
#else
  #define CAM_CNT 1
  #define CSI_DATA_FREQ 594U // data frequency (all lanes) [MHz] - SONY
#endif // #ifdef CAM_MAXIM

/*==============================================================================
*                             LOCAL MACROS
==============================================================================*/

/*==============================================================================
*                       GLOBAL VARIABLES
==============================================================================*/

/*==============================================================================
*                       LOCAL VARIABLES
==============================================================================*/

static void(*spIrqHandler[2])(enum CSI_IDX aCsiIdx, uint32_t aIrqType) = {NULL,NULL};
static CSI_DrvInfo_t *sppCsiDrvInfo[CSI_IDX_CNT] = {NULL,NULL};

/*==============================================================================
*                    External function declarations
==============================================================================*/

#ifdef __STANDALONE__
  extern void CsiSigHandler(int aSigNum, int aValue);
#endif //#ifdef __STANDALONE__

/*==============================================================================
*                        LOCAL FUNCTION PROTOTYPES
==============================================================================*/

#ifdef __STANDALONE__
  static int32_t CsiRegsMap(void);
#endif //#ifdef __STANDALONE__

static void CsiRegsUnmap(void);

static int32_t Siul2Init(enum CSI_IDX aCsiIdx);

static int32_t CSI_DRV_IrqsRequest(void);
static void CSI_DRV_IrqsFree(void);

#ifdef __STANDALONE__
  static void CSI0_IrqHandler(void);
  static void CSI0_ErrIrqHandler(void);
  static void CSI1_IrqHandler(void);
  static void CSI1_ErrIrqHandler(void);
#else // ifdef __STANDALONE__
  static irqreturn_t CSI0_IrqHandler(int32_t aIrq, void *apDev_id);
  static irqreturn_t CSI0_ErrIrqHandler(int32_t aIrq, void *apDev_id);
  static irqreturn_t CSI1_IrqHandler(int32_t aIrq, void *apDev_id);
  static irqreturn_t CSI1_ErrIrqHandler(int32_t aIrq, void *apDev_id);
#endif // else from #ifdef __STANDALONE__

/**
* @brief         Sends a signal to registered user process.
* @details       Sends a signal to registered user process.
*
* @param         aPid process PID
* @param         param aValue value to be passed with the signal
*
* @return        int32_t
*
* @note
*/

static int32_t Sig2UsrSend(pid_t aPid, int32_t aValue);

/*==============================================================================
*                                LOCAL FUNCTIONS
==============================================================================*/
#if defined(__KERNEL__) || defined(__STANDALONE__)
/**
* @brief         Setup CSI
* @details       Setup CSI, calls function for data initializing,
*                requestst irqs
*
* @return        int32_t
*
* @implements    CSI_DRV_Setup_Activity
*
* @note
*/
int32_t CSI_DRV_Setup(CSI_DrvInfo_t **appCsiDrvInfo)
{
  int32_t   lRet    = CSI_DRV_SUCCESS;
  uint32_t  i       = 0U;
  CSI_IDX_t lCsiIdx = 0U;

  if(appCsiDrvInfo != NULL)
  {
    for(i = 0; i < (uint32_t)CSI_IDX_CNT; i++)
    {
      sppCsiDrvInfo[i] = appCsiDrvInfo[i];

      // CSI IDX is relevant to i
      if(0U == i)
      {
        lCsiIdx = CSI_IDX_0;
      }
      else
      {
        lCsiIdx = CSI_IDX_1;
      }

      // setup pin control for the CSI interfaces
      if(sppCsiDrvInfo[i] != NULL)
      {
        (void)Siul2Init(lCsiIdx);
      } // if enabled
    } // for all possible CSI interface indexes

#ifdef __STANDALONE__
    // initialize pin config
    if(CsiRegsMap() != CSI_DRV_SUCCESS)
    {
      lRet = CSI_DRV_FAILURE;
      goto csi_setup_failed;
    } // if driver data init failed
#endif // #ifdef __STANDALONE__

    // request irqs
    if(CSI_DRV_IrqsRequest() != CSI_DRV_SUCCESS)
    {
      lRet = CSI_DRV_FAILURE;
    } // if failed to map Irqs
  } // if driver info exists
  else
  {
    lRet = CSI_DRV_FAILURE;
  } // if no driver info provided

  if( lRet != CSI_DRV_SUCCESS )
  {
#ifdef __STANDALONE__
  csi_setup_failed:
#endif // #ifdef __STANDALONE__
    for(i = 0; i < (uint32_t)CSI_IDX_CNT; i++)
    {
      sppCsiDrvInfo[i] = NULL;
    } // for all possible CSI interface indexes
  } // if failed

  return lRet;
} // CSI_DRV_Setup()

/*============================================================================*/
/**
* @brief        Close CSI irqs
* @details      Free CSI irqs, unmaps CSI registers
*
* @api
* @isr
*
* @implements CSI_Close_Activity
*
* @note
*/
void CSI_DRV_Close(void)
{
  uint32_t i = 0;

  CSI_DRV_IrqsFree();
  CsiRegsUnmap();

  for(i = 0; i < (uint32_t)CSI_IDX_CNT; i++)
  {
    sppCsiDrvInfo[i] = NULL;
  } // for all possible CSI interface indexes
} // CSI_DRV_Close()

/*============================================================================*/
#ifdef __STANDALONE__
/**
* @brief         Maps CSI
* @details       Maps CSI
*
* @return        int32_t
*
* @implements    CsiRegsMap_Activity
*
*/
static int32_t CsiRegsMap(void)
{
  int32_t lRet = CSI_DRV_SUCCESS;

  sppCsiDrvInfo[CSI_IDX_0]->mpRegs = (CSI_Regs_t*) ioremap(
                                                         (uintptr_t)CSI0_BASE,
                                                         sizeof(CSI_Regs_t)
                                                        );
  if(sppCsiDrvInfo[CSI_IDX_0]->mpRegs == NULL)
  {
    VDB_LOG_ERROR("Failed to map CSI0 registers.\n");
    lRet = CSI_DRV_FAILURE;
  } // if CSI0 registers not mapped
  else
  {
    sppCsiDrvInfo[CSI_IDX_1]->mpRegs = (CSI_Regs_t*) ioremap(
                                                           (uintptr_t)CSI1_BASE,
                                                           sizeof(CSI_Regs_t)
                                                          );
    if(sppCsiDrvInfo[CSI_IDX_1]->mpRegs == NULL)
    {
      VDB_LOG_ERROR("Failed to map CSI1 registers.\n");
      lRet = CSI_DRV_FAILURE;
    } // if CS1 registers not mapped

  } // if CSI2 registers not mapped
  return lRet;
} //CsiRegsMap()

#endif // #ifdef __STANDALONE__

/*============================================================================*/
/**
* @brief         Unmaps CSI Encoder
* @details       UnMaps CSI Encoder
*
* @return        void
*
* @implements    CsiRegsUnmap_Activity
*
*/
static void CsiRegsUnmap(void)
{
#ifdef __STANDALONE__
  if(sppCsiDrvInfo[CSI_IDX_0]->mpRegs)
  {
    iounmap(sppCsiDrvInfo[CSI_IDX_0]->mpRegs);
  }

  if(sppCsiDrvInfo[CSI_IDX_1]->mpRegs)
  {
    iounmap(sppCsiDrvInfo[CSI_IDX_1]->mpRegs);
  }
#endif //#ifdef __STANDALONE__

  sppCsiDrvInfo[CSI_IDX_0]->mpRegs = NULL;
  sppCsiDrvInfo[CSI_IDX_1]->mpRegs = NULL;
} // CsiRegsUnmap()

/*============================================================================*/

/**
* @brief         Inits System Integration Unit Lite2
* @details       Inits System Integration Unit Lite2
*
* @param         CSI_IDX
*
* @return        int32_t
*
*/
static int32_t Siul2Init(enum CSI_IDX aCsiIdx)
{
  int32_t lRet = CSI_DRV_SUCCESS;
  CSI_Siul2Regs_t *lpSiul2Regs = NULL;

  lpSiul2Regs = (CSI_Siul2Regs_t*) ioremap(SIUL2_BASE,
                                           sizeof(CSI_Siul2Regs_t));
  if(lpSiul2Regs == NULL)
  {
    VDB_LOG_ERROR("Failed to map SUIL2 registers.\n");
    lRet = CSI_DRV_FAILURE;
  } // if SUIL2 registers not mapped
  else
  {
    // *** turn on Sony cam ***
    if(aCsiIdx == CSI_IDX_0)
    {
      // wake up camera
      lpSiul2Regs->MSCR[38].B.MUX_MODE = 0;  // GPIO mux mode selected
      lpSiul2Regs->MSCR[38].B.OBE = 1;       // output buffer enabled
      lpSiul2Regs->MSCR[38].B.DSE = 7;       // Drive Strenght

      lpSiul2Regs->GPDO[9].B.PDO_4n2 = 1;
    } // if CSI_IDX_0

    if(aCsiIdx == CSI_IDX_1)
    {
      lpSiul2Regs->MSCR[31].B.MUX_MODE = 0;  // GPIO mux mode selected
      lpSiul2Regs->MSCR[31].B.OBE = 1;       // output buffer enabled
      lpSiul2Regs->MSCR[31].B.DSE = 7;       // Drive Strenght

      lpSiul2Regs->GPDO[7].B.PDO_4n3 = 1;
    } // if CSI_IDX_1

    iounmap(lpSiul2Regs);
  } // if SUIL2 registers not mapped
  return lRet;
} // Siul2Init(enum CSI_IDX aIdx)


/*============================================================================*/
/**
* @brief        CSI Configuration.
* @details      CSI Configuration.
*
* @param        CSI_Cfg_t structure
*
* @implements   CSI_DRV_Config_Activity
*
* @note
*/
void CSI_DRV_Config(const CSI_Cfg_t *apcCsiCfg)
{
  if(apcCsiCfg != NULL)
  {
    CSI_IDX_t  lCsiIdx = CSI_IDX_GET((apcCsiCfg->mIdKey));
    uint32_t   i       = 0;
    uint32_t   lMbps   = 0;
    uint32_t   lLanes  = apcCsiCfg->mLanes;
    uint32_t   lCamCnt = apcCsiCfg->mCamCnt;

    // This API only works when sppCsiDrvInfo is different than NULL
    if(sppCsiDrvInfo[lCsiIdx] != NULL)
    {
#ifndef __STANDALONE__
      msleep(10);
#endif //#ifndef __STANDALONE__

      if(apcCsiCfg->mLanes != 0U)
      {
        lLanes = apcCsiCfg->mLanes;

        // start csi config
        //Two Lanes to Receive MIPI Data
        sppCsiDrvInfo[lCsiIdx]->mpRegs->CONC.B.NULANE = (uint8_t)(lLanes - 1U);
        //Power down disabled
        sppCsiDrvInfo[lCsiIdx]->mpRegs->PHYC.B.PDRX   = 0x0;
        //DPHY clock and data lane 1Gbps
        // sppCsiDrvInfo[lCsiIdx]->mpRegs->PHYC.B.ENPDES = 0x0
     }
// Camera data rate for 2 MIPI CSI lanes coming from datasheet = 297Mbps = SONY
// UI = 3.367ns
// -------------------------------------------------
//         |Minimum   |Typical  | Maximum
// -------------------------------------------------
// THS_SETL|85+6xUI   | 142     | 145+10xUI=179
// -------------------------------------------------
// HSSETL  |(THS_SETL/15)-1| 9    | (THS_SETL/15)-1
// -------------------------------------------------

// Camera data rate for 2 MIPI CSI lanes coming from datasheet = 297Mbps = SONY
// UI = 3.367ns
// -------------------------------------------------
//         |Minimum   |Typical  | Maximum
// -------------------------------------------------
// THS_SETL|85+20=105 | 142     | 145+34=179
// -------------------------------------------------
// HSSETL  |(105/15)-1=6 | 9       | (179/15)-1=11
// -------------------------------------------------

// Camera data rate for 2 MIPI CSI lanes coming from datasheet = 185Mbps = OV
// UI = 5.405ns
// -------------------------------------------------
//         |Minimum   |Typical  | Maximum
// -------------------------------------------------
// THS_SETL|85+20=117 | 158     | 199
// -------------------------------------------------
// HSSETL  |(117/15)-1=7 | 9       | (199/15)-1=12
// -------------------------------------------------
//  lMbps = (apcCsiCfg->mpVc[i].mLineBytes / BYTES_PER_PIXEL * BITS_PER_PIXEL
//          apcCsiCfg->mpVc[i].mFrameHeight * FPS)>>20
//  lMbps = CSI_DATA_FREQ * CAM_CNT

      if(apcCsiCfg->mMbps == 0U)
      {
        if(lLanes != 0U)
        {
          lMbps = CSI_DATA_FREQ * (uint16_t)CAM_CNT / lLanes;
        }
      }
      else
      {
        if(lLanes != 0U)
        {
          lMbps = apcCsiCfg->mMbps * lCamCnt / lLanes;
        }
      }

      if(lMbps >= 1000U)
      {
        sppCsiDrvInfo[lCsiIdx]->mpRegs->PHYC.B.HSEL = 1;
      } // if High speed
      else
      {
        sppCsiDrvInfo[lCsiIdx]->mpRegs->PHYC.B.HSEL = 0;
      } // else from if high speed

      lMbps = HSSETL_COMPUTE(lMbps );
      sppCsiDrvInfo[lCsiIdx]->mpRegs->CLKCS.B.HSSETL  = (uint8_t)lMbps;

      if((lLanes > 4U) || (lLanes == 0U))
      {
        VDB_LOG_ERROR("0 csi lanes requested.\n");
      }
      else
      {
        if(lLanes == 4U)
        {
          sppCsiDrvInfo[lCsiIdx]->mpRegs->LAN3CS.B.D3HSET = (uint8_t)lMbps;
        }

        if(lLanes >= 3U)
        {
          sppCsiDrvInfo[lCsiIdx]->mpRegs->LAN2CS.B.D2HSET = (uint8_t)lMbps;
        }

        if(lLanes >= 2U)
        {
          sppCsiDrvInfo[lCsiIdx]->mpRegs->LAN1CS.B.D1HSET = (uint8_t)lMbps;
        }

        sppCsiDrvInfo[lCsiIdx]->mpRegs->LAN0CS.B.D0HSET = (uint8_t)lMbps;
      }

      for(i = 0; i < (uint32_t)CSI_CHNL_CNT; i++)
      {
        bool condition = CSI_VC_IDX_USED((apcCsiCfg->mIdKey), ((uint16_t)i));

        if(condition != false)
        {
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].SRTPTR.R           =
              apcCsiCfg->mpVc[i].mBufAddr;
          // length of buffer line in bytes (1312 pix, 2bytes/pix => 2624bytes)
          // 8bytes aligned - includes the padding
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].BUFLLEN.B.BUFLLEN  =
              (uint16_t)apcCsiCfg->mpVc[i].mLineStride;
          // length of buffer line in bytes (1312 pix, 2bytes/pix => 2624bytes)
          // without the padding
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].LINLEN.B.LINLEN    =
              (uint16_t)apcCsiCfg->mpVc[i].mLineBytes;
          // number of lines to generate DONE signal to Sequencer
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].LPDI.B.LPDI        =
              (uint8_t)apcCsiCfg->mpVc[i].mDoneLineCnt;
          // number of lines available in SRAM
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].NUMLINE.B.NULINE   =
              (uint16_t)apcCsiCfg->mpVc[i].mLineCnt;
          // number of lines expected per frame
          sppCsiDrvInfo[lCsiIdx]->mpRegs->CH[i].EXPCTDL.B.EXPLINES =
              (uint16_t)apcCsiCfg->mpVc[i].mFrameHeight;
        } // if VC used
      } // for, all possible Virtual Channels

      // apply and remember IRQ masks
      sppCsiDrvInfo[lCsiIdx]->mFrameIrqMask = apcCsiCfg->mFrameIrqMask &
        (CSI_IRQ_ERRLIN_MASK_ALL | CSI_IRQ_FRM_MASK_ALL);
      sppCsiDrvInfo[lCsiIdx]->mpRegs->INTRENVC.R  =
        sppCsiDrvInfo[lCsiIdx]->mFrameIrqMask;

      //Protocol Packet Error Interrupt Enable
      sppCsiDrvInfo[lCsiIdx]->mPPErrIrqMask = apcCsiCfg->mPPErrIrqMask &
                                              CSI_IRQ_ERRPP_MASK_ALL;
      sppCsiDrvInfo[lCsiIdx]->mpRegs->ERPPINTEN.R =
        sppCsiDrvInfo[lCsiIdx]->mPPErrIrqMask;

      //Phy Error Interrupt Enable Register
      sppCsiDrvInfo[lCsiIdx]->mPhyErrIrqMask = apcCsiCfg->mPhyErrIrqMask &
        CSI_IRQ_ERRPHY_MASK_ALL;
      sppCsiDrvInfo[lCsiIdx]->mpRegs->ERPHYIE.R =
        sppCsiDrvInfo[lCsiIdx]->mPhyErrIrqMask;

      VDB_LOG_NOTE("IRQs EN: 0x%x\n", sppCsiDrvInfo[lCsiIdx]->mpRegs->INTRENVC.R);

      /// *** Setup embedded data reception ***
      CSI_DRV_EmbdConfig(&(apcCsiCfg->mEmbeddedData));
    } // if sppCsiDrvInfo not NULL
    else
    {
      VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
    } // else from if sppCsiDrvInfo not NULL
  } // if config not NULL
  else
  {
    VDB_LOG_ERROR("Config structure is NULL.\n");
  } // else from if config not NULL
} // CSI_DRV_Config()

/*============================================================================*/
/**
* @brief        CSI Embedded data configuration.
* @details      CSI Embedded data configuration.
*
* @param        CSI_EmbdCfg_t structure
*
* @implements   CSI_DRV_EmbdConfig_Activity
*
* @note
*/
void CSI_DRV_EmbdConfig(const CSI_EmbdCfg_t *apcCfg)
{
  bool CSI_VC_IDX_USED_cond;

  if(apcCfg != NULL)
  {
    uint8_t  lCsiIdx  = CSI_IDX_GET((apcCfg->mVcIdFlags));

    if(apcCfg->mBufAddr != 0U)
    {
      uint32_t i = 0U;
      uint32_t lEmbdIrqs = 0x0U;
      uint8_t lEmbVcsEnbale = 0U;

      // This API only works when sppCsiDrvInfo is different than NULL
      if(sppCsiDrvInfo[lCsiIdx] != NULL)
      {
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDSP.R   = apcCfg->mBufAddr;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDLEN.R  = apcCfg->mSize;

        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBMSTR.R   = apcCfg->mMasterIdx;

        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIRQ1.R = apcCfg->mLineCnt1;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIRQ2.R = apcCfg->mLineCnt2;

        sppCsiDrvInfo[lCsiIdx]->mEmbLnCnt1 = apcCfg->mLineCnt1;
        sppCsiDrvInfo[lCsiIdx]->mEmbLnCnt2 = apcCfg->mLineCnt2;

        // enable requested channels
        for(i = 0; i < CSI_CHNL_CNT; i++)
        {
          CSI_VC_IDX_USED_cond = CSI_VC_IDX_USED((apcCfg->mVcIdFlags), ((uint16_t)i));

          if(CSI_VC_IDX_USED_cond != false)
          {
            lEmbVcsEnbale |= (1U << i);
          } // if VC used
        } // for all possible VCs
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDENB.R = lEmbVcsEnbale;

        // enable interrupts if any lines requested
        lEmbdIrqs = (uint32_t)((apcCfg->mLineCnt1 > 0U)?
                    (uint8_t)CSI_EMBD_IRQ_TYPE_EMBIS0:0U) |
                    ((apcCfg->mLineCnt2 > 0U)?
                    (uint8_t)CSI_EMBD_IRQ_TYPE_EMBIS1:0U);

        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIE.R   = lEmbdIrqs;
      } // if sppCsiDrvInfo not NULL
      else
      {
        VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
      } // else from if sppCsiDrvInfo not NULL
    } // if Buffer address not 0
    else
    {
      // This API only works when sppCsiDrvInfo is different than NULL
      if(sppCsiDrvInfo[lCsiIdx] != NULL)
      {
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDENB.R  = 0;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIE.R   = 0;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDINTS.R =
                                             (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS0 |
                                             (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS1;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDSP.R   = 0;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDLEN.R  = 0;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBMSTR.R   = 0;

        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIRQ1.R = 0;
        sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIRQ2.R = 0;

        /// *** update driver info structure ***
        sppCsiDrvInfo[lCsiIdx]->mEmbedStatus = (uint32_t)CSI_EMBD_STATUS_NA;
      } // if sppCsiDrvInfo not NULL
      else
      {
        VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
      } // else from if sppCsiDrvInfo not NULL
    } // else from if if Buffer address not 0

    VDB_LOG_NOTE("EMBD configured, IE: 0x%x\n",
                 sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDIE.R);
    VDB_LOG_NOTE("EMBD configured, IS: 0x%x\n",
                 sppCsiDrvInfo[lCsiIdx]->mpRegs->EMBEDINTS.R);
  }// if config not NULL
} // CSI_DRV_EmbdConfig()

/*============================================================================*/
/**
* @brief        CSI Embedded data configuration.
* @details      CSI Embedded data configuration.
*
* @param        CSI_IDX
*
* @implements   CSI_DRV_EmbdRecap_Activity
*
* @note
*/
void CSI_DRV_EmbdRecap(const enum CSI_IDX acCsiIdx)
{
  uint32_t       lEmbdIrqs;

  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[acCsiIdx] != NULL)
  {
    // enable interrupts if any lines requested
    lEmbdIrqs = (uint32_t)((sppCsiDrvInfo[acCsiIdx]->mEmbLnCnt1 > 0U)?
                (uint8_t)CSI_EMBD_IRQ_TYPE_EMBIS0:0U) |
                ((sppCsiDrvInfo[acCsiIdx]->mEmbLnCnt2 > 0U)?
                (uint8_t)CSI_EMBD_IRQ_TYPE_EMBIS1:0U);

    // allow rewrite of embdata and enable interrupts
    sppCsiDrvInfo[acCsiIdx]->mEmbedStatus        = (uint32_t)CSI_EMBD_STATUS_NA;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->EMBEDINTS.R = lEmbdIrqs;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->EMBEDIE.R   = lEmbdIrqs;
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL
} // CSI_DRV_EmbdRecap()

/*============================================================================*/
/**
* @brief        CSI Embedded status get.
* @details      Provides CSI Embedded data status.
*
* @param        CSI_IDX
*
* @implements   CSI_DRV_EmbdStatusGet_Activity
*
* @note
*/
uint32_t CSI_DRV_EmbdStatusGet(const enum CSI_IDX acCsiIdx)
{
  uint32_t  lEmbedStatus = 0U;

  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[acCsiIdx] != NULL)
  {
    lEmbedStatus = sppCsiDrvInfo[acCsiIdx]->mEmbedStatus;
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL

  return lEmbedStatus;
} // CSI_DRV_EmbdStatusGet()

/*============================================================================*/
/**
* @brief        CSI error status.
* @details      Provied CSI error status.
*
* @param         CSI_IDX
*
* @implements    CSI_DRV_ErrStatusGet_Activity
*
* @note
*/
CSI_ErrStatus_t * CSI_DRV_ErrStatusGet(const enum CSI_IDX acCsiIdx)
{
  uint32_t i = 0;
  CSI_ErrStatus_t *lErrStatus = NULL;

  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[acCsiIdx] != NULL)
  {
    // fetch latest error flags
    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mLineErr =
      sppCsiDrvInfo[acCsiIdx]->mpRegs->INTRSVC.R & CSI_IRQ_ERRLIN_MASK_ALL;

    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mProtPackErr =
      sppCsiDrvInfo[acCsiIdx]->mpRegs->ERRPPREG.R & CSI_IRQ_ERRPP_MASK_ALL;

    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mPhyErr =
      sppCsiDrvInfo[acCsiIdx]->mpRegs->ERRPHY.R & CSI_IRQ_ERRPHY_MASK_ALL;

    // remember line lengths for LLE
    for(i = 0; i < CSI_CHNL_CNT; i++)
    {
      sppCsiDrvInfo[acCsiIdx]->mErrStatus.mErrLength[i] =
          sppCsiDrvInfo[acCsiIdx]->mpRegs->CH[i].ERRLEN.R;
      sppCsiDrvInfo[acCsiIdx]->mErrStatus.mErrLine[i] =
          sppCsiDrvInfo[acCsiIdx]->mpRegs->CH[i].ERRLINE.R;
    } // for all channels

    // remember ECC error position
    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mEccErrPos =
      sppCsiDrvInfo[acCsiIdx]->mpRegs->ERRPOS.R;

    lErrStatus = &(sppCsiDrvInfo[acCsiIdx]->mErrStatus);
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL

  return lErrStatus;
} // CSI_DRV_ErrStatusGet()

/*============================================================================*/
/**
* @brief        CSI error clear.
* @details      Clears error flags.
*
* @param acCsiIdx
*
* @implements    CSI_DRV_ErrClear_Activity
*
* @note
*/
void CSI_DRV_ErrClear(const enum CSI_IDX acCsiIdx)
{
  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[acCsiIdx] != NULL)
  {
    // cleared error cache
    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mProtPackErr = 0;
    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mPhyErr      = 0;
    sppCsiDrvInfo[acCsiIdx]->mErrStatus.mLineErr     = 0;

    // reset error flags
    sppCsiDrvInfo[acCsiIdx]->mpRegs->INTRSVC.R  = CSI_IRQ_ERRLIN_MASK_ALL;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->ERRPPREG.R = CSI_IRQ_ERRPP_MASK_ALL;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->ERRPHY.R   = CSI_IRQ_ERRPHY_MASK_ALL;

    // enable interrupts
    sppCsiDrvInfo[acCsiIdx]->mpRegs->INTRENVC.R  =
      sppCsiDrvInfo[acCsiIdx]->mFrameIrqMask;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->ERPPINTEN.R =
      sppCsiDrvInfo[acCsiIdx]->mPPErrIrqMask;
    sppCsiDrvInfo[acCsiIdx]->mpRegs->ERPHYIE.R   =
      sppCsiDrvInfo[acCsiIdx]->mPhyErrIrqMask;
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL
} // CSI_DRV_ErrClear()

/*============================================================================*/
/**
* @brief        CSI driver stop.
* @details      CSI driver stop.
*
* @param acCsiIdx
*
* @implements    CSI_DRV_Stop_Activity
*
* @note
*/
void CSI_DRV_Stop(CSI_IdKey_t aCsiIdKey)
{
  uint8_t  lCsiIdx     = CSI_IDX_GET((aCsiIdKey));
  uint32_t i           = 0U;
  uint8_t lChnlMask   = 0U;
  bool CSI_VC_IDX_USED_cond;

  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[lCsiIdx] != NULL)
  {
    // reset user pid
    sppCsiDrvInfo[lCsiIdx]->mUserPid = 0;

    // get channel mask
    for(i = 0; i < CSI_CHNL_CNT; i++)
    {
      CSI_VC_IDX_USED_cond = CSI_VC_IDX_USED((aCsiIdKey),((uint16_t)i));

      if(CSI_VC_IDX_USED_cond != false)
      {
        lChnlMask |= (1U << i);
      } // if VC used
    } // for all possible VCs

    // Disable specified VCs
    sppCsiDrvInfo[lCsiIdx]->mpRegs->ENABLECH.R &= ~(lChnlMask);

    sppCsiDrvInfo[lCsiIdx]->mpRegs->RXEN.B.RXEN = 0; // Receive disable
    sppCsiDrvInfo[lCsiIdx]->mpRegs->SR.B.SOFRST = 1; // Receive disable
    sppCsiDrvInfo[lCsiIdx]->mpRegs->SR.B.SOFRST = 0; // Receive disable

#ifdef __STANDALONE__
    // disable IRQs
    OAL_IRQDisable(sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mFrm);
    OAL_IRQDisable(sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mErr);
    OAL_IRQDisable(sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mFrm);
    OAL_IRQDisable(sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mErr);
#else // ifdef __STANDALONE__

#endif // else from ifdef __STANDALONE__
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL
} // CSI_DRV_Stop();

/*============================================================================*/
/**
* @brief        CSI driver start.
* @details      CSI driver start, enables virtual channels.
*
* @param acCsiIdx
*
* @implements    CSI_DRV_Start_Activity
*
* @note
*/
void CSI_DRV_Start(CSI_IdKey_t aCsiIdKey)
{
  CSI_IDX_t  lCsiIdx        = CSI_IDX_GET((aCsiIdKey));
  bool CSI_VC_IDX_USED_cond;

  uint32_t i           = 0U;
  uint8_t lChnlEnable = 0U;
  uint32_t lCondition  = 0U;

  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[lCsiIdx] != NULL)
  {
// remember user PID
#ifndef __STANDALONE__
    sppCsiDrvInfo[lCsiIdx]->mUserPid = current->pid;
#else
    sppCsiDrvInfo[lCsiIdx]->mUserPid = 1;
#endif // end from ifndef __STANDALONE__

    // reset error status
    CSI_DRV_ErrClear(lCsiIdx);

    sppCsiDrvInfo[lCsiIdx]->mFrmCnt = 0;
    sppCsiDrvInfo[lCsiIdx]->mpRegs->RXEN.B.RXEN = 1; // Receive enable

    lCondition = sppCsiDrvInfo[lCsiIdx]->mpRegs->RESCS.B.CALCOM;

    while(lCondition == 0U)
    {
      lCondition = sppCsiDrvInfo[lCsiIdx]->mpRegs->RESCS.B.CALCOM;
    } // while not ready

    for(i = 0U; i < CSI_CHNL_CNT; i++)
    {
      CSI_VC_IDX_USED_cond = CSI_VC_IDX_USED((aCsiIdKey),((uint16_t)i));

      if(CSI_VC_IDX_USED_cond != false)
      {
        lChnlEnable |= (1U << i);
      } // if VC used
    } // for all possible VCs

    // Enable Virtual Channel Zero
    sppCsiDrvInfo[lCsiIdx]->mpRegs->ENABLECH.R = lChnlEnable;
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL
} // CSI_DRV_Start()

/*============================================================================*/
/**
* @brief        CSI HW IP soft reset.
* @details      Requests a soft reset of the CSI IP. All registers are reset and
*               all FIFOs are flushed.
*
* @param acCsiIdx
*
* @implements    CSI_DRV_SwReset_Activity
*
* @note
*/
void CSI_DRV_SwReset(CSI_IDX_t aCsiIdx)
{
  // This API only works when sppCsiDrvInfo is different than NULL
  if(sppCsiDrvInfo[aCsiIdx] != NULL)
  {
    // soft reset the HW IP
    sppCsiDrvInfo[aCsiIdx]->mpRegs->SR.B.SOFRST = 1;
    sppCsiDrvInfo[aCsiIdx]->mpRegs->SR.B.SOFRST = 0;
  } // if sppCsiDrvInfo not NULL
  else
  {
    VDB_LOG_ERROR("sppCsiDrvInfo is NULL.\n");
  } // else from if sppCsiDrvInfo not NULL

  // reset error status
  CSI_DRV_ErrClear(aCsiIdx);
} // CSI_DRV_SwReset()

/*============================================================================*/
/**
* @brief         Requests CSI irqs
* @details       Requests CSI irqs
*
* @return        int32_t
*/
static int32_t CSI_DRV_IrqsRequest(void)
{
  int32_t lRet = CSI_DRV_SUCCESS;
#ifndef __STANDALONE__

  if (request_irq(
                  sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mFrm,
                  &CSI0_IrqHandler,
                  0,
                  CSI_DEVICE_NAME,
                  NULL) != 0)
  {
    lRet = CSI_DRV_FAILURE;
    VDB_LOG_ERROR("Failed to request IRQ for CSI0.\n");
  } // if request for MSG IRQ failed

  if (request_irq(
                  sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mErr,
                  &CSI0_ErrIrqHandler,
                  0,
                  CSI_DEVICE_NAME,
                  NULL) != 0)
  {
    lRet = CSI_DRV_FAILURE;
    VDB_LOG_ERROR("Failed to request Error IRQ for CSI0.\n");
  } // if request for MSG IRQ failed

  if (request_irq(
                  sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mFrm,
                  &CSI1_IrqHandler,
                  0,
                  CSI_DEVICE_NAME,
                  NULL) != 0)
  {
    lRet = CSI_DRV_FAILURE;
    VDB_LOG_ERROR("Failed to request IRQ for CSI1.\n");
  } // if request for MSG_ACK IRQ failed

  if (request_irq(
                  sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mErr,
                  &CSI1_ErrIrqHandler,
                  0,
                  CSI_DEVICE_NAME,
                  NULL) != 0)
  {
    lRet = CSI_DRV_FAILURE;
    VDB_LOG_ERROR("Failed to request Error IRQ for CSI1.\n");
  } // if request for MSG_ACK IRQ failed
#else
  // *** CSI0 ***
  OAL_IRQInit(CSI0_IrqHandler,
              sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mFrm,
              0xA0,
              OAL_IRQ_EDGE
              );
  OAL_IRQInit(CSI0_ErrIrqHandler,
              sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mErr,
              0xA0,
              OAL_IRQ_EDGE
              );

  OAL_IRQEnable(CSI0_ERR_IRQ);
  OAL_IRQEnable(CSI0_FRM_IRQ);

  // *** CSI1 ***
  OAL_IRQInit(CSI1_IrqHandler,
              sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mFrm,
              0xA0,
              OAL_IRQ_EDGE
              );
  OAL_IRQInit(CSI1_ErrIrqHandler,
              sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mErr,
              0xA0,
              OAL_IRQ_EDGE
              );

  OAL_IRQEnable(CSI1_ERR_IRQ);
  OAL_IRQEnable(CSI1_FRM_IRQ);
#endif // from ifndef __STANDALONE__
  return lRet;
} // CSI_DRV_IrqsRequest()

/*============================================================================*/
/**
* @brief         Free CSI irqs
* @details       Free CSI irqs
*
* @implements    CSI_DRV_IrqsFree_Activity
*
* @note
*
*/
static void CSI_DRV_IrqsFree(void)
{
  free_irq(sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mFrm, NULL);
  free_irq(sppCsiDrvInfo[CSI_IDX_0]->mIrqNums.mErr, NULL);
  free_irq(sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mFrm, NULL);
  free_irq(sppCsiDrvInfo[CSI_IDX_1]->mIrqNums.mErr, NULL);
} // CSI_DRV_IrqsFree()


/*============================================================================*/
/**
* @brief         CSI irq registeration
* @details       CSI irq registeration
*
* @note no need for locking under the assumption only one camera type
*       per CSI I/F => only one handler to be registered.
*
*/

void CSI_IrqHandlerRegister(enum CSI_IDX aCsiIdx,
                            void(*apIrqHandler)(enum CSI_IDX aCsiIdx,
                                                uint32_t aIrqType)
                           )
{
  spIrqHandler[aCsiIdx] = apIrqHandler;
} // CSI_IrqHandlerRegister()

/*============================================================================*/
#ifndef __STANDALONE__
/**
* @brief         CSI irq handler
* @details       CSI irq handler
*
* @implements    CSI0_IrqHandler_Activity
*
* @note
*
*/
static irqreturn_t CSI0_IrqHandler(int32_t aIrq, void *apDev_id)
{
  irqreturn_t  ret = IRQ_HANDLED;
#else // ifndef __STANDALONE__
static void CSI0_IrqHandler()
{
#endif // else from #ifndef __STANDALONE__

  uint32_t lTmpIrqStatus = 0;
  uint32_t lIrqReset     = 0;
  uint32_t i             = 0;

  uint32_t lIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRSVC.R;
  uint32_t lIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRENVC.R;

  uint32_t lIrqFlags = lIrqFlag_1 & lIrqFlag_2;

  //evaluate Embedded data irq flags only once enabled
  uint32_t lEmbdIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDINTS.R;
  uint32_t lEmbdIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDIE.R;

  uint32_t lEmbdIrqFlags = lEmbdIrqFlag_1 & lEmbdIrqFlag_2;

  // repeat until IRQ flags not empty
  do
  {
    // *** frame start ***
    lTmpIrqStatus = (uint32_t)lIrqFlags & (
                  (uint32_t)CSI_IRQ_TYPE_SOF_0 | (uint32_t)CSI_IRQ_TYPE_SOF_1 |
                  (uint32_t)CSI_IRQ_TYPE_SOF_2 | (uint32_t)CSI_IRQ_TYPE_SOF_3);
    if(lTmpIrqStatus > 0U)
    {
      lIrqReset += lTmpIrqStatus; // reset frame start
    } // if frame start signaled

    // *** frame end ***
    lTmpIrqStatus = lIrqFlags & (
                  (uint32_t)CSI_IRQ_TYPE_EOF_0 | (uint32_t)CSI_IRQ_TYPE_EOF_1 |
                  (uint32_t)CSI_IRQ_TYPE_EOF_2 | (uint32_t)CSI_IRQ_TYPE_EOF_3);

    if(lIrqFlags > 0U)
    {
      lIrqReset += lTmpIrqStatus; // reset frame End
      sppCsiDrvInfo[CSI_IDX_0]->mFrmCnt++;
    } // if frame end signaled

    // *** line length/count error ***
    lTmpIrqStatus = lIrqFlags & CSI_IRQ_ERRLIN_MASK_ALL;

    if(lTmpIrqStatus > 0U)
    {
      // remember frame index
      sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mLineErrFrmId =
        sppCsiDrvInfo[CSI_IDX_0]->mFrmCnt;

      // pause receiving all line error IRQs until user reset
      sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mLineErr |= lTmpIrqStatus;
      sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRENVC.R   =
        sppCsiDrvInfo[CSI_IDX_0]->mFrameIrqMask & CSI_IRQ_FRM_MASK_ALL;

      for(i = 0; i < (uint32_t)CSI_CHNL_CNT; i++)
      {
        sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mErrLineCnt[i] =
                (uint32_t)(sppCsiDrvInfo[CSI_IDX_0]->mpRegs->CH[i].TOTLIN.R);
      }

      // *** send signal ***
      if(sppCsiDrvInfo[CSI_IDX_0]->mUserPid != 0)
      {
        int32_t lCsiEvent = 0;
        if ((lTmpIrqStatus & CSI_IRQ_ERRLIN_CNT_MASK_ALL) > 0U)
        {
          lCsiEvent |= CSI_IDX_0_LINE_CNT_ERR;
        }
        if ((lTmpIrqStatus & CSI_IRQ_ERRLIN_LEN_MASK_ALL) > 0U)
        {
          lCsiEvent |= CSI_IDX_0_LINE_LEN_ERR;
        }
        if(Sig2UsrSend(sppCsiDrvInfo[CSI_IDX_0]->mUserPid, lCsiEvent) < 0)
        {
          VDB_LOG_ERROR("FAILED to send signal to user-space process %u\n",
                        (uint32_t) sppCsiDrvInfo[CSI_IDX_0]->mUserPid);
        } // if signal not send
      } // if there is a user
    } // if frame end signaled

    /// *** embedded data IRQs ***
    if((lEmbdIrqFlags & (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS0) > 0U)
    {
      // stop interrupt reception
      sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDIE.R &=
                                            (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS1;
      sppCsiDrvInfo[CSI_IDX_0]->mEmbedStatus |= (uint32_t)CSI_EMBD_STATUS_1;
    } // if EMBD frame start signaled
    if((lEmbdIrqFlags & (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS1) > 0U)
    {
      // stop interrupt reception
      sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDIE.R &=
                                            (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS0;
      sppCsiDrvInfo[CSI_IDX_0]->mEmbedStatus    |= (uint32_t)CSI_EMBD_STATUS_2;
    } // if EMBD frame end signaled

    // reset frame data interrupts
    sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRSVC.R = lIrqReset;

    // do not reset embedded data interrupts !!!
    // (otherwise embd data will be ovrewritten)

    // call registerred handller if any
    if(spIrqHandler[0] != NULL)
    {
      spIrqHandler[0](CSI_IDX_0, lIrqReset);
    } // if handler assigned

    lIrqFlag_1   = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRSVC.R;
    lIrqFlag_2   = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->INTRENVC.R;

    lIrqFlags    = lIrqFlag_1 & lIrqFlag_2;

    lEmbdIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDINTS.R;
    lEmbdIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_0]->mpRegs->EMBEDIE.R;

    lEmbdIrqFlags = lEmbdIrqFlag_1 & lEmbdIrqFlag_2;

  }while((lIrqFlags + lEmbdIrqFlags) != 0u);

#ifndef __STANDALONE__
  return ret;
#endif // #ifndef __STANDALONE__
} // CSI0_IrqHandler()

/*============================================================================*/
#ifndef __STANDALONE__
/**
* @brief         CSI error irq handler
* @details       CSI error irq handler
*
* @implements    CSI0_ErrIrqHandler_Activity
*
* @note
*
*/
static irqreturn_t CSI0_ErrIrqHandler(int32_t aIrq, void *apDev_id)
{
  irqreturn_t ret = IRQ_HANDLED;
#else // ifndef __STANDALONE__
static void CSI0_ErrIrqHandler()
{
#endif // else from #ifndef __STANDALONE__

  // remember frame index
  sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mOthrErrFrmId =
      sppCsiDrvInfo[CSI_IDX_0]->mFrmCnt;

  // remember error flags
  sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mProtPackErr =
      sppCsiDrvInfo[CSI_IDX_0]->mpRegs->ERRPPREG.R & CSI_IRQ_ERRPP_MASK_ALL;

  sppCsiDrvInfo[CSI_IDX_0]->mErrStatus.mPhyErr =
      sppCsiDrvInfo[CSI_IDX_0]->mpRegs->ERRPHY.R & CSI_IRQ_ERRPHY_MASK_ALL;

  // disable err interupts
  sppCsiDrvInfo[CSI_IDX_0]->mpRegs->ERPPINTEN.R = 0;
  sppCsiDrvInfo[CSI_IDX_0]->mpRegs->ERPHYIE.R   = 0;

  // *** send signal ***
  if(sppCsiDrvInfo[CSI_IDX_0]->mUserPid != 0)
  {
    if(Sig2UsrSend(sppCsiDrvInfo[CSI_IDX_0]->mUserPid, (int32_t)CSI_IDX_0_PP_PHY_ERR) < 0)
    {
      VDB_LOG_ERROR("FAILED to send signal to user-space process %u\n",
                    (uint32_t) sppCsiDrvInfo[CSI_IDX_0]->mUserPid);
    } // if signal not send
  } // if there is a user
#ifndef __STANDALONE__
  return ret;
#endif // #ifndef __STANDALONE__
} // CSI0_ErrIrqHandler()

/*============================================================================*/
#ifndef __STANDALONE__
/**
* @brief         CSI irq handler
* @details       CSI irq handler
*
* @implements    CSI1_IrqHandler_Activity
*
* @note
*
*/
static irqreturn_t CSI1_IrqHandler(int32_t aIrq, void *apDev_id)
{
  irqreturn_t  ret = IRQ_HANDLED;
#else // ifndef __STANDALONE__
static void CSI1_IrqHandler()
{
#endif // else from #ifndef __STANDALONE__

  uint32_t lTmpIrqStatus = 0;
  uint32_t lIrqReset     = 0;
  uint32_t i             = 0;

  uint32_t lIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRSVC.R;
  uint32_t lIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRENVC.R;

  uint32_t lIrqFlags = lIrqFlag_1 & lIrqFlag_2;

  //evaluate Embedded data irq flags only once enabled
  uint32_t lEmbdIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDINTS.R;
  uint32_t lEmbdIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDIE.R;

  uint32_t lEmbdIrqFlags = lEmbdIrqFlag_1 & lEmbdIrqFlag_2;

  // repeat until IRQ flags not empty
  do
  {
    // *** frame start ***
    lTmpIrqStatus = lIrqFlags & ((uint32_t)CSI_IRQ_TYPE_SOF_0 |
                                 (uint32_t)CSI_IRQ_TYPE_SOF_1 |
                                 (uint32_t)CSI_IRQ_TYPE_SOF_2 |
                                 (uint32_t)CSI_IRQ_TYPE_SOF_3);
    if(lTmpIrqStatus > 0U)
    {
      lIrqReset += lTmpIrqStatus; // reset frame start
    } // if frame start signaled

    // *** frame end ***
    lTmpIrqStatus = lIrqFlags & ((uint32_t)CSI_IRQ_TYPE_EOF_0 |
                                 (uint32_t)CSI_IRQ_TYPE_EOF_1 |
                                 (uint32_t)CSI_IRQ_TYPE_EOF_2 |
                                 (uint32_t)CSI_IRQ_TYPE_EOF_3);

    if(lIrqFlags > 0U)
    {
      lIrqReset += lTmpIrqStatus; // reset frame End
      sppCsiDrvInfo[CSI_IDX_1]->mFrmCnt++;
    } // if frame end signaled

    // *** line length/count error ***
    lTmpIrqStatus = lIrqFlags & CSI_IRQ_ERRLIN_MASK_ALL;

    if(lTmpIrqStatus > 0U)
    {
      // remember frame index
      sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mLineErrFrmId =
        sppCsiDrvInfo[CSI_IDX_1]->mFrmCnt;

      // pause receiving all line error IRQs until user reset
      sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mLineErr |= lTmpIrqStatus;
      sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRENVC.R   =
        sppCsiDrvInfo[CSI_IDX_1]->mFrameIrqMask & CSI_IRQ_FRM_MASK_ALL;

      for(i = 0; i < (uint32_t)CSI_CHNL_CNT; i++)
      {
        sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mErrLineCnt[i] =
                (uint32_t)(sppCsiDrvInfo[CSI_IDX_1]->mpRegs->CH[i].TOTLIN.R);
      }

      // *** send signal ***
      if(sppCsiDrvInfo[CSI_IDX_1]->mUserPid != 0)
      {
        int32_t lCsiEvent = 0;
        if ((lTmpIrqStatus & CSI_IRQ_ERRLIN_CNT_MASK_ALL) > 0U)
        {
          lCsiEvent |= CSI_IDX_1_LINE_CNT_ERR;
        }
        if ((lTmpIrqStatus & CSI_IRQ_ERRLIN_LEN_MASK_ALL) > 0U)
        {
          lCsiEvent |= CSI_IDX_1_LINE_LEN_ERR;
        }
        if(Sig2UsrSend(sppCsiDrvInfo[CSI_IDX_1]->mUserPid, lCsiEvent) < 0)
        {
          VDB_LOG_ERROR("FAILED to send signal to user-space process %u\n",
                        (uint32_t) sppCsiDrvInfo[CSI_IDX_1]->mUserPid);
        } // if signal not send
      } // if there is a user
    } // if frame end signaled

    /// *** embedded data IRQs ***
    if((lEmbdIrqFlags & (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS0) > 0u)
    {
      // stop interrupt reception
      sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDIE.R &=
                                            (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS1;
      sppCsiDrvInfo[CSI_IDX_1]->mEmbedStatus |= (uint32_t)CSI_EMBD_STATUS_1;
    } // if EMBD frame start signaled
    if((lEmbdIrqFlags & (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS1) > 0U)
    {
      // stop interrupt reception
      sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDIE.R &=
                                            (uint32_t)CSI_EMBD_IRQ_TYPE_EMBIS0;
      sppCsiDrvInfo[CSI_IDX_1]->mEmbedStatus |= (uint32_t)CSI_EMBD_STATUS_2;
    } // if EMBD frame end signaled

    // reset frame data interrupts
    sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRSVC.R   = lIrqReset;

    // do not reset embedded data interrupts !!!
    // (otherwise embd data will be ovrewritten)

    // call registerred handller if any
    if(spIrqHandler[1] != NULL)
    {
      spIrqHandler[1](CSI_IDX_1, lIrqReset);
    } // if handler assigned

    lIrqFlag_1   = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRSVC.R;
    lIrqFlag_2   = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->INTRENVC.R;

    lIrqFlags    = lIrqFlag_1 & lIrqFlag_2;

    lEmbdIrqFlag_1 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDINTS.R;
    lEmbdIrqFlag_2 = sppCsiDrvInfo[CSI_IDX_1]->mpRegs->EMBEDIE.R;

    //evaluate Embedded data irq flags only once enabled
    lEmbdIrqFlags = lEmbdIrqFlag_1 & lEmbdIrqFlag_2;

  }while((lIrqFlags + lEmbdIrqFlags) != 0U);

#ifndef __STANDALONE__
  return ret;
#endif // #ifndef __STANDALONE__
} // CSI1_IrqHandler()

/*============================================================================*/
#ifndef __STANDALONE__
/**
* @brief         CSI error irq handler
* @details       CSI erorr irq handler
*
* @implements    CSI1_ErrIrqHandler_Activity
*
* @note
*
*/
static irqreturn_t CSI1_ErrIrqHandler(int32_t aIrq, void *apDev_id)
{
  irqreturn_t ret = IRQ_HANDLED;
#else // ifndef __STANDALONE__
static void CSI1_ErrIrqHandler()
{
#endif // else from #ifndef __STANDALONE__

  // remember frame index
  sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mOthrErrFrmId =
      sppCsiDrvInfo[CSI_IDX_1]->mFrmCnt;

  // remember error flags
  sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mProtPackErr =
      sppCsiDrvInfo[CSI_IDX_1]->mpRegs->ERRPPREG.R & CSI_IRQ_ERRPP_MASK_ALL;

  sppCsiDrvInfo[CSI_IDX_1]->mErrStatus.mPhyErr =
      sppCsiDrvInfo[CSI_IDX_1]->mpRegs->ERRPHY.R & CSI_IRQ_ERRPHY_MASK_ALL;

  // disable err interupts
  sppCsiDrvInfo[CSI_IDX_1]->mpRegs->ERPPINTEN.R = 0;
  sppCsiDrvInfo[CSI_IDX_1]->mpRegs->ERPHYIE.R   = 0;

  // *** send signal ***
  if(sppCsiDrvInfo[CSI_IDX_1]->mUserPid != 0)
  {
    if(Sig2UsrSend(sppCsiDrvInfo[CSI_IDX_1]->mUserPid, (int32_t)CSI_IDX_1_PP_PHY_ERR) < 0)
    {
      VDB_LOG_ERROR("FAILED to send signal to user-space process %u\n",
                    (uint32_t) sppCsiDrvInfo[CSI_IDX_1]->mUserPid);
    } // if signal not send
  } // if there is a user

#ifndef __STANDALONE__
  return ret;
#endif // #ifndef __STANDALONE__
} // CSI1_ErrIrqHandler()

/*============================================================================*/
/**
* @brief         Send signal to user.
* @details       Send signal to user.
*
*/
static int32_t Sig2UsrSend(pid_t aPid, int32_t aValue)
{
  int32_t lRet = 0;
#ifndef __STANDALONE__
  struct siginfo lInfo;
  struct pid *lpPidStruct;
  struct task_struct *lpTask;

  // send the signal to user space
  (void)memset(&lInfo, 0, sizeof(struct siginfo));
  lInfo.si_signo = CSI_ERR_SIGNAL;
  lInfo.si_code  = SI_QUEUE;
  lInfo.si_int   = (aValue);
  rcu_read_lock();
  lpPidStruct    = find_get_pid(aPid);
  lpTask         = pid_task(lpPidStruct,PIDTYPE_PID);
  if(lpTask == NULL)
  {
    VDB_LOG_ERROR("No such pid %u. Signal not sent.\n", (uint32_t) aPid);
    rcu_read_unlock();
    lRet = -1;
  } // if no such pidRegisterred
  else
  {
    rcu_read_unlock();
    lRet = send_sig_info(CSI_ERR_SIGNAL, &lInfo, lpTask);    //send the signal
    if (lRet < 0)
    {
      VDB_LOG_ERROR("Error sending signal to user space.\n");
    } // if sending signal failed
  } // else if no such pid
#else // ifndef __STANDALONE__
  CsiSigHandler((int)CSI_ERR_SIGNAL, (int)aValue);
#endif // else from #ifndef __STANDALONE__
  return lRet;
} // Sig2UsrSend()

/*============================================================================*/

#ifndef __STANDALONE__
  EXPORT_SYMBOL(CSI_IrqHandlerRegister);
#endif // #ifndef __STANDALONE__

#endif // #if defined(__KERNEL__) || defined(__STANDALONE__)
