/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright 2017 NXP
 * All Rights Reserved
 *
 ******************************************************************************
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
 * \file    static_ov10640_default.c
 * \brief   ISP (sequencer) static configuration
 * \author  Armin Strobl
 * \author
 * \version 0.1
 * \date    12-10-2017
 * \note    initial version
 ****************************************************************************/

#include "seq_static.h"

/*****************************************************************************
* consts
*****************************************************************************/
#define	SEQ_START	0

#define EMBD_REGS_START     (SEQ_START+1)       // FastDMA transfer of EMBD_REGS FDMA_channel2
#define EMBD_STATS_START    (SEQ_START+731)     // FastDMA transfer of EMBD_STATS FDMA_channel1
#define DECOMP_START        (SEQ_START+3)       // skip first 2 lines
#define CAMOUT_START        (DECOMP_START+1)    // CAM_OUTPUT
#define	HIST_START          (DECOMP_START+4)
#define	HDR_START           (DECOMP_START+2)

#define	DBAY0_START         (HDR_START+3)
#define	RGBY0_START         (DBAY0_START+2)
#define	LUT0_START          (RGBY0_START+2)
#define	CMPR0_START         (LUT0_START+2)
#define	DNSY0_START         (CMPR0_START+4)
#define	YOUT0_START         (DNSY0_START+2)     // ISP_OUTPUT0

#define	DBAY1_START         (HDR_START+4)
#define	RGBY1_START         (DBAY1_START+2)
#define	LUT1_START          (RGBY1_START+2)
#define	CMPR1_START         (LUT1_START+2)
#define	DNSY1_START         (CMPR1_START+4)
#define	YOUT1_START         (DNSY1_START+2)     // ISP_OUTPUT1

#define DNSC_START          (CMPR1_START+4)
#define	UVOUT_START         (DNSC_START+2)      // ISP_OUTPUTUV_1

/****************************************************************************/

void StaticSequencing(void)
{
  sStaticSeq = STAT_SEQ_MIPI;

  initFDMAnode(5, EMBD_STATS_START, 1);     /* EMBD_STATS */
  initFDMAnode(4, EMBD_REGS_START, 1);      /* EMBD_REGS */
  initIPUnode(15, DECOMP_START, 1, 0);      /* Histogram */
  initIPUnode(10, HIST_START, 4, 0);        /* Histogram */
  initIPUnode(11, HDR_START,  1, 0);        /* HDR */

  initIPUnode(2,  DBAY0_START, 2, 0);       /* Debayer0 */
  initIPUnode(3,  RGBY0_START, 2, 1);       /* RGB2Y0 */
  initIPUnode(4,  LUT0_START,  2, 1);       /* LUT4K0 */
  initIPUnode(5,  CMPR0_START, 2, 0);       /* COMPRESS0 */
  initIPUnode(12, DNSY0_START, 2, 0);       /* DENOISE0 */
  initFDMAnode(0, YOUT0_START, 2);          /* Y0 Output */

  initIPUnode(6,  DBAY1_START, 2, 0);       /* Debayer1 */
  initIPUnode(7,  RGBY1_START, 2, 0);       /* RGB2Y1 */
  initIPUnode(8,  LUT1_START,  2, 1);       /* LUT4K1 */
  initIPUnode(9,  CMPR1_START, 2, 0);       /* COMPRESS1 */
  initIPUnode(13, DNSY1_START, 2, 0);       /* DENOISE1 */
  initFDMAnode(1, YOUT1_START, 2);          /* Y1 Output */

  initIPUnode(14, DNSC_START,  2, 1);       /* DENOISE1 */
  initFDMAnode(2, UVOUT_START, 2);          /* UV0 Output */

  initFDMAnode(3, CAMOUT_START,1);          /* CAM Dump */
} // StaticSequencing()

/* EOF */
