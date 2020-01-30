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
 * \file    static_ov9716_mipi_s.c
 * \brief   ISP (sequencer) static configuration
 * \author
 * \author
 * \version 0.1
 * \date    04-04-2018
 * \note    initial version
 ****************************************************************************/

#include "seq_static.h"

/*****************************************************************************
* consts
*****************************************************************************/
// Y + UV semiplanar model

#define STATIC_SEQUENCE_MIPI

// *** defines of the engines start line numbers ***
#define SEQ_START 0
#define CAMOUT_START  (SEQ_START+1) // CAM_OUTPUT

#define ILUT_START  (SEQ_START+2)
#define HIST_START  (ILUT_START+8)
#define HDR_START   (ILUT_START+8)

#define DBAY0_START (HDR_START+6)
#define RGBY0_START (DBAY0_START+4)
#define LUT0_START  (RGBY0_START+4)
#define CMPR0_START (LUT0_START+4)
#define DNSY0_START (CMPR0_START+8)
#define YOUT0_START (DNSY0_START+4) // ISP_OUTPUT0

#define DBAY1_START (HDR_START+8)
#define RGBY1_START (DBAY1_START+4)
#define LUT1_START  (RGBY1_START+4)
#define CMPR1_START (LUT1_START+4)
#define DNSY1_START (CMPR1_START+8)
#define YOUT1_START (DNSY1_START+4) // ISP_OUTPUT1

#define DNSC_START  (CMPR1_START+8)
#define UVOUT_START (DNSC_START+4)  // ISP_OUTPUTUV_1

void StaticSequencing(void)
{
  sStaticSeq = STAT_SEQ_MIPI;

  initIPUnode(15,  ILUT_START, 2, 0); /* InLUT */
  initIPUnode(2,   HIST_START, 8, 0); /* Histogram */
  initIPUnode(14,  HDR_START,  2, 1); /* HDR */

  initIPUnode(3,  DBAY0_START, 4, 0); /* Debayer0 */
  initIPUnode(5,  RGBY0_START, 4, 1); /* RGB2Y0 */
  initIPUnode(7,  LUT0_START,  4, 1); /* LUT4K0 */
  initIPUnode(9,  CMPR0_START, 4, 0); /* COMPRESS0 */
  initIPUnode(11, DNSY0_START, 4, 0); /* DENOISE0 */
  initFDMAnode(2, YOUT0_START, 4);  /* Y0 Output */
  /*initFDMAnode(3, CAMOUT_START, 0x2);*//* UV0 Output */

  initIPUnode(4,  DBAY1_START, 4, 0); /* Debayer1 */
  initIPUnode(6,  RGBY1_START, 4, 1); /* RGB2Y1 */
  initIPUnode(8,  LUT1_START,  4, 1); /* LUT4K1 */
  initIPUnode(10,  CMPR1_START, 4, 0);  /* COMPRESS1 */
  initIPUnode(12, DNSY1_START, 4, 0); /* DENOISE1 */
  initFDMAnode(3, YOUT1_START, 4);  /* Y1 Output */
  /*initFDMAnode(4, FDMA4_START, 0x2);*//* UV1 Output */

  initIPUnode(13, DNSC_START,  4, 1); /* DENOISE1 */
  initFDMAnode(1, UVOUT_START, 4);  /* UV0 Output */

  initFDMAnode(0, CAMOUT_START,1);  /* CAM Dump */
} // StaticSequencing()

/* EOF */
