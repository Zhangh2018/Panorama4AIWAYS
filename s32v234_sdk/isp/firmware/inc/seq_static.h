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
 * \file    isp_static.h
 * \brief   ISP runtime firmware definitions
 * \author
 * \author
 * \version 0.1
 * \date    21-06-2016
 * \note    initial version
 ****************************************************************************/

#ifndef SEQSTATIC_H
#define SEQSTATIC_H

#include "typedefs.h"

/* *INDENT-OFF* */
#ifdef  __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

/*****************************************************************************
* consts
*****************************************************************************/

#define IPU_DEBUG_MODE    0xffffffff

#ifndef __ASSEMBLER__

/*****************************************************************************
* macros
*****************************************************************************/

#define STATIC_SEQ_ACTIVE    (STAT_SEQ_NONE != sStaticSeq)

/*****************************************************************************
* types
*****************************************************************************/

typedef enum
{
  STAT_SEQ_NONE = 0,
  STAT_SEQ_MIPI,
  STAT_SEQ_FDMA,
  STAT_SEQ_JPEG4,
  STAT_SEQ_VIU,
  STAT_SEQ_H264,
} static_seq_t;

/*****************************************************************************
* imported functions
*****************************************************************************/

extern void StaticSequencing(void);

/*****************************************************************************
* exported data
*****************************************************************************/

extern static_seq_t sStaticSeq;

/*****************************************************************************
* exported functions
*****************************************************************************/

extern void staticSequenceInit(void);
extern void staticSequence(void);
extern uint32_t staticSequenceWaitFinish(void);
extern void staticSequenceEventIPU(void);
extern void staticSequenceJpeg4(uint16_t aCnt, uint16_t aChan);
extern void     initIPUnode(uint16_t aNodeIx,
                            uint16_t aCycle,
                            uint16_t aPer,
                            uint16_t aInitReg);
extern void initFDMAnode(uint16_t aChan, uint16_t aCycle, uint16_t aPer);

#endif // __ASSEMBLER__

/* *INDENT-OFF* */
#ifdef  __cplusplus
}
#endif
/* *INDENT-ON* */

#endif /* SEQSTATIC_H */
/*EOF*/
