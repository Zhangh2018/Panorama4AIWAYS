/*****************************************************************************
 *
 * Copyright (c) 2017-2018 NXP
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// Generated from:
// X:/code/vsdk/s32v234_sdk/isp/graphs/jpeg_4stream/ispgraph/model/jpeg_4stream.isp 

#include "seq_static.h"

/*****************************************************************************
* consts
*****************************************************************************/

#define	IPUS1_START	0
#define	IPUS2_START	(IPUS1_START + 1)
#define	FDMA_OFFS	1

#define	FDMA0_START	(IPUS2_START+FDMA_OFFS)	// Chan0
#define	FDMA1_START	(IPUS2_START+FDMA_OFFS)	// Chan1
#define	FDMA2_START	(IPUS2_START+FDMA_OFFS)	// Chan2
#define	FDMA3_START	(IPUS2_START+FDMA_OFFS)	// Chan3

/****************************************************************************
 * this is a dummy StaticSequencing()
 ****************************************************************************/
/* The heart beat source is undefined - nothing to emit */
void StaticSequencing(void)
{
  sStaticSeq = STAT_SEQ_JPEG4;

  initIPUnode(1, IPUS2_START, 0x1, 0);	/* interleave_quad */
  initIPUnode(2, IPUS1_START, 0x1, 0);	/* interleave_copy_1to2 */

  initFDMAnode(0, FDMA0_START, 0x1);
  initFDMAnode(1, FDMA1_START, 0x1);
  initFDMAnode(2, FDMA2_START, 0x1);
  initFDMAnode(3, FDMA3_START, 0x1);
} // StaticSequencing()
/* EOF */
