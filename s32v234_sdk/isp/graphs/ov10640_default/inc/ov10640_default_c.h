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


#pragma once

/*****************************************************************************
* consts
*****************************************************************************/
#define   CAM_MIPI  1
#define   CAM_SAVE  1
#define   EMBEDDED_DATA  1
#define   MIPI_PORT  0
#define   OV10640  1

enum {
  FDMA_IX_ISP_OUTPUT0,
  FDMA_IX_ISP_OUTPUT1,
  FDMA_IX_ISP_OUTPUT_UV0,
  FDMA_IX_CAM_OUTPUT,
  FDMA_IX_EMBD_REGS,
  FDMA_IX_EMBD_STATS,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	HDRBuffer0;
extern SEQ_Buf_t	DENOISEY2_0Buffer0;
extern SEQ_Buf_t	Debayer0Buffer0;
extern SEQ_Buf_t	Debayer0Buffer1;
extern SEQ_Buf_t	Debayer0Buffer2;
extern SEQ_Buf_t	Debayer1Buffer0;
extern SEQ_Buf_t	Debayer1Buffer1;
extern SEQ_Buf_t	Debayer1Buffer2;
extern SEQ_Buf_t	DENOISEY2_1Buffer0;
extern SEQ_Buf_t	RGB2Y0Buffer0;
extern SEQ_Buf_t	LUT4K0Buffer0;
extern SEQ_Buf_t	RGB2Y1Buffer0;
extern SEQ_Buf_t	LUT4K1Buffer0;
extern SEQ_Buf_t	COMPRESS0_Buffer_0;
extern SEQ_Buf_t	COMPRESS1Buffer0;
extern SEQ_Buf_t	COMPRESS1Buffer1;
extern SEQ_Buf_t	COMPRESS0_Buffer_1;
extern SEQ_Buf_t	DENOISE_UVBuffer0;
extern SEQ_Buf_t	DecompressBuffer0;
extern SEQ_Buf_t	OV_QUADBuffer0;
extern SEQ_Buf_t	ISP_OUTPUT0Buffer;
extern SEQ_Buf_t	ISP_OUTPUT1Buffer;
extern SEQ_Buf_t	ISP_OUTPUT_UV0Buffer;
extern SEQ_Buf_t	CAM_OUTPUTBuffer;
extern SEQ_Buf_t	EMBD_REGSBuffer;
extern SEQ_Buf_t	EMBD_STATSBuffer;
