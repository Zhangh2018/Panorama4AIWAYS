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

enum {
  FDMA_IX_ISP_UYVY_OUTPUT_SONY_DUALEXP,
  FDMA_IX_END_SONY_DUALEXP
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec_sony_dualexp[];
extern char sequencer_srec_sony_dualexp[];

extern SEQ_Head_Ptr_t   gpGraph_sony_dualexp[];
extern GraphMetadata_t  gGraphMetadata_sony_dualexp;

extern SEQ_Buf_t	ISP_UYVY_OUTPUTBuffer_sony_dualexp;
extern SEQ_Buf_t	ConvertOutBuffer0_sony_dualexp;
extern SEQ_Buf_t	VignettingBuffer0_sony_dualexp;
extern SEQ_Buf_t	Debayer_GBuffer0_sony_dualexp;
extern SEQ_Buf_t	Debayer_GBuffer1_sony_dualexp;
extern SEQ_Buf_t	Debayer_BRBuffer2_sony_dualexp;
extern SEQ_Buf_t	Debayer_BRBuffer1_sony_dualexp;
extern SEQ_Buf_t	Debayer_BRBuffer0_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUVBuffer0_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUVBuffer1_sony_dualexp;
extern SEQ_Buf_t	RGB_2_YUVBuffer2_sony_dualexp;
extern SEQ_Buf_t	CAM0Buffer0_sony_dualexp;
extern SEQ_Buf_t	ChGain_ExposureBuffer0_sony_dualexp;
extern SEQ_Buf_t	DenoiseFilterBuffer0_sony_dualexp;
