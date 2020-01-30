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
  FDMA_IX_RGB888,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	ChGain_ExposureBuffer0;
extern SEQ_Buf_t	Debayer_GBuffer0;
extern SEQ_Buf_t	Debayer_GBuffer1;
extern SEQ_Buf_t	Debayer_BRBuffer2;
extern SEQ_Buf_t	Debayer_BRBuffer1;
extern SEQ_Buf_t	Debayer_BRBuffer0;
extern SEQ_Buf_t	RGB_2_YUVBuffer0;
extern SEQ_Buf_t	RGB_2_YUVBuffer1;
extern SEQ_Buf_t	RGB_2_YUVBuffer2;
extern SEQ_Buf_t	CAM0Buffer0;
extern SEQ_Buf_t	H264_EncoderBuffer0;
extern SEQ_Buf_t	U_DownScale2Buffer0;
extern SEQ_Buf_t	V_DownScale2Buffer0;
extern SEQ_Buf_t	rgb888_planar2interleavedBuffer0;
extern SEQ_Buf_t	RGB888Buffer;
