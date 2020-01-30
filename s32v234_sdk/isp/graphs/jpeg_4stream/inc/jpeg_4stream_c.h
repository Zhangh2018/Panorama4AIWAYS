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

#pragma once

/*****************************************************************************
* consts
*****************************************************************************/

enum {
  FDMA_IX_UYVY_0,
  FDMA_IX_UYVY_1,
  FDMA_IX_UYVY_2,
  FDMA_IX_UYVY_3,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	JpegDecBuffer1;
extern SEQ_Buf_t	JpegDecBuffer2;
extern SEQ_Buf_t	JpegDecBuffer0;
extern SEQ_Buf_t	Yuv420ToInterleavedBuffer0;
extern SEQ_Buf_t	Yuv420ToInterleavedBuffer1;
extern SEQ_Buf_t	UYVY_0Buffer;
extern SEQ_Buf_t	UYVY_1Buffer;
extern SEQ_Buf_t	Yuv420ToInterleavedBuffer2;
extern SEQ_Buf_t	Yuv420ToInterleavedBuffer3;
extern SEQ_Buf_t	UYVY_2Buffer;
extern SEQ_Buf_t	UYVY_3Buffer;
extern SEQ_Buf_t	JpegDecBuffer3;
extern SEQ_Buf_t	SRAMBuffer_0;
