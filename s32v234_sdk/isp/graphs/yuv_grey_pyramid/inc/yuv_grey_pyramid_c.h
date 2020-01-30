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
  FDMA_IX_YUV_TO_DDR,
  FDMA_IX_Y2_SCALED_TO_DDR,
  FDMA_IX_Y4_SCALED_TO_DDR,
  FDMA_IX_Y8_SCALED_TO_DDR,
  FDMA_IX_Y16_SCALED_TO_DDR,
  FDMA_IX_Y32_SCALED_TO_DDR,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	YUV_TO_DDRBuffer;
extern SEQ_Buf_t	ConvertYUVBuffer0;
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
extern SEQ_Buf_t	Y2_SCALED_TO_DDRBuffer;
extern SEQ_Buf_t	Y2_DOWNSCALEBuffer0;
extern SEQ_Buf_t	Y4_DOWNSCALEBuffer0;
extern SEQ_Buf_t	Y4_SCALED_TO_DDRBuffer;
extern SEQ_Buf_t	Y8_DOWNSCALEBuffer0;
extern SEQ_Buf_t	Y8_SCALED_TO_DDRBuffer;
extern SEQ_Buf_t	Y16_DOWNSCALEBuffer0;
extern SEQ_Buf_t	Y16_SCALED_TO_DDRBuffer;
extern SEQ_Buf_t	Y32_DOWNSCALEBuffer0;
extern SEQ_Buf_t	Y32_SCALED_TO_DDRBuffer;
