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
  FDMA_IX_FastDMA_Right_Out,
  FDMA_IX_FastDMA_Left_Out,
  FDMA_IX_FastDMA_LeftGs_Out,
  FDMA_IX_FastDMA_RightGs_Out,
  FDMA_IX_FastDMA_RgbSub_Out,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	Debayer_RGB_0Buffer0;
extern SEQ_Buf_t	FastDMA_Right_OutBuffer;
extern SEQ_Buf_t	MIPI1Buffer0;
extern SEQ_Buf_t	FastDMA_Left_OutBuffer;
extern SEQ_Buf_t	Debayer_GBuffer0;
extern SEQ_Buf_t	FastDMA_LeftGs_OutBuffer;
extern SEQ_Buf_t	FastDMA_RightGs_OutBuffer;
extern SEQ_Buf_t	FastDMA_RgbSub_OutBuffer;
extern SEQ_Buf_t	Debayer_rgb_subsampledBuffer0;
