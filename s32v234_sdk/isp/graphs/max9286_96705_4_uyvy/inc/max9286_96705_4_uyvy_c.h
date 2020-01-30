/*****************************************************************************
 *
 * Copyright 2018 NXP
 * All Rights Reserved
 *
 *****************************************************************************
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

// Generated from:
// C:/NXP/VisionSDK_S32V2_RTM_1_2_0_Exchange/s32v234_sdk/nra_isp/mbox/isp/graphs/max9286_96705_4_uyvy/max9286_96705_4_uyvy/model/max9286_96705_4_uyvy.isp 

#pragma once

/*****************************************************************************
* consts
*****************************************************************************/

enum {
  FDMA_IX_FastDMA_Out_SV1,
  FDMA_IX_FastDMA_Out_SV2,
  FDMA_IX_FastDMA_Out_SV3,
  FDMA_IX_FastDMA_Out_SV4,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	SRAMBuffer_MIPI;
extern SEQ_Buf_t	SRAMBuffer_SV1;
extern SEQ_Buf_t	SRAMBuffer_SV2;
extern SEQ_Buf_t	SRAMBuffer_SV3;
extern SEQ_Buf_t	SRAMBuffer_SV4;
extern SEQ_Buf_t	DDRBuffer_SV1;
extern SEQ_Buf_t	DDRBuffer_SV2;
extern SEQ_Buf_t	DDRBuffer_SV3;
extern SEQ_Buf_t	DDRBuffer_SV4;
