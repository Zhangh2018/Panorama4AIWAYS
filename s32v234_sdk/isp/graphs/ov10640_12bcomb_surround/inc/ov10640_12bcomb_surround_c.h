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
  FDMA_IX_ISP_OUTPUT0,
  FDMA_IX_ISP_OUTPUT1,
  FDMA_IX_END
};

/*****************************************************************************
* exported variables
*****************************************************************************/
extern char kmem_srec[];
extern char sequencer_srec[];

extern SEQ_Head_Ptr_t   gpGraph[];
extern GraphMetadata_t  gGraphMetadata;

extern SEQ_Buf_t	ISP_OUTPUT0Buffer;
extern SEQ_Buf_t	OV_QUADBuffer0;
extern SEQ_Buf_t	COMPRESS0Buffer0;
extern SEQ_Buf_t	Debayer0Buffer0;
extern SEQ_Buf_t	Debayer0Buffer2;
extern SEQ_Buf_t	Debayer0Buffer1;
extern SEQ_Buf_t	Debayer1Buffer0;
extern SEQ_Buf_t	Debayer1Buffer2;
extern SEQ_Buf_t	Debayer1Buffer1;
extern SEQ_Buf_t	COMPRESS1Buffer0;
extern SEQ_Buf_t	ISP_OUTPUT1Buffer;
extern SEQ_Buf_t	RGB2Y0Buffer0;
extern SEQ_Buf_t	LUT4K0Buffer0;
extern SEQ_Buf_t	RGB2Y1Buffer0;
extern SEQ_Buf_t	LUT4K1Buffer0;
