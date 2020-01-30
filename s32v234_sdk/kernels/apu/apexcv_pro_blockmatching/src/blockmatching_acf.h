/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright 2017-2018 NXP 
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
#ifndef BLOCKMATCHINGACF_H
#define BLOCKMATCHINGACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define APU_BM_SAD_K                              apu_bm_sad
#define APU_BM_SAD_KN                             XSTR(APU_BM_SAD_K)
#define INPUT_TEMPLATE                  "INPUT_TEMPLATE"
#define INPUT_WINDOW                    "INPUT_WINDOW"
#define OUTPUT_SCORE                    "OUTPUT_SCORE"
#define OUTPUT_POINTS                   "OUTPUT_POINTS"

extern  KERNEL_INFO _kernel_info_apu_bm_sad;

#endif //#ifdef ACF_KERNEL_METADATA

#endif /* BLOCKMATCHINGACF_H */
