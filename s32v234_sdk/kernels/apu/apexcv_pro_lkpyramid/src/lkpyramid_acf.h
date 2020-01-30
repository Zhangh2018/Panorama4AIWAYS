/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor
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

/*!*********************************************************************************
 * *  @file
 * *  @brief ACF metadata for the L-K optical flow pyramid
 * ***********************************************************************************/
#ifndef LKPYRAMIDACF_H
#define LKPYRAMIDACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define LKPYRAMID_TEMPL_BILINEAR_08U_7X7_K        lkpyramid_templ_bilinear_08u_7x7
#define LKPYRAMID_TEMPL_BILINEAR_08U_7X7_KN       XSTR(LKPYRAMID_TEMPL_BILINEAR_08U_7X7_K)
#define LKPYRAMID_HT_CENTRALDXDY_7X7_K            lkpyramid_ht_centraldxdy_7x7
#define LKPYRAMID_HT_CENTRALDXDY_7X7_KN           XSTR(LKPYRAMID_HT_CENTRALDXDY_7X7_K)
#define LKPYRAMID_CORE_7X7_K                      lkpyramid_core_7x7
#define LKPYRAMID_CORE_7X7_KN                     XSTR(LKPYRAMID_CORE_7X7_K)
#define INPUT_0                         "INPUT_0"
#define INPUT_1                         "INPUT_1"
#define OUTPUT_0                        "OUTPUT_0"
#define OUTPUT_1                        "OUTPUT_1"
#define INPUT_2                         "INPUT_2"
#define INPUT_3                         "INPUT_3"
#define INPUT_4                         "INPUT_4"
#define INPUT_5                         "INPUT_5"
#define INPUT_6                         "INPUT_6"
#define INPUT_7                         "INPUT_7"
#define OUTPUT_2                        "OUTPUT_2"
#define OUTPUT_3                        "OUTPUT_3"
#define OUTPUT_4                        "OUTPUT_4"

extKernelInfoDecl(LKPYRAMID_TEMPL_BILINEAR_08U_7X7_K);
extKernelInfoDecl(LKPYRAMID_HT_CENTRALDXDY_7X7_K);
extKernelInfoDecl(LKPYRAMID_CORE_7X7_K);

#endif /* ACF_KERNEL_METADATA */

#endif /* LKPYRAMIDACF_H */
