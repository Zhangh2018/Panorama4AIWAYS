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
/*!*****************************************************************************
*  @file
*  @brief ACF metadata for the \ref secBrief "BRIEF".
*******************************************************************************/

#ifndef BRIEFWRAPPEDFORACF_H
#define BRIEFWRAPPEDFORACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define COMPUTE_BRIEF_DESCRIPTOR_K                compute_brief_descriptor
#define COMPUTE_BRIEF_DESCRIPTOR_KN               XSTR(COMPUTE_BRIEF_DESCRIPTOR_K)
#define INPUT                           "INPUT"
#define FILTER_TYPE                     "FILTER_TYPE"
#define SMPL_PACKET                     "SMPL_PACKET"
#define NR_PACKETS_UL                   "NR_PACKETS_UL"
#define NR_PACKETS_UR                   "NR_PACKETS_UR"
#define NR_PACKETS_LL                   "NR_PACKETS_LL"
#define NR_PACKETS_LR                   "NR_PACKETS_LR"
#define PATCH_SIZE                      "PATCH_SIZE"
#define DESC_SIZE                       "DESC_SIZE"
#define COUNT                           "COUNT"
#define DESCRIPTOR_OUT                  "DESCRIPTOR_OUT"

extKernelInfoDecl(COMPUTE_BRIEF_DESCRIPTOR_K);
#endif /* ACF_KERNEL_METADATA */

#endif /* BRIEFWRAPPEDFORACF_H */
