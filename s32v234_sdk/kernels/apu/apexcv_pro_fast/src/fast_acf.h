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

#ifndef FASTACF_H
#define FASTACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define FAST_OFFSET_K                             fast_offset
#define FAST_OFFSET_KN                            XSTR(FAST_OFFSET_K)
#define FAST_K                                    fast
#define FAST_KN                                   XSTR(FAST_K)
#define FAST_SERIALIZED_K                         fast_serialized
#define FAST_SERIALIZED_KN                        XSTR(FAST_SERIALIZED_K)
#define FAST_NMS_K                                fast_nms
#define FAST_NMS_KN                               XSTR(FAST_NMS_K)
#define NMS3X3_K                                  nms3x3
#define NMS3X3_KN                                 XSTR(NMS3X3_K)
#define OUTPUT_OFFSETS                  "OUTPUT_OFFSETS"
#define INPUT_0                         "INPUT_0"
#define CIRCUMFERENCE                   "CIRCUMFERENCE"
#define OUTPUT_0                        "OUTPUT_0"
#define THRESHOLD                       "THRESHOLD"
#define OFFSET_TABLE                    "OFFSET_TABLE"
#define INPUT                           "INPUT"
#define OUT_PACKED                      "OUT_PACKED"
#define COUNTER                         "COUNTER"
#define OUT_MAX_SIZE                    "OUT_MAX_SIZE"
#define input                           "input"
#define output                          "output"

extKernelInfoDecl(FAST_OFFSET_K);
extKernelInfoDecl(FAST_K);
extKernelInfoDecl(FAST_SERIALIZED_K);
extKernelInfoDecl(FAST_NMS_K);
extKernelInfoDecl(NMS3X3_K);

#endif //#ifdef ACF_KERNEL_METADATA

#endif /* FASTACF_H */
