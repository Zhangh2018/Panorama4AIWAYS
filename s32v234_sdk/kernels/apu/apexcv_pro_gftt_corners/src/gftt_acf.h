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
/*!*********************************************************************************
*  @file
*  @brief ACF metadata for the \ref secGFTT "Good Features To Track".
***********************************************************************************/

#ifndef GFTTWRAPPEDFORACF_H
#define GFTTWRAPPEDFORACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define GFTT_WRAPPER_BOX7_NMS5_K                  gftt_wrapper_box7_nms5
#define GFTT_WRAPPER_BOX7_NMS5_KN                 XSTR(GFTT_WRAPPER_BOX7_NMS5_K)
#define GFTT_WRAPPER_BOX5_NMS5_K                  gftt_wrapper_box5_nms5
#define GFTT_WRAPPER_BOX5_NMS5_KN                 XSTR(GFTT_WRAPPER_BOX5_NMS5_K)
#define GFTT_WRAPPER_BOX3_NMS5_K                  gftt_wrapper_box3_nms5
#define GFTT_WRAPPER_BOX3_NMS5_KN                 XSTR(GFTT_WRAPPER_BOX3_NMS5_K)
#define GFTT_EXTRACT_K                            gftt_extract
#define GFTT_EXTRACT_KN                           XSTR(GFTT_EXTRACT_K)
#define GFTT_SORT_AND_FILTER_K                    gftt_sort_and_filter
#define GFTT_SORT_AND_FILTER_KN                   XSTR(GFTT_SORT_AND_FILTER_K)
#define INPUT                           "INPUT"
#define PARAMS                          "PARAMS"
#define OUTPUT                          "OUTPUT"
#define SVXX                            "SVXX"
#define SVXY                            "SVXY"
#define SVYY                            "SVYY"
#define NMS_X                           "NMS_X"
#define NMS                             "NMS"
#define BXX                             "BXX"
#define BXY                             "BXY"
#define BYY                             "BYY"
#define MAX_EIGEN                       "MAX_EIGEN"
#define COORD                           "COORD"
#define STREN                           "STREN"
#define COUNT                           "COUNT"
#define LOCAL_COORD                     "LOCAL_COORD"
#define LOCAL_STREN                     "LOCAL_STREN"
#define SRC                             "SRC"
#define FEATURE                         "FEATURE"
#define FEAT_COUNT                      "FEAT_COUNT"
#define INDEX                           "INDEX"

extKernelInfoDecl(GFTT_WRAPPER_BOX7_NMS5_K);
extKernelInfoDecl(GFTT_WRAPPER_BOX5_NMS5_K);
extKernelInfoDecl(GFTT_WRAPPER_BOX3_NMS5_K);
extKernelInfoDecl(GFTT_EXTRACT_K);
extKernelInfoDecl(GFTT_SORT_AND_FILTER_K);

#endif /* ACF_KERNEL_METADATA */

#endif /* GFTTWRAPPEDFORACF_H */
