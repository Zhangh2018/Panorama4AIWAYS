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
*  @brief ACF metadata for the \ref secHarrisCorner "harris corner".
***********************************************************************************/

#ifndef HARRISACF_H
#define HARRISACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HARRIS_CORNERS_BOX7_NMS5_K                harris_corners_box7_nms5
#define HARRIS_CORNERS_BOX7_NMS5_KN               XSTR(HARRIS_CORNERS_BOX7_NMS5_K)
#define HARRIS_CORNERS_BOX5_NMS5_K                harris_corners_box5_nms5
#define HARRIS_CORNERS_BOX5_NMS5_KN               XSTR(HARRIS_CORNERS_BOX5_NMS5_K)
#define HARRIS_CORNERS_BOX3_NMS5_K                harris_corners_box3_nms5
#define HARRIS_CORNERS_BOX3_NMS5_KN               XSTR(HARRIS_CORNERS_BOX3_NMS5_K)
#define HARRIS_EXTRACT_K                          harris_extract
#define HARRIS_EXTRACT_KN                         XSTR(HARRIS_EXTRACT_K)
#define HARRIS_SORT_AND_FILTER_K                  harris_sort_and_filter
#define HARRIS_SORT_AND_FILTER_KN                 XSTR(HARRIS_SORT_AND_FILTER_K)
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
#define COORD                           "COORD"
#define STREN                           "STREN"
#define COUNT                           "COUNT"
#define COORD1                          "COORD1"
#define STREN1                          "STREN1"
#define SRC                             "SRC"
#define FEATURE                         "FEATURE"
#define FEAT_OUT                        "FEAT_OUT"
#define INDEX                           "INDEX"

extKernelInfoDecl(HARRIS_CORNERS_BOX7_NMS5_K);
extKernelInfoDecl(HARRIS_CORNERS_BOX5_NMS5_K);
extKernelInfoDecl(HARRIS_CORNERS_BOX3_NMS5_K);
extKernelInfoDecl(HARRIS_EXTRACT_K);
extKernelInfoDecl(HARRIS_SORT_AND_FILTER_K);

#endif /* HARRISACF_H */

#endif /* HARRIS_ACF_H */
