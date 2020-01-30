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
*  \file canny_acf.h
*  \brief ACF Metadata  for Canny edge detector
***********************************************************************************/
#ifndef CANNY_H
#define CANNY_H
#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define CANNY_NON_MAXIMA_SUPPRESS_K               canny_non_maxima_suppress
#define CANNY_NON_MAXIMA_SUPPRESS_KN              XSTR(CANNY_NON_MAXIMA_SUPPRESS_K)
#define CANNY_NMS_PROMOTE_K                       canny_nms_promote
#define CANNY_NMS_PROMOTE_KN                      XSTR(CANNY_NMS_PROMOTE_K)
#define CANNY_PROMOTE_EDGES_K                     canny_promote_edges
#define CANNY_PROMOTE_EDGES_KN                    XSTR(CANNY_PROMOTE_EDGES_K)
#define CANNY_PROMOTE_EDGES_FULL_K                canny_promote_edges_full
#define CANNY_PROMOTE_EDGES_FULL_KN               XSTR(CANNY_PROMOTE_EDGES_FULL_K)
#define CANNY_CREATE_IMAGE_K                      canny_create_image
#define CANNY_CREATE_IMAGE_KN                     XSTR(CANNY_CREATE_IMAGE_K)
#define INPUT                           "INPUT"
#define LOW_THRESH                      "LOW_THRESH"
#define HIGH_THRESH                     "HIGH_THRESH"
#define OUTPUT                          "OUTPUT"

extKernelInfoDecl(CANNY_NON_MAXIMA_SUPPRESS_K);
extKernelInfoDecl(CANNY_NMS_PROMOTE_K);
extKernelInfoDecl(CANNY_PROMOTE_EDGES_K);
extKernelInfoDecl(CANNY_PROMOTE_EDGES_ITERATIONS_K);
extKernelInfoDecl(CANNY_PROMOTE_EDGES_FULL_K);
extKernelInfoDecl(CANNY_CREATE_IMAGE_K);

#endif /* ACF_KERNEL_METADATA */
#endif /* CANNY_H */
