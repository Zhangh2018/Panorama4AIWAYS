/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
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
#ifndef RESIZEACF_H
#define RESIZEACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HRSZ16_K                                  hrsz16
#define HRSZ16_KN                                 XSTR(HRSZ16_K)
#define VRSZ16_K                                  vrsz16
#define VRSZ16_KN                                 XSTR(VRSZ16_K)
#define HRSZ_K                                    hrsz
#define HRSZ_KN                                   XSTR(HRSZ_K)
#define VRSZ_K                                    vrsz
#define VRSZ_KN                                   XSTR(VRSZ_K)
#define SRC                             "SRC"
#define DST                             "DST"
#define SRC_WIDTH                       "SRC_WIDTH"
#define DST_WIDTH                       "DST_WIDTH"
#define HRSZ_DESC                       "HRSZ_DESC"
#define SRC_HEIGHT                      "SRC_HEIGHT"
#define DST_HEIGHT                      "DST_HEIGHT"
#define VRSZ_DESC                       "VRSZ_DESC"

extKernelInfoDecl(HRSZ_K);
extKernelInfoDecl(VRSZ_K);
extKernelInfoDecl(HRSZ16_K);
extKernelInfoDecl(VRSZ16_K);


#endif //#ifdef ACF_KERNEL_METADATA

#endif /* RESIZEACF_H */
