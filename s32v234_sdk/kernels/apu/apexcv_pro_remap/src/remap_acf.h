/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

#ifndef REMAPACF_H
#define REMAPACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define REMAP_BILINEAR_RGB_K            _remap_bilinear_rgb
#define REMAP_BILINEAR_RGB_KN      		XSTR(REMAP_BILINEAR_RGB_K)
#define REMAP_BILINEAR_GREYSCALE_K      _remap_bilinear_greyscale
#define REMAP_BILINEAR_GREYSCALE_KN     XSTR(REMAP_BILINEAR_GREYSCALE_K)
#define REMAP_BLOCK_SIZE_K              _remap_block_size
#define REMAP_BLOCK_SIZE_KN      		XSTR(REMAP_BLOCK_SIZE_K)
#define DST                             "DST"
#define SRC                             "SRC"
#define OFFSET                          "OFFSET"
#define DELTA                           "DELTA"
#define MAP                             "MAP"
#define TWO_HEIGHT                      "TWO_HEIGHT"
#define FOUR_HEIGHT                     "FOUR_HEIGHT"
#define EIGHT_HEIGHT                    "EIGHT_HEIGHT"
#define SIXTEEN_HEIGHT                  "SIXTEEN_HEIGHT"

extKernelInfoDecl(REMAP_BILINEAR_RGB_K);
extKernelInfoDecl(REMAP_BILINEAR_GREYSCALE_K);
extKernelInfoDecl(REMAP_BLOCK_SIZE_K);

#endif //#ifdef ACF_KERNEL_METADATA

#endif /* REMAPACF_H */
