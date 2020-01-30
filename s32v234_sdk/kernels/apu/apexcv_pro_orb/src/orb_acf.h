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

/*******************************************************************************
 *  \file orb_acf.h
 *  \brief ACF metadata for Orb kernels
 ******************************************************************************/

#ifndef ORBACF_H
#define ORBACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define ORB_ORIENTATION_K orb_orientation
#define ORB_ORIENTATION_KN XSTR(ORB_ORIENTATION_K)

#define ORB_RBRIEF_K orb_rbrief
#define ORB_RBRIEF_KN XSTR(ORB_RBRIEF_K)
#define INPUT                           "INPUT"
#define PATCH_SIZE                      "PATCH_SIZE"
#define RADIUS                          "RADIUS"
#define OUTPUT                          "OUTPUT"
#define ORIENTATION                     "ORIENTATION"
#define BIT_PATTERN                     "BIT_PATTERN"
#define DESCRIPTOR_SIZE_B               "DESCRIPTOR_SIZE_B"
#define COUNT                           "COUNT"

extKernelInfoDecl(ORB_ORIENTATION_K);
extKernelInfoDecl(ORB_RBRIEF_K);

#endif /* ACF_KERNEL_METADATA */

#endif  /* ORBACF_H */
