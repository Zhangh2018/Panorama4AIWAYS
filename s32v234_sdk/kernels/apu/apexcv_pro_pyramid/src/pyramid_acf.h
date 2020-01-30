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
*  @brief ACF metadata for the \ref secImagePyramid "image pyramid creation".
***********************************************************************************/
#ifndef PYRAMIDWRAPPEDFORACF_H
#define PYRAMIDWRAPPEDFORACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HORIZONTAL_GAUS_K                         horizontal_gaus
#define HORIZONTAL_GAUS_KN                        XSTR(HORIZONTAL_GAUS_K)
#define HORIZONTAL_GAUS_AND_EXPAND_K              horizontal_gaus_and_expand
#define HORIZONTAL_GAUS_AND_EXPAND_KN             XSTR(HORIZONTAL_GAUS_AND_EXPAND_K)
#define VERTICAL_GAUS_K                           vertical_gaus
#define VERTICAL_GAUS_KN                          XSTR(VERTICAL_GAUS_K)
#define VERTICAL_GAUS_AND_REDUCE_K                vertical_gaus_and_reduce
#define VERTICAL_GAUS_AND_REDUCE_KN               XSTR(VERTICAL_GAUS_AND_REDUCE_K)
#define INPUT                           "INPUT"
#define OUTPUT                          "OUTPUT"

extKernelInfoDecl(HORIZONTAL_GAUS_K);
extKernelInfoDecl(HORIZONTAL_GAUS_AND_EXPAND_K);
extKernelInfoDecl(VERTICAL_GAUS_K);
extKernelInfoDecl(VERTICAL_GAUS_AND_REDUCE_K);

#endif /* ACF_KERNEL_METADATA */

#endif /* PYRAMIDWRAPPEDFORACF_H */
