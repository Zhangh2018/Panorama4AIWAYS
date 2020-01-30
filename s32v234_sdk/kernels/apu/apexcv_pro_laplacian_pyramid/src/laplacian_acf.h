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
*  @brief ACF metadata for the Laplacian Pyramid
***********************************************************************************/
#ifndef ACFLAPLACIAN_H
#define ACFLAPLACIAN_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HORIZONTAL_GAUS_LAPLACIAN_MID_K           horizontal_gaus_laplacian_mid
#define HORIZONTAL_GAUS_LAPLACIAN_MID_KN          XSTR(HORIZONTAL_GAUS_LAPLACIAN_MID_K)
#define HORIZONTAL_GAUS_LAPLACIAN_LAST_K          horizontal_gaus_laplacian_last
#define HORIZONTAL_GAUS_LAPLACIAN_LAST_KN         XSTR(HORIZONTAL_GAUS_LAPLACIAN_LAST_K)
#define VERTICAL_GAUS_LAPLACIAN_MID_K             vertical_gaus_laplacian_mid
#define VERTICAL_GAUS_LAPLACIAN_MID_KN            XSTR(VERTICAL_GAUS_LAPLACIAN_MID_K)
#define VERTICAL_GAUS_LAPLACIAN_LAST_K            vertical_gaus_laplacian_last
#define VERTICAL_GAUS_LAPLACIAN_LAST_KN           XSTR(VERTICAL_GAUS_LAPLACIAN_LAST_K)
#define INPUT                           "INPUT"
#define OUTPUT                          "OUTPUT"
#define INPUT_GAUSS                     "INPUT_GAUSS"
#define OUTPUT_NEXT                     "OUTPUT_NEXT"
#define OUTPUT_REC                      "OUTPUT_REC"

extKernelInfoDecl(HORIZONTAL_GAUS_LAPLACIAN_MID_K);
extKernelInfoDecl(HORIZONTAL_GAUS_LAPLACIAN_LAST_K);
extKernelInfoDecl(VERTICAL_GAUS_LAPLACIAN_MID_K);
extKernelInfoDecl(VERTICAL_GAUS_LAPLACIAN_LAST_K);

#endif /* ACF_KERNEL_METADATA */

#endif /* ACFLAPLACIAN_H */
