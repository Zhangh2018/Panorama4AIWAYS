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
#ifndef TMO_HPP
#define TMO_HPP

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define CALC_GEO_MEAN_RGBE_K                      calc_geo_mean_rgbe
#define CALC_GEO_MEAN_RGBE_KN                     XSTR(CALC_GEO_MEAN_RGBE_K)
#define CALC_GEO_MEAN_OPENEXR_K                   calc_geo_mean_openexr
#define CALC_GEO_MEAN_OPENEXR_KN                  XSTR(CALC_GEO_MEAN_OPENEXR_K)
#define LDR_TRANSFORM_K                           ldr_transform
#define LDR_TRANSFORM_KN                          XSTR(LDR_TRANSFORM_K)

extKernelInfoDecl(CALC_GEO_MEAN_RGBE_K);
extKernelInfoDecl(CALC_GEO_MEAN_OPENEXR_K);
extKernelInfoDecl(LDR_TRANSFORM_K);

#endif // ACF_KERNEL_METADATA

#endif /* TMO_HPP */
