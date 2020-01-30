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
#ifndef LBPWRAPPEDFORACF_H
#define LBPWRAPPEDFORACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define LBP_EXTRACT_DESCRIPTOR_K                  lbp_extract_descriptor
#define LBP_EXTRACT_DESCRIPTOR_KN                 XSTR(LBP_EXTRACT_DESCRIPTOR_K)
#define COMPARE_DESCRIPTOR_K                      compare_descriptor
#define COMPARE_DESCRIPTOR_KN                     XSTR(COMPARE_DESCRIPTOR_K)
#define INPUT                           "INPUT"
#define PROCESS_SIZE                    "PROCESS_SIZE"
#define OUTPUT                          "OUTPUT"
#define INPUT_MODEL                     "INPUT_MODEL"
#define INPUT_TEST                      "INPUT_TEST"
#define CLOSEST_ID                      "CLOSEST_ID"
#define DISTANCE                        "DISTANCE"

extKernelInfoDecl(LBP_EXTRACT_DESCRIPTOR_K);
extKernelInfoDecl(COMPARE_DESCRIPTOR_K);

#endif // ACF_KERNEL_METADATA

#endif /* LBPWRAPPEDFORACF_H */
