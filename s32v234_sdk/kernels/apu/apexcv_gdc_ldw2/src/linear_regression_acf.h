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

#ifndef LINEARREGRESSION_H
#define LINEARREGRESSION_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define LINEAR_REGRESSION_K             linear_regression
#define LINEAR_REGRESSION_KN            XSTR(LINEAR_REGRESSION_K)
#define POINTS_DATA                     "POINTS_DATA"
#define MAX_VOTES_LEFT                  "MAX_VOTES_LEFT"
#define MAX_VOTES_RIGHT                 "MAX_VOTES_RIGHT"
#define DATA                            "DATA"

extKernelInfoDecl(LINEAR_REGRESSION_K);

#endif // end: #ifdef ACF_KERNEL_METADATA

#endif /* LINEARREGRESSION_H */
