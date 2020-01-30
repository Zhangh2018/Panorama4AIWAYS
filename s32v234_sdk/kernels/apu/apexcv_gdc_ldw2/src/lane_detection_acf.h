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
#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define LANE_DETECTION_K             lane_detection
#define LANE_DETECTION_KN            XSTR(LANE_DETECTION_K)
#define SRC                             "SRC"
#define VOTE_THRESHOLD                  "VOTE_THRESHOLD"
#define POINTS_DATA                     "POINTS_DATA"
#define MAX_LEFT                        "MAX_LEFT"
#define MAX_RIGHT                       "MAX_RIGHT"

extKernelInfoDecl(LANE_DETECTION_K);

#endif //#ifdef ACF_KERNEL_METADATA

#endif /* LANEDETECTION_H */
