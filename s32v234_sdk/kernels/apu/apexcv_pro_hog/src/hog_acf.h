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
#ifndef HOGACF_H
#define HOGACF_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HOG_GET_GRADIENT_K                        hog_get_gradient
#define HOG_GET_GRADIENT_KN                       XSTR(HOG_GET_GRADIENT_K)
#define HOG_GET_CELL_HISTOGRAMS_K                 hog_get_cell_histograms
#define HOG_GET_CELL_HISTOGRAMS_KN                XSTR(HOG_GET_CELL_HISTOGRAMS_K)
#define HOG_COMPUTE_BLOCK_HISTOGRAMS_K            hog_compute_block_histograms
#define HOG_COMPUTE_BLOCK_HISTOGRAMS_KN           XSTR(HOG_COMPUTE_BLOCK_HISTOGRAMS_K)
#define HOG_GET_BLOCK_HISTOGRAMS_K                hog_get_block_histograms
#define HOG_GET_BLOCK_HISTOGRAMS_KN               XSTR(HOG_GET_BLOCK_HISTOGRAMS_K)
#define HOG_GET_SCORES_K                          hog_get_scores
#define HOG_GET_SCORES_KN                         XSTR(HOG_GET_SCORES_K)
#define HOG_CONVERT_BLOCK_HISTOGRAMS_K            hog_convert_block_histograms
#define HOG_CONVERT_BLOCK_HISTOGRAMS_KN           XSTR(HOG_CONVERT_BLOCK_HISTOGRAMS_K)
#define IMAGE                           "IMAGE"
#define GRADIENT_MAGNITUDE              "GRADIENT_MAGNITUDE"
#define GRADIENT_ANGLE_BIN              "GRADIENT_ANGLE_BIN"
#define CELL_HISTOGRAMS                 "CELL_HISTOGRAMS"
#define BLOCK_HISTOGRAMS                "BLOCK_HISTOGRAMS"
#define ROW                             "ROW"
#define SCORES                          "SCORES"
#define SVM_FUNCTION                    "SVM_FUNCTION"
#define SVM_TRANSFORM_MODE              "SVM_TRANSFORM_MODE"
#define WIN_BLOCKS                      "WIN_BLOCKS"
#define X_LUT                           "X_LUT"
#define R_LUT                           "R_LUT"
#define Y_LUT                           "Y_LUT"
#define READY_BLOCK_HISTOGRAMS          "READY_BLOCK_HISTOGRAMS"
#define SCORES_ROW_BUFFER               "SCORES_ROW_BUFFER"
#define IMAGE_BLOCK_HISTOGRAMS          "IMAGE_BLOCK_HISTOGRAMS"

extKernelInfoDecl(HOG_GET_GRADIENT_K);
extKernelInfoDecl(HOG_GET_CELL_HISTOGRAMS_K);
extKernelInfoDecl(HOG_COMPUTE_BLOCK_HISTOGRAMS_K);
extKernelInfoDecl(HOG_GET_BLOCK_HISTOGRAMS_K);
extKernelInfoDecl(HOG_GET_SCORES_K);
extKernelInfoDecl(HOG_CONVERT_BLOCK_HISTOGRAMS_K);

#endif   // #ifdef ACF_KERNEL_METADATA

#endif /* HOGACF_H */
