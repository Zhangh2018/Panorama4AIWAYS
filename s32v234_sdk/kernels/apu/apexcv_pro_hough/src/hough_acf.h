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

/*******************************************************************************
 *  \file hough_acf.h
 *  \brief ACF metadata for Hough Transform
 ******************************************************************************/
#ifndef ACFHOUGH_H
#define ACFHOUGH_H

#ifdef ACF_KERNEL_METADATA
#include "common_stringify_macros.h"
#define HOUGH_INIT_PIXEL_OFFSET_K                 hough_init_pixel_offset
#define HOUGH_INIT_PIXEL_OFFSET_KN                XSTR(HOUGH_INIT_PIXEL_OFFSET_K)
#define HOUGH_INIT_CU_INDEX_K                     hough_init_cu_index
#define HOUGH_INIT_CU_INDEX_KN                    XSTR(HOUGH_INIT_CU_INDEX_K)
#define HOUGH_PIXEL_THRESHOLD_K                   hough_pixel_threshold
#define HOUGH_PIXEL_THRESHOLD_KN                  XSTR(HOUGH_PIXEL_THRESHOLD_K)
#define HOUGH_SCALARIZE_PIXELS_K                  hough_scalarize_pixels
#define HOUGH_SCALARIZE_PIXELS_KN                 XSTR(HOUGH_SCALARIZE_PIXELS_K)
#define HOUGH_ACCUMULATE_K                        hough_accumulate
#define HOUGH_ACCUMULATE_KN                       XSTR(HOUGH_ACCUMULATE_K)
#define HOUGH_GET_LINES_K                         hough_get_lines
#define HOUGH_GET_LINES_KN                        XSTR(HOUGH_GET_LINES_K)
#define OFFSET_X                        "OFFSET_X"
#define OFFSET_Y                        "OFFSET_Y"
#define PARAMS                          "PARAMS"
#define CU_INDEX                        "CU_INDEX"
#define PIXELS_X                        "PIXELS_X"
#define PIXELS_Y                        "PIXELS_Y"
#define PIXEL_COUNT                     "PIXEL_COUNT"
#define IMAGE                           "IMAGE"
#define THRESHOLD                       "THRESHOLD"
#define SCL_PIXELS_X                    "SCL_PIXELS_X"
#define SCL_PIXELS_Y                    "SCL_PIXELS_Y"
#define SCL_PIXEL_COUNT                 "SCL_PIXEL_COUNT"
#define VEC_PIXELS_X                    "VEC_PIXELS_X"
#define VEC_PIXELS_Y                    "VEC_PIXELS_Y"
#define VEC_PIXEL_COUNT                 "VEC_PIXEL_COUNT"
#define CU_COUNT                        "CU_COUNT"
#define ACCUMULATOR                     "ACCUMULATOR"
#define COS_TABLE                       "COS_TABLE"
#define SIN_TABLE                       "SIN_TABLE"
#define RHO_COUNT                       "RHO_COUNT"
#define LINES                           "LINES"
#define LINE_COUNT                      "LINE_COUNT"

extKernelInfoDecl(HOUGH_INIT_PIXEL_OFFSET_K);
extKernelInfoDecl(HOUGH_INIT_CU_INDEX_K);
extKernelInfoDecl(HOUGH_PIXEL_THRESHOLD_K);
extKernelInfoDecl(HOUGH_SCALARIZE_PIXELS_K);
extKernelInfoDecl(HOUGH_ACCUMULATE_K);
extKernelInfoDecl(HOUGH_GET_LINES_K);

#endif /* ACF_KERNEL_METADATA */
#endif  /* ACFHOUGH_H */
