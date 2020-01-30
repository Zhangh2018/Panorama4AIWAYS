/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017  NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file    iofiles.h
 * \brief   ISP sony_y graph
 * \author  Armin Strobl
 * \author
 * \version 0.1
 * \date    12-05-2015
 * \note    inital version
 ****************************************************************************/

#ifndef IOFILES_H
#define IOFILES_H

#define ISP_OUTPUT_YUV420  1
#define ISP_OUTPUT_YUV422  2
#define ISP_OUTPUT_RGB     3
#define ISP_OUTPUT_RGBY    4
#define ISP_OUTPUT         ISP_OUTPUT_YUV420


#define PIX_DEPTH          1
#define RGB_PIX_DEPTH      3
#define VS_LINE_DELAY      3
#define RED_LSSTARTPIX     0    // in values (L/S + VS)
#define RED_VSSTARTPIX     (RED_LSSTARTPIX+1)

// *****************************************************
// set geometries here
#define CAMS               (1)
#define CAM_PIXELS1        (1288)
//#define CAM_LINES          (731*2)  // MIPI: 2x(728+3)
#define CAM_LINES          (732)//(731)  // LVDS: 728+3
#define OUT_PIXELS1        (1280)
#define OUT_LINES          (720)
#define DISPLAY_PIXELS     (1280)
// *****************************************************

#define OUT_PIXELS         (OUT_PIXELS1*4)
#define OUT_STRIDE8_Y      (OUT_PIXELS)
#define OUT_STRIDE16_Y     (OUT_PIXELS*2)
#define OUT_PIXELS_RGB     (OUT_PIXELS*RGB_PIX_DEPTH)
#define OUT_STRIDE8_RGB    (OUT_PIXELS*RGB_PIX_DEPTH)
#define OUT_STRIDE16_RGB   (OUT_PIXELS*2*RGB_PIX_DEPTH)
#define OUT_PIXELS_YUV     (OUT_PIXELS*2)
#define OUT_STRIDE8_YUV    (OUT_PIXELS*2)
#define OUT_STRIDE16_YUV   (OUT_PIXELS*4)
#define OUT_PIXELS_UV      (OUT_PIXELS)
#define OUT_STRIDE8_UV     (OUT_PIXELS)
#define OUT_STRIDE16_UV    (OUT_PIXELS*2)
#define OUT_LINES2_0       (OUT_LINES>>1)
#define OUT_LINES2_1       (OUT_LINES>>1)
//#define OUT_LINESUV2_0     ((OUT_LINES+3)>>2)
//#define OUT_LINESUV2_1     ((OUT_LINES+3)>>2)
#define OUT_LINESUV2_0     ((OUT_LINES+1)>>1)
#define OUT_LINESUV2_1     ((OUT_LINES+1)>>1)
#define OUT_LINES3_0       ((OUT_LINES+2)/3)
#define OUT_LINES3_1       ((OUT_LINES+1)/3)
#define OUT_LINES3_2       ((OUT_LINES+0)/3)
#define OUT_LINES4         ((OUT_LINES)/4)

#define DISPLAY_PIXELS_RGB     (DISPLAY_PIXELS*RGB_PIX_DEPTH)
#define DISPLAY_PIXELSE_RGB    (DISPLAY_PIXELS*RGB_PIX_DEPTH)
#define DISPLAY_PIXELSO_RGB    (DISPLAY_PIXELS*RGB_PIX_DEPTH)
#define DISPLAY_STRIDE8_RGB    (DISPLAY_PIXELS*RGB_PIX_DEPTH)
#define DISPLAY_STRIDE16_RGB   (DISPLAY_PIXELS*RGB_PIX_DEPTH)
#define DISPLAY_PIXELS_YUV     (DISPLAY_PIXELS*2)
#define DISPLAY_PIXELSE_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_PIXELSO_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_STRIDE8_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_STRIDE16_YUV   (DISPLAY_PIXELS*4)
#define DISPLAY_PIXELS_Y       (DISPLAY_PIXELS)
#define DISPLAY_PIXELSE_Y      (DISPLAY_PIXELS)
#define DISPLAY_PIXELSO_Y      (DISPLAY_PIXELS)
#define DISPLAY_STRIDE8_Y      (DISPLAY_PIXELS)
#define DISPLAY_STRIDE16_Y     (DISPLAY_PIXELS*2)
#define DISPLAY_LINES          (OUT_LINES)

//#define IN_PIXELS1         (1288)    // for MIPI
#define CAM_PIXELS         (CAM_PIXELS1*CAMS)
#define CAM_PIXELS_STRIDE         (CAM_PIXELS1*CAMS*2)
#define IN_PIXELS1         (CAM_PIXELS1)
#define IN_STRIDE1_16      (IN_PIXELS1*2)

#define IN_PIXELS          (IN_PIXELS1*4)
#define IN_STRIDE16        (IN_PIXELS*2)

#define CAM_SAVE_PIXELS    (CAM_PIXELS)
#define CAM_SAVE_STRIDE16  (CAM_SAVE_PIXELS*2+16)

//#define UYVY_INPIX         (OUT_PIXELS)
//#define UYVY_OUTPIX        (OUT_PIXELS*2)
//#define UYVY_STRIDE8       (OUT_PIXELS*2)
//#define UYVY_STRIDE16      (OUT_PIXELS*4)

#define DENOISE_OUTPIX     (OUT_PIXELS)
#define DENOISE_LINEGAP    (2)
#define DENOISE_INPIX      (OUT_PIXELS+DENOISE_LINEGAP*4)
#define DENOISE_STRIDE8    (DENOISE_OUTPIX)
#define DENOISE_STRIDE16   (DENOISE_OUTPIX*2)
// for multi paths
#define DENOISE_LINES      (OUT_LINES)    // + 1 ??
#define DENOISE_LINES2_0   ((DENOISE_LINES+1)>>1)
#define DENOISE_LINES2_1   ((DENOISE_LINES+0)>>1)
#define DENOISE_LINES3_0   ((DENOISE_LINES+2)/3)
#define DENOISE_LINES3_1   ((DENOISE_LINES+1)/3)
#define DENOISE_LINES3_2   ((DENOISE_LINES+0)/3)


#define COMPRESS_PIX        (DENOISE_INPIX)
// to reduce COMPRESS_INPIX is to extend 4 pixels at the output per camera
//#define COMPRESS_INPIX      (COMPRESS_PIX-4*4)
#define COMPRESS_STRIDE8    (COMPRESS_PIX)
#define COMPRESS_STRIDE16   (COMPRESS_PIX*2)
// for multi paths
#define COMPRESS_LINES      (DENOISE_LINES+2)
#define COMPRESS_LINES2_0   ((COMPRESS_LINES+1)>>1)
#define COMPRESS_LINES2_1   ((COMPRESS_LINES+0)>>1)
#define COMPRESS_LINES3_0   ((COMPRESS_LINES+2)/3)
#define COMPRESS_LINES3_1   ((COMPRESS_LINES+1)/3)
#define COMPRESS_LINES3_2   ((COMPRESS_LINES+0)/3)

// using COMPRESS_INPIX to allow 1280 camera pixels
#define RGB2YUV_PIX        (COMPRESS_PIX)
#define RGB2YUV_STRIDE8    (RGB2YUV_PIX)
#define RGB2YUV_STRIDE16   (RGB2YUV_PIX*2)
// for multi paths
#define RGB2YUV_LINES      (COMPRESS_LINES)
#define RGB2YUV_LINES2_0   ((RGB2YUV_LINES+1)>>1)
#define RGB2YUV_LINES2_1   ((RGB2YUV_LINES+0)>>1)
#define RGB2YUV_LINES3_0   ((RGB2YUV_LINES+2)/3)
#define RGB2YUV_LINES3_1   ((RGB2YUV_LINES+1)/3)
#define RGB2YUV_LINES3_2   ((RGB2YUV_LINES+0)/3)

#define DEBAYER_OUTPIX     (RGB2YUV_PIX)
//#define DEBAYER_OUTPIXE    (RGB2YUV_PIX)
//#define DEBAYER_OUTPIXO    (RGB2YUV_PIX)
#define DEBAYER_LINEGAP    (2)
#define DEBAYER_INPIX      (DEBAYER_OUTPIX+DEBAYER_LINEGAP*4)
#define DEBAYER_STRIDE16   (DEBAYER_INPIX*2)
// for merging output buffers
#define DEBAYER_PIXOFFS2_0 (0)
#define DEBAYER_PIXOFFS2_1 (DEBAYER_OUTPIX)
#define DEBAYER_PIXOFFS3_0 (0)
#define DEBAYER_PIXOFFS3_1 (DEBAYER_OUTPIX)
#define DEBAYER_PIXOFFS3_2 (DEBAYER_OUTPIX*2)
// for multi paths
#define DEBAYER_LINES      (RGB2YUV_LINES)
#define DEBAYER_LINES2_0   ((DEBAYER_LINES+1)>>1)
#define DEBAYER_LINES2_1   ((DEBAYER_LINES+0)>>1)
#define DEBAYER_LINES3_0   ((DEBAYER_LINES+2)/3)
#define DEBAYER_LINES3_1   ((DEBAYER_LINES+1)/3)
#define DEBAYER_LINES3_2   ((DEBAYER_LINES+0)/3)

//// for keeping sepated processing paths
//#define DEBAYER_LINES      (OUT_LINES)
//#define DEBAYER_LINES2_0   (OUT_LINES>>1)
//#define DEBAYER_LINES2_1   (OUT_LINES>>1)
//#define DEBAYER_LINES3_0   ((OUT_LINES+2)/3)
//#define DEBAYER_LINES3_1   ((OUT_LINES+1)/3)
//#define DEBAYER_LINES3_2   ((OUT_LINES+0)/3)

#define HDR_OUTPIX         (DEBAYER_INPIX)
// line gap in single value per pixels
#define HDR_LINEGAP        ((DEBAYER_INPIX>>2)-(IN_PIXELS1>>1))
//#define HDR_INITIALGAP    ((HDR_LINEGAP >>1) & 0xfffe) ... done in kernel!
#define HDR_PROC           (HDR_OUTPIX)
#define HDR_OUTSTRIDE16    (HDR_OUTPIX*2)
#define HDR_LINES          (DEBAYER_LINES+2)

#define HIST_INPIX         (IN_PIXELS1*4)
#define HIST_LINEGAP       ((IN_PIXELS1>>1)-OUT_PIXELS1)
//#define HIST_INITIALGAP    (HIST_LINEGAP & 0xfffc)
#define HIST_INITIALGAP    (0*2)
#define HIST_PROC          (OUT_PIXELS1*2)
#define HIST_OFFSET0       (RED_LSSTARTPIX+HIST_INITIALGAP)
#define HIST_OFFSET1       (2*IN_PIXELS1+RED_LSSTARTPIX+HIST_INITIALGAP)
#define HIST_OFFSETVS0     (RED_VSSTARTPIX+HIST_INITIALGAP)
#define HIST_OFFSETVS1     (2*IN_PIXELS1+RED_VSSTARTPIX+HIST_INITIALGAP)
#define HIST_LINES         (OUT_LINES>>2)


//#define OUT_SHIFT          (PIX_DEPTH & 0x3) // RGB
#define OUT_SHIFT          (0) // for Y: (0x4 & 0x3)

#endif // IOFILES_H_
