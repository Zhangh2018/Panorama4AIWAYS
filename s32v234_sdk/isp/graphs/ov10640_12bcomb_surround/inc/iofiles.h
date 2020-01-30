/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
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
/**
 * \file    iofiles.h
 * \brief   ISP sony_y graph
 * \author  Armin Strobl
 * \author
 * \version 0.1
 * \date    12-05-2015
 * \note    inital version
 ****************************************************************************/

#ifndef	IOFILES_H
#define	IOFILES_H

#define	PIX_DEPTH          3

#define OUT_PIXELS         (1280*4)
#define OUT_PIXELS_RGB     (OUT_PIXELS*PIX_DEPTH)
#define OUT_STRIDE8_RGB    (OUT_PIXELS*PIX_DEPTH)
#define OUT_STRIDE16_RGB   (OUT_PIXELS*2*PIX_DEPTH)
#define OUT_PIXELS_YUV     (OUT_PIXELS*2)
#define OUT_STRIDE8_YUV    (OUT_PIXELS*2)
#define OUT_STRIDE16_YUV   (OUT_PIXELS*4)
#define	OUT_LINES          (720)
#define	OUT_LINES2_0       (OUT_LINES>>1)
#define	OUT_LINES2_1       (OUT_LINES>>1)
#define	OUT_LINES3_0       ((OUT_LINES+2)/3)
#define	OUT_LINES3_1       ((OUT_LINES+1)/3)
#define	OUT_LINES3_2       ((OUT_LINES+0)/3)
#define	OUT_LINES4         ((OUT_LINES)/4)

#define DISPLAY_PIXELS         (1920)
#define DISPLAY_PIXELS_RGB     (DISPLAY_PIXELS*PIX_DEPTH)
#define DISPLAY_PIXELSE_RGB    (DISPLAY_PIXELS*PIX_DEPTH)
#define DISPLAY_PIXELSO_RGB    (DISPLAY_PIXELS*PIX_DEPTH)
#define DISPLAY_STRIDE8_RGB    (DISPLAY_PIXELS*PIX_DEPTH)
#define DISPLAY_STRIDE16_RGB   (DISPLAY_PIXELS*PIX_DEPTH)
#define DISPLAY_PIXELS_YUV     (DISPLAY_PIXELS*2)
#define DISPLAY_PIXELSE_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_PIXELSO_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_STRIDE8_YUV    (DISPLAY_PIXELS*2)
#define DISPLAY_STRIDE16_YUV   (DISPLAY_PIXELS*4)
#define	DISPLAY_LINES          (OUT_LINES)

#define IN_PIXELS1         (1288)
#define IN_PIXELS          (1288*4)
#define IN_STRIDE16        (IN_PIXELS*2)
#define CAM_LINES          (OUT_LINES+8)

#define HIST_PIX           (1280)

#define DEBAYER_OUTPIX     (1280*4)
#define DEBAYER_OUTPIXE    (1280*4)
#define DEBAYER_OUTPIXO    (1280*4)
#define DEBAYER_INPIX      (1288*4)
#define DEBAYER_STRIDE16   ((1280*4)*2)
// for merging output buffers
#define DEBAYER_PIXOFFS2_0 (0)
#define DEBAYER_PIXOFFS2_1 (DEBAYER_OUTPIX)
#define DEBAYER_PIXOFFS3_0 (0)
#define DEBAYER_PIXOFFS3_1 (DEBAYER_OUTPIX)
#define DEBAYER_PIXOFFS3_2 (DEBAYER_OUTPIX*2)
#define DEBAYER_LINES      ((OUT_LINES+3))
#define DEBAYER_LINES2_0   ((OUT_LINES+0)>>1)
#define DEBAYER_LINES2_1   ((OUT_LINES+0)>>1)
#define	DEBAYER_LINES3_0   ((OUT_LINES+5)/3)
#define	DEBAYER_LINES3_1   ((OUT_LINES+4)/3)
#define	DEBAYER_LINES3_2   ((OUT_LINES+3)/3)

//// for keeping sepated processing paths
//#define DEBAYER_LINES      (OUT_LINES)
//#define DEBAYER_LINES2_0   (OUT_LINES>>1)
//#define DEBAYER_LINES2_1   (OUT_LINES>>1)
//#define	DEBAYER_LINES3_0   ((OUT_LINES+2)/3)
//#define	DEBAYER_LINES3_1   ((OUT_LINES+1)/3)
//#define	DEBAYER_LINES3_2   ((OUT_LINES+0)/3)


#define RGB2YUV_PIX        (1280*4)
#define RGB2YUV_PIX2       (1280*8)
#define RGB2YUV_STRIDE8    (1280*4)
#define RGB2YUV_STRIDE16   (1280*4*2)

#define UYVY_INPIX         (1280*4)
#define UYVY_OUTPIX        (1280*4*2)
#define UYVY_STRIDE8       (1280*4*2)
#define UYVY_STRIDE16      (1280*4*4)

#endif /* IOFILES_H */
