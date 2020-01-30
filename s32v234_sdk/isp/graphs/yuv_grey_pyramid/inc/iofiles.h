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
 * \brief   ISP RGB, YUV and GreyScale8 graph
 * \author  Armin Strobl
 * \author
 * \version 0.1
 * \date    15-01-2016
 * \note    inital version
 ****************************************************************************/

#ifndef	IOFILES_H
#define	IOFILES_H
/*
IPUS0:	Y4_Down
IPUS1:	ChanGain
IPUS2:	Debayer_G
IPUS3:	ConverOut
IPUS4:	Y8_Down
IPUS5:	Debayer_BR
IPUS6:	Y2_Down
IPUS7:	Y16_Down

IPUV0:	Y32_Down
IPUV1:	RGB_2_YUV
IPUV2:	;
IPUV3:	;
*/

#define	DDR_CEIL(_v,_g)			(((_v)+((_g)-1)) & ~((_g)-1))

#define	NUM_CAM_LINES			729
#define	NUM_IN_LINES			728
#define	NUM_OUT_PIXELS			1280
#define	NUM_OUT_LINES			720
#define	NUM_IN_M1			(NUM_IN_LINES-1)
#define	NUM_IN_M2			(NUM_IN_LINES-2)
#define	NUM_IN_M4			(NUM_IN_LINES-4)
#define	NUM_IN_M6			(NUM_IN_LINES-6)
#define	NUM_IN_M8			(NUM_IN_LINES-8)
#define	NUM_OUT_Y_BYTES			(1*NUM_OUT_PIXELS)
#define	NUM_OUT_YUV_BYTES		(2*NUM_OUT_PIXELS)
#define	NUM_OUT_RGB_BYTES		(3*NUM_OUT_PIXELS)
#define	NUM_OUT_Y2_SCALED_BYTES		(NUM_OUT_Y_BYTES/2)
#define	NUM_OUT_Y2_SCALED_LINES		(NUM_OUT_LINES/2)
#define	NUM_OUT_Y4_SCALED_BYTES		(NUM_OUT_Y_BYTES/4)
#define	NUM_OUT_Y4_SCALED_LINES		(NUM_OUT_LINES/4)
#define	NUM_OUT_Y8_SCALED_BYTES		(NUM_OUT_Y_BYTES/8)
#define	NUM_OUT_Y8_SCALED_LINES		(NUM_OUT_LINES/8)
#define	NUM_OUT_Y16_SCALED_BYTES	(NUM_OUT_Y_BYTES/16)
#define	NUM_OUT_Y16_SCALED_LINES	(NUM_OUT_LINES/16)
#define	NUM_OUT_Y32_SCALED_BYTES	48    // Align 16 for (NUM_OUT_Y_BYTES/32)
#define	NUM_OUT_Y32_SCALED_LINES	22   //(NUM_OUT_LINES/32)

#endif /* IOFILES_H */
