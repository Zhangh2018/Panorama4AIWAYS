/*
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright (c) 2017-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JPEGDCDLLDCMD_H
#define JPEGDCDLLDCMD_H

/**
* @file           jpegdcd_linux.h
* brief           Linux declarations for JPEG Decoder driver
*/

#define LLDCMD_APP_TYPE  'c'

/************************************************
    JPEGDCD LLDCMD function codes
*************************************************/

/************    Set Input Data   ***************/
#define JPEGDCD_LLDCMD_INPUTSTREAM_CFG           1U

/*************   Get FIFO Status   **************/
#define JPEGDCD_LLDCMD_FIFO_STATUS_GET           2U

/***********    Get Data From FIFO   ************/
#define JPEGDCD_LLDCMD_FIFO_DATA_GET             3U

/************   Set FIFO Watermark   ************/
#define JPEGDCD_LLDCMD_FIFO_WATERMARK_SET        4U

/**********   Set Output Buffers    *************/
#define JPEGDCD_LLDCMD_OUTSTREAM_CFG             5U

/***********   Enable Resample mode   ***********/
#define JPEGDCD_LLDCMD_OUTSAMPLEMODE_SET         6U

/**************   Set Bankstride    *************/
#define JPEGDCD_LLDCMD_BANKSTRIDE_SET            7U

/**************   Get Bankstride    *************/
#define JPEGDCD_LLDCMD_BANKSTRIDE_GET            8U

/******   Configure JPEG Decoder Start   ********/
#define JPEGDCD_LLDCMD_JPEG_CFGSTART             9U

/**********   Enable JPEG Decoder   *************/
#define JPEGDCD_LLDCMD_JPEG_DCDENABLE            10U

/*********   Configure input stream   ***********/
#define JPEGDCD_LLDCMD_JPEG_STREAMCFG_SET        11U

/*********   Get Enabled Interrupts    **********/
#define JPEGDCD_LLDCMD_IRQ_GET                   12U

/********* Set JPEG Number Of Streams  **********/
#define JPEGDCD_LLDCMD_JPEG_STREAMNUM_SET        13U

/********* Get JPEG Number Of Streams  **********/
#define JPEGDCD_LLDCMD_JPEG_STREAMNUM_GET        14U

/********* Set JPEG Compatibility Mode  *********/
#define JPEGDCD_LLDCMD_JPEG_MODE_SET             15U

/********* Get JPEG Compatibility Mode  *********/
#define JPEGDCD_LLDCMD_JPEG_MODE_GET             16U

/****************    SW Reset    ****************/
#define JPEGDCD_LLDCMD_JPEG_SW_RESET             17U

/************   Get Active Stream   *************/
#define JPEGDCD_LLDCMD_JPEG_STREAM_GET           18U

/**********   Set Reset Interval    *************/
#define JPEGDCD_LLDCMD_JPEG_RSTINTERVAL_SET      19U

/**********   Get Reset Interval    *************/
#define JPEGDCD_LLDCMD_JPEG_RSTINTERVAL_GET      20U

/***********   Get Frame Status    **************/
#define JPEGDCD_LLDCMD_JPEG_FRAMESTATUS_GET      21U

/**********   Get Error Status    ***************/
#define JPEGDCD_LLDCMD_JPEG_DCDCFGERR_GET        22U

/***********   Get MCU Row Status    ************/
#define JPEGDCD_LLDCMD_JPEG_MCUROWEND_GET        23U

/***********   Get Decoded Image    *************/
#define JPEGDCD_LLDCMD_JPEG_DCDIMAGE_GET         24U

/***********    Set JPEG Timeout    *************/
#define JPEGDCD_LLDCMD_JPEG_TIMEOUT_SET          25U

/***********   Enable Interrupts    *************/
#define JPEGDCD_LLDCMD_IRQ_CONTROL               26U

#endif /* JPEGDCDLLDCMD_H */
