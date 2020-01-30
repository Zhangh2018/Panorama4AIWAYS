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

#ifndef H264DCDLLDCMD_H
#define H264DCDLLDCMD_H

/**
* @file           h264dcd_linux.h
* brief           Linux declarations for H264 Decoder driver
*/

#define LLDCMD_APP_TYPE  'c'

/************************************************
    H264DCD LLDCMD function codes
*************************************************/

/************    Set Input Data   ***************/
#define H264DCD_LLDCMD_INPUTSTREAM_CFG           1U

/*************   Get FIFO Status   **************/
#define H264DCD_LLDCMD_FIFO_STATUS_GET           2U

/***********    Get Data From FIFO   ************/
#define H264DCD_LLDCMD_PCK_STATUS_GET            3U

/********   Set FIFO Watermark Level  ***********/
#define H264DCD_LLDCMD_FIFO_WATERMARK_SET        4U

/**************      Clear FIFO     *************/
#define H264DCD_LLDCMD_FIFO_CLEAR                5U

/**********      Set H264 Timeout      **********/
#define H264DCD_LLDCMD_TIMEOUT_SET               6U

/**********  Enable The TimeOut Check ***********/
#define H264DCD_LLDCMD_TIMEOUT_ENABLE            7U

/****************    Sw Reset    ****************/
#define H264DCD_LLDCMD_SW_RESET                  8U

/****** Set The Ouput Stream Configuration ******/
#define H264DCD_LLDCMD_OUTSTREAM_CFG             9U

/********       Set Sampling Mode      **********/
#define H264DCD_LLDCMD_OUTSAMPLEMODE_SET         10U

/*******   Set Output Image Colour Params  ******/
#define H264DCD_LLDCMD_OUTCOLOURDATA_SET         11U

/*********   Set The Data Flow Mode   ***********/
#define H264DCD_LLDCMD_DATAFLOWMODE_SET          12U

/*********   Get The Data Flow Mode   ***********/
#define H264DCD_LLDCMD_DATAFLOWMODE_GET          13U

/*******    Set The Bankstride Length    ********/
#define H264DCD_LLDCMD_BANKSTRIDE_SET            14U

/********   Get The Bankstride Length   *********/
#define H264DCD_LLDCMD_BANKSTRIDE_GET            15U

/***********      Channel Stop      *************/
#define H264DCD_LLDCMD_DCD_CH_STOP               16U

/*********  Set The Codded Image Size  **********/
#define H264DCD_LLDCMD_DCD_INFRAMESIZE_SET       17U

/*********   Set Memory Type & Filter    ********/
#define H264DCD_LLDCMD_DCD_CFG_SET               18U

/*********   Get Memory Type & Filter   *********/
#define H264DCD_LLDCMD_DCD_CFG_GET               19U

/*********   Set Threshold Levels    ************/
#define H264DCD_LLDCMD_DCD_THRLEVELS_SET         20U

/*********   Get Threshold Levels   *************/
#define H264DCD_LLDCMD_DCD_THRLEVELS_GET         21U

/********   Set Reference Memory Address ********/
#define H264DCD_LLDCMD_DCD_REFMEMORY_SET         22U

/********   Get Reference Memory Address ********/
#define H264DCD_LLDCMD_DCD_REFMEMORY_GET         23U

/**********   Get The Channel Status    *********/
#define H264DCD_LLDCMD_CH_STATUS_GET             24U

/********* Get Decoded Image Paramters  *********/
#define H264DCD_LLDCMD_PICDCD_PARAM_GET          25U

/********    Get Decoded Image Status    ********/
#define H264DCD_LLDCMD_PICDCD_STATUS_GET         26U

/******** Get Decoded Image Gen Params  *********/
#define H264DCD_LLDCMD_PICDCD_GENPARAMS_GET      27U

/*******    Enable/Disable Interrupts    ********/
#define H264DCD_LLDCMD_IRQ_CONTROL               28U

/*********  Get The Enabled Interrupts  *********/
#define H264DCD_LLDCMD_IRQ_GET                   29U

/*********  Get The TimeOut Intr Status  ********/
#define H264DCD_LLDCMD_IRQ_TIMEOUT_GET           30U

/*********   Get The Error Intr Status   ********/
#define H264DCD_LLDCMD_IRQ_ERROR_GET             31U

/*********  Get The Interrupts Flags   **********/
#define H264DCD_LLDCMD_IRQ_STREAMSTATUS_GET      32U

#endif /* H264DCDLLDCMD_H */
