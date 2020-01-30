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
/**
* \file     seq_lldcmd.h
* \brief    lld cmd numbers declarations for Sequencer driver
* \author   Tomas Babinec
* \version  0.1
* \date     30.5.2014
* \note
****************************************************************************/

#ifndef SEQLLDCMD_H
#define SEQLLDCMD_H

#ifdef __cplusplus 
extern "C" { 
#endif
  
///////////////////////////////////////////
//
// LLDCMD command codes numbers declaration
//
  
// LLDCMD: graph download
#define LLDCMD_SEQ_GRAPH_DOWNLOAD 6U
  
// LLDCMD: DDR buffer lists setup
#define LLDCMD_SEQ_DDRBUF_SET 7U

// LLDCMD: graph start
#define LLDCMD_SEQ_START 8U
  
// LLDCMD: graph start
#define LLDCMD_SEQ_AUX_DATA_GET 9U
  
// LLDCMD: graph stop
#define LLDCMD_SEQ_STOP 10U
  
// LLDCMD: graph stop wait
#define LLDCMD_SEQ_STOP_WAIT 11U
  
// LLDCMD: SEQ HW & driver reset
#define LLDCMD_SEQ_RESET 12U
  
// LLDCMD: SEQ boot
#define LLDCMD_SEQ_BOOT 13U

// LLDCMD: Irq serve
#define LLDCMD_SEQ_IRQ_SRV 14U
  
// LLDCMD: send message
#define LLDCMD_SEQ_MSG_SEND 15U

// LLDCMD: send message wait
#define LLDCMD_SEQ_MSG_SEND_WAIT 16U
  
// LLDCMD: request buffers
#define LLDCMD_SEQ_BUF_REQ 17U
  
// LLDCMD: queries buffer
#define LLDCMD_SEQ_BUF_QRY 18U

// LLDCMD: queries buffer
#define LLDCMD_SEQ_TIME_STAT 19U
  
// LLDCMD: removes buffers in all streams
#define LLDCMD_SEQ_BUF_PSH 20U
  
// LLDCMD: removes buffers in all streams
#define LLDCMD_SEQ_BUF_POP 21U
  
// LLDCMD: requests performance data for IPUs
#define LLDCMD_SEQ_IPU_PROFILE_REQ 22U
  
// LLDCMD: gets performance data for IPUs
#define LLDCMD_SEQ_IPU_PROFILE_GET 23U

// LLDCMD: gets Event log data
#define LLDCMD_SEQ_IPU_EVENT_LOG 24U  
  
// LLDCMD: enables Event logging
#define LLDCMD_SEQ_IPU_EVENT_LOG_ENABLE 25U

// LLDCMD: disables Event logging
#define LLDCMD_SEQ_IPU_EVENT_LOG_DISABLE 26U 
  
// LLDCMD: gets graph structure from PRAM
#define LLDCMD_SEQ_GRAPH_FETCH 27U
  
// LLDCMD: enables histogram readout for IPU engine
#define LLDCMD_SEQ_HIST_ENABLE 28U
  
// LLDCMD: disables histogram readout for IPU engine
#define LLDCMD_SEQ_HIST_DISABLE 29U
  
// LLDCMD: gets latest histogram data for particular IPU engine
#define LLDCMD_SEQ_HIST_GET 30U

// LLDCMD: enables statistics readout 
#define LLDCMD_SEQ_STAT_ENABLE 31U
  
// LLDCMD: disables statistics readout 
#define LLDCMD_SEQ_STAT_DISABLE 32U
  
// LLDCMD: gets latest statistic data for particular IPU engine
#define LLDCMD_SEQ_STAT_GET 33U

// LLDCMD: configures frame based graph patching
#define LLDCMD_SEQ_PATCH 34U  

// LLDCMD: sets up register list in PRAM
#define LLDCMD_SEQ_REGLIST_SET 35U

// LLDCMD: reads back register list from PRAM
#define LLDCMD_SEQ_REGLIST_GET 36U

#ifdef __cplusplus 
} 
#endif 

#endif /* SEQLLDCMD_H */