/*
 * Copyright (c) 2016 Freescale Semiconductor
 * Copyright (c) 2017-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 *
 * o Neither the name of NXP nor the names of its
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

/*==============================================================================
Revision History:
                              Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ----------------------
Roman Kubica                  14/10/2016                   Initial Version
Tomas Babinec                 04/01/2018     VSDK-1624     LLD CMD
==============================================================================*/

#ifndef H264ENCLLDCMD_H
#define H264ENCLLDCMD_H

#ifdef __cplusplus
extern "C"{
#endif

/**
* @file           h264enc_lldcmd.h 
*/



/*=============================================================================
*                                        INCLUDE FILES
==============================================================================*/


/*==============================================================================
*                            DEFINES AND MACROS
==============================================================================*/

/* LLDCMD: encodig configuration setup */
#define LLDCMD_H264_ENC_CONFIG_SET 0U

/* LLDCMD: set memory buffers */
#define LLDCMD_H264_ENC_BUFFERS_SET 1U

/* LLDCMD: how many more lines are available in input buffer */
#define LLDCMD_H264_ENC_FETCH 2U

/* LLDCMD: invokes bistream alarm address shift */
#define LLDCMD_H264_ENC_BS_ALARM_SHIFT 3U

/* LLDCMD: get current HW status */
#define LLDCMD_H264_ENC_STATUS_GET 4U

/* LLDCMD: stops the encoding sequence */
#define LLDCMD_H264_ENC_STOP 5U

/* LLDCMD: provides macroblock encoding status */
#define LLDCMD_H264_ENC_BS_ROW_DONE_GET 6U

/* LLDCMD */
#define LLDCMD_H264_ENC_RESET_VARS 7U

/*==============================================================================
*                                  ENUMS
==============================================================================*/

/*=============================================================================
*                        STRUCTURES AND OTHER TYPEDEFS
* ============================================================================*/

/*==============================================================================
*                        GLOBAL VARIABLE DECLARATIONS
==============================================================================*/

/*==============================================================================
*                                   CONSTANTS
==============================================================================*/


#ifdef __cplusplus
}
#endif

#endif /* H264ENCLLDCMD_H */
