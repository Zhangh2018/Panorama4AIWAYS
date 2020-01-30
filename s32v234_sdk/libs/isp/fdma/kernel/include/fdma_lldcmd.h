/*
 * Copyright 2017-2018 NXP
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

/**
* @file     fdma_lldcmd.h
* @brief    LLD CMD number declarations for fDMA driver
****************************************************************************/

/*==============================================================================
Revision History:
                        Modification      Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Tomas Babinec           27-May-2014                    Inital version.
Tomas Babinec           23-May-2017       vsdk-575     Tracebility.
Long Le Thanh           28-Nob-2017       vsdk-1329    Misra.
Tomas Babinec           03-Jan-2018       vsdk-1624    LLD CMD
==============================================================================*/

#ifndef FDMALLDCMD_H_
#define FDMALLDCMD_H_

/*=============================================================================
*                              INCLUDE FILES
==============================================================================*/


/*==============================================================================
*                            DEFINES AND MACROS
==============================================================================*/

/* LLDCMD command codes numbers allocation */

// LLDCMD: transaction descriptor reservation
#define LLDCMD_FDMA_TD_RESERVE 0U

// LLDCMD: transaction configuration
#define LLDCMD_FDMA_TD_CONFIG 1U

// LLDCMD: transaction scheduling
#define LLDCMD_FDMA_TD_SCHEDULE 2U

// LLDCMD: transaction configuration and scheduling
#define LLDCMD_FDMA_TD_CONFIG_SCHEDULE 3U

// LLDCMD: release transaction descriptor
#define LLDCMD_FDMA_TD_RELEASE 4U

// LLDCMD: get copy of current fDMA registers
#define LLDCMD_FDMA_SEQ_MODE_GET 5U

// LLDCMD: get copy of current fDMA registers
#define LLDCMD_FDMA_STATUS_GET 6U

// LLDCMD: update the user space copy of TD in TC
#define LLDCMD_FDMA_TC_GET 7U

// LLDCMD: get number of TD that are currently in use
#define LLDCMD_FDMA_TD_USED 8U

// LLDCMD: get the address of the TDT in SRAM
#define LLDCMD_FDMA_TDT_GET 9U

// LLDCMD: get last event for specified TD
#define LLDCMD_FDMA_EVENT_GET 10U

// LLDCMD: transaction configuration
#define LLDCMD_FDMA_TD_ISP_CONFIG 11U

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
*                        GLOBAL FUNCTION DECLARATIONS
==============================================================================*/


/*==============================================================================
*                                   CONSTANTS
==============================================================================*/


#endif /* FDMALLDCMD_H_ */
