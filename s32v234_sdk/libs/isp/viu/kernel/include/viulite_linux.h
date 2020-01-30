/*
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright 2017 NXP
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

/*==============================================================================
Revision History:
                        Modification       Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Cristian Tomescu        03-Jan-2016                   Init version
Xuyen Dao               05-Aug-2016        VSDK-251   Update template
Tomas Babinec           15-Sep-2016        VSDK-325   Added register defines
Cristian Tomescu        04-Nov-2016        VSDK-488   Splitting the driver
Cristian Tomescu        08-May-2017        VSDK-560   Fix MISRA violations
==============================================================================*/

#ifndef VIULITELINUX_H
#define VIULITELINUX_H

#ifdef __cplusplus
extern "C"{
#endif

/**
* @file           viulite_linux.h
* brief           linux declarations for viu driver and user applications
*/

/*==============================================================================
*                               INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==============================================================================*/
#include "viulite_types.h"

/*==============================================================================
*                                   CONSTANTS
==============================================================================*/
#define LLDCMD_APP_TYPE  'c'

/*==============================================================================
*                          DEFINES AND MACROS
==============================================================================*/
/* DMA Configuring */
#define VIULITE_LLDCMD_DMA_CONFIG              1U

/* DMA Start */
#define VIULITE_LLDCMD_DMA_START               3U

/* DMA Stop */
#define VIULITE_LLDCMD_DMA_STOP                4U

/* DMA Get Status */
#define VIULITE_LLDCMD_DMA_GET_STATUS          5U

/* Set Data Input Format */
#define VIULITE_LLDCMD_SET_VIDEOIN_FORMAT      6U

/* Get Data Input Format */
#define VIULITE_LLDCMD_GET_VIDEOIN_FORMAT      7U

/* Set Data Interface */
#define VIULITE_LLDCMD_SET_DATA_INTERFACE      8U

/* Get Data Interface */
#define VIULITE_LLDCMD_GET_DATA_INTERFACE      9U

/* Config Interrupt Status Flags */
#define VIULITE_LLDCMD_CONFIG_IRQS             10U

/* Reset Interrupt Status Flags */
#define VIULITE_LLDCMD_RESET_IRQSTATUS         11U

/* Get Interrupt Status Flags */
#define VIULITE_LLDCMD_GET_IRQSTATUS           12U

/* Sw Reset */
#define VIULITE_LLDCMD_SW_RESET                13U

/* Set ITU Error Code */
#define VIULITE_LLDCMD_EN_ITU_ERRCODE          14U

/* Get ITU Error Code */
#define VIULITE_LLDCMD_GET_ITU_ERRCODE         15U

/* Get Field Number */
#define VIULITE_LLDCMD_GET_FIELDNUM            16U

/* Get Sync Signals Status */
#define VIULITE_LLDCMD_GET_SYNC                17U

/*Get Frame Size: lines and pixels */
#define VIULITE_LLDCMD_GET_FRAME_SIZE          18U

/* Set Clipping Data */
#define VIULITE_LLDCMD_SET_CLIPPING            19U

/* Get Clipping Data */
#define VIULITE_LLDCMD_GET_CLIPPING            20U


/*==============================================================================
*                                     ENUMS
==============================================================================*/


/*==============================================================================
*                     STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/


/*==============================================================================
*                        GLOBAL VARIABLE DECLARATIONS
==============================================================================*/


/*==============================================================================
*                           FUNCTION PROTOTYPES
==============================================================================*/


#ifdef __cplusplus
}
#endif

#endif /* VIULITELINUX_H */

/** @} */
