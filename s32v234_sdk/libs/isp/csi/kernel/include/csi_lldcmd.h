/*
 * Copyright (c) 2015-2016 Freescale Semiconductor
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

/*==============================================================================
Revision History:
                        Modification       Tracking
Author (core ID)        Date D/M/Y         Number     Description of Changes
Tomas Babinec           28-Apr-2016                   Init version
Nguyen Tri Hai(B51945)  1-Dec-2017        VSDK-1225   Resolve Misra violations
Tomas Babinec           03-Jan-2018       VSDK-1624   LLD CMD
==============================================================================*/
#ifndef CSILLDCMD_H
#define CSILLDCMD_H

/**
* @file           csi_lldcmd.h
* @brief          Declaration of MipiCsi driver internal functionality
*/


/*=============================================================================
*                              INCLUDE FILES
==============================================================================*/


/*==============================================================================
*                            DEFINES AND MACROS
==============================================================================*/
#define LLDCMD_CSI_CONFIG          0U
#define LLDCMD_CSI_EMBD_CONFIG     1U
#define LLDCMD_CSI_START           4U
#define LLDCMD_CSI_STOP            5U
#define LLDCMD_CSI_SWRESET         6U

#define LLDCMD_CSI_EMBD_RECAP      34U
#define LLDCMD_CSI_EMBD_STATUS_GET 35U

#define LLDCMD_CSI_ERR_STATUS_GET  36U
#define LLDCMD_CSI_ERR_CLEAR       37U

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

#endif /* CSILLDCMD_H */

/* EOF */
