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
/**
* \file     cam_lldcmd.h
* \brief    lld command declarations for Generic camera driver
* \author   Tomas Babinec
* \version  0.1
* \date     16. June 2016
* \note
****************************************************************************/

#ifndef CAMLLDCMD_H
#define CAMLLDCMD_H

///////////////////////////////////////////
//
// LLDCMD command codes numbers allocation
//

// LLDCMD: reserve CAM interface
#define LLDCMD_CAM_CLI_ADD         0U
#define LLDCMD_CAM_CLI_REM         1U
#define LLDCMD_CAM_REG_RDB         2U
#define LLDCMD_CAM_REG_WRB         3U
#define LLDCMD_CAM_REG_RDS         4U
#define LLDCMD_CAM_REG_WRS         5U
#define LLDCMD_SONY_CONNECT        6U
#define LLDCMD_SONY_DISCONNECT     7U
#define LLDCMD_SONY_EXP_STATUS_GET 8U
#define LLDCMD_SONY_EXP_SENT       9U
#define LLDCMD_OV10640_CONNECT     10U
#define LLDCMD_OV10640_DISCONNECT  11U
#define LLDCMD_CAM_I2C_REPEAT_SET  12U
#define LLDCMD_CAM_I2C_REPEAT_GET  13U
#define LLDCMD_CAM_GPIO_CONTROL    14U
#define LLDCMD_CAM_RESET           15U
#define LLDCMD_CAM_POWER_CONTROL   16U

#endif /* CAMLLDCMD_H */

/* EOF */
