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
/**
* \file     cam.h
* \brief    declarations for Generic camera driver
* \author   Tomas Babinec
* \version  0.1
* \date     16-June-2016
* \note
****************************************************************************/

#ifndef CAM_H
#define CAM_H

#ifdef __cplusplus 
extern "C" { 
#endif

#include "isp_hw.h"
#include "cam_types.h"

/*****************************************************************************
* MACRO definitions
*****************************************************************************/

#define CAM_DRV_SUCCESS  0
#define CAM_DRV_FAILURE -1


#define I2C_CLI_CNT_MAX 50        ///< number of supported I2C clients
#define MAX_COMPATIBLE_BYTES 16

// name of the device file
#define CAM_DEVICE_NAME "cam_generic"

// magic number for fDMA driver
#define CAM_IOC_MAGIC 'c'

// gpio to control camera
#define CAM_GPIO_INVALID_PIN            (uint32_t)(-1)

#define CAM_GPIO_EVB_CSI0_RESET         38
#define CAM_GPIO_EVB_CSI1_RESET         31
#define CAM_GPIO_EVB_CSI0_POWER         37
#define CAM_GPIO_EVB_CSI1_POWER         30
#define CAM_GPIO_EVB_VIU0_RESET         38
#define CAM_GPIO_EVB_VIU1_RESET         31
#define CAM_GPIO_EVB_VIU0_POWER         37
#define CAM_GPIO_EVB_VIU1_POWER         30

#define CAM_GPIO_SBC_CSI0_RESET         38
#define CAM_GPIO_SBC_CSI1_RESET         31
#define CAM_GPIO_SBC_CSI0_POWER         CAM_GPIO_INVALID_PIN
#define CAM_GPIO_SBC_CSI1_POWER         CAM_GPIO_INVALID_PIN
#define CAM_GPIO_SBC_VIU0_RESET         CAM_GPIO_INVALID_PIN
#define CAM_GPIO_SBC_VIU1_RESET         CAM_GPIO_INVALID_PIN
#define CAM_GPIO_SBC_VIU0_POWER         CAM_GPIO_INVALID_PIN
#define CAM_GPIO_SBC_VIU1_POWER         CAM_GPIO_INVALID_PIN

// camera count
#define CAM_CSI_DEV_CNT                 2
#define CAM_VIU_DEV_CNT                 2

// compatible string
#define CAM_EVB_29288_COMPATIBLE_STRING       "fsl,s32v234-evb2016q4"
#define CAM_EVB_28899_COMPATIBLE_STRING       "fsl,s32v234-evb28899"
#define CAM_SBC_COMPATIBLE_STRING             "fsl,s32v234-sbc"

/*****************************************************************************
* Global variables
*****************************************************************************/
#ifdef __KERNEL__
extern uint32_t* gCsiResetPin;
extern uint32_t* gCsiPowerDownPin;
extern uint32_t* gViuResetPin;
extern uint32_t* gViuPowerDownPin;

extern uint32_t gEvbCsiResetPin[];
extern uint32_t gEvbCsiPowerDownPin[];
extern uint32_t gEvbViuResetPin[];
extern uint32_t gEvbViuPowerDownPin[];
extern uint32_t gSbcCsiResetPin[];
extern uint32_t gSbcCsiPowerDownPin[];
extern uint32_t gSbcViuResetPin[];
extern uint32_t gSbcViuPowerDownPin[];
#else // else from ifdef __KERNEL__
extern uint32_t gCsiResetPin[];
extern uint32_t gCsiPowerDownPin[];
extern uint32_t gViuResetPin[];
extern uint32_t gViuPowerDownPin[];
#endif // #ifdef __KERNEL__

#ifdef __cplusplus 
} //extern "C" 
#endif

#endif /* CAM_H */