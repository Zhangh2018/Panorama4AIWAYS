/*
 * Copyright (c) 2016 Freescale Semiconductor
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
/****************************************************************************
  * \file     isp_cam_ub964_api.h
  * \brief    public declarations for TI ser/des user space library.
  * \author   Thomas Brown
  * \version  0.1
  * \date     06.03.2018
  * \note
 ****************************************************************************/

//*****************************************************************************
// include
//*****************************************************************************
#include "../libs/isp/csi/kernel/include/csi.h"
#include "isp_cam_generic.h"
#include "../libs/isp/cam_generic/kernel/include/tiub964_types.h"

//*****************************************************************************
// constants
//*****************************************************************************

//*****************************************************************************
// types
//*****************************************************************************

//*****************************************************************************
// public functionality declarations
//*****************************************************************************

/****************************************************************************/
/** Toggle UB964 Rx channel Forwarding.
 *
 * \param aCsiIdx[in]     CSI port id
 * \param aCsiFwd[in] forwards the data to one of the CSI-2 output interfaces
 *
 * \return: no return value
*
 ****************************************************************************/
void TIUB964_ChanSelect ( const enum CSI_IDX aCsiIdx, uint8_t aCsiFwd );

/****************************************************************************/
/** Enables CSI transmission from TIUB964 camera.
 *
 * \param acUB964_Cfg reference to structure with tiub964 configuration
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT TIUB964_CsiEnable( const TIUB964_Cfg_t &acUB964_Cfg );

/****************************************************************************/
/** Disables CSI transmission from TIUB964 camera.
 *
 * \param acUB964_Cfg reference to structure with tiub964 configuration
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT TIUB964_CsiDisable( const TIUB964_Cfg_t &acUB964_Cfg );

/****************************************************************************/
/** Enables data transmission from TIUB964 OV10640 camera.
 *
 * \param acUB964_Cfg reference to structure with tiub964 configuration
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT TIUB964_OV10640_Start( const TIUB964_Cfg &acUB964_Cfg );

/****************************************************************************/
/** Disables data transmission from TIUB964 OV10640 camera.
 *
 * \param acUB964_Cfg reference to structure with tiub964 configuration
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT TIUB964_OV10640_Stop( const TIUB964_Cfg &acUB964_Cfg );

/****************************************************************************/
/** Configures AR0140 camera registers.
*
* \param aNumChan[in] number channel for UB964 Csi2 Tx port
*
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT TIUB964_AR0140_Init ( const enum CSI_IDX aCsiIdx, uint8_t aNumChan);

/****************************************************************************/
/** UB964 Configuration API
*
* \param acUB964_Cfg[in] reference to structure with tiub964 configuration
*
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT TIUB964_Init( const TIUB964_Cfg &acUB964_Cfg );

/****************************************************************************/
/** Close TIUB964 maxim.
*
* \param aCsiIdx[in]     CSI port id
*
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT TIUB964_Close(const enum CSI_IDX aCsiIdx);

/*********************************
** UB964 CSI2 TEST PATTERN      **
*********************************/
#ifdef __STANDALONE__
CAM_LIB_RESULT TIUB964_TestPattern_Setup( void );
CAM_LIB_RESULT TIUB964_TestPattern_Start( void );
CAM_LIB_RESULT TIUB964_TestPattern_Stop( void );
#endif
