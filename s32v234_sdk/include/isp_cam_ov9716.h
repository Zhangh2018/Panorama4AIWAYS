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
/**
* \file     ov9716_public.h
* \brief    public declarations for Omnivion9716 user space library
* \author   Tomas Babinec
* \version  0.1
* \date     14-October-2016
* \note
****************************************************************************/

#ifndef __OV9716_PUBLIC_H__
#define __OV9716_PUBLIC_H__

#include "isp_cam_generic.h"
#include "../libs/isp/cam_generic/kernel/include/ov9716_types.h"

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
/** Connect to the camera HW.
* 
* \param aCsiIdx[in] Csi interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_Open(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Disconnect from the camera HW.
* 
* \param aCsiIdx[in] Csi interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_Close(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Configures Ov9716 camera registers.
* 
* \param aCsiIdx[in] Csi interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_RegConfig(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Enables CSI transmission from OV9716 camera.
 * 
 * \param aCsiIdx which CSI interface the camera is connected to
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_CsiEnable(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Disables CSI transmission from OV9716 camera.
*
* \param aCsiIdx which CSI interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_CsiDisable(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Connect to the camera HW.
* 
* \param aViuIdx[in] Viu interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_Open(VIU_IDX aViuIdx);

/****************************************************************************/
/** Disconnect from the camera HW.
* 
* \param aViuIdx[in] Viu interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_Close(VIU_IDX aViuIdx);

/****************************************************************************/
/** Configures Ov9716 camera registers.
* 
* \param aViuIdx[in] Viu interface the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_RegConfig(VIU_IDX aViuIdx);

/****************************************************************************/
/** Enables data transmission from OV9716 camera.
 * 
 * \param aViuIdx which VIU index the camera is connected to
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_DataEnable(VIU_IDX aViuIdx);

/****************************************************************************/
/** Disables data transmission from OV9716 camera.
* 
* \param aViuIdx which VIU index the camera is connected to
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_DataDisable(VIU_IDX aViuIdx);

/****************************************************************************/
/** Configures Ov9716 camera for 3x12bit output mode (raw12)
 * 
 * \param aCsiIdx CSI interface index the camera is connected to.
 ****************************************************************************/
//void OV9716_Mode3x12(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Configures Ov9716 to do AEC and apply AWB.
 * 
 * \param aCsiIdx CSI interface index the camera is connected to.
 ****************************************************************************/
//void OV9716_AEC_WB_On(CSI_IDX aCsiIdx);

/****************************************************************************/
/** Configures Ov9716 to flip the image
 * 
 * \param aCsiIdx CSI interface index the camera is connected to.
 * \param on_off  1=on flip and mirror; 0=off no flip no mirror
 ****************************************************************************/
//void OV9716_Flip(CSI_IDX aCsiIdx,uint8_t on_off);

/****************************************************************************/
/** Configures Ov9716 for a defined fps
 * 
 * \param aCsiIdx CSI interface index the camera is connected to.
 * \param aFps frame per seconds value
 ****************************************************************************/
//void OV9716_Fps(CSI_IDX aCsiIdx, uint32_t aFps);

/****************************************************************************/
/** Configures Ov9716 channel gain related registers.
 * 
 * \param arExp reference to exposure parameters defining structure.
 ****************************************************************************/
//void OV9716_ChGain(OV9716_Exposure_t &arExp);

/****************************************************************************/
/** Configures geometry setting to OV9716 camera.
*
* \param  arGeom reference to structure with geometry parameters
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
//CAM_LIB_RESULT OV9716_GeometrySet(OV9716_Geometry_t &arGeom);

/****************************************************************************/
/** Reads Exposure setting from OV9716 camera.
*
* \param  arExp reference to structure with exposure parameters
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
//CAM_LIB_RESULT OV9716_ExposureGet(OV9716_Exposure_t &arExp);

/****************************************************************************/
/** Configures Exposure parameters to OV9716 camera.
*
* \param  arExp reference to structure with exposure parameters
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
//CAM_LIB_RESULT OV9716_ExposureSet(OV9716_Exposure_t &arExp);

/****************************************************************************/
/** Configures operational mode to OV9716 camera.
*
* \param  arGeom reference to structure with geometry parameters
* 
* \return CAM_LIB_SUCCESS if all ok, 
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
//void OV9716_ModeSet(OV9716_Geometry_t &arGeom);

/****************************************************************************/
/** Writes a value over I2C to Ov9716 camera register.
* 
* \param aCsiIdx[in]  Csi interface the camera is connected to
* \param aRegAddr[in] address of the register
* \param aVal[in]     value to be written
* 
* \return CAM_LIB_SUCCESS if all ok
*         CAM_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_RegWrite(CSI_IDX  aCsiIdx, 
                               uint16_t aRegAddr,
                               uint8_t  aVal
                              );

/****************************************************************************/
/** Reads a value over I2C from Ov9716 camera register.
* 
* \param aCsiIdx[in]  Csi interface the camera is connected to
* \param aRegAddr[in] address of the register
* \param aVal[out]    value read
* 
* \return CSI_LIB_SUCCESS if all ok
*         CSI_LIB_FAILURE otherwise
*****************************************************************************/
CAM_LIB_RESULT OV9716_RegRead(CSI_IDX  aCsiIdx, 
                              uint16_t aRegAddr,
                              uint8_t &arVal
                             );

/****************************************************************************/
/** Sets the exposure time parameter of the OV9716 camera for the selected
 *                        exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureTimeSet(OV9716_Exposure_t &arExp,
                                      OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Reads the exposure time parameter of the OV9716 camera for the selected
 *                       exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureTimeGet(OV9716_Exposure_t &arExp,
                                      OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Reads all the exposure parameters of the OV9716 camera for the selected
 *                      exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureAllExpSet(OV9716_Exposure_t &arExp,
                                        OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Sets all the exposure parameters of the OV9716 camera for the selected
 *                      exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureAllExpGet(OV9716_Exposure_t &arExp,
                                        OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Sets the digital gain parameter of the OV9716 camera for the selected
 *                      exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureDigitalGainSet(OV9716_Exposure_t &arExp,
                                             OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Reads the digital gain parameter of the OV9716 camera for the selected
 *                      exposure type.
 * 
 * \param arExp  reference to structure with exposure parameters
 *        aExpT  the exposure type
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureDigitalGainGet(OV9716_Exposure_t &arExp,
                                             OV9716_ExposureType_t aExpT);

/****************************************************************************/
/** Sets the analog gain parameters of the OV9716 camera
 * 
 * \param arExp  reference to structure with exposure parameters
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureAnalogConversionGainSet(OV9716_Exposure_t &arExp);

/****************************************************************************/
/** Reads the analog gain parameter of the OV9716 camera
 * 
 * \param arExp  reference to structure with exposure parameters
 * 
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise
 ****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureAnalogConversionGainGet(OV9716_Exposure_t &arExp);

/*****************************************************************************/
/** OV9716_ChGainSet updates channel gain values for white balancing
*
* \param  arCsiIdx    index to CSI port
* \param  arRGain     the red colour gain
* \param  arGGain     the green colour gain
* \param  arBGain     the blue colour gain
 *****************************************************************************/
CAM_LIB_RESULT OV9716_ChgainSet(CSI_IDX arCsiIdx, double  arRGain,
                                            double  arGGain, double  arBGain);

/*****************************************************************************/
/** OV9716_ExposureSet updates exposure parameters in camera
 *
 * \param  &arExp      the reference to the exposure data structure
 * \param  arLSUpdate  relative update for long+short exposure
 * \param  arVSUpdate  relative update for veryshort exposure
 *****************************************************************************/
CAM_LIB_RESULT OV9716_ExposureSet(OV9716_Exposure_t &arExp,
                                          float aLSUpdate, float aVSUpdate);


#endif // OV9716_PUBLIC_H

/* EOF */
