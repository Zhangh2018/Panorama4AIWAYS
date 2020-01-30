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
 * \file     ov9716_user.cpp
 * \brief    User-space library calls to access Ov9716 functionality.
 * \author   Tomas Babinec
 * \version  0.1
 * \date     14-October-2016
 * \note
 ****************************************************************************/
#include "s32vs234.h"
#include <stdint.h>
#include <errno.h>
#include <string.h>

#ifndef __STANDALONE__
  #include <signal.h>
  #include <fcntl.h>        // open
  #include <unistd.h>       // exit
  #include <sys/mman.h> // mmap
#else
  #include "ov9716_func.h"
#endif // else from #ifndef __STANDALONE__

#define VDB_LOG_NOTE_OFF

#include "ov9716_config_mipi.h"
#include "ov9716_config_viu.h"

#include "isp_cam_generic.h"
#include "cam_lldcmd.h"
#include "isp_cam_ov9716.h"
#include "vdb_log.h"

//****************************************************************************
// consts
//****************************************************************************

#define OV9716_COMPATIBLE_STRING "s32v-ov9716"
#define OV9716_EXP_LINES_MAX  750

#define OV9716_CHIP_ID_H_ADDR 0x300A  ///< chip id High byte
#define OV9716_CHIP_ID_L_ADDR 0x300B  ///< chip id Low byte

#define OV9716_SW_CTRL1_ADDR 0x3012
#define OV9716_SW_CTRL1_STDB 0x0     ///< software standby
#define OV9716_SW_CTRL1_STRM 0x1     ///< streaming

#define OV9716_SW_CTRL2_ADDR 0x3013  ///< bit 0 = software reset

#define OV9716_DVP_H_SIZE_H_ADDR 0x307C  ///< line length high byte
#define OV9716_DVP_H_SIZE_L_ADDR 0x307D
#define OV9716_DVP_V_SIZE_H_ADDR 0x307E  ///< frame length high byte
#define OV9716_DVP_V_SIZE_L_ADDR 0x307F

#define OV9716_HTS_H_ADDR 0x3080  ///< line length high byte
#define OV9716_HTS_L_ADDR 0x3081  ///< line length low byte
#define OV9716_VTS_H_ADDR 0x3082  ///< frame length high byte
#define OV9716_VTS_L_ADDR 0x3083  ///< frame length low byte

#define OV9716_READ_MODE_ADDR 0x3090
#define OV9716_READ_MODE_VR   0x4
#define OV9716_READ_MODE_HR   0x2

#define OV9716_READ_CTRL_ADDR 0x3091
#define OV9716_READ_CTRL_SHOW_STAT 0x8  ///< shows statistics rows
#define OV9716_READ_CTRL_SHOW_REGS 0x4  ///< shows register rows

#define OV9716_REAL_GAIN_L_H_ADDR  0x3105  ///< Long real gain
#define OV9716_REAL_GAIN_L_L_ADDR  0x3106  ///< Long real gain
#define OV9716_REAL_GAIN_S_H_ADDR  0x3107  ///< short real gain
#define OV9716_REAL_GAIN_S_L_ADDR  0x3108  ///< short real gain
#define OV9716_REAL_GAIN_VS_H_ADDR 0x3109  ///< veryshort real gain
#define OV9716_REAL_GAIN_VS_L_ADDR 0x310a  ///< veryshort real gain

#define OV9716_RATIO_LS_H_ADDR  0x310b  ///< long/short ratio
#define OV9716_RATIO_LS_L_ADDR  0x310c  ///< long/short ratio
#define OV9716_RATIO_LVS_H_ADDR 0x310d  ///< long/veryshort ratio
#define OV9716_RATIO_LVS_L_ADDR 0x310e  ///< long/veryshort ratio
#define OV9716_RATIO_SVS_H_ADDR 0x310f  ///< short/veryshort ratio
#define OV9716_RATIO_SVS_L_ADDR 0x3110  ///< short/veryshort ratio

#define OV9716_D_EV_L_3_ADDR 0x3111  ///< Longest EV byte 3
#define OV9716_D_EV_L_2_ADDR 0x3112  ///< Longest EV byte 2
#define OV9716_D_EV_L_1_ADDR 0x3113  ///< Longest EV byte 1
#define OV9716_D_EV_L_0_ADDR 0x3114  ///< Longest EV byte 0

#define OV9716_D_EV_S_3_ADDR 0x3115  ///< Shortest EV byte 3
#define OV9716_D_EV_S_2_ADDR 0x3116  ///< Shortest EV byte 2
#define OV9716_D_EV_S_1_ADDR 0x3117  ///< Shortest EV byte 1
#define OV9716_D_EV_S_0_ADDR 0x3118  ///< Shortest EV byte 0

// long channel gain
#define OV9716_R_R_GAIN_L_I_H_ADDR  0x31c3
#define OV9716_R_R_GAIN_L_I_L_ADDR  0x31c4
#define OV9716_R_GR_GAIN_L_I_H_ADDR 0x31c5
#define OV9716_R_GR_GAIN_L_I_L_ADDR 0x31c6
#define OV9716_R_GB_GAIN_L_I_H_ADDR 0x31c7
#define OV9716_R_GB_GAIN_L_I_L_ADDR 0x31c8
#define OV9716_R_B_GAIN_L_I_H_ADDR  0x31c9
#define OV9716_R_B_GAIN_L_I_L_ADDR  0x31ca
// short channel gain
#define OV9716_R_R_GAIN_S_I_H_ADDR  0x31cb
#define OV9716_R_R_GAIN_S_I_L_ADDR  0x31cc
#define OV9716_R_GR_GAIN_S_I_H_ADDR 0x31cd
#define OV9716_R_GR_GAIN_S_I_L_ADDR 0x31ce
#define OV9716_R_GB_GAIN_S_I_H_ADDR 0x31cf
#define OV9716_R_GB_GAIN_S_I_L_ADDR 0x31d0
#define OV9716_R_B_GAIN_S_I_H_ADDR  0x31d1
#define OV9716_R_B_GAIN_S_I_L_ADDR  0x31d2
// very short channel gain
#define OV9716_R_R_GAIN_VS_I_H_ADDR  0x31d3
#define OV9716_R_R_GAIN_VS_I_L_ADDR  0x31d4
#define OV9716_R_GR_GAIN_VS_I_H_ADDR 0x31d5
#define OV9716_R_GR_GAIN_VS_I_L_ADDR 0x31d6
#define OV9716_R_GB_GAIN_VS_I_H_ADDR 0x31d7
#define OV9716_R_GB_GAIN_VS_I_L_ADDR 0x31d8
#define OV9716_R_B_GAIN_VS_I_H_ADDR  0x31d9
#define OV9716_R_B_GAIN_VS_I_L_ADDR  0x31da

// **************************************************
// **** OV9716 registers address
// Frame DCG (HCG/LCG) Exposure (in Rows)
#define OV9716_CEXP_DCG_H                       0x30B6
#define OV9716_CEXP_DCG_L                       0x30B7
// Frame VS Exposure
#define OV9716_CEXP_VS_H                        0x30B8
#define OV9716_CEXP_VS_M                        0x30B9
// Fractional part 5bit (0 to 0x1F)
#define OV9716_CEXP_VS_L                        0x30Ba

#define OV9716_VS_DELAY_H                       0x30CE
#define OV9716_VS_DELAY_L                       0x30CF
#define OV9716_DIG_GAIN_HCG_H                   0x3162
#define OV9716_DIG_GAIN_HCG_L                   0x3163
#define OV9716_DIG_GAIN_LCG_H                   0x3164
#define OV9716_DIG_GAIN_LCG_L                   0x3165
#define OV9716_DIG_GAIN_VS_H                    0x3166
#define OV9716_DIG_GAIN_VS_L                    0x3167
#define OV9716_CG_GAIN                          0x30BB
#define OV9716_CHGAIN_L                         0x3360
#define OV9716_CHGAIN_S                         0x3368
#define OV9716_CHGAIN_VS                        0x3370

#define OV9716_GRP_SEL                          0x3464
#define OV9716_GRP_HOLD                         0x3467
#define OV9716_GRP                              0x8000

#define OV9716_GRP_CEXP_DCG_H      (OV9716_CEXP_DCG_H     | OV9716_GRP)
#define OV9716_GRP_CEXP_DCG_L      (OV9716_CEXP_DCG_L     | OV9716_GRP)
#define OV9716_GRP_CEXP_VS_H       (OV9716_CEXP_VS_H      | OV9716_GRP)
#define OV9716_GRP_CEXP_VS_L       (OV9716_CEXP_VS_L      | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_HCG_H  (OV9716_DIG_GAIN_HCG_H | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_HCG_L  (OV9716_DIG_GAIN_HCG_L | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_LCG_H  (OV9716_DIG_GAIN_LCG_H | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_LCG_L  (OV9716_DIG_GAIN_LCG_L | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_VS_H   (OV9716_DIG_GAIN_VS_H  | OV9716_GRP)
#define OV9716_GRP_DIG_GAIN_VS_L   (OV9716_DIG_GAIN_VS_L  | OV9716_GRP)
#define OV9716_GRP_CG_GAIN         (OV9716_CG_GAIN        | OV9716_GRP)
#define OV9716_GRP_CHGAIN_L        (OV9716_CHGAIN_L)//       | OV9716_GRP)
#define OV9716_GRP_CHGAIN_S        (OV9716_CHGAIN_S       | OV9716_GRP)
#define OV9716_GRP_CHGAIN_VS       (OV9716_CHGAIN_VS      | OV9716_GRP)

#ifdef __STANDALONE__
extern "C"
{
  unsigned long get_uptime_microS(void);
}
  #define msleep(_val) \
    {\
      unsigned long lStart = get_uptime_microS();\
      while((get_uptime_microS() - lStart) < _val*1000);\
    }
#endif // else from #ifdef __STANDALONE__

//****************************************************************************
// types
//****************************************************************************

//****************************************************************************
// global variables
//****************************************************************************
#ifndef __STANDALONE__
  extern int32_t gCamFD;   ///< cam driver file descriptor
#endif //ifndef __STANDALONE__

//*****************************************************************************
// local data
//*****************************************************************************

static CamI2cClient_t spClients[2] = {0}; // initialize to invalid

// 12bit combined exp by default
static uint32_t       sExpMode     = OV9716_EXP_12COM;

static I2C_BUS_IDX    sViuI2cBus[] = {I2C_BUS_IDX_1, I2C_BUS_IDX_0};

CAM_LIB_RESULT OV9716_ExposureDCGComputation(OV9716_Exposure_t &arExp,
                                             float arNewHCGExpTotal);

void OV9716_ExposureTotal(OV9716_Exposure_t &arExp, OV9716_ExposureType_t aExpT);
//*****************************************************************************
// functions definitions
//*****************************************************************************

// *** MipiCsi-2 ***

CAM_LIB_RESULT OV9716_Open(CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  CamI2cCfg_t    lCamCfg;

  if(CAM_Open() == CAM_LIB_SUCCESS)
  {
    if(spClients[aCsiIdx] == 0)
    {
#ifndef __STANDALONE__
      // power up cam
      if(CAM_LIB_FAILURE ==  CAM_PowerControl(aCsiIdx, SENSOR_TYPE_OV9716, 
                                              CAM_POWER_STATE_UP))
      {
        VDB_LOG_ERROR("Power up csi #%d fail.\n", aCsiIdx);
        lRet = CAM_LIB_FAILURE;
      }
#endif // ifndef __STANDALONE__
      lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
      lCamCfg.mI2cAddr      = I2C_OV9716_CAM_ADDR;
      lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;

      memcpy(lCamCfg.mpCompatible,
              OV9716_COMPATIBLE_STRING,
              strlen(OV9716_COMPATIBLE_STRING));


      spClients[aCsiIdx] = CAM_ClientAdd(lCamCfg);
    } // if not created yet
    else
    {
      VDB_LOG_ERROR("Ov9716 is already registered on CSI idx %u.\n",
                    (uint32_t)aCsiIdx);
      lRet = CAM_LIB_FAILURE;
    } // else if not created yet

    if(spClients[aCsiIdx] != 0)
    {
      int32_t lRetVal = 0;
#ifdef __STANDALONE__
      // TODO: implement OV9716_DRV_Setup for standalone and active below code
      // lRetVal = OV9716_DRV_Setup(spClients[aCsiIdx])
#endif // else #ifndef __STANDALONE__
      if (lRetVal < 0)
      {
        VDB_LOG_ERROR("Failed to connect the camera driver.\n");
        lRet = CAM_LIB_FAILURE;
      } // if ioctl returned failure
      else
      {
        uint32_t  lChipId   = 0;
        uint8_t  *lpChipIdH = ((uint8_t*)&lChipId + 1);
        uint8_t  *lpChipIdL = (uint8_t*)&lChipId;

        CAM_ReadSingle(spClients[aCsiIdx],
                      (uint16_t)OV9716_CHIP_ID_H_ADDR,
                      *lpChipIdH);
        CAM_ReadSingle(spClients[aCsiIdx],
                      (uint16_t)OV9716_CHIP_ID_L_ADDR,
                      *lpChipIdL);

        printf("Ov9716 chip ID 0x%x at csi_idx: %u\n", lChipId, aCsiIdx);
      } // else from if ioctl returned failure
    } // if client added successfully
    else
    {
      VDB_LOG_ERROR(
        "Failed to add Sony camera client: CSI idx %u, address 0x%x.\n",
        aCsiIdx,
        lCamCfg.mI2cAddr
      );
      lRet = CAM_LIB_FAILURE;
    } // else if client added successfully
  } // if generic camera driver ready
  else
  {
    VDB_LOG_ERROR("Failed to open generic camera driver.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if generic camera driver ready

  return lRet;
} // OV9716_Open()

//*****************************************************************************

CAM_LIB_RESULT OV9716_Close(CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  if(spClients[aCsiIdx] != 0)
  {
    int32_t lRetVal = 0;
#ifdef __STANDALONE__
    // TODO: implement OV9716_DRV_Close for standalone and active below code
    // lRetVal = OV9716_DRV_Close(spClients[aCsiIdx])
#endif // else #ifndef __STANDALONE__
#ifndef __STANDALONE__
    if(CAM_LIB_FAILURE ==  CAM_PowerControl(aCsiIdx, SENSOR_TYPE_OV9716,
                                            CAM_POWER_STATE_DOWN))
    {
      VDB_LOG_ERROR("Power down viu #%d fail.\n", aCsiIdx);
      lRet = CAM_LIB_FAILURE;
    }
#endif // ifndef __STANDALONE__
    if (lRetVal < 0)
    {
      VDB_LOG_ERROR("Failed to disconnect the camera driver.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ioctl returned failure

    if(CAM_ClientRem(spClients[aCsiIdx]) != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to remove the camera client.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ClientRem() failed
    else
    {
      spClients[aCsiIdx] = 0;
    } // else from if CLientRem() failed
  } // if camera client attached
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached
  return lRet;

} // OV9716_Close()

//*****************************************************************************

CAM_LIB_RESULT OV9716_RegConfig(CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  if(spClients[aCsiIdx] != 0)
  {
    uint32_t i         = 0;
    uint8_t *lpOvTable =
        //Ov9716_MIPI_1288_728_12bDCG_12bVS;
        Ov9716_MIPI_1288_728_12bDCG_12bVS_noVC;
    uint32_t lOvTabSz  =
       // sizeof(Ov9716_MIPI_1288_728_12bDCG_12bVS);
          sizeof(Ov9716_MIPI_1288_728_12bDCG_12bVS_noVC);

    //put camera to standby mode
    OV9716_CsiDisable(aCsiIdx);

    // *** basic configuration ***
    for(i = 0; i < lOvTabSz; i = i + 3)
    {
      CAM_WriteSingle(spClients[aCsiIdx],
                      (uint16_t)((lpOvTable[i] << 8) | lpOvTable[i + 1]),
                      (uint8_t)(lpOvTable[i + 2]));
      if(i==0 || i==3 || i==6)
      {
        msleep(100);
      }
    }// for all table entries

    sExpMode = OV9716_EXP_12COM;
  } // if camera client attached
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached

  return lRet;
} // CSI_RegConfig(CSI_IDX aCsiIdx)

//*****************************************************************************

CAM_LIB_RESULT OV9716_CsiEnable(CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  uint8_t lVal = OV9716_SW_CTRL1_STDB;

  // bring camera out of standby
  CAM_WriteSingle(spClients[aCsiIdx],
                  (uint16_t)OV9716_SW_CTRL1_ADDR,
                  (uint8_t)OV9716_SW_CTRL1_STRM);

  while(lVal != OV9716_SW_CTRL1_STRM)
  {
    CAM_ReadSingle(spClients[aCsiIdx], (uint16_t)OV9716_SW_CTRL1_ADDR, lVal);
  } // wait for streaming

  return lRet;
} // OV9716_CsiEnable(CSI_IDX aCsiIdx)

//*****************************************************************************

CAM_LIB_RESULT OV9716_CsiDisable(enum CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  uint8_t lVal = OV9716_SW_CTRL1_STRM;
  // bring camera to standby
  CAM_WriteSingle(spClients[aCsiIdx],
                  (uint16_t)OV9716_SW_CTRL1_ADDR,
                  (uint8_t)OV9716_SW_CTRL1_STDB);

  while(lVal != OV9716_SW_CTRL1_STDB)
  {
    CAM_ReadSingle(spClients[aCsiIdx], (uint16_t)OV9716_SW_CTRL1_ADDR, lVal);
  } // wait for standby

  return lRet;
} // OV9716_CsiDisable(enum CSI_IDX aCsiIdx)

//***************************************************************************

// *** VIU ***

CAM_LIB_RESULT OV9716_Open(VIU_IDX aViuIdx)
{
  CAM_LIB_RESULT lRet       = CAM_LIB_SUCCESS;
  CamI2cCfg_t    lCamCfg;
  uint32_t       lClientIdx = sViuI2cBus[aViuIdx];

  if(CAM_Open() == CAM_LIB_SUCCESS)
  {
    if(spClients[lClientIdx] == 0)
    {
      lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) lClientIdx;
      lCamCfg.mI2cAddr      = I2C_OV9716_CAM_ADDR;
      lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;

      memcpy(lCamCfg.mpCompatible,
              OV9716_COMPATIBLE_STRING,
              strlen(OV9716_COMPATIBLE_STRING));


      spClients[lClientIdx] = CAM_ClientAdd(lCamCfg);
    } // if not created yet
    else
    {
      VDB_LOG_ERROR("Ov9716 is already registered on VIU idx %u.\n",
                    (uint32_t)aViuIdx);
      lRet = CAM_LIB_FAILURE;
    } // else if not created yet

    if(spClients[lClientIdx] != 0)
    {
      uint32_t  lChipId   = 0;
      uint8_t  *lpChipIdH = ((uint8_t*)&lChipId + 1);
      uint8_t  *lpChipIdL = (uint8_t*)&lChipId;

      CAM_ReadSingle(spClients[lClientIdx],
                      (uint16_t)OV9716_CHIP_ID_H_ADDR,
                      *lpChipIdH);
      CAM_ReadSingle(spClients[lClientIdx],
                      (uint16_t)OV9716_CHIP_ID_L_ADDR,
                      *lpChipIdL);

      printf("Ov9716 chip ID 0x%x at VIU idx: %u\n", lChipId, aViuIdx);
    } // if client added successfully
    else
    {
      VDB_LOG_ERROR(
        "Failed to add Sony camera client: VIU idx %u, address 0x%x.\n",
        aViuIdx,
        lCamCfg.mI2cAddr
      );
      lRet = CAM_LIB_FAILURE;
    } // else if client added successfully
  } // if generic camera driver ready
  else
  {
    VDB_LOG_ERROR("Failed to open generic camera driver.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if generic camera driver ready

  return lRet;
} // OV9716_Open()

//*****************************************************************************

CAM_LIB_RESULT OV9716_Close(VIU_IDX aViuIdx)
{
  CAM_LIB_RESULT lRet       = CAM_LIB_SUCCESS;
  uint32_t       lClientIdx = sViuI2cBus[aViuIdx];

  if(spClients[lClientIdx] != 0)
  {
    if(CAM_ClientRem(spClients[lClientIdx]) != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to remove the camera client.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ClientRem() failed
    else
    {
      spClients[lClientIdx] = 0;
    } // else from if CLientRem() failed
  } // if camera client attached
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached
  return lRet;

} // OV9716_Close()

//*****************************************************************************

CAM_LIB_RESULT OV9716_RegConfig(VIU_IDX aViuIdx)
{
  CAM_LIB_RESULT lRet       = CAM_LIB_SUCCESS;
  uint32_t       lClientIdx = sViuI2cBus[aViuIdx];

  if(spClients[lClientIdx] != 0)
  {
    uint32_t i         = 0;
    uint8_t *lpOvTable = Ov9716Viu_Table;
    uint32_t lOvTabSz  = sizeof(Ov9716Viu_Table);

    // *** basic configuration ***
    for(i = 0; i < lOvTabSz; i = i + 3)
    {
      CAM_WriteSingle(spClients[lClientIdx],
                      (uint16_t)((lpOvTable[i] << 8) | lpOvTable[i + 1]),
                      (uint8_t)(lpOvTable[i + 2]));
    }// for all table entries
  } // if camera client attached
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached

  return lRet;
} // OV9716_RegConfig()

//*****************************************************************************

CAM_LIB_RESULT OV9716_DataEnable(VIU_IDX aViuIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  uint8_t lVal = OV9716_SW_CTRL1_STDB;

  // bring camera out of standby
  CAM_WriteSingle(spClients[sViuI2cBus[aViuIdx]],
                  (uint16_t)OV9716_SW_CTRL1_ADDR,
                  (uint8_t)OV9716_SW_CTRL1_STRM);

  while(lVal != OV9716_SW_CTRL1_STRM)
  {
    CAM_ReadSingle(spClients[sViuI2cBus[aViuIdx]],
                   (uint16_t)OV9716_SW_CTRL1_ADDR, lVal);
  } // wait for streaming

  return lRet;
} // OV9716_DataEnable()

//*****************************************************************************

CAM_LIB_RESULT OV9716_DataDisable(VIU_IDX aViuIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  uint8_t lVal = OV9716_SW_CTRL1_STRM;
  // bring camera to standby
  CAM_WriteSingle(spClients[sViuI2cBus[aViuIdx]],
                  (uint16_t)OV9716_SW_CTRL1_ADDR,
                  (uint8_t)OV9716_SW_CTRL1_STDB);

  while(lVal != OV9716_SW_CTRL1_STDB)
  {
    CAM_ReadSingle(spClients[sViuI2cBus[aViuIdx]],
                   (uint16_t)OV9716_SW_CTRL1_ADDR, lVal);
  } // wait for standby

  return lRet;
} // OV9716_DataDisable()

//***************************************************************************

// *** GENERIC ***

CAM_LIB_RESULT OV9716_RegWrite(CSI_IDX  aCsiIdx,
                                uint16_t aRegAddr,
                                uint8_t  aVal
                                )
{
  return CAM_WriteSingle(spClients[aCsiIdx], aRegAddr, aVal);
} // OV9716_RegWrite

//*****************************************************************************

CAM_LIB_RESULT OV9716_RegRead(CSI_IDX  aCsiIdx,
                               uint16_t aRegAddr,
                               uint8_t &arVal
                               )
{
  return CAM_ReadSingle(spClients[aCsiIdx], aRegAddr, arVal);
} // OV9716_RegRead


//***************************************************************************
CAM_LIB_RESULT OV9716_ExposureTimeSet(OV9716_Exposure_t &arExp,
                                      OV9716_ExposureType_t aExpT)
{
  uint8_t  lTemp;
  uint16_t lExpTime;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  switch(aExpT)
  {
    case OV9716_ExpType_VS:
      lExpTime = arExp.mVSExposureTime;
      lTemp = (uint8_t)lExpTime & 0x1f; //Only change fractional part to not change the delay
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_CEXP_VS_L, lTemp);
      break;

    case OV9716_ExpType_LCG:
      lExpTime = arExp.mDCGExposureTime;
      if(lExpTime > OV9716_EXP_LINES_MAX)
      {
        lExpTime = OV9716_EXP_LINES_MAX;
      }
      if(lExpTime < 2)
      {
        lExpTime = 2;
      }

      lTemp = (uint8_t)(lExpTime >> 8);
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_CEXP_DCG_H, lTemp);
      if (CAM_LIB_SUCCESS == lRet)
      {
        lTemp = (uint8_t)lExpTime;
        lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_CEXP_DCG_L, lTemp);
      }
      break;

    case OV9716_ExpType_HCG:
      lExpTime = arExp.mDCGExposureTime;
      if(lExpTime > OV9716_EXP_LINES_MAX)
      {
        lExpTime = OV9716_EXP_LINES_MAX;
      }
      if(lExpTime < 2)
      {
        lExpTime = 2;
      }

      lTemp = (uint8_t)(lExpTime >> 8);
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_CEXP_DCG_H, lTemp);
      if (CAM_LIB_SUCCESS == lRet)
      {
        lTemp = (uint8_t)lExpTime;
        lRet = CAM_WriteSingle(lClient,
                              (uint16_t)OV9716_CEXP_DCG_L, lTemp);
      }
      break;
  }
  return lRet;
}

//***************************************************************************
CAM_LIB_RESULT OV9716_ExposureTimeGet(OV9716_Exposure_t &arExp,
                                      OV9716_ExposureType_t aExpT)
{
  uint8_t  lTemp;
  uint16_t lExpTime;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  switch(aExpT)
  {
    case OV9716_ExpType_VS:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_VS_M, lTemp);
      lExpTime = lTemp << 5;

      if(CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_VS_L, lTemp);
        lExpTime += lTemp & 0x1f;
        arExp.mVSExposureTime = lExpTime;
      }
      break;

    case OV9716_ExpType_LCG:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_DCG_H, lTemp);
      lExpTime = lTemp << 8;

      if(CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_DCG_L, lTemp);
        lExpTime += lTemp;
        arExp.mDCGExposureTime = lExpTime;
      }
      break;

    case OV9716_ExpType_HCG:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_DCG_H, lTemp);
      lExpTime= lTemp<<8;

      if(CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CEXP_DCG_L, lTemp);
        lExpTime += lTemp;
        arExp.mDCGExposureTime = lExpTime;
      }
      break;
  }
//  arExp.mExposureTime[exp] = lExpTime;
  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT OV9716_ExposureAllExpSet(OV9716_Exposure_t &arExp,
                                        OV9716_ExposureType_t aExpT)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = OV9716_ExposureTimeSet(arExp,aExpT);

  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = OV9716_ExposureDigitalGainSet(arExp,aExpT);
  }
  return lRet;
}

//****************************************************************************
CAM_LIB_RESULT OV9716_ExposureAllExpGet(OV9716_Exposure_t &arExp,
                                        OV9716_ExposureType_t aExpT)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = OV9716_ExposureTimeGet(arExp,aExpT);

  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = OV9716_ExposureAnalogConversionGainGet(arExp);
  }

  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = OV9716_ExposureDigitalGainGet(arExp,aExpT);
  }
  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT OV9716_ExposureDigitalGainSet(OV9716_Exposure_t &arExp,
                                             OV9716_ExposureType_t aExpT)
{
  uint8_t  lGain;
  uint16_t lExpGain = arExp.mDigitalGain[aExpT];
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  switch(aExpT){
    case OV9716_ExpType_VS:
      lGain = (uint8_t)(lExpGain >> 8);
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_VS_H, lGain);

      if (CAM_LIB_SUCCESS == lRet)
      {
        lGain = (uint8_t)lExpGain;
        lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_VS_L, lGain);
      }
      break;

    case OV9716_ExpType_LCG:
      lGain = (uint8_t)(lExpGain >> 8);
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_LCG_H, lGain);

      if (CAM_LIB_SUCCESS == lRet)
      {
        lGain = (uint8_t)lExpGain;
        lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_LCG_L, lGain);
      }
      break;

    case OV9716_ExpType_HCG:
      lGain = (uint8_t)(lExpGain >> 8);
      lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_HCG_H, lGain);

      if (CAM_LIB_SUCCESS == lRet)
      {
        lGain = (uint8_t)lExpGain;
        lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_DIG_GAIN_HCG_L, lGain);
      }
      break;
  }
  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT OV9716_ExposureDigitalGainGet(OV9716_Exposure_t &arExp,
                                             OV9716_ExposureType_t aExpT)
{
  uint8_t  lTemp;
  uint16_t lDigGain;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  switch(aExpT)
  {
    case OV9716_ExpType_VS:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_VS_H, lTemp);
      lDigGain = lTemp << 8;

      if (CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_VS_L, lTemp);
        lDigGain += lTemp;
      }
      break;

    case OV9716_ExpType_LCG:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_LCG_H, lTemp);
      lDigGain = lTemp << 8;

      if (CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_LCG_L, lTemp);
        lDigGain += lTemp;
      }
      break;

    case OV9716_ExpType_HCG:
      lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_HCG_H, lTemp);
      lDigGain = lTemp << 8;

      if (CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_DIG_GAIN_HCG_L, lTemp);
        lDigGain += lTemp;
      }
      break;
  }
  arExp.mDigitalGain[aExpT] = lDigGain;
  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT OV9716_ExposureAnalogConversionGainSet(OV9716_Exposure_t &arExp)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint8_t lGain = (arExp.mVSConversionGainOn << 7) + arExp.mAnalogGain[OV9716_ExpType_HCG]
                + (arExp.mAnalogGain[OV9716_ExpType_LCG] << 2) + (arExp.mAnalogGain[OV9716_ExpType_VS] << 4);

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  lRet = CAM_WriteSingle(lClient, (uint16_t)OV9716_CG_GAIN, lGain);
  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT OV9716_ExposureAnalogConversionGainGet(OV9716_Exposure_t &arExp)
{
  uint8_t lGain, lAnaGain, lConvGain;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  lRet = CAM_ReadSingle(lClient, (uint16_t)OV9716_CG_GAIN, lGain);

  if (CAM_LIB_SUCCESS == lRet)
  {
    lAnaGain  = (lGain >> 4) & 0x3;
    lConvGain = (lGain >> 7) & 0x1;
    arExp.mAnalogGain[OV9716_ExpType_VS] = lAnaGain; // x1, x2, x4 or x8
    arExp.mVSConversionGainOn = lConvGain;

    lAnaGain = (lGain >> 2) & 0x3;
    arExp.mAnalogGain[OV9716_ExpType_LCG] = lAnaGain; // x1, x2, x4 or x8

    lAnaGain = lGain & 0x3;
    arExp.mAnalogGain[OV9716_ExpType_HCG] = lAnaGain; // x1, x2, x4 or x8
  }
  return lRet;
}

// **************************************************************************
CAM_LIB_RESULT OV9716_ChgainSet(CSI_IDX arCsiIdx, double  arRGain,
                                               double  arGGain, double  arBGain)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  for (uint8_t i = 0; i < 3; i++)
  {
    lRet = CAM_WriteSingle(spClients[arCsiIdx],
                           (uint16_t)(OV9716_GRP_CHGAIN_L + i*8),
                           (uint8_t)(((uint32_t)arRGain) >> 8));
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 1 + i*8),
                             (uint8_t)(((uint32_t)arRGain) & 0xff));
    }

    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 2 + i*8),
                             (uint8_t)(((uint32_t)arGGain) >> 8));
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 3 + i*8),
                             (uint8_t)(((uint32_t)arGGain) & 0xff));
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 4 + i*8),
                             (uint8_t)(((uint32_t)arGGain) >> 8));
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 5 + i*8),
                             (uint8_t)(((uint32_t)arGGain) & 0xff));
    }

    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 6 + i*8),
                             (uint16_t)(((uint32_t)arBGain) >> 8));
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(spClients[arCsiIdx],
                             (uint16_t)(OV9716_GRP_CHGAIN_L + 7 + i*8),
                             (uint8_t)(((uint32_t)arBGain) & 0xff));
    }
    msleep(2);
  } // for all color compononents
  return lRet;
}

// **************************************************************************
void OV9716_ExposureTotal(OV9716_Exposure_t &arExp, OV9716_ExposureType_t aExpT)
{
  if(aExpT == OV9716_ExpType_HCG)
  {
    arExp.mExposure[OV9716_ExpType_HCG] = (float)(arExp.mDCGExposureTime) *
                               (1<<arExp.mAnalogGain[(uint8_t)OV9716_ExpType_HCG]) *
                           ((float)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_HCG])/256); // 6.8b value
  }
  else if(aExpT == OV9716_ExpType_LCG)
  {
    arExp.mExposure[OV9716_ExpType_LCG] = (float)(arExp.mDCGExposureTime) *
                               (1<<arExp.mAnalogGain[(uint8_t)OV9716_ExpType_LCG]) *
                           ((float)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_LCG])/256); // 6.8b value
  }
  else
  {
    arExp.mExposure[OV9716_ExpType_VS] = ((float)(arExp.mVSExposureTime)/32) *     //8.5b
                                   (1<<arExp.mAnalogGain[(uint8_t)OV9716_ExpType_VS]) *
                                      arExp.mVSConversionGain *
                          ((float)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_VS])/256); // 6.8b value
  }
}

// ****************************************************************************
CAM_LIB_RESULT OV9716_ExposureDCGComputation(OV9716_Exposure_t &arExp,
                                                    float arNewHCGExpTotal)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  float    lNewHCGExposure, lNewLCGExposure, lNewExposure;
  float    lExpTime, lTempExp;
  float    lFloatDigGain, lGain, lRatio;
  uint8_t  lCurrCG, lAnGain;
  uint16_t lDigGain, lExpCoef=0;

  lNewHCGExposure = arNewHCGExpTotal;

  /* Maintain the ratio between LCG and HCG, saturation of the new exposure based on min/max */
  if( arNewHCGExpTotal > arExp.mMaxExposure[OV9716_ExpType_HCG])
  {
    /* Saturate HCG exposure to maximum */
    lNewHCGExposure = arExp.mMaxExposure[OV9716_ExpType_HCG];
  }
  else if(arNewHCGExpTotal < arExp.mMinExposure[OV9716_ExpType_HCG])
  {
    /* Saturate HCG exposure to minimum */
    lNewHCGExposure = arExp.mMinExposure[OV9716_ExpType_HCG];
  }

  /* Calculate LCG exposure based on HCG/LCG ratio */
  lNewLCGExposure = lNewHCGExposure / arExp.mRatioHLExp;
  if(lNewLCGExposure > arExp.mMaxExposure[OV9716_ExpType_LCG])
  {
    /* Saturate LCG exposure to maximum and
	recompute HCG exposure to keep ratio */
    lNewLCGExposure = arExp.mMaxExposure[OV9716_ExpType_LCG];
    lNewHCGExposure = lNewLCGExposure*arExp.mRatioHLExp;
  }
  else if(lNewLCGExposure < arExp.mMinExposure[OV9716_ExpType_LCG])
  {
    /* Saturate LCG exposure to minimum and recompute HCG exposure */
    lNewLCGExposure = arExp.mMinExposure[OV9716_ExpType_LCG];
    lNewHCGExposure = lNewLCGExposure * arExp.mRatioHLExp;
  }

  /* Compute one of the exposure (HCG or LCG) based of the priority parameter */
  lCurrCG = arExp.mExposurePriority;
  if(lCurrCG == (uint8_t)OV9716_ExpType_HCG)
  {
    lNewExposure = lNewHCGExposure;
  }
  else{
    lNewExposure = lNewLCGExposure;
  }

  lAnGain = arExp.mMinAnalogGain[lCurrCG];
  lDigGain = arExp.mMinDigitalGain[lCurrCG];
  lGain = (1 << lAnGain) * (float)(lDigGain) / 256;

  while((lAnGain < arExp.mMaxAnalogGain[lCurrCG])
          && (lNewExposure/lGain) >= arExp.mMaxDCGExposureTime)
  {  /* Maximising exposure time is not enough to reach
     new exposure and analog gain can be increased */
     lAnGain++;
     lGain = lGain * 2;
  }
  lExpTime = lNewExposure / lGain;

  if(lExpTime > arExp.mMaxDCGExposureTime)
  {  /* Maximise exposure time but also need to increase digital gain */
     lExpTime = arExp.mMaxDCGExposureTime;
     lFloatDigGain = lNewExposure/ (arExp.mMaxDCGExposureTime * (1<<lAnGain));
     lDigGain = (uint16_t)(lFloatDigGain * 256 + 0.5);
  }
  else if(arExp.mDCGDeflickeringEnable)   /* Deflickering */
  {
    lExpCoef = (uint16_t)(lExpTime / arExp.mDCGFlickerPeriod);
    if(lExpCoef < 1)
    {
      lExpCoef = 1;
    }
    lExpTime = lExpCoef*arExp.mDCGFlickerPeriod;
    /* Compensate with digital gain */
    lFloatDigGain = lNewExposure/ (lExpTime * (1 << lAnGain));
    lDigGain = (uint16_t)(lFloatDigGain * 256 + 0.5);
    arExp.mDCGExpCoef = lExpCoef;
  }

  arExp.mAnalogGain[lCurrCG] = lAnGain;
  arExp.mDigitalGain[lCurrCG] = lDigGain;
  arExp.mDCGExposureTime = (uint16_t)(lExpTime + 0.5);
  OV9716_ExposureTotal(arExp, (OV9716_ExposureType_t)lCurrCG);


  /* Compute second exposure parameters (exposure time is now fixed) */
  if(lCurrCG == OV9716_ExpType_HCG)
  {
    lNewExposure = lNewLCGExposure;
    lCurrCG = OV9716_ExpType_LCG;
  }
  else
  {
    lNewExposure = lNewHCGExposure;;
    lCurrCG = OV9716_ExpType_HCG;
  }

  lAnGain = arExp.mMinAnalogGain[lCurrCG];
  lDigGain = arExp.mMinDigitalGain[lCurrCG];
  lGain = (1 << lAnGain) * (float)(lDigGain) / 256;
  lTempExp = lExpTime * lGain;
  while((lAnGain < arExp.mMaxAnalogGain[lCurrCG])
          && (lTempExp * 2 < lNewExposure))
  {
          lAnGain++;
          lTempExp = lTempExp*2;
  }
  /* Adjust Digital gain */
  lFloatDigGain = lNewExposure / (lExpTime * (1 << lAnGain));
  lDigGain = (uint16_t)(lFloatDigGain * 256 + 0.5);

  arExp.mAnalogGain[lCurrCG] = lAnGain;
  arExp.mDigitalGain[lCurrCG] = lDigGain;
  OV9716_ExposureTotal(arExp,(OV9716_ExposureType_t)lCurrCG);

  /* Verify that parameters are whithin limits */
  if((arExp.mDigitalGain[OV9716_ExpType_LCG] < arExp.mMinDigitalGain[OV9716_ExpType_LCG])
          || (arExp.mDigitalGain[OV9716_ExpType_LCG] > arExp.mMaxDigitalGain[OV9716_ExpType_LCG]))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Digital Gain of LCG out of limit [%d ; %d]: %d\n",
                arExp.mMinDigitalGain[OV9716_ExpType_LCG],
                arExp.mMaxDigitalGain[OV9716_ExpType_LCG],
                arExp.mDigitalGain[OV9716_ExpType_LCG]);
    */
  }
  if((arExp.mAnalogGain[OV9716_ExpType_LCG] < arExp.mMinAnalogGain[OV9716_ExpType_LCG])
          || (arExp.mAnalogGain[OV9716_ExpType_LCG] > arExp.mMaxAnalogGain[OV9716_ExpType_LCG]))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Analog Gain of LCG out of limit [%d ; %d]: %d\n",
                1<<arExp.mMinAnalogGain[OV9716_ExpType_LCG],
                1<<arExp.mMaxAnalogGain[OV9716_ExpType_LCG],
                1<<arExp.mAnalogGain[OV9716_ExpType_LCG]);
    */
  }
  if((arExp.mDigitalGain[OV9716_ExpType_HCG] < arExp.mMinDigitalGain[OV9716_ExpType_HCG])
          || (arExp.mDigitalGain[OV9716_ExpType_HCG] > arExp.mMaxDigitalGain[OV9716_ExpType_HCG]))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Digital Gain of HCG out of limit [%d ; %d]: %d\n",
                arExp.mMinDigitalGain[OV9716_ExpType_HCG],
                arExp.mMaxDigitalGain[OV9716_ExpType_HCG],
                arExp.mDigitalGain[OV9716_ExpType_HCG]);
    */
  }
  if((arExp.mAnalogGain[OV9716_ExpType_HCG] < arExp.mMinAnalogGain[OV9716_ExpType_HCG])
          || (arExp.mAnalogGain[OV9716_ExpType_HCG] > arExp.mMaxAnalogGain[OV9716_ExpType_HCG]))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Analog Gain of HCG out of limit [%d ; %d]: %d\n",
                1<<arExp.mMinAnalogGain[OV9716_ExpType_HCG],
                1<<arExp.mMaxAnalogGain[OV9716_ExpType_HCG],
                1<<arExp.mAnalogGain[OV9716_ExpType_HCG]);
    */
  }
  if((arExp.mDCGExposureTime < arExp.mMinDCGExposureTime)
          || (arExp.mDCGExposureTime > arExp.mMaxDCGExposureTime))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Exposure time of DCG is out of limit [%d ; %d]: %d\n",
                1<<arExp.mMinDCGExposureTime,
                1<<arExp.mMaxDCGExposureTime,
                1<<arExp.mDCGExposureTime);
    */
  }

  /* Verify that the ratio between the exposure is ok */
  lRatio = arExp.mExposure[OV9716_ExpType_HCG] / arExp.mExposure[OV9716_ExpType_LCG];
  if((lRatio <= arExp.mRatioMargin * arExp.mRatioHLExp)
          || (lRatio >= (1 / arExp.mRatioMargin) * arExp.mRatioHLExp))
  {
    lRet = CAM_LIB_FAILURE;
    /*  printf("Ratio between HCG and LCG is not maintained\n    HCG exp:%f\n
                LCG exp:%f\n    Target ratio:%f\n    Current ratio:%f\n",
                arExp.mExposure[OV9716_ExpType_HCG],
                arExp.mExposure[OV9716_ExpType_LCG],
                arExp.mRatioHLExp,
                lRatio);
    */
  }
  return lRet;
} // OV9716_ExposureDCGComputation()

// ****************************************************************************
CAM_LIB_RESULT OV9716_ExposureSet(OV9716_Exposure_t &arExp,
                                  float aLSUpdate, float aVSUpdate)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamI2cClient_t lClient = arExp.mClient;
  if(lClient == 0)
  {
   lClient = spClients[arExp.mCsiIdx];
  } // if the client not specified directly

  /* check if increase is out of bounds */
  if (aLSUpdate >= 2.0)
  {
    aLSUpdate =2.0;
  }

  OV9716_ExposureDCGComputation(arExp, aLSUpdate*arExp.mExposure[(uint8_t)OV9716_ExpType_HCG]);

  /* Update new values for L/S exposures */
  lRet = CAM_WriteSingle(lClient,
                         (uint16_t)OV9716_GRP_CEXP_DCG_H,
                         (uint8_t)(arExp.mDCGExposureTime >> 8));
  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_CEXP_DCG_L,
                           (uint8_t)(arExp.mDCGExposureTime & 0xff));
  }

  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_CG_GAIN,
                           (uint8_t)(((arExp.mVSConversionGainOn & 1) << 7) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_VS] & 3) <<4) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_LCG] & 3) <<2) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_HCG] & 3) <<0)));
  }
  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_DIG_GAIN_HCG_H,
                           (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_HCG] >> 8));
  }
  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_DIG_GAIN_HCG_L,
                           (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_HCG] & 0xff));
  }
  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_DIG_GAIN_LCG_H,
                           (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_LCG] >> 8));
  }
  if(CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_DIG_GAIN_LCG_L,
                           (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_LCG] & 0xff));
  }

  if(aVSUpdate != 1.0)
  {
    //float lDigTemp;

    if(aVSUpdate<0.5)
    {
      aVSUpdate=0.5;
    }
    if(aVSUpdate > 2.0)
    {
      aVSUpdate=2.0;
    }

    lRet = CAM_WriteSingle(lClient,
                           (uint16_t)OV9716_GRP_CG_GAIN,
                           (uint8_t)(((arExp.mVSConversionGainOn & 1) << 7) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_VS] & 3) <<4) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_LCG] & 3) <<2) |
                           ((arExp.mAnalogGain[(uint8_t)OV9716_ExpType_HCG] & 3) <<0)));
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(lClient,
                             (uint16_t)OV9716_GRP_DIG_GAIN_VS_H,
                             (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_VS] >> 8));
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      lRet = CAM_WriteSingle(lClient,
                             (uint16_t)OV9716_GRP_DIG_GAIN_VS_L,
                             (uint8_t)(arExp.mDigitalGain[(uint8_t)OV9716_ExpType_VS] & 0xff));
    }

    if(CAM_LIB_SUCCESS == lRet)
    {
      /* update fractional part of exposure time */
      lRet = CAM_WriteSingle(lClient,
                             (uint16_t)OV9716_GRP_CEXP_VS_L,
                             (uint8_t)(arExp.mVSExposureTime & 0x1f));
    }
  }
 return lRet;
}

//******************************************************************************
/*EOF*/
