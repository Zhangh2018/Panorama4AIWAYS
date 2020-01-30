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
/*
 * \file     tiub964_type.h
 * \brief    types declarations for TI HW setup drivers
 * \author   Thomas Brown
 * \version  0.1
 * \date     06.03.2018
 * \note
 */
/*==============================================================================
*                                 INCLUDE FILES
==============================================================================*/

#include "../../../csi/kernel/include/csi_types.h"

/*==============================================================================
*                          CONST
==============================================================================*/
#define TIUB964_RESET_POLARITY 0  // active low
#define TIUB964_RESET_DELAY    10
#define TIUB964_PWDN_POLARITY  0  // active low
#define TIUB964_PWDN_DELAY     10

/*==============================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==============================================================================*/

//
// Rx Port Specific Config
//
typedef struct UB964RxPort_Cfg
{
  uint8_t   mRxPortSel;    //
  uint8_t   mSerAlias;     // Serialiser Alias    - 8b
  uint8_t   mSensorId;     // Sensor I2C ID       - 8b
  uint8_t   mSensorAlias;  // Sensor Alias        - 8b
  uint8_t   mVC_Map;       // VC for each Rx Port
  uint8_t   mFV_Polarity;  // 1 = FV Low for Frame Duration
  uint8_t   mBC_Gpio;      // 0 - no gpio
                           // 1 - bc_gpio[0] - set high
                           // 2 - bc_gpio[1] - set high
                           // 3 - bc_gpio[0] + gpio[1] - set high
} UB964RxPort_Cfg_t;

//
// Master tiub964 Configuration
//
typedef struct TIUB964_Cfg
{
  enum CSI_IDX      mCsiIdx;
  CSI_CameraType_t  mSensorType;
  uint8_t           mI2cPort;             // 964 I2C Bus Used - Default = 0
  uint8_t           mUB964_I2C_Addr;      // Deserialiser address - 7b
  uint8_t           mCsi2NumLanes;        // number of lanes from ub964
  uint8_t           mCsi2TxPort;          // 964 csi2 Tx port
  uint8_t           mNumChannels;         // 964 csi2 Tx port
  UB964RxPort_Cfg_t  mRxConfig[4];
} TIUB964_Cfg_t;

/*enum UB964_RX
{
  UB964_RIN_NONE = 0U,
  UB964_RIN0     = 1U,
  UB964_RIN1     = 2U,
  UB964_RIN2     = 4U,
  UB964_RIN3     = 8U,
};*/

//
// 964 has two I2C ports if I2C1 is used, it needs to be
// explicitly mapped to BC
//
enum I2C_PORT_964
{
  I2C_964_0 = 0U,
  I2C_964_1 = 1U,
}; // I2C_PORT_964 definition

//
// 0x4C - Rx Port Select Register
//
enum FPD3_PORT_SEL
{
  FPD3_PORT_0 = 0x01U,
  FPD3_PORT_1 = 0x12U,
  FPD3_PORT_2 = 0x24U,
  FPD3_PORT_3 = 0x38U,
}; // FPD3_PORT_SEL definition

//
// 0x32 - Enables writes to selected CSI2 Tx Module
//
enum CSI_PORT_SEL_964
{
  CSI_964_TX_0 = 0x01U,
  CSI_964_TX_1 = 0x12U,
}; // CSI_PORT_SEL_964 definition

//
// 964 Lane Configuration
//
enum CSI_LANES_964
{
  CSI_964_4_LANES = 0x00U,
  CSI_964_3_LANES = 0x10U,
  CSI_964_2_LANES = 0x20U,
  CSI_964_1_LANE  = 0x30U,
}; // CSI_LANES_964 definition

//
// MIPI CSI2 Data Types
//
enum CSI_DT_964
{
  CSI2_BLANKING   = 0x11U,
  CSI2_EMB_DATA   = 0x12U,
  CSI2_YUV422_8b  = 0x1EU,
  CSI2_YUV422_10b = 0x1FU,
  CSI2_RGB565     = 0x22U,
  CSI2_RGB888     = 0x24U,
  CSI2_RAW10      = 0x2BU,
  CSI2_RAW12      = 0x2CU,
}; // CSI_DT_964 definition

//
// Virtual Channel ID
//
enum CSI_VT_964
{
  CSI_964_VT_0 = 0x00U,
  CSI_964_VT_1 = 0x40U,
  CSI_964_VT_2 = 0x80U,
  CSI_964_VT_3 = 0xC0U,
}; // CSI_VT_964 definition

//
// 0x7C PORT_CONFIG2 FV Polarity
//
enum FV_POLARITY_964
{
  FV_ACTIVE_HIGH = 0U,
  FV_ACTIVE_LOW  = 1U,
}; // FV_POLARITY_964 definition


//
// 0x6E - BC GPIO Control for 0 & 1
//
// settings to deliver a const value of
// '1' to selected BC_GPIO
//
enum BC_GPIO_CTL
{
  BC_GPIO_NONE = 0x88U,
  BC_GPIO_0    = 0x89U,
  BC_GPIO_1    = 0x98U,
  BC_GPIO_0_1  = 0x99U,
}; // BC_GPIO_CTL definition

