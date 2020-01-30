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
/***************************************************************************
  * \file     ub964_config.h
  * \brief    definition of deserlizer UB964 camera registers configuration
  *           for mipi
  * \author   Thomas Brown
  * \version  0.1
  * \date     06.03.2018
  * \note
 ****************************************************************************/

/*==============================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==============================================================================*/

/*
** Generic structure for I2C Scripts
*/
typedef struct I2CInitScript8b
{
  unsigned char regaddL;
  unsigned char regval;
  unsigned char postamble;
} I2CInitScript8b_t;

/*
** Common 964 Setup - Initial
*/
I2CInitScript8b ub964_initial[] = {
  {0x01, 0x02, 100},  // Digital Reset
  {0x1F, 0x05, 100},
  {0x0A, 0x20,   0}, // I2C Timing for Ack and Reads
  {0x0B, 0x20,   0}, // SCL_HIGH & SCL_LOW to match vsdk timing
  {0x10, 0x81,   0}, // RX0 Lock > LED0
  {0x11, 0x85,   0}, // RX1 Lock > LED1
  {0x12, 0x89,   0}, // RX2 Lock > LED2
  {0x13, 0x8D,   0} // RX3 Lock > LED3
};

/*
** TiUB964 Test Pattern Generation
*/
I2CInitScript8b ub960_CSITest[] = {
  {0x32, 0x01, 0}, // Select & Enable CSI2-0
  {0x33, 0x21, 0}, // Enable and set 2 lane

  {0xB0, 0x00, 0}, // indirect reg access

  {0xB1, 0x01, 0}, // PGEN_CTL
  {0xB2, 0x01, 0}, //

  // 8 bars, 3byte colour field
  {0xB1, 0x02, 0}, // PGEN_CFG
  {0xB2, 0x33, 0}, //

  // RGB888 CSI2 DI
  {0xB1, 0x03, 0}, // PGEN_CSI_DI
  {0xB2, 0x24, 0}, //

  // Active Line Length = 3840 Bytes
  {0xB1, 0x04, 0}, // PGEN_LINE_SIZE1
  {0xB2, 0x0F, 0}, //
  {0xB1, 0x05, 0}, // PGEN_LINE_SIZE0
  {0xB2, 0x00, 0}, //

  // Colour Bar Width = 480 Bytes
  {0xB1, 0x06, 0}, // PGEN_BAR_SIZE1
  {0xB2, 0x01, 0}, //
  {0xB1, 0x07, 0}, // PGEN_BAR_SIZE0
  {0xB2, 0xE0, 0}, //

  // Active Lines Per Frame = 0x2D0 = 720
  {0xB1, 0x08, 0}, // PGEN_ACT_LPF1
  {0xB2, 0x02, 0}, //
  {0xB1, 0x09, 0}, // PGEN_ACT_LPF0
  {0xB2, 0xD0, 0}, //

  // Lines Per Frame Total ( Active + Vertical Blanking ) = 0x41A = 1050
  {0xB1, 0x0A, 0}, // PGEN_TOT_LPF1
  {0xB2, 0x04, 0}, //
  {0xB1, 0x0B, 0}, // PGEN_TOT_LPF0
  {0xB2, 0x1A, 0}, //

  // Line Period Value in 10nS units = 0xC67 = 31.75uS
  {0xB1, 0x0C, 0}, // PGEN_LINE_PD1
  {0xB2, 0x0C, 0}, //
  {0xB1, 0x0D, 0}, // PGEN_LINE_PD0
  {0xB2, 0x67, 0}, //

  // VBP = 33
  {0xB1, 0x0E, 0}, // PGEN_VBP
  {0xB2, 0x21, 0}, //

  // VFP = 10
  {0xB1, 0x0F, 0}, // PGEN_VFP
  {0xB2, 0x0A, 0}  //
};
