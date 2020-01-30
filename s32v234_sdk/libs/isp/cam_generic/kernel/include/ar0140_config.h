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

/******************************************************************************
  * \file     ar0140_config.h
  * \brief    definition of AR0140 camera registers configuration
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
** T.Brown
**
** Register Settings Adopted from AR0140AT-REV3.init file from OnSemi
**
** Tested 720p30 Linear Mode with 37.5 MHz PCLK with MARS 913A Module
**
*/

typedef struct I2C_Script_16b_Data
{
  uint16_t regadd; // 16 bit reg address
  uint16_t regval;  // 16 bit reg value
  uint16_t postamble; // Milisecond delay
} I2C_Script_16b_Data_t;

typedef struct I2C_Script_8b_Data
{
  uint16_t regadd; // 16 bit reg address
  uint8_t regval;  // 8 bit reg value
  uint16_t postamble; // Milisecond delay
} I2C_Script_8b_Data_t;

/*
** AR0140 - Imager Reset
*/
I2C_Script_16b_Data ar0140_reset[] = {
  {0x301A, 0x0001,  10}, // RESET_REGISTER
  {0x301A, 0x10D8,  10}  // RESET_REGISTER
};

I2C_Script_8b_Data ar0140_reset8b[] = {
  {0x301A, 0x00,  10}, // RESET_REGISTER
  {0x301B, 0x01,  10}, // RESET_REGISTER
  {0x301A, 0x10,  10},  // RESET_REGISTER
  {0x301B, 0xD8,  10}  // RESET_REGISTER
};

/*
** AR0140 - Load SDR Sequencer
*/
I2C_Script_16b_Data ar0140_sequencer[] = {

  {0x3054, 0x0800,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x304C, 0x0200,   0}, // OTPM_RECORD record_type=0x2 auto_length
  {0x304A, 0x0010,   6}, // OTPM_CONTROL auto_rd_start

  {0x3054, 0x0800,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x304C, 0x2000,   0}, // OTPM_RECORD record_type=0x20 auto_length
  {0x304A, 0x0010,   6}  // OTPM_CONTROL auto_rd_start
};

I2C_Script_8b_Data ar0140_sequencer8b[] = {

  {0x3054, 0x08,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x3055, 0x00,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x304C, 0x02,   0}, // OTPM_RECORD record_type=0x2 auto_length
  {0x304D, 0x00,   0}, // OTPM_RECORD record_type=0x2 auto_length
  {0x304A, 0x00,   6}, // OTPM_CONTROL auto_rd_start
  {0x304B, 0x10,   6}, // OTPM_CONTROL auto_rd_start

  {0x3054, 0x08,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x3055, 0x00,   0}, // OTPM_EXPR trigger_auto_ram_load
  {0x304C, 0x20,   0}, // OTPM_RECORD record_type=0x20 auto_length
  {0x304D, 0x00,   0}, // OTPM_RECORD record_type=0x20 auto_length
  {0x304A, 0x00,   6},  // OTPM_CONTROL auto_rd_start
  {0x304B, 0x10,   6}  // OTPM_CONTROL auto_rd_start
};

/*
** AR0140 - PLL Config - 42MHz PCLK
*/
I2C_Script_16b_Data ar0140_clocks_42[] = {

  {0x302A, 0x000A,   0}, // VT_PIX_CLK_DIV
  {0x302C, 0x0001,   0}, // VT_SYS_CLK_DIV
  {0x302E, 0x0004,   0}, // PRE_PLL_CLK_DIV
  {0x3030, 0x003E,   0}, // PLL_MULTIPLIER
  {0x3036, 0x000A,   0}, // OP_PIX_CLK_DIV
  {0x3038, 0x0001,   0}  // OP_SYS_CLK_DIV
};

/*
** AR0140 - PLL Config - 37.5MHz PCLK
*/
I2C_Script_16b_Data ar0140_clocks_37[] = {

  {0x302A, 0x0006,   0}, // VT_PIX_CLK_DIV
  {0x302C, 0x0002,   0}, // VT_SYS_CLK_DIV
  {0x302E, 0x0004,   0}, // PRE_PLL_CLK_DIV
  {0x3030, 0x0042,   0}, // PLL_MULTIPLIER
  {0x3036, 0x000C,   0}, // OP_PIX_CLK_DIV
  {0x3038, 0x0002,   0}  // OP_SYS_CLK_DIV
};

I2C_Script_8b_Data ar0140_clocks_378b[] = {

  {0x302A, 0x00,   0}, // VT_PIX_CLK_DIV
  {0x302B, 0x06,   0}, // VT_PIX_CLK_DIV
  {0x302C, 0x00,   0}, // VT_SYS_CLK_DIV
  {0x302D, 0x02,   0}, // VT_SYS_CLK_DIV
  {0x302E, 0x00,   0}, // PRE_PLL_CLK_DIV
  {0x302F, 0x04,   0}, // PRE_PLL_CLK_DIV
  {0x3030, 0x00,   0}, // PLL_MULTIPLIER
  {0x3031, 0x42,   0}, // PLL_MULTIPLIER
  {0x3036, 0x00,   0}, // OP_PIX_CLK_DIV
  {0x3037, 0x0C,   0}, // OP_PIX_CLK_DIV
  {0x3038, 0x00,   0},  // OP_SYS_CLK_DIV
  {0x3039, 0x02,   0}  // OP_SYS_CLK_DIV
};


/*
** AR0140 - Set Active Resolution Window 1280*800
*/
I2C_Script_16b_Data ar0140_res_1280_800[] = {

  {0x3002, 0x001C,   0}, // Y_ADDR_START
  {0x3004, 0x0016,   0}, // X_ADDR_START
  {0x3006, 0x033B,   0}, // Y_ADDR_END
  {0x3008, 0x0515,   0}, // X_ADDR_END
  {0x300A, 0x0336,   0}, // FRAME_LENGTH_LINES
  {0x300C, 0x0672,   0}  // LINE_LENGTH_PCK
};

/*
** AR0140 - Set Active Resolution Window 1288*731
*/
I2C_Script_16b_Data ar0140_res_720p30[] = {

  {0x3004, 0x0012,   0}, // Y_ADDR_START
  {0x3002, 0x0040,   0}, // X_ADDR_START
  {0x3008, 0x0519,   0}, // Y_ADDR_END
  {0x3006, 0x0317,   0}, // X_ADDR_END
  {0x300A, 0x02EE,   0}, // FRAME_LENGTH_LINES
  //{0x300C, 0x0CE4,   0}, // LINE_LENGTH_PCK - 15fps!!
  {0x300C, 0x0672,   0}, // LINE_LENGTH_PCK
  {0x3012, 0x002D,   0}, // LINE_LENGTH_PCK
  {0x30A2, 0x0001,   0}, // LINE_LENGTH_PCK
  {0x30A6, 0x0001,   0}, // LINE_LENGTH_PCK
  {0x3040, 0x0000,   0}  // LINE_LENGTH_PCK
};

I2C_Script_8b_Data ar0140_res_720p308b[] = {

  {0x3004, 0x00,   0}, // Y_ADDR_START
  {0x3005, 0x12,   0}, // Y_ADDR_START
  {0x3002, 0x00,   0}, // X_ADDR_START
  {0x3003, 0x40,   0}, // X_ADDR_START
  {0x3008, 0x05,   0}, // Y_ADDR_END
  {0x3009, 0x19,   0}, // Y_ADDR_END
  {0x3006, 0x03,   0}, // X_ADDR_END
  {0x3007, 0x17,   0}, // X_ADDR_END
  {0x300A, 0x02,   0}, // FRAME_LENGTH_LINES
  {0x300B, 0xEE,   0}, // FRAME_LENGTH_LINES
  {0x300C, 0x06,   0}, // LINE_LENGTH_PCK
  {0x300D, 0x72,   0}, // LINE_LENGTH_PCK
  {0x3012, 0x00,   0}, // LINE_LENGTH_PCK
  {0x3013, 0x2D,   0}, // LINE_LENGTH_PCK
  {0x30A2, 0x00,   0}, // LINE_LENGTH_PCK
  {0x30A3, 0x01,   0}, // LINE_LENGTH_PCK
  {0x30A6, 0x00,   0}, // LINE_LENGTH_PCK
  {0x30A7, 0x01,   0}, // LINE_LENGTH_PCK
  {0x3040, 0x00,   0},  // LINE_LENGTH_PCK
  {0x3041, 0x00,   0}  // LINE_LENGTH_PCK
};

/*
** AR0140 - AR0140 Rev3 Optimized Settings
*/
I2C_Script_16b_Data ar0140_rev3_optimize[] = {

  {0x3044, 0x0400,   0}, //Manufacturer-specific
  {0x3052, 0xA134,   0}, //Manufacturer-specific
  {0x3092, 0x010F,   0}, //Manufacturer-specific
  {0x30FE, 0x0080,   0}, //Manufacturer-specific
  {0x3ECE, 0x40FF,   0}, //Manufacturer-specific
  {0x3ED0, 0xFF40,   0}, //Manufacturer-specific
  {0x3ED2, 0xA906,   0}, //Manufacturer-specific
  {0x3ED4, 0x001F,   0}, //Manufacturer-specific
  {0x3ED6, 0x638F,   0}, //Manufacturer-specific
  {0x3ED8, 0xCC99,   0}, //Manufacturer-specific
  {0x3EDA, 0x0888,   0}, //Manufacturer-specific
  {0x3EDE, 0x8878,   0}, //Manufacturer-specific
  {0x3EE0, 0x7755,   0}, //Manufacturer-specific
  {0x3EE2, 0x5563,   0}, //Manufacturer-specific
  {0x3EE4, 0xAAE0,   0}, //Manufacturer-specific
  {0x3EE6, 0x3400,   0}, //Manufacturer-specific
  {0x3EEA, 0xA4FF,   0}, //Manufacturer-specific
  {0x3EEC, 0x80F0,   0}, //Manufacturer-specific
  {0x3EEE, 0x0000,   0}, //Manufacturer-specific
  {0x31E0, 0x1701,   0}  //Manufacturer-specific
};

I2C_Script_8b_Data ar0140_rev3_optimize8b[] = {

  {0x3044, 0x04,   0}, //Manufacturer-specific
  {0x3045, 0x00,   0}, //Manufacturer-specific
  {0x3052, 0xA1,   0}, //Manufacturer-specific
  {0x3053, 0x34,   0}, //Manufacturer-specific
  {0x3092, 0x01,   0}, //Manufacturer-specific
  {0x3093, 0x0F,   0}, //Manufacturer-specific
  {0x30FE, 0x00,   0}, //Manufacturer-specific
  {0x30FF, 0x80,   0}, //Manufacturer-specific
  {0x3ECE, 0x40,   0}, //Manufacturer-specific
  {0x3ECF, 0xFF,   0}, //Manufacturer-specific
  {0x3ED0, 0xFF,   0}, //Manufacturer-specific
  {0x3ED1, 0x40,   0}, //Manufacturer-specific
  {0x3ED2, 0xA9,   0}, //Manufacturer-specific
  {0x3ED3, 0x06,   0}, //Manufacturer-specific
  {0x3ED4, 0x00,   0}, //Manufacturer-specific
  {0x3ED5, 0x1F,   0}, //Manufacturer-specific
  {0x3ED6, 0x63,   0}, //Manufacturer-specific
  {0x3ED7, 0x8F,   0}, //Manufacturer-specific
  {0x3ED8, 0xCC,   0}, //Manufacturer-specific
  {0x3ED9, 0x99,   0}, //Manufacturer-specific
  {0x3EDA, 0x08,   0}, //Manufacturer-specific
  {0x3EDB, 0x88,   0}, //Manufacturer-specific
  {0x3EDE, 0x88,   0}, //Manufacturer-specific
  {0x3EDF, 0x78,   0}, //Manufacturer-specific
  {0x3EE0, 0x77,   0}, //Manufacturer-specific
  {0x3EE1, 0x55,   0}, //Manufacturer-specific
  {0x3EE2, 0x55,   0}, //Manufacturer-specific
  {0x3EE3, 0x63,   0}, //Manufacturer-specific
  {0x3EE4, 0xAA,   0}, //Manufacturer-specific
  {0x3EE5, 0xE0,   0}, //Manufacturer-specific
  {0x3EE6, 0x34,   0}, //Manufacturer-specific
  {0x3EE7, 0x00,   0}, //Manufacturer-specific
  {0x3EEA, 0xA4,   0}, //Manufacturer-specific
  {0x3EEB, 0xFF,   0}, //Manufacturer-specific
  {0x3EEC, 0x80,   0}, //Manufacturer-specific
  {0x3EED, 0xF0,   0}, //Manufacturer-specific
  {0x3EEE, 0x00,   0}, //Manufacturer-specific
  {0x3EEF, 0x00,   0}, //Manufacturer-specific
  {0x31E0, 0x17,   0},  //Manufacturer-specific
  {0x31E1, 0x01,   0}  //Manufacturer-specific
};

/*
** AR0140 - HDR Setting - Linear Mode
*/
I2C_Script_16b_Data ar0140_linear[] = {

  {0x3082, 0x0009,   0}, // Linear mode
  {0x318C, 0x0000,   0}, // Motion Compensation Off
  {0x3200, 0x0000,   0}, // ADACD_CONTROL DISABLED
  {0x31D0, 0x0000,   0}  // COMPANDING DISABLED
};

I2C_Script_8b_Data ar0140_linear8b[] = {

  {0x3082, 0x00,   0}, // Linear mode
  {0x3083, 0x09,   0}, // Linear mode
  {0x318C, 0x00,   0}, // Motion Compensation Off
  {0x318D, 0x00,   0}, // Motion Compensation Off
  {0x3200, 0x00,   0}, // ADACD_CONTROL DISABLED
  {0x3201, 0x00,   0}, // ADACD_CONTROL DISABLED
  {0x31D0, 0x00,   0},  // COMPANDING DISABLED
  {0x31D1, 0x00,   0}  // COMPANDING DISABLED
};

/*
** AR0140 - Configure Sensor Output
*/
I2C_Script_16b_Data ar0140_output[] = {
  {0x30B0, 0x0000,   0}, //DIGITAL_TEST
  {0x30BA, 0x012C,   0}, //DIGITAL_CTRL
  {0x31AC, 0x0C0C,   0}, //DATA_FORMAT_BITS
  {0x31AE, 0x0301,   0}, //SERAIL_FORAMT
  {0x301A, 0x10DC,   100} //RESET_REGISTER
};

I2C_Script_8b_Data ar0140_output8b[] = {

  {0x30B0, 0x00,   0}, //DIGITAL_TEST
  {0x30B1, 0x00,   0}, //DIGITAL_TEST
  {0x30BA, 0x01,   0}, //DIGITAL_CTRL
  {0x30BB, 0x2C,   0}, //DIGITAL_CTRL
  {0x31AC, 0x0C,   0}, //DATA_FORMAT_BITS
  {0x31AD, 0x0C,   0}, //DATA_FORMAT_BITS
  {0x31AE, 0x03,   0}, //SERAIL_FORAMT
  {0x31AF, 0x01,   0}, //SERAIL_FORAMT
  {0x301A, 0x10,   100}, //RESET_REGISTER
  {0x301B, 0xDC,   100} //RESET_REGISTER
};

/*
** AR0140 - Embed Auto Exposure Stats
*/
I2C_Script_16b_Data ar0140_AE_trigger[] = {

  {0x3064, 0x1902, 200},
  {0x3064, 0x1982,   0}
};
