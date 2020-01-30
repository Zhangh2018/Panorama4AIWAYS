/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
* All Rights Reserved
*
******************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
/*
 * File:		i2c.h
 * Purpose:		Internal Integrated Circuit bus routines
 *
 */
  
#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#include "typedefs.h"


/********************************************************************/

void I2C_Reset(vuint8_t i2c_module);

void I2C_Pin_Init(vuint32_t i2c_module);
int I2C_Init_Config(vuint8_t i2c_module, vuint8_t slave_address);

int I2C_start(vuint8_t module, vuint8_t slave_addr, vuint8_t operation);
int I2C_write(vuint8_t module, vuint8_t data);
int I2C_stop(vuint8_t module);
int I2C_read(vuint8_t module, vuint8_t sl_address, vuint8_t * data);

void I2C_Tx(vuint8_t module, vuint8_t adr_byte1, vuint8_t adr_byte2, vuint8_t data_byte);
void I2C8_TX(vuint8_t module, vuint8_t adr_byte, vuint8_t data_byte);

void I2CX_DevTx(vuint8_t module, vuint8_t dev, vuint8_t *addr_data, vuint8_t total_byte_cnt);
void I2CX_DevRx(vuint8_t module, vuint8_t dev, vuint8_t *addr, vuint8_t addr_len,vuint8_t *data, vuint8_t data_len);
/********************************************************************/
#define I2C0	0
#define I2C1	1
#define I2C2	2

#define WRITE	0
#define READ	1

#define OMV_CAM_ADDR 0x60//0x30
#define SONY_CAM_ADDR 0x34
#define HDMI_ADDR 0xE4

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */
