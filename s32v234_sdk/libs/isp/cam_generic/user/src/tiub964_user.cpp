/*****************************************************************************
 *
 * Freescale Confidential Proprietary
 *
 * Copyright (c) 2016 Freescale Semiconductor;
 * Copyright (c) 2016-17 NXP;
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
  * \file     tiub964_user.cpp
  * \brief    User-space library calls to access TI HW setup drivers.
  * \author   Thomas Brown
  * \version  0.1
  * \date     06.03.2018
  * \note
 ****************************************************************************/

//****************************************************************************
// include file
//****************************************************************************

#include "s32vs234.h"
#include <stdint.h>
#include <cstring>

#ifndef __STANDALONE__
  #include <fcntl.h>      // open
  #include <unistd.h>     // exit
  #include <sys/ioctl.h>  // ioctl
  #include <sys/mman.h>   // mmap
#else
extern "C"
{
  #include "i2c.h"
}

#endif // else from #ifndef __STANDALONE__

//#define VDB_LOG_NOTE_OFF

#include "isp_hw.h"
#include "isp_csi.h"
#include "isp_cam_generic.h"
#include "cam_lldcmd.h"
#include "vdb_log.h"

#include "ar0140_config.h"
#include "ub964_config.h"
#include "ov10642_config.h"
#include "isp_cam_ub964_api.h"

//****************************************************************************
// consts
//****************************************************************************

#define TI_DES_COMPATIBLE_STRING       "s32v-tiub964"
#define TI_SES_COMPATIBLE_STRING       "s32v-tiub933"
#define OV10640_COMPATIBLE_STRING      "s32v-ov10640"
#define AR0140_COMPATIBLE_STRING       "s32v-ar0140"

//****************************************************************************
// macros
//****************************************************************************

#ifdef __STANDALONE__
extern "C"
{
  uint64_t get_uptime_microS(void);
}
  #define msleep(_val) \
    {\
      uint64_t lStart = (uint64_t)get_uptime_microS();\
      while((get_uptime_microS() - lStart) < _val*1000U);\
    }
#endif // else from #ifdef __STANDALONE__

//****************************************************************************
// types
//****************************************************************************

typedef struct UB964QuadCam
{
  TIUB964_Cfg_t    mCfg;
  CamI2cClient_t   mDeserializer;
  CamI2cClient_t   mpSerializers[4];
  CamI2cClient_t   mpCameras[4];
} UB964QuadCam_t;

//*****************************************************************************
// local data
//*****************************************************************************

static UB964QuadCam_t spClients[2];
static bool spInitialized[2] = {false};

//*****************************************************************************
//
void TIUB964_ChanSelect (const enum CSI_IDX aCsiIdx, uint8_t aCsiFwd)
{
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x20, (uint8_t)aCsiFwd );
}

//*****************************************************************************
//
CAM_LIB_RESULT TIUB964_CsiEnable( const TIUB964_Cfg_t &acUB964_Cfg )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint8_t i;
  uint8_t lCsiFwd = 0xF0;
  CSI_IDX        lCsiIdx = acUB964_Cfg.mCsiIdx;

  // Build CSI Forwarding Mask
  // Clear bits in the MSB enable forwarding for corresponding Rx Port
  for( i=0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    lCsiFwd = lCsiFwd & ~((acUB964_Cfg.mRxConfig[i].mRxPortSel & 0x0F) << 4 );
  }

  lRet = CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x33, (uint8_t)( acUB964_Cfg.mCsi2NumLanes | 0x01 ));
  lRet = CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x20, (uint8_t)lCsiFwd );

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_CsiDisable( const TIUB964_Cfg_t &acUB964_Cfg )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  CSI_IDX        lCsiIdx = acUB964_Cfg.mCsiIdx;

  lRet = CAM_WriteSingle(spClients[lCsiIdx].mDeserializer,(uint8_t)0x33, (uint8_t)acUB964_Cfg.mCsi2NumLanes );

  return lRet;
}

//****************************************************************************

CAM_LIB_RESULT TIUB964_AR0140_Init (const enum CSI_IDX aCsiIdx, uint8_t aNumChan )
{
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;
  // uint16_t  l16bReg = 0x0000;
  // uint8_t lVal  = 0x00;
  uint32_t i, j;

  VDB_LOG_NOTE("AR0140 Register Configuration\n");

  for(j = 0; j < aNumChan; j++)
  {
    //
    // AR0140 Reset
    //
    for( i=0; i < (sizeof(ar0140_reset8b)/sizeof(ar0140_reset8b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_reset8b[i].regadd,
                      (uint8_t)ar0140_reset8b[i].regval);

      if(ar0140_reset8b[i].postamble)
      {
        msleep((uint32_t)Ov10642_1080p60_L[i].postamble);
      }
    }

    //
    // AR0140 LDR Sequencer
    //
    for( i=0; i < (sizeof(ar0140_sequencer8b)/sizeof(ar0140_sequencer8b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_sequencer8b[i].regadd,
                      (uint8_t)ar0140_sequencer8b[i].regval);

      if(ar0140_sequencer8b[i].postamble)
      {
        msleep((uint32_t)ar0140_sequencer8b[i].postamble);
      }
    }

    //
    // AR0140 Clocks 37MHz
    //
    for( i=0; i < (sizeof(ar0140_clocks_378b)/sizeof(ar0140_clocks_378b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_clocks_378b[i].regadd,
                      (uint8_t)ar0140_clocks_378b[i].regval);

      if(ar0140_clocks_378b[i].postamble)
      {
        msleep((uint32_t)ar0140_clocks_378b[i].postamble);
      }
    }

    //
    // AR0140 Resolution active pixel array
    //
    for( i=0; i < (sizeof(ar0140_res_720p308b)/sizeof(ar0140_res_720p308b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_res_720p308b[i].regadd,
                      (uint8_t)ar0140_res_720p308b[i].regval);

      if(ar0140_res_720p308b[i].postamble)
      {
        msleep((uint32_t)ar0140_res_720p308b[i].postamble);
      }
    }

    //
    // AR0140 Rev3 Optimized Settings
    //
    for( i=0; i < (sizeof(ar0140_rev3_optimize8b)/sizeof(ar0140_rev3_optimize8b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_rev3_optimize8b[i].regadd,
                      (uint8_t)ar0140_rev3_optimize8b[i].regval);

      if(ar0140_rev3_optimize8b[i].postamble)
      {
        msleep((uint32_t)ar0140_rev3_optimize8b[i].postamble);
      }
    }

    //
    // AR0140 Linear Mode
    //
    for( i=0; i < (sizeof(ar0140_linear8b)/sizeof(ar0140_linear8b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_linear8b[i].regadd,
                      (uint8_t)ar0140_linear8b[i].regval);

      if(ar0140_linear8b[i].postamble)
      {
        msleep((uint32_t)ar0140_linear8b[i].postamble);
      }
    }

    //
    // AR0140 Start Video Output
    //
    for( i=0; i < (sizeof(ar0140_output8b)/sizeof(ar0140_output8b[0])); i++)
    {
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[j],
                      (uint16_t)ar0140_output8b[i].regadd,
                      (uint8_t)ar0140_output8b[i].regval);

      if(ar0140_output8b[i].postamble)
      {
        msleep((uint32_t)ar0140_output8b[i].postamble);
      }
    }
  }

    VDB_LOG_NOTE("....Completed\n");

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_OV10640_Start( const TIUB964_Cfg_t &acUB964_Cfg )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint32_t i;
  uint8_t lCsiFwd = 0xF0;
  CSI_IDX        lCsiIdx = acUB964_Cfg.mCsiIdx;

  VDB_LOG_NOTE("Starting OV10640 Streams\n");

  // Build CSI Forwarding Mask
  // Clear bits in the MSB enable forwarding for corresponding Rx Port
  for( i=0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    lCsiFwd = lCsiFwd & ~((acUB964_Cfg.mRxConfig[i].mRxPortSel & 0x0F) << 4 );
  }

  lRet =  CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x33, (uint8_t)( acUB964_Cfg.mCsi2NumLanes | 0x01 ));
  lRet =  CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x20, lCsiFwd );

  // Start Stream
  for( i=0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    lRet = CAM_WriteSingle(spClients[lCsiIdx].mpCameras[i], (uint16_t)0x3012, (uint8_t)0x01);

    if( lRet == CAM_LIB_SUCCESS )
    {
      msleep(50);
    }
    else
    {
      return lRet;
    }
  }

  VDB_LOG_NOTE("OV10640 Started\n");

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_OV10640_Stop( const TIUB964_Cfg_t &acUB964_Cfg )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint16_t i;
  CSI_IDX        lCsiIdx = acUB964_Cfg.mCsiIdx;

  VDB_LOG_NOTE("Stopping OV10640 Streams\n");

  //  Stop all sensors streaming
  for( i = 0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    lRet = CAM_WriteSingle(spClients[lCsiIdx].mpCameras[i], (uint16_t)0x3012, (uint8_t)0x00);
    if( lRet == CAM_LIB_SUCCESS )
    {
      msleep(50);
    }
  }

  // Disable Deserialiser CSI2 Transmitter
  lRet = CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x33, (uint8_t)acUB964_Cfg.mCsi2NumLanes );

  if(lRet != CAM_LIB_SUCCESS )
  {
    return lRet;
  }

  if( lRet == CAM_LIB_SUCCESS )
  {
    VDB_LOG_NOTE("OV10640 Stopped\n");
  }

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_Init( const TIUB964_Cfg_t &acUB964_Cfg )
{
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;
  uint16_t l16bReg = 0x0000;
  uint8_t lVal = 0x00;
  uint32_t i, j;
  CSI_IDX        lCsiIdx = acUB964_Cfg.mCsiIdx;

  lRet = CAM_Open();
  if(lRet != CAM_LIB_SUCCESS)
  {
    return lRet;
  }
#ifndef __STANDALONE__
  if(CAM_LIB_FAILURE ==  CAM_PowerControl(lCsiIdx, SENSOR_TYPE_TIUB964, 
                                          CAM_POWER_STATE_UP))
  {
    VDB_LOG_ERROR("Power up viu #%d fail.\n", lCsiIdx);
    lRet = CAM_LIB_FAILURE;
  }
#endif // ifndef __STANDALONE__

  CamI2cCfg_t    lCamCfg;

  if(!spInitialized[lCsiIdx])
  {
    // *** prepare default I2C clients (deser/ser/cam) ***
    // deserializer
    lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) lCsiIdx;
    lCamCfg.mI2cAddr      = acUB964_Cfg.mUB964_I2C_Addr;
    lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

    memcpy(lCamCfg.mpCompatible,
           TI_DES_COMPATIBLE_STRING,
           strlen(TI_DES_COMPATIBLE_STRING));

    spClients[lCsiIdx].mDeserializer = CAM_ClientAdd(lCamCfg);

    if(spClients[lCsiIdx].mDeserializer == 0)
    {
      VDB_LOG_ERROR("Failed to add deserializer client.\n");
      lRet = CAM_LIB_FAILURE;
    }
  }
  else
  {
    VDB_LOG_ERROR("TIUB964 QuadCam setup already active on CSI idx %u.\n",
                 (uint32_t)lCsiIdx);
    lRet = CAM_LIB_FAILURE;
  }

  //
  // Check I2C Comms with Deserialiser
  //
  VDB_LOG_NOTE("TI UB964 I2C Test\n");

  lRet = CAM_ReadSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x00, lVal);

  if(lVal != (uint8_t)( acUB964_Cfg.mUB964_I2C_Addr << 1 ) )
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }

  VDB_LOG_NOTE("TI Deserializer Configuration.\n");

  //
  // 964 Common Register Configuration
  //
  for(i = 0; i < (sizeof(ub964_initial) / sizeof(ub964_initial[0])); i++)
  {
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)ub964_initial[i].regaddL, (uint8_t)ub964_initial[i].regval);

    if(ub964_initial[i].postamble)
    {
      msleep((uint32_t)ub964_initial[i].postamble);
    }
  }

  //
  // I2C1 not mapped to BC by default
  //
  if ( acUB964_Cfg.mI2cPort )
  {
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x0C, (uint8_t)0x77);
  }

  //
  // Enable Writes to CSI Tx - Registers
  //
  CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x32, (uint8_t)acUB964_Cfg.mCsi2TxPort);
  CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x33, (uint8_t)acUB964_Cfg.mCsi2NumLanes );

  //
  //  Rx Port Specific registers
  //
  for( i = 0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    // 1. BC Config & Rx Port Config
    // 2. Serialiser & Sensor Id & Alias
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x4C, (uint8_t)acUB964_Cfg.mRxConfig[i].mRxPortSel);
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x58, (uint8_t)0x58);
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x6D, (uint8_t)0x7E);
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x5C, (uint8_t)acUB964_Cfg.mRxConfig[i].mSerAlias );
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x5D, (uint8_t)acUB964_Cfg.mRxConfig[i].mSensorId );
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x65, (uint8_t)(acUB964_Cfg.mRxConfig[i].mSensorAlias << 1 ) );

    // Remote Reset/PDN GPIO
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x6E, (uint8_t)0x88);
    msleep((uint32_t)50);
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x6E, (uint8_t)acUB964_Cfg.mRxConfig[i].mBC_Gpio);
    msleep((uint32_t)50);

    // Finish 964 register configuration
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x7C, (uint8_t)acUB964_Cfg.mRxConfig[i].mFV_Polarity);
    CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x71, (uint8_t)acUB964_Cfg.mRxConfig[i].mVC_Map);

    // Add Cam Client for Serialisers and Sensors using selected Aliases
    lCamCfg.mI2cAddr = ( acUB964_Cfg.mRxConfig[i].mSerAlias >> 1);
    lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;
    memcpy(lCamCfg.mpCompatible, TI_SES_COMPATIBLE_STRING, strlen(TI_SES_COMPATIBLE_STRING));
    spClients[lCsiIdx].mpSerializers[i] = CAM_ClientAdd(lCamCfg);

    if(spClients[lCsiIdx].mpSerializers[i] != 0)
    {
      lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
      lCamCfg.mI2cAddr = acUB964_Cfg.mRxConfig[i].mSensorAlias;

      switch(acUB964_Cfg.mSensorType)
      {
        case (CSI_UB964_AR0140):
        {
            memcpy(lCamCfg.mpCompatible,
                 AR0140_COMPATIBLE_STRING,
                 strlen(AR0140_COMPATIBLE_STRING));
        } // case CSI_UB964_AR0140
         break;

        case (CSI_UB964_OV10640):

        case (CSI_UB964_OV10640_MULTI):
        {
            memcpy(lCamCfg.mpCompatible,
                 OV10640_COMPATIBLE_STRING,
                 strlen(OV10640_COMPATIBLE_STRING));
        } // case CSI_UB964_OV10640_MULTI
         break;

        default:
         break; // default
      } // switch(acUB964_Cfg.mSensorType)

      spClients[lCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

      if(spClients[lCsiIdx].mpCameras[i] == 0)
      {
        VDB_LOG_ERROR("Failed to add default %s camera client.\n",lCamCfg.mpCompatible );
        lRet = CAM_LIB_FAILURE;
      } // if failed to add camera default client
    }
    else
    {
      VDB_LOG_ERROR("Failed to add default serializer client.\n");
      lRet = CAM_LIB_FAILURE;
    }
  }

  if(lRet == CAM_LIB_SUCCESS)
  {
    spInitialized[lCsiIdx] = true;

    // reinit only if camera type has changed
    if( spClients[lCsiIdx].mCfg.mSensorType != acUB964_Cfg.mSensorType)
    {
      spClients[lCsiIdx].mCfg = acUB964_Cfg;
    } // if camera type has changed
  } // if all ok

  //
  // Change Default I2C Timings on the 933
  // Increasing SCL up to ~400kHz
  //
  for( i=0; i < acUB964_Cfg.mNumChannels; i++ )
  {
    CAM_WriteSingle( spClients[lCsiIdx].mpSerializers[i], (uint8_t)0x11, (uint8_t)0x32 );
    CAM_WriteSingle( spClients[lCsiIdx].mpSerializers[i], (uint8_t)0x12, (uint8_t)0x32 );
  }

  //
  //  IMAGE SENSOR CONFIGURATION
  //
  VDB_LOG_NOTE("Configure Sensors\n");

  switch(acUB964_Cfg.mSensorType)
  {
    case CSI_UB964_AR0140:
    {
      lRet = TIUB964_AR0140_Init (lCsiIdx, acUB964_Cfg.mNumChannels );
    } // case CSI_UB964_AR0140
     break;

    case CSI_UB964_OV10640:

    case CSI_UB964_OV10640_MULTI:
    {
      for( j = 0; j < acUB964_Cfg.mNumChannels; j++ )
      {
        for( i = 0; i < (sizeof(Ov10642_1080p60_L) / sizeof(Ov10642_1080p60_L[0])); i++ )
        {
          l16bReg = ( Ov10642_1080p60_L[i].regaddH << 8 ) + Ov10642_1080p60_L[i].regaddL;
          CAM_WriteSingle( spClients[lCsiIdx].mpCameras[j], (uint16_t)l16bReg, (uint8_t)Ov10642_1080p60_L[i].regval);

          if(Ov10642_1080p60_L[i].postamble)
          {
            msleep((uint32_t)Ov10642_1080p60_L[i].postamble);
          }
        }
      }
    } // case CSI_UB964_OV10640_MULTI
     break;

    default:
    {
      VDB_LOG_NOTE("Unsupported\n");
      lRet = CAM_LIB_FAILURE;
    } // default
     break;
  } // switch(acUB964_Cfg.mSensorType)

  //
  // Digital Reset of Deserialiser Restarts AEQ
  //
  VDB_LOG_NOTE("Resetting AEQ\n");
  CAM_WriteSingle(spClients[lCsiIdx].mDeserializer, (uint8_t)0x01, (uint8_t)0x01);
  msleep((uint32_t)50);

  VDB_LOG_NOTE("....Completed\n");

  return lRet;
}

//*****************************************************************************
CAM_LIB_RESULT TIUB964_Close(const enum CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  // if initialized
  if(spInitialized[aCsiIdx])
  {
     if(CAM_ClientRem(spClients[aCsiIdx].mDeserializer) != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to remove the deserializer client.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ClientRem() failed
    else
    {
      spClients[aCsiIdx].mDeserializer = 0;
    } // else from if CLientRem() failed  
     
     for(uint32_t i = (uint32_t)0; i < (uint32_t)4; i++)
    {
      if(spClients[aCsiIdx].mpSerializers[i] != 0)
      {
        if(CAM_ClientRem(
            spClients[aCsiIdx].mpSerializers[i]) != CAM_LIB_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to remove the seserializer client #%u.\n", i);
          lRet = CAM_LIB_FAILURE;
        } // if ClientRem() failed
        else
        {
          spClients[aCsiIdx].mpSerializers[i] = 0;
        } // else from if CLientRem() failed
      } // if seserializer used

      if(spClients[aCsiIdx].mpCameras[i] != 0)
      {
        if(CAM_ClientRem(spClients[aCsiIdx].mpCameras[i]) != CAM_LIB_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to remove the camera client #%u.\n", i);
          lRet = CAM_LIB_FAILURE;
        } // if ClientRem() failed
        else
        {
          spClients[aCsiIdx].mpCameras[i] = 0;
        } // else from if CLientRem() failed
      } // if seserializer used
    } // for all possible quadcam clients

    spInitialized[aCsiIdx] = false;     
  }

  return lRet;  
} 
//*****************************************************************************

#ifdef __STANDALONE__
CAM_LIB_RESULT TIUB964_TestPattern_Setup( void )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint8_t lpReadAddr[1] = {0};
  uint8_t lpReadVal[1] = {0};
  uint8_t lpWriteData[2] = {0};

  // Setup I2C bus
  I2C_Init_Config(I2C0, I2C_UB964_EVM_8b );

  // Test Communication with Deserialiser
  lpReadAddr[0] = 0x00;
  I2CX_DevRx(I2C0, I2C_UB964_EVM_7b, lpReadAddr, 1, lpReadVal, 1);
  I2CX_DevRx(I2C0, I2C_UB964_EVM_7b, lpReadAddr, 1, lpReadVal, 1);

  // Read out DEV ID from Deserilaiser - same as 8b I2C Address
  if(lpReadVal[0] != I2C_UB964_EVM_8b )
  {
      lRet = CAM_LIB_FAILURE;
  }
  else
  {
    // issue digital reset
    lpWriteData[0] = 0x01;
    lpWriteData[1] = 0x01;
    I2CX_DevTx(I2C0, I2C_UB964_EVM_7b, lpWriteData, 2);
  }

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_TestPattern_Start( void )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint32_t i;
  uint8_t lpWriteData[2] = {0};
  //
  // 964 Colour bar script
  //
  for(i = 0; i < (sizeof(ub960_CSITest)/sizeof(ub960_CSITest[0])); i++)
  {
    lpWriteData[0] = ub960_CSITest[i].regaddL;
    lpWriteData[1] = ub960_CSITest[i].regval;
    I2CX_DevTx(I2C0, I2C_UB964_EVM_7b, lpWriteData, 2);
  }

  return lRet;
}

//*****************************************************************************

CAM_LIB_RESULT TIUB964_TestPattern_Stop( void )
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint8_t lpWriteData[2] = {0};

  // Disable 964 CSI2
  lpWriteData[0] = 0x33;
  lpWriteData[1] = 0x20;
  I2CX_DevTx(I2C0, I2C_UB964_EVM_7b, lpWriteData, 2);

  return lRet;
}
#endif // ^^ STANDALONE IMPLEMENTATION ONLY

//*****************************************************************************

/*EOF*/
