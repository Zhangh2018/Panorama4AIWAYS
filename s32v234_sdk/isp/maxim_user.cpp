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
 * \file     maxim_user.cpp
 * \brief    User-space library calls to access maxim functionality.
 * \author   Tomas Babinec
 * \version  0.1
 * \date     16-December-2015
 * \note
 ****************************************************************************/
#include "s32vs234.h"
#include <stdint.h>
#include <cstring>

#ifndef __STANDALONE__
  #include <fcntl.h>      // open
  #include <unistd.h>     // exit
  #include <sys/mman.h>   // mmap
#else
extern "C"
{
  #include "i2c.h"
  #include "sony_func.h"
}

#endif // else from #ifndef __STANDALONE__

#define VDB_LOG_NOTE_OFF
#define MAXIM

#include "oal.h"

#include "isp_hw.h"
#include "isp_csi.h"
#include "isp_cam_generic.h"
#include "cam_lldcmd.h"
#include "isp_cam_maxim.h"
#include "isp_cam_sony.h"
#include "sony_config.h"
#include "ov10640_config.h"
#include "ov10635_config_max.h"
#include "ar0231_config.h"
#include "ov9716_config_lvds.h"
#include "vdb_log.h"

//****************************************************************************
// consts
//****************************************************************************

#define DESERIALIZER_COMPATIBLE_STRING "s32v-max9286"
#define DES9296B_COMPATIBLE_STRING     "s32v-max9296B"
#define SERIALIZER_COMPATIBLE_STRING   "s32v-max9271"
#define SER9295A_COMPATIBLE_STRING     "s32v-max9295A"
#define AR0231_COMPATIBLE_STRING       "s32v-ar0231"
#define SONY_COMPATIBLE_STRING         "s32v-sony-imx224"
#define OV10640_COMPATIBLE_STRING      "s32v-ov10640"
#define OV10635_COMPATIBLE_STRING      "s32v-ov10635"
#define OV9716_COMPATIBLE_STRING       "s32v-ov9716"
#define OV490_OV10640_COMPATIBLE_STRING      "s32v-ov490-ov10640"

#define OV10640_CHIP_ID_HIGH_BYTE 0x300A
#define OV10640_CHIP_ID_LOW_BYTE  0x300B
#define OV9716_GRP_CHGAIN_BASE    (0x3360)

#define INIT_DELAY 2 // delay before i2c config is attempted

//#define SERIALIZER_FREQUENCY   0xAE // i2c 100kHz
#define SERIALIZER_FREQUENCY   0xB6 // i2c 400kHz
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
#endif // #ifdef __STANDALONE__

//****************************************************************************
// types
//****************************************************************************

struct MaximQuadCam
{
  MAXIM_Cfg_t    mCfg;
  CamI2cClient_t mDeserializer;
  CamI2cClient_t mpSerializers[6]; // 0=original&broadcast,1-4=slots
  CamI2cClient_t mpCameras[6];     // 0=original&broadcast,1-4=slots
  CamI2cClient_t mCamDes[6];
}; // struct MaximQuadCam

//*****************************************************************************
// local functions
//*****************************************************************************
static CAM_LIB_RESULT Maxim9296BInitClients(const enum CSI_IDX acCsiIdx,
                                            const uint32_t     acChnlIdx);

CAM_LIB_RESULT MAXIM9286_96705_Channel_Check(uint8_t lVal);
CAM_LIB_RESULT MAXIM_CML_Level(uint32_t x);
CAM_LIB_RESULT MAXIM_PreemPhasis_Level(uint32_t x);
CAM_LIB_RESULT MAXIM_Spread_Spectrum(uint32_t x);
//****************************************************************************
// global variables
//****************************************************************************
#ifndef __STANDALONE__
  extern int32_t gCamFD;   ///< cam driver file descriptor
#endif //ifndef __STANDALONE__

//*****************************************************************************
// local data
//*****************************************************************************

static MaximQuadCam spClients[2];
static bool         spInitialized[2] = {false};

//*****************************************************************************
// functions definitions
//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_Open(const enum CSI_IDX acCsiIdx,
                                const uint32_t     acChnlIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  if(CAM_Open() == CAM_LIB_SUCCESS)
  {
    // if not initialized allready
    if(!spInitialized[acCsiIdx])
    {
      if(Maxim9296BInitClients(acCsiIdx, acChnlIdx) != CAM_LIB_SUCCESS)
      {
        lRet = CAM_LIB_FAILURE;
      } // if i2c clients init failed
    } // if not initialized yet
    else
    {
      VDB_LOG_ERROR("Maxim QuadCam setup already active on CSI idx %u.\n",
                    (uint32_t)acCsiIdx);
      lRet = CAM_LIB_FAILURE;
    } // else if not initialized yet

    // *** do I2C configuration
    if(lRet == CAM_LIB_SUCCESS)
    {
      if(MAXIM_9296B_AR0231_InitComplex(acCsiIdx, acChnlIdx) != CAM_LIB_SUCCESS)
      {
        lRet = CAM_LIB_FAILURE;
      } // if HW config failed
    } // if all ok so far
  } // if cam opened ok
  else
  {
    VDB_LOG_ERROR("Failed to open generic camera driver.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if cam opened ok

  return lRet;
}// MAXIM_9296B_Open()

//****************************************************************************

static CAM_LIB_RESULT Maxim9296BInitClients(const enum CSI_IDX acCsiIdx,
                                            const uint32_t     acChnlIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  CamI2cCfg_t    lCamCfg;

  // *** prepare I2C clients (deser/ser/cam) ***
  // deserializer
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) acCsiIdx;
  lCamCfg.mI2cAddr      = (uint8_t)I2C_MAX_9296B_DES_ADDR;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;

  memcpy(lCamCfg.mpCompatible,
          DES9296B_COMPATIBLE_STRING,
          strlen(DES9296B_COMPATIBLE_STRING));

  spClients[acCsiIdx].mDeserializer = CAM_ClientAdd(lCamCfg);

  if(spClients[acCsiIdx].mDeserializer != 0)
  {
    // serializer
    lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
    lCamCfg.mI2cAddr = I2C_MAX_9295A_ADDR(acChnlIdx);

    memcpy(lCamCfg.mpCompatible,
          SER9295A_COMPATIBLE_STRING,
          strlen(SER9295A_COMPATIBLE_STRING));

    spClients[acCsiIdx].mpSerializers[acChnlIdx] = CAM_ClientAdd(lCamCfg);

    if(spClients[acCsiIdx].mpSerializers[acChnlIdx] != 0)
    {
      // camera
      lCamCfg.mI2cAddr      = (uint8_t)I2C_MAX_AR0231_ADDR(acChnlIdx);
      memcpy(lCamCfg.mpCompatible,
            AR0231_COMPATIBLE_STRING,
            strlen(AR0231_COMPATIBLE_STRING));
      spClients[acCsiIdx].mpCameras[acChnlIdx] = CAM_ClientAdd(lCamCfg);
      if(spClients[acCsiIdx].mpCameras[acChnlIdx] == 0)
      {
        VDB_LOG_ERROR(
          "Failed to add %s camera client.\n",
          lCamCfg.mpCompatible
        );
        lRet = CAM_LIB_FAILURE;
      } // if failed to add camera client
      else
      {
        spInitialized[acCsiIdx] = true;
      }
    } // if default Serializer added successfully
    else
    {
      VDB_LOG_ERROR("Failed to add serializer client.\n");
      lRet = CAM_LIB_FAILURE;
    } // else from if default Serializer added successfully
  } // if Deserializer added successfully
  else
  {
    VDB_LOG_ERROR("Failed to add deserializer client.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if Deserializer added successfully

  return lRet;
} // Maxim9296BInitClients()

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_AR0231_InitComplex(const enum CSI_IDX acCsiIdx,
                                             const uint32_t     acChnlIdx)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint32_t    lCmdCnt = 0;

  // *** declare transfer structs ***
  I2cXfrSingle_t lpXfr[3];
  uint8_t       *lppData[3] = {(uint8_t*)&(lpXfr[MAXIM_DES_9296B].mData),
                               (uint8_t*)&(lpXfr[MAXIM_SER_9295A].mData),
                               (uint8_t*)&(lpXfr[MAXIM_CAM_AR0231].mData)};

  // *** init transfer structs ***
  lpXfr[MAXIM_DES_9296B].mDestDev  = spClients[acCsiIdx].mDeserializer;
  lpXfr[MAXIM_DES_9296B].mData     = 0;
  lpXfr[MAXIM_DES_9296B].mByteCnt  = 3;
  lpXfr[MAXIM_SER_9295A].mDestDev  = spClients[acCsiIdx].mpSerializers[0];
  lpXfr[MAXIM_SER_9295A].mByteCnt  = 3;
  lpXfr[MAXIM_SER_9295A].mData     = 0;
  lpXfr[MAXIM_CAM_AR0231].mDestDev = spClients[acCsiIdx].mpCameras[0];
  lpXfr[MAXIM_CAM_AR0231].mByteCnt = 4;
  lpXfr[MAXIM_CAM_AR0231].mData    = 0;

  // *** configure the HW setup ***
  // run initial Ser/Des config
  uint8_t  *lpTable = NULL;
  uint8_t  *lpCmd   = NULL;
  uint8_t    lDevId = 0;
  uint8_t   *lpData = NULL;

  lpTable = sppMax9295AMax9296B_RegConfig[0];
  lCmdCnt = sizeof(sppMax9295AMax9296B_RegConfig) /
  sizeof(sppMax9295AMax9296B_RegConfig[0]);

  for(uint32_t i = 0; i < lCmdCnt; i++)
  {
      lpCmd  = &lpTable[i*6];
      lDevId = lpCmd[0];
      lpData = lppData[lDevId];

      lpXfr[lDevId].mByteCnt = lpCmd[1]; // remember # data bytes
      lpCmd += 2; // shift to data bytes
      // copy i2c message
      for(uint32_t j = 0; j < lpXfr[lDevId].mByteCnt; j++)
      {
      lpData[j] = lpCmd[j];
    } // for all command data bytes

    lRes = CAM_WriteSingle(lpXfr[lDevId]);
    if(lRes != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("I2c command table 0 failed @ index %u.\n", i);
      lRet = CAM_LIB_FAILURE;
      return lRet;
    } // if i2c transaction failed
    msleep(lpCmd[lpXfr[lDevId].mByteCnt]);
  } // for all i2c commands

  // continue with Camera related config
  lCmdCnt = sizeof(sppAr0231_Table) /
            sizeof(sppAr0231_Table[0]);
  for(uint32_t i = 0; i < lCmdCnt; i++)
  {
    lpCmd  = sppAr0231_Table[i];
    lDevId = lpCmd[0];
    lpData = lppData[lDevId];

    lpXfr[lDevId].mByteCnt = lpCmd[1]; // remember # data bytes
    lpCmd += 2; // shift to data bytes
    // copy i2c message
    for(uint32_t j = 0; j < lpXfr[lDevId].mByteCnt; j++)
    {
      lpData[j] = lpCmd[j];
    } // for all command data bytes

    lRes = CAM_WriteSingle(lpXfr[lDevId]);
    if(lRes != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("I2c command table 1 failed @ index %u.\n", i);
      lRet = CAM_LIB_FAILURE;
      return lRet;
    } // if i2c transaction failed
    msleep(lpCmd[lpXfr[lDevId].mByteCnt]);
  } // for all i2c commands

  lRes = MAXIM_9296B_CsiDisable(acCsiIdx);
  if(lRes != CAM_LIB_SUCCESS)
  {
    VDB_LOG_ERROR("Failed to disable CSI output.\n");
    lRet = CAM_LIB_FAILURE;
  } // if failed to disable CSI output

  return lRet;
} // MAXIM_9296B_AR0231_InitComplex()

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_Close(const enum CSI_IDX acCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  // if initialized
  if(spInitialized[acCsiIdx])
  {
    if(CAM_ClientRem(spClients[acCsiIdx].mDeserializer) != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to remove the deserializer client.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ClientRem() failed
    else
    {
      spClients[acCsiIdx].mDeserializer = 0;
    } // else from if CLientRem() failed

    for(uint32_t i = (uint32_t)0; i < (uint32_t)5; i++)
    {
      if(spClients[acCsiIdx].mpSerializers[i] != 0)
      {
        if(CAM_ClientRem(
            spClients[acCsiIdx].mpSerializers[i]) != CAM_LIB_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to remove the seserializer client #%u.\n", i);
          lRet = CAM_LIB_FAILURE;
        } // if ClientRem() failed
        else
        {
          spClients[acCsiIdx].mpSerializers[i] = 0;
        } // else from if CLientRem() failed
      } // if seserializer used

      if(spClients[acCsiIdx].mpCameras[i] != 0)
      {
        if(CAM_ClientRem(spClients[acCsiIdx].mpCameras[i]) != CAM_LIB_SUCCESS)
        {
          VDB_LOG_ERROR("Failed to remove the camera client #%u.\n", i);
          lRet = CAM_LIB_FAILURE;
        } // if ClientRem() failed
        else
        {
          spClients[acCsiIdx].mpCameras[i] = 0;
        } // else from if CLientRem() failed
      } // if seserializer used
    } // for all possible quadcam clients

    spInitialized[acCsiIdx] = false;
  } // if Deserializer used

  return lRet;
} // MAXIM_9296B_Close()

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_CsiEnable(const enum CSI_IDX acCsiIdx)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  // ***  start deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mDeserializer;
  lXfr.mByteCnt = 3;

  lpData[0] = 0x03; lpData[1] = 0x13; lpData[2] = 0x00;

  // invoke read
  lRes = CAM_ReadSingle(lXfr);
  if (CAM_LIB_SUCCESS != lRes)
  {
    VDB_LOG_ERROR("Failed to read deserializer register.\n");
    lRet = CAM_LIB_FAILURE;
  } // if write failed

  lpData[2] |= 0x2;

  // invoke write
  lRes = CAM_WriteSingle(lXfr);
  if (CAM_LIB_SUCCESS != lRes)
  {
    VDB_LOG_ERROR("Failed to write deserializer register to start mipi stream.\n");
    lRet = CAM_LIB_FAILURE;
  } // if write failed

  return lRet;
} // MAXIM_9296B_CsiEnable(enum CSI_IDX acCsiIdx)

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_CsiDisable(const enum CSI_IDX acCsiIdx)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mDeserializer;
  lXfr.mByteCnt = 3;

  lpData[0] = 0x03; lpData[1] = 0x13; lpData[2] = 0x00;

  // invoke read
  lRes = CAM_ReadSingle(lXfr);
  if (CAM_LIB_SUCCESS != lRes)
  {
    VDB_LOG_ERROR("Failed to read deserializer register.\n");
    lRet = CAM_LIB_FAILURE;
  } // if write failed

  lpData[2] &= 0xfe;

  // invoke write
  lRes = CAM_WriteSingle(lXfr);
  if (CAM_LIB_SUCCESS != lRes)
  {
    VDB_LOG_ERROR("Failed to write deserializer register to stop mipi stream.\n");
    lRet = CAM_LIB_FAILURE;
  } // if write failed
  return lRet;
} // MAXIM_9296B_CsiDisable(enum CSI_IDX acCsiIdx)

//*****************************************************************************

CAM_LIB_RESULT MAXIM_AR0231_CAM_RegWrite(
  const enum CSI_IDX    acCsiIdx,
  uint8_t              *apData,
  const uint8_t         acBytesCnt,
  const enum MAXIM_CHNL acChnlIdx
)
{
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mpCameras[acChnlIdx];
  lXfr.mByteCnt = acBytesCnt;

  if(acBytesCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    for(uint32_t i = 0; i < acBytesCnt; i++)
    {
      lpData[i] = apData[i];
    } // for all data bytes
  }// if data fits to Xfr struct
  else
  {
    lXfr.mData = (uintptr_t)apData;
  } // else from if data fits to Xfr struct

  // invoke transfer
  return CAM_WriteSingle(lXfr);
} // MAXIM_AR0231_CAM_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_AR0231_CAM_RegRead(
  const enum CSI_IDX    acCsiIdx,
  uint8_t              *apData,
  const uint8_t         acBytesCnt,
  const enum MAXIM_CHNL acChnlIdx
)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mpCameras[acChnlIdx];
  lXfr.mByteCnt = acBytesCnt;

  if(acBytesCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    for(uint32_t i = 0; i < acBytesCnt; i++)
    {
      lpData[i] = apData[i];
    } // for all data bytes
  }// if data fits to Xfr struct
  else
  {
    lXfr.mData = (uintptr_t)apData;
  } // else from if data fits to Xfr struct

  lRes = CAM_ReadSingle(lXfr);

  if(acBytesCnt <= CAM_I2C_SINGLE_DATA_SZ)
  {
    for(uint32_t i = 0; i < acBytesCnt; i++)
    {
      apData[i] = lpData[i];
    } // for all data bytes
  }// if data fits to Xfr struct

  // invoke transfer
  return lRes;
} // MAXIM_AR0231_CAM_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9295A_SER_RegWrite(
  const enum CSI_IDX    acCsiIdx,
  const uint16_t        acRegAddr,
  const uint8_t         acVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mpSerializers[acChnlIdx];
  lXfr.mByteCnt = 3;

  lpData[0] = (acRegAddr >> 8);
  lpData[1] = (acRegAddr & 0xff);
  lpData[2] = acVal;

  // invoke transfer
  return CAM_WriteSingle(lXfr);
} // MAXIM_9295A_SER_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9295A_SER_RegRead(
  const enum CSI_IDX    acCsiIdx,
  const uint16_t        acRegAddr,
  uint8_t              &arVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mpSerializers[acChnlIdx];
  lXfr.mByteCnt = 3;

  lpData[0] = (acRegAddr >> 8);
  lpData[1] = (acRegAddr & 0xff);
  lpData[2] = 0;

  // invoke transfer
  lRes  = CAM_WriteSingle(lXfr);
  arVal = lpData[2];
  return lRes;
} // MAXIM_9295A_SER_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_DES_RegWrite(
  const enum CSI_IDX  acCsiIdx,
  const uint16_t      acRegAddr,
  const uint8_t       acVal
)
{
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mDeserializer;
  lXfr.mByteCnt = 3;

  lpData[0] = (acRegAddr >> 8);
  lpData[1] = (acRegAddr & 0xff);
  lpData[2] = acVal;

  // invoke transfer
  return CAM_WriteSingle(lXfr);
} // MAXIM_9296B_DES_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_9296B_DES_RegRead(
  const enum CSI_IDX  acCsiIdx,
  const uint16_t      acRegAddr,
  uint8_t            &arVal
)
{
  CAM_LIB_RESULT lRes = CAM_LIB_SUCCESS;
  // ***  stop deserializer mipi stream ***
  I2cXfrSingle_t lXfr;
  uint8_t       *lpData = (uint8_t*)&lXfr.mData;

  lXfr.mDestDev = spClients[acCsiIdx].mDeserializer;
  lXfr.mByteCnt = 3;

  lpData[0] = (acRegAddr >> 8);
  lpData[1] = (acRegAddr & 0xff);
  lpData[2] = 0;

  // invoke transfer
  lRes  = CAM_WriteSingle(lXfr);
  arVal = lpData[2];
  return lRes;
} // MAXIM_9296B_DES_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_Open(const MAXIM_Cfg_t &arConfig)
{
  CAM_LIB_RESULT lRet    = CAM_LIB_SUCCESS;

  if(CAM_Open() == CAM_LIB_SUCCESS)
  {
    CSI_IDX        lCsiIdx = arConfig.mCsiIdx;
    CamI2cCfg_t    lCamCfg;

    // if not initialized allready
    if(!spInitialized[lCsiIdx])
    {

      // *** prepare default I2C clients (deser/ser/cam) ***
      // deserializer
      lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) lCsiIdx;
      lCamCfg.mI2cAddr      = (uint8_t)I2C_MAXIM_DES_ADDR;
      lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

      memcpy(lCamCfg.mpCompatible,
              DESERIALIZER_COMPATIBLE_STRING,
              strlen(DESERIALIZER_COMPATIBLE_STRING));

      spClients[lCsiIdx].mDeserializer = CAM_ClientAdd(lCamCfg);

      if(spClients[lCsiIdx].mDeserializer != 0)
      {
        // serializer
        lCamCfg.mI2cAddr      = I2C_MAX_9271_ADDR(MAXIM_CHNL_ORIG_BC);
        spInitialized[lCsiIdx] = true;
        memcpy(lCamCfg.mpCompatible,
                SERIALIZER_COMPATIBLE_STRING,
                strlen(SERIALIZER_COMPATIBLE_STRING));

        spClients[lCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC] =
          CAM_ClientAdd(lCamCfg);

        if(spClients[lCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC] != 0)
        {
          // camera
          lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
          switch(arConfig.mSensorType)
          {
            case CSI_MAX_OV10640:
            {
              lCamCfg.mI2cAddr =
                     (uint8_t)I2C_MAX_OV10640_ADDR((uint8_t)MAXIM_CHNL_ORIG_BC);
              memcpy(lCamCfg.mpCompatible,
                OV10640_COMPATIBLE_STRING,
                strlen(OV10640_COMPATIBLE_STRING));
            } // case MAX_OV10640
              break;
            case CSI_MAX_SONY224:
            {
              lCamCfg.mI2cAddr =
                      (uint8_t)I2C_MAX_IMX224_ADDR((uint8_t)MAXIM_CHNL_ORIG_BC);
              memcpy(lCamCfg.mpCompatible,
                SONY_COMPATIBLE_STRING,
                strlen(SONY_COMPATIBLE_STRING));
            } // case MAX_SONY224
              break;
            case CSI_MAX_OV10635:
            {
              lCamCfg.mI2cAddr =
                     (uint8_t)I2C_MAX_OV10635_ADDR((uint8_t)MAXIM_CHNL_ORIG_BC);
              memcpy(lCamCfg.mpCompatible,
                OV10635_COMPATIBLE_STRING,
                strlen(OV10635_COMPATIBLE_STRING));
            } // case MAX_OV10635
              break;
            case CSI_MAX_OV9716:
            {
              lCamCfg.mI2cAddr = I2C_MAX_OV9716_ADDR(MAXIM_CHNL_ORIG_BC);
              memcpy(lCamCfg.mpCompatible,
                OV9716_COMPATIBLE_STRING,
                strlen(OV9716_COMPATIBLE_STRING));
            } // case MAX_OV9716
              break;
	  case CSI_MAX9286_96705:
	  {
		lCamCfg.mI2cAddr =
			   (uint8_t)I2C_MAX_OV490_ADDR((uint8_t)MAXIM_CHNL_ORIG_BC);
		memcpy(lCamCfg.mpCompatible,
		  OV490_OV10640_COMPATIBLE_STRING, 
		  strlen(OV490_OV10640_COMPATIBLE_STRING));
	  } // case CSI_MAX9286_96705
		break;

            default:
            {
              VDB_LOG_ERROR("Error: unknown sensor type.\n");
              lRet = CAM_LIB_FAILURE;
            } // default
              break;
          } // switch sensor type

          spClients[lCsiIdx].mpCameras[0] = CAM_ClientAdd(lCamCfg);
          if(spClients[lCsiIdx].mpCameras[0] == 0)
          {
            VDB_LOG_ERROR(
              "Failed to add default %s camera client.\n",
              lCamCfg.mpCompatible
            );
            lRet = CAM_LIB_FAILURE;
          } // if failed to add camera default client
        } // if default Serializer added successfully
        else
        {
          VDB_LOG_ERROR("Failed to add default serializer client.\n");
          lRet = CAM_LIB_FAILURE;
        } // else from if default Serializer added successfully

        if(lRet == CAM_LIB_FAILURE)
        {
          // free resource
          MAXIM_Close(lCsiIdx);
        }
      } // if Deserializer added successfully
      else
      {
        VDB_LOG_ERROR("Failed to add deserializer client.\n");
        lRet = CAM_LIB_FAILURE;
      } // else from if Deserializer added successfully
    } // if not created yet
    else
    {
      VDB_LOG_ERROR("Maxim QuadCam setup already active on CSI idx %u.\n",
                    (uint32_t)lCsiIdx);
      lRet = CAM_LIB_FAILURE;
    } // else if not created yet

    // *** initialize the rest of the HW setup ***
    if(lRet == CAM_LIB_SUCCESS)
    {
      // reinit only if camera type has changed
      if( spClients[lCsiIdx].mCfg.mSensorType != arConfig.mSensorType)
      {
        uint32_t i       = (uint32_t)0;
        uint32_t lCamCnt = (uint32_t)0;
        // get number of cameras used
        for(i = (uint32_t)0; i < (uint32_t)MAXIM_CAM_SLOTS; i++)
        {
          if(arConfig.mSensors[i] == MAXIM_CAM_ENABLED)
          {
            lCamCnt++;
          } // if sensor enabled
        } // for all possible MAXIM sensor slots

        spClients[lCsiIdx].mCfg = arConfig;

        switch(arConfig.mSensorType)
        {
          case CSI_MAX_OV10640:
          {
            lRet = MAXIM_OV10640_InitComplex(lCsiIdx, lCamCnt);
          } // case MAX_OV10640
            break;
          case CSI_MAX_SONY224:
          {
#ifndef __STANDALONE__
            int8_t lRetVal  = 0;
            OAL_LldCmd_t lCmd =
            {
              LLDCMD_SONY_CONNECT,
              (uintptr_t)spClients[lCsiIdx].mpCameras[0]
            };
            lRetVal = OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd);

            if (0L > lRetVal)
            {
              lRet = CAM_LIB_FAILURE;
            }
            else
            {
              lRet = CAM_LIB_SUCCESS;
            }
#else
            SONY_DRV_Setup(spClients[lCsiIdx].mpCameras[0]);
#endif // else #ifndef __STANDALONE__

            if(CAM_LIB_SUCCESS == lRet)
            {
              lRet = MAXIM_SONY_InitComplex(arConfig.mCsiIdx, lCamCnt);
            } // else from if failed
          } // case MAX_SONY224
            break;
          case CSI_MAX_OV10635:
          {
            lRet = MAXIM_OV10635_InitComplex(lCsiIdx, lCamCnt);
            VDB_LOG_WARNING("Ov10635 quad cam not implemented.");
          } // case MAX_OV10635
            break;
          case CSI_MAX_OV9716:
          {
            lRet = MAXIM_OV9716_InitComplex(lCsiIdx, lCamCnt);
            //lRet = MAXIM_DRV_OV10635_quadcam(apConfig->mCsiIdx, lCamCnt);
            VDB_LOG_WARNING("Ov9716 quad cam not implemented.");
          } // case MAX_OV9716
            break;
		case CSI_MAX9286_96705:
		{
		  lRet = MAXIM9286_96705_Init(lCsiIdx, lCamCnt);
		} // case MAX9286_96705
		  break;

          default:
          {
            VDB_LOG_ERROR("Unknown sensor type.");
            lRet = CAM_LIB_FAILURE;
          } // default
            break;
        } // switch sensor type

        if(lRet == CAM_LIB_FAILURE)
        {
          // free resource
          MAXIM_Close(lCsiIdx);
        }
      } // if camera type has changed
    } // if all ok

    
  } // if generic camera driver ready
  else
  {
    VDB_LOG_ERROR("Failed to open generic camera driver.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if generic camera driver ready

  return lRet;
} // SONY_Open()

//*****************************************************************************

CAM_LIB_RESULT MAXIM_Close(const enum CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  // if initialized
  if(spInitialized[aCsiIdx])
  {
    //********************************************************************
    //                    Reset All MAXIM configured registers           *
    //********************************************************************
    // TODO: hard code for MAXIM 9286 only.
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x69,
                  (uint8_t)0x0);
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x0A,
                  (uint8_t)0xFF);
    // 00100010
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x3F,
                    (uint8_t)0x22);
    msleep((uint32_t)2); // Wait 2ms
    // 00101110
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x3B,
                    (uint8_t)0x2E);
    // 00001011
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x15,
                    (uint8_t)0x0B);
    // 11110100
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x12,
                    (uint8_t)0xF4);
    // 00100010
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x01,
                    (uint8_t)0x22);
    // 11101111
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x00,
                    (uint8_t)0xEF);
    // 10110110
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x34,
                        (uint8_t)(0xB6));

    if(CAM_ClientRem(spClients[aCsiIdx].mDeserializer) != CAM_LIB_SUCCESS)
    {
      VDB_LOG_ERROR("Failed to remove the deserializer client.\n");
      lRet = CAM_LIB_FAILURE;
    } // if ClientRem() failed
    else
    {
      spClients[aCsiIdx].mDeserializer = 0;
    } // else from if CLientRem() failed

#ifndef __STANDALONE__
    if((spClients[aCsiIdx].mCfg.mSensorType == CSI_MAX_SONY224) && (spClients[aCsiIdx].mpCameras[0] != NULL))
    {
      OAL_LldCmd_t lCmd =
      {
        LLDCMD_SONY_DISCONNECT,
        (uintptr_t)spClients[aCsiIdx].mpCameras[0]
      };
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("Failed to disconnect the camera driver.\n");
        lRet = CAM_LIB_FAILURE;
      } // if failure
    }
#endif // #ifndef __STANDALONE__

    for(uint32_t i = (uint32_t)0; i < (uint32_t)5; i++)
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
   
    memset(&spClients[aCsiIdx].mCfg, 0, sizeof(MAXIM_Cfg_t));

    spInitialized[aCsiIdx] = false;
#ifndef __STANDALONE__
    if(CAM_LIB_FAILURE ==  CAM_PowerControl(aCsiIdx, SENSOR_TYPE_MAXIM_9286,
                                            CAM_POWER_STATE_DOWN))
    {
      VDB_LOG_ERROR("Power down viu #%d fail.\n", aCsiIdx);
      lRet = CAM_LIB_FAILURE;
    }
#endif // ifndef __STANDALONE__
  } // if Deserializer used

  return lRet;
} // MAXIM_Close()

//*****************************************************************************

CAM_LIB_RESULT MAXIM_CsiEnable(const enum CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  switch(spClients[aCsiIdx].mCfg.mSensorType)
  {
    case CSI_MAX_OV9716:
    {
      uint8_t lVal  = 0;
      uint8_t lVal2 = 0;
      uint16_t lAddr = ((uint16_t)0x30 << 8) + 0x12;

      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      (uint8_t)0x1);
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      (uint8_t)0x1); // camera streaming ON
      msleep(200);

      // Enable CSI output , BIT[3] = 1
      CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x15, lVal);
      CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                     (uint8_t)0x15, lVal|0x08);
      msleep(100);
      CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x15, lVal2);
      CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,
                      (uint8_t)0xf);
    } // case CSI_MAX_OV10635
      break;
    case CSI_MAX_OV10640: // fallthrough
    {
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[0],
                             (uint16_t)0x3012,
                             (uint8_t)0x1);
      // camera streaming ON
      if (CAM_LIB_SUCCESS == lRet)
      {
        lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[0],
                             (uint16_t)0x3012,
                             (uint8_t)0x1);

        msleep((uint32_t)200);
      }
    }  //case CSI_MAX_OV10640
    case CSI_MAX_OV10635: // fallthrough
    {
      uint8_t lVal  = (uint8_t)0;
      uint8_t lVal2 = (uint8_t)0;
      uint16_t lAddr = (uint16_t)((uint16_t)0x30 << 8) + (uint16_t)0x12;

      lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      (uint8_t)0x1);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      (uint8_t)0x1); // camera streaming ON
      msleep((uint32_t)200);

/*      printf("start MAXIM OV10640/OV10635\n"); */
      // Enable CSI output , BIT[3] = 1
      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,  lVal);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,
                      (uint8_t)(lVal | (uint8_t)0x08));
      msleep((uint32_t)100);
      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15, lVal2);
    } // case MAX_OV10635
      break;

    case CSI_MAX_SONY224:
    {
      uint8_t lVal  = (uint8_t)0;

      // Enable CSI output , BIT[3] = 1
      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15, lVal);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,
                      (uint8_t)(lVal | (uint8_t)0x08));
      msleep((uint32_t)100);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,
                      (uint8_t)0xF);
    } // case MAX_SONY224
      break;
	case CSI_MAX9286_96705:
	{
		printf("start MAXIM9286_96705\n");
		lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x15, (uint8_t)0x0B);//0B
		msleep((uint32_t)100);

	} // case MAX9286_96705
		break;

    default:
    {
      VDB_LOG_ERROR("Unknown sensor type.");
    } // default
      break;
  } // switch sensor type

  return lRet;
} // MAXIM_CsiEnable(enum CSI_IDX aCsiIdx)

//*****************************************************************************

CAM_LIB_RESULT MAXIM_CsiDisable(const enum CSI_IDX aCsiIdx)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  switch(spClients[aCsiIdx].mCfg.mSensorType)
  {
    case CSI_MAX_OV9716:
    {
      uint8_t lVal = (uint8_t)0;
      CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                      (uint8_t)0x15,
                      (uint8_t)(lVal&0x03));
    }
      break;
    case CSI_MAX_OV10640:
    {
      uint8_t lVal = (uint8_t)0;
      uint16_t lAddr = ((uint16_t)0x30 << 8) + 0x12;

      lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                             lAddr,
                             (uint8_t)0x0);

      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15, lVal);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15,
                            (uint8_t)(lVal&(uint8_t)0xF7));
    } // case MAX_OV10640
      break;
    case CSI_MAX_SONY224:
    {
      uint8_t lVal = (uint8_t)0;

      // Disable CSI output , BIT[3] = 0
      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15, lVal);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15, lVal&(uint8_t)0xF7);
    } // case MAX_SONY224
      break;
    case CSI_MAX_OV10635:
    {
      uint8_t lVal = (uint8_t)0;
      uint16_t lAddr = ((uint16_t)0x01 << 8) + 0x00;

      lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                             lAddr,
                             (uint8_t)0x0);

      // Disable CSI output , BIT[3] = 0
      lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15, lVal);
      lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                            (uint8_t)0x15, lVal&(uint8_t)0xF7);
    } // case MAX_OV10635
      break;
	case CSI_MAX9286_96705:
	{
			  
		lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x15, (uint8_t)0x03);
	} // case MAX9286_MAX96705
		break;
    default:
    {
      VDB_LOG_ERROR("Unknown sensor type.");
      lRet = CAM_LIB_FAILURE;
    } // default
      break;
  } // switch sensor type

  msleep((uint32_t)10);

  return lRet;
} // MAXIM_CsiDisable(enum CSI_IDX aCsiIdx)

//****************************************************************************

CAM_LIB_RESULT MAXIM_OV9716_InitComplex(enum CSI_IDX aCsiIdx, uint32_t acCamCnt)
{
  uint8_t        lVal     = 0;
  uint8_t        lVal3    = 0;
  uint8_t        lMAX9271 = 0;
  CAM_LIB_RESULT lRet     = CAM_LIB_SUCCESS;
  uint32_t       i        = 0;

  if(acCamCnt > MAXIM_CAM_SLOTS)
  {
    acCamCnt = MAXIM_CAM_SLOTS;
  }

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(aCsiIdx,I2C_MAXIM_DES_ADDR);
#endif
  //CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x34,0xAE);  // F4 before
  //CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x34,0xCE);  // John : i2c slave : setup hold time to 100khz
  // OV9716 requirement?
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x1C,
                  (uint8_t)0xF6);  // F4 before
  msleep(100);
  VDB_LOG_NOTE("Maxim Desializer ID test\n");
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                (uint8_t)0x1e, lVal);
  if(lVal != 0x40)
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }

  VDB_LOG_NOTE("Deserializer recognized.\n");

  /*CAM_ReadSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                 0x1e,
                 lVal);
  if(lVal == 0x09)  // check if OV9271
  {
    lMAX9271=1;
    printf("MAX9271 Serializer detected\n");
  }
  else if(lVal == 0x01)  // new serializer
  {
    printf("MAX96705 Serializer detected\n");
    lMAX9271=0;
  }
  else
  {
    lMAX9271=0;
    //printf("MAX96705 Serializer not identified\n");
    // deserializer not recognized
    printf("Serializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }*/
  lMAX9271=1;
  VDB_LOG_NOTE("Initialization of Reverse Control Channel Setup ");
  msleep(200);
  if(acCamCnt == 0x1)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0A,
                    (uint8_t)0x11); // enable all links control channels
  } // if one cam only
  else
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0A,
                    (uint8_t)0xff); // enable all links control channels (FWD+REV)
  } // else from if one cam only
  msleep(200);

  /*********************************************************************/
  /*                    Reverse Control Channel Setup                  */
  /*********************************************************************/

  if (lMAX9271)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x3F,
                    (uint8_t)0x4F); //Enable Custom Reverse Channel & First Pulse Length
    msleep(200); // Wait 2ms

    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    (uint8_t)0x08,
                    (uint8_t)0x01); //Serializer input gain and thresholds
    msleep(200); // Wait 2ms

    // doesn't work with the 0V9716
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x3B,
                    (uint8_t)0x1E); //Reverse Channel Amplitude to mid level and transition time
    msleep(2); // Wait 2ms
  }

  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04,
                  (uint8_t)0x43); //Enable Configuration Link
  msleep(5); // Wait 5ms

  msleep(2); // Wait 2ms

  if (lMAX9271)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x3B,
                    (uint8_t)0x19);  //Reverse Channel Amplitude level
    msleep(2); // Wait 2ms
  }

  VDB_LOG_NOTE("....Completed\n");

#ifdef I2C_100KHZ
  //change the SER I2C speed to 100kHz
  //CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC], 0x0D, 0xAE); //John
#endif // ifdef I2C_100KHZ
  msleep(100); // Wait 100ms


  /*********************************************************************/
  /*                   MAX9286 Pre MAX9271 Setup                       */
  /*********************************************************************/
  VDB_LOG_NOTE("Initialization of MAX9286 Pre MAX9271 Setup ");
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x41,
                  (uint8_t)0x00);    //John : High pass filter cut-off frequency //12-12
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x15,
                  (uint8_t)0x03);    //Disable CSI output
  if(acCamCnt == 0x1)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x12,
                    (uint8_t)0x77);    //CSI Lanes, CSI DBL, GMSL DBL, Data Type RAW 11/12
  } // if one cam only
  else
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x12,
                    (uint8_t)0xF7);    //CSI Lanes, CSI DBL, GMSL DBL, Data Type RAW 11/12
  } // else from if one cam only


  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x01,
                  (uint8_t)0x00);    //Internal frame sync uses manual mode, Internally generate frame sync, FSYNC/GPI is high impedance

  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x06,
                  (uint8_t)0x00);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x07,
                  (uint8_t)0xD4);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x08,
                  (uint8_t)0x30);    //FSYNC period
/*
#ifdef FPS_OV9716_30
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x06,0x00);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x07,0xD4);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x08,0x30);    //FSYNC period
#endif  //FPS_OV9716_30

#ifdef FPS_OV9716_27_5
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x06,0x74);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x07,0x95);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x08,0x42);    //FSYNC period
#endif  //FPS_OV9716_22

#ifdef FPS_OV9716_20
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x06,0x00);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x07,0x3E);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x08,0x49);    //FSYNC period
#endif  //FPS_OV9716_20

#ifdef FPS_OV9716_15
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x06,0x00);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x07,0xA8);    //FSYNC period
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x08,0x61);    //FSYNC period
#endif  //FPS_OV9716_15
  */
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x69,
                  (uint8_t)0x30);    //Auto Come Back
  if(acCamCnt == 0x1)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x00,
                    (uint8_t)0x81);    //Enable GMSL Links, all inputs, VSYNC comes from Camera
  } // if one cam only
  else
  {
    //CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x00,0x87);    //Enable GMSL Links, all inputs, VSYNC comes from Camera
    // proposed by Maxim
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x00,
                    (uint8_t)0x8F);    //Enable GMSL Links, all inputs, VSYNC comes from Ca
  } //else from if one cam only
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x07,
                  (uint8_t)0xC4);    //DBL, HBW (proposed by Maxim), Edge Select, HS/VS Encoding
  msleep(2); // Wait 2ms

  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x67,
                  (uint8_t)0xC0);    //by Maxim to avoid line lenth errors
  msleep(2); // Wait 2ms


  VDB_LOG_NOTE("....Completed\n");
  /*********************************************************************/
  /*                          Setup Link 0 - Setup Link CamCnt         */
  /*********************************************************************/
  VDB_LOG_NOTE("Initialization of virtual I2C addresses");
  lVal3 = 0;

  CamI2cCfg_t    lCamCfg;
  CamI2cCfg_t    lSerCfg;
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
  lSerCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lSerCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

  for (i = 1; i < (acCamCnt + 1); i++)
  {
    lVal3 = (lVal3 << 1) + 0x1;
    // Enable link 0 reverse control channel to serializer (sending)
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0xA,
                    (uint8_t)(0xF0+(1<<(i-1))));
    //Change serializer I2C Slave Address to I2C_MAX_9271_ADDR(i)
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    (uint8_t)0x0,
                    (uint8_t)(I2C_MAX_9271_ADDR(i) << 1));

    lSerCfg.mI2cAddr = I2C_MAX_9271_ADDR(i);
    spClients[aCsiIdx].mpSerializers[i] = CAM_ClientAdd(lSerCfg);

    // Unique Link 0 image sensor slave address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x9,
                    (uint8_t)(I2C_MAX_OV9716_ADDR(i) << 1));

    lCamCfg.mI2cAddr = I2C_MAX_OV9716_ADDR(i);
    spClients[aCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

    //Link [i] Image Sensor Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xA,
                    (uint8_t)(I2C_MAX_OV9716_ADDR(0) << 1));

    //Serializer Broadcast Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xB,
                    (uint8_t)(I2C_MAX_9271_ADDR(0) << 1));

    //Link [i] Serializer Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xC,
                    (uint8_t)(I2C_MAX_9271_ADDR(i) << 1));
#if 0
   //Serializer Broadcast Address
    CAM_ReadSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    0xB,
                    val8_0);

    //Link [i] Serializer Address
    CAM_ReadSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    0xC,
                    val8_1);
    printf(" final   addresses BC %x Line %x\n",val8_0,val8_1);
#endif // #if 0
  }
  VDB_LOG_NOTE("....Completed\n");

  /*********************************************************************/
  /*                   MAX9286 Post MAX9271 Setup                      */
  /*********************************************************************/
  VDB_LOG_NOTE("MAX9286 Post MAX9271 Setup \n");
  if(acCamCnt == 0x1)
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0a,
                    (uint8_t)0x11);    //Enable all I2C reverse and forward channels
  } // if one cam only
  else
  {
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0a,
                    (uint8_t)0xFF);    //Enable all I2C reverse and forward channels
  } // else from if one cam only
  msleep(2); // Wait 2ms

 // *** Camera reset ***
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3013,
                  (uint8_t)0x01);
  msleep(1000); //wait 1second
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3012,
                  (uint8_t)0x00); // make sure in standby
  msleep(200); //wait 200ms
  VDB_LOG_NOTE("....Completed\n");

  // *** Camera config ***
  VDB_LOG_NOTE("Configuring camera registers");
  {
    uint8_t *lpCamConfig = Ov9716_Table;
    uint32_t lConfigSize = sizeof(Ov9716_Table);
    uint32_t j = 0;
    for(j = 0; j < lConfigSize; j += 3) // Configure omnivision cameras
    {
      uint16_t lAddr = ((uint16_t)lpCamConfig[j] << 8) + lpCamConfig[j + 1];
        CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                        lAddr,
                        lpCamConfig[j+2]);
    }
    msleep(10);
  } // for all attached cameras
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3012,
                  (uint8_t)0x1);

  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x30BD,
                  (uint8_t)0x93); // FSIN Enable
  msleep(200); //wait 200ms
  //CAM_ReadSingle(spClients[aCsiIdx].mpCameras[1], 0x30BD, lVal2);
  //msleep(200); //wait 200ms
  //printf("Read OV9716 0x30BD = 0x%x\n", lVal2);
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3489,
                  (uint8_t)0x02); // GPIO1 Enable
  msleep(200); //wait 200ms
  //CAM_ReadSingle(spClients[aCsiIdx].mpCameras[1], 0x3489, lVal2);
  //printf("Read OV9716 0x3489 = 0x%x\n", lVal2);
  //msleep(200); //wait 200ms

     // *** Camera config END ***
  VDB_LOG_NOTE("....Completed\n");


  VDB_LOG_NOTE("Enabling serializer");
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04,
                  (uint8_t)0x83);    //Enable all serializer and disable configuration link
  msleep(5); // Wait 5ms
  VDB_LOG_NOTE("....Completed\n");

  MAXIM_CsiDisable(aCsiIdx);

#if 0
  uint8_t lpMaxDump[256] = {0};
  MAXIM_DeserDump(lpMaxDump, 256);

  for(uint32_t i = 0; i < 256; i ++)
  {
    (lpMaxDump[i] > 0xf)?printf("0x%x ", lpMaxDump[i]):printf("0x0%x ", lpMaxDump[i]);
    if(((i + 1) % 16) == 0)
    {
      printf("\n");
    }
  } // for all
#endif // 0
  return lRet;
} // MAXIM_OV9716_InitComplex()

//****************************************************************************

CAM_LIB_RESULT MAXIM_OV10640_InitComplex(const enum CSI_IDX aCsiIdx,
                                         uint32_t acCamCnt)
{
  uint8_t        lVal  = (uint8_t)0;
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(aCsiIdx,I2C_MAXIM_DES_ADDR);
#endif

  VDB_LOG_NOTE("Maxim OV10640 cameras test\n");
  lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x1e, lVal);
  if(lVal != (uint8_t)0x40)
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }

  VDB_LOG_NOTE("Deserializer recognized.\n");

  VDB_LOG_NOTE("Initialization of Reverse Control Channel Setup ");

  // enable all links control channels (FWD+REV)
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x0A,
                        (uint8_t)0xff);

  /*********************************************************************/
  /*                    Reverse Control Channel Setup                  */
  /*********************************************************************/

  //Enable Custom Reverse Channel & First Pulse Length
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x3F,
                        (uint8_t)0x4F);
  msleep((uint32_t)2); // Wait 2ms
  //Reverse Channel Amplitude to mid level and transition time
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x3B,
                        (uint8_t)0x1E);
  msleep((uint32_t)2); // Wait 2ms
  //Enable Configuration Link
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                        (uint8_t)0x04,
                        (uint8_t)0x43);
  msleep((uint32_t)5); // Wait 5ms
  //Serializer input gain and thresholds
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                        (uint8_t)0x08,
                        (uint8_t)0x01);
  msleep((uint32_t)2); // Wait 2ms
  //Reverse Channel Amplitude level
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x3B,
                        (uint8_t)0x19);
  msleep((uint32_t)2); // Wait 2ms

  VDB_LOG_NOTE("....Completed\n");

  //change the SER I2C speed to 100kHz
#ifndef __STANDALONE__ // use default I2C speed (400kHz)
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                        (uint8_t)0x0D,
                        (uint8_t)SERIALIZER_FREQUENCY);
#endif //#ifndef __STANDALONE__
  msleep((uint32_t)100); // Wait 100ms


  /*********************************************************************/
  /*                   MAX9286 Pre MAX9271 Setup                       */
  /*********************************************************************/
  VDB_LOG_NOTE("Initialization of MAX9286 Pre MAX9271 Setup ");
   //Disable CSI output
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x15,
                        (uint8_t)0x03);
  //CSI Lanes, CSI DBL, GMSL DBL, Data Type RAW 11/12
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x12,
                        (uint8_t)0xF7);
  //Internal frames sync uses semi-automatic mode
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x01,
                        (uint8_t)0x01);
  //Auto Come Back
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x69,
                        (uint8_t)0x30);
  //Enable GMSL Links, all inputs, VSYNC comes from Camera
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x00,
                        (uint8_t)0x8F);
  //DBL, Edge Select, HS/VS Encoding
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                        (uint8_t)0x07,
                        (uint8_t)0x84);    //DBL, Edge Select, HS/VS Encoding
  msleep((uint32_t)2); // Wait 2ms

  VDB_LOG_NOTE("....Completed\n");
  /*********************************************************************/
  /*                          Setup Link 0 - Setup Link CamCnt         */
  /*********************************************************************/
  VDB_LOG_NOTE("Initialization of virtual I2C addresses");

  uint8_t        lVal2 = (uint8_t)0;
  uint8_t        lVal3 = (uint8_t)0;

  CamI2cCfg_t    lCamCfg;
  CamI2cCfg_t    lSerCfg;
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
  lSerCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lSerCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

  memcpy(lCamCfg.mpCompatible,
          OV10640_COMPATIBLE_STRING,
          strlen(OV10640_COMPATIBLE_STRING));
  memcpy(lSerCfg.mpCompatible,
          SERIALIZER_COMPATIBLE_STRING,
          strlen(SERIALIZER_COMPATIBLE_STRING));

  uint32_t i = (uint32_t)0;
  if(acCamCnt > MAXIM_CAM_SLOTS)
  {
    acCamCnt = MAXIM_CAM_SLOTS;
  }
  for (i = (uint32_t)1; i < (acCamCnt + (uint32_t)1); i++)
  {
    lVal3 = (uint8_t)(lVal3 << 1) + (uint8_t)0x1;
     // Enable link 0 reverse control channel to serializer (sending)
    CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0xA,
                    (uint8_t)0xF0 + (uint8_t)((uint8_t)1<<(i-(uint32_t)1)));
    msleep((uint32_t)2);

    // //Change serializer I2C Slave Address to I2C_MAX_9271_ADDR(i)
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    (uint8_t)0x0,
                    (uint8_t)(I2C_MAX_9271_ADDR(i) << 1));

    lSerCfg.mI2cAddr = I2C_MAX_9271_ADDR(i);
    spClients[aCsiIdx].mpSerializers[i] = CAM_ClientAdd(lSerCfg);

    // Create unique image sensor slave address for link[i]
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x9,
                    (uint8_t)(I2C_MAX_OV10640_ADDR(i) << 1));

    lCamCfg.mI2cAddr = I2C_MAX_OV10640_ADDR(i);
    spClients[aCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

    //Remember original Image Sensor Address on link[i]
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[(uint8_t)i],
                    (uint8_t)0xA,
                    (uint8_t)(I2C_MAX_OV10640_ADDR(0U) << 1));

    //Serializer Broadcast Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xB,
                    (uint8_t)(I2C_MAX_9271_ADDR(0U) << 1));

    //Original Link [i] Serializer Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xC,
                    (uint8_t)(I2C_MAX_9271_ADDR(i) << 1));
  }
  VDB_LOG_NOTE("....Completed\n");

  /*********************************************************************/
  /*                   MAX9286 Post MAX9271 Setup                      */
  /*********************************************************************/
  VDB_LOG_NOTE("MAX9286 Post MAX9271 Setup \n");
  //Enable all I2C reverse and forward channels
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0a,
                    (uint8_t)0xFF);
  msleep((uint32_t)2); // Wait 2ms

  VDB_LOG_NOTE("Checking %d attached OV10640 cameras", acCamCnt);
  lVal3 = (uint8_t)0;
  for (i = (uint32_t)1; i < (acCamCnt + (uint32_t)1); i++)
  {
    CAM_ReadSingle(spClients[aCsiIdx].mpCameras[i],
                    (uint16_t)OV10640_CHIP_ID_HIGH_BYTE, lVal3);
    if (lVal3 != (uint8_t)0xA6)
    {
      VDB_LOG_NOTE(
        "camera ov10640_mipi was not found (high_byte) id %x\n",
        lVal3);
      return CAM_LIB_FAILURE;
    }

    msleep((uint32_t)5);

    CAM_ReadSingle(spClients[aCsiIdx].mpCameras[i],
                  (uint16_t)OV10640_CHIP_ID_LOW_BYTE,
                   lVal3);
    if (lVal3 != (uint8_t)0x40) {
      VDB_LOG_NOTE("camera ov10640_mipi was not found (low_byte)\n");
      return CAM_LIB_FAILURE;
    }
  } // for all attached cameras
  VDB_LOG_NOTE("....Completed\n");

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x34, lVal);
  // Disable Local Auto I2C ACK
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x34,
                  (uint8_t)(lVal & (uint8_t)0x7F));
  msleep((uint32_t)100);
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x34, lVal2);
  if(lVal2 != (uint8_t)(lVal & (uint8_t)0x7F))
  {
    VDB_LOG_NOTE("I2c check failed (Disable local auto i2c ack) 0x%x.\n",
                 lVal2);
  }
  VDB_LOG_NOTE("Reseting cameras");
  // *** Camera reset ***
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3013,
                  (uint8_t)0x01);
  msleep((uint32_t)200); //wait 200ms
  // make sure in standby
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3012,
                  (uint8_t)0x00);

  VDB_LOG_NOTE("....Completed\n");

  // *** Camera config ***
  VDB_LOG_NOTE("Configuring camera registers");
  {
    uint8_t *lpCamConfig = Ov10640_Table3;
    uint32_t lConfigSize = (uint32_t)sizeof(Ov10640_Table3);
    uint32_t j = (uint32_t)0;
    // Configure omnivision cameras
    for(j = (uint32_t)0; j < lConfigSize; j += (uint32_t)3)
    {
      uint16_t lAddr = (uint16_t)((uint16_t)lpCamConfig[j] << 8)
                                      + (uint16_t)lpCamConfig[j + (uint32_t)1];
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      lpCamConfig[j+(uint32_t)2]);
    }
    msleep((uint32_t)10);
    //   Enable External Frame Syncs
    for(j=(uint32_t)0; j < (uint32_t)sizeof(Ov10640_FsinTable); j+=(uint32_t)3)
    {
      uint16_t lAddr = (uint16_t)((uint16_t)Ov10640_FsinTable[j] << 8)
                                 + (uint16_t)Ov10640_FsinTable[j + (uint32_t)1];
      CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                      lAddr,
                      Ov10640_FsinTable[j+(uint32_t)2]);
      msleep((uint32_t)100);
    } // for all attached cameras

    CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                    (uint16_t)0x3012,
                    (uint8_t)0x1);
  } // Camera config
  VDB_LOG_NOTE("....Completed\n");

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x34, lVal);
  // Enable Local Auto I2C ACK
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x34,
                    (uint8_t)(lVal | (uint8_t)0x80));
  msleep((uint32_t)100);
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x34, lVal2);
  if(lVal2 != (uint8_t)(lVal | (uint8_t)0x80))
  {
    VDB_LOG_NOTE("I2c check failed (Enable local auto i2c ack) 0x%x.\n", lVal2);
  }
  VDB_LOG_NOTE("Enabling serializer");
   //Enable all serializer and disable configuration link
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    (uint8_t)0x04,
                    (uint8_t)0x83);
  msleep((uint32_t)5); // Wait 5ms
  VDB_LOG_NOTE("....Completed\n");

  MAXIM_CsiDisable(aCsiIdx);

/*
  uint8_t lpMaxDump[256] = {0};
  MAXIM_DeserDump(lpMaxDump, 256);

  for(uint32_t i = 0; i < 256; i ++)
  {
    if(lpMaxDump[i] > 0xf)
    {
     printf("0x%x ", lpMaxDump[i]);
    }
    else
    {
      printf("0x0%x ", lpMaxDump[i]);
    }
    if(((i + 1) % 16) == 0)
    {
      printf("\n");
    }
  } // for all
*/
  return lRet;
} // MAXIM_OV10640_InitComplex()

//****************************************************************************

CAM_LIB_RESULT MAXIM_OV10640_InitSimple(const enum CSI_IDX aCsiIdx,
                                                 uint32_t acCamCnt)
{
  uint8_t        lVal  = (uint8_t)0;
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

  uint32_t lCnt = (uint32_t)INIT_DELAY;
  // introduce i2c initialization delay if desired

  for(;lCnt > (uint32_t)0; lCnt--)
  {
    VDB_LOG_NOTE("%u \n", lCnt);
    msleep((uint32_t)500);
  }

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(aCsiIdx,I2C_MAXIM_DES_ADDR);
#endif

  // *** A ***
  VDB_LOG_NOTE("=== Section A ===\n");
  msleep((uint32_t)20);
  lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x1e, lVal);
  if(lVal != (uint8_t)0x40)
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }

  VDB_LOG_NOTE("Deserializer recognized.\n");
/*added
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer, 0x34, lVal);
  VDB_LOG_NOTE("DesReg addr 0x34 val = %x \n", lVal);

   enable only the first link control channel (FWD+REV)
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,0x0A, 0xF1);
   Jason ADD - begin
*/
  // enable all links control channels (FWD+REV)
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                       (uint8_t)0x0A,
                       (uint8_t)0xff);
  //Updated to current procedure 1-14-16 Blaine Ashley
  msleep((uint32_t)1000);
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                       (uint8_t)0x3F,
                       (uint8_t)0x4F); //BA 1-14-16
  msleep((uint32_t)1000); //BA 1-14-16
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                       (uint8_t)0x3B,
                       (uint8_t)0x1E); //BA 1-14-16
  msleep((uint32_t)1000); //BA 1-14-16
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                       (uint8_t)0x04,
                       (uint8_t)0x43); //BA 1-14-16
  msleep((uint32_t)1000); //BA 1-14-16
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                       (uint8_t)0x00,
                       (uint8_t)0x8F); //TB
  msleep((uint32_t)1000); //BA 1-14-16
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                       (uint8_t)0x08,
                       (uint8_t)0x01); //BA 1-14-16
  msleep((uint32_t)1000); //BA 1-14-16
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                       (uint8_t)0x3B,
                       (uint8_t)0x19); //BA 1-14-16
  msleep((uint32_t)1000); //BA 1-14-16


  VDB_LOG_NOTE("Please run initializatio  n\n");
  lCnt = (uint32_t)INIT_DELAY;
  for(;lCnt > (uint32_t)0; lCnt--)
  {
    VDB_LOG_NOTE("%u \n", lCnt);
    msleep((uint32_t)500);
  }
  VDB_LOG_NOTE("Cnt down finished\n");

  //change the SER I2C speed
#ifndef __STANDALONE__ // use default I2C speed (400kHz)
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x0D,
                  (uint8_t)SERIALIZER_FREQUENCY);
#endif  // #ifndef __STANDALONE__
  msleep((uint32_t)2000); //YW 1-20-16

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x15, lVal);
  // Disable CSI output Bits[3]=0b0
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x15,
                  (uint8_t)(lVal & (uint8_t)0xF7));


  uint32_t i = (uint32_t)0;
  if(acCamCnt > MAXIM_CAM_SLOTS)
  {
    acCamCnt = MAXIM_CAM_SLOTS;
  }
  VDB_LOG_NOTE("%u cameras attached.\n", acCamCnt);
  uint8_t lVal2 = (uint8_t)0;

  for(i = (uint32_t)0; i < acCamCnt; i++)
  {
    lVal2 = (uint8_t)(lVal2 << 1) + (uint8_t)1;
  } // for all cameras
  lVal2 += (uint8_t)0x80;
  // Auto detect csi clock source, disable VSYNC gen (will come from camera),
  // enable input links from used cameras
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x0,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0x00) | lVal2));
  // set MIPI-CSI-2 data type
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x12, lVal);
  // RAW12. single input mode, csidbl = 0
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x12,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0xC0) | (uint8_t)0x07));
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x01, lVal);
  // enable Frame Sync
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x01,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0x01) | (uint8_t)0x01));

  // *** B ***
  VDB_LOG_NOTE("=== Section B ===\n");

  CAM_ReadSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x7, lVal);

  VDB_LOG_NOTE("Register 0x07 of Serial[%u] contains 0x%x\n", 0, lVal);
  // Enable single mode, PCLK latched on Rising edge, Enable HS/VS encoding
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x7,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0x6F) | (uint8_t)0x04));

  msleep((uint32_t)1000); // Wait 3 ms min

  // *** F ***
  VDB_LOG_NOTE("=== Section F ===\n");

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x34, lVal);
  // Disable Local Auto I2C ACK
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x34,
                  (uint8_t)(lVal & (uint8_t)0x7F));
  msleep((uint32_t)1000);

  // reset the camera
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3013,
                  (uint8_t)0x01);
  msleep((uint32_t)100);

  // make sure in standby
  CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3012,
                  (uint8_t)0x00);
  msleep((uint32_t)1000);

  // *** Camera config ***
  {
    uint8_t *lpCamConfig = Ov10640_Table3;
    uint32_t lConfigSize = (uint32_t)sizeof(Ov10640_Table3);
    uint32_t j = (uint32_t)0;

    // Configure omnivision cameras
    for(j = (uint32_t)0; j < lConfigSize; j += (uint32_t)3)
    {
        uint16_t lAddr = (uint16_t)((uint16_t)lpCamConfig[j] << 8)
                                       + (uint16_t)lpCamConfig[j + (uint32_t)1];
        //for (i = 1; i < (acCamCnt + 1); i++)//BA 1-14-16
        //{        //BA 1-14-16
        CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                        lAddr,
                        lpCamConfig[j+(uint32_t)2]);
        msleep((uint32_t)20);
        //} // for all attached cameras//BA 1-14-16
    } // for all configurations
    VDB_LOG_NOTE("Configured camera registers.\n");

    //   Enable External Frame Syncs
    for(j = (uint32_t)0; j < sizeof(Ov10640_FsinTable); j += (uint32_t)3)
    {
        uint16_t lAddr = (uint16_t)((uint16_t)Ov10640_FsinTable[j] << 8)
                          + (uint16_t)Ov10640_FsinTable[j + (uint32_t)1];
        //for (i = 1; i < (acCamCnt + 1); i++)//BA 1-14-16
        //{        //BA 1-14-16
        CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                        lAddr,
                        Ov10640_FsinTable[j+(uint32_t)2]);
        msleep((uint32_t)200);
        //} // for all configurations//BA 1-14-16
    } // for all attached cameras

    // put cam to standby
    CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                        (uint16_t)0x3012,
                        (uint8_t)0x1);
    VDB_LOG_NOTE("FSYNC on. Cameras in standby\n");
  } // Camera config

  msleep((uint32_t)1000);
  VDB_LOG_NOTE("Enabled external frame syncs.\n");

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x34, lVal);
  // Enable Local Auto I2C ACK
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                        (uint8_t)0x34,
                        (uint8_t)(lVal | (uint8_t)0x80));
  msleep((uint32_t)1000);

  VDB_LOG_NOTE("=== Section H ===\n");
  lVal = (uint8_t)0;
  msleep((uint32_t)100);

  CAM_ReadSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04, lVal);
  // Try to enable serial links
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0xBF) | (uint8_t)0x80));
  msleep((uint32_t)2000);
  VDB_LOG_NOTE("Ser%u done.\n", i);

  msleep((uint32_t)1000);  // Wait 10 ms min
  // enable all links control channels (FWD+REV)
  msleep((uint32_t)1000);
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x12, lVal);

  // Enable CSI data lanes D0 - D3 , BIT[7:6]=01 // DBL OFF, CSI_DBL OFF
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x12,
                  (uint8_t)((uint8_t)(lVal & (uint8_t)0x0F) | (uint8_t)0xC0));

  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x31, lVal);
  VDB_LOG_NOTE("Des 0x31: 0x%x\n", lVal);

  msleep((uint32_t)1000);

  MAXIM_CsiDisable(aCsiIdx);
  msleep((uint32_t)3000);

  return lRet;
} // MAXIM_OV10640_InitSimple()

#if 1   //max96705+max9286  8bit  mark semi-auto new camera
CAM_LIB_RESULT MAXIM9286_96705_Init(enum CSI_IDX aCsiIdx,
                                         uint32_t acCamCnt)
{
  uint8_t        lVal  = 0;
  uint8_t        lVal2 = 0;
  uint8_t        lVal3 = 0;
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;
  uint32_t       i     = 0;
  uint8_t reg, sensor_addr = 0;
  unsigned int g_sensor_num = 0;
  unsigned char g_sensor_is_there = 0;
  uint8_t tmp,tmp1;

  if(acCamCnt > MAXIM_CAM_SLOTS)
  {
    acCamCnt = MAXIM_CAM_SLOTS;
  }

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(aCsiIdx,I2C_MAXIM_DES_ADDR);
#endif

  printf("Maxim9286_96705 cameras init\n");
/*
  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x1e, lVal);
  if(lVal != 0x40)
  {
    // deserializer not recognized
    printf("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }

  printf("Deserializer recognized.\n");

  printf("Initialization of Reverse Control Channel Setup ");
*/
	CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x15, (uint8_t)0x03);
	  msleep(10);

	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0A, (uint8_t)0x00);
	  msleep(10);

	/*********************add 20190822***********************/
		  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0C, (uint8_t)0x91);
		  msleep(10);	  
	      printf("add Deserializer address 0x0c value is 0x91 for polarity.\n");
		//for test yl cameral don't output  and don't output Duty ratio
	//	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, 0x0C, 0x99);
	//	  msleep(10);	  

	//Disable CSI Output,CSI-2 outputs on virtual channel 0,Set virtual channel according to the link number,disable CSI-2 output

	  //Enable PRBS test
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0E, (uint8_t)0x5F);
	  msleep(10);


	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x1B, (uint8_t)0x0F);	 //enable equalizer
	  msleep(10);
	  
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x32, (uint8_t)0x44);	 //
	  msleep(10);
	  
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x33, (uint8_t)0x44);	 //
	  msleep(10);


	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x12, (uint8_t)0xF3);//0xF3 //F4	  //YUV422,8bit,Double Data Rate, 4 data lane
	  msleep(5);

	  //Frame Sync

	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x06, (uint8_t)0x00);//25->00
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x07, (uint8_t)0xf0);//c0->f0
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x08, (uint8_t)0x25);//24->25
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x01, (uint8_t)0x00);	  //?\A8\A4?\A1\A4?\A1꨺?\A1\EA??\A8\B2\A8\AA?2?\A1\EA?0x01[7:6]=0b00,
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x02, (uint8_t)0x00);	  //?\A8\B4?Y???\A1\A7?\A8\B9\A8\AC\A8\AA?\A8\AE//00->30
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x63, (uint8_t)0x00);
	  msleep(10);

	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x64, (uint8_t)0x00);
	  msleep(10);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x69, (uint8_t)0x30);// enable automask autoback
	  msleep(10);

	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x05, (uint8_t)0x19);	  //?\A8\B4?Y???\A1\A7?\A8\B9\A8\AC\A8\AA?\A8\AE

	  msleep(100);

	  // Detect link
	  g_sensor_num = 0;
	  CAM_ReadSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x49,reg);  //?\A8\AC2alink?\A8\A6??,\A8\AE|???\A8\AC2a\A6\CC?link2,
	  reg = 0xFF;
	  g_sensor_is_there = ((reg >> 4) & 0xF) | (reg & 0xF);
	  if (g_sensor_is_there & (0x1 << 0))
		  g_sensor_num += 1;
	  if (g_sensor_is_there & (0x1 << 1))
		  g_sensor_num += 1;
	  if (g_sensor_is_there & (0x1 << 2))
		  g_sensor_num += 1;
	  if (g_sensor_is_there & (0x1 << 3))
		  g_sensor_num += 1;
	  printf("max9286_mipi: sensor number = %d.g_sensor_is_there = 0x%x\n", g_sensor_num,g_sensor_is_there); //\A6\CC?\A6\CC?\A8\AEDD\A1쨢\A1\E4?\A1\A4\A1ꡧ?\A8\AE\A8\A6?\A8\A8\A3\A4\A6\CC?\A8\A6???\A8\AA\A1\A4\A1\EA???\A8\BAy

	  if (g_sensor_num == 0) {
		  printf("%s: no camera connected.\n", __func__);
		  return CAM_LIB_FAILURE;
	  }

	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0E, (uint8_t)0x50);	  //Disable PRBS test
	  msleep(2);
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0D, (uint8_t)0x03);	  //???\A1\A7?\A8\B9

	  // Set link order in MIPI CSI-2 output
	  reg = 0xE4;  //Default setting
	  if (g_sensor_num == 1) {			  //??\A8\A2??\A8\AE1??\A8\A6???\A8\AA\A1\A4
		  switch (g_sensor_is_there) {
			  case 0x8: 	  //link3
				  reg = 0x27;
				  break;
			  case 0x4: 	  //link2
				  reg = 0xC6;
				  break;
			  case 0x2: 	  //link1
				  reg = 0xE1;
				  break;
			  case 0x1: 	  //link0
			  default:
				  reg = 0xE4;
				  break;
		  }
	  } else if (g_sensor_num == 2) {	  //\A8\A2??\A8\AE\A8\A2?2??\A8\A6???\A8\AA\A1\A4
		  switch (g_sensor_is_there) {
			  case 0xC:
				  reg = 0x4E;
				  break;
			  case 0xA:
				  reg = 0x72;
				  break;
			  case 0x9:
				  reg = 0x78;
				  break;
			  case 0x6:
				  reg = 0xD2;
				  break;
			  case 0x5:
				  reg = 0xD8;
				  break;
			  case 0x3:
			  default:
				  reg = 0xE4;
				  break;
		  }
	  } else if (g_sensor_num == 3) {	  //\A8\A2??\A8\AE\A8\A2?3??\A8\A6???\A8\AA\A1\A4
		  switch (g_sensor_is_there) {
			  case 0xE:
				  reg = 0x93;
				  break;
			  case 0xD:
				  reg = 0x9C;
				  break;
			  case 0xB:
				  reg = 0xB4;
				  break;
			  case 0x7:
			  default:
				  reg = 0xE4;
				  break;
		  }
	  }
	  //DesRegWrite(0x0B, reg);   //0x0BŒÄŽæÆ÷Îªlink out order
	  reg = 0xE0 | g_sensor_is_there;
	  //DesRegWrite(0x00, reg);   //enable all links,auto select
	  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x00, (uint8_t)0xEF);

  /*********************************************************************/
  /*                          Setup Link 0 - Setup Link CamCnt         */
  /*********************************************************************/
  printf("Initialization of virtual I2C addresses");
  lVal3 = 0;

  CamI2cCfg_t    lCamCfg;
  CamI2cCfg_t    lSerCfg;
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
  lSerCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lSerCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

  memcpy(lCamCfg.mpCompatible,
          OV490_OV10640_COMPATIBLE_STRING,
          strlen(OV490_OV10640_COMPATIBLE_STRING));
  memcpy(lSerCfg.mpCompatible,
          SERIALIZER_COMPATIBLE_STRING,
          strlen(SERIALIZER_COMPATIBLE_STRING));

  for (i = 1; i < (acCamCnt + 1); i++)
  {
    lVal3 = (lVal3 << 1) + 0x1;
    if (((0x1 << (i-1)) & g_sensor_is_there) == 0)
		{
			continue;
		}

		//Enable Link control channel
		reg = (0x11 << (i-1));
		CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0A, (uint8_t)reg);

	  	msleep(5);
    // //Change serializer I2C Slave Address to I2C_MAX_96705_ADDR(i)
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[0],
                    (uint8_t)0x0,
                    (uint8_t)(I2C_MAX_96705_ADDR(i) << 1));

    lSerCfg.mI2cAddr = I2C_MAX_96705_ADDR(i);
    spClients[aCsiIdx].mpSerializers[i] = CAM_ClientAdd(lSerCfg);

	CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x01, (uint8_t)(I2C_MAXIM_DES_ADDR << 1));
	msleep(5);

    // Unique Link 0 image sensor slave address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x9,
                    (uint8_t)(I2C_MAX_OV490_ADDR(i) << 1));

    lCamCfg.mI2cAddr = I2C_MAX_OV490_ADDR(i);
    spClients[aCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

    //Link [i] Image Sensor Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xA,
                    (uint8_t)(I2C_MAX_OV490_ADDR(0) << 1));

    //Serializer Broadcast Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xB,
                    (I2C_MAX_96705_ADDR(5) << 1));
    msleep(10);
    //Link [i] Serializer Address
    CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0xC,
                    (uint8_t)(I2C_MAX_96705_ADDR(i) << 1));
/*add hukai 20190902 enable aichi cam and delay aption sensor back porch */
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x20, (uint8_t)0x07); 
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x21, (uint8_t)0x06); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x22, (uint8_t)0x05); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x23, (uint8_t)0x04); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x24, (uint8_t)0x03); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x25, (uint8_t)0x02); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x26, (uint8_t)0x01); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x27, (uint8_t)0x00); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x30, (uint8_t)0x17); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x31, (uint8_t)0x16); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x32, (uint8_t)0x15); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x33, (uint8_t)0x14); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x34, (uint8_t)0x13); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x35, (uint8_t)0x12); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x36, (uint8_t)0x11); //
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x37, (uint8_t)0x10); //

  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x43, (uint8_t)0x25); 
  CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x47, (uint8_t)0x23); 


	CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x07, (uint8_t)0x84);
	msleep(5);
#if 0 //setting CML_Level  PreemPhasis_Level and   Spread_Spectrum  function
	CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x02, (uint8_t)0x3f);
	msleep(5);
	CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i], (uint8_t)0x06, (uint8_t)0x80);
	msleep(5);
#endif
  }

  printf("....Completed\n");

  /*********************************************************************/
  /*                   MAX9286 Post MAX9271 Setup                      */
  /*********************************************************************/
  printf("MAX9286 Post MAX96705 Setup \n");
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0a,(uint8_t)0xFF);    //Enable all I2C reverse and forward channels
  msleep(2); // Wait 2ms
  CAM_WriteSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x0a,(uint8_t)0xFF);    //Enable all I2C reverse and forward channels
  msleep(2); // Wait 2ms

    CAM_ReadSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x70, lVal);
    printf("Maxim9286_96705 Init statu lVal=0x%x, L:%d,R:%d,F:%d,B:%d.\n",lVal, (lVal>>6&(uint8_t)0x01),
                                     (lVal>>7&(uint8_t)0x01), (lVal>>4&(uint8_t)0x01), (lVal>>5&(uint8_t)0x01));
	    CAM_ReadSingle(spClients[aCsiIdx].mDeserializer, (uint8_t)0x49, lVal2);
	printf("@@@@@@@@@@@lVal2 = 0x%x\n",lVal2);

  msleep(100); // Wait 5ms
  printf("....Completed\n");

  MAXIM_CsiDisable(aCsiIdx);

  return lRet;
} // MAXIM_OV10640_InitComplex()
#endif

#if 0
//***********************add hukai 20190730****************************//
CAM_LIB_RESULT MAXIM9286_96705_Channel_Check(uint8_t lVal)
{
	CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;
	uint32_t  i = 0;
	uint8_t lVal3 =0;
	uint8_t reg;
	uint8_t tmp;
	uint8_t lVal2;
	uint8_t tmp1;
	uint8_t a[4] = {0};

	if(lVal != 0xf0) {		
		a[0] = (lVal>>4&0x01); //F
		a[1] = (lVal>>5&0x01); //B
		a[2] = (lVal>>6&0x01); //L
		a[3] = (lVal>>7&0x01); //R

		for (i = 0; i < 4; i++)
		{
			if (a[i] == 0)
			{
				i=i+1;				
				reg = (0x11 << (i-1));
				MAXIM_DES_RegWrite(CSI_IDX_0, 0x0A, reg);
				 msleep(5);
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x0, I2C_MAX_96705_ADDR(i) << 1, (MAXIM_CHNL)0);
				 msleep(5);

				MAXIM_SER_RegWrite(CSI_IDX_0, 0x1, I2C_MAXIM_DES_ADDR << 1,  (MAXIM_CHNL)i);
				  msleep(5);

				// Unique Link 0 image sensor slave address
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x9, I2C_MAX_OV490_ADDR(i) << 1, (MAXIM_CHNL)i);
				 msleep(5);
				
				//Link [i] Image Sensor Address
				MAXIM_SER_RegWrite(CSI_IDX_0, 0xA, I2C_MAX_OV490_ADDR(0) << 1, (MAXIM_CHNL)i);		
				 msleep(5);
				
				//Serializer Broadcast Address
				MAXIM_SER_RegWrite(CSI_IDX_0, 0xB, I2C_MAX_96705_ADDR(5) << 1, (MAXIM_CHNL)i);				
				 msleep(5);
				
				//Link [i] Serializer Address
				MAXIM_SER_RegWrite(CSI_IDX_0, 0xC, I2C_MAX_96705_ADDR(i) << 1, (MAXIM_CHNL)i);					
				 msleep(5);
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x07, 0X84, (MAXIM_CHNL)i);								
				 msleep(5);
				// MAXIM_DES_RegWrite(CSI_IDX_0, 0x1C, 0xF4);		
				//usleep(1000);
				// MAXIM_DES_RegWrite(CSI_IDX_0, 0x0A, 0xFF);		
				//usleep(1000);
				 MAXIM_DES_RegWrite(CSI_IDX_0, 0x0A, 0xFF);
				 msleep(5);
				//MAXIM_DES_RegRead(CSI_IDX_0, (uint8_t)0x49, lVal3);
			}
		}
		MAXIM_DES_RegRead(CSI_IDX_0, (uint8_t)0x49, lVal3);
		if(lVal3 ==0xf)
		{
			lRet = CAM_LIB_CHECK;
		}else 
		{
			lRet = CAM_LIB_FAILURE;
		}
	}
	 return lRet;
}
#endif

//*****************************************************************************
#if 0
CAM_LIB_RESULT MAXIM_CML_Level(uint32_t x)
 {
 	uint8_t lVal =0;
 	uint8_t tmp =0;
	CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

	if ((x< 2) || (x>10))
		{
			printf("input level invail\n");
			lRet = CAM_LIB_FAILURE;
		}
	else {
		for (uint32_t  i=1; i<5; i++)
		{
		 	MAXIM_SER_RegRead(CSI_IDX_0, (uint8_t)0x06, lVal,(MAXIM_CHNL)i);
			tmp = (lVal & 0xf);
			printf("%s->current level is %d\n",__func__,tmp);
		switch (x)
			{
			case 2:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x20 |tmp),  (MAXIM_CHNL)i);					
				break;
			case 3:	
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x30 |tmp), (MAXIM_CHNL)i);					
				break;
			case 4:	
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x40 |tmp), (MAXIM_CHNL)i);					
				break;
			case 5:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x50 |tmp), (MAXIM_CHNL)i);					
				break;
			case 6:	
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x60 |tmp), (MAXIM_CHNL)i);					
				break;
			case 7:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x70 |tmp), (MAXIM_CHNL)i);					
				break;
			case 8:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x80 |tmp), (MAXIM_CHNL)i);					
				break;
			case 9:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x90 |tmp), (MAXIM_CHNL)i);					
				break;
			case 10:
				MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0xa0 |tmp), (MAXIM_CHNL)i);					
				break;
			default:
				break;
			}
		}
	}	
	return lRet;
 } 
//#endif
//*****************************************************************************
//#if 1
CAM_LIB_RESULT MAXIM_PreemPhasis_Level(uint32_t x)
 {
 	uint8_t lVal =0;
 	uint8_t tmp =0;
	CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

	if((x>4) && (x<8) )
	{
		printf("@@@@preemphasis do not use\n");
		lRet = CAM_LIB_FAILURE;
	}
	for (uint32_t  i=1; i<5; i++)
	{
	 	MAXIM_SER_RegRead(CSI_IDX_0, (uint8_t)0x06, lVal, (MAXIM_CHNL)i);
		tmp = ((lVal>>4) & 0xf);
		printf("%s->current PreemPhasis level is %d\n",__func__,tmp);

		if (x==0)
		{
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0 |tmp),  (MAXIM_CHNL)i);	//off		
			printf("@@@@preemphasis is off\n");
		}
		if ((x>0) && (x<5))
		{
			switch (x)
				{
				case 1:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x01 |(tmp<<4)),  (MAXIM_CHNL)i);	//1.2dB			
					break;
				case 2:	
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x02 |(tmp<<4)), (MAXIM_CHNL)i);	// 2.5dB			
					break;
				case 3:	
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x03 |(tmp<<4)), (MAXIM_CHNL)i);	// 4.1dB				
					break;
				case 4:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x04 |(tmp<<4)), (MAXIM_CHNL)i);	//6.0dB				
					break;
				default:
					break;
				}
		}	
		if ((x>7) && (x<16)) 
		{
			switch (x)
				{
				case 8:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x08 |(tmp<<4)),  (MAXIM_CHNL)i);	//1.1dB			
					break;
				case 9:	
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x09 |(tmp<<4)), (MAXIM_CHNL)i);	// 2.2dB			
					break;
				case 10:	
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0a |(tmp<<4)), (MAXIM_CHNL)i);	// 3.3dB				
					break;
				case 11:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0b |(tmp<<4)), (MAXIM_CHNL)i);	//4.4dB				
					break;
				case 12:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0c |(tmp<<4)), (MAXIM_CHNL)i);	//6.0dB				
					break;
				case 13:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0d |(tmp<<4)), (MAXIM_CHNL)i);	//8.0dB				
					break;
				case 14:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0e |(tmp<<4)), (MAXIM_CHNL)i);	//10.5dB				
					break;
				case 15:
					MAXIM_SER_RegWrite(CSI_IDX_0, 0x06, (0x0f |(tmp<<4)), (MAXIM_CHNL)i);	//14.0dB				
					break;
				default:
					break;
				}
		}
	}		
	return lRet;
 } 
//#endif
//*****************************************************************************
//#if 1
CAM_LIB_RESULT MAXIM_Spread_Spectrum(uint32_t x)
 {
 	uint8_t lVal =0;
 	uint8_t tmp =0;
	CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;
	
	for (uint32_t  i=1; i<5; i++)
	{
	 	MAXIM_SER_RegRead(CSI_IDX_0, (uint8_t)0x02, lVal, (MAXIM_CHNL)i);
		tmp = (lVal & 0xf);
		printf("%s->current Spread_Spectrum level is %d\n",__func__,tmp);

		if ((x==0) || (x==4))
		{
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0x10 |tmp),  (MAXIM_CHNL)i);
			printf("@@@@spread spectrum is off\n");
			lRet = CAM_LIB_FAILURE;
		}
		switch (x)
		{
		case 1:
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0x30 |tmp),  (MAXIM_CHNL)i); //0.5%				
			break;
		case 2:	
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0x50 |tmp), (MAXIM_CHNL)i);	 //1.5%				
			break;
		case 3:	
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0x70 |tmp), (MAXIM_CHNL)i);	 //2%				
			break;
		case 5:
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0xb0 |tmp), (MAXIM_CHNL)i);  //1%
			break;
		case 6:	
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0xd0 |tmp), (MAXIM_CHNL)i);  //3%					
			break;
		case 7:
			MAXIM_SER_RegWrite(CSI_IDX_0, 0x02, (0xf0 |tmp), (MAXIM_CHNL)i);	 //4%				
			break;
		default:
			break;
		}
	}
	return lRet;
 } 
#endif

//*****************************************************************************


CAM_LIB_RESULT MAXIM_CAM_RegWrite(
  const enum CSI_IDX  acCsiIdx,
  const uint16_t acRegAddr,
  const uint8_t  acVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  return CAM_WriteSingle(spClients[acCsiIdx].mpCameras[acChnlIdx],
                         acRegAddr,
                         acVal);
} // MAXIM_CAM_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_CAM_RegRead(
  const enum CSI_IDX acCsiIdx,
  const uint16_t acRegAddr,
  uint8_t  &arVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  return CAM_ReadSingle(spClients[acCsiIdx].mpCameras[acChnlIdx],
                        acRegAddr,
                        arVal);
} // MAXIM_CAM_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_SER_RegWrite(
  const enum CSI_IDX  acCsiIdx,
  const uint8_t acRegAddr,
  const uint8_t  acVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  return CAM_WriteSingle(spClients[acCsiIdx].mpSerializers[acChnlIdx],
                         acRegAddr,
                         acVal);
} // MAXIM_SER_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_SER_RegRead(
  const enum CSI_IDX  acCsiIdx,
  const uint8_t acRegAddr,
  uint8_t  &arVal,
  const enum MAXIM_CHNL acChnlIdx
)
{
  return CAM_ReadSingle(spClients[acCsiIdx].mpSerializers[acChnlIdx],
                        acRegAddr,
                        arVal);
} // MAXIM_SER_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_DES_RegWrite(
  const enum CSI_IDX  acCsiIdx,
  const uint8_t acRegAddr,
  const uint8_t  acVal
)
{
  return CAM_WriteSingle(spClients[acCsiIdx].mDeserializer, acRegAddr, acVal);
} // MAXIM_DES_RegWrite

//*****************************************************************************

CAM_LIB_RESULT MAXIM_DES_RegRead(
  const enum CSI_IDX  acCsiIdx,
  const uint8_t acRegAddr,
  uint8_t &arVal
)
{
  return CAM_ReadSingle(spClients[acCsiIdx].mDeserializer, acRegAddr, arVal);
} // MAXIM_DES_RegRead

//*****************************************************************************

CAM_LIB_RESULT MAXIM_OV10640_ExposureSet(OV10640_Exposure_t &arExp)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[0];
  return OV10640_ExposureSet(arExp);
} // MAXIM_OV10640_ExposureSet

//****************************************************************************

CAM_LIB_RESULT MAXIM_OV10635_InitComplex(const enum CSI_IDX acCsiIdx,
                                         uint32_t acCamCnt)
{
  uint8_t        lVal      = (uint8_t)0;
  CAM_LIB_RESULT lRet      = CAM_LIB_SUCCESS;
  I2cXfrSingle_t lDesXfr;
  I2cXfrSingle_t lSerXfr;
  I2cXfrSingle_t lCamXfr;
  uint8_t       *lpDesData = (uint8_t*)&lDesXfr.mData;
  uint8_t       *lpSerData = (uint8_t*)&lSerXfr.mData;
  uint8_t       *lpCamData = (uint8_t*)&lCamXfr.mData;

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(acCsiIdx, I2C_MAXIM_DES_ADDR);
#endif

  // *** init transfer structs ***
  lDesXfr.mDestDev = spClients[acCsiIdx].mDeserializer;
  lDesXfr.mByteCnt = 2;
  lSerXfr.mDestDev = spClients[acCsiIdx].mpSerializers[0];
  lSerXfr.mByteCnt = 2;
  lCamXfr.mDestDev = spClients[acCsiIdx].mpCameras[0];
  lCamXfr.mByteCnt = 3;

  // *** begin config ***
  VDB_LOG_NOTE("Deserializer connection test\n");
  lpDesData[0] = 0x1e; lpDesData[1] = 0x0;
  lRet = CAM_ReadSingle(lDesXfr);
  if(lpDesData[1] != (uint8_t)0x40)
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lpDesData[1]);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }
  VDB_LOG_NOTE("Deserializer recognized.\n");

  printf("------------------- Hello world -----------------\n");

  // enable all links control channels (FWD+REV)
  lpDesData[0] = 0x0a; lpDesData[1] = 0xff;
  lRet = CAM_WriteSingle(lDesXfr);

  /*********************************************************************/
  /*                    Reverse Control Channel Setup                  */
  /*********************************************************************/

  //Enable Custom Reverse Channel & First Pulse Length
  lpDesData[0] = 0x3f; lpDesData[1] = 0x4f;
  lRet = CAM_WriteSingle(lDesXfr);
  msleep((uint32_t)2); // Wait 2ms
  //Reverse Channel Amplitude to mid level and transition time
  lpDesData[0] = 0x3b; lpDesData[1] = 0x1e;
  lRet = CAM_WriteSingle(lDesXfr);
  msleep((uint32_t)2); // Wait 2ms
  //Enable Configuration Link
  lpSerData[0] = 0x04; lpSerData[1] = 0x43;
  lRet = CAM_WriteSingle(lSerXfr);
  msleep((uint32_t)5); // Wait 5ms
  //Serializer input gain and thresholds
  lpSerData[0] = 0x08; lpSerData[1] = 0x01;
  lRet = CAM_WriteSingle(lSerXfr);
  msleep((uint32_t)2); // Wait 2ms
  //Reverse Channel Amplitude level
  lpDesData[0] = 0x3b; lpDesData[1] = 0x19;
  lRet = CAM_WriteSingle(lDesXfr);
  msleep((uint32_t)2); // Wait 2ms

  //change the SER I2C speed to 100kHz
  lpSerData[0] = 0x0d; lpSerData[1] = SERIALIZER_FREQUENCY;
  lRet = CAM_WriteSingle(lSerXfr);
  msleep((uint32_t)100); // Wait 100ms

  /*********************************************************************/
  /*                   MAX9286 Pre MAX9271 Setup                       */
  /*********************************************************************/
  //Disable CSI output
  lpDesData[0] = 0x15; lpDesData[1] = 0x03;
  lRet = CAM_WriteSingle(lDesXfr);
  //CSI Lanes, CSI DBL, GMSL DBL, Data YUV422 10bit Type dbl check datatype
  lpDesData[0] = 0x12; lpDesData[1] = 0xf4;
  lRet = CAM_WriteSingle(lDesXfr);
  //Frame Sync
  lpDesData[0] = 0x01; lpDesData[1] = 0x02;
  lRet = CAM_WriteSingle(lDesXfr);
  //Auto Come Back
  lpDesData[0] = 0x69; lpDesData[1] = 0x30;
  lRet = CAM_WriteSingle(lDesXfr);
  //Enable GMSL Links
  lpDesData[0] = 0x00; lpDesData[1] = 0xff;
  lRet = CAM_WriteSingle(lDesXfr);
  //DBL, Edge Select, HS/VS Encoding
  lpSerData[0] = 0x07; lpSerData[1] = 0x84;
  lRet = CAM_WriteSingle(lSerXfr);
  msleep((uint32_t)2); // Wait 2ms

  /*********************************************************************/
  /*                          Setup Link 0 - Setup Link CamCnt         */
  /*********************************************************************/

  CamI2cCfg_t    lCamCfg;
  CamI2cCfg_t    lSerCfg;
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) acCsiIdx;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
  lSerCfg.mI2cBusIdx    = (I2C_BUS_IDX) acCsiIdx;
  lSerCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

  memcpy(lCamCfg.mpCompatible,
          OV10635_COMPATIBLE_STRING,
          strlen(OV10635_COMPATIBLE_STRING));
  memcpy(lSerCfg.mpCompatible,
          SERIALIZER_COMPATIBLE_STRING,
          strlen(SERIALIZER_COMPATIBLE_STRING));

  uint32_t i = (uint32_t)0;
  lVal = (uint8_t)0;
  if(acCamCnt > MAXIM_CAM_SLOTS)
  {
    acCamCnt = MAXIM_CAM_SLOTS;
  }
  for (i = (uint32_t)1; i < (acCamCnt + (uint32_t)1); i++)
  {
    lVal = (uint8_t)(lVal << 1) + 0x1;
    // Enable link 0 reverse control channel to serializer (sending)
    lpDesData[0] = 0x0a;
    lpDesData[1] = (uint8_t)((uint8_t)0xF0+(uint8_t)((uint8_t)1<<(i-(uint32_t)1)));
    lRet         = CAM_WriteSingle(lDesXfr);

    // //Change serializer I2C Slave Address to I2C_MAX_9271_ADDR(i)
    lSerXfr.mDestDev = spClients[acCsiIdx].mpSerializers[0];

    lpSerData[0] = 0x00;
    lpSerData[1] = (uint8_t)((uint8_t)I2C_MAX_9271_ADDR(i) << 1);
    lRet         = CAM_WriteSingle(lSerXfr);

    lSerCfg.mI2cAddr = (uint8_t)I2C_MAX_9271_ADDR(i);
    spClients[acCsiIdx].mpSerializers[i] = CAM_ClientAdd(lSerCfg);

    // Unique Link 0 image sensor slave address
    lSerXfr.mDestDev = spClients[acCsiIdx].mpSerializers[i];

    lpSerData[0] = 0x09;
    lpSerData[1] = (uint8_t)((uint8_t)I2C_MAX_OV10635_ADDR(i) << 1);
    lRet         = CAM_WriteSingle(lSerXfr);

    lCamCfg.mI2cAddr = (uint8_t)I2C_MAX_OV10635_ADDR(i);
    spClients[acCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

    //Link [i] Image Sensor Address
    lpSerData[0] = 0x0a;
    lpSerData[1] = (uint8_t)(I2C_MAX_OV10635_ADDR((uint8_t)0) << 1);
    lRet         = CAM_WriteSingle(lSerXfr);

    //Serializer Broadcast Address
    lpSerData[0] = 0x0b;
    lpSerData[1] = (uint8_t)(I2C_MAX_9271_ADDR(0U) << 1);
    lRet         = CAM_WriteSingle(lSerXfr);

    //Link [i] Serializer Address
    lpSerData[0] = 0x0c;
    lpSerData[1] = (uint8_t)((uint8_t)I2C_MAX_9271_ADDR(i) << 1);
    lRet         = CAM_WriteSingle(lSerXfr);
  }

  /*********************************************************************/
  /*                   MAX9286 Post MAX9271 Setup                      */
  /*********************************************************************/
  //Enable all I2C reverse and forward channels
  lpDesData[0] = 0x0a; lpDesData[1] = 0xff;
  lRet = CAM_WriteSingle(lDesXfr);

  msleep((uint32_t)2); // Wait 2ms

  // *** F ***
  VDB_LOG_NOTE("=== Section F ===\n");
  lpDesData[0] = 0x34; lpDesData[1] = 0x00;
  lRet = CAM_ReadSingle(lDesXfr);

  // Disable Local Auto I2C ACK
  lpDesData[0] = 0x34; lpDesData[1] &= 0x7f;
  lRet = CAM_WriteSingle(lDesXfr);

  msleep((uint32_t)2); // Wait 2ms
  // *** Camera config ***
  {
    uint8_t *lpCamConfig = OV10635_Table;
    uint32_t lConfigSize = (uint32_t)sizeof(OV10635_Table);
    uint32_t j = (uint32_t)0;

    // Configure omnivision cameras
    for(j = (uint32_t)0; j < lConfigSize; j += (uint32_t)3)
    {
      lpCamData[0] = lpCamConfig[j];
      lpCamData[1] = lpCamConfig[j + (uint32_t)1];
      lpCamData[2] = lpCamConfig[j + (uint32_t)2];
      lRet         = CAM_WriteSingle(lCamXfr);
    } // for all configurations
    VDB_LOG_NOTE("Configured camera registers.\n");

    msleep((uint32_t)10);

    //   Enable External Frame Syncs
    for(j=(uint32_t)0; j < (uint32_t)sizeof(Ov10635_FsinTable); j+=(uint32_t)3)
    {
      lpCamData[0] = Ov10635_FsinTable[j];
      lpCamData[1] = Ov10635_FsinTable[j + (uint32_t)1];
      lpCamData[2] = Ov10635_FsinTable[j + (uint32_t)2];
      lRet         = CAM_WriteSingle(lCamXfr);
    } // for all attached cameras

    VDB_LOG_NOTE("FSYNC on. Cameras in standby\n");
  } // Camera config

  lpDesData[0] = 0x34; lpDesData[1] = 0x00;
  lRet         = CAM_ReadSingle(lDesXfr);

  // Enable Local Auto I2C ACK
  lpDesData[0] = 0x34; lpDesData[1] |= 0x80;
  lRet         = CAM_WriteSingle(lDesXfr);

  /*********************************************************************/
  /*               MAX9286 Post Image Sensor Setup                     */
  /*********************************************************************/

  //Enable all serializer and disable configuration link
  lSerXfr.mDestDev = spClients[acCsiIdx].mpSerializers[0];
  lpSerData[0]     = 0x04; lpSerData[1] = 0x83;
  lRet             = CAM_WriteSingle(lSerXfr);
  msleep((uint32_t)5); // Wait 5ms

  MAXIM_CsiDisable(acCsiIdx);

  return lRet;
} // MAXIM_OV10635_InitComplex()

//****************************************************************************

CAM_LIB_RESULT MAXIM_SONY_InitComplex(const enum CSI_IDX aCsiIdx,
                                                 uint32_t acCamCnt)
{
  uint8_t        lVal  = (uint8_t)0;
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

#ifdef __STANDALONE__
  VDB_LOG_NOTE("Initialize the I2C\n");
  I2C_Init_Config(aCsiIdx,I2C_MAXIM_DES_ADDR);
#endif

  VDB_LOG_NOTE("Deserializer connection test\n");
  lRet = CAM_ReadSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x1e, lVal);
  if(lVal != (uint8_t)0x40)
  {
    // deserializer not recognized
    VDB_LOG_NOTE("Deserializer not recognized 0x%x.\n", lVal);
    lRet = CAM_LIB_FAILURE;
    return lRet;
  }
  VDB_LOG_NOTE("Deserializer recognized.\n");

  /*********************************************************************/
  /*                    Reverse Control Channel Setup                  */
  /*********************************************************************/
  // Enable Custom Reverse Channel & First Pulse lengh
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x3F,
                   (uint8_t)0x4F);
  msleep((uint32_t)2); // Wait 2ms
  // First pulse length rise time changed from 300ns to 200ns
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x3B,
                   (uint8_t)0x1E);
  msleep((uint32_t)2); // Wait 2ms
  // Enable configuration link, use I2C link as local control channel
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04,
                   (uint8_t)0x43);
  msleep((uint32_t)5); // Wait 5ms
  // Enable high threshold for reverse channel input buffer.
  // This increases immunity to power supply noise when the coaxial link is used
  // for power as well as signal.
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x08,
                   (uint8_t)0x81);
  msleep((uint32_t)2); // Wait 2ms
  // Increase reverse amplitude from 100mV to 170mV.
  // This compensates for the higher threshold of step 5.
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x3B,
                          (uint8_t)0x19);
  msleep((uint32_t)2); // Wait 2ms

  //Auto Come Back
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x69,
                          (uint8_t)0x30);

  // change the SER I2C speed to 100kHz fro Linux
  // on SA use default I2C speed (400kHz)
  #ifndef __STANDALONE__
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                         (uint8_t)0x0D,
                          (uint8_t)SERIALIZER_FREQUENCY);
  #endif //#ifndef __STANDALONE__
  msleep((uint32_t)100); // Wait 100ms

  /*********************************************************************/
  /*                   MAX9286 Pre MAX9271 Setup                       */
  /*********************************************************************/
  // Disable CSI output
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x15,
                          (uint8_t)0x03);
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                         (uint8_t)0x07,
                          (uint8_t)0xA5);
  msleep((uint32_t)2); // Wait 2ms
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x0C,
                          (uint8_t)0xB9);
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x12,
                         (uint8_t)0xF7);
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x1C,
                         (uint8_t)0x05);
  msleep((uint32_t)1000); // Wait 100ms
  // Internally generate frame sync, FSYNC/GPI outputs frame sync,
  // Internal frame sync uses automatic mode
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x01,
                          (uint8_t)0x00);
  msleep((uint32_t)1000); // Wait 100ms
  //FPS calculation , FPS defined in sony_config.h
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x08,
                         (uint8_t)((37500000/FPS) >> 16));    //MSB byte
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x07,
                         (uint8_t)((37500000/FPS) >> 8));      //Mid byte
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x06,
                         (uint8_t)(37500000/FPS));            //Low byte

  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x63,
                          (uint8_t)0x00);     //
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                         (uint8_t)0x64,
                          (uint8_t)0x00);     //

  uint32_t i = (uint32_t)0;
  lVal = (uint8_t)0;
  if(acCamCnt > (uint32_t)MAXIM_CAM_SLOTS)
  {
    acCamCnt = (uint32_t)MAXIM_CAM_SLOTS;
  }

  for(i = (uint32_t)0; i < acCamCnt; i++)
  {
    lVal = (uint8_t)(lVal << 1) + 1;
  } // for all cameras
  // Auto detect csi clock source, disable VSYNC gen (will come from camera),
  // enable input links from used cameras
  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,(uint8_t)0x0,
                         (uint8_t)((uint8_t)0xE0 + lVal));

  msleep((uint32_t)10); // Wait 10ms

  /*********************************************************************/
  /*                          Setup Link 0 - Setup Link CamCnt         */
  /*********************************************************************/

  CamI2cCfg_t    lCamCfg;
  CamI2cCfg_t    lSerCfg;
  lCamCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lCamCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_16;
  lSerCfg.mI2cBusIdx    = (I2C_BUS_IDX) aCsiIdx;
  lSerCfg.mRegAddrWidth = I2C_REG_ADDR_WIDTH_8;

  memcpy(lCamCfg.mpCompatible,
          SONY_COMPATIBLE_STRING,
          strlen(SONY_COMPATIBLE_STRING));
  memcpy(lSerCfg.mpCompatible,
          SERIALIZER_COMPATIBLE_STRING,
          strlen(SERIALIZER_COMPATIBLE_STRING));

  // 4. GMSL Link Setup
  lVal = (uint8_t)0;
  for (i = (uint32_t)1; i < acCamCnt + (uint32_t)1; i++)
  {
    lVal = (uint8_t)(lVal << 1) + 0x1;
    // Enable link 0 reverse control channel to serializer (sending)
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
              (uint8_t)0x0A,
               (uint8_t)0xF0+(uint8_t)((uint8_t)1<<((uint8_t)(i-(uint32_t)1))));
    msleep((uint32_t)10); // Wait 10ms

    // Change I2C slave ID of MAX9271 to 0x82
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                    (uint8_t)0x00,
                    (uint8_t)((uint8_t)I2C_MAX_9271_ADDR(i) << 1));

    lSerCfg.mI2cAddr = (uint8_t)I2C_MAX_9271_ADDR(i);
    spClients[aCsiIdx].mpSerializers[i] = CAM_ClientAdd(lSerCfg);

    msleep((uint32_t)10); // Wait 10ms
    // Unique Link 0 image sensor slave address
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x09,
                    (uint8_t)((uint8_t)I2C_MAX_IMX224_ADDR(i) << 1));

    lCamCfg.mI2cAddr = (uint8_t)I2C_MAX_IMX224_ADDR(i);
    spClients[aCsiIdx].mpCameras[i] = CAM_ClientAdd(lCamCfg);

    msleep((uint32_t)10); // Wait 10ms
    // Link image sensor slave address
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x0A,
                    (uint8_t)(I2C_MAX_IMX224_ADDR((uint8_t)0) << 1));
    msleep((uint32_t)10); // Wait 10ms
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x0B,
                    (uint8_t)(I2C_MAX_9271_ADDR((uint8_t)0) << 1));
    msleep((uint32_t)10); // Wait 10ms
    // Program original address of MAX9271 = 0x82
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[i],
                    (uint8_t)0x0C,
                    (uint8_t)((uint8_t)(I2C_MAX_9271_ADDR(i) << 1)));
    msleep((uint32_t)10); // Wait 10ms
  }

  lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                    (uint8_t)0x0A,
                     (uint8_t)0xFF);
  msleep((uint32_t)2); // Wait 2ms
  // 5. Image Sensor Initialization

  // *** Camera config ***
  {
    uint8_t *lpCamConfig = IMX224_CMOS_SDR_HD720p_12bit_30fps;
    uint32_t lConfigSize = (uint32_t)sizeof(IMX224_CMOS_SDR_HD720p_12bit_30fps);
    uint32_t j = (uint32_t)0;

    // Configure sony cameras
    for(j = (uint32_t)3; j < lConfigSize; j += (uint32_t)3)
    {
        uint16_t lAddr =(uint16_t)((uint16_t)lpCamConfig[j] << 8)
                                       + (uint16_t)lpCamConfig[j + (uint32_t)1];
        lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_0],
                        lAddr,
                        lpCamConfig[j+(uint32_t)2]);
        lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_1],
                        lAddr,
                        lpCamConfig[j+(uint32_t)2]);
        lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_2],
                        lAddr,
                        lpCamConfig[j+(uint32_t)2]);
        lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_3],
                        lAddr,
                        lpCamConfig[j+(uint32_t)2]);
        msleep((uint32_t)20);
    } // for all configurations
    VDB_LOG_NOTE("Configured camera registers.\n");
  } // Camera config

  if (CAM_LIB_SUCCESS == lRet)
  {
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpCameras[MAXIM_CHNL_ORIG_BC],
                  (uint16_t)0x3000,
                   (uint8_t)0x00);    //getting camera to normal mode

    /*********************************************************************/
    /*                   MAX9286 Post MAX9271 Setup                      */
    /*********************************************************************/

    lRet = CAM_WriteSingle(spClients[aCsiIdx].mpSerializers[MAXIM_CHNL_ORIG_BC],
                  (uint8_t)0x04,
                   (uint8_t)0x83);
    msleep((uint32_t)5); // Wait 5ms
    // Enable CSI data lanes D0, D1, D2, D3
    lRet = CAM_WriteSingle(spClients[aCsiIdx].mDeserializer,
                  (uint8_t)0x12,
                   (uint8_t)0xF7);
    msleep((uint32_t)5); // Wait 5ms
    MAXIM_CsiDisable(aCsiIdx);
  }

  return lRet;
} // MAXIM_SONY_InitComplex()

//***************************************************************************

void MAXIM_DeserDump(uint8_t* const apDump, uint32_t aSize)
{
  uint32_t i = (uint32_t)0;

  aSize = (aSize > (uint32_t)64)?(uint32_t)64:aSize;

  for( i = (uint32_t)0x0; i < aSize; i++)
  {
    uint8_t lVal = (uint8_t)0;

    CAM_ReadSingle(spClients[0].mDeserializer, (uint8_t)i, lVal);
    apDump[i] = lVal;
    msleep((uint32_t)30);
  }
} // MAXIM_DeserDump()

// **************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureSet(CSI_IDX aCsiIdx,
                                        uint8_t aCamNum,
                                        OV9716_Exposure_t &arExp,
                                        float aLSUpdate,
                                        float aVSUpdate)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
  uint8_t lCamIndex;

  if (aCamNum == MAXIM_CHNL_ORIG_BC)
  {
    lCamIndex = 0;       // use index for cam0 and broadcast
  }
  else
  {
    lCamIndex = aCamNum - 1;    // use index for camx
  }

  arExp.mClient = spClients[aCsiIdx].mpCameras[lCamIndex];

  lRet = OV9716_ExposureSet(arExp, aLSUpdate, aVSUpdate);

  return lRet;
}
//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureTimeSet(OV9716_Exposure_t &arExp,
                            OV9716_ExposureType_t aExpT)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureTimeSet(arExp, aExpT);
}

//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureTimeGet(OV9716_Exposure_t &arExp,
                                      OV9716_ExposureType_t aExpT)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureTimeGet(arExp, aExpT);
}

//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureDigitalGainSet(OV9716_Exposure_t &arExp,
                                                   OV9716_ExposureType_t aExpT)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureDigitalGainSet(arExp, aExpT);
}

//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureDigitalGainGet(OV9716_Exposure_t &arExp,
                                                   OV9716_ExposureType_t aExpT)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureDigitalGainGet(arExp, aExpT);
}

//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureAnalogConversionGainGet
                                                   (OV9716_Exposure_t &arExp)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureAnalogConversionGainGet(arExp);
}

//***************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ExposureAnalogConversionGainSet
                                                   (OV9716_Exposure_t &arExp)
{
  arExp.mClient = spClients[arExp.mCsiIdx].mpCameras[(MAXIM_CHNL)MAXIM_CHNL_0];

  return OV9716_ExposureAnalogConversionGainSet(arExp);
}

// **************************************************************************
CAM_LIB_RESULT MAXIM_OV9716_ChgainSet(CSI_IDX arCsiIdx, uint8_t arCamNum,
                           double  arRGain, double  arGGain, double  arBGain)
{
  CAM_LIB_RESULT lRet  = CAM_LIB_SUCCESS;

  /* for all color compononents */
  for (uint8_t i = 0; i < 3; i++)
  {
    MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE     + i*8,
                   ((uint32_t)arRGain)>>8,
                   (MAXIM_CHNL)arCamNum);
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 1 + i*8,
                   ((uint32_t)arRGain)&0xff,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 2 + i*8,
                   ((uint32_t)arGGain)>>8,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 3 + i*8,
                   ((uint32_t)arGGain)&0xff,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 4 + i*8,
                   ((uint32_t)arGGain)>>8,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 5 + i*8,
                   ((uint32_t)arGGain)&0xff,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 6 + i*8,
                   ((uint32_t)arBGain)>>8,
                   (MAXIM_CHNL)arCamNum);
    }
    if(CAM_LIB_SUCCESS == lRet)
    {
      MAXIM_CAM_RegWrite(arCsiIdx,OV9716_GRP_CHGAIN_BASE + 7 + i*8,
                   ((uint32_t)arBGain)&0xff,
                   (MAXIM_CHNL)arCamNum);
    }
    msleep(2);
  }
  return lRet;
}
//***************************************************************************


#ifdef msleep_bck
  #undef msleep
  #define msleep msleep_bck
#endif

/*EOF*/
