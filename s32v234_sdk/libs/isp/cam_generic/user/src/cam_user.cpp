/*****************************************************************************
 *
 * Freescale Confidential Proprietary
 *
 * Copyright (c) 2015-16 Freescale Semiconductor;
 * Copyright (c) 2016-18 NXP;
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
 * \file     cam_user.cpp
 * \brief    User-space library calls to access Generic camera driver
 *           functionality.
 * \author   Tomas Babinec
 * \version  0.1
 * \date     13-June-2016
 * \note
 ****************************************************************************/
#include "s32vs234.h"
#include <stdint.h>
#include <cstring>

#ifndef __STANDALONE__
  #include <fcntl.h>		// open
  #include <unistd.h>		// exit
  #include "cam_lldcmd.h"
  #include <sys/mman.h>	// mmap
#else
  #include "cam_func.h"
#endif // else from #ifndef __STANDALONE__

#define VDB_LOG_NOTE_OFF

#include "oal.h"

#include "isp_cam_generic.h"
#include "vdb_log.h"
#include "isp_hw.h"
#include "ov10640_types.h"
#include "sony_types.h"
#include "ov10640_types.h"
#include "ov10635_types.h"
#include "ov9716_types.h"
#include "tiub964_types.h"
#include "maxim_types.h"

//****************************************************************************
// consts
//****************************************************************************


//****************************************************************************
// types
//****************************************************************************


//****************************************************************************
// global variables
//****************************************************************************

//*****************************************************************************
// local data
//*****************************************************************************

int32_t gCamFD = -1;    ///< cam driver file descriptor

//****************************************************************************
// local function declarations
//****************************************************************************
/****************************************************************************/
/** Reset CSI camera by setting reset pin
 *
 * \param aInterfaceType[in] camera interface
 * \param aCamIdx[in] index of camera
 * \param aSensorType[in] type of camera
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
static CAM_LIB_RESULT CAM_Reset(const CAM_INF_TYPE_t aInterfaceType,
                         const uint8_t aCamIdx,
                         const CAM_SENSOR_TYPE_t aSensorType);

/****************************************************************************/
/** Reset camera by setting reset pin
 *
 * \param aInterfaceType[in] camera interface
 * \param aCamIdx[in] the index of camera
 * \param aSensorType[in] type of camera
 * \param aPowerState[in] the power state(up/down)
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
static CAM_LIB_RESULT CAM_PowerControl(const CAM_INF_TYPE_t aInterfaceType,
                                const uint8_t aCamIdx,
                                const CAM_SENSOR_TYPE_t aSensorType,
                                const CAM_POWER_STATE_t aPowerState);
//*****************************************************************************
// functions definitions
//*****************************************************************************

CAM_LIB_RESULT CAM_Open()
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

#ifndef __STANDALONE__
  if(gCamFD == -1)
  {
    gCamFD = open("/dev/cam_generic", O_RDWR);
    if (gCamFD == -1)
    {
      VDB_LOG_ERROR("Can't open device file: %s\n", CAM_DEVICE_NAME);
      lRet = CAM_LIB_FAILURE;
    } // if open failed
  } // if not opened yet
  else
  {
    VDB_LOG_WARNING("Device file: %s opened already.\n", CAM_DEVICE_NAME);
  } // else from if not opened yet
#else // from ifndef __STANDALONE__
  if(CAM_DRV_Setup() != CAM_DRV_SUCCESS)
  {
    lRet = CAM_LIB_FAILURE;
  } // if Setup failed
  else
  {
    gCamFD = 0;
    lRet   = CAM_LIB_SUCCESS;
  } // else from if setup failed
#endif // #ifndef __STANDALONE__

  return lRet;
} // CAM_Reserve()

//*****************************************************************************

CAM_LIB_RESULT CAM_Close()
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;
#ifndef __STANDALONE__
  if(gCamFD != -1)
  {
    if (close(gCamFD) != 0)
    {
      VDB_LOG_ERROR("Failed to close device file %s.\n", CAM_DEVICE_NAME);
      lRet = CAM_LIB_FAILURE;
    } // if open failed
    else
    {
      gCamFD = -1;
    } // else if open failed
  } // not openned yet
  else
  {
    VDB_LOG_WARNING(" already closed.\n");
  } // else from if not openned yet
#else
  CAM_DRV_Close();
  gCamFD = -1;
#endif // else from #ifndef __STANDALONE__
  return lRet;
} // CAM_Release()

//*****************************************************************************
// Camera client management
//*****************************************************************************

CamI2cClient_t CAM_ClientAdd(CamI2cCfg_t &arCfg)
{
  CamI2cClient_t lRet = NULL;

#ifndef __STANDALONE__
  if(gCamFD == -1)
  {
    VDB_LOG_ERROR("CAM device file not opened.\n");
  } // if CAM not opened
  else
  {
    CamI2cCfg_t lCfgCpy = arCfg;
    OAL_LldCmd_t lCmd = {LLDCMD_CAM_CLI_ADD, (uintptr_t)&lCfgCpy};
    if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
    {
      VDB_LOG_ERROR("Adding new camera client failed\n");
    } // if lldcmd returned failure
    else
    {
      lRet = *((CamI2cClient_t*)&lCfgCpy);
    } // else from if lldcmd returned failure
  } // else from if CAM not opened
#else // #ifndef __STANDALONE__
  lRet = CAM_DRV_ClientAdd(&arCfg);
#endif // else #ifndef __STANDALONE__

  return lRet;
} // CAM_ClientAdd()

//*****************************************************************************

CAM_LIB_RESULT CAM_ClientRem(CamI2cClient_t aHandle)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  //*** check if valid handle ***
  if(aHandle != NULL)
  {
#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_CLI_REM, (uintptr_t)aHandle};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("Removing camera client failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
    if(CAM_DRV_ClientRem(aHandle) != CAM_DRV_SUCCESS)
    {
      VDB_LOG_ERROR("Removing camera client failed\n");
      lRet = CAM_LIB_FAILURE;
    } // if client removal failure
#endif // else #ifndef __STANDALONE__
  }
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached

  return lRet;
} // CAM_ClientRem()

//*****************************************************************************
// I2c Read/Write
//*****************************************************************************

CAM_LIB_RESULT CAM_ReadBatch(I2cXfrReadBatch_t &arBatch)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  //*** check if valid handle ***
  if(arBatch.mDestDev != NULL)
  {
#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_REG_RDB, (uintptr_t)&arBatch};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("I2c read batch failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__

    I2cXfrSingle_t lXfr;
    lXfr.mDestDev = (CamGeneric_t*)arBatch.mDestDev;

    for(uint32_t i = 0; i < arBatch.mCmdCnt; i++)
    {
      lXfr.mByteCnt = arBatch.mpByteCnt[i];
      lXfr.mData = arBatch.mpData[i];

      if(CAM_DRV_Read(&lXfr) != CAM_DRV_SUCCESS)
      {
        VDB_LOG_ERROR("Failed to read from I2C.\n");
        lRet = CAM_LIB_FAILURE;
      } // if failed to read the data
      else
      {
          if(lXfr.mByteCnt <= CAM_I2C_SINGLE_DATA_SZ)
          {
             //write data back
             arBatch.mpData[i] = lXfr.mData;
          } // if data fit into structure
      }

    } // for all commands in this part of the batch
#endif // else #ifndef __STANDALONE__
  }
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached

  return lRet;
} // CAM_ReadBatch()

//*****************************************************************************

CAM_LIB_RESULT CAM_WriteBatch(I2cXfrWriteBatch &arBatch)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  //*** check if valid handle ***
  if(arBatch.mDestDev != NULL)
  {
#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_REG_WRB, (uintptr_t)&arBatch};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("I2c write batch failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
    CamGeneric_t *lpCam     = (CamGeneric_t*)arBatch.mDestDev;

    for(uint32_t i = 0; i < arBatch.mCmdCnt; i++)
    {
      if(CAM_DRV_CmdWrite(lpCam,
                    &(arBatch.mpCmdArr[i])) != CAM_DRV_SUCCESS)
      {
        VDB_LOG_ERROR("Failed to write to I2C command.\n");
        lRet = CAM_LIB_FAILURE;
      } // if failed to write the data
    } // for all commands in this part of the batch
#endif // else #ifndef __STANDALONE__
  }
  else
  {
    VDB_LOG_ERROR("No such client connected.\n");
    lRet = CAM_LIB_FAILURE;
  } // else from if camera client attached

  return lRet;
} // CAM_WriteBatch()


//*****************************************************************************
CAM_LIB_RESULT CAM_ReadSingle(I2cXfrSingle_t &arXfr)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  //*** check if valid handle ***
#ifndef __STANDALONE__
  if(gCamFD == -1)
  {
    VDB_LOG_ERROR("CAM device file not opened.\n");
    lRet = CAM_LIB_FAILURE;
  } // if CAM not opened
  else
  {
    OAL_LldCmd_t lCmd = {LLDCMD_CAM_REG_RDS, (uintptr_t)&arXfr};
	if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
	{
	  VDB_LOG_ERROR("I2c read single failed\n");
	  lRet = CAM_LIB_FAILURE;
	} // if lldcmd returned failure
	else
	{
	  lRet = CAM_LIB_SUCCESS;
	} // else from if lldcmd returned failure
  } // else from if CAM not opened
#else // #ifndef __STANDALONE__
  if(CAM_DRV_Read(&arXfr) != CAM_DRV_SUCCESS)
  {
    lRet = CAM_LIB_FAILURE;
  } // if read failed
#endif // else #ifndef __STANDALONE__

  return lRet;
} // CAM_ReadSingle()


//*****************************************************************************

CAM_LIB_RESULT CAM_ReadSingle(const CamI2cClient_t aHandle,
                              const uint8_t aRegAddr,
                              uint8_t  &arVal)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  VDB_LOG_NOTE("Read: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(arVal), aRegAddr, arVal);
  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;

  uint8_t *lpData = (uint8_t*)(&lXfr.mData);

  lXfr.mByteCnt = 2;
  lpData[0] = aRegAddr;

  lRet = CAM_ReadSingle(lXfr);

  if(lRet == CAM_LIB_SUCCESS)
  {
    arVal = lpData[1];
  } // if(lRet == CAM_LIB_SUCCESS)

  return lRet;
} //CAM_ReadSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_ReadSingle(const CamI2cClient_t aHandle,
                              const uint16_t aRegAddr,
                              uint8_t  &arVal)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  VDB_LOG_NOTE("Read: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(arVal), aRegAddr, arVal);
  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;

  uint8_t *lpData = (uint8_t*)(&lXfr.mData);

  lXfr.mByteCnt = 3;

  lpData[0] = (uint8_t)((aRegAddr & (uint16_t)0xFF00U) >> 8);
  lpData[1] = (uint8_t)(aRegAddr & (uint16_t)0x00FFU);

  lRet = CAM_ReadSingle(lXfr);

  if(lRet == CAM_LIB_SUCCESS)
  {
    arVal = lpData[2];
  } // if(lRet == CAM_LIB_SUCCESS)

  return lRet;
} //CAM_ReadSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_ReadSingle(const CamI2cClient_t aHandle,
                              const uint16_t aRegAddr,
                              uint16_t  &arVal)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  VDB_LOG_NOTE("Read: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(arVal), aRegAddr, arVal);
  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;

  uint8_t *lpData = (uint8_t*)(&lXfr.mData);

  lXfr.mByteCnt = 4;

  lpData[0] = (uint8_t)((aRegAddr & (uint16_t)0xFF00U) >> 8);
  lpData[1] = (uint8_t)(aRegAddr & (uint16_t)0x00FFU);

  lRet = CAM_ReadSingle(lXfr);

  if(lRet == CAM_LIB_SUCCESS)
  {
    arVal = (lpData[2] << 8) | lpData[3];
  } // if(lRet == CAM_LIB_SUCCESS)

  return lRet;
} //CAM_ReadSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_WriteSingle(const CamI2cClient_t aHandle,
                               const uint8_t       aRegAddr,
                               const uint8_t        aVal
                              )
{
  VDB_LOG_NOTE("Write: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(aVal), aRegAddr, aVal);

  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;
  uint8_t *lpData = (uint8_t*)(&lXfr.mData);
  lXfr.mByteCnt = 2;

  lpData[0] = aRegAddr;
  lpData[1] = aVal;

  return CAM_WriteSingle(lXfr);
} // CAM_WriteSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_WriteSingle(const CamI2cClient_t aHandle,
                               const uint16_t       aRegAddr,
                               const uint8_t        aVal
                              )
{
  VDB_LOG_NOTE("Write: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(aVal), aRegAddr, aVal);

  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;
  uint8_t *lpData = (uint8_t*)(&lXfr.mData);
  lXfr.mByteCnt = 3;

  lpData[0] = (uint8_t)((aRegAddr & (uint16_t)0xFF00U) >> 8);
  lpData[1] = (uint8_t)(aRegAddr & (uint16_t)0x00FFU);
  lpData[2] = aVal;

  return CAM_WriteSingle(lXfr);
} // CAM_WriteSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_WriteSingle(const CamI2cClient_t aHandle,
                               const uint16_t       aRegAddr,
                               const uint16_t        aVal
                              )
{
  VDB_LOG_NOTE("Write: %d-%d %x:%x.\n", sizeof(aRegAddr), sizeof(aVal), aRegAddr, aVal);

  I2cXfrSingle_t lXfr;
  memset(&lXfr, 0, sizeof(I2cXfrSingle_t));
  lXfr.mDestDev = aHandle;
  uint8_t *lpData = (uint8_t*)(&lXfr.mData);
  lXfr.mByteCnt = 4;

  lpData[0] = (uint8_t)((aRegAddr & (uint16_t)0xFF00U) >> 8);
  lpData[1] = (uint8_t)(aRegAddr & (uint16_t)0x00FFU);
  lpData[2] = (uint8_t)((aVal & (uint16_t)0xFF00U) >> 8);
  lpData[3] = (uint8_t)(aVal & (uint16_t)0x00FFU);

  return CAM_WriteSingle(lXfr);
} // CAM_WriteSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_WriteSingle(const I2cXfrSingle_t &arcXfr)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

#ifndef __STANDALONE__
  if(gCamFD == -1)
  {
    VDB_LOG_ERROR("CAM device file not opened.\n");
    lRet = CAM_LIB_FAILURE;
  } // if CAM not opened
  else
  {
    OAL_LldCmd_t lCmd = {LLDCMD_CAM_REG_WRS, (uintptr_t)&arcXfr};
    if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
    {
      VDB_LOG_ERROR("I2c write single failed\n");
      lRet = CAM_LIB_FAILURE;
    } // if lldcmd returned failure
  } // else from if CAM not opened
#else // #ifndef __STANDALONE__
  if(CAM_DRV_Write(&arcXfr) != CAM_DRV_SUCCESS)
  {
    lRet = CAM_LIB_FAILURE;
  } // if failed to read the data
#endif // else #ifndef __STANDALONE__

  return lRet;
} // CAM_WriteSingle()

//*****************************************************************************

CAM_LIB_RESULT CAM_I2C_SetRepeat(const uint32_t aI2cRepeat)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

#ifndef __STANDALONE__
    uint32_t lI2cRepeat = aI2cRepeat;

    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_I2C_REPEAT_SET, (uintptr_t)&lI2cRepeat};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("I2c set up the repeat number failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
   // Do nothing
#endif // else #ifndef __STANDALONE__

  return lRet;
} //CAM_I2C_RepeatSet()

//*****************************************************************************

CAM_LIB_RESULT CAM_I2C_GetRepeat(uint32_t &aI2cRepeat)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

#ifndef __STANDALONE__

    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_I2C_REPEAT_GET, (uintptr_t)&aI2cRepeat};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("I2c get the current repeat number failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
   // Do nothing
#endif // else #ifndef __STANDALONE__

  return lRet;
} //CAM_I2C_RepeatGet()

CAM_LIB_RESULT CAM_ControlGpio(const GpioControlCmd_t aGpioCmd)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      OAL_LldCmd_t lCmd = {LLDCMD_CAM_GPIO_CONTROL, (uintptr_t)&aGpioCmd};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("Gpio control pin failed\n");
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
   // Do nothing
#endif // else #ifndef __STANDALONE__

  return lRet;

}

/****************************************************************************/
/** Reset VIU camera by setting reset pin
 *
 * \param aViuIdx[in] index of VIU camera
 * \param aSensorType[in] type of VIU camera
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_Reset(const VIU_IDX aViuIdx, const CAM_SENSOR_TYPE_t aSensorType)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = CAM_Reset(CAM_INF_TYPE_VIU, (uint8_t)aViuIdx, aSensorType);

  return lRet;
}

/****************************************************************************/
/** Reset CSI camera by setting reset pin
 *
 * \param aCsiIdx[in] index of CSI camera
 * \param aSensorType[in] type of CSI camera
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_Reset(const CSI_IDX aCsiIdx, const CAM_SENSOR_TYPE_t aSensorType)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = CAM_Reset(CAM_INF_TYPE_CSI, (uint8_t)aCsiIdx, aSensorType);

  return lRet;
}

/****************************************************************************/
/** Reset CSI camera by setting reset pin
 *
 * \param aInterfaceType[in] camera interface
 * \param aCamIdx[in] index of camera
 * \param aSensorType[in] type of camera
 *
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_Reset(const CAM_INF_TYPE_t aInterfaceType,
                         const uint8_t aCamIdx,
                         const CAM_SENSOR_TYPE_t aSensorType)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamResetPinCmd_t aCmd;
#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      uint32_t lResetPolarity;
      uint32_t lDelayTime;
      switch (aSensorType)
      {
        case SENSOR_TYPE_OV10640:
        {
          lResetPolarity = OV10640_RESET_POLARITY;
          lDelayTime = OV10640_RESET_DELAY;
        } // ov10640 camera
        break;

        case SENSOR_TYPE_OV10635:
        {
          lResetPolarity = OV10635_RESET_POLARITY;
          lDelayTime = OV10635_RESET_DELAY;
        } // ov10635 camera
        break;

        case SENSOR_TYPE_OV9716:
        {
          lResetPolarity = OV9716_RESET_POLARITY;
          lDelayTime = OV9716_RESET_DELAY;
        } // ov9716 camera
        break;

        case SENSOR_TYPE_SONY:
        {
          lResetPolarity = SONY_RESET_POLARITY;
          lDelayTime = SONY_RESET_DELAY;
        } // sony camera
        break;

        case SENSOR_TYPE_TIUB964:
        {
          lResetPolarity = TIUB964_RESET_POLARITY;
          lDelayTime = TIUB964_RESET_DELAY;
        } // tiub964 camera
        break;

        case SENSOR_TYPE_MAXIM_9286:
        {
          lResetPolarity = MAXIM_9286_RESET_POLARITY;
          lDelayTime = MAXIM_9286_RESET_DELAY;
        } // maxim 9286
        break;

        default:
        {
          lResetPolarity = CAM_RESET_POLARITY_DEFAULT;
          lDelayTime = CAM_RESET_DELAY_DEFAULT;
        }
        break;
      }

      aCmd.mCamType = aInterfaceType;
      aCmd.mIndex = (uint8_t)aCamIdx;
      aCmd.mResetPolarity = lResetPolarity;
      aCmd.mResetDelayTime = lDelayTime;

      OAL_LldCmd_t lCmd = {LLDCMD_CAM_RESET, (uintptr_t)&aCmd};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("Reset cam fail with type: %d, index: %d\n",
                      aInterfaceType, aCamIdx);
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
  VDB_LOG_WARNING("This function is not supported on stand alone");
#endif // else #ifndef __STANDALONE__

  return lRet;
}

/****************************************************************************/
/** Control VIU camera power by setting associated pin
 *
 * \param aViuIdx[in] the index of VIU camera
 * \param aSensorType[in] type of VIU camera
 * \param aPowerState[in] the power state(up/down)
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_PowerControl(const VIU_IDX aViuIdx, const CAM_SENSOR_TYPE_t aSensorType, const CAM_POWER_STATE_t aPowerState)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = CAM_PowerControl(CAM_INF_TYPE_VIU, (uint8_t)aViuIdx, aSensorType, aPowerState);

  return lRet;
}

/****************************************************************************/
/** Control CSI camera power by setting associated pin
 *
 * \param aCsiIdx[in] the index of CSI camera
 * \param aSensorType[in] type of CSI camera
 * \param aPowerState[in] the power state(up/down)
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_PowerControl(const CSI_IDX aCsiIdx, const CAM_SENSOR_TYPE_t aSensorType, const CAM_POWER_STATE_t aPowerState)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  lRet = CAM_PowerControl(CAM_INF_TYPE_CSI, (uint8_t)aCsiIdx, aSensorType, aPowerState);

  return lRet;
}
/****************************************************************************/
/** Reset camera by setting reset pin
 *
 * \param aInterfaceType[in] camera interface
 * \param aCamIdx[in] the index of camera
 * \param aSensorType[in] type of camera
 * \param aPowerState[in] the power state(up/down)
 * \return CAM_LIB_SUCCESS if all ok
 *         CAM_LIB_FAILURE otherwise.
 ****************************************************************************/
CAM_LIB_RESULT CAM_PowerControl(const CAM_INF_TYPE_t aInterfaceType,
                                const uint8_t aCamIdx,
                                const CAM_SENSOR_TYPE_t aSensorType,
                                const CAM_POWER_STATE_t aPowerState)
{
  CAM_LIB_RESULT lRet = CAM_LIB_SUCCESS;

  CamPowerControlCmd_t aCmd;
#ifndef __STANDALONE__
    if(gCamFD == -1)
    {
      VDB_LOG_ERROR("CAM device file not opened.\n");
      lRet = CAM_LIB_FAILURE;
    } // if CAM not opened
    else
    {
      uint32_t lPowerPolarity;
      uint32_t lPowerDelayTime;
      uint32_t lResetPolarity;
      uint32_t lResetDelayTime;
      switch (aSensorType)
      {
        case SENSOR_TYPE_OV10640:
        {
          lPowerPolarity = OV10640_PWDN_POLARITY;
          lResetPolarity = OV10640_RESET_POLARITY;
          lPowerDelayTime = OV10640_PWDN_DELAY;
          lResetDelayTime = OV10640_RESET_DELAY;
        } // ov10640 camera
        break;

        case SENSOR_TYPE_OV10635:
        {
          lPowerPolarity = OV10635_PWDN_POLARITY;
          lResetPolarity = OV10635_RESET_POLARITY;
          lPowerDelayTime = OV10635_PWDN_DELAY;
          lResetDelayTime = OV10635_RESET_DELAY;
        } // ov10635 camera
        break;

        case SENSOR_TYPE_OV9716:
        {
          lPowerPolarity = OV9716_PWDN_POLARITY;
          lResetPolarity = OV9716_RESET_POLARITY;
          lPowerDelayTime = OV9716_PWDN_DELAY;
          lResetDelayTime = OV9716_RESET_DELAY;
        } // ov9716 camera
        break;

        case SENSOR_TYPE_SONY:
        {
          lPowerPolarity = SONY_PWDN_POLARITY;
          lResetPolarity = SONY_RESET_POLARITY;
          lPowerDelayTime = SONY_PWDN_DELAY;
          lResetDelayTime = SONY_RESET_DELAY;
        } // sony camera
        break;

        case SENSOR_TYPE_TIUB964:
        {
          lPowerPolarity = TIUB964_PWDN_POLARITY;
          lResetPolarity = TIUB964_RESET_POLARITY;
          lPowerDelayTime = TIUB964_PWDN_DELAY;
          lResetDelayTime = TIUB964_RESET_DELAY;
        } // tiub964 camera
        break;

        case SENSOR_TYPE_MAXIM_9286:
        {
          lPowerPolarity = MAXIM_9286_PWDN_POLARITY;
          lResetPolarity = MAXIM_9286_RESET_POLARITY;
          lPowerDelayTime = MAXIM_9286_PWDN_DELAY;
          lResetDelayTime = MAXIM_9286_RESET_DELAY;
        } // maxim 9286
        break;

        default:
        {
          lPowerPolarity = CAM_PWDN_POLARITY_DEFAULT;
          lResetPolarity = CAM_RESET_POLARITY_DEFAULT;
          lPowerDelayTime = CAM_PWDN_DELAY_DEFAULT;
          lResetDelayTime = CAM_RESET_DELAY_DEFAULT;
        }
        break;
      }

      aCmd.mCamType = aInterfaceType;
      aCmd.mIndex = aCamIdx;
      aCmd.mVal = aPowerState;
      aCmd.mPowerPolarity = lPowerPolarity;
      aCmd.mPowerDelayTime = lPowerDelayTime;
      aCmd.mResetPolarity = lResetPolarity;
      aCmd.mResetDelayTime = lResetDelayTime;

      OAL_LldCmd_t lCmd = {LLDCMD_CAM_POWER_CONTROL, (uintptr_t)&aCmd};
      if (OAL_LldCall((OAL_LldHandle_t)gCamFD, &lCmd) != OAL_SUCCESS)
      {
        VDB_LOG_ERROR("Config cam power fail with type: %d, index: %d\n",
                      aCamIdx);
        lRet = CAM_LIB_FAILURE;
      } // if lldcmd returned failure
    } // else from if CAM not opened
#else // #ifndef __STANDALONE__
  VDB_LOG_WARNING("This function is not supported on stand alone");
#endif // else #ifndef __STANDALONE__

  return lRet;
}

//*****************************************************************************

/*EOF*/