/*****************************************************************************
 *
 * Freescale Confidential Proprietary
 *
 * Copyright (c) 2016 Freescale Semiconductor;
 * Copyright (c) 2016-2018 NXP
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


/*==============================================================================
*                                 INCLUDE FILES
==============================================================================*/

#include <cstdint>
#include <string>

#include <fcntl.h>      // open
#include <unistd.h>     // exit
#include <sys/mman.h>   // mmap

#include "s32vs234.h"
#include "oal.h"
#include "isp_h264dec.h"
#include "vdb_log.h"
#include "isp_hw.h"

/*==============================================================================
*                                LOCAL MACROS
==============================================================================*/

#define VDB_LOG_NOTE_OFF

/*==============================================================================
*                               GLOBAL CONSTANTS
===============================================================================*/

std::string const H264DecDeviceName ("h264dcd");
std::string const H264DecDeviceFile ("/dev/h264dcd");

/*==============================================================================
*                               GLOBAL VARIABLES
==============================================================================*/

static int32_t sH264DecFd = -1;    ///< H264DCD driver file descriptor


/*==============================================================================
*                               GLOBAL FUNCTIONS
==============================================================================*/

/*============================================================================*/
/**
* @brief          Open H264DCD device
* @details        The function opens the H264DCD device to enable operations
*
* @param[in]      void
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_Open_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_Open(void)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        sH264DecFd = open(H264DecDeviceFile.c_str(), O_RDWR);
        if (sH264DecFd == -1)
        {
            VDB_LOG_ERROR("Can't open device file: %s\n", H264DecDeviceName.c_str());
            lRet = H264DEC_LIB_FAILURE;
        } // if open failed
    } // if not opened yet
    else
    {
        VDB_LOG_WARNING("Device file: %s opened already.\n", H264DecDeviceName.c_str());
    } // else from if not opened yet

    return lRet;
} // H264DEC_Open()

/*============================================================================*/
/**
* @brief          Close H264DCD device
* @details        The function closes the H264DCD device
*
* @param[in]      void
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_Close_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_Close(void)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd != -1)
    {
        if (close(sH264DecFd) != 0)
        {
            VDB_LOG_ERROR("Failed to close device file %s.\n", H264DecDeviceName.c_str());
            lRet = H264DEC_LIB_FAILURE;
        } // if close failed
        else
        {
            sH264DecFd = -1;
        } // else if close failed
    } // not openned yet
    else
    {
        VDB_LOG_WARNING("H264Dec already closed.\n");
    } // else from if not openned yet

    return lRet;
} // H264DEC_Close()

/*============================================================================*/
/**
* @brief          Set the parameters of the data source
* @details        The function sets address and size of the input data packet
*                 for the specified input stream
*
* @param[in]      arcH264DecInConfig   H264DCD_STREAM_DATA_CONFIG structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_InConfig_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_InConfig(
                     const H264DCD_STREAM_DATA_CONFIG &arcH264DecInConfig)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_INPUTSTREAM_CFG, 
          reinterpret_cast<uintptr_t>(&arcH264DecInConfig)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC input data.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_InConfig(const H264DCD_STREAM_DATA_CONFIG &arcH264DecInConfig)

/*============================================================================*/
/**
* @brief          Set the watermark level.
* @details        The function sets the watermark level for
*                 specified FIFO
*
* @param[in]      arcWtmLevel        The watermark level
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_WtmLevelSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_WtmLevelSet(const H264DCD_FIFO_WTM &arcWtmLevel)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_FIFO_WATERMARK_SET, 
          reinterpret_cast<uintptr_t>(&arcWtmLevel)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC watermark level.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_WtmLevelSet(const H264DCD_FIFO_WTM &arcWtmLevel)

/*============================================================================*/
/**
* @brief          Set timeout for H264DCD
* @details        The function sets timeout for decoding operation
*
* @param[in]      acTimeout  Timeout value
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_TimeoutSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_TimeoutSet(const uint16_t acTimeout)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_TIMEOUT_SET, 
          reinterpret_cast<uintptr_t>(&acTimeout)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to set timeout.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd failed
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_TimeoutSet(const uint16_t acTimeout)

/*============================================================================*/
/**
* @brief          Enable timeout for H264DCD
* @details        The function enables timeout for decoding operation
*
* @param[in]      arcToEn         Enable flag
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_TimeoutEnable_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_TimeoutEnable(const H264DCD_TIMEOUT_ENABLE &arcToEn)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_TIMEOUT_ENABLE, 
          reinterpret_cast<uintptr_t>(&arcToEn)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to enable timeout.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd failed
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_TimeoutEnable(const H264DCD_TIMEOUT_ENABLE &arcToEn)

/*============================================================================*/
/**
* @brief          SW reset
* @details        The function sets and resets the SW_RST bit resetting
*                 the Input DMA Wrapper, the Output DMA Wrapper and
*                 the H264 Decoder Core
*
* @param[in]      void
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_SwReset_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_SwReset(void)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_SW_RESET, 
          (uintptr_t)0
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to SW reset the H264DEC.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_SwReset()

/*============================================================================*/
/**
* @brief          Set the parameters for output data
* @details        The function sets for the selected data stream the components
*                 buffer addresses and the number of lines per component used in
*                 the luma output circular component buffer
*
* @param[in]      arcH264DecOutConfig    H264DCD_OUTPUT_BUFFCFG structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_OutConfig_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_OutConfig(const H264DCD_OUTPUT_BUFFCFG &arcH264DecOutConfig)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_OUTSTREAM_CFG, 
          reinterpret_cast<uintptr_t>(&arcH264DecOutConfig)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC output.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_OutConfig(const H264DCD_OUTPUT_BUFFCFG &arcH264DecOutConfig)

/*============================================================================*/
/**
* @brief          Change resampling mode
* @details        The function sets sampling mode according to
*                 outputregsmpl_mode variable
*
* @param[in]      arcOutSampleMode    H264DCD_OUTPUT_SAMPLEMODE structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_OutSampleModeSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_OutSampleModeSet(
                     const H264DCD_OUTPUT_SAMPLEMODE &arcOutSampleMode)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_OUTSAMPLEMODE_SET, 
          reinterpret_cast<uintptr_t>(&arcOutSampleMode)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC output.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd failed
    } // else from if H264Dec not opened

  return lRet;
} // H264DEC_OutSampleModeSet(const H264DCD_OUTPUT_SAMPLEMODE &arcOutSampleMode)

/*============================================================================*/
/**
* @brief          Set colour format
* @details        The function sets the colour format and the component
*                 samples bit width for the selected data stream
*
* @param[in]      arcOutColData   H264DCD_OUTPUT_COLOURDATA structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_OutColorDataSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_OutColorDataSet(
                     const H264DCD_OUTPUT_COLOURDATA &arcOutColData)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_OUTCOLOURDATA_SET, 
          reinterpret_cast<uintptr_t>(&arcOutColData)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC output color data.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd failed
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_OutColorDataSet(const H264DCD_OUTPUT_COLOURDATA arcOutColData)

/*============================================================================*/
/**
* @brief          Set flow mode
* @details        The function sets the flow mode
*
* @param[in]      acDataFlowMode     Mode
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_DataFlowModeSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_DataFlowModeSet(const H264DCD_DATAFLOW_MODE acDataFlowMode)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DATAFLOWMODE_SET, 
          reinterpret_cast<uintptr_t>(&acDataFlowMode)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC data flow mode.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd failed
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_DataFlowModeSet(const H264DCD_DATAFLOW_MODE acDataFlowMode)

/*============================================================================*/
/**
* @brief          Set bankstride
* @details        The function sets the address offset of each
*                 start of macroblock
*
* @param[in]      arcBankStride      Address offset
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_BankStrideSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_BankStrideSet(const H264DCD_OUTPUT_BKSTRIDE &arcBankStride)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_BANKSTRIDE_SET, 
          reinterpret_cast<uintptr_t>(&arcBankStride)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC bankstride.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_BankStrideSet(const H264DCD_OUTPUT_BKSTRIDE &arcBankStride)

/*============================================================================*/
/**
* @brief          Set the coded image size
* @details        The function sets the coded image size in terms of macroblocks
*
* @param[in]      arcFrmSize       H264DCD_INFRAME_SIZE structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_InFrameSizeSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_InFrameSizeSet(const H264DCD_INFRAME_SIZE &arcFrmSize)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DCD_INFRAMESIZE_SET, 
          reinterpret_cast<uintptr_t>(&arcFrmSize)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)  
        {
            VDB_LOG_ERROR("Failed to config H264DEC input frame size.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_InFrameSizeSet(const H264DCD_INFRAME_SIZE &arcFrmSize)

/*============================================================================*/
/**
* @brief          Channel control
* @details        The function controls channel activity
*
* @param[in]      arcChnlCtrl   Channel status
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_ChannelStop_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_ChannelStop(const H264DCD_CH_CONTROL &arcChnlCtrl)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DCD_CH_STOP, 
          reinterpret_cast<uintptr_t>(&arcChnlCtrl)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)  
        {
            VDB_LOG_ERROR("Failed to stop H264DEC channel.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_ChannelStop(const H264DCD_CH_CONTROL &arcChnlCtrl)

/*============================================================================*/
/**
* @brief          Configure decoder
* @details        The function sets the type of the used memory SRAM or DDR
*                 and enable or disable the deblocking filters
*
* @param[in]      arcDeblockCfg       H264DCD_DECOD_CONFIG structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_DeblockSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_DeblockSet(const H264DCD_DECOD_CONFIG &arcDeblockCfg)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DCD_CFG_SET, 
          reinterpret_cast<uintptr_t>(&arcDeblockCfg)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)  
        {
            VDB_LOG_ERROR("Failed to setup deblocking.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

  return lRet;
} // H264DEC_DeblockSet(const H264DCD_DECOD_CONFIG &arcDeblockCfg)

/*============================================================================*/
/**
* @brief          Set threshold level
* @details        The function sets the threshold level
*
* @param[in]      arcThreshLevels      Threshold level
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_ThreshLevelsSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_ThreshLevelsSet(
                     const H264DCD_DECOD_THRLEVELS &arcThreshLevels)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DCD_THRLEVELS_SET, 
          reinterpret_cast<uintptr_t>(&arcThreshLevels)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)  
        {
            VDB_LOG_ERROR("Failed to config H264DEC threshold levels.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_ThreshLevelsSet(const H264DCD_DECOD_THRLEVELS &arcThreshLevels)

/*============================================================================*/
/**
* @brief          Set address for reference data
* @details        The function sets the address of the memory zone where the
*                 decoder reads or stores the reference data for the
*                 selected channel
*
* @param[in]      arcRefMemory     Memory address
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_RefMemorySet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_RefMemorySet(const H264DCD_DECOD_REFMEM &arcRefMemory)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_DCD_REFMEMORY_SET, 
          reinterpret_cast<uintptr_t>(&arcRefMemory)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to config H264DEC reference memory.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_RefMemorySet(const H264DCD_DECOD_REFMEM &arcRefMemory)

/*============================================================================*/
/**
* @brief          Get the status of the FIFO stack
* @details        The function gets the current active location of the
*                 FIFO pointer and the number of pipelined configuration under
*                 process or to be processed
*
* @param[out]     arStatus     H264DCD_STREAM_FIFO_STATUS structure
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_FifoStatusGet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_FifoStatusGet(H264DCD_STREAM_FIFO_STATUS &arStatus)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_FIFO_STATUS_GET, 
          reinterpret_cast<uintptr_t>(&arStatus)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to get fifo status.\n");
            lRet = H264DEC_LIB_FAILURE;
        }
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_FifoStatusGet(H264DCD_STREAM_FIFO_STATUS &arStatus)

/*============================================================================*/
/**
* @brief          Set interrupts
* @details        The function enables or disables the ERR_REQ_EN,
*                 STREAM_TIME_OUT, END_OF_FRAME_x, MBROW_DONE_x,
*                 STRx_FIFO_WATERMARK, DEC_OOS_STRx, FRM_CYCCNT_GRT_THRy_x
*                 interrupts according to the bit mask sent as parameter
*
* @param[in]      acIrqMask   Interrupt mask
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_IrqMaskSet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_IrqMaskSet(const uint32_t acIrqMask)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_IRQ_CONTROL, 
          reinterpret_cast<uintptr_t>(&acIrqMask)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to set IRQ mask.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd returned failure
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_IrqMaskSet(const uint32_t acIrqMask)

/*============================================================================*/
/**
* @brief          Get interrupts
* @details        The function returns which of the interrupts are enabled
*                 at the moment
*
* @param[out]     arIrqMask   Interrupt mask
*
* @return         H264DEC_LIB_RESULT
* @api
* @isr
*
* @pre
* @post
*
* @implements     H264DEC_IrqMaskGet_Activity
*
* @note
*/
H264DEC_LIB_RESULT H264DEC_IrqMaskGet(uint32_t &arIrqMask)
{
    H264DEC_LIB_RESULT lRet = H264DEC_LIB_SUCCESS;

    if(sH264DecFd == -1)
    {
        VDB_LOG_ERROR("H264Dec device file not opened.\n");
        lRet = H264DEC_LIB_FAILURE;
    } // if H264Dec not opened
    else
    {
        OAL_LldCmd_t lCmd = 
        {
          H264DCD_LLDCMD_IRQ_GET, 
          reinterpret_cast<uintptr_t>(&arIrqMask)
        };
        if (OAL_LldCall(static_cast<OAL_LldHandle_t>(sH264DecFd), &lCmd) != OAL_SUCCESS)
        {
            VDB_LOG_ERROR("Failed to get IRQ mask.\n");
            lRet = H264DEC_LIB_FAILURE;
        } // if lldcmd returned failure
    } // else from if H264Dec not opened

    return lRet;
} // H264DEC_IrqMaskGet(uint32_t &arIrqMask)
