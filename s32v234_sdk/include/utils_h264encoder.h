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

/*!*********************************************************************************
*  \file
*  \brief h264 encoder
***********************************************************************************/

#ifndef H264ENCODER_H
#define H264ENCODER_H

#include "s32vs234.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>    /* open */
#include <unistd.h>   /* exit */
#include <sys/mman.h>   /* mmap */
#include <stdint.h>
#include <sys/types.h>
#include <sys/ioctl.h>  /* ioctl */
#include "isp_h264enc.h"
#include "vdb_log.h"
#include "oal.h"
#include <thread>

#include "common_time_measure.h"
#include "isp_hw.h"
#include "semaphore_std.hpp"

//*****************************************************************************
// consts
//*****************************************************************************

#define REG_CNT 38

//*****************************************************************************
// types
//*****************************************************************************


enum H264ENC_InputFormat {RGB , YUV420p, YUV420sp};
/*============================================================================
* @brief         EventHandlerClass
* @details       EventHandlerClass
*
* @return
*/
class EventHandlerClass : H264ENC_EventHandlerClass
{
  uint32_t mVal;
  public:
    EventHandlerClass():mVal(0){};

    void H264EncEventHandler(uint32_t )
    {
      mVal = 3;
      VDB_LOG_NOTE("User defined class called to handle event number \n");
    }; // SeqEventHandler(uint32_t aEventType)

    uint32_t ValGet() {return mVal;};
};

/*============================================================================
* @brief         H264Encoder class
* @details       H264Encoder class
*
* @return
*/
class H264Encoder
{
  public:

    /// OnInitListener
    // typedef EncoderDone::slot_type EncoderDoneListener;

    /*============================================================================
    * @brief         H264Encoder setOnEncoderDoneListener
    * @details       H264Encoder setOnEncoderDoneListener
    *
    * @return
    */
    void setOnEncoderDoneListener(std::function< void(unsigned char *, int, long) > listener);

    /*============================================================================
    * @brief         H264Encoder getInstance
    * @details       H264Encoder getInstance
    *
    * @return        H264Encoder*
    */
    static H264Encoder* getInstance();

    /*============================================================================
    * @brief         H264Encoder H264Encoder_setup
    * @details       H264Encoder_setup set the with, height and line_number for
    *                video or image encoder
    *
    * @return
    */
    static void H264Encoder_setup(int width, int height, int lines_number);
    static void H264Encoder_setup(int width, int height, int lines_number, enum H264ENC_InputFormat input_format);
    /*============================================================================
    * @brief         H264Encoder Destructor
    * @details       H264Encoder Destructor
    *
    * @return
    */
    /**
     * Destructor - frees resources
     */
    ~H264Encoder();

    /*============================================================================
    * @brief         H264Encoder Initialize
    * @details       H264Encoder Initialize allocate and set h264enc before encoder
    *                for each video or image
    *
    * @return        bool
    */
    bool Initialize();

    /*============================================================================
    * @brief         H264Encoder EncoderFrame
    * @details       H264Encoder EncoderFrame
    *
    * @return
    */
    void EncoderFrame(unsigned char* pDst, unsigned char* pDataLumi, unsigned char* pDataChroma, int stride, bool wait);
    void EncoderFrame(unsigned char* pDst, unsigned char* pFrame, bool wait);

    /*============================================================================
    * @brief         H264Encoder EncoderLastFrame
    * @details       H264Encoder EncoderLastFrame
    *
    * @return        int
    */
    int EncoderLastFrame(unsigned char* pDst);

    /*============================================================================
    * @brief         H264Encoder Close
    * @details       H264Encoder Close deallocate and reset h264enc device
    *
    * @return
    */
    void Close();

    // void WaitUntilEncodingDone();

    int encoded_data_length;

  private:
    /*============================================================================
    * @brief         H264Encoder Constructor
    * @details       H264Encoder Constructor
    *
    * @return
    */
    H264Encoder();

    /*============================================================================
    * @brief         H264Encoder EncodingConfigure
    * @details       EncodingConfigure function setup h264encoder registers
    *
    * @return
    */
    void EncodingConfigure();

    /*============================================================================
    * @brief         H264Encoder DataSave
    * @details       H264Encoder DataSave
    *
    * @return       H264ENC_LIB_RESULT
    */
    H264ENC_LIB_RESULT DataSave(const char* apcFile, uint8_t* apData, uint32_t aSize);

    /*============================================================================
    * @brief         H264Encoder DataTxtSave
    * @details       H264Encoder DataTxtSave
    *
    * @return       H264ENC_LIB_RESULT
    */
    H264ENC_LIB_RESULT DataTxtSave(const char* apcFile, uint8_t* apData, uint32_t aSize);

    /*============================================================================
    * @brief         H264Encoder DataLoad
    * @details       H264Encoder DataLoad
    *
    * @return       H264ENC_LIB_RESULT
    */
    H264ENC_LIB_RESULT DataLoad(const char* apcFile, uint8_t* &aprData, uint32_t &aSize);

    /*============================================================================
    * @brief         H264Encoder DdrBuffersAlloc
    * @details       H264Encoder DdrBuffersAlloc
    *
    * @return       int32_t
    */
    int32_t DdrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical, size_t aMemSize, uint32_t aBufferCnt);

    /*============================================================================
    * @brief         H264Encoder DdrBuffersFree
    * @details       H264Encoder DdrBuffersFree
    *
    * @return
    */
    void DdrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical, uint32_t aBufferCnt);

    /*============================================================================
    * @brief         H264EncRegPrint
    * @details       H264EncRegPrint
    *
    * @return
    */
    void H264EncRegPrint(void);

    /*============================================================================
    * @brief         H264Encoder run_thread
    * @details       H264Encoder run_thread
    *
    * @return
    */
    void run_thread();


    std::function< void(unsigned char *, int, long) > onEncode;

    static int width_;

    static int height_;

    static int lines_number_;

    EventHandlerClass lHandler;

    uint8_t *lpSramBuff;
    void    *lpSramBuffHandle;

    volatile unsigned char memSetValue = 0; //to achieve non-optimized SRAM memset. Needs 64bit approach, but in Optimized version compiler replaced it with LIBC memset.

    H264ENC_Status_t lEncStatus;

    unsigned long long segment_data_volume;

    unsigned long lastAddress;

    int m_IMAGE_WIDTH;
    int m_IMAGE_HEIGHT;
    int m_LINES_NUMBER;

    int m_IN_BUFF_OFF;     //Y channel input buffer
    int m_IN_BUFF_SIZE;

    int m_IN_BUFF_U_OFF;   //U channel input buffer
    int m_IN_BUFF_U_SIZE;

    int m_IN_BUFF_V_OFF;  //V channel input buffer
    int m_IN_BUFF_V_SIZE;

    int m_OUT_BUFF_OFF;  //output buffer
    int m_OUT_BUFF_ADDR;
    int m_OUT_BUFF_SIZE;

    /// Running Thread
    std::thread run_t_;

    /// Semaphore for encoding thread
    //boost::interprocess::interprocess_semaphore start_sem;
    semaphore start_sem;

    //boost::mutex io_mutex;
    std::mutex io_mutex;

    /// Value that kills thread
    bool can_work;

    bool isWorking;

    //boost::condition_variable condition;
    std::condition_variable condition;

    unsigned char* global_pDst;
    unsigned char* global_pFrame;
    unsigned char* global_pFrameWait;
};

//*****************************************************************************
// Struct
//*****************************************************************************

//*****************************************************************************
// local functions
//*****************************************************************************

/*============================================================================
* @brief         EventHandler
* @details       EventHandler
*
* @return
*/
static void EventHandler(uint32_t aEventType);

/*============================================================================
* @brief         HwMap
* @details       HwMap
*
* @return       H264ENC_LIB_RESULT
*/
H264ENC_LIB_RESULT HwMap(void);

/*============================================================================
* @brief         HwUnmap
* @details       HwUnmap
*
* @return       H264ENC_LIB_RESULT
*/
H264ENC_LIB_RESULT HwUnmap(void);

/*============================================================================
* @brief         MemMap
* @details       MemMap
*
* @return       uint8_t*
*/
uint8_t* MemMap(uint32_t aSize, uint32_t aOffset);

/*============================================================================
* @brief         MemUnmap
* @details       MemUnmap
*
* @return
*/
void MemUnmap(uint32_t aSize, uint8_t *apMem);

/*============================================================================
* @brief         MemPrint
* @details       MemPrint
*
* @return
*/
void MemPrint(uint8_t *apMem, uint32_t aSize, const char* apcCaption);

/*============================================================================
* @brief         MemPrint
* @details       MemPrint
*
* @return
*/
void MemPrint(uint8_t *apMem, uint32_t aSize);

/*============================================================================
* @brief         H264Encoder H264EncRegTxtSave
* @details       H264Encoder H264EncRegTxtSave
*
* @return       H264ENC_LIB_RESULT
*/
H264ENC_LIB_RESULT H264EncRegTxtSave(const char* apcFile);

#endif /* H264ENCODER_H */
