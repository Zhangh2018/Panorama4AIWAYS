/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
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
*  \brief ApexCV Resize Library.
***********************************************************************************/

#ifndef APEXCVPRORESIZE_H
#define APEXCVPRORESIZE_H

#include <icp_data.h>
#include <sumat.hpp>
#include <acf_process_apu.h>
#include <apexcv_error_codes.hpp>
#include <apexcv_host_base_class.h>

namespace apexcv{

/*!*********************************************************************************
*  \brief Apex Resize.
*
*  apexcv::Resize is the host-ACF interface for creating, initializing,
*  executing and releasing \ref pagResize "image resize" on Apex.
*  Apex Resize uses memcpy when source image width/height is equal to destination image 
*  width/height. for performance consideration, resized result image buffer needs be 
*  allocated DMA friendly. for destination size of width and height, the dst buffer should be 
*  allocated to a SUMat with size of (width+127)/128*128+32, height+32, and use SUMat ROI 
*  Rect(0, 0, width, height) to specify actual dst image size.   
*   
***********************************************************************************/
class Resize: public ApexcvHostBaseBaseClass
{
public:

/**********************************************************************************
*  Default constructor.
*  Allocate resources for the ACF process 
*  Supported datatypes are: 8bit unsigned integer to 16 bit unsigned integer.
*  All even image dimensions are supported from 64 to 1024 for both input and output.
*  Vertical scale factor - upscaling currently limited to 32x.
*  Default data format is 8bit unsigned.
***********************************************************************************/
   Resize();

/**********************************************************************************
*  Destructor.
*  Release resources for the ACF process.
***********************************************************************************/
   ~Resize();

/*!*********************************************************************************
*  \brief Initialize the resize ACF process. This function will initialize all objects
*  of Resize and it will only be called once.
*
*  \return APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
***********************************************************************************/
   APEXCV_LIB_RESULT Initialize(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                  vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */);

/*!*********************************************************************************
*  \brief Reconnect IO
*  Reconnects the input and output ports to the object. This is how to
*  update the image containers the object uses.
*  This function support full type reconnect meaning that changes in size and type
*  (8 bit, 16 bit) are possible if within allowed range.
*  Note: the object needs to be initialized before calling reconnectIO.
*
*  \return APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
***********************************************************************************/
   APEXCV_LIB_RESULT ReconnectIO(vsdk::SUMat &aSrcImage     /*!< Source memory buffer. */,
                   vsdk::SUMat &aDestImage    /*!< Destination memory buffer. */);

  
/*!*********************************************************************************
*  \brief Execute the ACF resize process.
*
*  \return APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
***********************************************************************************/   
   APEXCV_LIB_RESULT Process();


   APEXCV_LIB_RESULT SelectApuConfiguration(ACF_APU_CFG aApuConfig,
                              int32_t aApexId);

private:

   int mInitState = 0;
   ACF_APU_CFG mApuConfig;
   int32_t     mApexId;
   bool        mWeSelectedApuConfiguration;
   int  mDataType;

/*!*********************************************************************************
*  \brief Internal ACF resize calls.
*
*  These functions are chosen by process_acf() based on the resize use case.
*
*  \return Error code (zero on success).
***********************************************************************************/  
   APEXCV_LIB_RESULT copy(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT copy_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_vrsz();
   APEXCV_LIB_RESULT acf_vrsz_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_vrsz_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_hrsz();
   APEXCV_LIB_RESULT acf_hrsz_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_hrsz_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_resize();
   APEXCV_LIB_RESULT acf_resize_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_resize_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);

   APEXCV_LIB_RESULT copy16(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT copy16_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_vrsz16();
   APEXCV_LIB_RESULT acf_vrsz16_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_vrsz16_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_hrsz16();
   APEXCV_LIB_RESULT acf_hrsz16_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_hrsz16_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_resize16();
   APEXCV_LIB_RESULT acf_resize16_init(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   APEXCV_LIB_RESULT acf_resize16_conf(vsdk::SUMat& aSrc, vsdk::SUMat& aDst);
   int mProcessType;
   unsigned int mInit; /*!< \brief Indicates all process have been initialized or not */
   bool mIsResizeInitialized;
   bool mIsResizeVInitialized;
   bool mIsResizeHInitialized;
   bool mIsCopyInitialized;
   bool mIsResize16Initialized;
   bool mIsResize16VInitialized;
   bool mIsResize16HInitialized;
   bool mIsCopy16Initialized;
   vsdk::SUMat mSrcWidthUMat;
   vsdk::SUMat mDstWidthUMat;
   vsdk::SUMat mSrcHeightUMat;
   vsdk::SUMat mDstHeightUMat;
   vsdk::SUMat moOffsUMat;
   vsdk::SUMat moInvalidUMat;
   vsdk::Rect mRect;
   int mSrcWidth, mDstWidth, mSrcHeight, mDstHeight;
   ACF_Process_APU *mpProcess;
   ACF_Process_APU *mpProcessVH; /*!< \brief Pointer to resize 8 bit processes with vertical first*/
   ACF_Process_APU *mpProcessHV; /*!< \brief Pointer to resize 8 bit processes with horizontal first*/
   ACF_Process_APU *mpProcessV; /*!< \brief Pointer to resize vertical 8 bit processes */
   ACF_Process_APU *mpProcessH; /*!< \brief Pointer to resize horizontal 8 bit processes */
   ACF_Process_APU *mpProcess16VH; /*!< \brief Pointer to resize 16 bit processes with vertical first*/
   ACF_Process_APU *mpProcess16HV; /*!< \brief Pointer to resize 16 bit processes with horizontal first*/
   ACF_Process_APU *mpProcess16V; /*!< \brief Pointer to resize vertical 16 bit processes */
   ACF_Process_APU *mpProcess16H; /*!< \brief Pointer to resize horizontal 16 bit processes */
};
}  /* namespace apex */

#endif   /* APEXCVPRORESIZE_H */
