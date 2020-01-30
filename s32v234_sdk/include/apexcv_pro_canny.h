/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
* All Rights Reserved
*
***********************************************************************************
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
***********************************************************************************/

/*!*********************************************************************************
*  \file apexcv_pro_canny.h
*  \brief ApexCV Canny Edge Detector
***********************************************************************************/

#ifndef APEXCVPROCANNY_H
#define APEXCVPROCANNY_H

#include <stdint.h>
#include <sumat.hpp>
#include <apexcv_error_codes.hpp>
#include <apexcv_host_base_class.h>

namespace apexcv{


/*!*********************************************************************************
*  \brief ApexCV Canny Edge Detector.
*
*  apexcv::Canny is the Host-ACF interface for creating, initializing,
*  executing and releasing the \ref pagCanny on Apex.
***********************************************************************************/
class Canny: public ApexcvHostBaseBaseClass
{
  public:

  struct CannyConfig
  {
    uint32_t mIters;          /*!< The number of initial extra edge promotion iterations to process */
    bool     mIsUseProcessCombined; /*!< false: selecting separated kernel process, true: selecting combined kernel process */


    CannyConfig():
    mIters(0),
    mIsUseProcessCombined(false)
    {
    }
  };

  /**********************************************************************************
  *  Default Constructor.
  *  Allocate resources for the ACF processes.
  ***********************************************************************************/
  Canny();

  /**********************************************************************************
  *  Destructor.
  *  Release resources for the ACF processes.
  ***********************************************************************************/
  ~Canny();

/*!*********************************************************************************
*  \brief Initialization
*
*  Initializes the intermediate buffers needed for the processes, initializes the
*  ACF processes and connect buffers to processes' IO. The number of iterations refers 
*  to the number of times the block connection process is to be run.
***********************************************************************************/
   APEXCV_LIB_RESULT Initialize(vsdk::SUMat& aSrc    /*!< 8-bit grayscale source image */,
                                vsdk::SUMat& aDst    /*!< 8-bit destination image */,
                                uint16_t aLow       /*!< 16-bit low threshold for edge hysteresis */,
                                uint16_t aHigh      /*!< 16-bit high threshold for edge hysteresis */
                                );

/*!*********************************************************************************
*  \brief Reconnect IO
*
*  Use this to reconnect the input and output buffers. This only needs to
*  be done if the connected Input/Outputs are changed. If only the data within (no size,
*  data pointer, or type changes), then this does not need to be called.
***********************************************************************************/
   APEXCV_LIB_RESULT ReconnectIO(vsdk::SUMat& aSrc  /*!< 8-bit grayscale source image */,
                                 vsdk::SUMat& aDst  /*!< 8-bit destination image */
                                 );

/*!*********************************************************************************
 *  \brief Set Edge Promotion Iteration and select separated kernel process or combined kernel process
 *
 *  Set give configuration data to the configuration structure in Canny class
 ***********************************************************************************/
   APEXCV_LIB_RESULT SetConfiguration(CannyConfig config);

/*!*********************************************************************************
*  \brief Return the configuration structure
*
*  Returns current configuration structure in Canny class.
***********************************************************************************/
   void GetConfiguration(CannyConfig& aConfig);

/*!*********************************************************************************
*  \brief Set the Edge Hysteresis Thresholds
*
*  Sets the low and high thresholds for edge hysteresis. Only affects the _process()_
*  call. This does not affect the _promoteEdges()_ call.
***********************************************************************************/
   APEXCV_LIB_RESULT SetThresholds(uint16_t aLow, uint16_t aHigh);

/*!*********************************************************************************
*  \brief Return the Edge Hysteresis Thresholds
*
*  Returns the low and high thresholds for edge hysteresis.
***********************************************************************************/
   APEXCV_LIB_RESULT GetThresholds(uint16_t& aLow, uint16_t& aHigh);

/*!*********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apexcv::Canny::Initialize
*  function must be called with the appropriate parameters.
*  
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
***********************************************************************************/
   APEXCV_LIB_RESULT Process();


/*!*********************************************************************************
*  \brief Runs the Edge promotion extra times
*
*  This will run extra edge promotion iterations on a the previously obtained result
*  using either _promoteEdges()_ or _process()_ or _processCombined()_.
*  Only valid after running _process()_ or _processCombined()_.
***********************************************************************************/
   APEXCV_LIB_RESULT PromoteEdges(int aIterations);

private:
/***********************************************************************************
*  \brief Release resources.
*   Release resources for the ACF processes and intermediate buffers.
***********************************************************************************/
   void Release();

/***********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apex::Canny::initialize
*  function must be called with the appropriate parameters.
*
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
*
*  This will run the version where the kernels are split into multiple graphs.
***********************************************************************************/
   APEXCV_LIB_RESULT ProcessSeparated();

/*!*********************************************************************************
*  \brief Runs the Canny algorithm
*
*  This will run the Canny algorithm. Before this is called, the apexcv::Canny::Initialize
*  function must be called with the appropriate parameters.
*  
*  The input image is an 8-bit grayscale image. The Canny detection algorithm works best
*  if the image has been smoothed to get rid of noise. The output is an 8-bit grayscale
*  image with the detected edges set to 255 and non edges set to 0.
*
*  This will run the version where every kernel is in one graph.
*  However, this version will detect less edges than the normal process (with zero
*  extra iterations) where the kernels are split into multiple graphs.
***********************************************************************************/
   APEXCV_LIB_RESULT ProcessCombined();

   vsdk::SUMat mEdgeMapImg_a;   /*!< First full buffer for the edge map image */
   vsdk::SUMat mEdgeMapImg_b;   /*!< Second full buffer for the edge map image */
   vsdk::SUMat mLowThreshImg;   /*!< Low threshold image */
   vsdk::SUMat mHighThreshImg;  /*!< High threshold image */
   
   vsdk::SUMat* mEdgeMapInput;  /*!< Pointer to the edge map image to be used as the input  */
   vsdk::SUMat* mEdgeMapOutput; /*!< Pointer to the edge map image to be used as the output */
   vsdk::SUMat  mDestOutput;    /*!< buffer for the output edge map image */

   bool     mInit;           /*!< Indicates if the process has been initialized */
   int      mWidth;          /*!< The width of the source image */
   int      mHeight;         /*!< The height of the source image */
   int32_t  mBlockWidth;     /*!< The width of the block size used in the edge promotion process */
   int32_t  mBlockHeight;    /*!< The height of the block size used in theedge promotion process */
   int      mIndex;          /*!< Which edge map image is the most up to date */
   bool     mProcessed;      /*!< Indicates if the _process()_ function has been called */
   
   void* mFullProcess;           /*!< Canny Edge Detection ACF process */
   void* mFullIterProcess;       /*!< Canny Edge Detection ACF process without image create graph*/
   
   void* mSuppressProcess;       /*!< Non-Maxima Suppression ACF process */
   void* mPromoteFullProcess;    /*!< Internal Block edge promotion ACF process */
   void* mPromoteProcess;        /*!< External Block edge promotion ACF process */

   CannyConfig mConfig;          /*!< Configuration structure for Canny Process */
}; /* class Canny */

}  /* namespace apex */

#endif /* APEXCVPROCANNY_H */
