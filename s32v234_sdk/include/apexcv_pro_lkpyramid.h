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
*   @file apexcv_pro_lkpyramid.h
*   @brief ApexCV L-K Pyramid Optical Flow.
***********************************************************************************/

#ifndef APEXCVPROLKPYRAMID_H
#define APEXCVPROLKPYRAMID_H 

#include <stdint.h>

// ACF HOST LIB
#include "icp_data.h"
#include "icp_feature_32s.h"
#include "apexcv_pro_pyramid_multi.h"
#include "apexcv_host_base_class.h"
#include "apexcv_error_codes.hpp"


// Kernel specific setup (need keep sync with apu's lkpyramid_setup.h
#define LKPYRAMID_FLOATING_COOR_Q 8
#define LKPYRAMID_PATCH_BORDER    6
#define LKPYRAMID_PATCH_BW_7x7    20

namespace apexcv {

/*!*********************************************************************************
*  \brief ApexCV L-K Pyramid Optical Flow.
* 
*  apexcv::LKPyramidOpticalFlow is the Host-ACF interface for creating, initializing,
*  executing and releasing the \ref pagLKPyramid on Apex.
**********************************************************************************/

class LKPyramidOpticalFlow: public ApexcvHostBaseBaseClass
{
public:

/**********************************************************************************
*  Default Constructor.
*
*  Allocate resources for the ACF processes.
***********************************************************************************/
  LKPyramidOpticalFlow();

/**********************************************************************************
*  Destructor.
*
*  Release resources for the ACF processes.
***********************************************************************************/
  ~LKPyramidOpticalFlow();

/*!*********************************************************************************
*  \brief Initialization
* 
*  Initializes the intermediate buffers needed for the process, and initializes the
*  ACF processes. The size of internal buffers determined by aMaxCorners / aW / aH; 
*  pyramid layers cannot exceed 4; 
*  Only supported box_size is 7 for now; 
***********************************************************************************/
   APEXCV_LIB_RESULT Initialize(   vsdk::SUMat& aSrc0Desc   /*!< 8-bit grayscale template source image (frame[t-1]) */,
                     vsdk::SUMat& aSrc1Desc   /*!< 8-bit grayscale patch source image (frame[t]) */,
                     icp::Feature32SDescriptor& aCoor0Desc    /*!< template features list. X/Y coordinates are signed Q23.8 format */,
                     icp::Feature32SDescriptor& aCoor1Desc    /*!< patch features list. X/Y coordinates are signed Q23.8 format */,
                     icp::Feature32SDescriptor& aCoor1Desc_O  /*!< output features list. X/Y coordinates are signed Q23.8 format */,
                     int aMaxCorners           /*!< maximum number of corners going to be tracked */,
                     int aW                    /*!< image width  */,
                     int aH                    /*!< image height */,
                     int aPyrLayers = 1        /*!< Number of pyramid layers. 0 means no pyramid. Tracking on original resolution directly. Default is 1. Maximum is 4 */,
                     int aBoxSize   = 7        /*!< Box size. Only support 7 for now. */,
                     int aNumIter   = 10       /*!< Number of iterations within each pyramid level. Default is 10 */,
                     int aReqPadding = 0       /*!< Require image padding on each layer. 0: no padding to save cycles. 1: require padding. Default is 0 */
               );


/*!*********************************************************************************
*  \brief Reconnect IO
*
*  Use this to connect the input, output buffers and features to be tracked. This will perform
*  preprocessing so it is always needed to
*  be done before "process" call. The number of features in Feature Descriptors cannot exceed the max_corners
*  used in Initialize().
* ***********************************************************************************/
   APEXCV_LIB_RESULT ReconnectIO(  vsdk::SUMat& aSrc0Desc /*!< 8-bit grayscale template source image (frame[t-1]) */,    
                     vsdk::SUMat& aSrc1Desc /*!< 8-bit grayscale patch source image (frame[t])*/,
                     icp::Feature32SDescriptor& aCoor0Desc    /*!< template features list. X/Y coordinates are signed Q23.8 format */,
                     icp::Feature32SDescriptor& aCoor1Desc    /*!< patch features list. X/Y coordinates are signed Q23.8 format */,
                     icp::Feature32SDescriptor& aCoor1Desc_O  /*!< output features list. X/Y coordinates are signed Q23.8 format */
               );

/*!*********************************************************************************
*  \brief Set Box filter Size
*
*  Change the Box filter size used. only support 7 for now.
***********************************************************************************/
   APEXCV_LIB_RESULT SetBoxSize(int aBoxSize /*!< Box size. Only support 7 for now. */
               );

/*!*********************************************************************************
*  \brief Set Pyramid Layers
*
*  Change the pyramid layers used. Maximum is 4. 
***********************************************************************************/
   APEXCV_LIB_RESULT SetPyrLayers(int aPyrLayers /*!< Number of pyramid layers. Maximum is 4 */
               );

/*!*********************************************************************************
*  \brief Set Pyramid Layers
*
*  Change the number of iterations within each pyramid layer.  
***********************************************************************************/
   APEXCV_LIB_RESULT SetNumIter(int aNumIter    /*!< Number of iterations within each pyramid level. */
               );

/*!*********************************************************************************
*  \brief Process 
*  
*  \return Error code for the ACF execution (zero on success).
*  "position" field in output features' descriptor represent the tracked features' 
*  X/Y coordinate in signed Q23.8 format; "reverve" field in output features' 
*  descriptor represent whether feature is successfully tracked. 0: failed; 1: 
*  succeeded; "strength" field in output features' descriptor represent the sum of 
*  pixels' grayscale absolute difference between template and tracked patch window, 
*  i.e. the lower the better tracked.
***********************************************************************************/
   APEXCV_LIB_RESULT Process();

/*!*********************************************************************************
 *  \brief SelectApexCore
 *  \return Error code for the ACF execution (zero on success).
 *
 *  Select which APEX core (0 or 1) to be selected to run the processing.
 *  This function has to be called after initialize() and can be executed per frame base.
 ***********************************************************************************/
   APEXCV_LIB_RESULT SelectApexCore(int32_t aApexId /*!< The ID of the desired APEX (e.g if there are 2 APEXs, valid values for lApexId would be 0 and 1).*/
               );
   
private:

/*!*********************************************************************************
*  \brief update internal SUMat
*
*  After process number of features changed, need to update internal SUMat
***********************************************************************************/
   void UpdateInternalUMat (int aMultiNumFeature, int aNumFeaturesExt);

/*!*********************************************************************************
*  \brief fill in level=0 image
*
***********************************************************************************/
   void FillImageZeroLevel();

/*!*********************************************************************************
*  \brief fill in pyramid levels
*
*  fill in pyramid levels coordinates and offsets
***********************************************************************************/
   void FillPyramidLevel(int aMultiNumFeature);

/*!*********************************************************************************
*  \brief fill in offset
*
***********************************************************************************/
   void FillOffset(int aMultiNumFeature);

/*!****************************************************************************
* \brief Initializes the class: connects the buffer to the process port,
* and allocates/initializes any internal buffers.
* \param[in]  aSrc0 Source memory buffer. Datatype is VSDK_CV_8UC1.
* \param[in]  aSrc1 Source memory buffer. Datatype is VSDK_CV_8UC1.
* \param[out] aDst0Array[0] Destination memory buffer for scale pyramid 1/2. Datatype is VSDK_CV_8UC1.
* \param[out] aDst0Array[1] Destination memory buffer for scale pyramid 1/4. Datatype is VSDK_CV_8UC1.
* \param[out] aDst0Array[2] Destination memory buffer for scale pyramid 1/8. Datatype is VSDK_CV_8UC1.
* \param[out] aDst0Array[3] Destination memory buffer for scale pyramid 1/16. Datatype is VSDK_CV_8UC1.
* \param[out] aDst1Array[0] Destination memory buffer for scale pyramid 1/2. Datatype is VSDK_CV_8UC1.
* \param[out] aDst1Array[1] Destination memory buffer for scale pyramid 1/4. Datatype is VSDK_CV_8UC1.
* \param[out] aDst1Array[2] Destination memory buffer for scale pyramid 1/8. Datatype is VSDK_CV_8UC1.
* \param[out] aDst1Array[3] Destination memory buffer for scale pyramid 1/16. Datatype is VSDK_CV_8UC1.
* \return Error code for the initialization - see apexcv_error_codes.hpp
******************************************************************************/
   APEXCV_LIB_RESULT InitPyramidMulti (
            vsdk::SUMat&  aSrc0   /*!< Source 0 memory buffer. */,
            vsdk::SUMat&  aSrc1   /*!< Source 1 memory buffer. */,
            vsdk::SUMat (&aDst0Array)[apexcv::PyramidMultiCreation::PYRAMID_LEVELS]   /*!< Destination 0 memory buffer. */,
            vsdk::SUMat (&aDst1Array)[apexcv::PyramidMultiCreation::PYRAMID_LEVELS]   /*!< Destination 1 memory buffer. */
   );

/*!****************************************************************************
*  \brief Run the pyramid_multi creation process.
*
*  \return Error code (zero on success).
***********************************************************************************/
   APEXCV_LIB_RESULT ProcessPyramidMulti();

   void Release();          /*!< Release resources for the ACF processes. */

   void *mCoor0Oal;         /*!< Pointer to the template features descriptor's buffer */
   void *mCoor1Oal;         /*!< Pointer to the patch features descriptor's buffer */
   void *mSrc0offsOal;      /*!< Pointer to the template offset buffer for each features to track */
   void *mSrc1offsOal;      /*!< Pointer to the patch offset buffer for each features to track */
   void *mSrc1offsBaseOal;  /*!< Pointer to the base offset buffer for each pyramid image */
   void *mSrc0AllOal;       /*!< Pointer to the contiguous 2D image buffer to hold all template pyramid layers */
   void *mSrc1AllOal;       /*!< Pointer to the contiguous 2D image buffer to hold all patch pyramid layers */

   void *mOutCoorBuf;       /*!< Pointer to the output features buffer */

   vsdk::SUMat mParamsDesc;                   /*!< SUMat of configuration parameters. Bytes [0-3]: Full span of source image; [4]: NUM_ITER; [5-7]: K_0, K_1, K_2 */

   vsdk::CustomPointerSUMat mCoor0Desc;       /*!< Custom Pointer uMat of template features. */
   vsdk::CustomPointerSUMat mCoor0Desc_Next;  /*!< Custom Pointer uMat of template features for next pyramid layer */
   vsdk::CustomPointerSUMat mCoor1Desc;       /*!< Custom Pointer uMat of patch features */
   vsdk::CustomPointerSUMat mCoor1Desc_O;     /*!< Custom Pointer uMat of output patch features */
   vsdk::CustomPointerSUMat mSrc0OffsDesc;    /*!< Custom Pointer uMat of template offset for each features to track */
   vsdk::CustomPointerSUMat mSrc0OffsDesc_Next; /*!< Custom Pointer uMat of template offset for next pyramid layer */
   vsdk::CustomPointerSUMat mSrc1OffsDesc;      /*!< Custom Pointer uMat of patch offset for each features to track */
   vsdk::CustomPointerSUMat mSrc1OffsBaseDesc;  /*!< Custom Pointer uMat of base offset for each pyramid image */
   vsdk::CustomPointerSUMat mSrc1OffsDesc_O;    /*!< Custom Pointer uMat of patch offset for next pyramid layer */
   vsdk::CustomPointerSUMat mSrc0AllDesc;       /*!< Custom Pointer uMat of contiguous 2D image buffer to hold all template pyramid layers */
   vsdk::CustomPointerSUMat mSrc1AllDesc;       /*!< Custom Pointer uMat of contiguous 2D image buffer to hold all patch pyramid layers */
   vsdk::SUMat mSrc0PyrMulDst[apexcv::PyramidMultiCreation::PYRAMID_LEVELS];  /*!< SUMat of template image for each pyramid layer */
   vsdk::SUMat mSrc1PyrMulDst[apexcv::PyramidMultiCreation::PYRAMID_LEVELS];  /*!< SUMat of patch image for each pyramid layer */
   vsdk::SMat mSrc0DescMat;
   vsdk::SMat mSrc1DescMat;
   vsdk::CustomPointerSUMat mSrc0PyrDesc[apexcv::PyramidMultiCreation::PYRAMID_LEVELS+1];  /*!< Custom Pointer uMat of template image for each pyramid layer */
   vsdk::CustomPointerSUMat mSrc1PyrDesc[apexcv::PyramidMultiCreation::PYRAMID_LEVELS+1];  /*!< Custom Pointer uMat of patch image for each pyramid layer */
   icp::Feature32S *mpCoor0;
   icp::Feature32S *mpCoor1;
   bool mInit;              /*!< Indicates if the process has been initialized */
   int mPyrLayers;          /*!< Number of pyramid layers */
   int mBoxSize;            /*!< Box size */
   int mNumIter;            /*!< Number of iterations within each pyramid level. */
   int mReqPadding;         /*!< Require image padding on each pyramid layer. */

   int mWidth;              /*!< Origional image resolution width */
   int mHeight;             /*!< Origional image resolution height*/
   int mSpan;               /*!< Full span */

   int mFrmProc;            /*!< Counter of how many frames has been tracked. For internal debugging purpose */ 
   int mNumFeatures;        /*!< Number of features being tracked */
   int mNumFeatures_ext;    /*!< Padded number of features being tracked */

   void* mProcess;                     /*!< multi-scale tracker's graph */
   void* mProcessDownMulti[apexcv::PyramidMultiCreation::PYRAMID_LEVELS];
   PyramidMultiCreation mPyramidMultiCreation;      /*!< 4 layers pyramid creation graph */

   int32_t     mApexId;       /*!< Index of the APEX core selected to run the algorithm */
};

} /* namespace apex */

#endif /* APEXCVPROLKPYRAMID_H */
