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
 *  \brief ApexCV Image Remap
 ***********************************************************************************/

#ifndef APEXCVPROREMAP_H
#define APEXCVPROREMAP_H

#include <sumat.hpp>
#include "icp_data.h"
#include "icp_types.h"
#include <apexcv_host_base_class.h>
#include <apexcv_error_codes.hpp>

namespace apexcv {





  /*!*********************************************************************************
   *  \brief Apex Remap.
   *
   *  apexcv::Remap is the host-ACF interface for creating, initializing,
   *  executing and releasing \ref pagRemap "image remap" on Apex.
   ***********************************************************************************/
  class Remap : public ApexcvHostBaseBaseClass
  {
  public:
  /*!*********************************************************************************
   *  \brief Interpolation options.
   ***********************************************************************************/
  enum INTER_TYPE
  {
    INTER_NN = 0        /*!< Nearest neighbour interpolation. */,
    INTER_LINEAR = 1    /*!< Bilinear interpolation. */,
    INTER_CUBIC = 2     /*!< Bicubic interpolation over 4x4 pixel neighborhood. */,
    INTER_AREA = 3      /*!< Resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method. */,
    INTER_LANCZOS4 = 4  /*!< Lanczos interpolation over 8x8 pixel neighborhood. */
  };

  /*!*********************************************************************************
   *  \brief Border options.
   ***********************************************************************************/
  enum BORDER_TYPE
  {
    BORDER_CONSTANT = 0   /*!< Set the border to a specified value. */,
    BORDER_REPLICATE = 1  /*!< Replicate the border. */,
    BORDER_REFLECT = 2    /*!< Reflect the border. */,
    BORDER_WRAP = 3       /*!< Wrap the border. */
  };

    /**********************************************************************************
     *  Default constructor.
     *  Allocate resources for the ACF process.
     ***********************************************************************************/
    Remap();

    /**********************************************************************************
     *  Destructor.
     *  Release resources for the ACF process.
     ***********************************************************************************/
    ~Remap();

    /*!*********************************************************************************
    *  \brief        Initialize object (required).
    *  \details      This function initializes the object.  The function Process() can
    *                be called to execute the processing on the APEX core.  To process
    *                another image buffer, use ReconnectIO(...).
    *  \return       APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
    ***********************************************************************************/
    APEXCV_LIB_RESULT Initialize(float* aMap, /*!<[in] X, Y pair of the location in the input image for each pixel in the output image. */
      vsdk::SUMat aSrc      /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
      vsdk::SUMat aDst      /*!<[in,out] 8-bit grayscale destination image, VSDK_CV_8UC1. */,
      INTER_TYPE aInterp    /*!<[in] Interpolation mode, only INTER_LINEAR is supported. */,
      BORDER_TYPE aBorder   /*!<[in] Border management scheme, only BORDER_REPLICATE is supported. */,
      uint32_t aBorderValue /*!<[in] Border value, not supported. */);


    /*!*********************************************************************************
    *  \brief        Reconnect IO (optional).
    *  \details      This function allows to change the Input and Output images without re-initializing.
    *  \return       APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
    ***********************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(
      vsdk::SUMat &aSrc /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
      vsdk::SUMat &aDst /*!<[in,out] 8-bit grayscale destination image, VSDK_CV_8UC1. */);

    /*!*********************************************************************************
    *  \brief      Start processing and return when done.
    *  \details    Execute code on selected APEX core (default is Apex core 0).
    *              This function is called after initialize() and is executed on a per frame base.
    *  \return     APEXCV_LIB_RESULT Error code (APEXCV_SUCCESS on success).
    ***********************************************************************************/
    APEXCV_LIB_RESULT Process();

    /*!*********************************************************************************
     *  \brief Generate the look-up table for top view perspective.
     *
     *  \return Error code for the execution (APEXCV_SUCCESS on success).
     ***********************************************************************************/
    APEXCV_LIB_RESULT GenerateLUTFromCalibLoader(const char* apFilename,
      uint32_t aDstWidth,
      uint32_t aDstHeight,
      uint32_t aSrcWidth,
      uint32_t aSrcHeight,
      uint32_t aDestBlockWidth,
      uint32_t aDestBlockHeight,
      uint32_t aRefSrcBlockWidth,
      uint32_t aRefSrcBlockHeight);

    /*!*********************************************************************************
     *  \brief Returns the LUTs needed for the remap ACF process.
     *
     *  This method returns the LUTs which was configured by initialize().
     *
     *  \return Error code for the execution (zero on success).
     ***********************************************************************************/
    int RetLUTs(vsdk::SUMat   &aDeltaDesc,
      vsdk::SUMat   &aLocalOffsetDesc,
      vsdk::SUMat   &aBlockOffsetDesc);

    /*!*********************************************************************************
     *  \brief Generate random float map.
     *
     *  This method generates a random float map based off the input parameter's dimensions.
     *  map must be a pointer to an appropriate sized buffer.
     *  max_offset_per_dim specifies the maximum radius of the dst pixel from the source,
     *  ie. a value of 3 means the dst pixel can be chosen from a 3x3 window centered at
     *  the x, y location of the current pixel.
     *
     ***********************************************************************************/
static void GenerateFloatMap(
      vsdk::SUMat& aInput     /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
      vsdk::SUMat& aOutput    /*!<[in] 8-bit grayscale destination image, VSDK_CV_8UC1, used for dimensions only. */,
      float* apMap            /*!<[out], X, Y pair of the location in the input image for each pixel in the output image. */,
      float aMaxOffsetPerDim  /*!<[in] Allowed variation in the float map from a direct mapping input to output.*/,
      int aSeed               /*!<[in] Random number generator seed. */);

  protected:
    void*  mpParamLoader;
    void*  mpGroundToDewarp;
    void*  mpGroundToWarp;
    float  mTopViewPixelToMeterRatioX;
    float  mTopViewPixelToMeterRatioY;

  private:
    void Release();

    /*!*********************************************************************************
     *  \brief ACF scenario block descriptor struct
     *
     *  This struct describes a potential scenario to be used with ACF.
     *
     ***********************************************************************************/
    struct blockDesc
    {
      uint16_t srcWidth;     /*!< source chunk width */
      uint16_t srcHeight;    /*!< source chunk height */
      uint16_t dstWidth;     /*!< destination chunk width */
      uint16_t dstHeight;    /*!< destination chunk height */
      float ratio;           /*!< destination to source size ratio */
      int32_t memSize;       /*!< CMEM usage in bytes */
    };

    /*!*********************************************************************************
     *  \brief Chosen valid ACF scenario
     *
     *  This contains a valid configuration to be used to set the ACF scenario.
     *
     ***********************************************************************************/
    blockDesc validConfig;

    // Keep image info
    int32_t mSrcWidth;
    int32_t mSrcHeight;
    uint32_t mSrcStride;
    int32_t mDstWidth;
    int32_t mDstHeight;

    // Containers for offeset/delta
    vsdk::SUMat moOffset;     /*!< Motion comp offset buffer. */
    vsdk::SUMat mLocalOffset; /*!< CU local offset buffer. */
    vsdk::SUMat mDelta;       /*!< Interpolation delta buffer. */

    // Pointers for the above containers
    void* mpLocalDstOal;

    void   *mpRemapBlockProc;
    void   *mpRemapProc;

    int32_t mInitState;

    int32_t FloatToFixedOffsets(const float *apcMapRGB, vsdk::SUMat &aFixedOffset, const int32_t acISrcWidth, const int32_t acISrcHeight);
    int32_t InitMoOffsets(vsdk::SUMat &aFixedOffset, const int32_t acSrcStride);
    int32_t FixedToLocalOffsets(vsdk::SUMat &aFixedOffset, const int32_t acSrcStride);
    void PopulateBlockDesc(vsdk::SUMat aData, blockDesc *apdesc);
    int32_t AdjustImageWidth(int32_t aWidth);
    int32_t AdjustNumberOfCU(int32_t aWidth);
  };
}  /* namespace apexcv */

#endif   /* APEXCVPROREMAP_H */
