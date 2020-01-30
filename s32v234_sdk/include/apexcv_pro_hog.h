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

/*!*****************************************************************************
*  \file
*  \brief Apex HOG Object Detector based on:
*  N. Dalal and B. Triggs. Histograms of Oriented Gradients for Human Detection.
*  INRIA, 2005.  https://lear.inrialpes.fr/people/triggs/pubs/Dalal-cvpr05.pdf
*
*  The HOG parameters are as follows:
*  histogram bins = 8
*  cell size = block size = 8x8 pixels
*  block stride = 8x8 pixels
*  detection window stride = 4x4 pixels
*  The descriptor size is:
*  Nd = (detection window blocks)*(histogram bins) =
*  (detection window width)*(detection window height)*(histogram bins)/(blocks width*blocks height) =
*  (detection window width)*(detection window height)*8/(8*8) =
*  (detection window width)*(detection window height)/8
*
*  The SVM decision function comes from training the detector on a class of objects
*  (e.g. pedestrians). By specifying the decision function you specify the class
*  of objects you wish to detect. The size of the decision function is (Nd + 1)
*  for (SVM weights + offset) doubles.
*******************************************************************************/

#ifndef APEXCVPROHOG_H
#define APEXCVPROHOG_H

#include <apexcv_host_base_class.h>
#include <apexcv_error_codes.hpp>
#include <sumat.hpp>

namespace apexcv
{
/*!*****************************************************************************
*  \brief Apex HOG class.
*
*  apexcv::Hog is the host interface for HOG detection running on Apexcore. \ref pagHOG
*
*  The process takes an 8-bit grayscale image and computes the 8-bin normalized
*  block histogram every 4x4 pixels.
*  If SVM decision function is provided in the initialization, it multiplies the
*  blocks histograms with SVM weights for each detection window and returns the
*  scores for the object detection. Otherwise it returns the computed block histograms.
*******************************************************************************/
  class Hog: public ApexcvHostBaseClass
  {
  public:

    /*!***************************************************************************
    * \brief Specifies how the weights of SVM decision function will be applied
    * in the multiplication with block histograms of the detection window.
    *****************************************************************************/
    enum class SVMTransformMode
    {
      NONE = 0,                 /*!< Apply SVM as it is */
      HORIZONTAL_SYMMETRY,      /*!< Apply SVM once as it is and once mirrored horizontally */
      HORIZONTAL_MIRRORING      /*!< Apply SVM with horizontally mirrored weights */
    };

    /*!***************************************************************************
    * \brief The HOG parameters for the linear SVM classifier.
    *****************************************************************************/
    struct Config
    {
      uint32_t mDetWinWidth;              /*!< The width of the detection window in pixels */
      uint32_t mDetWinHeight;             /*!< The height of the detection window in pixels */
      uint32_t mBlockWidth;               /*!< The horizontal size of a Hog block. Keep unchanged. */
      uint32_t mBlockHeight;              /*!< The vertical size of a Hog block. Keep unchanged. */
      uint32_t mStrideWidth;              /*!< The horizontal stride of the detection window. Keep unchanged. */
      uint32_t mStrideHeight;             /*!< The vertical stride of the detection window. Keep unchanged. */
      uint32_t mHistogramBins;            /*!< The number of bins in HOG descriptor. Keep unchanged. */
      SVMTransformMode mSVMTransformMode; /*!< SVM Transformation Mode for manipulating existing SVM detector */

      Config();
    };

    /*!*****************************************************************************
    *  \brief Class to compute SVM detector from Hog blocks
    *
    *  apexcv::Hog::SVM is used in case of separate detection, when the apexcv::Hog class
    *  compute the hog blocks only. \ref pagHOG
    *
    *  The process takes an 8-bin normalized block histogram computed every 4x4 pixels.
    *  It multiplies the block histograms with SVM weights for each detection window
    *  and returns the scores for the object detection.
    *******************************************************************************/
    class SVM: public ApexcvHostBaseClass
    {
    public:

      virtual ~SVM();

      /*!***************************************************************************
      * \brief Sets the configuration for the SVM class
      * 
      * Set the HOG parameters for the linear SVM classifier. It is used when changing
      * from default setting only. It should be called before Initialize().
      *
      * The size of the detection window should not be bigger than 128x128 pixels.
      * If some dimension is not integer multiple of the HOG block size, it is rounded
      * down per the block size.
      *****************************************************************************/
      APEXCV_LIB_RESULT SetConfig(const Config &aConfig);

      /*!***************************************************************************
      * \brief Gets the configuration for the class
      * 
      * Returns the HOG parameters for the linear SVM classifier.
      *****************************************************************************/
      void GetConfig(Config &aConfig) const;

      /*!***************************************************************************
      *  \brief Initialization of the SVM object
      * 
      *  \return Error code for ACF process initialization, APEXCV_SUCCESS on
      *  success.
      *
      *  Prepares the object for execution. Initialization is done at this stage.
      *  For the size of the arguments, please refer to ReconnectIO() of Hog class
      *
      ******************************************************************************/
      APEXCV_LIB_RESULT Initialize(const vsdk::SUMat &aSrc  /*!<[in] Blocks histograms, output of Hog::Process(), VSDK_CV_8UC1 */,
                                   const vsdk::SUMat &aSVM  /*!<[in] Vector of descriptorSize + 1 elements of type double, VSDK_CV_64FC1. */,
                                         vsdk::SUMat &aDst  /*!<[in,out] Scores from HOG object detection, VSDK_CV_16SC1. */);
      /*!****************************************************************************
      *  \brief Reconnect IO
      *
      *  Use this to reconnect the input and output buffers and allocate the output.
      *  This only needs to be done if the connected Input/Outputs are changed.
      *  If only the data within changes (no size, data pointer, or type changes), then this
      *  does not need to be called.
      *  This function will allocation aDst if it has not been pre-allocated with the right dimensions.
      *  For the size of the arguments, please refer to ReconnectIO() of Hog class
      ******************************************************************************/
      APEXCV_LIB_RESULT ReconnectIO(const vsdk::SUMat &aSrc /*!<[in] Blocks histograms, output of Hog::Process(), VSDK_CV_8UC1. */,
                                          vsdk::SUMat &aDst /*!<[in,out] Scores from HOG object detection, VSDK_CV_16SC1. */);

    private:
      APEXCV_LIB_RESULT Create();

    private:
      Config mConfig;

      // For ACF ports
      vsdk::SUMat mSVMUMat;               /*!< SVM decision function memory buffer */
      vsdk::SUMat mWinBlocksUMat;         /*!< Duplicated size of the detection window in number of blocks */
      vsdk::SUMat mSVMTransformModeUMat;  /*!< Transformation mode as UMat for the graph port */
    };

    virtual ~Hog();

    /*!***************************************************************************
    * \brief Sets the configuration for the Hog class, used only if in full detector mode.
    *
    * Set the HOG parameters for the linear SVM classifier. It is used when changing
    * from default setting only. It should be called before Initialize().
    *
    * The size of the detection window should not be bigger than 128x128 pixels.
    * If some dimension is not integer multiple of the HOG block size, it is rounded
    * down per the block size.
    *****************************************************************************/
    APEXCV_LIB_RESULT SetConfig(const Config &aConfig);

    /*!***************************************************************************
    * \brief Returns the HOG parameters for the linear SVM classifier.
    *****************************************************************************/
    void GetConfig(Config &aConfig) const;

    /*!***************************************************************************
    *  \brief Initialization of the Hog object for detection mode.
    *
    *  \return Error code for ACF process initialization, APEXCV_SUCCESS on
    *  success.
    *
    *  Prepares the object for execution. Initialization is done at this stage.
    *  For the size of the arguments, please refer to ReconnectIO()
    ******************************************************************************/
    APEXCV_LIB_RESULT Initialize(const vsdk::SUMat &aSrc  /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
                                 const vsdk::SUMat &aSVM  /*!<[in] Vector of descriptorSize + 1 elements of type double, VSDK_CV_64FC1. */,
                                       vsdk::SUMat &aDst  /*!<[in,out] Scores from HOG object detection, VSDK_CV_16SC1. */);

    /*!***************************************************************************
    *  \brief Initialization of the Hog object for block mode.
    *
    *  \return Error code for ACF process initialization, APEXCV_SUCCESS on
    *  success.
    *
    *  Prepares the object for execution. Initialization is done at this stage.
    *  For the size of the arguments, please refer to ReconnectIO()
    ******************************************************************************/
    APEXCV_LIB_RESULT Initialize(const vsdk::SUMat &aSrc  /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
                                       vsdk::SUMat &aDst  /*!<[in,out] Blocks histograms, VSDK_CV_8UC1. */);

    /*!****************************************************************************
    *  \brief Reconnect IO
    *
    *  Use this to reconnect the input and output buffers and allocate the output.
    *  This only needs to be done if the connected Input/Outputs are changed.
    *  If only the data within changes (no size, data pointer, or type changes), then this
    *  does not need to be called.
    *  This function will allocation aDst if it has not been pre-allocated with the right dimensions.
    *
    *  The width of the source image should be multiple 64. The height of the source
    *  image should be multiple 4.
    *  Detection mode: The output is the scores matrix of VSDK_CV_16SC1. For a (WinxHin)
    *  sized image and wxh sized detection window the output size is:
    *  (Wout, Hout) = (floor((Win-w)/4) + 1, floor((Hin-h)/4) + 1)
    *  Block mode: The output is the block histogram matrix of VSDK_CV_8UC1. For a (WinxHin)
    *  sized image and window stride 4x4 the output size is:
    *  (Wout, Hout) = (floor((Win-4)/4)*8, floor((Hin-4)/4))
    ******************************************************************************/
    APEXCV_LIB_RESULT ReconnectIO(const vsdk::SUMat &aSrc /*!<[in] 8-bit grayscale source image, VSDK_CV_8UC1. */,
                                        vsdk::SUMat &aDst /*!<[in,out] Detect mode: Scores from HOG object detection, VSDK_CV_16SC1,
                                                                    Block mode: Blocks histograms, VSDK_CV_8UC1. */);

    /*!***************************************************************************
    *  \brief Concatenate blocks to form HOG descriptor
    *  \return Error code for the ACF execution (APEXCV_SUCCESS on success).
    *
    *  The HOG descriptor is a column-wise concatenation of the block histograms
    *  in a detection window.  If the input image size is not an integer mutiple
    *  of the window size and stride, partial blocks at the right and bottom edges
    *  are not computed. The length-Nd descriptors are packed as an 8-bit unsigned
    *  Nd-channel image.
    *  For a (WinxHin) sized image and wxh sized block window the output size
    *  is: (Wout, Hout) = (floor((Win-w)/4)*Nd, floor((Hin-h)/4))
    *  Note that the memory requirement of the output descriptor image is large:
    *  Nd/16 = 64 times the input image.
    *****************************************************************************/
    static APEXCV_LIB_RESULT GetDescriptors(const Config &aConfig           /*!<[in] HOG configuration */,
                                            const vsdk::SUMat &aBlocksHist  /*!<[in] Blocks histograms, output of Hog::Process(), VSDK_CV_8UC1. */,
                                                  vsdk::SUMat &aDescriptor  /*!<[out] HOG descriptor, populated on successful return, VSDK_CV_8UC1. */);

  private:
    enum TYPE
    {
      DETECT    /*< SVM detection result for each detection window */,
      BLOCKHIST /*< 8-bin normalized block histogram computed every 4x4 pixels */
    };
    APEXCV_LIB_RESULT Create(TYPE aHogType);

  private:
    Config mConfig;

    // For ACF ports
    vsdk::SUMat mSVMUMat;               /*!< SVM decision function memory buffer */
    vsdk::SUMat mWinBlocksUMat;         /*!< Duplicated size of the detection window in number of blocks */
    vsdk::SUMat mSVMTransformModeUMat;  /*!< Transformation mode as SUMat for the graph port */
  };
}  /* namespace apex */

#endif /* APEXCVPROHOG_H */
