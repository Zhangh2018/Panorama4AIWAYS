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
 * *  @file lkpyramid_ref.h
 * *  @brief ApexCV L-K pyramid reference model.
 * ***********************************************************************************/

#ifndef LKPYRAMIDREF_H
#define LKPYRAMIDREF_H

#include <oal.h>
#include <icp_data.h>

#define MAX_NUM_PYRAMID    4
#define MAX_NUM_ITERATIONS 20

class LKPyramidRef
{
public:
   enum {
      LKPYRAMID_REF_SUCCESS         = 0,
      LKPYRAMID_REF_UNINITIALIZED   = 1,
      LKPYRAMID_REF_PROCESS_NOT_RUN,
      LKPYRAMID_REF_OUT_OF_MEMORY,
      LKPYRAMID_REF_INVALID_ARG,
      LKPYRAMID_REF_ACF_PROCESS_ERROR,
      LKPYRAMID_REF_ALREADY_INITIALIZED
   };

   LKPyramidRef();
   ~LKPyramidRef();

   int Initialize(   icp::DataDescriptor& src0    /*!< 8-bit grayscale source image 0 */,
                     icp::DataDescriptor& src1    /*!< 8-bit grayscale source image 1 */,
                     icp::DataDescriptor& coor0   /*!< 32-bit coordinates in image 0 */,
                     icp::DataDescriptor& coor1   /*!< 32-bit coordinates in image 1 */,
                     icp::DataDescriptor& coor1_o /*!< 32-bit coordinates in image 1 */,
                     int pyr_layers = 1,       /*!< Number of pyramid layers */
                     int box_size   = 7        /*!< Box size */,
                     int num_iter   = 10       /*!< Number of iterations within each pyramid level */,
                     int req_padding = 0       /*!< Require padding on each layer */
               );

   int Process(      icp::DataDescriptor& src0    /*!< 8-bit grayscale source image 0 */,
                     icp::DataDescriptor& src1    /*!< 8-bit grayscale source image 1 */,
                     icp::DataDescriptor& coor0   /*!< 32-bit coordinates in image 0 */,
                     icp::DataDescriptor& coor1   /*!< 32-bit coordinates in image 1 */,
                     icp::DataDescriptor& coor1_o /*!< 32-bit coordinates in image 1 */
               );

private:

   void Release();

   void *mCoor0Oal;
   void *mCoor1Oal;
   void *mCoor1Oal_O;

   void *mSrc0offsOal;
   void *mSrc1offsOal;

   void *mSrc0AllOal;
   void *mSrc1AllOal;

   icp::DataDescriptor mCoor0Desc; /*!< 32-bit coordinates in image 0 */
   icp::DataDescriptor mCoor1Desc; /*!< 32-bit coordinates in image 1 */
   icp::DataDescriptor mCoor1Desc_O; /*!< 32-bit coordinates in image 1 */

   icp::DataDescriptor mSrc0Offs;
   icp::DataDescriptor mSrc1Offs;

   icp::DataDescriptor mSrc0AllDesc;
   icp::DataDescriptor mSrc1AllDesc;
   icp::DataDescriptor mSrc0PyrDesc[MAX_NUM_PYRAMID+1]; /*!< 8-bit grayscale source image 0 */
   icp::DataDescriptor mSrc1PyrDesc[MAX_NUM_PYRAMID+1]; /*!< 8-bit grayscale source image 0 */


   bool mInit;                /*!< Indicates if the process has been initialized */
   int mPyrLayers;           /*!< Number of pyramid layers */
   int mBoxSize;             /*!< Box size */
   int mNumIter;             /*!< Number of iterations within each pyramid level */
   int mReqPadding;          /*!< Require padding on each layer */

   int8_t mK0;
   int8_t mK1;
   int8_t mK2;

   int mWidth;
   int mHeight;
   int mSpan;
};

#endif /* LKPYRAMIDREF_H */
