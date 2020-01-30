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
 * *  @file lktracker_ref.h
 * *  @brief ApexCV L-K tracker reference model.
 * ***********************************************************************************/

#ifndef LKTRACKERREF_H
#define LKTRACKERREF_H

#include <oal.h>
#include <sumat.hpp>
#include <icp_data.h>
#include "icp_feature_32s.h"

class LKTrackerRef
{
public:
   enum {
      LKTRACKER_REF_SUCCESS         = 0,
      LKTRACKER_REF_UNINITIALIZED   = 1,
      LKTRACKER_REF_PROCESS_NOT_RUN,
      LKTRACKER_REF_OUT_OF_MEMORY,
      LKTRACKER_REF_INVALID_ARG,
      LKTRACKER_REF_ACF_PROCESS_ERROR,
      LKTRACKER_REF_ALREADY_INITIALIZED
   };

   LKTrackerRef();
   ~LKTrackerRef();

   int Initialize(   vsdk::SMat& aSrc0          /*!< 8-bit grayscale source image 0 */,
                     vsdk::SMat& src1          /*!< 8-bit grayscale source image 1 */,
                     icp::Feature32SDescriptor& aCoor0   /*!< 32-bit coordinates in image 0*/,
                     icp::Feature32SDescriptor& aCoor1   /*!< 32-bit coordinates in image 1 */,
                     icp::Feature32SDescriptor& coor1_o /*!< 32-bit coordinates in image 1 */,
                     int box_size   = 7        /*!< Box size */,
                     int num_iter   = 10       /*!< Number of iterations within each pyramid level */
               );

   int Process();

private:

   void Release();

   void *mK0Oal;
   void *mK1Oal;
   void *mK2Oal;
   void *mFracxy0Oal;
   void *mSrc0offsOal;
   void *mSrc1offsOal;


   icp::Feature32SDescriptor mCoor0Desc; /*!< 32-bit coordinates in image 0*/
   icp::Feature32SDescriptor mCoor1Desc; /*!< 32-bit coordinates in image 1 */
   icp::Feature32SDescriptor mCoor1Desc_O; /*!< 32-bit coordinates in image 1 */

   icp::DataDescriptor mK0Desc;
   icp::DataDescriptor mK1Desc;
   icp::DataDescriptor mK2Desc;
   icp::DataDescriptor mSrc0Desc; /*!< 8-bit grayscale source image 0 */
   icp::DataDescriptor mSrc1Desc; /*!< 8-bit grayscale source image 1 */
   icp::DataDescriptor mFracXY0Desc;
   icp::DataDescriptor mSrc0Offs;
   icp::DataDescriptor mSrc1Offs;

   bool mInit;                /*!< Indicates if the process has been initialized */
   int mPyrLayers;           /*!< Number of pyramid layers */
   int mBoxSize;             /*!< Box size */
   int mNumIter;             /*!< Number of iterations within each pyramid level */
};

#endif /* LKTRACKERREF_H */
