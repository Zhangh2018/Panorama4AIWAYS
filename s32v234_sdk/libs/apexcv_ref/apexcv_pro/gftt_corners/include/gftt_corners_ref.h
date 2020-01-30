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
*  \file gftt_corners_ref.h
*  \brief Reference kernels for image gftt corners algorithm
***********************************************************************************/

#ifndef APEXCVGFTTCORNERSREF_H
#define APEXCVGFTTCORNERSREF_H

#include <stdint.h>

// ACF HOST LIB
#include <icp_data.h>
#include <sumat.hpp>
#include <icp_feature.h>

namespace ref{

/*!*********************************************************************************
*  \brief Apex GFTT Corner Detector.
*
*  apex::GFTTCornerDetector is the host-ACF interface for creating, initializing,
*  executing and releasing the \ref pagGFTT on Apex.
***********************************************************************************/
class GFTTCorners
{
public:

/*!*********************************************************************************
*  \brief Default constructor.
*  Allocate resources for the ACF processes.
***********************************************************************************/
   GFTTCorners();

/*!*********************************************************************************
*  \brief Destructor.
*   Release resources for the ACF processes.
***********************************************************************************/
   ~GFTTCorners();

/*!*********************************************************************************
* \Brief Returns a 16-bit corner score image.
*
************************************************************************************/
   icp::DataDescriptor retCornerImage();

/*!*********************************************************************************
*  \brief Get the GFTT corner scores.
*  \return Error code for the ACF execution (zero on success).
*
*  For each pixel the corner score (referred to as "corner response" in
*  \cite GFTT_CORNER) is computed.  The output dst is an image of 16-bit signed.
*  The corners list are also be outputed in "corners".
*
***********************************************************************************/
   int process(vsdk::SMat src,
               icp::FeatureDescriptor& corners,
               const int cMinDistance,
               const int cQualityLevel,
               const int cBoxSize,
               const int cNmsRadius,
               const int cMaxTotalCorners,
               const int cUseHarrisDetector,
               const int cHarrisK,
               const int cHarrisThreshold);
private:
   icp::DataDescriptor mCornerImage;        /*!< GFTT corner ref image buffer. */
   void* mCornerImagePtr;                   /*!< Pointer to the Corner Image memory buffer. */
};

}  /* namespace apexcv */

#endif /* APEXCVGFTTCORNERSREF_H */
