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

/*!****************************************************************************
*  \file apexcv_fast_ref.hpp
*  \brief Reference kernels for fast corner detection algorithm
******************************************************************************/

#ifndef FASTREF_H
#define FASTREF_H
#include <stdint.h>

#ifdef __cplusplus

namespace ref
{
  class FastRef
  {
public:
  /*!*********************************************************************************
  *  \Fast Default constructor.
  *  Allocate resources for the ACF processes.
  ***********************************************************************************/
  FastRef();

  /*!*********************************************************************************
  *  \Fast Destructor.
  *   Release resources for the ACF processes.
  ***********************************************************************************/
  ~FastRef();

  /*!*********************************************************************************
  *  \Fast Get corner scores.
  *  \return Error code for the ACF execution (zero on success).
  *
  *  For each pixel the corner score (referred to as "keypoints" in
  *  \oimg is computed.  The output dst is an image of 16-bit signed.
  *  The corners list are also be outputed in "corners".
  *
  ***********************************************************************************/
  int RefFast9(uint8_t *apOImg, const uint8_t *apcIImg, int aThreshold, int aNms, int aW, int aH);
  };
}

extern "C"  {
#endif // #ifdef __cplusplus

int RefFast9(uint8_t *apOImg, const uint8_t *apcIImg, int aThreshold, int aNms, int aW, int aH);

#ifdef __cplusplus
}
#endif // #ifdef __cplusplus

#endif /* FASTREF_H */

