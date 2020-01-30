/**************************************************************************************************
 * 
 * NXP Confidential Proprietary
 *
 * (C) Copyright CogniVue Corporation. 2014 All right reserved.
 * Copyright 2017-2018 NXP 
 * All Rights Reserved
 *
 *****************************************************************************
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
 **************************************************************************************************/
 
/*!*********************************************************************************
*  @file
*  @brief ACF metadata and wrapper function for the \ref secHarrisCorner "harris corner".
***********************************************************************************/
#ifdef APEX2_EMULATE
#include "acf_kernel.hpp" // if using the ACF emulation library
using namespace APEX2;
#endif

#ifdef ACF_KERNEL_METADATA

#include "flt_box_generic_acf.h"
/*!*********************************************************************************
*  \brief ACF metadata for pixel summation.
*  \see UG-10267-03 ACF User Guide, Section 3.2.2
***********************************************************************************/
KERNEL_INFO  kernelInfoConcat(FLT_BOX_GENERIC_K)
(
   FLT_BOX_GENERIC_KN,
   2,
   __port(  __index(0),
      __identifier(FLT_BOX_GENERIC_K_IN),
      __attributes(ACF_ATTR_VEC_IN),
      __spatial_dep(2, 2, 0, 0),
      __e0_data_type(d16s),
      __e0_size(1, 1),
      __ek_size(4, 8)),

   __port(  __index(1),
      __identifier(FLT_BOX_GENERIC_K_OUT),
      __attributes(ACF_ATTR_VEC_OUT),
      __spatial_dep(0, 0, 0, 0),
      __e0_data_type(d16s),
      __e0_size(1, 1),
      __ek_size(4, 8))
);

/*!*********************************************************************************
*  \brief ACF metadata for pixel summation.
*  \see UG-10267-03 ACF User Guide, Section 3.2.2
***********************************************************************************/
KERNEL_INFO   kernelInfoConcat(HARRIS_BOX_5X5_K)
(
   HARRIS_BOX_5X5_KN,
   2,
   __port(  __index(0),
      __identifier(HARRIS_BOX_K_IN),
      __attributes(ACF_ATTR_VEC_IN),
      __spatial_dep(2, 2, 2, 2),
      __e0_data_type(d16s),
      __e0_size(1, 1),
      __ek_size(4, 8)),

   __port(  __index(1),
      __identifier(HARRIS_BOX_K_OUT),
      __attributes(ACF_ATTR_VEC_OUT),
      __spatial_dep(0, 0, 0, 0),
      __e0_data_type(d16s),
      __e0_size(1, 1),
      __ek_size(4, 8))
);

#endif /* ACF_KERNEL_METADATA */

#ifdef ACF_KERNEL_IMPLEMENTATION

#include "flt_box_generic_apu.h"
/*!*********************************************************************************
*  \brief ACF wrapper function for box 5x5 scaled/optimized for harris.
*  \see UG-10267-03 ACF User Guide, Section 3.2.3
***********************************************************************************/
void apu_flt_box_generic_16s(
               kernel_io_desc lIn      /*!< The source buffer. */,
               kernel_io_desc lOut     /*!< The destination buffer. */
)
{
   vec16s* lpvIn     = (vec16s*)lIn.pMem;
   vec16s* lpvOut    = (vec16s*)lOut.pMem;
   if (ACF_RET_VAR(ACF_VAR_FIRST_TILE_FLAG)) {
      apu_flt_box_generic_init(
                  lpvOut, lOut.chunkSpan>>1,
                  lpvIn, lIn.chunkSpan>>1,
                  lIn.chunkWidth, lIn.chunkHeight, 5, 5, 2);
      apu_flt_box_generic(
                  lpvOut+(lOut.chunkSpan>>1), lOut.chunkSpan>>1,
                  lpvIn, lIn.chunkSpan>>1,
                  lIn.chunkWidth, lIn.chunkHeight-(5>>1)-1, 5, 5, 2);
   } else {
      apu_flt_box_generic(
                  lpvOut, lOut.chunkSpan>>1,
                  lpvIn, lIn.chunkSpan>>1,
                  lIn.chunkWidth, lIn.chunkHeight, 5, 5, 2);
   }
   
   if (ACF_RET_VAR(ACF_VAR_LAST_TILE_FLAG)) {
      apu_flt_box_generic_end(
                  lpvOut+(lIn.chunkHeight-(5>>1))*(lOut.chunkSpan>>1), lOut.chunkSpan>>1,
                  lIn.chunkWidth, lIn.chunkHeight, 5, 5, 2);
   }
}


/*!*********************************************************************************
*  \brief ACF wrapper function for box 5x5 scaled/optimized for harris.
*  \see UG-10267-03 ACF User Guide, Section 3.2.3
***********************************************************************************/
void harris_box_5x5_16s(
               kernel_io_desc lIn      /*!< The source buffer. */,
               kernel_io_desc lOut     /*!< The destination buffer. */
)
{
   vec16s* lpvIn     = (vec16s*)lIn.pMem;
   vec16s* lpvOut    = (vec16s*)lOut.pMem;
   
   apu_harris_box_5x5(
               lpvOut, lOut.chunkSpan>>1,
               lpvIn, lIn.chunkSpan>>1,
               lIn.chunkWidth, lIn.chunkHeight, 2);
}

#endif /* ACF_KENREL_IMPLEMENTATION */

