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
*  \file affine_bilinear_interpolate_proc_desc.hpp
*  \brief ACF Affine Transform Bilinear Interpolate Process Description.
***********************************************************************************/

#ifndef AFFINEBILINEARINTERPOLATEPROCDESC_HPP
#define AFFINEBILINEARINTERPOLATEPROCDESC_HPP
/*!*********************************************************************************
*  @file
*  @brief ACF process description for the \ref secAffineTransform "affine transformation"
*  bilinear interpolation.
***********************************************************************************/
#include <acf_process_desc_apu.hpp>
#include "affine_bilinear_interpolate_graph.hpp"

/*!*********************************************************************************
*  \brief ACF process description for the \ref secAffineTransform "affine transformation"
*  bilinear interpolation.
*  \see UG-10267-03 ACF User Guide, Section 3.4
***********************************************************************************/
class affine_bilinear_interpolate_proc_desc : public ACF_Process_Desc_APU
{
public:
/*!********************************************************************************* 
*  \brief Create the ACF process description.
*  
*  In this function we 
*     - initialize the process and give it a unique identifier.
*     - set the input chunk size
***********************************************************************************/
   void Create()
   {
      Initialize(mAtBiInterpGraph, "AFFINE_BILINEAR_INTERPOLATE");
      FlagInputAsChunkBasedIndirect("INPUT");
   }
   
   /*! \brief \ref at_bilinear_interpolate_graph "ACF bilinear interpolation graph" of the \ref secAffineTransform "affine transformation". */
   affine_bilinear_interpolate_graph mAtBiInterpGraph;
};

#endif /* AFFINEBILINEARINTERPOLATEPROCDESC_HPP */
