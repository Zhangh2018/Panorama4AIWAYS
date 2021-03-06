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
*  @file
*  @brief ACF process description for the \ref secGFTTCorner "GFTT Corner" detection
***********************************************************************************/
#ifndef GFTTCOMPUTEEXTRACTAPUPROCESSDESC_HPP
#define GFTTCOMPUTEEXTRACTAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "gftt_b7n5_compute_extract_graph.hpp"

/*!*********************************************************************************
*  \brief ACF process description for the \ref secGFTTCorner "GFTT Corner" detection
*  \see UG-10267-03 ACF User Guide, Section 3.4
***********************************************************************************/
class gftt_b7n5_compute_extract_apu_process_desc : public ACF_Process_Desc_APU
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
      Initialize(mGraph, "GFTT_B7N5_COMPUTE_EXTRACT");
   }

   gftt_b7n5_compute_extract_graph mGraph;
};

#endif /* GFTTCOMPUTEEXTRACTAPUPROCESSDESC_HPP */


