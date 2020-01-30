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
*  \file
*  \brief ACF process description for the resize implementation.
***********************************************************************************/
#ifndef RESIZE16HVPROCESSDESC_HPP
#define RESIZE16HVPROCESSDESC_HPP


#include "acf_process_desc_apu.hpp"
#include "resize16_h_v_graph.hpp"

/*!*********************************************************************************
*  \brief ACF process description for resize.
*  \see UG-10267-03 ACF User Guide, Section 3.4
***********************************************************************************/
class resize16_h_v_process_desc : public ACF_Process_Desc_APU
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
      Initialize(mResizeGraph, "RESIZE16_H_V");
      FlagInputAsChunkBasedIndirect("SRC");

   }
   
/*! \ref resize_h_v_graph "ACF graph" of resizer. */
   resize16_h_v_graph mResizeGraph;
};     
#endif /* RESIZE16HVPROCESSDESC_HPP */

