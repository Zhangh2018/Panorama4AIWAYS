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
 * *  @file lkpyramid_full_7x7_apu_process_desc.hpp
 * *  @brief Process description for L-K Pyramid 7x7 template size full graph 
 * ***********************************************************************************/

#ifndef LKPYRAMIDFULL7X7APUPROCDESC_HPP
#define LKPYRAMIDFULL7X7APUPROCDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "lkpyramid_full_7x7_graph.hpp"

class lkpyramid_full_7x7_apu_proc_desc : public ACF_Process_Desc_APU
{
public:
   void Create()
   {
      Initialize(mGraph, "LKPYRAMID_FULL_7X7");
		FlagInputAsChunkBasedIndirect("INPUT_TEMPLATE");
		FlagInputAsChunkBasedIndirect("INPUT_PATCH");
   }
   lkpyramid_full_7x7_graph mGraph;
};

#endif /* LKPYRAMIDFULL7X7APUPROCDESC_HPP */
