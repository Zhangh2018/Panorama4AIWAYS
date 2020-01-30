/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015, Digital Media Professionals. All rights reserved.
* Copyright (c) 2016 Freescale Semiconductor
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


#ifndef TMOLDRTRANSFORMAPUPROCESSDESC_HPP
#define TMOLDRTRANSFORMAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "tmo_graph.hpp"

class tmo_ldr_transform_apu_process_desc : public ACF_Process_Desc_APU
{
public:
   void Create()
   {
      Initialize(mGraph, "TMO_LDR_TRANSFORM");
      //SetInputChunkSize("INTERMEDIATE_EXP", 16, 12);
   }

   tmo_ldr_transform_graph mGraph;
};

#endif /* TMOLDRTRANSFORMAPUPROCESSDESC_HPP */
