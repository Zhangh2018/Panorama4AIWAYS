/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
***************************************************************************
  
/*!*********************************************************************************
*  @file magnitude_16s_proc_desc.hpp
*  @brief Process description for magnitude (signed 16 bit)
***********************************************************************************/
#ifndef MAGNITUDE16SAPUPROCESSDESC_HPP
#define MAGNITUDE16SAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include <acf_graph.hpp>
#include "graph_templates.h"
#include "arithmetic_acf.h"

GRAPH_CLASS_DEF_2In_1Out(MAGNITUDE_16s_K)

#define MAG_16S_P_ID  MAGNITUDE_16S
#define MAG_16S_P_STR  XSTR(MAG_16S_P_ID)

class magnitude_16s_apu_process_desc : public ACF_Process_Desc_APU
{
public:
  void Create()
  {
    Initialize(mGraph, MAG_16S_P_STR);
  }

  graphClassName(MAGNITUDE_16s_K) mGraph;
};


#ifdef APEX2_EMULATE
REGISTER_PROCESS_TYPE(MAG_16S_P_ID, magnitude_16s_apu_process_desc)
#endif

#endif /* MAGNITUDE16SAPUPROCESSDESC_HPP */
