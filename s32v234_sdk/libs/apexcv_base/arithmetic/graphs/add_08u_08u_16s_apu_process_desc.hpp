/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2018 NXP 
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
*  @file add_08u_08u_16s_proc_desc.hpp
*  @brief Process description for add (unsigned 8 bit)
***********************************************************************************/
#ifndef ADD08U08U16SAPUPROCESSDESC_HPP
#define ADD08U08U16SAPUPROCESSDESC_HPP

#include <acf_graph.hpp>
#include <acf_process_desc_apu.hpp>
#include "common_stringify_macros.h"
#include <acf_graph.hpp>
#include "arithmetic_acf.h"
#include "graph_templates.h"

GRAPH_CLASS_DEF_3In_1Out(ADD_08u_08u_16s_K);

#define ADD_08U_08U_16S_P_ID    ADD_08U_08U_16S
#define ADD_08U_08U_16S_P_STR  XSTR(ADD_08U_08U_16S_P_ID)

class add_08u_08u_16s_apu_process_desc : public ACF_Process_Desc_APU
{
public:

   void Create()
   {
      Initialize(mGraph, ADD_08U_08U_16S_P_STR);
   }
   graphClassName(ADD_08u_08u_16s_K) mGraph;

   typedef add_08u_08u_16s_apu_process_desc _procType;
};


#ifdef APEX2_EMULATE
REGISTER_PROCESS_TYPE(ADD_08U_08U_16S_P_ID, add_08u_08u_16s_apu_process_desc)
#endif


#endif /* ADD08U08U16SAPUPROCESSDESC_HPP */
