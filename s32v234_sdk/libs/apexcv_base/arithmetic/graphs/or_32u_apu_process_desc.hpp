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
*  @file or_32u_proc_desc.hpp
*  @brief Process description for bitwise or (unsigned 32 bit)
***********************************************************************************/
#ifndef OR32UAPUPROCESSDESC_HPP
#define OR32UAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>

#include "arithmetic_acf.h"
#include <acf_graph.hpp>
#include "graph_templates.h"

GRAPH_CLASS_DEF_2In_1Out(OR_32u_K);

#define OR_32U_P_ID    OR_32U
#define OR_32U_P_STR  XSTR(OR_32U_P_ID)

class or_32u_apu_process_desc : public ACF_Process_Desc_APU
{
public:

  void Create()
  {
    Initialize(mGraph, OR_32U_P_STR);
  }
  graphClassName(OR_32u_K) mGraph;
};


#ifdef APEX2_EMULATE
REGISTER_PROCESS_TYPE(OR_32U_P_ID, or_32u_apu_process_desc)
#endif

#endif /* OR32UAPUPROCESSDESC_HPP */
