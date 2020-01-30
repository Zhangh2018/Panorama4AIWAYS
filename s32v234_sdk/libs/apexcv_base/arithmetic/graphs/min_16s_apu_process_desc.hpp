/*****************************************************************************
* 
* NXP Confidential Proprietary
*
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
*  @file min_16s_proc_desc.hpp
*  @brief Process description for min (signed 16 bit)
***********************************************************************************/
#ifndef MIN16SAPUPROCESSDESC_HPP
#define MIN16SAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include <acf_graph.hpp>
#include "graph_templates.h"
#include "arithmetic_acf.h"

GRAPH_CLASS_DEF_2In_1Out(MIN_16s_K)

#define MIN_16S_P_ID   MIN_16S
#define MIN_16S_P_STR  XSTR(MIN_16S_P_ID)

class min_16s_apu_process_desc : public ACF_Process_Desc_APU
{
public:
  void Create()
  {
    Initialize(mGraph, MIN_16S_P_STR);
  }

  graphClassName(MIN_16s_K) mGraph;
};


#ifdef APEX2_EMULATE
REGISTER_PROCESS_TYPE(MIN_16S_P_ID, min_16s_apu_process_desc)
#endif

#endif /* MIN16SAPUPROCESSDESC_HPP */
