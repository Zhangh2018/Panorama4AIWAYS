/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
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
 *  @file e1e3mps1_batch_graph.hpp
 *  @brief Graph for E1E3MPS1_MODULE
 * ***********************************************************************************/
#ifndef E1E3MPS1MODULEGRAPH_HPP
#define E1E3MPS1MODULEGRAPH_HPP

#include <acf_graph.hpp>
#include "apexdnn_sn_acf.h"

class e1e3mps1_batch_graph : public ACF_Graph
{
public:
   e1e3mps1_batch_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_E1E3MPS1_MODULE_NHCW_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("e1e3mps1_batch_graph");

      /* add kernels */
      AddKernel("APU_E1E3MPS1_MODULE", APU_E1E3MPS1_MODULE_NHCW_FORWARD_KN);

      /* add graph port */
      AddInputPort ("E1E3MPS1_MODULE_INPUT_IMAGE");
      AddInputPort ("E1E3MPS1_MODULE_INPUT_WEIGHT");
      AddInputPort ("E1E3MPS1_MODULE_INPUT_PARAMS");
      AddOutputPort("E1E3MPS1_MODULE_OUTPUT_S1");

      /* specify connection */
      Connect(GraphPort("E1E3MPS1_MODULE_INPUT_IMAGE"),   KernelPort("APU_E1E3MPS1_MODULE", "INPUT_IMAGE"));
      Connect(GraphPort("E1E3MPS1_MODULE_INPUT_WEIGHT"),  KernelPort("APU_E1E3MPS1_MODULE", "INPUT_WEIGHT"));
      Connect(GraphPort("E1E3MPS1_MODULE_INPUT_PARAMS"),  KernelPort("APU_E1E3MPS1_MODULE", "INPUT_PARAMS"));

      Connect(KernelPort("APU_E1E3MPS1_MODULE", "OUTPUT_S1"), GraphPort("E1E3MPS1_MODULE_OUTPUT_S1"));
   }
};

#endif /* E1E3MPS1MODULEGRAPH_HPP */

