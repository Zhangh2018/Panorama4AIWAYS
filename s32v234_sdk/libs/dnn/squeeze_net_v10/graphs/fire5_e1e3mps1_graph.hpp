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
 * *  @file fire5_e1e3mps1_graph.hpp
 * *  @brief Graph for FIRE5_E1E3MPS1 
 * ***********************************************************************************/
#ifndef FIRE5E1E3MPS1GRAPH_HPP
#define FIRE5E1E3MPS1GRAPH_HPP

#include <acf_graph.hpp>
#include "neural_network_acf.h"

class fire5_e1e3mps1_graph : public ACF_Graph
{
public:
   fire5_e1e3mps1_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_FIRE5_E1E3MPS1_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("fire5_e1e3mps1_graph");

      /* add kernels */
      AddKernel("FIRE5_E1E3MPS1", APU_FIRE5_E1E3MPS1_FORWARD_KN);
      SetKernelPortOutputDelay("FIRE5_E1E3MPS1", "OUTPUT_S1", 1);

      /* add graph port */
      AddInputPort("FIRE5_E1E3MPS1_INPUT_IMAGE");
      AddInputPort("FIRE5_E1E3MPS1_INPUT_WEIGHT");
      AddInputPort("FIRE5_E1E3MPS1_INPUT_PARAMS");
      AddOutputPort("FIRE5_E1E3MPS1_OUTPUT_S1");


      /* specify connection */
      Connect(GraphPort("FIRE5_E1E3MPS1_INPUT_IMAGE"),   KernelPort("FIRE5_E1E3MPS1", "INPUT_IMAGE"));
      Connect(GraphPort("FIRE5_E1E3MPS1_INPUT_WEIGHT"),  KernelPort("FIRE5_E1E3MPS1", "INPUT_WEIGHT"));
      Connect(GraphPort("FIRE5_E1E3MPS1_INPUT_PARAMS"),  KernelPort("FIRE5_E1E3MPS1", "INPUT_PARAMS"));

      Connect(KernelPort("FIRE5_E1E3MPS1", "OUTPUT_S1"), GraphPort("FIRE5_E1E3MPS1_OUTPUT_S1"));
   }
};

#endif /* FIRE5E1E3MPS1GRAPH_HPP */

