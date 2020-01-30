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
 * *  @file fire9_2nd_s1_graph.hpp
 * *  @brief Graph for FIRE9_2ND_S1 
 * ***********************************************************************************/
#ifndef FIRE92NDS1GRAPH_HPP
#define FIRE92NDS1GRAPH_HPP

#include <acf_graph.hpp>
#include "neural_network_acf.h"

class fire9_2nd_s1_graph : public ACF_Graph
{
public:
   fire9_2nd_s1_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_FIRE9_2ND_S1_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("fire9_2nd_s1_graph");

      /* add kernels */
      AddKernel("FIRE9_2ND_S1", APU_FIRE9_2ND_S1_FORWARD_KN);

      /* add graph port */
      AddInputPort("FIRE9_2ND_S1_INPUT_IMAGE");
      AddInputPort("FIRE9_2ND_S1_INPUT_WEIGHT");
      AddInputPort("FIRE9_2ND_S1_INPUT_PARAMS");
      AddOutputPort("FIRE9_2ND_S1_OUTPUT_S1");


      /* specify connection */
      Connect(GraphPort("FIRE9_2ND_S1_INPUT_IMAGE"),   KernelPort("FIRE9_2ND_S1", "INPUT_IMAGE"));
      Connect(GraphPort("FIRE9_2ND_S1_INPUT_WEIGHT"),  KernelPort("FIRE9_2ND_S1", "INPUT_WEIGHT"));
      Connect(GraphPort("FIRE9_2ND_S1_INPUT_PARAMS"),  KernelPort("FIRE9_2ND_S1", "INPUT_PARAMS"));

      Connect(KernelPort("FIRE9_2ND_S1", "OUTPUT_S1"), GraphPort("FIRE9_2ND_S1_OUTPUT_S1"));
   }
};

#endif /* FIRE92NDS1GRAPH_HPP */

