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
 * *  @file fire10_1st_e1e3_graph.hpp
 * *  @brief Graph for FIRE10_1ST_E1E3 
 * ***********************************************************************************/
#ifndef FIRE101STE1E3GRAPH_HPP
#define FIRE101STE1E3GRAPH_HPP

#include <acf_graph.hpp>
#include "neural_network_acf.h"

class fire10_1st_e1e3_graph : public ACF_Graph
{
public:
   fire10_1st_e1e3_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_FIRE10_1ST_E1E3_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("fire10_1st_e1e3_graph");

      /* add kernels */
      AddKernel("FIRE10_1ST_E1E3", APU_FIRE10_1ST_E1E3_FORWARD_KN);

      /* add graph port */
      AddInputPort("FIRE10_1ST_E1E3_INPUT_IMAGE");
      AddInputPort("FIRE10_1ST_E1E3_INPUT_WEIGHT");
      AddInputPort("FIRE10_1ST_E1E3_INPUT_PARAMS");
      AddOutputPort("FIRE10_1ST_E1E3_OUTPUT_E1");
      AddOutputPort("FIRE10_1ST_E1E3_OUTPUT_E3");


      /* specify connection */
      Connect(GraphPort("FIRE10_1ST_E1E3_INPUT_IMAGE"),   KernelPort("FIRE10_1ST_E1E3", "INPUT_IMAGE"));
      Connect(GraphPort("FIRE10_1ST_E1E3_INPUT_WEIGHT"),  KernelPort("FIRE10_1ST_E1E3", "INPUT_WEIGHT"));
      Connect(GraphPort("FIRE10_1ST_E1E3_INPUT_PARAMS"),  KernelPort("FIRE10_1ST_E1E3", "INPUT_PARAMS"));

      Connect(KernelPort("FIRE10_1ST_E1E3", "OUTPUT_E1"), GraphPort("FIRE10_1ST_E1E3_OUTPUT_E1"));
      Connect(KernelPort("FIRE10_1ST_E1E3", "OUTPUT_E3"), GraphPort("FIRE10_1ST_E1E3_OUTPUT_E3"));
   }
};

#endif /* FIRE101STE1E3GRAPH_HPP */

