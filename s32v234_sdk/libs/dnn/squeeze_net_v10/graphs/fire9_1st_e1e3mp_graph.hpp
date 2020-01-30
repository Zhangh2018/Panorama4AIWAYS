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
 * *  @file fire9_1st_e1e3mp_graph.hpp
 * *  @brief Graph for FIRE9_1ST_E1E3MP 
 * ***********************************************************************************/
#ifndef FIRE91STE1E3MPGRAPH_HPP
#define FIRE91STE1E3MPGRAPH_HPP

#include <acf_graph.hpp>
#include "neural_network_acf.h"

class fire9_1st_e1e3mp_graph : public ACF_Graph
{
public:
   fire9_1st_e1e3mp_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_FIRE9_1ST_E1E3MP_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("fire9_1st_e1e3mp_graph");

      /* add kernels */
      AddKernel("FIRE9_1ST_E1E3MP", APU_FIRE9_1ST_E1E3MP_FORWARD_KN);
      SetKernelPortOutputDelay("FIRE9_1ST_E1E3MP", "OUTPUT_MP1", 1);
      SetKernelPortOutputDelay("FIRE9_1ST_E1E3MP", "OUTPUT_MP3", 1);

      /* add graph port */
      AddInputPort("FIRE9_1ST_E1E3MP_INPUT_IMAGE");
      AddInputPort("FIRE9_1ST_E1E3MP_INPUT_WEIGHT");
      AddInputPort("FIRE9_1ST_E1E3MP_INPUT_PARAMS");
      AddOutputPort("FIRE9_1ST_E1E3MP_OUTPUT_MP1");
      AddOutputPort("FIRE9_1ST_E1E3MP_OUTPUT_MP3");


      /* specify connection */
      Connect(GraphPort("FIRE9_1ST_E1E3MP_INPUT_IMAGE"),   KernelPort("FIRE9_1ST_E1E3MP", "INPUT_IMAGE"));
      Connect(GraphPort("FIRE9_1ST_E1E3MP_INPUT_WEIGHT"),  KernelPort("FIRE9_1ST_E1E3MP", "INPUT_WEIGHT"));
      Connect(GraphPort("FIRE9_1ST_E1E3MP_INPUT_PARAMS"),  KernelPort("FIRE9_1ST_E1E3MP", "INPUT_PARAMS"));

      Connect(KernelPort("FIRE9_1ST_E1E3MP", "OUTPUT_MP1"), GraphPort("FIRE9_1ST_E1E3MP_OUTPUT_MP1"));
      Connect(KernelPort("FIRE9_1ST_E1E3MP", "OUTPUT_MP3"), GraphPort("FIRE9_1ST_E1E3MP_OUTPUT_MP3"));
   }
};

#endif /* FIRE91STE1E3MPGRAPH_HPP */

