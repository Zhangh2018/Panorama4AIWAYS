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
 * *  @file fire10_2nd_conv10ap_top_graph.hpp
 * *  @brief Graph for FIRE10_2ND_CONV10AP_TOP
 * ***********************************************************************************/
#ifndef FIRE102NDCONV10APTOPGRAPH_HPP
#define FIRE102NDCONV10APTOPGRAPH_HPP

#include <acf_graph.hpp>
#include "neural_network_acf.h"

class fire10_2nd_conv10ap_top_graph : public ACF_Graph
{
public:
   fire10_2nd_conv10ap_top_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_FIRE10_2ND_CONV10AP_TOP_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("fire10_2nd_conv10ap_top_graph");

      /* add kernels */
      AddKernel("FIRE10_2ND_CONV10AP_TOP", APU_FIRE10_2ND_CONV10AP_TOP_FORWARD_KN);

      /* add graph port */
      AddInputPort("FIRE10_2ND_CONV10AP_TOP_INPUT_IMAGE");
      AddInputPort("FIRE10_2ND_CONV10AP_TOP_INPUT_WEIGHT");
      AddInputPort("FIRE10_2ND_CONV10AP_TOP_INPUT_ACCM");
      AddInputPort("FIRE10_2ND_CONV10AP_TOP_INPUT_PARAMS");
      AddOutputPort("FIRE10_2ND_CONV10AP_TOP_OUTPUT_AP");


      /* specify connection */
      Connect(GraphPort("FIRE10_2ND_CONV10AP_TOP_INPUT_IMAGE"),   KernelPort("FIRE10_2ND_CONV10AP_TOP", "INPUT_IMAGE"));
      Connect(GraphPort("FIRE10_2ND_CONV10AP_TOP_INPUT_WEIGHT"),  KernelPort("FIRE10_2ND_CONV10AP_TOP", "INPUT_WEIGHT"));
      Connect(GraphPort("FIRE10_2ND_CONV10AP_TOP_INPUT_ACCM"),    KernelPort("FIRE10_2ND_CONV10AP_TOP", "INPUT_ACCM"));
      Connect(GraphPort("FIRE10_2ND_CONV10AP_TOP_INPUT_PARAMS"),  KernelPort("FIRE10_2ND_CONV10AP_TOP", "INPUT_PARAMS"));

      Connect(KernelPort("FIRE10_2ND_CONV10AP_TOP", "OUTPUT_AP"), GraphPort("FIRE10_2ND_CONV10AP_TOP_OUTPUT_AP"));
   }
};

#endif /* FIRE102NDCONV10APTOPGRAPH_HPP */

