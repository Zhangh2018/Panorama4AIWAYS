/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

/*!*****************************************************************************
*  \file
*  \brief ACF graph for the \ref secBrief "BRIEF".
*******************************************************************************/
#ifndef COMPUTEBRIEFDESCRIPTOR16GRAPH_HPP
#define COMPUTEBRIEFDESCRIPTOR16GRAPH_HPP

#include <acf_graph.hpp>
#include "brief_acf.h"


class compute_brief_descriptor_graph : public ACF_Graph
{
public:
compute_brief_descriptor_graph() : ACF_Graph()
{
  XREGISTER_ACF_KERNEL(COMPUTE_BRIEF_DESCRIPTOR_K)
}

void Create()
{
  // set identifier for graph
  SetIdentifier("compute_brief_descriptor_graph");

  // add kernel
  AddKernel("COMPUTE_DESCRIPTOR", "compute_brief_descriptor");

  // add graph ports
  AddInputPort("INPUT");
  AddInputPort("FILTER_TYPE");
  AddInputPort("SMPL_PACKET");
  AddInputPort("NR_PACKETS_UL");
  AddInputPort("NR_PACKETS_UR");
  AddInputPort("NR_PACKETS_LL");
  AddInputPort("NR_PACKETS_LR");
  AddInputPort("PATCH_SIZE");
  AddInputPort("DESC_SIZE");
  AddOutputPort("COUNT");
  AddOutputPort("DESCRIPTOR_OUT");

  // specify connections
  Connect(GraphPort("INPUT"), KernelPort("COMPUTE_DESCRIPTOR", "INPUT"));
  Connect(GraphPort("FILTER_TYPE"), KernelPort("COMPUTE_DESCRIPTOR", "FILTER_TYPE"));
  Connect(GraphPort("SMPL_PACKET"), KernelPort("COMPUTE_DESCRIPTOR", "SMPL_PACKET"));
  Connect(GraphPort("NR_PACKETS_UL"), KernelPort("COMPUTE_DESCRIPTOR", "NR_PACKETS_UL"));
  Connect(GraphPort("NR_PACKETS_UR"), KernelPort("COMPUTE_DESCRIPTOR", "NR_PACKETS_UR"));
  Connect(GraphPort("NR_PACKETS_LL"), KernelPort("COMPUTE_DESCRIPTOR", "NR_PACKETS_LL"));
  Connect(GraphPort("NR_PACKETS_LR"), KernelPort("COMPUTE_DESCRIPTOR", "NR_PACKETS_LR"));
  Connect(GraphPort("DESC_SIZE"), KernelPort("COMPUTE_DESCRIPTOR", "DESC_SIZE"));
  Connect(GraphPort("PATCH_SIZE"), KernelPort("COMPUTE_DESCRIPTOR", "PATCH_SIZE"));
  Connect(KernelPort("COMPUTE_DESCRIPTOR", "COUNT"), GraphPort("COUNT"));
  Connect(KernelPort("COMPUTE_DESCRIPTOR", "DESCRIPTOR_OUT"), GraphPort("DESCRIPTOR_OUT"));
}
};


#endif /* COMPUTEBRIEFDESCRIPTOR16GRAPH_HPP */


