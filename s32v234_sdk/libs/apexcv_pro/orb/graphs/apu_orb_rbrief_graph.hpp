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

#ifndef APUORBRBRIEFGRAPH_HPP
#define APUORBRBRIEFGRAPH_HPP

#include <acf_graph.hpp>
#include "orb_acf.h"

class apu_orb_rbrief_graph : public ACF_Graph
{
public:
apu_orb_rbrief_graph() : ACF_Graph()
{
  XREGISTER_ACF_KERNEL(ORB_RBRIEF_K)
}

void Create()
{
  // Set identifier for graph
  SetIdentifier("apu_orb_rbrief_graph");

  // Add kernels
  AddKernel("_orb_rbrief", ORB_RBRIEF_KN);

  // Add graph port
  AddInputPort("INPUT");
  AddInputPort("ORIENTATION");
  AddInputPort("BIT_PATTERN");
  AddInputPort("DESCR_SIZE_B");
  AddInputPort("PATCH_SIZE");

  AddOutputPort("OUTPUT");
  AddOutputPort("COUNT_DESCR");

  // Input connection
  Connect(GraphPort("INPUT"), KernelPort("_orb_rbrief", "INPUT"));
  Connect(GraphPort("ORIENTATION"), KernelPort("_orb_rbrief", "ORIENTATION"));
  Connect(GraphPort("BIT_PATTERN"), KernelPort("_orb_rbrief", "BIT_PATTERN"));
  Connect(GraphPort("DESCR_SIZE_B"), KernelPort("_orb_rbrief", "DESCRIPTOR_SIZE_B"));
  Connect(GraphPort("PATCH_SIZE"), KernelPort("_orb_rbrief", "PATCH_SIZE"));

  // Output connection
  Connect(KernelPort("_orb_rbrief", "OUTPUT"), GraphPort("OUTPUT"));
  Connect(KernelPort("_orb_rbrief", "COUNT"), GraphPort("COUNT_DESCR"));
}
};
#endif /* APUORBRBRIEFGRAPH_HPP */
