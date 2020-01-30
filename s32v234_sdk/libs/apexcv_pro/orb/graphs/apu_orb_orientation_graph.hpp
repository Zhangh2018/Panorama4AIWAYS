/*******************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2017 NXP
 * All Rights Reserved
 *
 *******************************************************************************
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
 ******************************************************************************/

#ifndef APUORBORIENTATIONGRAPH_HPP
#define APUORBORIENTATIONGRAPH_HPP

#include <acf_graph.hpp>
#include "orb_acf.h"

class apu_orb_orientation_graph : public ACF_Graph
{
public:
apu_orb_orientation_graph() : ACF_Graph()
{
  XREGISTER_ACF_KERNEL(ORB_ORIENTATION_K)
}

void Create()
{
  // Set identifier for graph
  SetIdentifier("apu_orb_orientation_graph");

  // Add kernels
  AddKernel("_orb_orientation", ORB_ORIENTATION_KN);

  // Add graph port
  AddInputPort("INPUT");
  AddInputPort("PATCH_SIZE");
  AddInputPort("RADIUS");
  AddOutputPort("OUTPUT");

  // Input connection
  Connect(GraphPort("INPUT"), KernelPort("_orb_orientation", "INPUT"));
  Connect(GraphPort("PATCH_SIZE"), KernelPort("_orb_orientation", "PATCH_SIZE"));
  Connect(GraphPort("RADIUS"), KernelPort("_orb_orientation", "RADIUS"));

  // Output connection
  Connect(KernelPort("_orb_orientation", "OUTPUT"), GraphPort("OUTPUT"));
}
};

#endif /* APUORBORIENTATIONGRAPH_HPP */
