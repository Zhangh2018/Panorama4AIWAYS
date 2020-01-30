/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2014-2016 Freescale Semiconductor
 * Copyright 2017 NXP
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
#ifndef APUFASTGRAPH_HPP
#define APUFASTGRAPH_HPP

#include <acf_kernel.hpp>
#include <acf_graph.hpp>
#include "apu_fast_graph_names.h"

#include "fast9_acf.h"
#include <common_stringify_macros.h>

class apu_fast_graph : public ACF_Graph
{
public:
  const char *m_graphName;

public:
  apu_fast_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(FAST9_K);
  }

  void Create()
  {
    // Set identifier for graph
    SetIdentifier(FAST9_GR_STR);

    // add kernels
    std::string fastKnInst("FAST");
    AddKernel(fastKnInst, FAST9_KN);

    // add graph port
    AddInputPort(FAST9_GR_IN_IMG);
    AddInputPort(FAST9_GR_IN_THR);
    AddOutputPort(FAST9_GR_OUT);

    // specify connections
    // connecting offset kernels inputs
    Connect(GraphPort(FAST9_GR_IN_IMG),
            KernelPort(fastKnInst, FAST9_KN_IN_IMG));

    Connect(GraphPort(FAST9_GR_IN_THR),
            KernelPort(fastKnInst, FAST9_KN_IN_THR));

    // connecting fast kernel output to graphs output
    Connect(KernelPort(fastKnInst, FAST9_KN_OUT), GraphPort(FAST9_GR_OUT));
  }
};

#endif /* APUFASTGRAPH_HPP */
