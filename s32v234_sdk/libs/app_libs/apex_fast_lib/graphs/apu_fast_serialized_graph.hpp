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
#ifndef APUFASTSERIALIZEDGRAPH_HPP
#define APUFASTSERIALIZEDGRAPH_HPP

#include <acf_graph.hpp>
#include "fast_apu.h"
#include "fast_acf.h"

class apu_fast_serialized_graph : public ACF_Graph
{
public:
  const char *m_graphName;

public:
  apu_fast_serialized_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(FAST_OFFSET_K)
    XREGISTER_ACF_KERNEL(FAST_SERIALIZED_K)
  }

  void Create()
  {
    // Set identifier for graph
    SetIdentifier("fast_serialized_graph");

    // add kernels
    AddKernel("OFFSETS", FAST_OFFSET_KN);
    AddKernel("FAST_SERIALIZED", FAST_SERIALIZED_KN);

    // add graph port
    AddInputPort("INPUT");
    AddInputPort("THRESHOLD");
    AddInputPort("CIRCUMFERENCE");
    AddInputPort("OUT_MAX_SIZE");

    AddOutputPort("OUTPUT");
    AddOutputPort("OUT_COUNTER");

    // specify connections
    // connecting offset kernels inputs
    Connect(GraphPort("INPUT"), KernelPort("OFFSETS", "INPUT_0"));
    Connect(GraphPort("CIRCUMFERENCE"), KernelPort("OFFSETS", "CIRCUMFERENCE"));

    // connecting offset kernel output into fast kernel input
    Connect(KernelPort("OFFSETS", "OUTPUT_OFFSETS"),
            KernelPort("FAST_SERIALIZED", "OFFSET_TABLE"));

    // connecting other fast kernel inputs
    Connect(GraphPort("INPUT"), KernelPort("FAST_SERIALIZED", "INPUT"));
    Connect(GraphPort("THRESHOLD"), KernelPort("FAST_SERIALIZED", "THRESHOLD"));
    Connect(GraphPort("OUT_MAX_SIZE"),
            KernelPort("FAST_SERIALIZED", "OUT_MAX_SIZE"));

    // connecting fast kernel output to graphs output
    Connect(KernelPort("FAST_SERIALIZED", "OUT_PACKED"), GraphPort("OUTPUT"));
    Connect(KernelPort("FAST_SERIALIZED", "COUNTER"), GraphPort("OUT_COUNTER"));
  }
};

#endif /* APUFASTSERIALIZEDGRAPH_HPP */
