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
#ifndef APUFASTNMSGRAPH_HPP
#define APUFASTNMSGRAPH_HPP

#include <acf_graph.hpp>
#include "fast9_acf.h"
#include "nms_acf.h"
#include "apu_fast_graph_names.h"

class apu_fast_nms_graph : public ACF_Graph
{
public:
  const char *m_graphName;

public:
  apu_fast_nms_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(FAST9_K);
    XREGISTER_ACF_KERNEL(NMS_K);
  }

  void Create()
  {
    // Set identifier for graph
    SetIdentifier("fast_graph_nms");

    // add kernels
    std::string lFastKnInst("FAST"), lNMSKnInst("NMS");
    AddKernel(lFastKnInst, FAST9_KN);
    AddKernel(lNMSKnInst, NMS_KN);

    // add graph port
    AddInputPort(FAST9_GR_IN_IMG);
    AddInputPort(FAST9_GR_IN_THR);
    AddOutputPort(FAST9_GR_OUT);

    // specify connections
    Connect(GraphPort(FAST9_GR_IN_IMG),
            KernelPort(lFastKnInst, FAST9_KN_IN_IMG));
    Connect(GraphPort(FAST9_GR_IN_THR),
            KernelPort(lFastKnInst, FAST9_KN_IN_THR));

    Connect(KernelPort(lFastKnInst, FAST9_KN_OUT),
            KernelPort(lNMSKnInst, NMS_KN_IN));

    // NMS
    Connect(KernelPort(lNMSKnInst, NMS16_KN_OUT), GraphPort(FAST9_GR_OUT));
  }
};

#endif /* APUFASTNMSGRAPH_HPP */
