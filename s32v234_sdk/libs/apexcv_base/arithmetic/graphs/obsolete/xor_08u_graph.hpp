/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
***************************************************************************
  
/*!*********************************************************************************
*  @file xor_08u_graph.hpp
*  @brief ACF graph for bitwise xor (unsigned 8 bit)
***********************************************************************************/
#ifndef XOR08UGRAPH_HPP
#define XOR08UGRAPH_HPP

#include <acf_graph.hpp>
#include "arithmetic_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"

class xor_08u_graph : public ACF_Graph
{
public:
  xor_08u_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(XOR_08u_K);
  }
  
   void Create()
   {
      //set identifier for graph
      SetIdentifier("xor_08u_graph");
      
      //add kernels
      AddKernel("_xor_08u", XOR_08u_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddInputPort(GR_IN_1);

      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_xor_08u", GR_IN_0));
      Connect(GraphPort(GR_IN_1),   KernelPort("_xor_08u", GR_IN_1));
      Connect(KernelPort("_xor_08u", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* XOR08UGRAPH_HPP */