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
  

#ifndef BGR888TOGREYGRAPH_HPP
#define BGR888TOGREYGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"
class bgr888_to_grey_graph : public ACF_Graph
{
public:
  bgr888_to_grey_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(BGR888_TO_GREY_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("bgr888_to_grey_graph");
      
      //add kernels
      AddKernel("_bgr888_to_grey", BGR888_TO_GREY_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_bgr888_to_grey", "INPUT_0"));
      Connect(KernelPort("_bgr888_to_grey", GR_OUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* BGR888TOGREYGRAPH_HPP */
