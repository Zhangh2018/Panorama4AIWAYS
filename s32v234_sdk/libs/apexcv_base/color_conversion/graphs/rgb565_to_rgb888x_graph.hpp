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
  
/*!*********************************************************************************
*  @file rgb565_to_rgb888x_graph.hpp
*  @brief ACF graph for rgb565 to rgb888
***********************************************************************************/
#ifndef RGB565TORGB888XGRAPH_HPP
#define RGB565TORGB888XGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"


class rgb565_to_rgb888x_graph : public ACF_Graph
{
public:
  rgb565_to_rgb888x_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(RGB565_TO_RGB888X_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("rgb565_to_rgb888x_graph");
      
      //add kernels
      AddKernel("_rgb565_to_rgb888x", RGB565_TO_RGB888X_KN);

      //add graph ports
      AddInputPort(GR_IN_0);

      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_rgb565_to_rgb888x", INPUT_0));
      Connect(KernelPort("_rgb565_to_rgb888x", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* RGB565TORGB888XGRAPH_HPP */
