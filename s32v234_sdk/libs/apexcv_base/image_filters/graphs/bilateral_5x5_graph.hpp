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
****************************************************************************/
  
/*!*********************************************************************************
*  @file bilateral_5x5_graph.hpp
*  @brief ACF graph for bilateral 5x5 filter
***********************************************************************************/
#ifndef BILATERAL5X5GRAPH_HPP
#define BILATERAL5X5GRAPH_HPP

#include <acf_graph.hpp>
#include "image_filters_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"

class  bilateral_5x5_graph: public ACF_Graph
{
public:
  bilateral_5x5_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(BILATERAL_5X5_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("bilateral_5x5_graph");
      
      //add kernels
      AddKernel("_bilateral_5x5", BILATERAL_5X5_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddInputPort("SIGMA_R");
      AddInputPort("SIGMA_D");
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),  KernelPort("_bilateral_5x5", INPUT_0));
    Connect(GraphPort("SIGMA_D"), KernelPort("_bilateral_5x5", SIGMA_D));
      Connect(GraphPort("SIGMA_R"), KernelPort("_bilateral_5x5", SIGMA_R));

      Connect(KernelPort("_bilateral_5x5", OUTPUT_0),  GraphPort(GR_OUT_0));
   }
};
#endif /* BILATERAL5X5GRAPH_HPP */

