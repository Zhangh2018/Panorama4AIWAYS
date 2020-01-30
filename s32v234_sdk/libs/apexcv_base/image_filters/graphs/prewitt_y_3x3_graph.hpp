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
*  @file prewitt_y_3x3_graph.hpp
*  @brief ACF graph for prewitt y 3x3
***********************************************************************************/
 
#ifndef PREWITTY3X3GRAPH_HPP
#define PREWITTY3X3GRAPH_HPP

#include <acf_graph.hpp>
#include "image_filters_acf.h"
#include "graph_templates.h"

class prewitt_y_3x3_graph : public ACF_Graph
{
public:
  prewitt_y_3x3_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(PREWITT_Y_3X3_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("prewitt_y_3x3_graph");
      
      //add kernels
      AddKernel("_prewitt_y_3x3", PREWITT_Y_3X3_KN);

      //add graph ports
      AddInputPort(GR_IN_0);

      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_prewitt_y_3x3", INPUT_0));
      Connect(KernelPort("_prewitt_y_3x3", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};
#endif /* PREWITTY3X3GRAPH_HPP */
