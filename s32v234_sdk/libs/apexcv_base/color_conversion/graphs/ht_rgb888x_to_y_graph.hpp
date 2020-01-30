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
*  @file ht_rgb888x_to_y_graph.hpp
*  @brief ACF graph for ht_8UC3 to y
***********************************************************************************/
#ifndef HTRGB888XTOYGRAPH_HPP
#define HTRGB888XTOYGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"

class ht_rgb888x_to_y_graph : public ACF_Graph
{
public:
  ht_rgb888x_to_y_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(HT_RGB888X_TO_Y_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("ht_rgb888x_to_y_graph");

      //add kernels
      AddKernel("_ht_rgb888x_to_y", HT_RGB888X_TO_Y_KN);

      //add graph ports
      AddInputPort("INPUT_0");
      AddInputPort("KA");
      AddInputPort("KB");
      AddInputPort("KC");
      AddInputPort("KR");
      AddOutputPort("OUTPUT_0");

      //specify connections
      Connect(GraphPort("INPUT_0"),   KernelPort("_ht_rgb888x_to_y", "INPUT_0"));
      Connect(GraphPort("KA"),        KernelPort("_ht_rgb888x_to_y", "KA"));
      Connect(GraphPort("KB"),        KernelPort("_ht_rgb888x_to_y", "KB"));
      Connect(GraphPort("KC"),        KernelPort("_ht_rgb888x_to_y", "KC"));
      Connect(GraphPort("KR"),        KernelPort("_ht_rgb888x_to_y", "KR"));
      Connect(KernelPort("_ht_rgb888x_to_y", "OUTPUT_0"), GraphPort("OUTPUT_0"));
   }
};

#endif /* HTRGB888XTOYGRAPH_HPP */
