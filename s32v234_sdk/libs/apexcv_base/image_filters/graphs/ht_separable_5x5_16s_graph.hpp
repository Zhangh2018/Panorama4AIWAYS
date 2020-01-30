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
*  @file ht_separable_5x5_16s_graph.hpp
*  @brief ACF graph for hand-tuned separable 5x5 filter
***********************************************************************************/
 
#ifndef HTSEPARABLE5X516SGRAPH_HPP
#define HTSEPARABLE5X516SGRAPH_HPP

#include <acf_graph.hpp>
#include "image_filters_acf.h"
#include "graph_templates.h"

class ht_separable_5x5_16s_graph : public ACF_Graph
{
public:
  ht_separable_5x5_16s_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(HT_SEPARABLE_5X5_16S_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("ht_separable_5x5_16s_graph");
      
      //add kernels
      AddKernel("_ht_separable_5x5_16s", HT_SEPARABLE_5X5_16S_KN);

      //add graph ports
     AddInputPort(GR_IN_0);
      AddInputPort("F_ROW");
      AddInputPort("F_COL");
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_ht_separable_5x5_16s", INPUT_0));
      Connect(GraphPort("F_ROW"),     KernelPort("_ht_separable_5x5_16s", "F_ROW"));
      Connect(GraphPort("F_COL"),     KernelPort("_ht_separable_5x5_16s", "F_COL"));
      Connect(KernelPort("_ht_separable_5x5_16s", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};
#endif /* HTSEPARABLE5X516SGRAPH_HPP */
