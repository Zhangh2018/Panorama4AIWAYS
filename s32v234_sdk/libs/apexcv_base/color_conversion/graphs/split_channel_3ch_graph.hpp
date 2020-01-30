/*****************************************************************************
* 
* NXP Confidential Proprietary
*
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
*  @file 
*  @brief ACF graph
***********************************************************************************/

#ifndef SPLITCHANNEL3CHGRAPH_HPP
#define SPLITCHANNEL3CHGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include "graph_templates.h"


class split_channel_3ch_graph : public ACF_Graph
{
public:
  split_channel_3ch_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(SPLIT_CHANNEL_3CH_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("split_channel_3ch_graph");
      
      //add kernels
      const std::string KERNEL1 = "kernel1";
      AddKernel(KERNEL1, SPLIT_CHANNEL_3CH_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddOutputPort(GR_OUT_0);
      AddOutputPort(GR_OUT_1);
      AddOutputPort(GR_OUT_2);

      //specify connections
      Connect(GraphPort(GR_IN_0), KernelPort(KERNEL1, INPUT_0));
      Connect(KernelPort(KERNEL1, OUTPUT_0), GraphPort(GR_OUT_0));
      Connect(KernelPort(KERNEL1, OUTPUT_1), GraphPort(GR_OUT_1));
      Connect(KernelPort(KERNEL1, OUTPUT_2), GraphPort(GR_OUT_2));
   }
};

#endif /* SPLITCHANNEL3CHGRAPH_HPP */
