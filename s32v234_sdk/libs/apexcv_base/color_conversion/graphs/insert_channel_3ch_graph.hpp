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

#ifndef INSERTCHANNEL3CHGRAPH_HPP
#define INSERTCHANNEL3CHGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"


class insert_channel_3ch_graph : public ACF_Graph
{
public:
  insert_channel_3ch_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(INSERT_CHANNEL_3CH_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("insert_channel_3ch_graph");
      
      //add kernels
      const std::string KERNEL1 = "kernel1";
      AddKernel(KERNEL1, INSERT_CHANNEL_3CH_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddInputPort(GR_IN_1);
      AddInputPort(CH_ID);  
  
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0), KernelPort(KERNEL1, INPUT_0));
      Connect(GraphPort(GR_IN_1), KernelPort(KERNEL1, INPUT_1));
      Connect(GraphPort(CH_ID), KernelPort(KERNEL1, CH_ID));
      Connect(KernelPort(KERNEL1, OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* INSERTCHANNEL3CHGRAPH_HPP */
