/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright 2018 NXP 
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
*  @file gaussian_9x9_08u_graph.hpp
*  @brief ACF graph for gaussian 9x9 (unsigned 8 bit)
***********************************************************************************/
 
#ifndef GAUSSIAN9X908UGRAPH_HPP
#define GAUSSIAN9X908UGRAPH_HPP

#include <acf_graph.hpp>
#include "image_filters_acf.h"
#include "graph_templates.h"

class gaussian_9x9_08u_graph : public ACF_Graph
{
public:
  gaussian_9x9_08u_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(GAUSSIAN_9X1_08U_K);
    XREGISTER_ACF_KERNEL(GAUSSIAN_1X9_08U_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("gaussian_9x9_08u_graph");
      
      const std::string KERNEL1 = "kernel1";
      const std::string KERNEL2 = "kernel2";
      
      //add kernels
      AddKernel(KERNEL1, GAUSSIAN_9X1_08U_KN);
      AddKernel(KERNEL2, GAUSSIAN_1X9_08U_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort(KERNEL1, INPUT_0));
      Connect(KernelPort(KERNEL1, OUTPUT_0),   KernelPort(KERNEL2, INPUT_0));
      Connect(KernelPort(KERNEL2, OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* GAUSSIAN9X908UGRAPH_HPP */