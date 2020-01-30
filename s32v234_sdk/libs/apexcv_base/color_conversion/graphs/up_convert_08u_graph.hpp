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
*  @file 
*  @brief ACF graph
***********************************************************************************/

#ifndef UPCONVERT08UGRAPH_HPP
#define UPCONVERT08UGRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include "graph_templates.h"


class up_convert_08u_graph : public ACF_Graph
{
public:
  up_convert_08u_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(UP_CONVERT_08U_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("up_convert_08u_graph");
      
      //add kernels
      const std::string KERNEL1 = "kernel1";
      AddKernel(KERNEL1, UP_CONVERT_08U_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddInputPort(GR_IN_1);
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0), KernelPort(KERNEL1, INPUT_0));
      Connect(GraphPort(GR_IN_1), KernelPort(KERNEL1, INPUT_1));
      Connect(KernelPort(KERNEL1, OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* UPCONVERT08UGRAPH_HPP */
