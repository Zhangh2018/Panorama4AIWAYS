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
  
#ifndef GREYTORGB888GRAPH_HPP
#define GREYTORGB888GRAPH_HPP

#include <acf_graph.hpp>
#include "color_conversion_acf.h"
#include<acf_graph.hpp>
#include "graph_templates.h"
class grey_to_rgb888_graph : public ACF_Graph
{
public:
  grey_to_rgb888_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(GREY_TO_RGB888_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("grey_to_rgb888_graph");
      
      //add kernels
      AddKernel("_grey_to_rgb888", GREY_TO_RGB888_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_grey_to_rgb888", "INPUT_0"));
      Connect(KernelPort("_grey_to_rgb888", GR_OUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* GREYTORGB888GRAPH_HPP */
