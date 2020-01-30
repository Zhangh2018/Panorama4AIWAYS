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
*  @file integral_image_graph.hpp
*  @brief ACF graph for integral image
***********************************************************************************/
#ifndef INTEGRALIMAGEGRAPH_HPP
#define INTEGRALIMAGEGRAPH_HPP

#include <acf_graph.hpp>
#include "integral_image_acf.h"
#include <acf_graph.hpp>
#include <graph_templates.h>
class integral_image_graph : public ACF_Graph
{
public:
  integral_image_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(INTEGRAL_IMAGE_K);
  }   void Create()
   {
      //set identifier for graph
      SetIdentifier("integral_image_graph");
      
      //add kernels
      AddKernel("_integral_image", INTEGRAL_IMAGE_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),   KernelPort("_integral_image", INPUT_0));
      Connect(KernelPort("_integral_image", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* INTEGRALIMAGEGRAPH_HPP */