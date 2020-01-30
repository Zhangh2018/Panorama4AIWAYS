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
*  @file interp_bilinear_grayscale_graph.hpp
*  @brief ACF graph for bilinear grayscale 08u interpolation
***********************************************************************************/

#ifndef INTERPBILINEARGRAYSCALEGRAPH_HPP
#define INTERPBILINEARGRAYSCALEGRAPH_HPP

#include <acf_graph.hpp>
#include "interpolation_acf.h"
#include "graph_templates.h"

class interp_bilinear_grayscale_graph : public ACF_Graph
{
public:
  interp_bilinear_grayscale_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(INTERP_BILIN_GRAYSC_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("bilinear_grayscale_graph");
      
      //add kernels
    AddKernel("_interp_bilinear_grayscale", INTERP_BILIN_GRAYSC_KN);

      //add graph ports
      AddInputPort(GR_IN_0);
      AddInputPort("DELTA_XY");
      AddOutputPort(GR_OUT_0);

      //specify connections
      Connect(GraphPort(GR_IN_0),    KernelPort("_interp_bilinear_grayscale", INPUT_0));
      Connect(GraphPort("DELTA_XY"),   KernelPort("_interp_bilinear_grayscale", DELTA_XY));
      Connect(KernelPort("_interp_bilinear_grayscale", OUTPUT_0), GraphPort(GR_OUT_0));
   }
};

#endif /* INTERPBILINEARGRAYSCALEGRAPH_HPP */
