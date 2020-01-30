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
*  @file bicubic.hpp
*  @brief ACF graph for bicubic 08u interpolation
***********************************************************************************/
#ifndef INTERPBICUBICGRAYSCALEGRAPH_H
#define INTERPBICUBICGRAYSCALEGRAPH_H

#include <acf_graph.hpp>
#include "interpolation_acf.h"
class interp_bicubic_grayscale_graph : public ACF_Graph
{
public:
  interp_bicubic_grayscale_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(INTERP_BICUBIC_GRAYSC_K);
  }
   void Create()
   {
      //set identifier for graph
      SetIdentifier("interp_bicubic_grayscale_graph");

      //add kernels      
    AddKernel("_interp_bicubic_grayscale", INTERP_BICUBIC_GRAYSC_KN);

      //add graph ports
      AddInputPort("INPUT_0");
      AddInputPort("Y_OFFSET");
      AddInputPort("X_OFFSET");
      AddOutputPort("OUTPUT_0");

      //specify connections
      Connect(GraphPort("INPUT_0"),    KernelPort("_interp_bicubic_grayscale", "INPUT_0"));
      Connect(GraphPort("Y_OFFSET"),   KernelPort("_interp_bicubic_grayscale", "X_OFFSET"));
      Connect(GraphPort("X_OFFSET"),   KernelPort("_interp_bicubic_grayscale", "Y_OFFSET"));
      Connect(KernelPort("_interp_bicubic_grayscale", "OUTPUT_0"), GraphPort("OUTPUT_0"));
   }
};

#endif /* INTERPBICUBICGRAYSCALEGRAPH_H */
