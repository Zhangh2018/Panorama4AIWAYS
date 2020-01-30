/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
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
*  \file pyramid_multi_down_graph.hpp
*  \brief ACF graph for the \ref secImagePyramidMulti "image pyramid_multi creation".
***********************************************************************************/
#ifndef PYRAMIDMULTIDOWNGRAPH_HPP
#define PYRAMIDMULTIDOWNGRAPH_HPP

#include <acf_graph.hpp>
#include "pyramid_acf.h"

class pyramid_multi_down_graph : public ACF_Graph
{
public:
   pyramid_multi_down_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(HORIZONTAL_GAUS_K)
      XREGISTER_ACF_KERNEL(VERTICAL_GAUS_AND_REDUCE_K)
   }

   void Create()
   {
      // set identifier for graph
      SetIdentifier("pyramid_multi_down_graph");

      // add kernels
      AddKernel("GAUS_X_0",  HORIZONTAL_GAUS_KN);
      AddKernel("GAUS_Y_0",  VERTICAL_GAUS_AND_REDUCE_KN);

      AddKernel("GAUS_X_1",  HORIZONTAL_GAUS_KN);
      AddKernel("GAUS_Y_1",  VERTICAL_GAUS_AND_REDUCE_KN);

      // add graph ports
      AddInputPort("INPUT");
      AddOutputPort("OUTPUT_0");
      AddOutputPort("OUTPUT_1");

      // specify connections
      Connect( GraphPort("INPUT"),              KernelPort("GAUS_X_0", "INPUT") );
      Connect( KernelPort("GAUS_X_0", "OUTPUT"),     KernelPort("GAUS_Y_0", "INPUT") );
      Connect( KernelPort("GAUS_Y_0", "OUTPUT"),     KernelPort("GAUS_X_1", "INPUT") );
      Connect( KernelPort("GAUS_X_1", "OUTPUT"),     KernelPort("GAUS_Y_1", "INPUT") );

      Connect( KernelPort("GAUS_Y_0", "OUTPUT"),     GraphPort("OUTPUT_0") );
      Connect( KernelPort("GAUS_Y_1", "OUTPUT"),     GraphPort("OUTPUT_1") );
   }
};


#endif /* PYRAMIDMULTIDOWNGRAPH_HPP */

