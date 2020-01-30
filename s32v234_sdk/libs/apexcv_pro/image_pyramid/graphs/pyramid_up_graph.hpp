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
*  \file pyramid_graph.hpp
*  \brief ACF graph for the \ref secImagePyramid "image pyramid creation".
***********************************************************************************/
#ifndef PYRAMIDUPGRAPH_HPP
#define PYRAMIDUPGRAPH_HPP

#include <acf_graph.hpp>
#include "pyramid_acf.h"

class pyramid_up_graph : public ACF_Graph
{
public:
   pyramid_up_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(HORIZONTAL_GAUS_AND_EXPAND_K)
      XREGISTER_ACF_KERNEL(VERTICAL_GAUS_K)
   }

   void Create()
   {
      // set identifier for graph
      SetIdentifier("pyramid_up_graph");
      
      // add kernels
      AddKernel("GAUS_X",     HORIZONTAL_GAUS_AND_EXPAND_KN);
      AddKernel("GAUS_Y",     VERTICAL_GAUS_KN);
      
      // add graph ports
      AddInputPort("INPUT");
      AddOutputPort("OUTPUT");
      
      // specify connections
      Connect( GraphPort("INPUT"),              KernelPort("GAUS_X", "INPUT"));
      Connect( KernelPort("GAUS_X", "OUTPUT"),  KernelPort("GAUS_Y", "INPUT"));
      Connect( KernelPort("GAUS_Y", "OUTPUT"),  GraphPort("OUTPUT"));
   }
   
};


#endif /* PYRAMIDUPGRAPH_HPP */


