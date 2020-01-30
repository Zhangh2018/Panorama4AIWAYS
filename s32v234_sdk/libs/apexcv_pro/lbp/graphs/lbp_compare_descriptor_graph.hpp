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
#ifndef LBPCOMPAREDESCRIPTORGRAPH_HPP
#define LBPCOMPAREDESCRIPTORGRAPH_HPP

#include <acf_graph.hpp>
#include "lbp_acf.h"

class lbp_compare_descriptor_graph : public ACF_Graph
{
public:
   lbp_compare_descriptor_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(COMPARE_DESCRIPTOR_K)
   }

   void Create()
   {
      // set identifier for graph
      SetIdentifier("lbp_compare_descriptor_graph");
      
      // add kernels
      AddKernel("COMPARE_DESCRIPTOR",  COMPARE_DESCRIPTOR_KN);
      
      // add graph ports
      AddInputPort("INPUT_MODEL");
      AddInputPort("INPUT_TEST");
      AddOutputPort("CLOSEST_ID");
      AddOutputPort("DISTANCE");
      
      // specify connections
      Connect( GraphPort("INPUT_MODEL"),     KernelPort("COMPARE_DESCRIPTOR", "INPUT_MODEL"));
      Connect( GraphPort("INPUT_TEST"),      KernelPort("COMPARE_DESCRIPTOR", "INPUT_TEST"));
      
      Connect( KernelPort("COMPARE_DESCRIPTOR", "CLOSEST_ID"), GraphPort("CLOSEST_ID"));
      Connect( KernelPort("COMPARE_DESCRIPTOR", "DISTANCE"),   GraphPort("DISTANCE"));
   }
};

#endif /* LBPCOMPAREDESCRIPTORGRAPH_HPP */
