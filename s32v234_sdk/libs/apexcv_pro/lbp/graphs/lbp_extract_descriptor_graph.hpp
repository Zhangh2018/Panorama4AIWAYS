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
#ifndef LBPEXTRACTDESCRIPTORGRAPH_HPP
#define LBPEXTRACTDESCRIPTORGRAPH_HPP

#include <acf_graph.hpp>
#include "lbp_acf.h"

class lbp_extract_descriptor_graph : public ACF_Graph
{
public:
   lbp_extract_descriptor_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(LBP_EXTRACT_DESCRIPTOR_K)
   }

   void Create()
   {
      // set identifier for graph
      SetIdentifier("lbp_extract_descriptor_graph");
      
      // add kernels
      AddKernel("EXTRACT_DESCRIPTOR",  LBP_EXTRACT_DESCRIPTOR_KN);
      
      // add graph ports
      AddInputPort("INPUT");
      AddInputPort("PROCESS_SIZE");
      AddOutputPort("OUTPUT");
      
      // specify connections
      Connect( GraphPort("INPUT"),        KernelPort("EXTRACT_DESCRIPTOR", "INPUT"));
      Connect( GraphPort("PROCESS_SIZE"), KernelPort("EXTRACT_DESCRIPTOR", "PROCESS_SIZE"));
      
      Connect( KernelPort("EXTRACT_DESCRIPTOR", "OUTPUT"),  GraphPort("OUTPUT"));      
   }
};

#endif /* LBPEXTRACTDESCRIPTORGRAPH_HPP */
