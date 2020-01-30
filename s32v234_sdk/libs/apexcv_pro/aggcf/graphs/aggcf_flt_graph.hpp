/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor
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

#ifndef AGGCFFLTGRAPH_HPP
#define AGGCFFLTGRAPH_HPP

#include <acf_graph.hpp>
#include "aggcf_acf.h"


class aggcf_flt_graph : public ACF_Graph
{
public:
   aggcf_flt_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(AGG_ACF_FLT_K)
   }

   void Create()
   {
      // Set identifier for graph
      SetIdentifier("aggcf_flt_graph");

      // add kernels
      AddKernel("AGGCF_FLT3x3", AGG_ACF_FLT_KN);

      // add graph port
      AddInputPort("INPUT");
      AddInputPort("PARAMS");
      
      AddOutputPort("OUTPUT");
      

      // specify connections
      Connect(  GraphPort("INPUT"),                     KernelPort("AGGCF_FLT3x3", "INPUT"));
      Connect(  GraphPort("PARAMS"),                    KernelPort("AGGCF_FLT3x3", "PARAMS"));
      Connect(  KernelPort("AGGCF_FLT3x3", "OUTPUT"),   GraphPort("OUTPUT"));

   }
};

#endif /* AGGCFFLTGRAPH_HPP */

