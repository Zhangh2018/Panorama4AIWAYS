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

#ifndef AGGCFFLTNORMGRAPH_HPP
#define AGGCFFLTNORMGRAPH_HPP

#include <acf_graph.hpp>
#include "aggcf_acf.h"


class aggcf_flt_norm_graph : public ACF_Graph
{
public:
   aggcf_flt_norm_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(AGG_ACF_FLT_NORM_K)
   }

   void Create()
   {
      // Set identifier for graph
      SetIdentifier("aggcf_flt_norm_graph");

      // add kernels
      AddKernel("AGGCF_FLT3x3_NORM", AGG_ACF_FLT_NORM_KN);

      // add graph port
      AddInputPort("INPUT");
      AddInputPort("PARAMS");
      
      AddOutputPort("OUTPUT");
      

      // specify connections
      Connect(  GraphPort("INPUT"),                     KernelPort("AGGCF_FLT3x3_NORM", "INPUT"));
      Connect(  GraphPort("PARAMS"),                    KernelPort("AGGCF_FLT3x3_NORM", "PARAMS"));
      Connect(  KernelPort("AGGCF_FLT3x3_NORM", "OUTPUT"),   GraphPort("OUTPUT"));

   }
};

#endif /* AGGCFFLTNORMGRAPH_HPP */

