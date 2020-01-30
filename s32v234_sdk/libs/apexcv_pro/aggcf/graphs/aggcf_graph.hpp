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

#ifndef AGGCFGRAPH_HPP
#define AGGCFGRAPH_HPP

#include <acf_graph.hpp>
#include "aggcf_acf.h"

class aggcf_graph : public ACF_Graph
{
public:
   aggcf_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(AGG_ACF_K)
   }

   void Create()
   {
      // Set identifier for graph
      SetIdentifier("aggcf_graph");

      // add kernels
      AddKernel("AGGCF", AGG_ACF_KN);

      // add graph port
      AddInputPort("INPUTL");
      AddInputPort("INPUTU");
      AddInputPort("INPUTV");
      
      AddOutputPort("OUTPUTM");
      AddOutputPort("OUTPUTO");
      AddInputPort("PARAMS");
      
      // specify connections
      Connect(  GraphPort("INPUTL"),                     KernelPort("AGGCF", "INPUTL"));
      Connect(  GraphPort("INPUTU"),                     KernelPort("AGGCF", "INPUTU"));
      Connect(  GraphPort("INPUTV"),                     KernelPort("AGGCF", "INPUTV"));
      
      Connect(  KernelPort("AGGCF", "OUTPUTM"),          GraphPort("OUTPUTM")); 
      Connect(  KernelPort("AGGCF", "OUTPUTO"),          GraphPort("OUTPUTO")); 
      Connect(  GraphPort("PARAMS"),                     KernelPort("AGGCF", "PARAMS"));

   }
};

#endif /* AGGCFGRAPH_HPP */

