/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
***************************************************************************
  
/*!*********************************************************************************
*  @file add_08u_graph.hpp
*  @brief ACF graph for add (unsigned 8 bit)
***********************************************************************************/
#ifndef ADD08UGRAPH_HPP
#define ADD08UGRAPH_HPP

#include <acf_graph.hpp>
#include "arithmetic.h"

class add_08u_graph : public ACF_Graph
{
public:
  add_08u_graph() : ACF_Graph()
{
    XREGISTER_ACF_KERNEL(ADD_08u_K);
  }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("add_08u_graph");
      
      //add kernels
      AddKernel("_add_08u", ADD_08u_KN);

      //add graph ports
      AddInputPort("INPUT_0");
      AddInputPort("INPUT_1");

      AddOutputPort("OUTPUT_0");

      //specify connections
      Connect(GraphPort("INPUT_0"),   KernelPort("_add_08u", "INPUT_0"));
      Connect(GraphPort("INPUT_1"),   KernelPort("_add_08u", "INPUT_1"));
      Connect(KernelPort("_add_08u", "OUTPUT_0"), GraphPort("OUTPUT_0"));
   }
};

#endif /* ADD08UGRAPH_HPP */
