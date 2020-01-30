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
#ifndef BLOCKMATCHINGGRAPH_HPP
#define BLOCKMATCHINGGRAPH_HPP

#include <acf_graph.hpp>
#include "blockmatching_acf.h"

class blockmatching_graph : public ACF_Graph
{
public:
   blockmatching_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_BM_SAD_K)
   }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("blockmatching_graph");

      //add kernels
      AddKernel("mySad", APU_BM_SAD_KN);

      //add graph ports
      AddInputPort("INPUT_TEMPLATE");
      AddInputPort("INPUT_WINDOW");
      AddOutputPort("OUTPUT_SCORE");
      AddOutputPort("OUTPUT_POINTS");

      //specify connections
      Connect(GraphPort("INPUT_TEMPLATE"),            KernelPort("mySad", "INPUT_TEMPLATE"));
      Connect(GraphPort("INPUT_WINDOW"),              KernelPort("mySad", "INPUT_WINDOW"));
      Connect(KernelPort("mySad", "OUTPUT_SCORE"),    GraphPort("OUTPUT_SCORE"));
      Connect(KernelPort("mySad", "OUTPUT_POINTS"), GraphPort("OUTPUT_POINTS"));
   }
};

#endif /* BLOCKMATCHINGGRAPH_HPP */
