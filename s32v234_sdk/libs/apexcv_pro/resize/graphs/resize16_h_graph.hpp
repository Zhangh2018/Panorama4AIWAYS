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
*  \file
*  \brief ACF graph definition for the resize implementation.
***********************************************************************************/
#ifndef RESIZE16HGRAPH_HPP
#define RESIZE16HGRAPH_HPP



#include <acf_graph.hpp>
#include "resize_acf.h"


class resize16_h_graph : public ACF_Graph
{
public:

   resize16_h_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(HRSZ16_K)
   }


   void Create()
   {
      //set identifier for graph
      SetIdentifier("resize16_h_graph");
      
      //add kernels
      AddKernel("resize16_horizontal", HRSZ16_KN);

      //add graph ports
      AddInputPort("SRC");
      AddInputPort("SRC_WIDTH");
      AddOutputPort("DST");
      AddInputPort("DST_WIDTH");

      // connect graph ports
      Connect(GraphPort("SRC"),                       KernelPort("resize16_horizontal", "SRC"));
      Connect(GraphPort("SRC_WIDTH"),                 KernelPort("resize16_horizontal", "SRC_WIDTH"));
      Connect(GraphPort("DST_WIDTH"),                 KernelPort("resize16_horizontal", "DST_WIDTH"));
      Connect(KernelPort("resize16_horizontal", "DST"), GraphPort("DST"));
   }
};
     
#endif /* RESIZE16HGRAPH_HPP */
