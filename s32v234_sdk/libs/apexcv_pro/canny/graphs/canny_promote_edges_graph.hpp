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
*  \file canny_promote_edges_graph.hpp
*  \brief Graph for Canny edge promotion - Only checks block padding
***********************************************************************************/
#ifndef CANNYPROMOTEEDGESGRAPH_HPP
#define CANNYPROMOTEEDGESGRAPH_HPP


#include <acf_graph.hpp>
#include "canny_acf.h"

class canny_promote_edges_graph : public ACF_Graph
{
public:
   canny_promote_edges_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(CANNY_PROMOTE_EDGES_K)
   }

   void Create()
   {
      SetIdentifier("canny_promote_edges_graph");
      
      AddKernel("EDGE0",      CANNY_PROMOTE_EDGES_KN);
      AddKernel("EDGE1",      CANNY_PROMOTE_EDGES_KN);
      AddKernel("EDGE2",      CANNY_PROMOTE_EDGES_KN);
      AddKernel("EDGE3",      CANNY_PROMOTE_EDGES_KN);

      AddInputPort("INPUT");
      AddOutputPort("OUTPUT");

      Connect( GraphPort(           "INPUT"),   KernelPort("EDGE0",  "INPUT"));
      Connect( KernelPort("EDGE0",  "OUTPUT"),  KernelPort("EDGE1",  "INPUT"));
      Connect( KernelPort("EDGE1",  "OUTPUT"),  KernelPort("EDGE2",  "INPUT"));
      Connect( KernelPort("EDGE2",  "OUTPUT"),  KernelPort("EDGE3",  "INPUT"));
      Connect( KernelPort("EDGE3",  "OUTPUT"),  GraphPort("OUTPUT"));
   }
};

#endif /* CANNYPROMOTEEDGESGRAPH_HPP */

