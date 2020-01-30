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
*  \file canny_full_graph.hpp
*  \brief Graph for Canny edge detection
***********************************************************************************/
#ifndef CANNYFULLGRAPH_HPP
#define CANNYFULLGRAPH_HPP


#include <acf_graph.hpp>
#include "canny_acf.h"

class canny_full_graph : public ACF_Graph
{
public:
   canny_full_graph() : ACF_Graph()
   {
       XREGISTER_ACF_KERNEL(CANNY_NMS_PROMOTE_K)
       XREGISTER_ACF_KERNEL(CANNY_PROMOTE_EDGES_K)
       XREGISTER_ACF_KERNEL(CANNY_CREATE_IMAGE_K)
   }

   void Create()
   {
      SetIdentifier("canny_full_graph");
      AddKernel("SUPPRESS",   CANNY_NMS_PROMOTE_KN);
      AddKernel("EDGE0",      CANNY_PROMOTE_EDGES_KN);
      AddKernel("IMAGE",      CANNY_CREATE_IMAGE_KN);
      
      AddInputPort("INPUT");
      AddInputPort("LOW_THRESH");
      AddInputPort("HIGH_THRESH");
      AddOutputPort("OUTPUT");
      AddOutputPort("EDGE_OUTPUT");
      
      Connect( GraphPort("LOW_THRESH"),            KernelPort("SUPPRESS",  "LOW_THRESH"));
      Connect( GraphPort("HIGH_THRESH"),           KernelPort("SUPPRESS",  "HIGH_THRESH"));
      Connect( GraphPort("INPUT"),                 KernelPort("SUPPRESS",  "INPUT"));
      Connect( KernelPort("SUPPRESS",  "OUTPUT"),  KernelPort("EDGE0",  "INPUT"));
      Connect( KernelPort("EDGE0",  "OUTPUT"),     KernelPort("IMAGE",  "INPUT"));
      Connect( KernelPort("EDGE0",  "OUTPUT"),     GraphPort("EDGE_OUTPUT"));
      Connect( KernelPort("IMAGE",  "OUTPUT"),     GraphPort("OUTPUT"));
   }
};

#endif /* CANNYFULLGRAPH_HPP */

