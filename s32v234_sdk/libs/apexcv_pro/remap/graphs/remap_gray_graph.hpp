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
#ifndef REMAPBILINEARGRAPH_HPP
#define REMAPBILINEARGRAPH_HPP


#include <acf_graph.hpp>
#include "remap_acf.h"

// Small test case
class remap_bilinear_graph : public ACF_Graph
{
public:
   remap_bilinear_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(REMAP_BILINEAR_GREYSCALE_K)
   }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("remap_bilinear_grayscale");

      //add kernels
      AddKernel("myRemap", REMAP_BILINEAR_GREYSCALE_KN);

      //add graph ports
      AddInputPort("SRC_IMG");
      AddInputPort("LOCAL_OFFSET");
      AddInputPort("DELTA");
      AddOutputPort("DST_IMG");

      //specify connections
      Connect(GraphPort("SRC_IMG"), KernelPort("myRemap", "SRC"));
      Connect(GraphPort("LOCAL_OFFSET"), KernelPort("myRemap", "OFFSET"));
      Connect(GraphPort("DELTA"), KernelPort("myRemap", "DELTA"));
      Connect(KernelPort("myRemap", "DST"), GraphPort("DST_IMG"));
   }
};

#endif /* REMAPBILINEARGRAPH_HPP */

