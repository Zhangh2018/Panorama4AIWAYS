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
#ifndef REMAPBLOCKSIZEGRAPH_HPP
#define REMAPBLOCKSIZEGRAPH_HPP


#include <acf_graph.hpp>
#include "remap_acf.h"

class remap_block_size_graph : public ACF_Graph
{
public:
   remap_block_size_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(REMAP_BLOCK_SIZE_K)
   }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("remap_block_size");

      //add kernels
      AddKernel("block_size", REMAP_BLOCK_SIZE_KN);

      //add graph ports
      AddInputPort("MAP");
      AddOutputPort("TWO_HEIGHT");
      AddOutputPort("FOUR_HEIGHT");
      AddOutputPort("EIGHT_HEIGHT");
      AddOutputPort("SIXTEEN_HEIGHT");

      //specify connections
      Connect(GraphPort("MAP"), KernelPort("block_size", "MAP"));
      Connect(KernelPort("block_size", "TWO_HEIGHT"), GraphPort("TWO_HEIGHT"));
      Connect(KernelPort("block_size", "FOUR_HEIGHT"), GraphPort("FOUR_HEIGHT"));
      Connect(KernelPort("block_size", "EIGHT_HEIGHT"), GraphPort("EIGHT_HEIGHT"));
      Connect(KernelPort("block_size", "SIXTEEN_HEIGHT"), GraphPort("SIXTEEN_HEIGHT"));
   }
};

#endif /* REMAPBLOCKSIZEGRAPH_HPP */

