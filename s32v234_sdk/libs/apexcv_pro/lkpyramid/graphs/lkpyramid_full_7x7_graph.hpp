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
 * *  @file lkpyramid_full_7x7_graph.hpp
 * *  @brief Graph for L-K Pyramid full graph 7x7 window
 * ***********************************************************************************/

#ifndef LKPYRAMIDFULL7X7GRAPH_HPP
#define LKPYRAMIDFULL7X7GRAPH_HPP

#include <acf_graph.hpp>
#include "lkpyramid_acf.h"

class lkpyramid_full_7x7_graph : public ACF_Graph
{
public:
   lkpyramid_full_7x7_graph() : ACF_Graph()
   {
       XREGISTER_ACF_KERNEL(LKPYRAMID_TEMPL_BILINEAR_08U_7X7_K)
       XREGISTER_ACF_KERNEL(LKPYRAMID_HT_CENTRALDXDY_7X7_K)
       XREGISTER_ACF_KERNEL(LKPYRAMID_CORE_7X7_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("lkpyramid_full_7x7_graph");

		/* add kernels */
		AddKernel("TEMPLATE_BILINEAR_7X7", LKPYRAMID_TEMPL_BILINEAR_08U_7X7_KN);
		AddKernel("TEMPLATE_DXDY_7X7",     LKPYRAMID_HT_CENTRALDXDY_7X7_KN);
		AddKernel("CORE_7X7",              LKPYRAMID_CORE_7X7_KN);

		/* add graph port */
		AddInputPort("INPUT_TEMPLATE");
		AddInputPort("INPUT_PATCH");
		AddInputPort("INPUT_COOR_XY0");
      AddInputPort("PARAMS");            // Bytes [0-3]: Full span of source image; [4]: NUM_ITER; [5-7]: K_0, K_1, K_2
		AddInputPort("INPUT_COOR_XY1");
		AddInputPort("INPUT_OFFSET_XY1");
		AddInputPort("TEMPL_COOR_NEXT");
		AddOutputPort("OUTPUT_COOR_XY1");
		AddOutputPort("OUTPUT_OFFSET_XY1");
		AddOutputPort("OUTPUT_TEMPL_OFFSET_NEXT");

		/* specify connection */
		Connect(GraphPort("INPUT_TEMPLATE"),                     KernelPort("TEMPLATE_BILINEAR_7X7", "INPUT_0"));
		Connect(GraphPort("INPUT_COOR_XY0"),                     KernelPort("TEMPLATE_BILINEAR_7X7", "INPUT_1"));
		Connect(KernelPort("TEMPLATE_BILINEAR_7X7", "OUTPUT_0"), KernelPort("TEMPLATE_DXDY_7X7", "INPUT_0"));
		Connect(GraphPort("PARAMS"),                             KernelPort("TEMPLATE_DXDY_7X7", "INPUT_1"));

		Connect(GraphPort("INPUT_PATCH"),                        KernelPort("CORE_7X7", "INPUT_0"));
		Connect(KernelPort("TEMPLATE_BILINEAR_7X7", "OUTPUT_0"), KernelPort("CORE_7X7", "INPUT_1"));
		Connect(KernelPort("TEMPLATE_DXDY_7X7", "OUTPUT_0"),     KernelPort("CORE_7X7", "INPUT_2"));
		Connect(KernelPort("TEMPLATE_DXDY_7X7", "OUTPUT_1"),     KernelPort("CORE_7X7", "INPUT_3"));
		Connect(GraphPort("INPUT_COOR_XY1"),                     KernelPort("CORE_7X7", "INPUT_4"));

		Connect(GraphPort("INPUT_OFFSET_XY1"),                   KernelPort("CORE_7X7", "INPUT_5"));
		Connect(GraphPort("PARAMS"),                             KernelPort("CORE_7X7", "INPUT_6"));
		Connect(GraphPort("TEMPL_COOR_NEXT"),                    KernelPort("CORE_7X7", "INPUT_7"));


		Connect(KernelPort("CORE_7X7", "OUTPUT_0"), GraphPort("OUTPUT_COOR_XY1"));
		Connect(KernelPort("CORE_7X7", "OUTPUT_1"), GraphPort("OUTPUT_OFFSET_XY1"));
		Connect(KernelPort("CORE_7X7", "OUTPUT_2"), GraphPort("OUTPUT_TEMPL_OFFSET_NEXT"));
   }
};

#endif /* LKPYRAMIDFULL7X7GRAPH_HPP */

