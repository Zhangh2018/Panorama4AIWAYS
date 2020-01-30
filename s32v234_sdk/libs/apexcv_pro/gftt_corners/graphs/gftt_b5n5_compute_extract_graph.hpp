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
*  @file
*  @brief ACF graph for the \ref secGFTTCorner "GFTT Compute and Extract".
***********************************************************************************/
#ifndef GFTTCOMPUTEEXTRACTGRAPH_HPP
#define GFTTCOMPUTEEXTRACTGRAPH_HPP

#include <acf_graph.hpp>
#include "gftt_acf.h"
#include "apexcv_pro_gftt_harris_common.h"


/*!*********************************************************************************
*  \brief ACF graph for the \ref secGFTTCornerDetector compute corner strength
*         using linear filter, box filter size 5, and non-maxima suppression size 5.
*         Graph also extract the first 4096 highest corners.
*
*
*  \image html   gftt_corners_detect_graph.jpg "ACF Graph for the gftt compute and extract"
*  \image latex  gftt_corners_detect_graph.pdf "ACF Graph for the gftt compute and extract." width = 14cm
***********************************************************************************/
class gftt_b5n5_compute_extract_graph : public ACF_Graph
{
public:
	gftt_b5n5_compute_extract_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(GFTT_WRAPPER_BOX5_NMS5_K)
      XREGISTER_ACF_KERNEL(GFTT_EXTRACT_K)
      XREGISTER_ACF_KERNEL(GFTT_SORT_AND_FILTER_K)
   }

   /*!*********************************************************************************
   *  \brief Create the ACF graph.
   *
   *  In this function we
   *    - set the graph's unique identifier.
   *    - add the kernels
   *    - add graph ports for the source image , corner coefficient and destination image
   *    - connect graph ports to kernel ports
   ***********************************************************************************/
   void Create()
   {
      // Set identifier for graph
      SetIdentifier("gftt_b5n5_compute_extract_graph");

      // add kernels
      AddKernel("GFTT",   GFTT_WRAPPER_BOX5_NMS5_KN);
      AddKernel("EXTRACT",  GFTT_EXTRACT_KN);
      AddKernel("SORT",     GFTT_SORT_AND_FILTER_KN);

      SetKernelPortOutputDelay("GFTT", "OUTPUT", PORT_OUTPUT_DELAY_BOX5_NMS5);

      // add graph port
      AddInputPort("INPUT");
      AddInputPort("PARAMS");
      AddOutputPort("OUTPUT_IMAGE");
      AddOutputPort("OUTPUT_CORNERLIST");
      AddOutputPort("OUTPUT_COUNT");

      // specify connections
      // gftt
      Connect(  GraphPort("INPUT"),                      KernelPort("GFTT", "INPUT"));
      Connect(  GraphPort("PARAMS"),                     KernelPort("GFTT", "PARAMS"));
      // extract
      Connect(  KernelPort("GFTT", "OUTPUT"),            KernelPort("EXTRACT", "SRC"));
      Connect(  KernelPort("GFTT", "MAX_EIGEN"),         KernelPort("EXTRACT", "MAX_EIGEN"));
      Connect(  GraphPort("PARAMS"),                     KernelPort("EXTRACT", "PARAMS"));
      // sort
      Connect(  GraphPort("PARAMS"),                     KernelPort("SORT", "PARAMS"));
      Connect(  KernelPort("EXTRACT", "COORD"),          KernelPort("SORT", "COORD"));
      Connect(  KernelPort("EXTRACT", "STREN"),          KernelPort("SORT", "STREN"));
      Connect(  KernelPort("EXTRACT", "COUNT"),          KernelPort("SORT", "COUNT"));
      // graph out
      Connect(  KernelPort("GFTT", "OUTPUT"),            GraphPort("OUTPUT_IMAGE"));
      Connect(  KernelPort("SORT",  "FEATURE"),          GraphPort("OUTPUT_CORNERLIST"));
      Connect(  KernelPort("SORT",  "FEAT_COUNT"),       GraphPort("OUTPUT_COUNT"));
   }
};

#endif /* GFTTCOMPUTEEXTRACTGRAPH_HPP */
