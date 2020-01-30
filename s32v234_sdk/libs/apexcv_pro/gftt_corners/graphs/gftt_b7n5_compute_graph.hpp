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
#ifndef GFTTCOMPUTEGRAPH_HPP
#define GFTTCOMPUTEGRAPH_HPP

#include <acf_graph.hpp>
#include "gftt_acf.h"
#include "apexcv_pro_gftt_harris_common.h"


/*!*********************************************************************************
*  \brief ACF graph for the \ref secGFTTCornerDetector compute corner strength 
*         using linear filter, box filter size 7, and non-maxima suppression size 5.
*         Graph also extract the first 4096 highest corners. 
*
*
*  \image html   gftt_corners_detect_graph.jpg "ACF Graph for the gftt compute and extract"
*  \image latex  gftt_corners_detect_graph.pdf "ACF Graph for the gftt compute and extract." width = 14cm
***********************************************************************************/
class gftt_b7n5_compute_graph : public ACF_Graph
{
public:
	gftt_b7n5_compute_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(GFTT_WRAPPER_BOX7_NMS5_K)
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
      SetIdentifier("gftt_b7n5_compute_graph");

      // add kernels
      AddKernel("GFTT",   GFTT_WRAPPER_BOX7_NMS5_KN);

      SetKernelPortOutputDelay("GFTT", "OUTPUT", PORT_OUTPUT_DELAY_BOX7_NMS5);

      // add graph port
      AddInputPort("INPUT");
      AddInputPort("PARAMS");
      AddOutputPort("OUTPUT_IMAGE");
      // specify connections
      // gftt
      Connect(  GraphPort("INPUT"),                     KernelPort("GFTT", "INPUT"));
      Connect(  GraphPort("PARAMS"),                    KernelPort("GFTT", "PARAMS"));

      // graph out
      Connect(  KernelPort("GFTT", "OUTPUT"),            GraphPort("OUTPUT_IMAGE"));
   }
};

#endif /* GFTTCOMPUTEGRAPH_HPP */
