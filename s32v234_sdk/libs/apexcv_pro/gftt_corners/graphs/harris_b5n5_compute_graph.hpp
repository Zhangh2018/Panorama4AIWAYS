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
*  @brief ACF graph for the \ref secHarrisCorner "Harris Corner".
***********************************************************************************/
#ifndef HARRISCOMPUTEGRAPH_HPP
#define HARRISCOMPUTEGRAPH_HPP

#include <acf_graph.hpp>
#include "harris_acf.h"
#include "apexcv_pro_gftt_harris_common.h"


/*!*********************************************************************************
*  \brief ACF graph for the \ref secHarrisCorner "Harris Corner" using a 3x3 linear filter,
*         a 5x5 box filter, and a 5x5 non-maxima suppression.
*
*  \image html  harris_corner_graph.jpg "ACF Graph for the harris corner."
*  \image latex harris_corner_graph.pdf "ACF Graph for the harris corner." width = 14cm
***********************************************************************************/
class harris_b5n5_compute_graph : public ACF_Graph
{
public:
   harris_b5n5_compute_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(HARRIS_CORNERS_BOX5_NMS5_K)
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
      SetIdentifier("harris_b5n5_compute_graph");

      // add kernels
      AddKernel("HARRIS", HARRIS_CORNERS_BOX5_NMS5_KN);

      SetKernelPortOutputDelay("HARRIS", "OUTPUT", PORT_OUTPUT_DELAY_BOX5_NMS5);

      // add graph port
      AddInputPort("INPUT");
      AddInputPort("PARAMS");
      AddOutputPort("OUTPUT_IMAGE");

      // specify connections
      Connect(  GraphPort("INPUT"),                     KernelPort("HARRIS", "INPUT"));
      Connect(  GraphPort("PARAMS"),                    KernelPort("HARRIS", "PARAMS"));
      Connect(  KernelPort("HARRIS", "OUTPUT"),         GraphPort("OUTPUT_IMAGE"));
   }
};

#endif /* HARRISCOMPUTEGRAPH_HPP */

