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
*  \file affine_transform_graph.hpp
*  \brief ACF Affine Transform Graph
***********************************************************************************/

#ifndef AFFINEBILINEARINTERPOLATEGRAPH_HPP
#define AFFINEBILINEARINTERPOLATEGRAPH_HPP
/*!*********************************************************************************
*  @file
*  @brief ACF graph for the \ref secAffineTransform "affine transformation".
***********************************************************************************/
#include <acf_graph.hpp>
#include "affine_transform_acf.h"

/*!*********************************************************************************
*  \brief ACF graph for the \ref secAffineTransform "affine transformation" bilinear interpolation.
*
*  \image html  affine_transform_graph1.jpg "ACF Graph for bilinear interpolation."
*  \image latex affine_transform_graph1.pdf "ACF Graph for bilinear interpolation." width = 14cm
***********************************************************************************/
class affine_bilinear_interpolate_graph : public ACF_Graph
{
public:
   affine_bilinear_interpolate_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(AFFINE_BILINEAR_INTERPOLATE_K)
   }

    /*!*********************************************************************************
    *  \brief Create the ACF graph.
    *
    *  In this function we
    *     - set the graph's unique identifier.
    *     - add bilinear interpolation kernel.
    *     - add graph ports for the source image, input local offset, input bilinear coefficient
   *       and destination image
    *     - connect graph ports to kernel ports
    ***********************************************************************************/
   void Create()
   {
      // set identifier
      SetIdentifier("affine_bilinear_interpolate_graph");
      
      // add kernel
      AddKernel("BI_INTERP",  AFFINE_BILINEAR_INTERPOLATE_KN);
      
      // add graph ports
      AddInputPort("INPUT");
      AddInputPort("MATRIX");
      AddInputPort("IMAGE_WIDTH");
      AddInputPort("IMAGE_HEIGHT");
      AddOutputPort("OUTPUT");
      
      // specify connections
      Connect( GraphPort("INPUT"),                 KernelPort("BI_INTERP", "INPUT"));
      Connect( GraphPort("MATRIX"),                KernelPort("BI_INTERP", "MATRIX"));
      Connect( GraphPort("IMAGE_WIDTH"),           KernelPort("BI_INTERP", "IMAGE_WIDTH"));
      Connect( GraphPort("IMAGE_HEIGHT"),          KernelPort("BI_INTERP", "IMAGE_HEIGHT"));
      Connect( KernelPort("BI_INTERP", "OUTPUT"),  GraphPort("OUTPUT"));
   }
};

#endif /* AFFINEBILINEARINTERPOLATEGRAPH_HPP */

