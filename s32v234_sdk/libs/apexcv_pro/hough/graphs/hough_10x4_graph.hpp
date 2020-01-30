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
*  \brief ACF graph for Hough Transform.
***********************************************************************************/
#ifndef HOUGH10X4GRAPH_HPP
#define HOUGH10X4GRAPH_HPP

#include <acf_graph.hpp>
#include "hough_acf.h"

/*!*********************************************************************************
*  \brief ACF graph for Hough Transform.
***********************************************************************************/
class Hough_Graph : public ACF_Graph
{
public:
Hough_Graph() : ACF_Graph()
{
  XREGISTER_ACF_KERNEL(HOUGH_INIT_PIXEL_OFFSET_K)
  XREGISTER_ACF_KERNEL(HOUGH_PIXEL_THRESHOLD_K)
  XREGISTER_ACF_KERNEL(HOUGH_SCALARIZE_PIXELS_K)
  XREGISTER_ACF_KERNEL(HOUGH_ACCUMULATE_K)
  XREGISTER_ACF_KERNEL(HOUGH_INIT_CU_INDEX_K)
  XREGISTER_ACF_KERNEL(HOUGH_GET_LINES_K)
}

/*!*********************************************************************************
*  \brief Create the ACF graph.
*
*  In this function we
*     - set the graph's unique identifier.
*     - add the column filter kernel
*     - add graph ports for the source image, filter coefficients and destination image
*     - connect graph ports to kernel ports
***********************************************************************************/
void Create()
{
  //set identifier for graph
  SetIdentifier("Hough_Graph");

  // Add kernels
  AddKernel("initPixelOffset", HOUGH_INIT_PIXEL_OFFSET_KN);
  AddKernel("pixelThreshold", HOUGH_PIXEL_THRESHOLD_KN);
  AddKernel("scalarizePixels", HOUGH_SCALARIZE_PIXELS_KN);
  AddKernel("accumulate", HOUGH_ACCUMULATE_KN);
  AddKernel("initCuIndex", HOUGH_INIT_CU_INDEX_KN);
  AddKernel("getLines", HOUGH_GET_LINES_KN);

  // Graph ports input
  AddInputPort("IMAGE");
  AddInputPort("PIXEL_THRESHOLD");
  AddInputPort("PIXEL_OFFSET_PARAMS");
  AddInputPort("CU_COUNT");
  AddInputPort("COS_TABLE");
  AddInputPort("SIN_TABLE");
  AddInputPort("RHO_COUNT");
  AddInputPort("CU_INDEX_PARAMS");
  AddInputPort("LINE_THRESHOLD_PARAMS");

  // Graph ports output
  AddOutputPort("LINES");
  AddOutputPort("LINE_COUNT");

  // Connect input ports
  Connect(GraphPort("IMAGE"), KernelPort("pixelThreshold", "IMAGE"));
  Connect(GraphPort("PIXEL_THRESHOLD"), KernelPort("pixelThreshold", "THRESHOLD"));
  Connect(GraphPort("PIXEL_OFFSET_PARAMS"), KernelPort("initPixelOffset", "PARAMS"));
  Connect(GraphPort("CU_COUNT"), KernelPort("scalarizePixels", "CU_COUNT"));
  Connect(GraphPort("COS_TABLE"), KernelPort("accumulate", "COS_TABLE"));
  Connect(GraphPort("SIN_TABLE"), KernelPort("accumulate", "SIN_TABLE"));
  Connect(GraphPort("RHO_COUNT"), KernelPort("accumulate", "RHO_COUNT"));
  Connect(GraphPort("CU_INDEX_PARAMS"), KernelPort("initCuIndex", "PARAMS"));
  Connect(GraphPort("LINE_THRESHOLD_PARAMS"), KernelPort("getLines", "PARAMS"));

  // Connect kernel ports
  Connect(KernelPort("initPixelOffset", "OFFSET_X"), KernelPort("pixelThreshold", "OFFSET_X"));
  Connect(KernelPort("initPixelOffset", "OFFSET_Y"), KernelPort("pixelThreshold", "OFFSET_Y"));
  Connect(KernelPort("pixelThreshold", "PIXELS_X"), KernelPort("scalarizePixels", "VEC_PIXELS_X"));
  Connect(KernelPort("pixelThreshold", "PIXELS_Y"), KernelPort("scalarizePixels", "VEC_PIXELS_Y"));
  Connect(KernelPort("pixelThreshold", "PIXEL_COUNT"), KernelPort("scalarizePixels", "VEC_PIXEL_COUNT"));
  Connect(KernelPort("scalarizePixels", "SCL_PIXELS_X"), KernelPort("accumulate", "PIXELS_X"));
  Connect(KernelPort("scalarizePixels", "SCL_PIXELS_Y"), KernelPort("accumulate", "PIXELS_Y"));
  Connect(KernelPort("scalarizePixels", "SCL_PIXEL_COUNT"), KernelPort("accumulate", "PIXEL_COUNT"));
  Connect(KernelPort("accumulate", "ACCUMULATOR"), KernelPort("getLines", "ACCUMULATOR"));
  Connect(KernelPort("initCuIndex", "CU_INDEX"), KernelPort("getLines", "CU_INDEX"));

  // Connect output ports
  Connect(KernelPort("getLines", "LINES"), GraphPort("LINES"));
  Connect(KernelPort("getLines", "LINE_COUNT"), GraphPort("LINE_COUNT"));
}
};

#endif /* HOUGH10X4GRAPH_HPP */

