/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
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
*  @file histogram_graph.hpp
*  @brief ACF graph for histogram
***********************************************************************************/
#ifndef HISTOGRAMGRAPH_HPP
#define HISTOGRAMGRAPH_HPP

#include <acf_graph.hpp>

#include <histogram_acf.h>
#include <histogram_graph_names.h>

class histogram_graph : public ACF_Graph
{
public:
  histogram_graph()
    : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(OPTIM_HISTOGRAM_K)
  }

  void Create()
  {
    //set identifier for graph
    SetIdentifier("histogram_graph");

    //add kernels
    const std::string histoKn("_histogram");
    AddKernel(histoKn, OPTIM_HISTOGRAM_KN);

    //add graph ports
    AddInputPort(HISTOGR_IN_IMG);
    AddOutputPort(HISTOGR_OUT_HISTO);

    //specify connections
    Connect(GraphPort(HISTOGR_IN_IMG), KernelPort(histoKn, INPUT_0));
    Connect(KernelPort(histoKn, OUTPUT_0), GraphPort(HISTOGR_OUT_HISTO));
  }
};

#endif /* HISTOGRAMGRAPH_HPP */
