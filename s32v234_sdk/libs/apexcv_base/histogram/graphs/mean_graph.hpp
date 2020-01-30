/*****************************************************************************
*
* NXP Confidential Proprietary
* Copyright (c) 2018 NXP
* All Rights Reserved
*
*****************************************************************************
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
*  @file mean_graph.hpp
*  @brief ACF graph for mean
***********************************************************************************/
#ifndef MEANGRAPH_HPP
#define MEANGRAPH_HPP

#include <acf_graph.hpp>
#include "histogram_acf.h"
#include "graph_templates.h"

class mean_graph : public ACF_Graph
{
public:
  mean_graph()
    : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(MEAN_K)
  }

  void Create()
  {
    //set identifier for graph
    SetIdentifier("mean_graph");

    //add kernels
    std::string meanKn("mean_kn");
    AddKernel(meanKn, MEAN_KN);

    //add graph ports
    AddInputPort(GR_IN_0);
    AddOutputPort(GR_OUT_0);

    //specify connections
    Connect(GraphPort(GR_IN_0), KernelPort(meanKn, INPUT_0));
    Connect(KernelPort(meanKn, OUTPUT_0), GraphPort(GR_OUT_0));
  }
};

#endif /* MEANGRAPH_HPP */
