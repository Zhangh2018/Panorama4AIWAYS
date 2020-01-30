/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright 2018 NXP
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
*  \brief ACF graph definition for object detection based on precomputed block's HOG
*  and SVM decision function.
***********************************************************************************/
#ifndef HOGSVMGRAPH_HPP
#define HOGSVMGRAPH_HPP



#include <acf_graph.hpp>
#include "hog_acf.h"

class hog_svm_graph : public ACF_Graph
{
public:
  hog_svm_graph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(HOG_CONVERT_BLOCK_HISTOGRAMS_K)
    XREGISTER_ACF_KERNEL(HOG_GET_SCORES_K)
  }

  void Create()
  {
    //set identifier for graph
    SetIdentifier("hog_svm_graph");

    //add kernels
    AddKernel("convertBlockHistograms", HOG_CONVERT_BLOCK_HISTOGRAMS_KN);
    AddKernel("getScores", HOG_GET_SCORES_KN);

    //add graph ports
    AddInputPort("IMAGE_BLOCK_HISTOGRAMS");
    AddInputPort("SVM_FUNCTION");
    AddInputPort("SVM_TRANSFORM_MODE");
    AddInputPort("WIN_BLOCKS");
    AddOutputPort("SCORES");

    // connect graph ports
    Connect(GraphPort("IMAGE_BLOCK_HISTOGRAMS"),  KernelPort("convertBlockHistograms", "IMAGE_BLOCK_HISTOGRAMS"));
    Connect(GraphPort("SVM_FUNCTION"),            KernelPort("getScores", "SVM_FUNCTION"));
    Connect(GraphPort("SVM_TRANSFORM_MODE"),      KernelPort("getScores", "SVM_TRANSFORM_MODE"));
    Connect(GraphPort("WIN_BLOCKS"),              KernelPort("getScores", "WIN_BLOCKS"));

    // connect kernel ports
    Connect(KernelPort("convertBlockHistograms", "BLOCK_HISTOGRAMS"), KernelPort("getScores", "BLOCK_HISTOGRAMS"));
    Connect(KernelPort("getScores", "SCORES"), GraphPort("SCORES"));
  }
};

#endif /* HOGSVMGRAPH_HPP */
