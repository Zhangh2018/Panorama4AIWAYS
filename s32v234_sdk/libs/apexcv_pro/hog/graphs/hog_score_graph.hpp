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
*  \brief ACF graph definition for the HOG implementation.
***********************************************************************************/
#ifndef HOGSCOREGRAPH_HPP
#define HOGSCOREGRAPH_HPP



#include <acf_graph.hpp>
#include "hog_acf.h"

class hog_score_graph : public ACF_Graph
{
public:
   hog_score_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(HOG_GET_GRADIENT_K)
      XREGISTER_ACF_KERNEL(HOG_GET_CELL_HISTOGRAMS_K)
      XREGISTER_ACF_KERNEL(HOG_COMPUTE_BLOCK_HISTOGRAMS_K)
      XREGISTER_ACF_KERNEL(HOG_GET_SCORES_K)
   }

   void Create()
   {
      //set identifier for graph
      SetIdentifier("hog_score_graph");
      
      //add kernels
      AddKernel("getGradient", HOG_GET_GRADIENT_KN);
      AddKernel("getCellHistograms", HOG_GET_CELL_HISTOGRAMS_KN);
      AddKernel("computeBlockHistograms", HOG_COMPUTE_BLOCK_HISTOGRAMS_KN);
      AddKernel("getScores", HOG_GET_SCORES_KN);

      //add graph ports
      AddInputPort("IMAGE");
      AddInputPort("SVM_FUNCTION");
      AddInputPort("SVM_TRANSFORM_MODE");
      AddInputPort("WIN_BLOCKS");
      AddOutputPort("SCORES");

      // connect graph ports
      Connect(GraphPort("IMAGE"),               KernelPort("getGradient", "IMAGE"));
      Connect(GraphPort("SVM_FUNCTION"),        KernelPort("getScores",   "SVM_FUNCTION"));
      Connect(GraphPort("SVM_TRANSFORM_MODE"),  KernelPort("getScores",   "SVM_TRANSFORM_MODE"));
      Connect(GraphPort("WIN_BLOCKS"),          KernelPort("getScores",   "WIN_BLOCKS"));

      // connect kernel ports
      Connect(KernelPort("getGradient", "GRADIENT_MAGNITUDE"), KernelPort("getCellHistograms", "GRADIENT_MAGNITUDE"));
      Connect(KernelPort("getGradient", "GRADIENT_ANGLE_BIN"), KernelPort("getCellHistograms", "GRADIENT_ANGLE_BIN"));  
      
      Connect(KernelPort("getCellHistograms", "CELL_HISTOGRAMS"), KernelPort("computeBlockHistograms", "CELL_HISTOGRAMS"));
            
      Connect(KernelPort("computeBlockHistograms", "BLOCK_HISTOGRAMS"), KernelPort("getScores", "BLOCK_HISTOGRAMS"));
      
      Connect(KernelPort("getScores", "SCORES"), GraphPort("SCORES"));
   }
};

#endif /* HOGSCOREGRAPH_HPP */
