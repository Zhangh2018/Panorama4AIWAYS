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
#ifndef HISTOGRAMEQUALIZATIONGRAPH_HPP
#define HISTOGRAMEQUALIZATIONGRAPH_HPP

#include <acf_graph.hpp>

#include <histogram_equalization_graph_names.h>
#include <histogram_equalization_acf.h>

class histogram_equalization_graph : public ACF_Graph
{
public:
  const char* m_graphName;
public:
   histogram_equalization_graph() 
     : ACF_Graph()
   {
     XREGISTER_ACF_KERNEL(HIST_EQU_K)
     XREGISTER_ACF_KERNEL(EQU_LUT_K)
   }

   void Create()
   {
      //set identifier for graph
      SetIdentifier(HISTO_EQU_GN);

      //add kernels
      const std::string lutk("lut_kernel_instance1");
    AddKernel(lutk,  EQU_LUT_KN);
      const std::string hek("histogram_equalization_kernel_instance1");
      AddKernel(hek,     HIST_EQU_KN);

      //add graph ports
      AddInputPort(HISTO_EQU_IN_IMG);
      AddInputPort(HISTO_EQU_NUM_PIXELS);
      AddInputPort(HISTO_EQU_HISTO);
    AddOutputPort(HISTO_EQU_OUT_IMG);

    //connecting graph input into second kernel
      Connect(GraphPort(HISTO_EQU_HISTO), KernelPort(lutk, HIST_LUT_K_HISTO));
      Connect(GraphPort(HISTO_EQU_NUM_PIXELS), KernelPort(lutk, HIST_LUT_K_NUM_PIXELS));
     
      
      //connecting second kernels output to the last kernels input
      Connect(KernelPort(lutk, HIST_LUT_K_LUT), KernelPort(hek, HIST_EQU_K_LUT));
      Connect(GraphPort(HISTO_EQU_IN_IMG), KernelPort(hek, HIST_EQU_K_IN_IMG));

      //connecting last kernel output to the graph
      Connect(KernelPort(hek, HIST_EQU_K_OUT_IMG), GraphPort(HISTO_EQU_OUT_IMG));
   }
};

#endif /* HISTOGRAMEQUALIZATIONGRAPH_HPP */


