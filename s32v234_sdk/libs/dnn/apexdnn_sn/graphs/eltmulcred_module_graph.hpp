/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
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
 *  @file eltmulcred_module_graph.hpp
 *  @brief Graph for ELTMULCRED_MODULE
 * ***********************************************************************************/
#ifndef ELTMULCREDMODULEGRAPH_HPP
#define ELTMULCREDMODULEGRAPH_HPP

#include <acf_graph.hpp>
#include "apexdnn_sn_acf.h"

class eltmulcred_module_graph : public ACF_Graph
{
public:
   eltmulcred_module_graph() : ACF_Graph()
   {
      XREGISTER_ACF_KERNEL(APU_ELTMULCRED_MODULE_NHCW_FORWARD_K)
   }

   void Create()
   {
      /* set identifier for graph */
      SetIdentifier("eltmulcred_module_graph");

      /* add kernels */
      AddKernel("APU_ELTMULCRED_MODULE", APU_ELTMULCRED_MODULE_NHCW_FORWARD_KN);

      /* add graph port */
      AddInputPort ("ELTMULCRED_MODULE_INPUT_IMAGE");
      AddInputPort ("ELTMULCRED_MODULE_INPUT_WEIGHT");
      AddInputPort ("ELTMULCRED_MODULE_INPUT_BIAS");
      AddInputPort ("ELTMULCRED_MODULE_INPUT_PARAMS");
      AddOutputPort("ELTMULCRED_MODULE_OUTPUT");

      /* specify connection */
      Connect(GraphPort("ELTMULCRED_MODULE_INPUT_IMAGE"),   KernelPort("APU_ELTMULCRED_MODULE", "INPUT_IMAGE"));
      Connect(GraphPort("ELTMULCRED_MODULE_INPUT_WEIGHT"),  KernelPort("APU_ELTMULCRED_MODULE", "INPUT_WEIGHT"));
      Connect(GraphPort("ELTMULCRED_MODULE_INPUT_BIAS"),    KernelPort("APU_ELTMULCRED_MODULE", "INPUT_BIAS"));
      Connect(GraphPort("ELTMULCRED_MODULE_INPUT_PARAMS"),  KernelPort("APU_ELTMULCRED_MODULE", "INPUT_PARAMS"));

      Connect(KernelPort("APU_ELTMULCRED_MODULE", "OUTPUT"), GraphPort("ELTMULCRED_MODULE_OUTPUT"));
   }
};

#endif /* ELTMULCREDMODULEGRAPH_HPP */

