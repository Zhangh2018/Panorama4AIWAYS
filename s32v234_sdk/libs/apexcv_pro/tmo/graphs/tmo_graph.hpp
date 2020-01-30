/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015, Digital Media Professionals. All rights reserved.
* Copyright (c) 2016 Freescale Semiconductor
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
#ifndef CALCGEOMEAN_HPP
#define CALCGEOMEAN_HPP

#include <acf_graph.hpp>
#include "tmo_acf.h"

class tmo_calc_geo_mean_rgbe_graph : public ACF_Graph
{
public:
   tmo_calc_geo_mean_rgbe_graph() : ACF_Graph() {
#ifdef APEX2_EMULATE
      XREGISTER_ACF_KERNEL(APU_GEO_MEAN_RGBE_K);
#else
      XREGISTER_ACF_KERNEL(CALC_GEO_MEAN_RGBE_K)
#endif
   }
   void Create()
   {
      SetIdentifier("tmo_calc_geo_mean_rgbe_graph");

      AddKernel("CALC_GEO_MEAN", CALC_GEO_MEAN_RGBE_KN);

      AddInputPort("INPUT");
      AddOutputPort("INTERMEDIATE_EXP");
      AddOutputPort("INTERMEDIATE_MAN");
      AddOutputPort("WORLD_EXP");
      AddOutputPort("WORLD_MAN");
      AddOutputPort("GEO_MEAN_EXP");
      AddOutputPort("GEO_MEAN_MAN");

      Connect(GraphPort("INPUT"), KernelPort("CALC_GEO_MEAN", "INPUT"));

      Connect(KernelPort("CALC_GEO_MEAN", "INTERMEDIATE_EXP"), GraphPort("INTERMEDIATE_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "INTERMEDIATE_MAN"), GraphPort("INTERMEDIATE_MAN"));
      Connect(KernelPort("CALC_GEO_MEAN", "WORLD_EXP"), GraphPort("WORLD_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "WORLD_MAN"), GraphPort("WORLD_MAN"));
      Connect(KernelPort("CALC_GEO_MEAN", "GEO_MEAN_EXP"), GraphPort("GEO_MEAN_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "GEO_MEAN_MAN"), GraphPort("GEO_MEAN_MAN"));
   }
};

class tmo_calc_geo_mean_openexr_graph : public ACF_Graph
{
public:
   tmo_calc_geo_mean_openexr_graph() : ACF_Graph() {
#ifdef APEX2_EMULATE
      XREGISTER_ACF_KERNEL(APU_GEO_MEAN_OPENEXR_K);
#else
      XREGISTER_ACF_KERNEL(CALC_GEO_MEAN_OPENEXR_K)
#endif
   }
   void Create()
   {
      SetIdentifier("tmo_calc_geo_mean_openexr_graph");

      AddKernel("CALC_GEO_MEAN", CALC_GEO_MEAN_OPENEXR_KN);

      AddInputPort("INPUT");
      AddOutputPort("INTERMEDIATE_EXP");
      AddOutputPort("INTERMEDIATE_MAN");
      AddOutputPort("WORLD_EXP");
      AddOutputPort("WORLD_MAN");
      AddOutputPort("GEO_MEAN_EXP");
      AddOutputPort("GEO_MEAN_MAN");

      Connect(GraphPort("INPUT"), KernelPort("CALC_GEO_MEAN", "INPUT"));

      Connect(KernelPort("CALC_GEO_MEAN", "INTERMEDIATE_EXP"), GraphPort("INTERMEDIATE_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "INTERMEDIATE_MAN"), GraphPort("INTERMEDIATE_MAN"));
      Connect(KernelPort("CALC_GEO_MEAN", "WORLD_EXP"), GraphPort("WORLD_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "WORLD_MAN"), GraphPort("WORLD_MAN"));
      Connect(KernelPort("CALC_GEO_MEAN", "GEO_MEAN_EXP"), GraphPort("GEO_MEAN_EXP"));
      Connect(KernelPort("CALC_GEO_MEAN", "GEO_MEAN_MAN"), GraphPort("GEO_MEAN_MAN"));
   }
};

class tmo_ldr_transform_graph : public ACF_Graph
{
public:
   tmo_ldr_transform_graph() : ACF_Graph() {
#ifdef APEX2_EMULATE
      XREGISTER_ACF_KERNEL(APU_LDR_TRANSFORM_K);
#else
      XREGISTER_ACF_KERNEL(LDR_TRANSFORM_K)
#endif
   }
   void Create()
   {
      SetIdentifier("tmo_ldr_transform_graph");

      AddKernel("LDR_TRANS", LDR_TRANSFORM_KN);

      AddInputPort("INTERMEDIATE_EXP");
      AddInputPort("INTERMEDIATE_MAN");
      AddInputPort("WORLD_EXP");
      AddInputPort("WORLD_MAN");
      AddInputPort("GEO_MEAN_EXP");
      AddInputPort("DIV_VALUE");
      AddInputPort("KEY_VALUE");
      AddOutputPort("OUTPUT");

      Connect(GraphPort("INTERMEDIATE_EXP"), KernelPort("LDR_TRANS", "INTERMEDIATE_EXP"));
      Connect(GraphPort("INTERMEDIATE_MAN"), KernelPort("LDR_TRANS", "INTERMEDIATE_MAN"));
      Connect(GraphPort("WORLD_EXP"), KernelPort("LDR_TRANS", "WORLD_EXP"));
      Connect(GraphPort("WORLD_MAN"), KernelPort("LDR_TRANS", "WORLD_MAN"));
      Connect(GraphPort("GEO_MEAN_EXP"), KernelPort("LDR_TRANS", "GEO_MEAN_EXP"));
      Connect(GraphPort("DIV_VALUE"), KernelPort("LDR_TRANS", "DIV_VALUE"));
      Connect(GraphPort("KEY_VALUE"), KernelPort("LDR_TRANS", "KEY_VALUE"));

      Connect(KernelPort("LDR_TRANS", "OUTPUT"), GraphPort("OUTPUT"));
   }
};
#endif /* CALCGEOMEAN_HPP */

