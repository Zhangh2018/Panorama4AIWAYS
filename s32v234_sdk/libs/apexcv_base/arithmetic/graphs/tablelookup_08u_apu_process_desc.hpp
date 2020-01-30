/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
***************************************************************************

/*==================================================================================================
Revision History:
                              Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Adrian Grigore (nxa11567)     01/02/2017    VSDK-552      Created
---------------------------   ----------    ------------  ------------------------------------------
==================================================================================================*/
/*!*********************************************************************************
*  \file tablelookup_08u_apu_process_desc.hpp
*  \brief ACF graph for Table Lookup
***********************************************************************************/
#ifndef TABLELOOKUP08UAPUPROCESSDESC_HPP
#define TABLELOOKUP08UAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "graph_templates.h"
#include <acf_graph.hpp>
#include "arithmetic_acf.h"

GRAPH_CLASS_DEF_2In_1Out(TABLE_LOOKUP_K)

#define TABLELOOKUP_08U_ID  TABLELOOKUP_08U
#define TABLELOOKUP_08U_STR  XSTR(TABLELOOKUP_08U_ID)

class tablelookup_08u_apu_process_desc : public ACF_Process_Desc_APU
{
public:
   void Create()
   {
      Initialize(mGraph, TABLELOOKUP_08U_STR);
   }
   
   graphClassName(TABLE_LOOKUP_K) mGraph;
};


#ifdef APEX2_EMULATE
REGISTER_PROCESS_TYPE(TABLELOOKUP_08U_ID, tablelookup_08u_apu_process_desc)
#endif

#endif /* TABLELOOKUP08UAPUPROCESSDESC_HPP */


