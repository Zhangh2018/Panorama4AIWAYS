/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
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
***************************************************************************/

  
/*!*********************************************************************************
*  @file abs_08u_graph.hpp
*  @brief ACF graph for absolute difference
***********************************************************************************/
#ifndef ALLARITHMETICGRAPHREGISTRATIONS_H
#define ALLARITHMETICGRAPHREGISTRATIONS_H


#ifndef APEX2_EMULATE
#include <icp_data.h>
using namespace icp;

#include <ABS_08U.hpp>
#include <ABSDIFF_08U.hpp>
#include <ABSDIFF_16S.hpp>
#include <ACCUMULATE_16S.hpp>
#include <ACCUMULATE_SQUARED_16S.hpp>
#include <ACCUMULATE_WEIGHTED_08U.hpp>
#include <ADD_08U.hpp>
#include <ADD_08U_08U_16S.hpp>
#include <ADD_08U_16S_16S.hpp>
#include <ADD_16S.hpp>
#include <AND_08U.hpp>
#include <AND_16U.hpp>
#include <AND_32U.hpp>
#include <CLZ_08U.hpp>
#include <CLZ_08S.hpp>
#include <CLZ_16U.hpp>
#include <CLZ_16S.hpp>
#include <MAGNITUDE_16S.hpp>
#include <MAX_08U.hpp>
#include <MAX_16S.hpp>
#include <MIN_08U.hpp>
#include <MIN_16S.hpp>
#include <MUL_08U.hpp>
#include <MUL_08U_08U_16S.hpp>
#include <MUL_08U_16S_16S.hpp>
#include <MUL_16S.hpp>
#include <NOT_08U.hpp>
#include <OR_08U.hpp>
#include <OR_16U.hpp>
#include <OR_32U.hpp>
#include <PHASE_08U.hpp>
#include <SUB_08U.hpp>
#include <SUB_08U_08U_16S.hpp>
#include <SUB_08U_16S_16S.hpp>
#include <SUB_16S.hpp>
#include <SUB_16S_08U_16S.hpp>
#include <TABLELOOKUP_08U.hpp>
#include <THRESHOLD_08U.hpp>
#include <THRESHOLD_16U.hpp>
#include <THRESHOLD_32U.hpp>
#include <THRESHOLD_RANGE_08U.hpp>
#include <THRESHOLD_RANGE_16U.hpp>
#include <THRESHOLD_RANGE_32U.hpp>
#include <XOR_08U.hpp>
#include <XOR_16U.hpp>
#include <XOR_32U.hpp>
#else
#include "acf_lib.hpp"
using namespace APEX2;

#include "abs_08u_apu_process_desc.hpp"
#include "abs_08u_apu_process_desc.hpp"
#include "absdiff_08u_apu_process_desc.hpp"
#include "absdiff_16s_apu_process_desc.hpp"
#include "accumulate_16s_apu_process_desc.hpp"
#include "accumulate_squared_16s_apu_process_desc.hpp"
#include "accumulate_weighted_08u_apu_process_desc.hpp"
#include "add_08u_apu_process_desc.hpp"
#include "add_08u_08u_16s_apu_process_desc.hpp"
#include "add_08u_16s_16s_apu_process_desc.hpp"
#include "add_16s_apu_process_desc.hpp"
#include "and_08u_apu_process_desc.hpp"
#include "and_16u_apu_process_desc.hpp"
#include "and_32u_apu_process_desc.hpp"
#include "clz_08u_apu_process_desc.hpp"
#include "clz_08s_apu_process_desc.hpp"
#include "clz_16u_apu_process_desc.hpp"
#include "clz_16s_apu_process_desc.hpp"
#include "magnitude_16s_apu_process_desc.hpp"
#include "max_08u_apu_process_desc.hpp"
#include "max_16s_apu_process_desc.hpp"
#include "min_08u_apu_process_desc.hpp"
#include "min_16s_apu_process_desc.hpp"
#include "mul_08u_apu_process_desc.hpp"
#include "mul_08u_08u_16s_apu_process_desc.hpp"
#include "mul_08u_16s_16s_apu_process_desc.hpp"
#include "mul_16s_apu_process_desc.hpp"
#include "not_08u_apu_process_desc.hpp"
#include "or_08u_apu_process_desc.hpp"
#include "or_16u_apu_process_desc.hpp"
#include "or_32u_apu_process_desc.hpp"
#include "phase_08u_apu_process_desc.hpp"
#include "sub_08u_apu_process_desc.hpp"
#include "sub_08u_08u_16s_apu_process_desc.hpp"
#include "sub_08u_16s_16s_apu_process_desc.hpp"
#include "sub_16s_apu_process_desc.hpp"
#include "sub_16s_08u_16s_apu_process_desc.hpp"
#include "tablelookup_08u_apu_process_desc.hpp"
#include "threshold_08u_apu_process_desc.hpp"
#include "threshold_16u_apu_process_desc.hpp"
#include "threshold_32u_apu_process_desc.hpp"
#include "threshold_range_08u_apu_process_desc.hpp"
#include "threshold_range_16u_apu_process_desc.hpp"
#include "threshold_range_32u_apu_process_desc.hpp"
#include "xor_08u_apu_process_desc.hpp"
#include "xor_16u_apu_process_desc.hpp"
#include "xor_32u_apu_process_desc.hpp"

#endif


#endif /* ALLARITHMETICGRAPHREGISTRATIONS_H */
