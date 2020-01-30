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
*  @file abs_08u_graph.hpp
*  @brief ACF graph for absolute difference
***********************************************************************************/
#ifndef ALLCOLCONVGRAPHREGISTRATIONS_H
#define ALLCOLCONVGRAPHREGISTRATIONS_H


#ifndef APEX2_EMULATE
#include <icp_data.h>
using namespace icp;

#include <RGB565_TO_RGB888X.hpp>
#include <RGB888X_TO_Y.hpp>
#include <RGB888X_TO_RGB565.hpp>
#include <RGB888X_TO_YUV.hpp>
#include <CH_EXTR_RGB.hpp>
#include <CH_EXTR_RGBX.hpp>


#else
#include "acf_data_descriptor.hpp" 
#include "acf_lib.hpp"
using namespace APEX2;

#include "rgb565_to_rgb888x_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB565_TO_RGB888X, rgb565_to_rgb888x_apu_process_desc)

#include "rgb888x_to_rgb565_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_RGB565, rgb888x_to_rgb565_apu_process_desc)

#include "rgb888x_to_y_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_Y, rgb888x_to_y_apu_process_desc)

#include "rgb888x_to_yuv_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(RGB888X_TO_YUV, rgb888x_to_yuv_apu_process_desc)

#include "ch_extr_rgb_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(CH_EXTR_RGB, ch_extr_rgb_apu_process_desc)

#include "ch_extr_rgbx_apu_process_desc.hpp"
REGISTER_PROCESS_TYPE(CH_EXTR_RGBX, ch_extr_rgbx_apu_process_desc)

#endif


#endif /* ALLCOLCONVGRAPHREGISTRATIONS_H */
