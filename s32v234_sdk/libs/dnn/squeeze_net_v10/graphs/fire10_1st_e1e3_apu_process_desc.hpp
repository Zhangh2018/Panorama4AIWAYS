/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2016 Freescale Semiconductor
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
 * *  @file fire10_1st_e1e3_apu_process_desc.hpp
 * *  @brief Process description for FIRE10_1ST_E1E3 graph
 * ***********************************************************************************/
#ifndef FIRE101STE1E3APUPROCESSDESC_HPP
#define FIRE101STE1E3APUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "fire10_1st_e1e3_graph.hpp"

class fire10_1st_e1e3_apu_process_desc : public ACF_Process_Desc_APU
{
public:
   void Create()
   {
      Initialize(mGraph, "FIRE10_1ST_E1E3");
      FlagInputAsVerticalSdOverlap("FIRE10_1ST_E1E3_INPUT_IMAGE");
   }
   fire10_1st_e1e3_graph mGraph; 
};

#endif /* FIRE101STE1E3APUPROCESSDESC_HPP */

