/*****************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015, Digital Media Professionals. 
 * Copyright (c) 2016-2017 NXP Semiconductor;
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


#ifndef TMOCALCGEOMEANRGBEAPUPROCESSDESC_HPP
#define TMOCALCGEOMEANRGBEAPUPROCESSDESC_HPP

#include <acf_process_desc_apu.hpp>
#include "tmo_graph.hpp"

class tmo_calc_geo_mean_rgbe_apu_process_desc : public ACF_Process_Desc_APU
{
public:
   void Create()
   {
      Initialize(mGraph, "TMO_CALC_GEO_MEAN_RGBE");
   }

   tmo_calc_geo_mean_rgbe_graph mGraph;
};

#endif /* TMOCALCGEOMEANRGBEAPUPROCESSDESC_HPP */

