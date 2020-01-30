/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor
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

#ifndef AGGCFAPUHTI4PROCESSDESC_HPP
#define AGGCFAPUHTI4PROCESSDESC_HPP


#include <acf_process_desc_apu.hpp>
#include "aggcf_hist_ti4_graph.hpp"

class aggcf_apu_hti4_process_desc : public ACF_Process_Desc_APU
{
public:
/*!********************************************************************************* 
*  \brief Create the ACF process description.
*  
*  In this function we 
*     - initialize the process and give it a unique identifier.
*     - set the input chunk size
***********************************************************************************/
   void Create()
   {
      Initialize(mGraph, "AGGCF_HIST_TI4");
   }
   
   aggcfhistti4_graph mGraph;
};





#endif /* AGGCFAPUHTI4PROCESSDESC_HPP */
