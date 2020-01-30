/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
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
*  \brief APEX-CV Base Library
***********************************************************************************/

#include "opencv2/core/cvdef.h"
#include "apexcv_core.h"

#include <stdint.h>

namespace apexcv{


  template<>
  int cv_type<uint8_t>(int nrChannels);

  template<>
  int cv_type<uint16_t>(int nrChannels);
  
  template<>
  int cv_type<uint32_t>(int nrChannels);

  template<>
  int cv_type<int8_t>(int nrChannels);

  template<>
  int cv_type<int16_t>(int nrChannels);

  template<>
  int cv_type<int32_t>(int nrChannels);


}  /* namespace apexcv */
