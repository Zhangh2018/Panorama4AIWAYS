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
*  \file
*  \brief APEX-CV Base Library
***********************************************************************************/

#ifndef APEXCVCORE_HPP
#define APEXCVCORE_HPP


#include "opencv2/core/cvdef.h"

namespace apexcv{
    
  template<typename X, typename Y>
  struct is_same
  {
    static const bool value = false;
  };

  template<typename X>
  struct is_same < X, X >
  {
    static const bool value = true;
  };

  

  template <typename D>
  int cv_type(int nrChannels)
  {
    if (is_same<D, uint8_t>::value)
      return CV_8UC(nrChannels);

    else if (is_same<D, uint16_t>::value)
      return CV_16UC(nrChannels);

    else if (is_same<D, uint32_t>::value)
      return CV_32SC(nrChannels);

    else if (is_same<D, int8_t>::value)
      return CV_8SC(nrChannels);

    else if (is_same<D, int16_t>::value)
      return CV_16SC(nrChannels);

    else if (is_same<D, int32_t>::value)
      return CV_32SC(nrChannels);

    else if (is_same<D, float>::value)
      return CV_32FC(nrChannels);

    else if (is_same<D, double>::value)
      return CV_64FC(nrChannels);

    else
      return -1;
  }


}  /* namespace apexcv */
#endif /* APEXCVCORE_HPP */
