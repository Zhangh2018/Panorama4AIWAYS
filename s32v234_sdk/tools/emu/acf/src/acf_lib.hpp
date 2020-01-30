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
****************************************************************************/
/**
* \file APU_ACF.h
* \brief APU ACF framework simulation
* \author Igor Aleksandrowicz
* \version
* \date 12-February-2014
****************************************************************************/

#ifndef ACFLIB_HPP
#define ACFLIB_HPP

#ifndef ACF_KERNEL_METADATA
#define ACF_KERNEL_METADATA
#endif

#ifndef ACF_KERNEL_IMPLEMENTATION
#define ACF_KERNEL_IMPLEMENTATION
#endif

#include "acf_process_desc_apu.hpp"
#include "common_stringify_macros.h"
#include <acf_common.h>

#ifdef USE_MULTITHREADING
#include <pthread.h>
#endif

namespace APEX2
{

#ifdef USE_MULTITHREADING
  extern pthread_mutex_t apex_Mutex;
#endif

  //Within-system APEX processor ID. Doesn't really matter for the library, included for SDK compatibility.
  enum APEX_ID
  {
    APEX_APEX0,
    APEX_APEX1,
	APEX_APEX_BOTH
  };

#ifndef XREGISTER_ACF_KERNEL
//syntax sugar macros
#define REGISTER_ACF_KERNEL(METADATA, FUNCTION) (ACF_Kernel::RegisterPrototypeInDatabase(ACF_Kernel((METADATA), (void*)(FUNCTION))));
#define XREGISTER_ACF_KERNEL(Kernel) REGISTER_ACF_KERNEL(kernelInfoConcat(Kernel), Kernel)
#endif

  //#define REGISTER_PROCESS_TYPE(NAME, DESC) typedef DESC NAME;
#define REGISTER_PROCESS_TYPE(NAME, DESC) class NAME : public DESC\
  {\
  public:\
    NAME() : DESC() {};\
    NAME(APEX_ID ):DESC() {};\
};

  void APEX_Init();
  void APEX_Deinit();
  int StartCriticalSection();
  int EndCriticalSection();

}

using namespace APEX2;

#endif /* ACFLIB_HPP */
