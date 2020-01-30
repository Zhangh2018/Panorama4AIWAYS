/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright(c) 2017 NXP
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
*INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include <stdio.h>

#ifdef APEX2_EMULATE
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include <acf_process_apu.h>
#endif

#include <apexcv_host_base_class.h>


namespace apexcv{

  /************************************
  * Select APEX to run
  ***********************************/
  APEXCV_LIB_RESULT ApexcvHostBaseClass::SelectApexCore(int aApexId)
  {
    if (nullptr == mpProcess)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (0 == mvInfoClusters.size())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
#ifndef APEX2_EMULATE
    ACF_Process_APU* pACF = (ACF_Process_APU*)mvInfoClusters.at(0).get_ACF();
    int lRetVal = pACF->SelectApuConfiguration(ACF_APU_CFG__DEFAULT, aApexId);
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_SELECTAPUCONFIG;
    }
#endif
    return APEXCV_SUCCESS;
  }



  /************************************
  * Select Process
  ***********************************/
  APEXCV_LIB_RESULT ApexcvHostBaseClass::Process()
  {
    if (nullptr == mpProcess)
    {
      return APEXCV_ERROR_OBJECT_ISNOT_INITIALIZED;
    }
    if (0 == mvInfoClusters.size())
    {
      return APEXCV_ERROR_INTERNAL_ERROR;
    }
    ACF_Process_APU* pACF = (ACF_Process_APU*)mvInfoClusters.at(0).get_ACF();
    int lRetVal = 0;
    lRetVal |= pACF->Start();
    lRetVal |= pACF->Wait();
    if (0 != lRetVal)
    {
      return APEXCV_ERROR_ACF_PROC_EXEC;
    }

      return APEXCV_SUCCESS;
  }
}