/*****************************************************************************
* 
* NXP Confidential Proprietary
*
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

/*!*********************************************************************************
*  \file apexcv_host_base_class.h
*  \brief APEX-CV Host Base Class
***********************************************************************************/

#ifndef APEXCVHOSTBASECLASS_H
#define APEXCVHOSTBASECLASS_H

#include <iostream>
#include <cstdint>
#include <vector>

#include <apexcv_error_codes.hpp>
#include <apexcv_enum_types.hpp>


namespace apexcv
{

  /***********************************************************************************
  *  Base class for APEX-CV Base modules
  *  Provides common functionality for all APEX-CV modules.
  ***********************************************************************************/
  class ApexcvHostBaseBaseClass
  {

  friend class ApexcvHostBaseFriend;
  public:
    ApexcvHostBaseBaseClass() : mType(0), mpProcess(nullptr) {};
    virtual ~ApexcvHostBaseBaseClass() {};

  protected:
    class InfoCluster
    {
      void * pACF;
      std::string ACFName;
      std::vector<std::string> mvPortName;

    public:
      InfoCluster():
        pACF(nullptr)
        {}
      void set_ACF(std::string const str, void* const paACF)
      {
        pACF = paACF;
        ACFName = str;
      }
      void* get_ACF() const
      {
        return pACF;
      }
      const std::string(&get_ACFName() const)
      {
        return ACFName;
      }
      const std::vector<std::string>(&get_vPortName() const)
      {
        return mvPortName;
      }
      void push_PortName(std::string const str)
      {
        mvPortName.push_back(str);
      }
      InfoCluster(void* const paACF, std::string const str)
      {
        set_base_ACF(paACF, str);
      }
      void set_base_ACF( void* const paACF, std::string const str)
      {
        pACF = paACF;
        ACFName = "*";
        mvPortName.push_back(str);
      }
    };

    int mType;
    void* mpProcess;
    std::vector<InfoCluster> mvInfoClusters;
  };

  class ApexcvHostBaseClass : public ApexcvHostBaseBaseClass
  {
    public:

    /*!*********************************************************************************
    *  \brief      Select the APEX Core
    *  \details    Select which APEX core (0 or 1) to be selected to run the processing.
    *              This function has to be called after initialize() and can be executed on a per frame base.
    *  \return     APEXCV Error code (APEXCV_SUCCESS on success).
    ***********************************************************************************/
    APEXCV_LIB_RESULT SelectApexCore(
      int  aApexId  /*!< [in] ID of the APEX core used for performing the processing (0 or 1). */);

    /*!*********************************************************************************
    *  \brief      Start processing and return when done.
    *  \details    Execute code on selected APEX core (default is Apex core 0).
    *              This function is called after initialize() and is executed on a per frame base.
    *  \return     APEXCV Error code (APEXCV_SUCCESS on success).
    ***********************************************************************************/
    APEXCV_LIB_RESULT Process();

  };

}
#endif /* APEXCVHOSTBASECLASS_H */

