/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

/*!****************************************************************************
*  \file apexcv_pro_pyramid_multi.h
*  \brief ApexCV Host wrapper for pyramid creation
*******************************************************************************/

#ifndef APEXCVPROPYRAMIDMULTI_H
#define APEXCVPROPYRAMIDMULTI_H

#include <sumat.hpp>
#include "acf_process_apu.h"
#include <apexcv_error_codes.hpp>
#include <apexcv_host_base_class.h>


namespace apexcv {

/*!****************************************************************************
*  \brief Pyramid_multi creation class
*
*  This class is an interface for using the pyramid_multi creation algorithm.
******************************************************************************/

class PyramidMultiCreation: public ApexcvHostBaseBaseClass
{
public:

/*****************************************************************************
*  Default constructor.
******************************************************************************/
   PyramidMultiCreation();

/*****************************************************************************
*  Default destructor.
******************************************************************************/
   ~PyramidMultiCreation();

   static const int32_t PYRAMID_LEVELS = 4;/*!< This is static constant variable that defined pyramid multi level. */

/*!****************************************************************************
* \brief Initializes the class: connects the buffer to the process port,
* and allocates/initializes any internal buffers.
* \param[in]  aSrc Source memory buffer. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[0] Destination memory buffer for scale pyramid 1/2. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[1] Destination memory buffer for scale pyramid 1/4. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[2] Destination memory buffer for scale pyramid 1/8. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[3] Destination memory buffer for scale pyramid 1/16. Datatype is VSDK_CV_8UC1.
* \return Error code for the initialization - see apexcv_error_codes.hpp
******************************************************************************/
   APEXCV_LIB_RESULT Initialize(    vsdk::SUMat&  aSrc,
                                    vsdk::SUMat (&aDstArray)[PYRAMID_LEVELS]
                  );

/*!****************************************************************************
*  \brief Connects the input/outputs to the process
*
*  Use this to reconnect the Input and Output Buffers. This only needs to
*  be done if the connected Input/Outputs are changed. If only the data within (no size,
*  or type changes), then this does not need to be called.
* \param[in]  aSrc Source memory buffer. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[0] Destination memory buffer for scale pyramid 1/2. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[1] Destination memory buffer for scale pyramid 1/4. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[2] Destination memory buffer for scale pyramid 1/8. Datatype is VSDK_CV_8UC1.
* \param[out] aDstArray[3] Destination memory buffer for scale pyramid 1/16. Datatype is VSDK_CV_8UC1.
* \return Error code for the initialization - see apexcv_error_codes.hpp
******************************************************************************/
   APEXCV_LIB_RESULT ReconnectIO(     vsdk::SUMat&  aSrc,
                                      vsdk::SUMat (&aDstArray)[PYRAMID_LEVELS]
                  );

/*!****************************************************************************
*  \brief Run the pyramid_multi creation process.
*
*  \return Error code (zero on success).
***********************************************************************************/
   APEXCV_LIB_RESULT Process();


/*!*********************************************************************************
 *  \brief SelectApexCore
 *  \return Error code for the ACF execution (zero on success).
 *
 *  Select which APEX core (0 or 1) to be selected to run the processing.
 *  This function has to be called after initialize() and can be executed per frame base.
 ***********************************************************************************/
   APEXCV_LIB_RESULT SelectApexCore(int32_t aApexId /*!< The ID of the desired APEX (e.g if there are 2 APEXs, valid values for lApexId would be 0 and 1).*/
               );

private:
   int mInitState;      /*!< Flag indicating the process has been initialized. */
   int mWidth;         /*!< Input width.  */
   int mHeight;        /*!< Input height. */
   int mLevels;

   void *mProcessDownMulti0;
   void *mProcessDownMulti1;

   int32_t     mApexId;
}; /* class PyramidCreation */

} /* namespace apex */

#endif /* APEXCVPROPYRAMIDMULTI_H */

