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
/*!****************************************************************************
*  \file pyramid_multi_ref.h
*  \brief Reference kernels for image pyramid_multi creation algorithm
*******************************************************************************/

#ifndef APEXCVPYRAMIDMULTIREF_HPP
#define APEXCVPYRAMIDMULTIREF_HPP

#include "icp_data.h"
#include <sumat.hpp>

#define MAX_PYRAMID_MULTI_LEVELS_REF 4

namespace ref {

/*!****************************************************************************
*  \brief Pyramid_multi creation class
*
*  This class is an interface for using the pyrami_multi  creation algorithm.
******************************************************************************/  
class PyramidMultiCreation
{
public:

/*!****************************************************************************
*  \brief Default constructor.
******************************************************************************/
   PyramidMultiCreation();

/*!****************************************************************************
*  \brief Default destructor.
******************************************************************************/
   ~PyramidMultiCreation();

/*!****************************************************************************
*  \brief Run the pyramid_multi creation process.
*
*  Upscale or downscale _src_ buffer and stores the result in _dst_ buffer.
*  The process will upscale _src_ buffer if _isPyramidUp_ = true.
*  The process will downscale _src_ buffer if _isPyramidUp_ = false.
*  Default is _isPyramidUp_ = true.
*
*  Supported datatypes are:
*    - unsigned 8 bit to unsigned 8 bit
*
*  \return Error code (zero on success).
***********************************************************************************/
   int process(vsdk::SUMat  src,   /*!< Source memory buffer. Datatype is ICP_DATATYPE_08U. */
               vsdk::SUMat*  dst,   /*!< Destination memory buffer. */
               bool isPyramidUp);   /*!< Set true if execute pyramid up, or false if execute pyramid down. */

}; /* class PyramidCreation */

} /* namespace apex */

#endif /* APEXCVPYRAMIDMULTIREF_HPP */

