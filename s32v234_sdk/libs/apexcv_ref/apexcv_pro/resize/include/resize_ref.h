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

 /*!*********************************************************************************
*  \file
*  \brief ApexCV Resize Library.
***********************************************************************************/

#ifndef APEXCVRESIZEREF_HPP
#define APEXCVRESIZEREF_HPP

#include <icp_data.h>
#include "sumat.hpp"

namespace ref{

/*!*********************************************************************************
*  \brief Resize.
*
*  Resize is the host-ACF interface for creating, initializing,
*  executing and releasing \ref pagResize "image resize" on Apex.
***********************************************************************************/
class Resize
{
public:

/*!*********************************************************************************
*  \brief Default constructor.
*  Allocate resources for the ACF process.
***********************************************************************************/
   Resize();

/*!*********************************************************************************
*  \brief Destructor.
*  Release resources for the ACF process.
***********************************************************************************/
   ~Resize();
   
/*!*********************************************************************************
*  \brief Execute the ACF resize process.
*
*  The output buffer of dst may be reallocated by this function.
*  This allocation includes space for the output produced by the APEX.
*
*  \return Error code (zero on success).
***********************************************************************************/   
   int process(vsdk::SUMat& src,
               vsdk::SUMat& dst);


private:
   icp::DataDescriptor* mSrcImagePtr;
   icp::DataDescriptor* mDestImagePtr;
   int mInitState;
};
}  /* namespace apex */

#endif   /* APEXCVRESIZEREF_HPP */
