/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2018 NXP Semiconductor;
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

#ifndef _BUF_DESC_HPP
#define _BUF_DESC_HPP

#include <stdint.h>

//forward declarations
class DataDesc;

//////////////////////////////////////////////////////////////////////////
/// BufDesc describes a buffer in memory by pairing a DataDesc with an address.
//////////////////////////////////////////////////////////////////////////
class BufDesc
{
public:

   //---------
   //methods
   //---------

   /////////////////////////////////////////////////////////////////////////
   /// Initialize buffer descriptor.
   ///
   /// \param[in] lpDataDesc
   /// Pointer to the buffer description.
   ///
   /// \param[in] lDataAddress
   /// Address of the buffer described by "lpDataDesc".  Whether this address
   /// is physical or virtual will depend on the use case and whether or not
   /// the buffer is meant to accessed via DMA.
   ///
   /// \return
   /// 0 if successful, non-zero if an error occurred.
   /////////////////////////////////////////////////////////////////////////
   int32_t Init(DataDesc* lpDataDesc, uintptr_t lDataAddress);
   
   //---------
   //members
   //---------

   DataDesc* mpDataDesc;
   uintptr_t mDataAddress; //meant to contain the DMA friendly (i.e. physical) address to the start of the data described by mDataDesc
};

#endif //_BUF_DESC_HPP
