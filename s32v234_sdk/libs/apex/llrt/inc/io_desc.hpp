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

#ifndef _IO_DESC_HPP
#define _IO_DESC_HPP

#include <stdint.h>
#include <data_desc.hpp>

//NOTE:
//- size of IoDesc must be consistent whether or not the processor architecture is 32 or 64 bit!!
//- alignment of the struct itself is supposed to correspond to the alignment of the strictest member (i.e. mAddr).
//  On a 64-bit system, the struct should be aligned on 64-bit boundary due to the uint64_t member. The 
//  struct contains explicit alignment padding after mDataDesc to make sure mAddr is aligned on a 64-bit boundary.
//  With the alignment padding, sizeof(IoDesc) should be 152 bytes on both 64 and 32 bit architectures.

class IoDesc
{
public:
   DataDesc mDataDesc;       //160 bytes
   uint64_t mAddr;           //8   bytes
};

#endif //_IO_DESC_HPP
