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

#ifndef _APU_MEM_MGMT_HPP
#define _APU_MEM_MGMT_HPP

#include <stdint.h>

/*NOTES:
- vector allocation is specifically typed because native does not (cannot?) support transparent
casting from one vector type to another
- there is no alignment parameter here yet; definitely required in the future (or perhaps all allocations can default
to 4 or 8 byte alignment to keep things simple?)
*/

vec08s* ApuMemAllocVec08(int32_t lSizeInBytes);
vec16s* ApuMemAllocVec16(int32_t lSizeInBytes);
vec32s* ApuMemAllocVec32(int32_t lSizeInBytes);
int8_t* ApuMemAllocSclDmem(int32_t lSizeInBytes);
int8_t* ApuMemAllocSclSmem(int32_t lSizeInBytes);
int8_t* ApuMemAllocSclCmem(int32_t lSizeInBytes);
int32_t ApuMemFree(vec08s* lpPtr);
int32_t ApuMemFree(vec16s* lpPtr);
int32_t ApuMemFree(vec32s* lpPtr);
int32_t ApuMemFree(int8_t* lpPtr);

















#if 0

#ifdef __clang__
#define DECLARE_BYTE_ALIGNED(_TYPE, _NAME, _BYTE_ALIGNMENT) \
_TYPE _NAME __attribute__((aligned(_BYTE_ALIGNMENT)))
#elif __chess__
#define DECLARE_BYTE_ALIGNED(_TYPE, _NAME, _BYTE_ALIGNMENT) \
_TYPE chess_storage(%_BYTE_ALIGNMENT) _NAME
#elif _WIN32
#define DECLARE_BYTE_ALIGNED(_TYPE, _NAME, _BYTE_ALIGNMENT) \
_TYPE _NAME
#else //standalone for now ... may need others here depending on required compiler support
#define DECLARE_BYTE_ALIGNED(_TYPE, _NAME, _BYTE_ALIGNMENT) \
_TYPE _NAME __attribute__((aligned(_BYTE_ALIGNMENT)))
#endif

vec08s* AllocFromVecHeap08_dummy(int32_t sizeInBytes);
vec16s* AllocFromVecHeap16_dummy(int32_t sizeInBytes);
vec32s* AllocFromVecHeap32_dummy(int32_t sizeInBytes);
int8_t* AllocFromSclHeap_dummy(int32_t sizeInBytes);
int8_t* AllocFromSclHeapDmem_dummy(int32_t sizeInBytes);
int8_t* AllocFromSclHeapSmem_dummy(int32_t sizeInBytes);

#endif

#endif //_APU_MEM_MGMT_HPP
