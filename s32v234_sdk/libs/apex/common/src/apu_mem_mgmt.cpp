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

#include <malloc.h>
#include <c_malloc.h>

//TODO: all scalar memory alloc is currently routed through malloc; need to look into this if we want to
//      differentiate between dmem, smem, scalar cmem

vec08s* ApuMemAllocVec08(int32_t lSizeInBytes)
{
   vec08s* lpPtr = (vec08s*)c_malloc(lSizeInBytes);
   return lpPtr;
}

vec16s* ApuMemAllocVec16(int32_t lSizeInBytes)
{
   vec16s* lpPtr = (vec16s*)c_malloc(lSizeInBytes);
   return lpPtr;
}

vec32s* ApuMemAllocVec32(int32_t lSizeInBytes)
{
   vec32s* lpPtr = (vec32s*)c_malloc(lSizeInBytes);
   return lpPtr;
}

int8_t* ApuMemAllocSclDmem(int32_t lSizeInBytes)
{
   int8_t* lpPtr = (int8_t*)malloc(lSizeInBytes);
   return lpPtr;
}

int8_t* ApuMemAllocSclSmem(int32_t lSizeInBytes)
{
   int8_t* lpPtr = (int8_t*)malloc(lSizeInBytes);
   return lpPtr;
}

int8_t* ApuMemAllocSclCmem(int32_t lSizeInBytes)
{
   int8_t* lpPtr = (int8_t*)malloc(lSizeInBytes);
   return lpPtr;
}

int32_t ApuMemFree(vec08s* lpPtr)
{
   c_free_((uint32_t)(uintptr_t)lpPtr);
   return 0;
}

int32_t ApuMemFree(vec16s* lpPtr)
{
   c_free_((uint32_t)(uintptr_t)lpPtr);
   return 0;
}

int32_t ApuMemFree(vec32s* lpPtr)
{
   c_free_((uint32_t)(uintptr_t)lpPtr);
   return 0;
}

int32_t ApuMemFree(int8_t* lpPtr)
{
   free(lpPtr);
   return 0;
}












#if 0

#include <util.h>

#define VECTOR_HEAP_SIZE_IN_BYTES (3584)
#define SCALAR_HEAP_SIZE_IN_BYTES_DMEM (1024*64)
//#define SCALAR_HEAP_SIZE_IN_BYTES_SMEM (1024*64)

DECLARE_BYTE_ALIGNED(static vec08s, gVecHeap[VECTOR_HEAP_SIZE_IN_BYTES],    16);
DECLARE_BYTE_ALIGNED(static int8_t, gSclHeapDmem[SCALAR_HEAP_SIZE_IN_BYTES_DMEM], 16);
//DECLARE_BYTE_ALIGNED(static int8_t, gSclHeapSmem[SCALAR_HEAP_SIZE_IN_BYTES_SMEM], 16);

static int32_t gsVecHeapSizeInBytes = VECTOR_HEAP_SIZE_IN_BYTES;
static int32_t gsSclHeapSizeInBytesDmem = SCALAR_HEAP_SIZE_IN_BYTES_DMEM;
//static int32_t gsSclHeapSizeInBytesSmem = SCALAR_HEAP_SIZE_IN_BYTES_SMEM;
static int32_t gVecHeapIndex = 0;
static int32_t gSclHeapIndexDmem = 0;
//static int32_t gSclHeapIndexSmem = 0;

vec08s* ApuMemAllocVec08(int32_t lSizeInBytes)
{
   vec08s* lRetVal = 0;
   if ((gsVecHeapSizeInBytes - gVecHeapIndex) >= lSizeInBytes)
   {
      lRetVal = &gVecHeap[gVecHeapIndex];
      gVecHeapIndex += ((lSizeInBytes+3)&0xFFFFFFFC); //ensure all allocated buffers are aligned on 4-byte boundaries
   }
   return lRetVal;
}

vec16s* ApuMemAllocVec16(int32_t lSizeInBytes)
{
   return (vec16s*)ApuMemAllocVec08(lSizeInBytes);
}

vec32s* ApuMemAllocVec32(int32_t lSizeInBytes)
{
   return (vec32s*)ApuMemAllocVec08(lSizeInBytes);
}

int8_t* ApuMemAllocSclDmem(int32_t lSizeInBytes)
{
   int8_t* lRetVal = 0;
   if ((gsSclHeapSizeInBytesDmem - gSclHeapIndexDmem) >= lSizeInBytes)
   {
      lRetVal = &gSclHeapDmem[gSclHeapIndexDmem];
      gSclHeapIndexDmem += ((lSizeInBytes+15)&0xFFFFFFF0); //ensure all allocated buffers are aligned on 16-byte boundaries
   }
   return lRetVal;
}

int8_t* ApuMemAllocSclSmem(int32_t lSizeInBytes)
{
   return ApuMemAllocSclDmem(lSizeInBytes);
}

int8_t* ApuMemAllocSclCmem(int32_t lSizeInBytes)
{
   return ApuMemAllocSclDmem(lSizeInBytes);
}

int32_t ApuMemFree(vec08s* lpPtr)
{
   return 0;
}

int32_t ApuMemFree(vec16s* lpPtr)
{
   return 0;
}

int32_t ApuMemFree(vec32s* lpPtr)
{
   return 0;
}

int32_t ApuMemFree(int8_t* lpPtr)
{
   return 0;
}

#endif





