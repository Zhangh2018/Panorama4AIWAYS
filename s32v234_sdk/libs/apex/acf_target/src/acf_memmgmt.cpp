/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include "acf_memmgmt.h"

#if 0
//#define VECTOR_HEAP_SIZE_IN_BYTES (1024)
//#define SCALAR_HEAP_SIZE_IN_BYTES (1024*15)
//DECLARE_BYTE_ALIGNED(static vec08s, gVecHeap[VECTOR_HEAP_SIZE_IN_BYTES], 4);
//DECLARE_BYTE_ALIGNED(static int8_t, gSclHeap[SCALAR_HEAP_SIZE_IN_BYTES], 4);
#endif

static int32_t gsVecHeapSizeInBytes = 0;
static int32_t gsSclHeapSizeInBytes = 0;
static vec08s* gspVecHeap           = 0;
static int8_t* gspSclHeap           = 0;
static int32_t gVecHeapIndex        = 0;
static int32_t gSclHeapIndex        = 0;

void InitVecHeap(vec08s* pVecHeapBase, int32_t heapSizeInBytes)
{
   gspVecHeap = pVecHeapBase;
   gsVecHeapSizeInBytes = heapSizeInBytes;
   gVecHeapIndex = 0;
}

void InitSclHeap(int8_t* pSclHeapBase, int32_t heapSizeInBytes)
{
   gspSclHeap = pSclHeapBase;
   gsSclHeapSizeInBytes = heapSizeInBytes;
   gSclHeapIndex = 0;
}

vec08s* AllocFromVecHeap(int32_t sizeInBytes)
{
   vec08s* lRetVal = 0;
   if ((gsVecHeapSizeInBytes - gVecHeapIndex) >= sizeInBytes)
   {
      lRetVal = &gspVecHeap[gVecHeapIndex];
      gVecHeapIndex += ((sizeInBytes+3)&0xFFFFFFFC); //ensure all allocated buffers are aligned on 4-byte boundaries
   }
   return lRetVal;
}

int8_t* AllocFromSclHeap(int32_t sizeInBytes)
{
   int8_t* lRetVal = 0;
   if ((gsSclHeapSizeInBytes - gSclHeapIndex) >= sizeInBytes)
   {
      lRetVal = &gspSclHeap[gSclHeapIndex];
      gSclHeapIndex += ((sizeInBytes+15)&0xFFFFFFF0); //ensure all allocated buffers are aligned on 16-byte boundaries
   }
   return lRetVal;
}

vec08s* RetVecHeapBaseAddr()
{
   return gspVecHeap;
}
