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

#ifndef APU2KERNELPORT_H
#define APU2KERNELPORT_H

#include <stdint.h>

// NB:  This file needs to be APU Compiler Compliant.

typedef struct _kernel_io_desc
{
   void* pMem;
   uint16_t chunkWidth;
   uint16_t chunkHeight;
   uint16_t chunkSpan;

   //--------------------------------------
   //following fields are for framework use
   //--------------------------------------
   uint16_t _framework_lmindex; //ACF_LocalMemDesc index (in gLocalMemArray[]) associated with this io
   int16_t  _framework_cbindex; //keep track of logical location within circular buffer
} kernel_io_desc;
//--------------------------------


#if 0

// Port Structs
// To be used in kernel definitions
// NB: n ports requires n-1 strides.
#define MAX_D_PER_PORT 2

// cx
struct _port_vec08u
{
   vec08u*  mPtr;
   uint16_t mSize[2];    // width, height
   uint16_t mStride[1];  // span or stride??
};

typedef enum
{
   input = 1,
   output,
} PORT_INTERFACE_TYPE;


// CMEM
// 8-bit
struct _PORT08S_CMEM_STRUCT_tag{
  vec08s     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT08S_CMEM_STRUCT_tag PORT08S_CMEM;
struct _PORT08U_CMEM_STRUCT_tag{
  vec08u     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT08U_CMEM_STRUCT_tag PORT08U_CMEM;

// 16-bit
struct _PORT16S_CMEM_STRUCT_tag{
  vec08s     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT16S_CMEM_STRUCT_tag PORT16S_CMEM;
struct _PORT16U_CMEM_STRUCT_tag{
  vec08u     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT16U_CMEM_STRUCT_tag PORT16U_CMEM;

// 32-bit
struct _PORT32S_CMEM_STRUCT_tag{
  vec08s     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT32S_CMEM_STRUCT_tag PORT32S_CMEM;
struct _PORT32U_CMEM_STRUCT_tag{
  vec08u     *ptr;
  uint8_t    size[MAX_D_PER_PORT];
  uint16_t   stride[MAX_D_PER_PORT-1];
  PORT_INTERFACE_TYPE    portIF;
};
typedef _PORT32U_CMEM_STRUCT_tag PORT32U_CMEM;

// DMEM
// as above, for now
typedef _PORT08S_CMEM_STRUCT_tag PORT08S_DMEM;
typedef _PORT08U_CMEM_STRUCT_tag PORT08U_DMEM;
typedef _PORT16S_CMEM_STRUCT_tag PORT16S_DMEM;
typedef _PORT16U_CMEM_STRUCT_tag PORT16U_DMEM;
typedef _PORT32S_CMEM_STRUCT_tag PORT32S_DMEM;
typedef _PORT32U_CMEM_STRUCT_tag PORT32U_DMEM;

#endif //#if 0

#endif /* APU2KERNELPORT_H */
