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

#ifndef ACFUTILITIES_H
#define ACFUTILITIES_H

#include <acf_common.h>
#include <acf_hwutilities.h>
#include <apu2_kernel_port.h> //needed for 'kernel_io_desc'
#include <acf_cbmanager.hpp>
#include <stdint.h>

#ifdef __ACF_REDUCE_CODE_SIZE__
#define _RETIODESC_SCL   RetIoDescScl_noinline
#define _RETIODESC_VEC08 RetIoDescVec08_noinline
#define _RETIODESC_VEC16 RetIoDescVec16_noinline
#define _RETIODESC_VEC32 RetIoDescVec32_noinline
#else
#define _RETIODESC_SCL   RetIoDescScl
#define _RETIODESC_VEC08 RetIoDescVec08
#define _RETIODESC_VEC16 RetIoDescVec16
#define _RETIODESC_VEC32 RetIoDescVec32
#endif

extern int16_t gsACF_FIRST_TILE_FLAG;
extern int16_t gsACF_LAST_TILE_FLAG;

void memcpy_int(int* lpSrc, int* lpDst, int lSize); //size is in units of 'ints'
void memset_vec16s(vec16s* lpDst, int16_t lVal, int lSize); //size is in units of "vec16s"
//kernel_io_desc RetIoDesc(ACF_LocalMemDesc& lLmDesc, int lIter, int lLmIndex); //only called by resize...
void ACF_CollectVarLengthScatteredLists(int lIter,
                                        ACF_LocalMemDesc lLmDescCmem,
                                        ACF_LocalMemDesc lLmDescDmem,
                                        vec16u lvBytesPushedIntoFifo,
                                        ACF_AdvTransferInfo* lpAdvTransferInfo,
                                        uint32_t* lpTotalFifoSize,
                                        int lStartCu, int lTileWidthInChunks);

/////////////////////////////////////////////////////////////////////////
/// Indicate to ACF that the kernel output 'lIoDescVecBuffer' contains
/// a series of 1D lists of length 'lvVecBufferSize'.
///
/// \param[in] lIoDescVecBuffer
/// The kernel_io_desc associated with the desired ACF_ATTR_VEC_OUT_FIFO port.
///
/// \param[in] lvVecBufferSize
/// A vector containing the 1D list size information in bytes.
///
/// \param lIoDescFifoSize
/// The kernel_io_desc associated with the ACF_ATTR_SCL_OUT_STATIC_FIXED port
/// that will contain the final 1D list size in bytes.
/////////////////////////////////////////////////////////////////////////
void ACF_PUSH_VEC_BUFFER_TO_FIFO(kernel_io_desc lIoDescVecBuffer,
                                 vec16u lvVecBufferSize,
                                 kernel_io_desc lIoDescFifoSize);

/////////////////////////////////////////////////////////////////////////
/// \par Descriptions:
/// This function performs padding in CMEM.
/// \param [in] lLM                   Structure which contains the information detailing the layout of CMEM,
///                                       including allocated padding buffer sizes, chunk height
///                                       and width, buffer pointers, number of buffers, and
///                                       e0 information
/// \param [in] lCbIndex              Circular buffer index
/// \param [in] lIter                 The current iteration number.
/// \param [in] lNumIterations        The total number of iterations to be processed.
/// \param [in] lTileWidthInChunks    The width of the processing, in CUS.  Currently, it is assumed
///                                       that the processing starts at CU 0.
/// \param [in] lPadTop               The number of e0's to pad at the top of the chunk.
/// \param [in] lPadBot               The number of e0's to pad at the bottom of the chunk.
/// \param [in] lPadLeft              The number of e0's to pad at the left of the chunk.
/// \param [in] lPadRight             The number of e0's to pad at the right of the chunk.
/// \return
///   No return value is given.
/////////////////////////////////////////////////////////////////////////

/* All 12 padding functions below have this same signature:
void ACF_PAD_##_e0####_L#######R(ACF_LocalMemDesc* lpLM,
                                 int lCbIndex,
                                 int lIter,
                                 int lNumIterations,
                                 int lTileWidthInChunks,
                                 int lPadTop,
                                 int lPadBot,
                                 int lPadLeft,
                                 int lPadRight,
                                 int lPadChOffsetTop,
                                 int lPadChOffsetBot,
                                 int lPadChOffsetLeft,
                                 int lPadChOffsetRight,
                                 int lCuArrayWidth);*/

void ACF_PAD_08_e0xis1_LequaltoR    (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_08_e0xis1_LnotequaltoR (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_08_e0xnot1_LequaltoR   (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_08_e0xnot1_LnotequaltoR(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_16_e0xis1_LequaltoR    (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_16_e0xis1_LnotequaltoR (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_16_e0xnot1_LequaltoR   (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_16_e0xnot1_LnotequaltoR(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_32_e0xis1_LequaltoR    (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_32_e0xis1_LnotequaltoR (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_32_e0xnot1_LequaltoR   (ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_32_e0xnot1_LnotequaltoR(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_08(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_16(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);
void ACF_PAD_32(ACF_LocalMemDesc*, int, int, int, int, int, int, int, int, int, int, int, int, int);

/////////////////////////////////////////////////////////////////////////
/// \par Descriptions:
/// This function performs circular buffer related padding, in CMEM.
/// \param [in] lLM                   Structure which contains the information detailing the layout of CMEM,
///                                       including allocated padding buffer sizes, chunk height
///                                       and width, buffer pointers, number of buffers, and
///                                       e0 information
/// \param [in] lCbIndex              Circular buffer index
/// \param [in] lIter                 The current iteration number.
/// \param [in] lNumIterations        The total number of iterations to be processed.
/// \param [in] lPadTop               The number of e0 blocks to pad at the top of the chunk.
/// \param [in] lPadBot               The number of e0 blocks to pad at the bottom of the chunk.
/// \param [in] lPadLeft              The number of e0 blocks to pad at the left of the chunk.
/// \param [in] lPadRight             The number of e0 blocks to pad at the right of the chunk.
/// \return
///   No return value is given.
/////////////////////////////////////////////////////////////////////////

void ACF_COPYBUF(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft);
void ACF_COPYBUF_08(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft);
void ACF_COPYBUF_16(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft);
void ACF_COPYBUF_32(ACF_LocalMemDesc* lpLM, int lCbIndex, int lIter, int lNumIterations, int lPadTop, int lPadBot, int lPadLeft);

int LmInit(ACF_LocalMemDesc* pLmDesc,
           acf_scenario_buffer_data* pSenarioBufferData,
           ICP_DATATYPE dataType, int e0x, int e0y,
           int pairedLmIndex,
           int padOffsetR, int padOffsetRefChunkWidth, int padOffsetB, int padOffsetRetChunkHeight,
           int memTypeFlag, //0=SCL, 1=VEC
           int verDepOverlapFlag);

#if (1 == APEX_SIM_NATIVE)
void SimLmInitAdjustment(ACF_LocalMemDesc* lpLmDescArray, int lNumLm, uint32_t lCmemHeapBaseAddr);
#endif

#if ((1 == ACF_SIM_ISS) || (1 == ACF_SIM_NATIVE))
#define ACF_WAIT(X) //empty
void AdvanceSimulation(int lApexId);
#endif

#if (!((1 == ACF_SIM_ISS) || (1 == ACF_SIM_NATIVE)))
#define ACF_WAIT(X) wait(X); //this function exists on target by default (waits until X expires or interrupt occurs)
#endif

void IoDescInit(kernel_io_desc* pIoDesc,
                ACF_LocalMemDesc* pLmDesc,
                int lmIndex);

kernel_io_desc* RetIoDescScl_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
kernel_io_desc* RetIoDescVec08_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
kernel_io_desc* RetIoDescVec16_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
kernel_io_desc* RetIoDescVec32_noinline(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);

void LlineArrayForward(AcfCircularLineManager* lMan);
void LlineArrayForwardBuf(AcfCircularLineManager* lMan);

inline void LlineArrayForwardPow2(AcfCircularLineManager* lMan)
{
   for (int lIter = 0; lIter < lMan->lNumOfLines; ++lIter)
   {
      lMan->pLldata[lIter].lineIndex = (lMan->pLldata[lIter].lineIndex + 1) & lMan->lPow2Mask;
      lMan->pLldata[lIter].linePointer = lMan->lLpLut[lMan->pLldata[lIter].lineIndex];
   }
}

inline void LlineArrayForwardBufPow2(AcfCircularLineManager* lMan)
{
   for (int lIter = 0; lIter < lMan->lNumOfLines; ++lIter)
   {
      lMan->pLldata[lIter].lineIndex = (lMan->pLldata[lIter].lineIndex + 1) & lMan->lPow2Mask;
      lMan->pLldata[lIter].linePointer = lMan->lLpLut[lMan->pLldata[lIter].lineIndex];
   }
}

//---------------------
//INLINE FUNCTIONS
//---------------------

#ifdef CGV_FORCE_INLINE
#define CGV_INLINE_PREFIX inline
#else
#define CGV_INLINE_PREFIX
#endif

CGV_INLINE_PREFIX kernel_io_desc* RetIoDescScl(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec08(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec16(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec32(ACF_LocalMemDesc* lpLmDesc, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescScl(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec08(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec16(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX kernel_io_desc* RetIoDescVec32(CbManager* lpCbManager, kernel_io_desc* lpKernelIoDesc);
CGV_INLINE_PREFIX void ACF_UpdateTileFlags(int lIter, int lNumIter);
CGV_INLINE_PREFIX void* RetAddrScl_iter(ACF_LocalMemDesc* lpLM, int lIter);
CGV_INLINE_PREFIX void* RetAddrVec08_iter(ACF_LocalMemDesc* lpLM, int lIter);
CGV_INLINE_PREFIX void* RetAddrVec16_iter(ACF_LocalMemDesc* lpLM, int lIter);
CGV_INLINE_PREFIX void* RetAddrVec32_iter(ACF_LocalMemDesc* lpLM, int lIter);

#ifdef CGV_FORCE_INLINE
#include "acf_utilities_inline.h" //include implementations for inline functions
#endif

#endif /* ACFUTILITIES_H */
