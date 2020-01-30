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

// -----------------------------------------------
// Do not re-order below define/include sequence!
// -----------------------------------------------
#define __HRSZ_HAL_SCOPE__
#include "string.h" ////memset 

typedef unsigned int REG32;
typedef signed int REG32S;
#include "apex_642cl_hrsz_map.h"
////#include "apex_642cl_map.h"

//// see apex_642cl_hrsz_map.h
////volatile __APEX_642CL_HRSZ_CORE_DCR        *gpHrszReg[2];     // 32'h00000000
////volatile APEX_642CL_HRSZ_PUSH_POP_CORE_DCR *gpHrszFifoReg[2]; // 32'h00000100
volatile APEX_642CL_HRSZ_CORE_DCR        *gpHrszReg[2];     // 32'h00000000
volatile APEX_642CL_HRSZ_PUSH_POP_CORE_DCR *gpHrszFifoReg[2]; // 32'h00000100
// -----------------------------------------------

#include "hw_reg_user.h"
#include "register_access.h"
////#include "debug.h"

////#include <stdio.h>
// #include "system.h"


#include "hrsz_hal.h"

//#define APEX_ASSERT(expr)   {assert(expr);}
#define APEX_ASSERT(expr)



// *****************************************************************************
// Horizontal Resizer driver implementation
// *****************************************************************************

static unsigned int gMaxHrszCh = 0;

/*!
 * Find max available Hrsz channels
 */
LIB_RESULT hrsz_hal_chk_hw(void)
{
   unsigned int i;

   APEX_ASSERT(gpHrszReg != NULL);

   for(i = 0; i < HRSZ_MAX_CORE; i++)
   {
      // Detect using INT Mask Register
      // Unmask all INT Mask register
      gpHrszReg[i]->IrqMask.as32BitValue = 0;
      if(gpHrszReg[i]->IrqMask.as32BitValue == 0)
      {
         // Mask all INT Mask register
         gpHrszReg[i]->IrqMask.as32BitValue = 0xFFFFFFFF;
         if(gpHrszReg[i]->IrqMask.as32BitValue != 0)
         {
            // Increase available channel number only if this register is accessible.
            gMaxHrszCh++;
         }
      }
   }

   return LIB_SUCCESS;
}

unsigned int hrsz_hal_get_max_channel(void)
{
   return gMaxHrszCh;
}

LIB_RESULT hrsz_hal_init(void)
{
   static int gHrszInitialized = 0;
   void      *vpHrszReg        = NULL;

   // hApex2IntDisable(APEX2_INT_SEQUENCER);

   if(gHrszInitialized == 0) // don't do this twice
   {
      // Initialize global pointer to Hrsz register map
      #if 0
      // gpHrszReg           = (APEX_642CL_HRSZ_DCR *)ReturnVirtualHwPtr(0, HW_REG_HOR_SCALER);
      gpHrszReg        = (volatile APEX_642CL_HRSZ_DCR *)APEX_642CL_HRSZ_BASE_ADDRESS;
      #else
//      vpHrszReg        = ReturnVirtualHwPtr(0, HW_REG_HRSZ);
      vpHrszReg        = ReturnVirtualHwPtr(0, HW_REG_HOR_SCALER);
////      gpHrszReg[0]     = (volatile __APEX_642CL_HRSZ_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_CORE_0_OFFSET);
////      gpHrszReg[1]     = (volatile __APEX_642CL_HRSZ_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_CORE_1_OFFSET);
      gpHrszReg[0]     = (volatile APEX_642CL_HRSZ_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_CORE_0_OFFSET);
      gpHrszReg[1]     = (volatile APEX_642CL_HRSZ_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_CORE_1_OFFSET);
      gpHrszFifoReg[0] = (volatile APEX_642CL_HRSZ_PUSH_POP_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_PUSH_POP_CORE_0_OFFSET);
      gpHrszFifoReg[1] = (volatile APEX_642CL_HRSZ_PUSH_POP_CORE_DCR *)((uint32_t)vpHrszReg + APEX_642CL_HRSZ_PUSH_POP_CORE_1_OFFSET);
      #endif

      hrsz_hal_chk_hw();

      gHrszInitialized = 1;
   }

   return LIB_SUCCESS;
}

LIB_RESULT hrsz_hal_en(unsigned int core_id)
{
   reg32write(gpHrszReg[core_id]->Instruction, ClkGateEnb, 1);

   return LIB_SUCCESS;
}

LIB_RESULT hrsz_hal_dis(unsigned int core_id)
{
   reg32write(gpHrszReg[core_id]->Instruction, ClkGateEnb, 0);

   return LIB_SUCCESS;
}

void hrsz_hal_set_src_woi(unsigned int core_id,
                          unsigned int width,
                          unsigned int height,
                          unsigned int h_offset,
                          unsigned int v_offset,
                          unsigned int h_valid,
                          unsigned int v_valid)
{
   APEX_ASSERT(gpHrszReg != NULL);
   APEX_ASSERT((h_offset + h_valid) <= width);
   APEX_ASSERT((v_offset + v_valid) <= height);

   reg32write(gpHrszReg[core_id]->Woi1SrcRes,
              Width, width,
              Height, height);
   reg32write(gpHrszReg[core_id]->Woi1Crop0,
              HOffset, h_offset,
              VOffset, v_offset);
   reg32write(gpHrszReg[core_id]->Woi1Crop1,
              HValid, h_valid,
              VValid, v_valid);
}

void hrsz_hal_set_dst_woi(unsigned int core_id,
                          unsigned int width,
                          unsigned int /*height*/,
                          unsigned int h_offset,
                          unsigned int v_offset,
                          unsigned int h_valid,
                          unsigned int v_valid)
{
   APEX_ASSERT(gpHrszReg != NULL);
   APEX_ASSERT((h_offset + h_valid) <= width);
   //APEX_ASSERT((v_offset + v_valid) <= height);

   reg32write(gpHrszReg[core_id]->Woi2SrcRes,
              Width, width);
   reg32write(gpHrszReg[core_id]->Woi2Crop0,
              HOffset, h_offset,
              VOffset, v_offset);
   reg32write(gpHrszReg[core_id]->Woi2Crop1,
              HValid, h_valid,
              VValid, v_valid);
}

void hrsz_hal_flush_fifo(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));

   reg32write(gpHrszReg[core_id]->FlushInit,
              Flush0, 1,
              Flush1, 1,
              Flush2, 1);
}

#if 0
void hrsz_hal_set_inst(unsigned int core_id,
                       unsigned int in_format,
                       unsigned int out_format,
                       unsigned int en_lpf,
                       unsigned int en_ppf)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   reg32write(gpHrszReg[core_id]->Instruction,
              InFormat, in_format,
              OutFormat, out_format,
              LpfEn, en_lpf,
              PpfEn, en_ppf,
              ClkGateEnb, 1);
}
#endif
void hrsz_hal_set_data_format(unsigned int core_id,
                              unsigned int in_format,
                              unsigned int out_format)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   reg32write(gpHrszReg[core_id]->Instruction,
              InFormat, in_format,
              OutFormat, out_format);
}

#if 0
// Clamp for output only
void hrsz_hal_set_clamp(unsigned int core_id, unsigned short format, HRSZ_CLAMP_LEVEL level)
{
   APEX_ASSERT(gpHrszReg != NULL);
   // APEX_ASSERT(gHrszClampMaxLut[level][format] > gHrszClampMinLut[level][format]);

   reg32write(gpHrszReg[core_id]->OutClamp,
              Min, gHrszClampMinLut[level][format],
              Max, gHrszClampMaxLut[level][format]);
}
#endif

#if 0
void hrsz_hal_start(unsigned int core_id,
                    unsigned int in_format,
                    unsigned int out_format,
                    unsigned int en_lpf,
                    unsigned int en_ppf)
{
   APEX_ASSERT(gpHrszReg != NULL);

   hrsz_hal_set_inst(core_id, in_format, out_format, en_lpf, en_ppf);
   // hrsz_hal_set_clamp(core_id, 0, 0xFFFF);

   reg32write(gpHrszReg[core_id]->FlushInit,
              Flush0, 1,
              Flush1, 1,
              Flush2, 1);
   reg32write(gpHrszReg[core_id]->FlushInit,
              Init, 1);
   reg32write(gpHrszReg[core_id]->FlushInit,
              Start, 1);
}

#else
void hrsz_hal_start(unsigned int core_id)
{
   APEX_ASSERT(gpHrszReg != NULL);

   // hrsz_hal_set_inst(core_id, in_format, out_format, en_lpf, en_ppf);
   // hrsz_hal_set_clamp(core_id, 0, 0xFFFF);

   #if 0
   reg32write(gpHrszReg[core_id]->FlushInit,
              Flush0, 1,
              Flush1, 1,
              Flush2, 1);
   #else
   reg32write(gpHrszReg[core_id]->FlushInit, Flush0, 1);
   reg32write(gpHrszReg[core_id]->FlushInit, Flush1, 1);
   reg32write(gpHrszReg[core_id]->FlushInit, Flush2, 1);
   #endif
   reg32write(gpHrszReg[core_id]->FlushInit,
              Init, 1);
   reg32write(gpHrszReg[core_id]->FlushInit,
              Start, 1);
}
#endif

void hrsz_hal_set_lpf_coeff(unsigned int   core_id,
                            unsigned char *pLpfCoeff,
                            unsigned int   rounding_factor)
{
   APEX_ASSERT(gpHrszReg != NULL);
   APEX_ASSERT(pLpfCoeff != NULL);

   reg32write(gpHrszReg[core_id]->LpfCoef[0],
              YCoef0, pLpfCoeff[0],
              YCoef1, pLpfCoeff[1],
              YCoef2, pLpfCoeff[2]);
   reg32write(gpHrszReg[core_id]->LpfCoef[1],
              YCoef0, pLpfCoeff[3],
              YCoef1, pLpfCoeff[4],
              YCoef2, pLpfCoeff[5]);
   reg32write(gpHrszReg[core_id]->LpfCoef[2],
              YCoef0, pLpfCoeff[6],
              YCoef1, pLpfCoeff[7],
              YCoef2, pLpfCoeff[8]);
   reg32write(gpHrszReg[core_id]->LpfCoef3,
              YCoef0, pLpfCoeff[9],
              YCoef1, pLpfCoeff[10]);
   reg32write(gpHrszReg[core_id]->Instruction,
              LpfRoundFactor, rounding_factor);
   reg32write(gpHrszReg[core_id]->Instruction,
              LpfEn, 1);
}

void hrsz_hal_set_avg_mode(unsigned int core_id, unsigned char avg_mode)
{
   APEX_ASSERT(gpHrszReg != NULL);
   APEX_ASSERT(avg_mode <= 5);

   reg32write(gpHrszReg[core_id]->Instruction,
              HAvgMode, avg_mode);
}

float hrsz_hal_get_scale_factor(unsigned int in_res, unsigned int out_res)
{
   float f_src_h_valid = (float)in_res;
   float f_dst_h_valid = (float)out_res;

   return f_dst_h_valid / f_src_h_valid;
}

#if 0
static signed short hrsz_hal_get_signed_clamp(signed int data, unsigned int left_shift_num)
{
   signed int temp, min, max;

   min  = 0x3FF80; // {1'b1, 17'h1FF80};
   max  = 0x001FF; // {1'b0, 17'h001FF};

   temp = data << left_shift_num;

   if(temp > max)
   {
      temp = max;
   }
   else if(temp < min)
   {
      temp = min;
   }

   return (signed short)temp;
}
#endif

#if 0
signed short hrsz_hal_apply_rounding_factor(signed short data, unsigned int rounding_factor)
{
   signed short temp, min, max;

   // min = 0x3FF80; // {1'b1, 17'h1FF80};
   // max = 0x001FF; // {1'b0, 17'h001FF};
   min = -512;
   max = 511;

   if(rounding_factor <= 3)
   {
      // signed right shift
      temp = data >> rounding_factor;
   }
   else // if ( (rounding_factor >= 4) && (rounding_factor <= 7) )
   {
      // signed left shift
      temp = data << (rounding_factor - 3);
   }

   if(temp > max)
   {
      temp = max;
   }
   else if(temp < min)
   {
      temp = min;
   }

   return temp;
}

void hrsz_hal_set_ppf_coeff(unsigned int core_id, unsigned int lut_to_use, unsigned int rounding_factor)
{
   unsigned int i;

   APEX_ASSERT(gpHrszReg != NULL);

   for(i = 0; i < 10; i++)
   {
      gpHrszReg[core_id]->PpfFilter[i].asBitField.Filter0 = hrsz_hal_apply_rounding_factor(gHrszPpfLut[lut_to_use][(i * 3) + 0], rounding_factor);
      gpHrszReg[core_id]->PpfFilter[i].asBitField.Filter1 = hrsz_hal_apply_rounding_factor(gHrszPpfLut[lut_to_use][(i * 3) + 1], rounding_factor);
      gpHrszReg[core_id]->PpfFilter[i].asBitField.Filter2 = hrsz_hal_apply_rounding_factor(gHrszPpfLut[lut_to_use][(i * 3) + 2], rounding_factor);
   }

   gpHrszReg[core_id]->PpfFilter10.asBitField.Filter0 = hrsz_hal_apply_rounding_factor(gHrszPpfLut[lut_to_use][(i * 3) + 0], rounding_factor);
   gpHrszReg[core_id]->PpfFilter10.asBitField.Filter1 = hrsz_hal_apply_rounding_factor(gHrszPpfLut[lut_to_use][(i * 3) + 1], rounding_factor);
}
#endif

#if 0
void hrsz_hal_set_lpf_filter(unsigned short *pLpfFilter)
{
   unsigned int i;

   APEX_ASSERT(gpHrszReg != NULL);
   APEX_ASSERT(pLpfFilter != NULL);

   for(i = 0; i < 10; i++)
   {
      gpHrszReg->PpfFilter[i].asBitField.Filter0 = pLpfFilter[(i * 3) + 0];
      gpHrszReg->PpfFilter[i].asBitField.Filter1 = pLpfFilter[(i * 3) + 1];
      gpHrszReg->PpfFilter[i].asBitField.Filter2 = pLpfFilter[(i * 3) + 2];
   }

   gpHrszReg->PpfFilter10.asBitField.Filter0 = pLpfFilter[(i * 3) + 0];
   gpHrszReg->PpfFilter10.asBitField.Filter1 = pLpfFilter[(i * 3) + 1];
}
#endif

#if 0
void hrsz_hal_set_ppf(unsigned int core_id,
                      unsigned int in_res,
                      unsigned int out_res,
                      unsigned int rounding_factor)
{
   unsigned int lut_to_use;
   float        scale_factor;

   APEX_ASSERT(gpHrszReg != NULL);

   // Select LUT based upon scaling ratio
   scale_factor = hrsz_hal_get_scale_factor(in_res, out_res);
   if(scale_factor > 1.0)
   {
      // Scale Up
      lut_to_use = 0;
   }
   else if(scale_factor > 0.5)
   {
      // Scale Down for scaling ratio 1 to 0.5
      lut_to_use = 1;
   }
   else
   {
      // Scale Down for scaling ratio 0.4999 to 0.000001
      lut_to_use = 2;
   }

   // Set polyphase filter coefficient
   hrsz_hal_set_ppf_coeff(core_id, lut_to_use, rounding_factor);

   // Select PPF Tap
   reg32write(gpHrszReg[core_id]->Instruction,
              PpfEn, (in_res != out_res),
              PpfTapSelect, (lut_to_use == 2),
              PpfRoundFactor, rounding_factor);

   // Set PPF resolution
   reg32write(gpHrszReg[core_id]->PpfImg1Res,
              Width, out_res);

   // Set PPF Scale Factor
   reg32write(gpHrszReg[core_id]->PpfScale,
              HscaleFactor, (in_res * 1024) / out_res);
}
#endif

#if 0
void hrsz_hal_cfg(unsigned int core_id,
                  unsigned int src_width,
                  unsigned int src_height,
                  unsigned int src_h_offset,
                  unsigned int src_v_offset,
                  unsigned int src_h_valid,
                  unsigned int src_v_valid,
                  unsigned int dst_width,
                  unsigned int dst_height,
                  unsigned int dst_h_offset,
                  unsigned int dst_v_offset,
                  unsigned int dst_h_valid,
                  unsigned int dst_v_valid,
                  unsigned int rounding_factor)
{
   unsigned char LpfCoeff[9] = {0, 0, 0, 0, 64, 0, 0, 0, 0};
   float         scale_factor;
   unsigned int  h_avg_mode;

   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   APEX_ASSERT(pCfgData != NULL);
   APEX_ASSERT(pCfgData->src_h_valid <= pCfgData->dst_h_valid);
   APEX_ASSERT(pCfgData->src_v_valid == pCfgData->dst_v_valid);

   // Selecting how to configure Scaler sub modules depends on utilization of the line memories available for respective resolution.

   // Set src woi
   hrsz_hal_set_src_woi(core_id,
                        pCfgData->src_width,
                        pCfgData->src_height,
                        pCfgData->src_h_offset,
                        pCfgData->src_v_offset,
                        pCfgData->src_h_valid, pCfgData->src_v_valid);

   hrsz_hal_set_lpf_coeff(core_id, LpfCoeff, 3);

   // Set horizontal average mode
   scale_factor = hrsz_hal_get_scale_factor(pCfgData->src_h_valid, pCfgData->dst_h_valid);
   if(scale_factor > 0.5)
   {
      h_avg_mode = 0;
   }
   else if(scale_factor > 0.25)
   {
      h_avg_mode = 1;
   }
   else if(scale_factor > 0.125)
   {
      h_avg_mode = 2;
   }
   else if(scale_factor > 0.0625)
   {
      h_avg_mode = 3;
   }
   else if(scale_factor > 0.03125)
   {
      h_avg_mode = 4;
   }
   else
   {
      h_avg_mode = 5;
   }

   hrsz_hal_set_avg_mode(core_id, h_avg_mode);

   // Configure polyphase filter
   hrsz_hal_set_ppf(core_id,
                    pCfgData->src_h_valid >> h_avg_mode,
                    pCfgData->dst_h_valid,
                    pCfgData->rounding_factor);

   // Set dst woi
   hrsz_hal_set_dst_woi(core_id,
                        pCfgData->dst_width,
                        pCfgData->dst_height,
                        pCfgData->dst_h_offset,
                        pCfgData->dst_v_offset,
                        pCfgData->dst_h_valid,
                        pCfgData->dst_v_valid);
}
#endif

void hrsz_hal_cfg(unsigned int core_id, APEX_642CL_HRSZ_CORE_DCR *pHrszCfg)
{
   volatile unsigned int *pReg = NULL;
   unsigned int           i;

   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   APEX_ASSERT(pHrszCfg != NULL);

   // Configure McDma channel specific registers
   #if 0
   volatile MCDMA_CFG *pMcDmaChCfgRegs = NULL;
   pMcDmaChCfgRegs = (volatile MCDMA_CFG *)(&gpHrszReg->Ch[chNo]);
   memcpy(pMcDmaChCfgRegs, pMcDmaCfg, sizeof(MCDMA_CFG));
   #else // In order to keep Volatile attribute...
   pReg = (volatile unsigned int *)&gpHrszReg[core_id]->Instruction;

   APEX_ASSERT((unsigned int)(pReg) % 4 == 0);
   APEX_ASSERT((unsigned int)(pHrszCfg) % 4 == 0);
   APEX_ASSERT(sizeof(APEX_642CL_HRSZ_CORE_DCR) % 4 == 0);

   // Copy all configuration fields except FlushInit register that controls starting of Hrsz
   for(i = 0; i < (sizeof(APEX_642CL_HRSZ_CORE_DCR) - 4) / 4; i++)
   {
      pReg[i] = ((unsigned int *)&pHrszCfg->Instruction.as32BitValue)[i];
   }
   #endif
}

// Unmask irq
void hrsz_hal_en_irq(unsigned int core_id, unsigned int mask)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   gpHrszReg[core_id]->IrqMask.as32BitValue &= ~mask;
}

// Mask irq
void hrsz_hal_dis_irq(unsigned int core_id, unsigned int mask)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   gpHrszReg[core_id]->IrqMask.as32BitValue |= mask;
}

// Get Int Mask
unsigned int hrsz_hal_get_int_mask(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszReg[core_id]->IrqMask.as32BitValue;
}

// Read irq status
unsigned int hrsz_hal_get_irq(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszReg[core_id]->IrqStatus.as32BitValue;
}

// Clear irq flag
void hrsz_hal_clr_irq(unsigned int core_id, unsigned int mask)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   gpHrszReg[core_id]->IrqClear.as32BitValue = mask;
}

// return 0: Interrupt was disabled.
// return 1: Interrupt was enabled.
unsigned int hrsz_hal_is_en_irq(unsigned int core_id, unsigned int mask)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return !(gpHrszReg[core_id]->IrqMask.as32BitValue & mask);
}

unsigned int hrsz_hal_is_infifo_empty(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszReg[core_id]->FifoStatus.asBitField.IfifoEmpty;
}

unsigned int hrsz_hal_is_outfifo_empty(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszReg[core_id]->FifoStatus.asBitField.OfifoEmpty;
}

unsigned int hrsz_hal_get_fifo_status(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszReg[core_id]->FifoStatus.as32BitValue;
}

void hrsz_hal_push_fifo(unsigned int core_id, unsigned int data)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   gpHrszFifoReg[core_id]->as32BitValue = data;
}

unsigned int hrsz_hal_pop_fifo(unsigned int core_id)
{
   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));
   return gpHrszFifoReg[core_id]->as32BitValue;
}

void hrsz_hal_wait_polling(unsigned int core_id)
{
   unsigned int bitMask = 1 << core_id;

   APEX_ASSERT((gpHrszReg != NULL) && (core_id < HRSZ_MAX_CORE));

   while(!(gpHrszReg[core_id]->IrqStatus.as32BitValue & bitMask)) ;
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// from hrsz_drv.cpp
const signed short gHrszPpfLut[3][32] = {
   // LUT[0] supports SCALE UP
   { 0,  -11, -17, -18, -15, -10, -5, -1, 256, 245, 220, 184, 143, 100, 58,  23,  0,  23, 58, 100, 143, 184, 220, 245, 0, -1, -5, -10, -15, -18, -17, -11, },
   // LUT[1] supports SCALE DOWN 1 to 0.5
   { 28, 15,  7,   2,   0,   0,   0,  0,  200, 194, 178, 156, 128, 98,  71,  47,  28, 47, 71, 98,  128, 156, 178, 194, 0, 0,  0,  0,   0,   2,   7,   15,  },
   // LUT[2] supports SCALE DOWN 0.4999 to 0.000001
   { 2,  0,   0,   0,   32,  19,  11, 5,  94,  80,  64,  47,  94,  105, 106, 105, 32, 47, 64, 80,  2,   5,   11,  19,  0, 0,  0,  0,   0,   0,   0,   0,   },
};
signed short hrsz_hal_apply_rounding_factor(signed short data, unsigned int rounding_factor)
{
   signed short temp, min, max;

   // min = 0x3FF80; // {1'b1, 17'h1FF80};
   // max = 0x001FF; // {1'b0, 17'h001FF};
   // min = -512;
   min = -128;
   max = 511;

   if(rounding_factor <= 3)
   {
      // signed right shift
      temp = data >> (3 - rounding_factor);
   }
   else // if ( (rounding_factor >= 4) && (rounding_factor <= 7) )
   {
      // signed left shift
      temp = data << (rounding_factor - 3);
   }

   if(temp > max)
   {
      temp = max;
   }
   else if(temp < min)
   {
      temp = min;
   }

   return temp;
}

void hrsz_hal_set_ppf_coeff(APEX_642CL_HRSZ_CORE_DCR * *ppCfgData,
                            unsigned int                lut_to_use,
                            unsigned int                rounding_factor)
{
   uint32_t       i;
   signed short coef;

   APEX_ASSERT((ppCfgData != NULL) && (*ppCfgData != NULL));

   for(i = 0; i < 10; i++)
   {
      coef                                          = gHrszPpfLut[lut_to_use][(i * 3) + 0];
      (*ppCfgData)->PpfFilter[i].asBitField.Filter0 = hrsz_hal_apply_rounding_factor(coef, rounding_factor);
      coef                                          = gHrszPpfLut[lut_to_use][(i * 3) + 1];
      (*ppCfgData)->PpfFilter[i].asBitField.Filter1 = hrsz_hal_apply_rounding_factor(coef, rounding_factor);
      coef                                          = gHrszPpfLut[lut_to_use][(i * 3) + 2];
      (*ppCfgData)->PpfFilter[i].asBitField.Filter2 = hrsz_hal_apply_rounding_factor(coef, rounding_factor);
   }

   coef                                         = gHrszPpfLut[lut_to_use][(i * 3) + 0];
   (*ppCfgData)->PpfFilter10.asBitField.Filter0 = hrsz_hal_apply_rounding_factor(coef, rounding_factor);
   coef                                         = gHrszPpfLut[lut_to_use][(i * 3) + 1];
   (*ppCfgData)->PpfFilter10.asBitField.Filter1 = hrsz_hal_apply_rounding_factor(coef, rounding_factor);
}

void hrsz_hal_set_ppf(APEX_642CL_HRSZ_CORE_DCR *pCfgData,
                      unsigned int              in_res,
                      unsigned int              out_res,
                      unsigned int              padded_width,
                      unsigned int              rounding_factor)
{
   unsigned int lut_to_use;
   float        scale_factor;

   APEX_ASSERT(pCfgData != NULL);

   // Select LUT based upon scaling ratio
   scale_factor = hrsz_hal_get_scale_factor(in_res, out_res);
   if(scale_factor > 1.0)
   {
      // Scale Up
      lut_to_use = 0;
   }
   else if(scale_factor > 0.5)
   {
      // Scale Down for scaling ratio 1 to 0.5
      lut_to_use = 1;
   }
   else
   {
      // Scale Down for scaling ratio 0.4999 to 0.000001
      lut_to_use = 2;
   }

   // Set polyphase filter coefficient
   hrsz_hal_set_ppf_coeff(&pCfgData, lut_to_use, rounding_factor);

   // Select PPF Tap
   pCfgData->Instruction.asBitField.PpfEn        = (in_res != out_res);
   pCfgData->Instruction.asBitField.PpfTapSelect = (lut_to_use == 2);
   // pCfgData->Instruction.asBitField.PpfRoundFactor = rounding_factor;

   // Set PPF resolution
   pCfgData->PpfImg1Res.asBitField.Width = padded_width;

   // Set PPF Scale Factor
// NB: reference code:   
//      const int rsz_sinc   = (sw * rsz_one) / dw;
//      const int rsz_sbeg   = (rsz_sinc >> 1);
   pCfgData->PpfScale.asBitField.HscaleFactor = (in_res * 1024) / out_res;
   pCfgData->PpfScale.asBitField.HscaleStartFactor = pCfgData->PpfScale.asBitField.HscaleFactor >> 1;

}














int *hrsz_hal_get_cfg_data(uint32_t core_id)
{
   APEX_ASSERT(core_id < hrsz_hal_get_max_channel());
   return (int *)gpHrszReg[core_id];
}


LIB_RESULT hrsz_hal_cfgData(unsigned int core_id, HRSZ_CFG *pCfgData)
{
   // unsigned char LpfCoeff[9] = {0, 0, 0, 0, 64, 0, 0, 0, 0};
   float  scale_factor;
   uint32_t h_avg_mode;

   // At first, clear entire configuration parameters!
   memset((void*)gpHrszReg[core_id], 0, sizeof(APEX_642CL_HRSZ_CORE_DCR));

   hrsz_hal_en(core_id);

   APEX_ASSERT((pCfgData != NULL) && (core_id < hrsz_hal_get_max_channel()));
   APEX_ASSERT(pCfgData->src_v_valid == pCfgData->dst_height);
   APEX_ASSERT((pCfgData->src_h_offset + pCfgData->src_h_valid) <= pCfgData->src_width);
   APEX_ASSERT((pCfgData->src_v_offset + pCfgData->src_v_valid) <= pCfgData->src_height);
   APEX_ASSERT((pCfgData->dst_h_offset + pCfgData->dst_h_valid) <= pCfgData->dst_width);
   APEX_ASSERT((pCfgData->dst_v_offset + pCfgData->dst_v_valid) <= pCfgData->dst_height);

   // Enable Hrsz
   gpHrszReg[core_id]->Instruction.asBitField.ClkGateEnb = 1;

   // Set src/dst data format
   gpHrszReg[core_id]->Instruction.asBitField.InFormat  = pCfgData->src_format;
   gpHrszReg[core_id]->Instruction.asBitField.OutFormat = pCfgData->dst_format;

   // Selecting how to configure Hrsz sub modules depends on utilization of the line memories
   // available for respective resolution.

   // Set src woi
   gpHrszReg[core_id]->Woi1SrcRes.asBitField.Width  = pCfgData->src_width;
   gpHrszReg[core_id]->Woi1SrcRes.asBitField.Height = pCfgData->src_height;
   gpHrszReg[core_id]->Woi1Crop0.asBitField.HOffset = pCfgData->src_h_offset;
   gpHrszReg[core_id]->Woi1Crop0.asBitField.VOffset = pCfgData->src_v_offset;
   gpHrszReg[core_id]->Woi1Crop1.asBitField.HValid  = pCfgData->src_h_valid;
   gpHrszReg[core_id]->Woi1Crop1.asBitField.VValid  = pCfgData->src_v_valid;

   // Set LPF coefficient
   gpHrszReg[core_id]->LpfCoef[0].asBitField.YCoef0 = pCfgData->lpfCoeff[0];
   gpHrszReg[core_id]->LpfCoef[0].asBitField.YCoef1 = pCfgData->lpfCoeff[1];
   gpHrszReg[core_id]->LpfCoef[0].asBitField.YCoef2 = pCfgData->lpfCoeff[2];
   gpHrszReg[core_id]->LpfCoef[1].asBitField.YCoef0 = pCfgData->lpfCoeff[3];
   gpHrszReg[core_id]->LpfCoef[1].asBitField.YCoef1 = pCfgData->lpfCoeff[4];
   gpHrszReg[core_id]->LpfCoef[1].asBitField.YCoef2 = pCfgData->lpfCoeff[5];
   gpHrszReg[core_id]->LpfCoef[2].asBitField.YCoef0 = pCfgData->lpfCoeff[6];
   gpHrszReg[core_id]->LpfCoef[2].asBitField.YCoef1 = pCfgData->lpfCoeff[7];
   gpHrszReg[core_id]->LpfCoef[2].asBitField.YCoef2 = pCfgData->lpfCoeff[8];
   gpHrszReg[core_id]->LpfCoef3.asBitField.YCoef0   = pCfgData->lpfCoeff[9];
   gpHrszReg[core_id]->LpfCoef3.asBitField.YCoef1   = pCfgData->lpfCoeff[10];

   // Set rounding factor
   gpHrszReg[core_id]->Instruction.asBitField.LpfRoundFactor = pCfgData->lpf_rounding_factor;
   gpHrszReg[core_id]->Instruction.asBitField.PpfRoundFactor = pCfgData->ppf_rounding_factor;
   
   // Disable average block! (see email May 15 2014 from Dan L.)
   //h_avg_mode = 0;
   
   // Set horizontal average mode
   scale_factor = hrsz_hal_get_scale_factor(pCfgData->src_h_valid, pCfgData->dst_scale_width);
   if(scale_factor > 0.25) {
      h_avg_mode = 0;
   } else if(scale_factor > 0.125) {
      h_avg_mode = 1;
   } else if(scale_factor > 0.0625) {
      h_avg_mode = 2;
   } else if(scale_factor > 0.03125) {
      h_avg_mode = 3;
   } else if (scale_factor > 0.015625) {
      h_avg_mode = 4;
   } else {
      h_avg_mode = 5;
   }
   
   gpHrszReg[core_id]->Instruction.asBitField.HAvgMode = h_avg_mode;

   if(pCfgData->en_lpf)
   {
      gpHrszReg[core_id]->Instruction.asBitField.LpfEn = 1;
   }

   // Configure poly-phase filter
   hrsz_hal_set_ppf((APEX_642CL_HRSZ_CORE_DCR *)gpHrszReg[core_id],
                    pCfgData->src_h_valid >> h_avg_mode,
                    pCfgData->dst_scale_width,
                    pCfgData->dst_width,
                    pCfgData->ppf_rounding_factor);

   // Set output clamp
   gpHrszReg[core_id]->OutClamp.asBitField.Min = pCfgData->dst_clamp_min;
   gpHrszReg[core_id]->OutClamp.asBitField.Max = pCfgData->dst_clamp_max;

   // Set dst woi
   gpHrszReg[core_id]->Woi2SrcRes.asBitField.Width  = pCfgData->dst_width;
   gpHrszReg[core_id]->Woi2SrcRes.asBitField.Height = pCfgData->dst_height;
   gpHrszReg[core_id]->Woi2Crop0.asBitField.HOffset = pCfgData->dst_h_offset;
   gpHrszReg[core_id]->Woi2Crop0.asBitField.VOffset = pCfgData->dst_v_offset;
   gpHrszReg[core_id]->Woi2Crop1.asBitField.HValid  = pCfgData->dst_h_valid;
   gpHrszReg[core_id]->Woi2Crop1.asBitField.VValid  = pCfgData->dst_v_valid;

   return LIB_SUCCESS;
}

// Similar to hrsz_hal_cfgData, but puts the 
LIB_RESULT hrsz_hal_get_sideband(uint32_t* pBuffer, HRSZ_CFG *pCfgData)
{
   APEX_642CL_HRSZ_SIDEBAND* pSideband = (APEX_642CL_HRSZ_SIDEBAND*) pBuffer;

   // unsigned char LpfCoeff[9] = {0, 0, 0, 0, 64, 0, 0, 0, 0};
   float  scale_factor;
   uint32_t h_avg_mode;

   // At first, clear entire configuration parameters!
   memset((void*)pSideband, 0, sizeof(APEX_642CL_HRSZ_SIDEBAND));

   APEX_ASSERT((pCfgData != NULL) && (core_id < hrsz_hal_get_max_channel()));
   APEX_ASSERT(pCfgData->src_v_valid == pCfgData->dst_height);
   APEX_ASSERT((pCfgData->src_h_offset + pCfgData->src_h_valid) <= pCfgData->src_width);
   APEX_ASSERT((pCfgData->src_v_offset + pCfgData->src_v_valid) <= pCfgData->src_height);
   APEX_ASSERT((pCfgData->dst_h_offset + pCfgData->dst_h_valid) <= pCfgData->dst_width);
   APEX_ASSERT((pCfgData->dst_v_offset + pCfgData->dst_v_valid) <= pCfgData->dst_height);

   // Enable Hrsz
   pSideband->Instruction.asBitField.ClkGateEnb = 1;

   // Set src/dst data format
   pSideband->Instruction.asBitField.InFormat  = pCfgData->src_format;
   pSideband->Instruction.asBitField.OutFormat = pCfgData->dst_format;

   // Selecting how to configure Hrsz sub modules depends on utilization of the line memories
   // available for respective resolution.

   // Set src woi
   pSideband->Woi1SrcRes.asBitField.Width  = pCfgData->src_width;
   pSideband->Woi1SrcRes.asBitField.Height = pCfgData->src_height;
   pSideband->Woi1Crop0.asBitField.HOffset = pCfgData->src_h_offset;
   pSideband->Woi1Crop0.asBitField.VOffset = pCfgData->src_v_offset;
   pSideband->Woi1Crop1.asBitField.HValid  = pCfgData->src_h_valid;
   pSideband->Woi1Crop1.asBitField.VValid  = pCfgData->src_v_valid;

   // Set LPF coefficient
   pSideband->LpfCoef[0].asBitField.YCoef0 = pCfgData->lpfCoeff[0];
   pSideband->LpfCoef[0].asBitField.YCoef1 = pCfgData->lpfCoeff[1];
   pSideband->LpfCoef[0].asBitField.YCoef2 = pCfgData->lpfCoeff[2];
   pSideband->LpfCoef[1].asBitField.YCoef0 = pCfgData->lpfCoeff[3];
   pSideband->LpfCoef[1].asBitField.YCoef1 = pCfgData->lpfCoeff[4];
   pSideband->LpfCoef[1].asBitField.YCoef2 = pCfgData->lpfCoeff[5];
   pSideband->LpfCoef[2].asBitField.YCoef0 = pCfgData->lpfCoeff[6];
   pSideband->LpfCoef[2].asBitField.YCoef1 = pCfgData->lpfCoeff[7];
   pSideband->LpfCoef[2].asBitField.YCoef2 = pCfgData->lpfCoeff[8];
   pSideband->LpfCoef3.asBitField.YCoef0   = pCfgData->lpfCoeff[9];
   pSideband->LpfCoef3.asBitField.YCoef1   = pCfgData->lpfCoeff[10];

   // Set rounding factor
   pSideband->Instruction.asBitField.LpfRoundFactor = pCfgData->lpf_rounding_factor;
   pSideband->Instruction.asBitField.PpfRoundFactor = pCfgData->ppf_rounding_factor;

   
   // Disable average block! (see email May 15 2014 from Dan L.)
   //h_avg_mode = 0;
  
   // Set horizontal average mode
   scale_factor = hrsz_hal_get_scale_factor(pCfgData->src_h_valid, pCfgData->dst_scale_width);
   if(scale_factor > 0.25) {
      h_avg_mode = 0;
   } else if(scale_factor > 0.125) {
      h_avg_mode = 1;
   } else if(scale_factor > 0.0625) {
      h_avg_mode = 2;
   } else if(scale_factor > 0.03125) {
      h_avg_mode = 3;
   } else if (scale_factor > 0.015625) {
      h_avg_mode = 4;
   } else {
      h_avg_mode = 5;
   }

   pSideband->Instruction.asBitField.HAvgMode = h_avg_mode;
 
   if(pCfgData->en_lpf)
   {
      pSideband->Instruction.asBitField.LpfEn = 1;
   }

   // Configure poly-phase filter
   hrsz_hal_set_ppf((APEX_642CL_HRSZ_CORE_DCR *)pSideband,
                    pCfgData->src_h_valid >> h_avg_mode,
                    pCfgData->dst_scale_width,
                    pCfgData->dst_width,
                    pCfgData->ppf_rounding_factor);

   // Set output clamp
   pSideband->OutClamp.asBitField.Min = pCfgData->dst_clamp_min;
   pSideband->OutClamp.asBitField.Max = pCfgData->dst_clamp_max;

   // Set dst woi
   pSideband->Woi2SrcRes.asBitField.Width  = pCfgData->dst_width;
   pSideband->Woi2SrcRes.asBitField.Height = pCfgData->dst_height;
   pSideband->Woi2Crop0.asBitField.HOffset = pCfgData->dst_h_offset;
   pSideband->Woi2Crop0.asBitField.VOffset = pCfgData->dst_v_offset;
   pSideband->Woi2Crop1.asBitField.HValid  = pCfgData->dst_h_valid;
   pSideband->Woi2Crop1.asBitField.VValid  = pCfgData->dst_v_valid;

   return LIB_SUCCESS;
}
