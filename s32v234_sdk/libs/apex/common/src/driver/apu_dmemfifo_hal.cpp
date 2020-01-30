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

// INCLUDE

#include <apu_dmemfifo_hal.h>
#include <hw_reg_user.h>

typedef unsigned int REG32;
#include "apex_642cl_dmem_fifo_map.h"
#include "apu_microkernel_hal.h"
 
/*
#define nbuf_bits       4
#define nbuf_mask       ((1<<nbuf_bits)  - 1)
#define nbuf_shif       (0)

#define wsiz_bits       3
#define wsiz_mask       ((1<<wsiz_bits)  - 1)
#define wsiz_shif       (nbuf_shif+nbuf_bits)

#define enar_bits       1
#define enar_mask       ((1<<enar_bits)  - 1)
#define enar_shif       (wsiz_shif+wsiz_bits)

#define bsiz_bits       16
#define bsiz_mask       ((1<<bsiz_bits)  - 1)
#define bsiz_shif       (enar_shif+enar_bits)
*/
#define config_fields       4

int apu_dmemfifo_hal_configure(  int32_t buff_size,   int32_t enable_addr_update,
                                 int32_t word_size,   int32_t num_buffers,
                                 const int32_t fifo_mode, int fifo_channel)
{
   int rval = 0;
   
   
   // NOTE: 32-bit WRITE
   /*
   DMEMFIFO_CONFIG  dmem_config_reg;
   
   dmem_config_reg.asBitfield.num_buffer      = num_buffers;
   dmem_config_reg.asBitfield.word_size       = word_size;
   dmem_config_reg.asBitfield.ena_addr_update = enable_addr_update;
   dmem_config_reg.asBitfield.buffer_size     = buff_size;
   
   uint32_t num_buffer        :  4;    // 0
   uint32_t word_size         :  3;    // 6-4
   uint32_t ena_addr_update   :  1;    // 7
   uint32_t buffer_size       :  16;    // 8-23
   uint32_t reserved          :  8;    // 31-24
   */

   
   int32_t dmem_config                 = 0;
         int fields[config_fields][3]  = {   {4},           {3},        {1},                 {16}     };
   const int config[config_fields]     = {   (num_buffers-1),   word_size,  enable_addr_update,  (buff_size-1)};
   
   fields[0][1]   = (1<<fields[0][0]) - 1;
   fields[0][2]   = (0);
   dmem_config    |= ((config[0]&fields[0][1]) << fields[0][2]);
   for (int i=1; i<config_fields; i++)
   {
      fields[i][1]    = (1<<fields[i][0]) - 1;
      fields[i][2]    = (fields[i-1][0] + fields[i-1][2]);
      dmem_config    |= ((config[i]&fields[i][1]) << fields[i][2]);
   }
   
   
   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);
   if (fifo_mode == 0)
      DMEM_FIFO_REG->ConfigIn.as32BitValue = dmem_config;
   else
      DMEM_FIFO_REG->ConfigOut.as32BitValue = dmem_config;

   return 0;
}

// Can't exceed the number of buffers configured
/*
rev 17 & before
//CMEM        ? 0x00000000
//APU0 DMEM   ? 0x00400000
//APU0 PMEM   ? 0x0042
//APU1 DMEM   ? 0x0044
//APU0 PMEM   ? 0x0046
rev 19
//APU0 DMEM   ? 0x00000000
//APU0 PMEM   ? 0x00008000
//APU1 DMEM   ? 0x00010000
//APU0 PMEM   ? 0x00018000
uint32_t start_addr     :  15;   // 0-14
uint32_t reserved       :  2;   // 16-15
uint32_t peripheral_addr        :  15;   // 31-17
*/
#define buff_fields  3
int apu_dmemfifo_hal_buff_start(int start_buff_index, int num_buffers, const int32_t* buff_addr, const int fifo_mode, int fifo_channel)
{
   int rval = 0;

   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);

   int32_t p_addr = 0;
   for (int i = start_buff_index; i < (start_buff_index+num_buffers); i++)
   {
      p_addr   = buff_addr[i];
      if (fifo_mode == DMEM_FIFO_IN)
      {
         DMEM_FIFO_REG->InBuffer[i].as32BitValue = p_addr;
      }
      else
      {
         DMEM_FIFO_REG->OutBuffer[i].as32BitValue = p_addr;
      }
   }

   return 0;
}

int apu_dmemfifo_hal_config(  int32_t buffer_size,   int enable_addr_update,
                              int word_size,          int num_buffers,        const int32_t* buff_addr,
                              const int fifo_mode, int fifo_channel)
{
   int rval = 0;

   apu_dmemfifo_hal_configure(buffer_size, enable_addr_update, word_size, num_buffers, fifo_mode, fifo_channel);
   if (enable_addr_update == 1) apu_dmemfifo_hal_buff_start(0, (num_buffers+1), buff_addr, fifo_mode, fifo_channel);

   return 0;
}

int apu_dmemfifo_hal_clr(int fifo_mode, int fifo_channel)
{
   int rval = 0;

   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);

   if (fifo_mode == DMEM_FIFO_IN)
   {
      DMEM_FIFO_REG->Maintenance.as32BitValue  = 0x01;
   }
   else if (fifo_mode == DMEM_FIFO_OUT)
   {
      DMEM_FIFO_REG->Maintenance.as32BitValue  = 0x02;
   }
   return 0;
}

int apu_dmemfifo_hal_status( int32_t* fifo_status, int fifo_mode, int fifo_channel)
{
   int rval = 0;

   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);

   if (fifo_mode == DMEM_FIFO_IN)
      fifo_status[0] = DMEM_FIFO_REG->InStatus.as32BitValue;
   else
      fifo_status[0] = DMEM_FIFO_REG->OutStatus.as32BitValue;


   return 0;
}

void apu_dmemfifo_hal_pull_block(int fifo_mode, int fifo_channel, int buff_index)
{
   int32_t fifo_status;
   apu_dmemfifo_hal_status(&fifo_status, fifo_mode, fifo_channel);
   if (fifo_mode == DMEM_FIFO_IN)
   {
      while (!(fifo_status & (1 << buff_index)))
      {
         apu_dmemfifo_hal_status(&fifo_status, fifo_mode, fifo_channel);
      }
   }
   else
   {
         while ((fifo_status & (1 << buff_index)))
         {
            apu_dmemfifo_hal_status(&fifo_status, fifo_mode, fifo_channel);
         }
   }
}

int apu_dmemfifo_hal_reset(int* list, int res_set, int processor_id, int fifo_channel)
{
   int rval = 0;

   return 0;
}

void apu_dmemfifo_hal_reset_perf_counter(int fifo_channel)
{
   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);

   DMEM_FIFO_REG->Performance.asBitField.PerfCnt = 0;
}

int apu_dmemfifo_hal_read_perf_counter(int fifo_channel)
{
   HW_REG_ID lHwRegId;
   if (fifo_channel == 0)
      lHwRegId = HW_REG_DMEM_FIFO_0;
   else
      lHwRegId = HW_REG_DMEM_FIFO_1;
   volatile APEX_642CL_DMEM_FIFO_DCR*  const DMEM_FIFO_REG = (APEX_642CL_DMEM_FIFO_DCR*)ReturnVirtualHwPtr(0, lHwRegId);

   return DMEM_FIFO_REG->Performance.asBitField.PerfCnt;
}
