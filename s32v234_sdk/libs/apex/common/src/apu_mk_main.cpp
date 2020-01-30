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
// SYSTEM
#include "hw_reg_user.h"
// MICROKERNEL CONFIGURATIONS
#include "apu_microkernel_hal.h"
// MICROKERNEL DECODER LIBRARY
#include "apu_mk_lib.h"
// DMEM FIFO HAL
#include "apu_dmemfifo_hal.h"
// KERNEL LIST
#include "apu_microkernel.h"

//#include "kernel_manager.h"
//extern vec16s __cmem APTR[16];
//extern vec16s __cmem T002_SRC[16];

//
//volatile int   DMEM_USEQ_REG[8192];
//vec16s __cmem  CMEM_RESERVED[2048];

void  apumk_cid_init();
void  apumk_dmem_init();

int   pdd_profile_000();


extern volatile int   DMEM_SHARED[APU_DMEM_COM_SIZE];
extern volatile int   DMEM_CMD_CNT[APU_DMEM_CMD_CNT_SIZE];
extern volatile int   DMEM_CMD_QUE[APU_DMEM_CMD_QUE_SIZE];


extern const int32_t MK_KERNEL_DB[APU_DMEM_KERNEL_DB_ENTRY];


/*
   !!!!!! IMPORTANT !!!!!!!
   Buffer used by DMEM FIFO allocated as part of the shared memory specified in the .bcf
*/
#if (COMPILE_FOR_SIM == 0)
const int32_t buff_addr   = (APU_DMEM_OFFSET_BASE + 0x7000);               // word size aligned!!!!!
                                                      // have to be continuous for now
#else
const int32_t buff_addr   = DMEM_CMD_QUE_OFFSET;               // match with DMEM_CMD_QUE in bcf
#endif
const int32_t start_addr[16] = 
                              {   
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*0 ), (buff_addr + CONFIG_QUE_BUF_SIZ*1),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*2 ), (buff_addr + CONFIG_QUE_BUF_SIZ*3),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*4 ), (buff_addr + CONFIG_QUE_BUF_SIZ*5),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*6 ), (buff_addr + CONFIG_QUE_BUF_SIZ*7),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*8 ), (buff_addr + CONFIG_QUE_BUF_SIZ*9),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*10), (buff_addr + CONFIG_QUE_BUF_SIZ*11),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*12), (buff_addr + CONFIG_QUE_BUF_SIZ*13),
                                 (buff_addr + CONFIG_QUE_BUF_SIZ*14), (buff_addr + CONFIG_QUE_BUF_SIZ*15)
                              };
#if (TEST_OUTBUFF == 1)
const int32_t buff1_addr   = (APU_DMEM_OFFSET_BASE+ 0x7400);               // word size aligned!!!!!
const int32_t out_addr[16] = 
                              {   
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*0 ), (buff1_addr + CONFIG_QUE_BUF_SIZ*1),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*2 ), (buff1_addr + CONFIG_QUE_BUF_SIZ*3),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*4 ), (buff1_addr + CONFIG_QUE_BUF_SIZ*5),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*6 ), (buff1_addr + CONFIG_QUE_BUF_SIZ*7),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*8 ), (buff1_addr + CONFIG_QUE_BUF_SIZ*9),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*10), (buff1_addr + CONFIG_QUE_BUF_SIZ*11),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*12), (buff1_addr + CONFIG_QUE_BUF_SIZ*13),
                                 (buff1_addr + CONFIG_QUE_BUF_SIZ*14), (buff1_addr + CONFIG_QUE_BUF_SIZ*15)
                              };
#endif
// store a pattern at address 0
/*
inline
void
init_vec()
{
   {
      volatile vec16u*  d = (vec16u*)0;
      vec16u   v = 0;
      int16u   s = 0xFF00;
      for (int c=0; c<32; ++c)
      {
         v  = vput(v, s, c);
         s += 0xFF01;
      }
      d[0] = v;
   }
   {
      volatile vec08u*  d = (vec08u*)2;
      vec08u   v = 0;
      int08u   s = 0x00;
      for (int c=0; c<32; ++c)
      {
         v  = vput(v, s, c);
         s += 0x01;
      }
      d[0] = v;
   }
   {
      volatile vec08u*  d = (vec08u*)3;
      vec08u   v = 0;
      int08u   s = 0xFF;
      for (int c=0; c<32; ++c)
      {
         v  = vput(v, s, c);
         s += 0xFF;
      }
      d[0] = v;
   }

}
*/

void apu_sim_host_tx(int message)
{
   ++DMEM_SHARED[0];
   DMEM_SHARED[2] = message;
}

void apu_sim_host_rx()
{
   ++DMEM_SHARED[5];
}

////////////////////////////////////////////////////////////////////////////////////
     
int APU2_MK_MAIN_MEM()
{
   int rval = 0;
   //int mid= 0;
   int msg[4];
   const int mask = 0x1FFFF;     // roughly 2k, check bcf
   
   //while (1)
   {
      //const int m = (mid & 0x03);
      //mid++;

      ////////////////////////////
      // STAGE 1:
      // RX: START OF 1 APU QUEUE
      ////////////////////////////

      //apumk_apu_com_rx(msg[m]);
      
      ///////////////////////////////////////////////
      // STAGE 2:
      // RX: START OF 1 TILE PROCESS (REMOVE LATER???)
      // TX: DONE OF 1 TILE PROCESS
      ////////////////////////////////////////////////
      rval = DMEM_CMD_CNT[0];
      rval = apumk_apu_decode((const int*)(&DMEM_CMD_QUE[0]), 0, mask, (int*)(&DMEM_CMD_CNT[0]));
   }
   return rval;
}

////////////////////////////////////////////////////////////////////////////////////
int APU2_MK_MAIN_DMEMFIFO(int total_buf)
{
   int rval = 0;
   
   //int msg[4];
   // configure DMEM FIFO, the first half used for debugging
   const int num_buf             = CONFIG_QUE_BUF_NUM;         // 0:1 buffer, 1: 2 buffers
   const int num_buff_read       = 1;
   int8_t enable_update          = 1;
   int32_t buf_size              = CONFIG_QUE_BUF_SIZ;         // 3: 4bytes, 4: 5 bytes (driver takes care of the -1)
   int wsize                     = DMEM_FIFO_W64;              // 2:32bits, 3:64bits
   int read_index_32             = 0;
   
   // read
   //int mid= 0;
   int32_t in_status             = 0;
   int8_t queue_done             = 0;
   
   // CONFIG IN
   apu_dmemfifo_hal_configure( buf_size, enable_update, wsize, num_buf, DMEM_FIFO_IN, 0);
   


   // PUSH INITIAL START ADDR
   int local_tb   = total_buf;
   int buf_index  = 0;
   int buf_masked[2] = {(buf_index & CONFIG_QUE_BUF_MSK), ((buf_index+1) & CONFIG_QUE_BUF_MSK)};
   
   // NOTE:
   // HAVE TO START >=2 BUFFERS
   int buf_config_start = CONFIG_QUE_BUF_NUM;   
   // NOTE:
   // RIGHT AFTER ADDR UPDATE, BEFORE DMA STARTS, FIFO STATUS: 1 => 0
   // AS SOON AS BUFFER IS FILLER, STATUS: 0 => 1
   // PUSH START ALL AVAILABLE BUFFERS
   apu_dmemfifo_hal_buff_start(buf_masked[0], buf_config_start, start_addr, DMEM_FIFO_IN, 0);
   // SEQUENTIAL PROCESSING, PUSH IN ADDR => STATUS => READ IN BUFFER => WRITE TO OUT BUBBFER => PUSH OUT ADDR
   
   //int* buff_rptr  = start_addr[buf_masked[0]];
   while (queue_done == 0)
   {
      //const int m = (mid & 0x03);
      //mid++;

      // ASSUME THERE IS AT LEAST ONE BUFFER
      // READ TOTAL COMMAND SIZE FROM BUFFER_0[0]
      // READ 2 BUFFERS AT A TIME
      if (buf_index < (total_buf))
      {
         apu_dmemfifo_hal_status(&in_status, DMEM_FIFO_IN, 0);
         

         
         // IF STATUS IS HIGH: PROCESS THAT BUFFER
         // NOTE: if push 4 buffers at the beginning, current buffer index will be 3
         //int current_inbuff = (in_status>>16) & 0x0F;
#if (COMPILE_FOR_SIM == 0) 
         int status_mask   = (1<<buf_masked[0]) | (1<<buf_masked[1]);
         while ((in_status & status_mask) != status_mask) {apu_dmemfifo_hal_status(&in_status, DMEM_FIFO_IN, 0);};
#endif

         DMEM_SHARED[7] = 0x12345678;
         ///////////////////////////////////////////////
         // RX: START OF 1 TILE PROCESS (REMOVE LATER???)
         // TX: DONE OF 1 TILE PROCESS
         ////////////////////////////////////////////////
         // FOR NOW
         // IN WORDS
         DMEM_CMD_CNT[0] = (CONFIG_QUE_BUF_SIZ32 - (read_index_32%CONFIG_QUE_BUF_SIZ32));
         DMEM_CMD_CNT[1] = 0;
         //apumk_apu_com_rx(msg[m]);
         // DECODE ONE TILE
         apumk_apu_decode((const int*)(start_addr[0]), read_index_32, QUE_ADDR_MASK_32, (int*)(&DMEM_CMD_CNT[0]));
         // UPDATE READ PTR
         read_index_32 += DMEM_CMD_CNT[1];
         //DMEM_SHARED[7] = read_index_32;
         //apumk_apu_com_tx(buf_masked[0]);
      }
      else if (buf_index == local_tb)
      {
         rval        = buf_index;
         queue_done  = 1;
      }
      // PUSH BACK ONE BUFFER
#if (COMPILE_FOR_SIM == 0) 
      if ((buf_index+buf_config_start) < total_buf)
      {
         apu_dmemfifo_hal_buff_start(buf_masked[0], num_buff_read, start_addr, DMEM_FIFO_IN, 0);

      }
#endif
      // UPDATE BUFFER INDEX
      buf_index++;
      buf_masked[0] = ( buf_index   & CONFIG_QUE_BUF_MSK);
      buf_masked[1] = ((buf_index+1)& CONFIG_QUE_BUF_MSK);

   }
   
   return rval;
}

////////////////////////////////////////////////////////////////////////////////////
int APU2_MK_TEST_DMEMFIFO()
{
   int rval = 0;
   int msg = 0;

   // configure DMEM FIFO, the first half used for debugging
   const int total_buf           = 16;
   const int num_buf             = CONFIG_QUE_BUF_NUM;         // 0:1 buffer, 1: 2 buffers
   const int num_buff_read       = 1;
   int8_t enable_update          = 1;
   int32_t buf_size              = CONFIG_QUE_BUF_SIZ;         // 3: 4bytes, 4: 5 bytes (driver takes care of the -1)
   int wsize                     = DMEM_FIFO_W64;              // 2:32bits, 3:64bits
   
   // read
   int mid= 0;
   static int32_t buf_idx[num_buf+1]   = {0};
   int32_t num_buff_full               = 0;
   int32_t in_status                   = 0;
   int32_t out_status                  = 0;
          
   // WAIT FOR TX TO CONFIG
   apu_dmemfifo_hal_status(&in_status, DMEM_FIFO_IN, 0);
   apumk_apu_com_tx(in_status);
   apumk_apu_com_rx(msg);
   
   // CONFIG IN & OUT
   apu_dmemfifo_hal_configure( buf_size, enable_update, wsize, num_buf, DMEM_FIFO_IN, 0);
   
#if (TEST_OUTBUFF == 1)
   apu_dmemfifo_hal_configure( buf_size, enable_update, wsize, num_buf, DMEM_FIFO_OUT, 0);
#endif             

   // WAIT FOR TX TO START UPDATING ADDR
   apu_dmemfifo_hal_status( &in_status, DMEM_FIFO_IN, 0);
   apumk_apu_com_tx(in_status);
   apu_dmemfifo_hal_status( &out_status, DMEM_FIFO_OUT, 0);
   apumk_apu_com_tx(out_status);
   apumk_apu_com_rx(msg);

   // PUSH INITIAL START ADDR
   int buf_index  = 0;

   // NOTE:
   // RIGHT AFTER ADDR UPDATE, BEFORE DMA STARTS, FIFO STATUS: 1 => 0
   // AS SOON AS BUFFER IS FILLER, STATUS: 0 => 1
   
   
   // SEQUENTIAL PROCESSING, PUSH IN ADDR => STATUS => READ IN BUFFER => WRITE TO OUT BUBBFER => PUSH OUT ADDR
   while (1)
   {
      //const int m = (mid & 0x03);
      //mid++;
   
      // AFTER PUSH WAIT FOR DMA
      /*
      apu_dmemfifo_hal_status(buf_idx, &status, DMEM_FIFO_IN);
      apumk_apu_com_tx(status);
      apumk_apu_com_rx(msg[m]);  // wait for continue
      */
      
      int buf_masked = (buf_index&0x0F);
      
      if (buf_index < total_buf)
      {
         apu_dmemfifo_hal_buff_start(buf_masked, num_buff_read, start_addr, DMEM_FIFO_IN, 0);

      }
      
      // READ IN BUFFER STATUS
      apu_dmemfifo_hal_status(&in_status, DMEM_FIFO_IN, 0);
      
      // IF STATUS IS HIGH: PROCESS THAT BUFFER
      int current_inbuff = (in_status>>16) & 0x0F;
      while (((in_status >> current_inbuff)&0x01) != 1) {apu_dmemfifo_hal_status(&in_status, DMEM_FIFO_IN, 0);};
      
#if (TEST_OUTBUFF == 1)
      // COPY
      {
         /*
         int8_t* src_buff = (int8_t*)start_addr[buf_masked];
         int8_t* dst_buff = (int8_t*)out_addr[buf_masked];
         for (int k=0; k<buf_size; k++)
         {
            uint8_t data = src_buff[k];
            dst_buff[k]  = data;
         }
         */
         // CHECK IF THIS OUT BUFFER IS DRAINED BEFORE USE IT
         apu_dmemfifo_hal_status(&out_status, DMEM_FIFO_OUT, 0);
         while (((out_status >> buf_masked)&0x01) != 0) {apu_dmemfifo_hal_status(&out_status, DMEM_FIFO_OUT, 0);};
         // BUFFER I WANT TO USE IS DRAINED PREVIOUSLY, SAFE TO PUSH ADDR
         {
            apu_dmemfifo_hal_buff_start(buf_masked, num_buff_read, start_addr, DMEM_FIFO_OUT, 0);  
         }
         
      }
#endif
      
      // UPDATE BUFFER INDEX
      buf_index++;
   }
   
   return rval;
}

////////////////////////////////////////////////////////////////////////////////////
// ENABLE_LIB: TESTING PURPOSE ONLY (SIM)
// 0: SEND COMMAND AND COMMAND QUEUE
// 1: ENABLE COMMAND QUEUE (DMEM FIFO)
// 2: MK DMEM FIFO TEST
#define ENABLE_LIB                     0


int apu_mk_main()
{
   int rval    = 0;
   int apu_id  = 0;
   int msg[4];

   // INITIALIZE HW REGISTER BASE ADDR
   InitVirtualHwPtrArray();

   apumk_cid_init();
   apumk_hal_dmem_init(0, apu_id, APU_TO_APU_MEM);
   
   // SIMULATE COMPILE AND PUSH COMMAND LIST
#if ((COMPILE_FOR_SIM > 0) && (ENABLE_LIB < 2))

   static int func_addr[REGISTERED_FUNC]  = {0};
   const int TEST_KN                      = 3;
   for (int i=0; i<TEST_KN; i++)
   {
      func_addr[i]  = MK_KERNEL_DB[i];
   }
   

   
   const int cmd_list_size             = 32;
   int cmd_list[cmd_list_size]         = {0};
   const int np_list[TEST_KN]          = {5, 6, 3};
   rval = apu_sim_host_gen_cmd(cmd_list, (const int*)func_addr, (const int*)np_list, TEST_KN, nTiles);
   
   apumk_hal_send_list(cmd_list);
   
   // simulate host send message, one way comm
   #if (ENABLE_LIB == 0)
   apu_sim_host_tx(0x10000001);
   #else
   apu_sim_host_tx(0x20000004);
   #endif

#else
   // COMMUNICATE BACK TO INDICATE INIT IS DONE
   apumk_apu_com_tx(0xF0010000);
#endif

   //////////////////////////////////////
   // MK Main:
   // ASSUME COMMAND QUEUE STARTS ALREADY
   // MK DMEM FIFO TEST: 
   //////////////////////////////////////
   
#if (ENABLE_LIB < 2)
      // WAIT FOR TX TO START UPDATING ADDR
      /*
      apumk_apu_com_rx(msg[0]);
      uint32_t msg_type   = (msg[0] >> 31) & 0x01;
      int msg_val    = (msg[0]      ) & 0x7FFFFFFF;
      if (msg_type == 0)
      {
         // send 1/N command
         APU2_MK_MAIN_MEM();
      }
      else if (msg_type == 1)
      {
         // push 1/N command and DMA
         APU2_MK_MAIN_DMEMFIFO(msg_val);
      }
      */

   while (1)
   {
      apumk_apu_com_rx(msg[0]);
      uint32_t msg_type = RET_HDR(msg[0]);
      int msg_val       = RET_VAL(msg[0]);

      if (msg_type == HDR_HOST_WRITE)
      {
         // send 1/N command
         rval  = APU2_MK_MAIN_MEM();
      }
      else if (msg_type == HDR_HOST_DMA)
      {
         // push 1/N command and DMA
         // rval  = APU2_MK_MAIN_DMEMFIFO(msg_val);

      }
      apumk_apu_com_tx(APU_MSG_DONE(rval));

   }
#elif (ENABLE_LIB == 2)
   APU2_MK_TEST_DMEMFIFO();
#endif


   return rval;
}
