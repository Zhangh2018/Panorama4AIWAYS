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

//#include "apu_sample_kernel.h"
#include "apu_mk_lib.h"
#include "apu_microkernel.h"
// System Driver Directory
#include "apu_microkernel_hal.h"
#include <seq_hal.h>

#define APU_COM_WAIT_CC 10
#include <time_helper.h>

volatile int32_t DMEM_KERNEL_NUM[1] = {REGISTERED_FUNC};
#if  (USE_FUNC_PTR > 0)
/*
myFunct arrayKernel[REGISTERED_FUNC] =
{
   &sample_kernel0,
   &sample_kernel1,
   &acf1_testkernel_add,
};
*/

extern volatile KERNEL_INFO DMEM_KERNEL_DB[APU_DMEM_KERNEL_DB_ENTRY];
extern const int32_t MK_KERNEL_DB[APU_DMEM_KERNEL_DB_ENTRY];

/*
int arrayKernel[REGISTERED_FUNC][] =
{
   &sample_kernel0,
   &sample_kernel1,
   &acf1_testkernel_add,
};
*/
#else
/*
int arrayKernel[REGISTERED_FUNC] =
{
   0,
   1,
   2,
};
*/

volatile KERNEL_INFO DMEM_KERNEL_DB[APU_DMEM_KERNEL_DB_ENTRY] =
{
   {(int32_t)(0), "sample_kernel0"     },
   {(int32_t)(1), "sample_kernel1"     },
   {(int32_t)(2), "acf1_testkernel_add"},
   {0xFFFFFFFF,   0                    }
};

const int32_t MK_KERNEL_DB[APU_DMEM_KERNEL_DB_ENTRY] =
{
   (int32_t)(0),
   (int32_t)(1),
   (int32_t)(2)
};

#endif




//volatile int            DMEM_DEBUG[APU_DMEM_DEBUG_SIZE];
//volatile int   DMEM_MK_MEM[APU_DMEM_SHARED_TOTAL];
volatile int   DMEM_SHARED[APU_DMEM_COM_SIZE];
volatile int   DMEM_CMD_CNT[APU_DMEM_CMD_CNT_SIZE];
volatile int   DMEM_CMD_QUE[APU_DMEM_CMD_QUE_SIZE];
/*
volatile int            DMEM_CMD_CNT[APU_DMEM_CMD_CNT_SIZE];
volatile int            DMEM_CMD_QUE[APU_DMEM_CMD_QUE_SIZE];

*/
//volatile SEQ_REGISTERS* const DMEM_USEQ_REG = (SEQ_REGISTERS*)APU_DMEM_SEQ_REG_BASE;



//extern vec16s CID;
vec16s         CID;

void
apumk_cid_init()
{
   vec16s cid = 0;
   
   for (int c=0; c<32; ++c)
   {
      cid = vput(cid, c, c);
   }
   
   CID = cid;
}



/***********************************************************************
// COM IMPLEMENTATION
***********************************************************************/

//
void
apumk_apu_com_wait(int c, int rta)
{
   static int wait_ctr[4] = { 0, 0, 0, 0 };
   static int l = 0;
   _wait_cc(c);
   // for (; c>0; --c) { ++l;}

   ++wait_ctr[rta];
}

// APU
int
apumk_apu_com_rx(int& msg)
{
   int rval = 0;
   
   // WAIT FOR A MESSAGE
   while (!(DMEM_SHARED[0] > DMEM_SHARED[1]))
   {
      // apumk_apu_com_wait(100, 0);
      apumk_apu_com_wait(APU_COM_WAIT_CC, 0);
   }
      
   msg = DMEM_SHARED[2];
   // SEND ACKNOWLEDGE
   ++DMEM_SHARED[1];
   
   return rval;
}

int
apumk_apu_com_tx(int msg)
{
   DMEM_SHARED[6] = msg;
   // SEND A MESSAGE
   ++DMEM_SHARED[4];
   // WAIT ACKNOWLEDGE
   while (DMEM_SHARED[4]  > DMEM_SHARED[5])
   {
      apumk_apu_com_wait(APU_COM_WAIT_CC, 3);
   }
   
   return 0;
}



/*

//
vec16s TEST_DST[16];
vec08u TEST_SRC[64];

int
test_sample_func0()
{
   int rval = 0;

   //sample_func0(TEST_DST, 4, TEST_SRC + 2 + 2*8, 8, 4, 4);
   
   return rval;
}
//
int pdd_apu_push_cmd_0( int x, int num_params, int cmd, int* param_list)
{
   int rval = 0;

   return rval;

}


//

int
pdd_sim_host_push_cmd_0()
{
   int (*func_addr)() = &test_sample_func0;
      
   int q_idx            = 0;
   int cmd_word         = 0;
   int apu_cmd_word     = 0;
  
   
   int rval = 0;

   rval += func_addr();
   
   static int param_list[8] = {0};
   for (int i=0; i< (1<<3); i++)
   {
      // push apu kernel with max 8 parameters
      cmd_word += pdd_apu_push_cmd_0(0, i,(int)func_addr, param_list);   
   }
   return rval;
}
*/


/***********************************************************************
// APU MICROKERNEL DECODE COMMAND QUEUE
***********************************************************************/
#define CIRCULAR_BUFFER_RET(base, readIdx, mask) (base[(readIdx) & mask])
#if 0 // func pointer test case
int
apu_com_decode()
{
   int array_idx           = 0;
   int num_param           = 0;
   int32_t func_val        = 0x7f000000;     // !!!! init to something > 0x8000
   static int argv[32]     = {0};
   
   bool found              = false;
   

#if 0
   for (int i=0; i<REGISTER_FUNC; i++)
   {
      //if(func_val == (int)arrayKernel[i]) 
      if(func_val == MK_KERNEL_DB[array_idx])
      {
         array_idx = i;
         found = true;
      } 
   }
#else
   do
   {
      if(func_val == MK_KERNEL_DB[array_idx])  found = true;
      else array_idx++;
   } while ((array_idx<REGISTER_FUNC)&&(!found));

#endif

   int (*func)(int, char**)  = (myFunct)MK_KERNEL_DB[array_idx];
   func(num_param, argv);
   return 0;
}


#else // real implementation of decode
// DECODE APU KERNEL COMMAND
int
apu_kernel_decode(const int* cmd_queue, int idx, int msk, int* total_cw, int* debug_info)
{
   int rval       = 0;

   /*
    * As the sole user of the MK, ACF currently invokes at most one command at a time,
    * so the need to copy the parameter array from the global cmd_queue into the local static 'param_array'
    * is not required at this time (it's a waste of time and memory). In the future the 'queue' aspect
    * will likely be revisited.
    */

   const int cmd_id                    = idx;
   int w0                              = CIRCULAR_BUFFER_RET(cmd_queue, cmd_id, msk);
   int num_param                       = ((w0 >> CMD_TYPE_BITS)   & NUM_PARAM_MASK);
   int num_cmd                         = SIZ_CMD_ARRAY(num_param);
   //static int param_array[MAX_NUM_PARAM + 1]  = {0}; //(disabled - use cmd_queue directly)
   int func_val                        = ((w0 >> 16)              & 0xFFFF);
   int array_idx                       = 0;
   int found                           = 0;
   
   // query function pointer list
   do
   {
      if(func_val == MK_KERNEL_DB[array_idx])  found = 1;
      else array_idx++;
   } while ((array_idx<REGISTERED_FUNC)&&(found==0));


   
   if (found == 1)
   {
#if (FLAG_PARAM_32BIT   == 1)
#if 0 //disabling parameter copy (use cmd_queue directly)
      for (int i=1, j=0; i<(num_cmd); i++, j++)
      {
         int cw      = CIRCULAR_BUFFER_RET(cmd_queue, cmd_id+i, msk);
         param_array[j]     = (cw & 0xFFFFFFFF);
      }
#endif
#else
      for (int i=1, j=0; i<(num_cmd); i++, j+=2)
      {
         int cw      = CIRCULAR_BUFFER_RET(cmd_queue, cmd_id+i, msk);
         argv[j]     = ((cw >> 16)              & 0xFFFF);
         argv[j+1]   = ((cw >>  0)              & 0xFFFF);
      }
#endif
#if (USE_FUNC_PTR > 0)

      //DMEM_SHARED[3] = (MK_KERNEL_DB[array_idx]);
      // abosolute mode
      //myFunct func  =  (myFunct)(MK_KERNEL_DB[array_idx] + PMEM_APU_START_ADDR);
      // relative mode
      myFunct func  =  (myFunct)(MK_KERNEL_DB[array_idx] + 0);
      //DMEM_SHARED[7] = (volatile int)func;
      rval = func((int* const)(cmd_queue+1), num_param); //(use cmd_queue directly)
      //DMEM_SHARED[7] = 0x23589674;

#else
      // sample_kernel0(num_param, argv);
      switch (array_idx)
      {
      case 0:
         sample_kernel0(num_param, (const char**)argv); 
         break;
      case 1:
         sample_kernel1(num_param, (const char**)argv);    
         break;
      case 2:
         acf1_testkernel_add(num_param, (const char**)argv);
      default:
         break;
      }
#endif
      debug_info[0]   = (int)func_val;

      // debug
      //sample_func_set((vec08u __cmem*)320,  8, 22, 8, 4);
      //sample_func_dmem_set((unsigned char*)DMEM_DEBUG, 8, 22, 8, 4);
   }
   else rval   = -1;
   
   //rval  = num_cmd;
   total_cw[0] = num_cmd;
   return rval;
}

// DECODE SEQ
int
apu_useq_decode(const int* cmd_queue, int idx, int msk, int seq_cmd_type, int* debug_info)
{
   
   int rval = 0;
   
   int proc_id             = ((CIRCULAR_BUFFER_RET(cmd_queue, idx, msk) >> SET_INDEX_START)   & 0xFF);
   const int seq_res_set   = CIRCULAR_BUFFER_RET(cmd_queue, idx+1, msk);

   if (seq_cmd_type == SEQ_CMD_DEC)
   {
      // SET UP DEC
      // NO SUPPORT FOR MULTIPLE IRQs CURRENTLY
#if (COMPILE_FOR_SIM==0)
      if (0 != seq_hal_query_interrupt(0, proc_id))
      {
         // GENERATE A WARNING: SHOULD NEVER HAPPEN!
         seq_hal_clear_interrupt(0, proc_id);
      }
#endif
      debug_info[0]                 = seq_res_set;
      
      // NOW WAIT FOR RSC TO BE READY
#if (COMPILE_FOR_SIM==0)   
      if (seq_res_set != 0)
      {
         int i                      = 0;
         int n                      = 0;
         int wait_ready             = 0;
         static int res_array[32]   = {0};
         
         // search for resource id
         int local_set              = seq_res_set;
         for (int i=0; i<32; i++)
         {
            int res_id = (local_set & 0x01);
            if (res_id != 0)
            {
               res_array[n]   = i;
               n++;
            }
            local_set >>= 1;
         }
         
         while (wait_ready == 0)
         {
            wait_ready = 0;
            int counter = 0;
            for (int i=0; i<n; i++)
            {
               //if (DMEM_USEQ_REG.rsc_ctr[res_array[i]] > 0) counter++;  
               if (seq_hal_ret_rsc_counter(0, res_array[i]) > 0) counter++;
            }
            
            if (counter == n) wait_ready =1;
            
         };
         
         seq_hal_proc_dec_rscmask(0, proc_id, 1<<seq_res_set);
         
#if 1
         // 595 40830000  lw r4,0(r3)
         // 596 00842805  and r4,r4,r5
         // 597 00852018  sne r4,r5,r4
         // 598 1004fffb  bnez r4,#-5
         do {} while(!seq_hal_query_interrupt(0, proc_id));
#else
         DMEM_SHARED[3] = 0;
         int isr;
         do {
            isr = (DMEM_USEQ_REG.isr);
            //DMEM_SHARED[3] = isr;
         } 
         while((1<<proc_id) != (isr & (1<<proc_id)));
#endif      
         seq_hal_clear_interrupt(0, proc_id);

      }
      

#endif

   }
   else if (seq_cmd_type == SEQ_CMD_INC)
   {
      seq_hal_proc_inc_rscmask(0, proc_id, 1<<seq_res_set);
   }
   else rval = -1;

   return rval;
}

// DECODE MAIN
int
apumk_apu_decode(const int* APU_CMD_QUE, const int read_index, const int buff_size_mask, int* QUEUE_COUNT)
{
   int rval       = 0;
   int cmd_id     = read_index;
   int msk        = buff_size_mask;
   int debug_info = 0;
   int useq_info  = 0;
   const int* pCMD_QUEUE   = APU_CMD_QUE;
         int* pQUE_COUNT   = QUEUE_COUNT;
         
   while ((pQUE_COUNT[0]>pQUE_COUNT[1]) && (rval == 0))
   {
      //int cmd_type   = (pCMD_QUEUE[cmd_id] & CMD_TYPE_MASK);
      int cmd_type   = (CIRCULAR_BUFFER_RET(pCMD_QUEUE, cmd_id, msk) & CMD_TYPE_MASK);
      int host_info  = 0;
      //int cmd_left   = pQUE_COUNT[0] - pQUE_COUNT[1];
      
      switch (cmd_type)
      {
         case APU_COMMAND:
         {
             
               int num_param     = ((CIRCULAR_BUFFER_RET(pCMD_QUEUE, cmd_id, msk) >> CMD_TYPE_BITS)   & NUM_PARAM_MASK);
               int num_cmd       = 1 + ((num_param+1)>>1);
               //if (cmd_left >= num_cmd)
               {
                  rval = apu_kernel_decode(pCMD_QUEUE, cmd_id, msk, &num_cmd, &debug_info);
   
                  pQUE_COUNT[1]   += num_cmd;
                  cmd_id          += num_cmd;
               }


         }
         break;
         
         case SEQ_COMMAND:
         {

            //if (cmd_left >= SEQ_CMD_WORDS)
            {
               int seq_cmd_type  = 0;
               seq_cmd_type      = ((CIRCULAR_BUFFER_RET(pCMD_QUEUE, cmd_id, msk) >> CMD_TYPE_BITS) & SEQ_CMD_TYPE_MASK);
               
#if (COMPILE_FOR_SIM == 0) 
               // only enabled on platform for the two way communication  
               if (seq_cmd_type == SEQ_CMD_DEC)
               {
                  //apumk_apu_com_rx(host_info);
               }
               else if (seq_cmd_type == SEQ_CMD_INC) 
               {
                  //apumk_apu_com_tx(debug_info);
               }
#endif             
               rval = apu_useq_decode(pCMD_QUEUE, cmd_id, msk, seq_cmd_type, &debug_info);
               
               pQUE_COUNT[1]   += SEQ_CMD_WORDS;
               cmd_id          += SEQ_CMD_WORDS;
            }
            //else rval = 0x01;
         }
         break;
         case SIZ_COMMAND:
         {
               int num_cmd       = 1;

               pQUE_COUNT[1]   += num_cmd;
               cmd_id          += num_cmd;
         }
         break;
         
         case NUL_COMMAND:
         {
               pQUE_COUNT[1]   += 1;
               cmd_id          += 1;
         }
         break;
      }

   }

   return rval;
}


// DECODE MAIN
int
apumk_apu_decode_dmemfifo()
{
   int rval = 0;
   return rval;
}
#endif




#if 0
//
volatile int   DMEM_SHARED[1024];

#define PDD_COM_WAIT_CC 100

int
apu_com_rx(int& msg)
{
   // WAIT FOR A MESSAGE
   while (!(DMEM_SHARED[0] > DMEM_SHARED[1]))
   {
      apumk_apu_com_wait(100, 0);
   }
      
   msg = DMEM_SHARED[2];
   // SEND ACKNOWLEDGE
   ++DMEM_SHARED[1];
   
   return 0;
}

int
apu_com_tx(int msg)
{
   DMEM_SHARED[6] = msg;
   // SEND A MESSAGE
   ++DMEM_SHARED[4];
   // WAIT ACKNOWLEDGE
   while (DMEM_SHARED[4]  > DMEM_SHARED[5])
      apumk_apu_com_wait(100, 3);

   return 0;
}

void
apu_com_wait(int c, int rta)
{
   static int wait_ctr[4] = { 0, 0, 0, 0 };
   
   wait(APU_COM_WAIT_CC);
   
   ++wait_ctr[rta];
}
#endif

