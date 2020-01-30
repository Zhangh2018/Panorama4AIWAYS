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

#include "swt_report.h"
#include <stdio.h>


#if defined(VALIDATION)

   #define REG32 uint32_t

   #define APEX_642CL_BASE_ADDRESS  (0x00000000)

   #include "apex_642cl_map.h"
   #include "apex_642cl_dmem_fifo_map_type.h"

   #define APEX_DMEM_FIFO_0_PERFORMANCE APEX_642CL_DMEM_FIFO_0_PERFORMANCE
   #define APEX_DMEM_FIFO_1_PERFORMANCE APEX_642CL_DMEM_FIFO_1_PERFORMANCE

#else

   #if defined(VALIDATION3)

      #define REG32 uint32_t

      #define APEX_BASE_ADDRESS  (0x00000000)

      #include "apex_map.h"
      #include "apex_dmem_fifo_map_type.h"

   #endif

#endif

// Gloabal Variables
#ifndef gSWT_RPT_EVENT
//! Report Event Logger
SWT_RPT_EVENT*    gSWT_RPT_EVENT  = 0;
#endif

//! Report Function Database (Level)
int               gSWT_RPT_LEVEL  = 0;

#ifndef gSWT_RPT_DBASE
//! Report Function Database (Function)
#define SWT_RPT_DBASE_TBL_SIZE   63
SWT_RPT_RESULT    gSWT_RPT_DBASE_TBL[SWT_RPT_DBASE_TBL_SIZE]; //SWT_RPT_RESULT*   gSWT_RPT_DBASE_TBL;
SWT_RPT_RESULT*   gSWT_RPT_DBASE  = gSWT_RPT_DBASE_TBL;
int               gSWT_RPT_DBASE_SIZE;
SWT_RPT_RESULT* gSWT_RPT_DBASE_BEG;
SWT_RPT_RESULT* gSWT_RPT_DBASE_END;
#endif

/*!
   Initialize SWT Report.
 */
int
SWT_InitTestReport()
{
   int rval = 0;

   gSWT_RPT_DBASE_SIZE  = 0;
   gSWT_RPT_DBASE_BEG = &gSWT_RPT_DBASE_TBL[0];
   gSWT_RPT_DBASE_END = &gSWT_RPT_DBASE_TBL[SWT_RPT_DBASE_TBL_SIZE];
   gSWT_RPT_DBASE     = gSWT_RPT_DBASE_BEG;
   
   printf("gSWT_RPT_DBASE_SIZE = %6i (%08X)\n", sizeof(gSWT_RPT_DBASE_SIZE), (int)(&gSWT_RPT_DBASE_SIZE));
   printf("gSWT_RPT_DBASE      = %6i (%08X)\n", sizeof(gSWT_RPT_DBASE),      (int)(&gSWT_RPT_DBASE));
   printf("gSWT_RPT_DBASE_TBL  = %6i (%08X)\n", sizeof(gSWT_RPT_DBASE_TBL), (int)(&gSWT_RPT_DBASE_TBL[0]));
   printf("SWT_RPT_RESULT      = %6i\n", sizeof(gSWT_RPT_DBASE_TBL[0]));
   
   return rval;
}

/*! 
   Print SWT Report
 */
int   
SWT_PrintTestReport()
{
   int rval = 0;
   
   //*****************************************************
   //gSWT_RPT_DBASE_TBL  = (SWT_RPT_RESULT*)(0x80000000+0x07C10);
   //gSWT_RPT_DBASE      = (SWT_RPT_RESULT*)(0x80000000+0x07C04);
   //gSWT_RPT_DBASE_SIZE = *((int*)(0x80000000+0x07C00));
   //gSWT_RPT_DBASE_BEG  = (SWT_RPT_RESULT*)(0x80000000+0x07C08);
   //gSWT_RPT_DBASE_END  = (SWT_RPT_RESULT*)(0x80000000+0x07C0C);
   //*****************************************************

   printf("gSWT_RPT_DBASE_SIZE : %6i (%08X)\n", sizeof(gSWT_RPT_DBASE_SIZE), (int)(&gSWT_RPT_DBASE_SIZE));
   printf("gSWT_RPT_DBASE      : %6i (%08X)\n", sizeof(gSWT_RPT_DBASE),      (int)(&gSWT_RPT_DBASE));
   printf("gSWT_RPT_DBASE_TBL  : %6i (%08X)\n", sizeof(gSWT_RPT_DBASE_TBL), (int)(&gSWT_RPT_DBASE_TBL[0]));
   printf("SWT_RPT_RESULT      : %6i\n", sizeof(gSWT_RPT_DBASE_TBL[0]));
   printf("gSWT_RPT_DBASE_SIZE = %6i\n", gSWT_RPT_DBASE_SIZE);
   
#ifndef gSWT_RPT_DBASE   
   // for (int t=0; (&gSWT_RPT_DBASE_TBL[t]) < gSWT_RPT_DBASE; ++t)
   int tbeg = gSWT_RPT_DBASE_SIZE - SWT_RPT_DBASE_TBL_SIZE;
   if( tbeg < 0 )
   {
       tbeg = 0;
   }
   
   for (int t=tbeg; t < gSWT_RPT_DBASE_SIZE; ++t)
   {
       const int ti = t % SWT_RPT_DBASE_TBL_SIZE;
      //
      const char* module   = gSWT_RPT_DBASE_TBL[ti].function->module->module;
      const char* block    = gSWT_RPT_DBASE_TBL[ti].function->module->block;
      const char* file     = gSWT_RPT_DBASE_TBL[ti].function->module->file;
      //
      const char* function = gSWT_RPT_DBASE_TBL[ti].function->function;
      const int   level    = gSWT_RPT_DBASE_TBL[ti].level;
      const int   rval     = gSWT_RPT_DBASE_TBL[ti].rval;
      const int   exec     = gSWT_RPT_DBASE_TBL[ti].exec;
      
      char test_name[1024];
      sprintf(test_name, "%s::%s::%s", module, block, function);
      printf("[%4i][%2i] %-64.64s| %-6.6s| %8i| %6i| %8i| %6i\n", 
         t,
         level,
         test_name,
         ((rval) == 0)?("DONE"):("ERROR"),
         (rval),
         exec,
         0,    // Heap?
         0);   // Stack
      
   }
#endif
   
   return rval;
}
 
/*!
   Return stack information.
 */
int
SWT_LOG_GetStack()
{
   // return OAL_GetStackInformation(0, 0);
   return 0;
}

/*!
   Return heap information.
 */
int
SWT_LOG_GetHeap()
{
   // return OAL_GetUsedHeapSpace();
   return 0;
}

/*!
   Return time information.
 */
int
SWT_LOG_GetTime()
{
#if defined(VALIDATION) || defined(VALIDATION3)
   volatile unsigned int *pCnt = (volatile unsigned int *)(APEX_DMEM_FIFO_0_PERFORMANCE);
   return *pCnt;
#else
   // return RTCReadTicCounter();
   return 0;
#endif
}

/*!
   Rest performance counter.
 */
int
SWT_LOG_ResetCnt(int /*apuId*/)
{
    #if defined(VALIDATION) || defined(VALIDATION3)

       volatile unsigned int *pCnt;

       switch (apuId)
       {
           case 0:
               pCnt = (volatile unsigned int *)(APEX_DMEM_FIFO_0_PERFORMANCE);
               break;
           case 1:
               pCnt = (volatile unsigned int *)(APEX_DMEM_FIFO_1_PERFORMANCE);
               break;
           default:
               //Error, APUID out of range!!!
               return -1;
               break;
       }
       *pCnt = 0;

    #endif
    
    return 0;
}
