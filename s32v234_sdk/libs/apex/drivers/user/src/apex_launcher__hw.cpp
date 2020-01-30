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

#include <string>
#include <stdio.h>
#include <apex_launcher.hpp>
#include <oal.h>
#include <apu_hal.h>
#include <seq_hal.h>
#include <cmem_if_hal.h>
#include <global_hal.h>
#include <kernel_manager.h>
#include <apu_microkernel_hal.h>
#include <apex_interrupt.h>
#include <time_helper.h>

CMEM_IF_APU_CFG TranslateToCmemIfApuCfg(int32_t lApuId, int32_t lCuArraySize)
{
   CMEM_IF_APU_CFG lCmemIfApuCfg = CFG_UNKNOWN_CONFIGURATION;
   if ((32 == lCuArraySize && 0 == lApuId) || (32 == lCuArraySize && 1 == lApuId))
      lCmemIfApuCfg = CFG_642__APU_0_CU_0_31_SMEM_0_1__APU_1_CU_32_63_SMEM_2_3;
   else if (64 == lCuArraySize && 0 == lApuId)
      lCmemIfApuCfg = CFG_642__APU_0_CU_0_63_SMEM_0_3;
   return lCmemIfApuCfg;
}

#if 0
  #define INTR_DEBUG_PRINTF(...) printf( __VA_ARGS__)
#else
  #define INTR_DEBUG_PRINTF(...)
#endif

class ApexTaskInterrupCallbackParams
{
public:
   ApexTaskInterrupCallbackParams();

   int32_t mApexId;
   int32_t mApuId;
   OAL_SEMAPHORE* mpWaitSemaphore;
   int32_t* mpWaitSync;
   void (*mpUserDefinedCallback)(void* lpParam, int32_t* lpRetVal);
   void* mpUserDefinedCallbackParam;
   int32_t* mpUserDefinedCallbackRetVal;
};

ApexTaskInterrupCallbackParams::ApexTaskInterrupCallbackParams()
{
   mApexId = 0;
   mApuId = 0;
   mpWaitSemaphore = 0;
   mpWaitSync = 0;
   mpUserDefinedCallback = 0;
   mpUserDefinedCallbackParam = 0;
   mpUserDefinedCallbackRetVal = 0;
}

ApexTask::ApexTask()
{
   mpNativeFunction = 0;
   mApuBinaryPath = "UNDEFINED";
   mpIssSim = 0;
   mpParamsLocal = 0;
   mpIoDescInLocal = 0;
   mpIoDescOutLocal = 0;
   mpParams = 0;
   mParamSizeInBytes = 0;
   mpIoDescIn = 0;
   mNumIoDescIn = 0;
   mpIoDescOut = 0;
   mNumIoDescOut = 0;
   mpAcfIoDescIn = 0;
   mNumAcfIoDescIn = 0;
   mpAcfIoDescOut = 0;
   mNumAcfIoDescOut = 0;
   mpAcfIoMceDescIn = 0;
   mNumAcfIoMceDescIn = 0;
   mApexIdx = 0;
   mClusterIdx = 0;
   mApuIdx = 0;
   mCuArraySize = 0;
   mpApuExecutable = 0;
   mpUserDefinedCallback = 0;
   mpUserDefinedCallbackParam = 0;
   mpUserDefinedCallbackRetVal = 0;
   mCallbackRetVal = 0;
   mApuProgFuncPtr = 0;
   mLoadSegments = 0;
   mpLoadPmem = 0;
   mLoadPmemSizeInWords = 0;
   mpLoadDmem = 0;
   mLoadDmemSizeInWords = 0;
   mpLoadCmem = 0;
   mLoadCmemSizeInWords = 0;
   mpPmemLoadOal = 0;
   mpDmemLoadOal = 0;
   mpCmemLoadOal = 0;
   mpLoadSegmentsLLOal = 0;
   mWaitTimeoutInUs = WAIT_TIMEOUT_IN_US;
   mWaitMode = WAIT_MODE_INTERRUPT;
   mWaitSync = 0;
   mWaitSemaphore = 0;
   mCmemHeapBaseAddrOffset = 0;
   mSmemBaseAddrOffset = 0;
   mPmemBaseAddrOffset = 0;
   mPmemSizeInBytes = 0;
   mDmemBaseAddrOffset = 0;
   mDmemSizeInBytes = 0;
   mDmemHeapBaseAddrOffset = 0;
   mMaxCmemSize = 0;
   mMaxDmemSize = 0;
   mMaxSmemSize = 0;
   mStaticCmemReqInBytes = 0;
   mStaticDmemReqInBytes = 0;
   mPmemReqInBytes = 0;
   mCmemHeapSizeInBytes = 0;
   mDmemHeapSizeInBytes = 0;
   mApuTaskRetVal = 0;
   mpCallbackParam = 0;
   mpAcfIoMceDescInLocal = 0;
   mpAcfIoDescInLocal = 0;
   mpAcfIoDescOutLocal = 0;
}

ApexTask::~ApexTask()
{
   if (mpPmemLoadOal)       OAL_MemoryFree(mpPmemLoadOal);
   if (mpDmemLoadOal)       OAL_MemoryFree(mpDmemLoadOal);
   if (mpCmemLoadOal)       OAL_MemoryFree(mpCmemLoadOal);
   if (mpLoadSegmentsLLOal) OAL_MemoryFree(mpLoadSegmentsLLOal);
   if (mWaitSemaphore)      OAL_SemaphoreDelete(mWaitSemaphore);
   if (mpCallbackParam)     delete mpCallbackParam;
}

int32_t ApexTask::SetupIss(std::string lPathToBinary, std::string lUuidFilePath)
{
   printf("ApexTask::SetupIss -> ISS path has not been implemented for APEX2!\n");
   return APEXTASK_ERROR;
}

int32_t ApexTask::SetupNative(int32_t (*lpFunc) (void*))
{
   printf("ApexTask::SetupIss -> Native path has not been implemented for APEX2!\n");
   return APEXTASK_ERROR;
}

int32_t ApexTask::SetupHw(struct ApexIpc_ApuExecutable* lpApuExecutable)
{
   printf("ApexTask::SetupHw -> this version is not supported on APEX2!\n");
   return APEXTASK_ERROR;
}

int32_t ApexTask::SetupHw(std::string lApuFunctionName,
                          const LOAD_SEGMENT_t* lLoadSegments,
                          const SEG_MEM_TYPE* lpLoadPmem, const int32_t lLoadPmemSizeInWords,
                          const SEG_MEM_TYPE* lpLoadDmem, const int32_t lLoadDmemSizeInBytes,
                          const SEG_MEM_TYPE* lpLoadCmem, const int32_t lLoadCmemSize)
{
   int32_t lRetVal = APEXTASK_SUCCESS;

   mProcessIdentifier = lApuFunctionName;
   
   mLoadSegments        = lLoadSegments;
   mpLoadPmem           = lpLoadPmem;
   mLoadPmemSizeInWords = (lLoadPmemSizeInWords+3) & 0xFFFFFFFC; //round up to 128-bit
   mpLoadDmem           = lpLoadDmem;
   mLoadDmemSizeInWords = ((lLoadDmemSizeInBytes+15) & 0xFFFFFFF0)>>2; //round up to 128-bit and convert to words
   mpLoadCmem           = lpLoadCmem;
   mLoadCmemSizeInWords = lLoadCmemSize;
   
   //copy apu load into OAL buffers so it can be accessed by DMA
   if (0 != apu_hal_CopyLoadToOalMemory(mpLoadPmem, mLoadPmemSizeInWords,
                                        mpLoadDmem, mLoadDmemSizeInWords,
                                        mpLoadCmem, mLoadCmemSizeInWords,
                                        &mpPmemLoadOal,
                                        &mpDmemLoadOal,
                                        &mpCmemLoadOal))
   {
      lRetVal = APEXTASK_ERROR;
   }
   
   //initialize 'mApuProgFuncPtr'
   KernelManager lKernelManager;
   if (0 == lKernelManager.InitLoadKernelDB(0, mLoadSegments)) //first param is 'apuid' but it is not used ...
   {
      //the name of the apu program is the same as the process identifier
      if (-1 == lKernelManager.ReturnKernelID((int*)&mApuProgFuncPtr, lApuFunctionName.c_str()))
      {
         printf("ApexTask::SetupHwSetupHw -> '%s' could not be found in the APU load\n", lApuFunctionName.c_str());
         lRetVal = APEXTASK_ERROR;
      }
   }
   else
   {
      lRetVal = APEXTASK_ERROR;
   }
   
   //create wait semaphore
   int32_t lRes = OAL_SemaphoreCreate(&mWaitSemaphore, lApuFunctionName.c_str(), OAL_SEMAPHORE_COUNTING, 0);
   if (OAL_SUCCESS != lRes)
      lRetVal = APEXTASK_ERROR;

   //extract apex memory requirements from the load (static cmem, static dmem, program mem)
   apu_hal_QueryLoad_MEM_REQ(mLoadSegments, &mStaticCmemReqInBytes, &mStaticDmemReqInBytes, &mPmemReqInBytes);

   return lRetVal;
}

int32_t ApexTask::SetApuCfg(int32_t lApexIdx, int32_t lClusterIdx, int32_t lApuIdx, int32_t lCuArraySize)
{
   int32_t lRetVal = APEXTASK_SUCCESS;

   mApexIdx       = lApexIdx;
   mClusterIdx    = lClusterIdx; //ignored in apex2 case
   mApuIdx        = lApuIdx;
   mCuArraySize   = lCuArraySize;
   
   //currently, SMEM base is decided depending on which APU is in use (0 or 1)
   if (1 == mApuIdx) mSmemBaseAddrOffset = (64*1024);
   else              mSmemBaseAddrOffset = 0;

   int32_t lCut2Flag = 0;
   if (832 == global_hal_RetRtlBuildNumber(mApexIdx)) //if RTL build is cut 2.0
      lCut2Flag = 1;

   //determine max avail mem based on apu configuration
   if (0 == RetApuMemSizes(TranslateToCmemIfApuCfg(mApuIdx, mCuArraySize), &mMaxSmemSize, &mMaxCmemSize))
   {
      //----------------------------------------
      //split available SMEM into PMEM and DMEM
      //----------------------------------------

      int32_t lTmp = 0;

      if (0 == lCut2Flag) //if RTL build is NOT cut 2.0
      {
         //WORKAROUND FOR CLOVER-386
         //for the purpose of this calculation, assume PMEM will occupy 'entire' SMEM banks to ensure there is no overlap between
         //the SMEM bank used for DMEM and the SMEM bank used for PMEM; round up required PMEM to multiple of SMEM_BANK_SIZE
         const int32_t SMEM_BANK_SIZE = (1024*32); //32K
         mPmemSizeInBytes = mPmemReqInBytes;
         if (mPmemSizeInBytes%SMEM_BANK_SIZE != 0)
            mPmemSizeInBytes = ((mPmemReqInBytes/SMEM_BANK_SIZE)+1)*SMEM_BANK_SIZE;
      }
      else //RTL build is cut 2.0
      {
         //ensure 16-byte alignment of PMEM region in SMEM
         lTmp = ((mPmemReqInBytes&0xF) == 0) ? 0 : (16-(mPmemReqInBytes&0xF));
         mPmemSizeInBytes = mPmemReqInBytes + lTmp;
      }

      mDmemBaseAddrOffset = 0;                               //DMEM set to reside at the start of the allocated SMEM region
      mPmemBaseAddrOffset = mMaxSmemSize - mPmemSizeInBytes; //PMEM set to reside at the end of the allocated SMEM region
      mDmemSizeInBytes = mMaxSmemSize - mPmemSizeInBytes;    //DMEM size = all allocated SMEM that doesn't belong to PMEM

      //DEBUGGING: swap order so PMEM occurs at start of the allocated SMEM region and DMEM occurs after it (FOR DEBUGGING!!!)
      //mPmemBaseAddrOffset = 0;                            //PMEM set to reside at the start of the allocated SMEM region
      //mDmemBaseAddrOffset = mPmemSizeInBytes;             //DMEM set to reside just after PMEM

      //---------------------
      //calculate heap sizes
      //---------------------

      if (mPmemSizeInBytes <= (mMaxSmemSize - mStaticDmemReqInBytes))
      {
         //create dmem/cmem heaps with remainder of SMEM region (between linked mapped DMEM at the top and PMEM at the bottom)

         //DMEM HEAP
         lTmp = ((mStaticDmemReqInBytes&0xF) == 0) ? 0 : (16-(mStaticDmemReqInBytes&0xF)); //start of dmem heap should be 16-byte aligned
         mDmemHeapBaseAddrOffset = mStaticDmemReqInBytes + lTmp; //assume DMEM heap follows 'blob' of DMEM reserved by the linker
         mDmemHeapSizeInBytes = mMaxSmemSize - (mStaticDmemReqInBytes + lTmp) - mPmemSizeInBytes;

         //CMEM HEAP
         lTmp = ((mStaticCmemReqInBytes&0x3) == 0) ? 0 : (4-(mStaticCmemReqInBytes&0x3)); //start of cmem heap should be 4-byte aligned
         mCmemHeapBaseAddrOffset = mStaticCmemReqInBytes + lTmp; //assume CMEM heap follows 'blob' of CMEM reserved by the linker
         mCmemHeapSizeInBytes = mMaxCmemSize - (mStaticCmemReqInBytes + lTmp);
      }
      else
      {
         printf("ACF_Process_APU::DetectAvailApexMemory() -> insufficient program memory space for %s (%d bytes required).\n", mProcessIdentifier.c_str(), mPmemReqInBytes);
         lRetVal = APEXTASK_ERROR;
      }

   #if 0
         printf("CMEM HEAP: OFFSET = 0x%08X | SIZE = %d\n", mCmemHeapBaseAddrOffset, mCmemHeapSizeInBytes);
         printf("SMEM:      OFFSET = 0x%08X | SIZE = %d (first %d bytes reserved by linker)\n", mSmemBaseAddrOffset, mMaxSmemSize, mStaticDmemReq);
         printf("   PMEM: OFFSET = 0x%08X | SIZE = %d\n", mPmemBaseAddrOffset, mPmemSizeInBytes);
         printf("   DMEM: OFFSET = 0x%08X | SIZE = %d\n", mDmemBaseAddrOffset, mDmemSizeInBytes);
         printf("      DMEM STATIC BLOB: OFFSET = 0x00000000 | SIZE = %d\n", mStaticDmemReq);
         printf("      DMEM HEAP:        OFFSET = 0x%08X | SIZE = %d\n", mDmemHeapBaseAddrOffset, mDmemHeapSizeInBytes);
         printf("\n");
   #endif

   }
   else
   {
      printf("ApexTask::SetApuCfg -> failed to determine upper memory limits based on APU configuration\n");
      lRetVal = APEXTASK_ERROR;
   }

   int32_t lClover389Flag = 0;
   if (0 == lCut2Flag) //if RTL build is NOT cut 2.0
   {
      //CLOVER-389 workaround:
      //in the case where the APEX is configured for multi-apu operation, all mem-to-mem dma writes directly to the first 64K of SMEM and first 128K of CMEM must be single beat!!
      if (0 == lApuIdx && 32 == lCuArraySize)
         lClover389Flag = 1;
   }

   //generated LL for apu loading (needs to be done whenever APU configuration changes!)
   if (0 != apu_hal_GenLoadLL(mApexIdx, mApuIdx, mLoadSegments, mpLoadPmem, mpLoadDmem, mpLoadCmem,
                              (mpPmemLoadOal != 0) ? (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(mpPmemLoadOal, ACCESS_PHY) : 0,
                              (mpDmemLoadOal != 0) ? (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(mpDmemLoadOal, ACCESS_PHY) : 0,
                              (mpCmemLoadOal != 0) ? (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(mpCmemLoadOal, ACCESS_PHY) : 0,
                              mSmemBaseAddrOffset + mPmemBaseAddrOffset, //pmem offset
                              mSmemBaseAddrOffset + mDmemBaseAddrOffset, //dmem offset
                              mDmemSizeInBytes,
                              TranslateToCmemIfApuCfg(lApuIdx, lCuArraySize),
                              &mpLoadSegmentsLLOal,
                              lClover389Flag))
   {
      printf("ApexTask::SetApuCfg -> apu_hal_GenLoadLL failure\n");
      lRetVal = APEXTASK_ERROR;
   }

   return lRetVal;
}

int32_t ApexTask::QueryApuLoadMemReq(int32_t* lpStaticCmemReqInBytes, int32_t* lpStaticDmemReqInBytes, int32_t* lpPmemReqInBytes)
{
   int32_t lRetVal = APEXTASK_SUCCESS;

   *lpStaticCmemReqInBytes = mStaticCmemReqInBytes;
   *lpStaticDmemReqInBytes = mStaticDmemReqInBytes;
   *lpPmemReqInBytes       = mPmemReqInBytes;

   return lRetVal;
}

int32_t ApexTask::QueryAvailHeapSize(int32_t* lpCmemHeapSizeInBytes,  int32_t* lpDmemHeapSizeInBytes)
{
   int lRetVal = APEXTASK_SUCCESS;

   *lpCmemHeapSizeInBytes = mCmemHeapSizeInBytes;
   *lpDmemHeapSizeInBytes = mDmemHeapSizeInBytes;

   return lRetVal;
}

int32_t ApexTask::QueryApex2SpecificVals(int32_t* lpSmemBaseAddrOffset,
                                         int32_t* lpDmemBaseAddrOffset,
                                         int32_t* lpPmemBaseAddrOffset,
                                         int32_t* lpCmemHeapBaseAddrOffset,
                                         int32_t* lpDmemHeapBaseAddrOffset)
{
   int lRetVal = APEXTASK_SUCCESS;

   *lpSmemBaseAddrOffset     = mSmemBaseAddrOffset;
   *lpDmemBaseAddrOffset     = mDmemBaseAddrOffset;
   *lpPmemBaseAddrOffset     = mPmemBaseAddrOffset;
   *lpCmemHeapBaseAddrOffset = mCmemHeapBaseAddrOffset;
   *lpDmemHeapBaseAddrOffset = mDmemHeapBaseAddrOffset;

   return lRetVal;
}

void ApexTask::SetParams(void* lpParams, int32_t lParamSizeInBytes,
                         IoDesc* lpIoDescIn, int32_t lNumIoDescIn,
                         IoDesc* lpIoDescOut, int32_t lNumIoDescOut)
{
   printf("ApexTask::SetParams -> this version is not yet supported on APEX2\n");
}

void ApexTask::SetParams(void* lpParams, int32_t lParamSizeInBytes,
                         struct _acf_io_mce_desc* lpIoMceDescIn, int32_t lNumIoMceDescIn,
                         struct _acf_io_desc* lpIoDescIn, int32_t lNumIoDescIn,
                         struct _acf_io_desc* lpIoDescOut, int32_t lNumIoDescOut)
{
   mpParams           = lpParams;
   mParamSizeInBytes  = lParamSizeInBytes;
   mpAcfIoMceDescIn   = lpIoMceDescIn;
   mNumAcfIoMceDescIn = lNumIoMceDescIn;
   mpAcfIoDescIn      = lpIoDescIn;
   mNumAcfIoDescIn    = lNumIoDescIn;
   mpAcfIoDescOut     = lpIoDescOut;
   mNumAcfIoDescOut   = lNumIoDescOut;
}

void ApexTaskInterruptCallback(void* lpParam, int32_t* lpRetVal)
{
   ApexTaskInterrupCallbackParams* lpCallbackParam = (ApexTaskInterrupCallbackParams*)lpParam;

   INTR_DEBUG_PRINTF("[%08X] ApexTaskInterruptCallback() activated\n", lpApexTask);

   int32_t lRes = 0;

   //clear calback
   int32_t lInterrupt = ((0 == lpCallbackParam->mApuId) ? SEQUENCER_RES_30 : SEQUENCER_RES_31); //TODO: clean this up (should not make this assumption)
   lRes |= APEX_InterruptClearCallback(lpCallbackParam->mApexId, lInterrupt);

    //release wait sync
 #ifdef __STANDALONE__
   *lpCallbackParam->mpWaitSync = 1;
#else //all other OS
   lRes |= OAL_SemaphoreRelease(*lpCallbackParam->mpWaitSemaphore);
#endif

   if (0 != lRes)
   {
      printf("ApexTaskInterruptCallback failure!!\n");
      *lpRetVal = lRes;
   }
   else
   {
      *lpRetVal = 0;
   }

   //call user defined callback
   if (0 != lpCallbackParam->mpUserDefinedCallback)
      lpCallbackParam->mpUserDefinedCallback(lpCallbackParam->mpUserDefinedCallbackParam, lpCallbackParam->mpUserDefinedCallbackRetVal);
}

int32_t ApexTask::Start()
{
   int32_t lRetVal = APEXTASK_SUCCESS;
   
   //setup interrupt stuff
   int32_t lInterrupt = ((0 == mApuIdx) ? SEQUENCER_RES_30 : SEQUENCER_RES_31); //TODO: clean this up (should not make this assumption)
   if (0 == mpCallbackParam)
      mpCallbackParam = new ApexTaskInterrupCallbackParams;
   mpCallbackParam->mApexId = mApexIdx;
   mpCallbackParam->mApuId = mApuIdx;
   mpCallbackParam->mpWaitSemaphore = &mWaitSemaphore;
   mpCallbackParam->mpWaitSync = &mWaitSync;
   mpCallbackParam->mpUserDefinedCallback = mpUserDefinedCallback;
   mpCallbackParam->mpUserDefinedCallbackParam = mpUserDefinedCallbackParam;
   mpCallbackParam->mpUserDefinedCallbackRetVal = mpUserDefinedCallbackRetVal;
   mCallbackRetVal = 0;
   
   if (0 == APEX_InterruptRegisterCallback(mApexIdx, lInterrupt, &ApexTaskInterruptCallback, mpCallbackParam, &mCallbackRetVal))
   {
      INTR_DEBUG_PRINTF("[%08X] ACF_Process_APU::Start_Setup() -> apexid=%d apuid=%d intr=%d sem=%X\n", (uintptr_t)this, mApexId, mApuId, mAcfResInfo.zeroSeqRes_Done, (intptr_t)mWaitSemaphore);

      //configure the apu
      cmem_if_hal_set_apu_cfg(mApexIdx, TranslateToCmemIfApuCfg(mApuIdx, mCuArraySize));
      cmem_if_hal_set_apu_pmem_start(mApexIdx, mApuIdx, mSmemBaseAddrOffset + mPmemBaseAddrOffset);
      cmem_if_hal_set_apu_dmem_start(mApexIdx, mApuIdx, mSmemBaseAddrOffset + mDmemBaseAddrOffset);

      int32_t lLocalRet = 0;
      
      //perform apu load
      //----------------
   #if 1 //use DMA-based segment loading (faster)
      int32_t lDmaCh = ((0 == mApuIdx) ? 0 : 4); //TODO: clean this up (should not make this assumption)
      lLocalRet = apu_hal_LoadSegments_DMA(mApexIdx, mApuIdx, lDmaCh, mpLoadSegmentsLLOal);
   #else //use memcpy based segment loading (better for debugging)
      lLocalRet = apu_hal_LoadSegment(mApexIdx, mApuIdx, mLoadSegments);
   #endif

      if (0 == lLocalRet)
      {
         //enable the apu
         apu_hal_Enable(mApexIdx, mApuIdx, 1); //enable the apu
         
         //mk handshake
         int32_t lMsg = 0;
         lLocalRet = apumk_hal_host_com_rx(mApexIdx, (int)mApuIdx, &lMsg, 500000);
         if (0 == lLocalRet)
         {
            //mk send command
            lLocalRet = apumk_hal_send_cmd(mApexIdx, mApuIdx, (int)mApuProgFuncPtr, mParamSizeInBytes/4, (const int32_t*)mpParams);
            if (0 == lLocalRet)
            {
               //mk start execution
               lLocalRet = apumk_hal_host_com_tx(mApexIdx, (int)mApuIdx, HOST_MSG_SENDCMD, 500000);
               if (0 == lLocalRet)
               {
                  //EVERYTHING OK!
               }
               else //mk start execution failure
               {
                  printf("ApexTask::Start -> apu mk communication failed (apumk_hal_host_com_tx, return value = %d)\n", lLocalRet);
                  lRetVal = APEXTASK_ERROR;
               }
            }
            else //mk send command failure
            {
               printf("ApexTask::Start -> apu send command failure (apumk_hal_send_cmd, return value = %d)\n", lLocalRet);
               lRetVal = APEXTASK_ERROR;
            }
         }
         else //mk handshake failure
         {
            printf("ApexTask::Start -> apu mk communication failure (apumk_hal_host_com_rx, return value = %d))\n", lLocalRet);
            lRetVal = APEXTASK_ERROR;
         }
      }
      else //apu load failure
      {
         printf("ApexTask::Start -> apu load failure\n");
         lRetVal = APEXTASK_ERROR;
      }
   }
   else //APEX_InterruptRegisterCallback failure
   {
      printf("ApexTask::Start -> APEX_InterruptRegisterCallback failure\n");
      lRetVal = APEXTASK_ERROR;
   }

   return lRetVal;
}

int32_t ApexTask::Start(void (*lpCallback)(void* lpParam, int32_t* lpRetVal),
                        void* lpCallbackParam,
                        int32_t* lpCallbackRetVal)
{
   mpUserDefinedCallback       = lpCallback;
   mpUserDefinedCallbackParam  = lpCallbackParam;
   mpUserDefinedCallbackRetVal = lpCallbackRetVal;

   return Start();
}

int32_t ApexTask::Wait()
{
   int32_t lRetVal = APEXTASK_SUCCESS;
   
   if (WAIT_MODE_INTERRUPT == mWaitMode)
   {      
#ifdef __STANDALONE__
      uint64_t lTimeoutProbe = RET_TICKS;
      while (0 == mWaitSync)
      {
         if (CONVERT_TICKS_TO_US(RET_TICKS - lTimeoutProbe) >= mWaitTimeoutInUs)
         {
            printf("ApexTask::Wait -> polling-based timeout (%d us)\n", mWaitTimeoutInUs);
            lRetVal = APEXTASK_ERROR_TIMEOUT;
            break;
         }
      }
      mWaitSync = 0; //reset state back to 0
#else //all other OS
      if (OAL_SUCCESS != OAL_SemaphoreObtain(mWaitSemaphore, mWaitTimeoutInUs))
      {
         //if timeout occurs, immediately disable the apex.  there is still a small window between when the host detects the timeout and when
         //the APU is disabled during which the APU program may release the 'done' semaphore (this is only a concern if the
         //APU program is continuing to execute and the timeout lines up closely with the APU program completion). to handle this case,
         //the semaphore will be obtained again with a small timeout after disabling the APU.
         apu_hal_Enable(mApexIdx, mApuIdx, 0);
         OAL_SemaphoreObtain(mWaitSemaphore, 50);

         INTR_DEBUG_PRINTF("[%08X] ApexTask::Wait -> failed to obtain semaphore %X (seq_isr = 0x%08X)\n", (uintptr_t)this, (uintptr_t)mWaitSemaphore, seq_hal_query_interrupts(mApexId));
         printf("ApexTask::Wait -> semaphore-based timeout (%d us)\n", mWaitTimeoutInUs);
         lRetVal = APEXTASK_ERROR_TIMEOUT;
      }
#endif
   }
   else //polling-based wait mode
   {
      uint64_t lTimeoutProbe = RET_TICKS;

      uint32_t lSeqIsrVal = 0;
      int32_t lDoneInterrupt = ((0 == mApuIdx) ? 30 : 31);
      uint32_t lMask = (1ul<<lDoneInterrupt);
      while ((lSeqIsrVal & lMask) != lMask)
      {
         lSeqIsrVal = seq_hal_query_interrupts(mApexIdx);

         if (CONVERT_TICKS_TO_US(RET_TICKS - lTimeoutProbe) >= mWaitTimeoutInUs)
         {
            apu_hal_Enable(mApexIdx, mApuIdx, 0);

            printf("ApexTask::Wait -> polling-based timeout (%d us)\n", mWaitTimeoutInUs);
            lRetVal = APEXTASK_ERROR_TIMEOUT;
            break;
         }
      }

      seq_hal_clear_interrupts(mApexIdx, lMask);

      //call user defined callback
      if (0 != mpUserDefinedCallback)
         mpUserDefinedCallback(mpUserDefinedCallbackParam, mpUserDefinedCallbackRetVal);
   }

   //clear user-defined callback
   mpUserDefinedCallback = 0;

   if (APEXTASK_SUCCESS == lRetVal)
   {
      //sync with mk to retrieve apu program return value
      int32_t lMsg = 0;
      int lRetVal_rx = apumk_hal_host_com_rx(mApexIdx, (int)mApuIdx, &lMsg, 500000);
      if (0 == lRetVal_rx)
      {
         mApuTaskRetVal = RET_VAL(lMsg);
      }
      else
      {
         printf("ApexTask::Wait -> apu mk communication failed (apumk_hal_host_com_rx, return value = %d)\n", lRetVal_rx);
         lRetVal = APEXTASK_ERROR;
      }
   }

   //disable the apu (redundant in timeout failure case, but no harm in it)
   apu_hal_Enable(mApexIdx, mApuIdx, 0);

   return lRetVal;
}

int32_t ApexTask::Wait(int32_t lTimeoutInUs, int32_t& lApuTaskRetVal)
{
   int32_t lRetVal = APEXTASK_SUCCESS;

   mWaitTimeoutInUs = lTimeoutInUs;

   lRetVal = Wait();

   lApuTaskRetVal = mApuTaskRetVal;

   return lRetVal;
}
