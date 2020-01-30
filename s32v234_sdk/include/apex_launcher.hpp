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

#ifndef _APEX_LAUNCHER_HPP
#define _APEX_LAUNCHER_HPP

#include <stdint.h>
#include <string>
#include <oal.h>

//forward declarations
class XimExec;
struct ApexIpc_ApuExecutable;
class IoDesc;
struct _acf_io_mce_desc;
struct _acf_io_desc;
class ApexTaskInterrupCallbackParams;

#ifndef SEG_HOST_TYPE
typedef int64_t  SEG_HOST_TYPE;
typedef uint32_t SEG_MEM_TYPE;
#endif
typedef SEG_HOST_TYPE LOAD_SEGMENT_t[4];
   
/*NOTES:
- currently from an APU binary perspective, iss uses *.elf, hw uses apu load, and native uses nothing.  from an
  interface perspective this cannot be nicely abstracted.
- user will be allowed to have their own param struct, however they must identify IO descriptors to the launcher. this
  is required because ApexTask needs to be aware of inputs and outputs in order to abstract the additional steps required
  for ISS simulation (i.e. in the ISS path there is a need to copy input data over to ISS XMEM, and copy outputs from ISS
  XMEM back to host memory).
- since we haven't attempted to unify apu loading between apex2 and apexd, and param format between acf and llrt, there are apex2/acf versions of various functions (e.g. SetupHw, SetParams).
  these differences can hopefully be merged in the future.

Example user-defined parameter class:

class MyParams
{
public:
   IoDesc myInputs[6];
   IoDesc myOutputs[2];
   int32_t myParam0;
   int32_t myParam1;
   int32_t myParamArray0[8];
   int32_t myParamArray1[5];
   ...
};
*/

//ASSUMPTION:
//users APU function has the signature: int32_t <function_name>(const void* <pointer to params>);

#define WAIT_TIMEOUT_IN_US 1000000 //1s

typedef enum _WAIT_MODE
{
   WAIT_MODE_INTERRUPT = 0,
   WAIT_MODE_POLLING
} WAIT_MODE;

//ApexTask error codes
enum
{
   APEXTASK_SUCCESS       = 0,
   APEXTASK_ERROR         = 1,
   APEXTASK_ERROR_TIMEOUT = 2
};

class ApexTask
{
public:
   
   ApexTask();
   ~ApexTask();

   //---------------------------------------
   //methods specific to the execution path 
   //---------------------------------------

   int32_t SetupIss(std::string lPathToBinary, std::string lUuidFilePath);
   int32_t SetupNative(int32_t (*lpFunc) (void*));
   int32_t SetupHw(struct ApexIpc_ApuExecutable* lpApuExecutable);
   int32_t SetupHw(std::string lApuFunctionName,
                   const LOAD_SEGMENT_t* lLoadSegments,
                   const SEG_MEM_TYPE* lpLoadPmem, const int32_t lLoadPmemSizeInWords,
                   const SEG_MEM_TYPE* lpLoadDmem, const int32_t lLoadDmemSizeInBytes,
                   const SEG_MEM_TYPE* lpLoadCmem, const int32_t lLoadCmemSize);
   
   //----------------------------------------------
   //methods common to ISS, native, and hw targets
   //----------------------------------------------

   //apexd style loading
   int32_t SetApuCfg(int32_t lApexIdx, int32_t lClusterIdx, int32_t lApuIdx, int32_t lCuArraySize);

   int32_t QueryApuLoadMemReq(int32_t* lpStaticCmemReqInBytes, int32_t* lpStaticDmemReqInBytes, int32_t* lpPmemReqInBytes);
   int32_t QueryAvailHeapSize(int32_t* lpCmemHeapSizeInBytes,  int32_t* lpDmemHeapSizeInBytes);

#if defined(APEX_TARGET_APEXD) //APEXD HW target
   //nothing here!
#else //APEX2 HW target
   int32_t QueryApex2SpecificVals(int32_t* lpSmemBaseAddrOffset,
                                  int32_t* lpDmemBaseAddrOffset,
                                  int32_t* lpPmemBaseAddrOffset,
                                  int32_t* lpCmemHeapBaseAddrOffset,
                                  int32_t* lpDmemHeapBaseAddrOffset);
#endif

   //llrt/generic style params
   void SetParams(void* lpParams, int32_t lParamSizeInBytes,
                  IoDesc* lpIoDescIn, int32_t lNumIoDescIn,
                  IoDesc* lpIoDescOut, int32_t lNumIoDescOut);
   //acf style params (for interim compatibility with ACF params until they are updated to use the generic style)
   void SetParams(void* lpParams, int32_t lParamSizeInBytes,
                  struct _acf_io_mce_desc* lpIoMceDescIn, int32_t lNumIoMceDescIn,
                  struct _acf_io_desc* lpIoDescIn, int32_t lNumIoDescIn,
                  struct _acf_io_desc* lpIoDescOut, int32_t lNumIoDescOut);

   int32_t Start();
   int32_t Start(void (*lpCallback)(void* lpParam, int32_t* lpRetVal),
                 void* lpCallbackParam,
                 int32_t* lpCallbackRetVal);
   int32_t Wait();
   int32_t Wait(int32_t lTimeoutInUs, int32_t& lApuTaskRetVal);

private:
   
   //iss path specific-------------------

   int32_t MapIoToIssXmemRegion();
   void CopyInputsFromHostMemToIssXmem();
   void CopyOutputsFromIssXmemToHostMem();
   void CopyParamsFromHostMemToIssDmem();
   std::string mApuBinaryPath;
   XimExec* mpIssSim;
   void* mpParamsLocal;
   IoDesc* mpIoDescInLocal;
   IoDesc* mpIoDescOutLocal;
   struct _acf_io_mce_desc* mpAcfIoMceDescInLocal;
   struct _acf_io_desc* mpAcfIoDescInLocal;
   struct _acf_io_desc* mpAcfIoDescOutLocal;

   //native path specific
   int32_t (*mpNativeFunction) (void*);

   //hw path specific-----------

   //apexd style apu load

   struct ApexIpc_ApuExecutable* mpApuExecutable;
   //<will eventually need to copy load info into OAL buffer like apex2 does>

   //apex2 style apu load

   const LOAD_SEGMENT_t* mLoadSegments;
   const SEG_MEM_TYPE* mpLoadPmem;
   int32_t mLoadPmemSizeInWords;
   const SEG_MEM_TYPE* mpLoadDmem;
   int32_t mLoadDmemSizeInWords;
   const SEG_MEM_TYPE* mpLoadCmem;
   int32_t mLoadCmemSizeInWords;
   void* mpPmemLoadOal;
   void* mpDmemLoadOal;
   void* mpCmemLoadOal;
   void* mpLoadSegmentsLLOal;
   int32_t mCmemHeapBaseAddrOffset; //CMEM 'heap' offset (relative to base CMEM address)
   int32_t mSmemBaseAddrOffset;     //SMEM offset (relative to start of 128K SMEM region associated with the APEX)
   int32_t mPmemBaseAddrOffset;     //PMEM region offset (relative to mSmemBaseAddrOffset)
   int32_t mPmemSizeInBytes;        //PMEM size
   int32_t mDmemBaseAddrOffset;     //DMEM region offset (relative to mSmemBaseAddrOffset)
   int32_t mDmemSizeInBytes;        //DMEM size
   int32_t mDmemHeapBaseAddrOffset; //DMEM 'heap' offset (relative to mDmemBaseAddrOffset)

   //common----------------

   //NOTE:
   // - should i abstract profiling retrieval as well?

   std::string mProcessIdentifier;

   int32_t mMaxCmemSize;
   int32_t mMaxDmemSize;
   int32_t mMaxSmemSize;
   int32_t mStaticCmemReqInBytes;
   int32_t mStaticDmemReqInBytes;
   int32_t mPmemReqInBytes;
   int32_t mCmemHeapSizeInBytes;
   int32_t mDmemHeapSizeInBytes;

   void* mpParams;
   int32_t mParamSizeInBytes;
   int32_t mApexIdx;
   int32_t mClusterIdx;
   int32_t mApuIdx;
   int32_t mCuArraySize;
   int32_t mWaitTimeoutInUs;
   WAIT_MODE mWaitMode;
   int32_t mApuTaskRetVal;
   //interrupt stuff
   OAL_SEMAPHORE mWaitSemaphore; //for OS case
   int32_t mWaitSync;            //for bare metal (i.e. standalone) case
   ApexTaskInterrupCallbackParams* mpCallbackParam;
   int32_t mCallbackRetVal;
   void (*mpUserDefinedCallback)(void* lpParam, int32_t* lpRetVal);
   void* mpUserDefinedCallbackParam;
   int32_t* mpUserDefinedCallbackRetVal;

   //apex2 style cfgs

   int32_t mApuProgFuncPtr; //function pointer associated with acf generated program (req for old uKernel)

   //llrt/generic style ios

   IoDesc* mpIoDescIn;
   int32_t mNumIoDescIn;
   IoDesc* mpIoDescOut;
   int32_t mNumIoDescOut;

   //acf style ios

   struct _acf_io_mce_desc* mpAcfIoMceDescIn;
   int32_t mNumAcfIoMceDescIn;
   struct _acf_io_desc* mpAcfIoDescIn;
   int32_t mNumAcfIoDescIn;
   struct _acf_io_desc* mpAcfIoDescOut;
   int32_t mNumAcfIoDescOut;
};

#endif //_APEX_LAUNCHER_HPP
