#ifndef _ACF_PROCESS_APU_AGGCF_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_AGGCF_NATIVE_KERNEL_WRAPPER

#include <AGGCF.hpp>

//structure encapsulating the information ACF requires for each I/O
typedef struct _acf_io_info {                                      
    int32_t      pBuf;   //CL framework will initialize this with a pointer to global memory (based on associated cl_mem entity)
    int32_t      width;  //width in unit of elements
    int32_t      height; //height in unit of elements
    int32_t      span;   //(2D only) the number of bytes required to jump from one line of bytes in memory to the 'next' line of bytes in memory
    icp::DATATYPE elementDataType; //element data type (icp::DATATYPE_08U, icp::DATATYPE_16S, etc.
    int32_t      elementDimX;     //element X dimension
    int32_t      elementDimY;     //element Y dimension
} acf_io_info;

/******************************************************
pArgs must point to an array with the following format:

acf_io_info MYKERNEL_args[6];

where:
   [0] corresponds to input 'INPUTL'
   [1] corresponds to input 'INPUTU'
   [2] corresponds to input 'INPUTV'
   [3] corresponds to input 'PARAMS'
   [4] corresponds to output 'OUTPUTM'
   [5] corresponds to output 'OUTPUTO'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void AGGCF_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUTL
   ACF_DataDesc INPUTL;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUTL.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUTU
   ACF_DataDesc INPUTU;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUTU.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUTV
   ACF_DataDesc INPUTV;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   INPUTV.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PARAMS
   ACF_DataDesc PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTM
   ACF_DataDesc OUTPUTM;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   OUTPUTM.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTO
   ACF_DataDesc OUTPUTO;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   OUTPUTO.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   AGGCF lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUTL", INPUTL);
   lProcess.ConnectIO("INPUTU", INPUTU);
   lProcess.ConnectIO("INPUTV", INPUTV);
   lProcess.ConnectIO("PARAMS", PARAMS);
   lProcess.ConnectIO("OUTPUTM", OUTPUTM);
   lProcess.ConnectIO("OUTPUTO", OUTPUTO);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_AGGCF_NATIVE_KERNEL_WRAPPER
