#ifndef _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_NATIVE_KERNEL_WRAPPER

#include <GFTT_B7N5_COMPUTE.hpp>

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

acf_io_info MYKERNEL_args[3];

where:
   [0] corresponds to input 'INPUT'
   [1] corresponds to input 'PARAMS'
   [2] corresponds to output 'OUTPUT_IMAGE'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void GFTT_B7N5_COMPUTE_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PARAMS
   ACF_DataDesc PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_IMAGE
   ACF_DataDesc OUTPUT_IMAGE;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   OUTPUT_IMAGE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   GFTT_B7N5_COMPUTE lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("PARAMS", PARAMS);
   lProcess.ConnectIO("OUTPUT_IMAGE", OUTPUT_IMAGE);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_GFTT_B7N5_COMPUTE_NATIVE_KERNEL_WRAPPER
