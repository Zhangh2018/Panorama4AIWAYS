#ifndef _ACF_PROCESS_APU_APU_FAST_SERIALIZED_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_APU_FAST_SERIALIZED_NATIVE_KERNEL_WRAPPER

#include <APU_FAST_SERIALIZED.hpp>

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
   [0] corresponds to input 'INPUT'
   [1] corresponds to input 'THRESHOLD'
   [2] corresponds to input 'CIRCUMFERENCE'
   [3] corresponds to input 'OUT_MAX_SIZE'
   [4] corresponds to output 'OUTPUT'
   [5] corresponds to output 'OUT_COUNTER'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void APU_FAST_SERIALIZED_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //THRESHOLD
   ACF_DataDesc THRESHOLD;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   THRESHOLD.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //CIRCUMFERENCE
   ACF_DataDesc CIRCUMFERENCE;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   CIRCUMFERENCE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUT_MAX_SIZE
   ACF_DataDesc OUT_MAX_SIZE;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   OUT_MAX_SIZE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT
   ACF_DataDesc OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUT_COUNTER
   ACF_DataDesc OUT_COUNTER;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   OUT_COUNTER.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   APU_FAST_SERIALIZED lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("THRESHOLD", THRESHOLD);
   lProcess.ConnectIO("CIRCUMFERENCE", CIRCUMFERENCE);
   lProcess.ConnectIO("OUT_MAX_SIZE", OUT_MAX_SIZE);
   lProcess.ConnectIO("OUTPUT", OUTPUT);
   lProcess.ConnectIO("OUT_COUNTER", OUT_COUNTER);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_APU_FAST_SERIALIZED_NATIVE_KERNEL_WRAPPER
