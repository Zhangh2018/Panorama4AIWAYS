#ifndef _ACF_PROCESS_APU_APU_ORB_RBRIEF_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_APU_ORB_RBRIEF_NATIVE_KERNEL_WRAPPER

#include <APU_ORB_RBRIEF.hpp>

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

acf_io_info MYKERNEL_args[7];

where:
   [0] corresponds to indirect input 'INPUT' source
   [1] corresponds to indirect input 'INPUT' offset
   [2] corresponds to input 'ORIENTATION'
   [3] corresponds to input 'BIT_PATTERN'
   [4] corresponds to input 'DESCR_SIZE_B'
   [5] corresponds to input 'PATCH_SIZE'
   [6] corresponds to output 'OUTPUT'
   [7] corresponds to output 'COUNT_DESCR'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void APU_ORB_RBRIEF_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT_src;
   ACF_DataDesc INPUT_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUT_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //ORIENTATION
   ACF_DataDesc ORIENTATION;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   ORIENTATION.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //BIT_PATTERN
   ACF_DataDesc BIT_PATTERN;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   BIT_PATTERN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DESCR_SIZE_B
   ACF_DataDesc DESCR_SIZE_B;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   DESCR_SIZE_B.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PATCH_SIZE
   ACF_DataDesc PATCH_SIZE;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   PATCH_SIZE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT
   ACF_DataDesc OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //COUNT_DESCR
   ACF_DataDesc COUNT_DESCR;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   COUNT_DESCR.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   APU_ORB_RBRIEF lProcess;
   lProcess.Initialize();
   lProcess.ConnectIndirectInput("INPUT", INPUT_src, INPUT_offset);
   lProcess.ConnectIO("ORIENTATION", ORIENTATION);
   lProcess.ConnectIO("BIT_PATTERN", BIT_PATTERN);
   lProcess.ConnectIO("DESCR_SIZE_B", DESCR_SIZE_B);
   lProcess.ConnectIO("PATCH_SIZE", PATCH_SIZE);
   lProcess.ConnectIO("OUTPUT", OUTPUT);
   lProcess.ConnectIO("COUNT_DESCR", COUNT_DESCR);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_APU_ORB_RBRIEF_NATIVE_KERNEL_WRAPPER
