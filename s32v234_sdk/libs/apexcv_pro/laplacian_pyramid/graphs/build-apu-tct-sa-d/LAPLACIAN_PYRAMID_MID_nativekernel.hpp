#ifndef _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID_NATIVE_KERNEL_WRAPPER

#include <LAPLACIAN_PYRAMID_MID.hpp>

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
   [1] corresponds to output 'OUTPUT_PYRAMID'
   [2] corresponds to output 'OUTPUT_NEXT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void LAPLACIAN_PYRAMID_MID_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_PYRAMID
   ACF_DataDesc OUTPUT_PYRAMID;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   OUTPUT_PYRAMID.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_NEXT
   ACF_DataDesc OUTPUT_NEXT;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   OUTPUT_NEXT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   LAPLACIAN_PYRAMID_MID lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("OUTPUT_PYRAMID", OUTPUT_PYRAMID);
   lProcess.ConnectIO("OUTPUT_NEXT", OUTPUT_NEXT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_LAPLACIAN_PYRAMID_MID_NATIVE_KERNEL_WRAPPER
