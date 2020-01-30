#ifndef _ACF_PROCESS_APU_CANNY_FULL_ITERATION_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_CANNY_FULL_ITERATION_NATIVE_KERNEL_WRAPPER

#include <CANNY_FULL_ITERATION.hpp>

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

acf_io_info MYKERNEL_args[5];

where:
   [0] corresponds to input 'INPUT'
   [1] corresponds to input 'LOW_THRESH'
   [2] corresponds to input 'HIGH_THRESH'
   [3] corresponds to output 'OUTPUT'
   [4] corresponds to output 'EDGE_OUTPUT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void CANNY_FULL_ITERATION_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //LOW_THRESH
   ACF_DataDesc LOW_THRESH;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   LOW_THRESH.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //HIGH_THRESH
   ACF_DataDesc HIGH_THRESH;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   HIGH_THRESH.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT
   ACF_DataDesc OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //EDGE_OUTPUT
   ACF_DataDesc EDGE_OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   EDGE_OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   CANNY_FULL_ITERATION lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("LOW_THRESH", LOW_THRESH);
   lProcess.ConnectIO("HIGH_THRESH", HIGH_THRESH);
   lProcess.ConnectIO("OUTPUT", OUTPUT);
   lProcess.ConnectIO("EDGE_OUTPUT", EDGE_OUTPUT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_CANNY_FULL_ITERATION_NATIVE_KERNEL_WRAPPER
