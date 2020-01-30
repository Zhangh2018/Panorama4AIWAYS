#ifndef _ACF_PROCESS_APU_AGGCF_SCALE_DOWN4_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_AGGCF_SCALE_DOWN4_NATIVE_KERNEL_WRAPPER

#include <AGGCF_SCALE_DOWN4.hpp>

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

acf_io_info MYKERNEL_args[2];

where:
   [0] corresponds to input 'SRC'
   [1] corresponds to output 'DST'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void AGGCF_SCALE_DOWN4_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //SRC
   ACF_DataDesc SRC;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   SRC.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DST
   ACF_DataDesc DST;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   DST.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   AGGCF_SCALE_DOWN4 lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("SRC", SRC);
   lProcess.ConnectIO("DST", DST);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_AGGCF_SCALE_DOWN4_NATIVE_KERNEL_WRAPPER
