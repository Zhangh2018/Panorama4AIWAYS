#ifndef _ACF_PROCESS_APU_APU_FAST_NMS_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_APU_FAST_NMS_NATIVE_KERNEL_WRAPPER

#include <APU_FAST_NMS.hpp>

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
   [0] corresponds to input 'FAST_GR_IN_IMG'
   [1] corresponds to input 'FAST_GR_IN_THR'
   [2] corresponds to output 'FAST_GR_OUT_IMG'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void APU_FAST_NMS_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //FAST_GR_IN_IMG
   ACF_DataDesc FAST_GR_IN_IMG;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   FAST_GR_IN_IMG.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //FAST_GR_IN_THR
   ACF_DataDesc FAST_GR_IN_THR;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   FAST_GR_IN_THR.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //FAST_GR_OUT_IMG
   ACF_DataDesc FAST_GR_OUT_IMG;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   FAST_GR_OUT_IMG.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   APU_FAST_NMS lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("FAST_GR_IN_IMG", FAST_GR_IN_IMG);
   lProcess.ConnectIO("FAST_GR_IN_THR", FAST_GR_IN_THR);
   lProcess.ConnectIO("FAST_GR_OUT_IMG", FAST_GR_OUT_IMG);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_APU_FAST_NMS_NATIVE_KERNEL_WRAPPER
