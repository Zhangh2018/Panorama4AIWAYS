#ifndef _ACF_PROCESS_APU_HT_RGB888X_TO_Y_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_HT_RGB888X_TO_Y_NATIVE_KERNEL_WRAPPER

#include <HT_RGB888X_TO_Y.hpp>

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
   [0] corresponds to input 'INPUT_0'
   [1] corresponds to input 'KA'
   [2] corresponds to input 'KB'
   [3] corresponds to input 'KC'
   [4] corresponds to input 'KR'
   [5] corresponds to output 'OUTPUT_0'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void HT_RGB888X_TO_Y_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT_0
   ACF_DataDesc INPUT_0;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_0.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //KA
   ACF_DataDesc KA;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   KA.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //KB
   ACF_DataDesc KB;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   KB.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //KC
   ACF_DataDesc KC;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   KC.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //KR
   ACF_DataDesc KR;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   KR.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_0
   ACF_DataDesc OUTPUT_0;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   OUTPUT_0.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   HT_RGB888X_TO_Y lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT_0", INPUT_0);
   lProcess.ConnectIO("KA", KA);
   lProcess.ConnectIO("KB", KB);
   lProcess.ConnectIO("KC", KC);
   lProcess.ConnectIO("KR", KR);
   lProcess.ConnectIO("OUTPUT_0", OUTPUT_0);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_HT_RGB888X_TO_Y_NATIVE_KERNEL_WRAPPER
