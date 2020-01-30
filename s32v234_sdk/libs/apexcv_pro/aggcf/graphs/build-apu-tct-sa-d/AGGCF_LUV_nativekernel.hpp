#ifndef _ACF_PROCESS_APU_AGGCF_LUV_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_AGGCF_LUV_NATIVE_KERNEL_WRAPPER

#include <AGGCF_LUV.hpp>

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

acf_io_info MYKERNEL_args[4];

where:
   [0] corresponds to input 'INPUT'
   [1] corresponds to output 'OUTPUTL'
   [2] corresponds to output 'OUTPUTU'
   [3] corresponds to output 'OUTPUTV'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void AGGCF_LUV_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTL
   ACF_DataDesc OUTPUTL;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   OUTPUTL.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTU
   ACF_DataDesc OUTPUTU;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   OUTPUTU.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTV
   ACF_DataDesc OUTPUTV;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   OUTPUTV.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   AGGCF_LUV lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("OUTPUTL", OUTPUTL);
   lProcess.ConnectIO("OUTPUTU", OUTPUTU);
   lProcess.ConnectIO("OUTPUTV", OUTPUTV);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_AGGCF_LUV_NATIVE_KERNEL_WRAPPER
