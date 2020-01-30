#ifndef _ACF_PROCESS_APU_AGGCF_HIST_TI2_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_AGGCF_HIST_TI2_NATIVE_KERNEL_WRAPPER

#include <AGGCF_HIST_TI2.hpp>

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

acf_io_info MYKERNEL_args[10];

where:
   [0] corresponds to input 'INPUTM'
   [1] corresponds to input 'INPUTO'
   [2] corresponds to input 'OFFSET'
   [3] corresponds to input 'PARAMS'
   [4] corresponds to output 'OUTPUTH1'
   [5] corresponds to output 'OUTPUTH2'
   [6] corresponds to output 'OUTPUTH3'
   [7] corresponds to output 'OUTPUTH4'
   [8] corresponds to output 'OUTPUTH5'
   [9] corresponds to output 'OUTPUTH6'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void AGGCF_HIST_TI2_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUTM
   ACF_DataDesc INPUTM;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUTM.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUTO
   ACF_DataDesc INPUTO;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUTO.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OFFSET
   ACF_DataDesc OFFSET;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   OFFSET.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PARAMS
   ACF_DataDesc PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH1
   ACF_DataDesc OUTPUTH1;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   OUTPUTH1.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH2
   ACF_DataDesc OUTPUTH2;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   OUTPUTH2.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH3
   ACF_DataDesc OUTPUTH3;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   OUTPUTH3.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH4
   ACF_DataDesc OUTPUTH4;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   OUTPUTH4.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH5
   ACF_DataDesc OUTPUTH5;
   lpIo = (acf_io_info*)((int32_t)pArgs+8*sizeof(acf_io_info));
   OUTPUTH5.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUTH6
   ACF_DataDesc OUTPUTH6;
   lpIo = (acf_io_info*)((int32_t)pArgs+9*sizeof(acf_io_info));
   OUTPUTH6.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   AGGCF_HIST_TI2 lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUTM", INPUTM);
   lProcess.ConnectIO("INPUTO", INPUTO);
   lProcess.ConnectIO("OFFSET", OFFSET);
   lProcess.ConnectIO("PARAMS", PARAMS);
   lProcess.ConnectIO("OUTPUTH1", OUTPUTH1);
   lProcess.ConnectIO("OUTPUTH2", OUTPUTH2);
   lProcess.ConnectIO("OUTPUTH3", OUTPUTH3);
   lProcess.ConnectIO("OUTPUTH4", OUTPUTH4);
   lProcess.ConnectIO("OUTPUTH5", OUTPUTH5);
   lProcess.ConnectIO("OUTPUTH6", OUTPUTH6);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_AGGCF_HIST_TI2_NATIVE_KERNEL_WRAPPER
