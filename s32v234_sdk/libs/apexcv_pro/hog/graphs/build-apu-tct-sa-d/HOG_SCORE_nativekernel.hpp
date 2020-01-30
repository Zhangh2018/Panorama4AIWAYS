#ifndef _ACF_PROCESS_APU_HOG_SCORE_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_HOG_SCORE_NATIVE_KERNEL_WRAPPER

#include <HOG_SCORE.hpp>

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
   [0] corresponds to input 'IMAGE'
   [1] corresponds to input 'SVM_FUNCTION'
   [2] corresponds to input 'SVM_TRANSFORM_MODE'
   [3] corresponds to input 'WIN_BLOCKS'
   [4] corresponds to output 'SCORES'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void HOG_SCORE_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //IMAGE
   ACF_DataDesc IMAGE;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   IMAGE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SVM_FUNCTION
   ACF_DataDesc SVM_FUNCTION;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   SVM_FUNCTION.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SVM_TRANSFORM_MODE
   ACF_DataDesc SVM_TRANSFORM_MODE;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   SVM_TRANSFORM_MODE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //WIN_BLOCKS
   ACF_DataDesc WIN_BLOCKS;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   WIN_BLOCKS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SCORES
   ACF_DataDesc SCORES;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   SCORES.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   HOG_SCORE lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("IMAGE", IMAGE);
   lProcess.ConnectIO("SVM_FUNCTION", SVM_FUNCTION);
   lProcess.ConnectIO("SVM_TRANSFORM_MODE", SVM_TRANSFORM_MODE);
   lProcess.ConnectIO("WIN_BLOCKS", WIN_BLOCKS);
   lProcess.ConnectIO("SCORES", SCORES);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_HOG_SCORE_NATIVE_KERNEL_WRAPPER
