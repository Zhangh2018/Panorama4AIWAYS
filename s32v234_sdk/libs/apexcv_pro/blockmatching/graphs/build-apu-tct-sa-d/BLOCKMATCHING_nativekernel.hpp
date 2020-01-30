#ifndef _ACF_PROCESS_APU_BLOCKMATCHING_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_BLOCKMATCHING_NATIVE_KERNEL_WRAPPER

#include <BLOCKMATCHING.hpp>

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
   [0] corresponds to indirect input 'INPUT_TEMPLATE' source
   [1] corresponds to indirect input 'INPUT_TEMPLATE' offset
   [2] corresponds to indirect input 'INPUT_WINDOW' source
   [3] corresponds to indirect input 'INPUT_WINDOW' offset
   [4] corresponds to output 'OUTPUT_SCORE'
   [5] corresponds to output 'OUTPUT_POINTS'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void BLOCKMATCHING_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT_TEMPLATE
   ACF_DataDesc INPUT_TEMPLATE_src;
   ACF_DataDesc INPUT_TEMPLATE_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_TEMPLATE_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUT_TEMPLATE_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUT_WINDOW
   ACF_DataDesc INPUT_WINDOW_src;
   ACF_DataDesc INPUT_WINDOW_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   INPUT_WINDOW_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   INPUT_WINDOW_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_SCORE
   ACF_DataDesc OUTPUT_SCORE;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   OUTPUT_SCORE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_POINTS
   ACF_DataDesc OUTPUT_POINTS;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   OUTPUT_POINTS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   BLOCKMATCHING lProcess;
   lProcess.Initialize();
   lProcess.ConnectIndirectInput("INPUT_TEMPLATE", INPUT_TEMPLATE_src, INPUT_TEMPLATE_offset);
   lProcess.ConnectIndirectInput("INPUT_WINDOW", INPUT_WINDOW_src, INPUT_WINDOW_offset);
   lProcess.ConnectIO("OUTPUT_SCORE", OUTPUT_SCORE);
   lProcess.ConnectIO("OUTPUT_POINTS", OUTPUT_POINTS);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_BLOCKMATCHING_NATIVE_KERNEL_WRAPPER
