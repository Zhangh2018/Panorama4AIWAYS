#ifndef _ACF_PROCESS_APU_RESIZE_V_H_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_RESIZE_V_H_NATIVE_KERNEL_WRAPPER

#include <RESIZE_V_H.hpp>

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
   [0] corresponds to indirect input 'SRC' source
   [1] corresponds to indirect input 'SRC' offset
   [2] corresponds to input 'SRC_WIDTH'
   [3] corresponds to input 'SRC_HEIGHT'
   [4] corresponds to input 'DST_WIDTH'
   [5] corresponds to input 'DST_HEIGHT'
   [6] corresponds to output 'DST'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void RESIZE_V_H_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //SRC
   ACF_DataDesc SRC_src;
   ACF_DataDesc SRC_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   SRC_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   SRC_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SRC_WIDTH
   ACF_DataDesc SRC_WIDTH;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   SRC_WIDTH.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SRC_HEIGHT
   ACF_DataDesc SRC_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   SRC_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DST_WIDTH
   ACF_DataDesc DST_WIDTH;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   DST_WIDTH.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DST_HEIGHT
   ACF_DataDesc DST_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   DST_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DST
   ACF_DataDesc DST;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   DST.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   RESIZE_V_H lProcess;
   lProcess.Initialize();
   lProcess.ConnectIndirectInput("SRC", SRC_src, SRC_offset);
   lProcess.ConnectIO("SRC_WIDTH", SRC_WIDTH);
   lProcess.ConnectIO("SRC_HEIGHT", SRC_HEIGHT);
   lProcess.ConnectIO("DST_WIDTH", DST_WIDTH);
   lProcess.ConnectIO("DST_HEIGHT", DST_HEIGHT);
   lProcess.ConnectIO("DST", DST);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_RESIZE_V_H_NATIVE_KERNEL_WRAPPER
