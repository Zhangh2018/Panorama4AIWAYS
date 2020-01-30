#ifndef _ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION_NATIVE_KERNEL_WRAPPER

#include <HISTOGRAM_EQUALIZATION.hpp>

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
   [0] corresponds to input 'INPUT_0'
   [1] corresponds to input 'NUM_PIXELS'
   [2] corresponds to input 'IMAGE_HISTOGRAM'
   [3] corresponds to output 'OUTPUT_0'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void HISTOGRAM_EQUALIZATION_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT_0
   ACF_DataDesc INPUT_0;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_0.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //NUM_PIXELS
   ACF_DataDesc NUM_PIXELS;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   NUM_PIXELS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //IMAGE_HISTOGRAM
   ACF_DataDesc IMAGE_HISTOGRAM;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   IMAGE_HISTOGRAM.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_0
   ACF_DataDesc OUTPUT_0;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   OUTPUT_0.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   HISTOGRAM_EQUALIZATION lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT_0", INPUT_0);
   lProcess.ConnectIO("NUM_PIXELS", NUM_PIXELS);
   lProcess.ConnectIO("IMAGE_HISTOGRAM", IMAGE_HISTOGRAM);
   lProcess.ConnectIO("OUTPUT_0", OUTPUT_0);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_HISTOGRAM_EQUALIZATION_NATIVE_KERNEL_WRAPPER
