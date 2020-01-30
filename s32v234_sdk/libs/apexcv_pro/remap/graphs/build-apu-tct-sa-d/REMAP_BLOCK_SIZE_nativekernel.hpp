#ifndef _ACF_PROCESS_APU_REMAP_BLOCK_SIZE_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_REMAP_BLOCK_SIZE_NATIVE_KERNEL_WRAPPER

#include <REMAP_BLOCK_SIZE.hpp>

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
   [0] corresponds to input 'MAP'
   [1] corresponds to output 'TWO_HEIGHT'
   [2] corresponds to output 'FOUR_HEIGHT'
   [3] corresponds to output 'EIGHT_HEIGHT'
   [4] corresponds to output 'SIXTEEN_HEIGHT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void REMAP_BLOCK_SIZE_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //MAP
   ACF_DataDesc MAP;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   MAP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //TWO_HEIGHT
   ACF_DataDesc TWO_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   TWO_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //FOUR_HEIGHT
   ACF_DataDesc FOUR_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   FOUR_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //EIGHT_HEIGHT
   ACF_DataDesc EIGHT_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   EIGHT_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SIXTEEN_HEIGHT
   ACF_DataDesc SIXTEEN_HEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   SIXTEEN_HEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   REMAP_BLOCK_SIZE lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("MAP", MAP);
   lProcess.ConnectIO("TWO_HEIGHT", TWO_HEIGHT);
   lProcess.ConnectIO("FOUR_HEIGHT", FOUR_HEIGHT);
   lProcess.ConnectIO("EIGHT_HEIGHT", EIGHT_HEIGHT);
   lProcess.ConnectIO("SIXTEEN_HEIGHT", SIXTEEN_HEIGHT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_REMAP_BLOCK_SIZE_NATIVE_KERNEL_WRAPPER
