#ifndef _ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR_NATIVE_KERNEL_WRAPPER

#include <LBP_COMPARE_DESCRIPTOR.hpp>

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
   [0] corresponds to input 'INPUT_MODEL'
   [1] corresponds to input 'INPUT_TEST'
   [2] corresponds to output 'CLOSEST_ID'
   [3] corresponds to output 'DISTANCE'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void LBP_COMPARE_DESCRIPTOR_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT_MODEL
   ACF_DataDesc INPUT_MODEL;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_MODEL.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUT_TEST
   ACF_DataDesc INPUT_TEST;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUT_TEST.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //CLOSEST_ID
   ACF_DataDesc CLOSEST_ID;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   CLOSEST_ID.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DISTANCE
   ACF_DataDesc DISTANCE;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   DISTANCE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   LBP_COMPARE_DESCRIPTOR lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT_MODEL", INPUT_MODEL);
   lProcess.ConnectIO("INPUT_TEST", INPUT_TEST);
   lProcess.ConnectIO("CLOSEST_ID", CLOSEST_ID);
   lProcess.ConnectIO("DISTANCE", DISTANCE);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_LBP_COMPARE_DESCRIPTOR_NATIVE_KERNEL_WRAPPER
