#ifndef _ACF_PROCESS_APU_LKTRACKER_FULL_7X7_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_LKTRACKER_FULL_7X7_NATIVE_KERNEL_WRAPPER

#include <LKTRACKER_FULL_7X7.hpp>

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
   [0] corresponds to indirect input 'INPUT_TEMPLATE' source
   [1] corresponds to indirect input 'INPUT_TEMPLATE' offset
   [2] corresponds to indirect input 'INPUT_PATCH' source
   [3] corresponds to indirect input 'INPUT_PATCH' offset
   [4] corresponds to input 'INPUT_COOR_XY0'
   [5] corresponds to input 'INPUT_COOR_XY1'
   [6] corresponds to input 'PARAMS'
   [7] corresponds to output 'OUTPUT_COOR_XY1'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void LKTRACKER_FULL_7X7_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT_TEMPLATE
   ACF_DataDesc INPUT_TEMPLATE_src;
   ACF_DataDesc INPUT_TEMPLATE_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_TEMPLATE_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUT_TEMPLATE_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUT_PATCH
   ACF_DataDesc INPUT_PATCH_src;
   ACF_DataDesc INPUT_PATCH_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   INPUT_PATCH_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   INPUT_PATCH_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUT_COOR_XY0
   ACF_DataDesc INPUT_COOR_XY0;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   INPUT_COOR_XY0.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INPUT_COOR_XY1
   ACF_DataDesc INPUT_COOR_XY1;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   INPUT_COOR_XY1.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PARAMS
   ACF_DataDesc PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT_COOR_XY1
   ACF_DataDesc OUTPUT_COOR_XY1;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   OUTPUT_COOR_XY1.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   LKTRACKER_FULL_7X7 lProcess;
   lProcess.Initialize();
   lProcess.ConnectIndirectInput("INPUT_TEMPLATE", INPUT_TEMPLATE_src, INPUT_TEMPLATE_offset);
   lProcess.ConnectIndirectInput("INPUT_PATCH", INPUT_PATCH_src, INPUT_PATCH_offset);
   lProcess.ConnectIO("INPUT_COOR_XY0", INPUT_COOR_XY0);
   lProcess.ConnectIO("INPUT_COOR_XY1", INPUT_COOR_XY1);
   lProcess.ConnectIO("PARAMS", PARAMS);
   lProcess.ConnectIO("OUTPUT_COOR_XY1", OUTPUT_COOR_XY1);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_LKTRACKER_FULL_7X7_NATIVE_KERNEL_WRAPPER
