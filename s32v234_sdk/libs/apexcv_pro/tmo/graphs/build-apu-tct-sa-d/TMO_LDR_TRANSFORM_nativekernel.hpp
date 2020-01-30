#ifndef _ACF_PROCESS_APU_TMO_LDR_TRANSFORM_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_TMO_LDR_TRANSFORM_NATIVE_KERNEL_WRAPPER

#include <TMO_LDR_TRANSFORM.hpp>

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

acf_io_info MYKERNEL_args[8];

where:
   [0] corresponds to input 'INTERMEDIATE_EXP'
   [1] corresponds to input 'INTERMEDIATE_MAN'
   [2] corresponds to input 'WORLD_EXP'
   [3] corresponds to input 'WORLD_MAN'
   [4] corresponds to input 'GEO_MEAN_EXP'
   [5] corresponds to input 'DIV_VALUE'
   [6] corresponds to input 'KEY_VALUE'
   [7] corresponds to output 'OUTPUT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void TMO_LDR_TRANSFORM_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INTERMEDIATE_EXP
   ACF_DataDesc INTERMEDIATE_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INTERMEDIATE_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INTERMEDIATE_MAN
   ACF_DataDesc INTERMEDIATE_MAN;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INTERMEDIATE_MAN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //WORLD_EXP
   ACF_DataDesc WORLD_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   WORLD_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //WORLD_MAN
   ACF_DataDesc WORLD_MAN;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   WORLD_MAN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //GEO_MEAN_EXP
   ACF_DataDesc GEO_MEAN_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   GEO_MEAN_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DIV_VALUE
   ACF_DataDesc DIV_VALUE;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   DIV_VALUE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //KEY_VALUE
   ACF_DataDesc KEY_VALUE;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   KEY_VALUE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //OUTPUT
   ACF_DataDesc OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   TMO_LDR_TRANSFORM lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INTERMEDIATE_EXP", INTERMEDIATE_EXP);
   lProcess.ConnectIO("INTERMEDIATE_MAN", INTERMEDIATE_MAN);
   lProcess.ConnectIO("WORLD_EXP", WORLD_EXP);
   lProcess.ConnectIO("WORLD_MAN", WORLD_MAN);
   lProcess.ConnectIO("GEO_MEAN_EXP", GEO_MEAN_EXP);
   lProcess.ConnectIO("DIV_VALUE", DIV_VALUE);
   lProcess.ConnectIO("KEY_VALUE", KEY_VALUE);
   lProcess.ConnectIO("OUTPUT", OUTPUT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_TMO_LDR_TRANSFORM_NATIVE_KERNEL_WRAPPER
