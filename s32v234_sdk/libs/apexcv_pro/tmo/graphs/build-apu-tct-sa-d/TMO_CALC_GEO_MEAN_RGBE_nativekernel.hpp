#ifndef _ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_RGBE_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_RGBE_NATIVE_KERNEL_WRAPPER

#include <TMO_CALC_GEO_MEAN_RGBE.hpp>

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

acf_io_info MYKERNEL_args[7];

where:
   [0] corresponds to input 'INPUT'
   [1] corresponds to output 'INTERMEDIATE_EXP'
   [2] corresponds to output 'INTERMEDIATE_MAN'
   [3] corresponds to output 'WORLD_EXP'
   [4] corresponds to output 'WORLD_MAN'
   [5] corresponds to output 'GEO_MEAN_EXP'
   [6] corresponds to output 'GEO_MEAN_MAN'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void TMO_CALC_GEO_MEAN_RGBE_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INTERMEDIATE_EXP
   ACF_DataDesc INTERMEDIATE_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INTERMEDIATE_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //INTERMEDIATE_MAN
   ACF_DataDesc INTERMEDIATE_MAN;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   INTERMEDIATE_MAN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //WORLD_EXP
   ACF_DataDesc WORLD_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   WORLD_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //WORLD_MAN
   ACF_DataDesc WORLD_MAN;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   WORLD_MAN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //GEO_MEAN_EXP
   ACF_DataDesc GEO_MEAN_EXP;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   GEO_MEAN_EXP.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //GEO_MEAN_MAN
   ACF_DataDesc GEO_MEAN_MAN;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   GEO_MEAN_MAN.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   TMO_CALC_GEO_MEAN_RGBE lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("INPUT", INPUT);
   lProcess.ConnectIO("INTERMEDIATE_EXP", INTERMEDIATE_EXP);
   lProcess.ConnectIO("INTERMEDIATE_MAN", INTERMEDIATE_MAN);
   lProcess.ConnectIO("WORLD_EXP", WORLD_EXP);
   lProcess.ConnectIO("WORLD_MAN", WORLD_MAN);
   lProcess.ConnectIO("GEO_MEAN_EXP", GEO_MEAN_EXP);
   lProcess.ConnectIO("GEO_MEAN_MAN", GEO_MEAN_MAN);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_TMO_CALC_GEO_MEAN_RGBE_NATIVE_KERNEL_WRAPPER
