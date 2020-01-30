#ifndef _ACF_PROCESS_APU_ELTMULCRED_MODULE_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_ELTMULCRED_MODULE_NATIVE_KERNEL_WRAPPER

#include <ELTMULCRED_MODULE.hpp>

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
   [0] corresponds to input 'ELTMULCRED_MODULE_INPUT_IMAGE'
   [1] corresponds to input 'ELTMULCRED_MODULE_INPUT_WEIGHT'
   [2] corresponds to input 'ELTMULCRED_MODULE_INPUT_BIAS'
   [3] corresponds to input 'ELTMULCRED_MODULE_INPUT_PARAMS'
   [4] corresponds to output 'ELTMULCRED_MODULE_OUTPUT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void ELTMULCRED_MODULE_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //ELTMULCRED_MODULE_INPUT_IMAGE
   ACF_DataDesc ELTMULCRED_MODULE_INPUT_IMAGE;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   ELTMULCRED_MODULE_INPUT_IMAGE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //ELTMULCRED_MODULE_INPUT_WEIGHT
   ACF_DataDesc ELTMULCRED_MODULE_INPUT_WEIGHT;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   ELTMULCRED_MODULE_INPUT_WEIGHT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //ELTMULCRED_MODULE_INPUT_BIAS
   ACF_DataDesc ELTMULCRED_MODULE_INPUT_BIAS;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   ELTMULCRED_MODULE_INPUT_BIAS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //ELTMULCRED_MODULE_INPUT_PARAMS
   ACF_DataDesc ELTMULCRED_MODULE_INPUT_PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   ELTMULCRED_MODULE_INPUT_PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //ELTMULCRED_MODULE_OUTPUT
   ACF_DataDesc ELTMULCRED_MODULE_OUTPUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   ELTMULCRED_MODULE_OUTPUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   ELTMULCRED_MODULE lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("ELTMULCRED_MODULE_INPUT_IMAGE", ELTMULCRED_MODULE_INPUT_IMAGE);
   lProcess.ConnectIO("ELTMULCRED_MODULE_INPUT_WEIGHT", ELTMULCRED_MODULE_INPUT_WEIGHT);
   lProcess.ConnectIO("ELTMULCRED_MODULE_INPUT_BIAS", ELTMULCRED_MODULE_INPUT_BIAS);
   lProcess.ConnectIO("ELTMULCRED_MODULE_INPUT_PARAMS", ELTMULCRED_MODULE_INPUT_PARAMS);
   lProcess.ConnectIO("ELTMULCRED_MODULE_OUTPUT", ELTMULCRED_MODULE_OUTPUT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_ELTMULCRED_MODULE_NATIVE_KERNEL_WRAPPER
