#ifndef _ACF_PROCESS_APU_HOUGH_20X2_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_HOUGH_20X2_NATIVE_KERNEL_WRAPPER

#include <HOUGH_20X2.hpp>

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

acf_io_info MYKERNEL_args[11];

where:
   [0] corresponds to input 'IMAGE'
   [1] corresponds to input 'PIXEL_THRESHOLD'
   [2] corresponds to input 'PIXEL_OFFSET_PARAMS'
   [3] corresponds to input 'CU_COUNT'
   [4] corresponds to input 'COS_TABLE'
   [5] corresponds to input 'SIN_TABLE'
   [6] corresponds to input 'RHO_COUNT'
   [7] corresponds to input 'CU_INDEX_PARAMS'
   [8] corresponds to input 'LINE_THRESHOLD_PARAMS'
   [9] corresponds to output 'LINES'
   [10] corresponds to output 'LINE_COUNT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void HOUGH_20X2_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //IMAGE
   ACF_DataDesc IMAGE;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   IMAGE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PIXEL_THRESHOLD
   ACF_DataDesc PIXEL_THRESHOLD;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   PIXEL_THRESHOLD.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PIXEL_OFFSET_PARAMS
   ACF_DataDesc PIXEL_OFFSET_PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   PIXEL_OFFSET_PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //CU_COUNT
   ACF_DataDesc CU_COUNT;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   CU_COUNT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //COS_TABLE
   ACF_DataDesc COS_TABLE;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   COS_TABLE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SIN_TABLE
   ACF_DataDesc SIN_TABLE;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   SIN_TABLE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //RHO_COUNT
   ACF_DataDesc RHO_COUNT;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   RHO_COUNT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //CU_INDEX_PARAMS
   ACF_DataDesc CU_INDEX_PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   CU_INDEX_PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //LINE_THRESHOLD_PARAMS
   ACF_DataDesc LINE_THRESHOLD_PARAMS;
   lpIo = (acf_io_info*)((int32_t)pArgs+8*sizeof(acf_io_info));
   LINE_THRESHOLD_PARAMS.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //LINES
   ACF_DataDesc LINES;
   lpIo = (acf_io_info*)((int32_t)pArgs+9*sizeof(acf_io_info));
   LINES.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //LINE_COUNT
   ACF_DataDesc LINE_COUNT;
   lpIo = (acf_io_info*)((int32_t)pArgs+10*sizeof(acf_io_info));
   LINE_COUNT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   HOUGH_20X2 lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("IMAGE", IMAGE);
   lProcess.ConnectIO("PIXEL_THRESHOLD", PIXEL_THRESHOLD);
   lProcess.ConnectIO("PIXEL_OFFSET_PARAMS", PIXEL_OFFSET_PARAMS);
   lProcess.ConnectIO("CU_COUNT", CU_COUNT);
   lProcess.ConnectIO("COS_TABLE", COS_TABLE);
   lProcess.ConnectIO("SIN_TABLE", SIN_TABLE);
   lProcess.ConnectIO("RHO_COUNT", RHO_COUNT);
   lProcess.ConnectIO("CU_INDEX_PARAMS", CU_INDEX_PARAMS);
   lProcess.ConnectIO("LINE_THRESHOLD_PARAMS", LINE_THRESHOLD_PARAMS);
   lProcess.ConnectIO("LINES", LINES);
   lProcess.ConnectIO("LINE_COUNT", LINE_COUNT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_HOUGH_20X2_NATIVE_KERNEL_WRAPPER
