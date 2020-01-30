#ifndef _ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR_NATIVE_KERNEL_WRAPPER

#include <COMPUTE_BRIEF_DESCRIPTOR.hpp>

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
   [0] corresponds to indirect input 'INPUT' source
   [1] corresponds to indirect input 'INPUT' offset
   [2] corresponds to input 'FILTER_TYPE'
   [3] corresponds to input 'SMPL_PACKET'
   [4] corresponds to input 'NR_PACKETS_UL'
   [5] corresponds to input 'NR_PACKETS_UR'
   [6] corresponds to input 'NR_PACKETS_LL'
   [7] corresponds to input 'NR_PACKETS_LR'
   [8] corresponds to input 'PATCH_SIZE'
   [9] corresponds to input 'DESC_SIZE'
   [10] corresponds to output 'COUNT'
   [11] corresponds to output 'DESCRIPTOR_OUT'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void COMPUTE_BRIEF_DESCRIPTOR_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //INPUT
   ACF_DataDesc INPUT_src;
   ACF_DataDesc INPUT_offset;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   INPUT_src.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   INPUT_offset.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //FILTER_TYPE
   ACF_DataDesc FILTER_TYPE;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   FILTER_TYPE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //SMPL_PACKET
   ACF_DataDesc SMPL_PACKET;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   SMPL_PACKET.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //NR_PACKETS_UL
   ACF_DataDesc NR_PACKETS_UL;
   lpIo = (acf_io_info*)((int32_t)pArgs+4*sizeof(acf_io_info));
   NR_PACKETS_UL.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //NR_PACKETS_UR
   ACF_DataDesc NR_PACKETS_UR;
   lpIo = (acf_io_info*)((int32_t)pArgs+5*sizeof(acf_io_info));
   NR_PACKETS_UR.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //NR_PACKETS_LL
   ACF_DataDesc NR_PACKETS_LL;
   lpIo = (acf_io_info*)((int32_t)pArgs+6*sizeof(acf_io_info));
   NR_PACKETS_LL.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //NR_PACKETS_LR
   ACF_DataDesc NR_PACKETS_LR;
   lpIo = (acf_io_info*)((int32_t)pArgs+7*sizeof(acf_io_info));
   NR_PACKETS_LR.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //PATCH_SIZE
   ACF_DataDesc PATCH_SIZE;
   lpIo = (acf_io_info*)((int32_t)pArgs+8*sizeof(acf_io_info));
   PATCH_SIZE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DESC_SIZE
   ACF_DataDesc DESC_SIZE;
   lpIo = (acf_io_info*)((int32_t)pArgs+9*sizeof(acf_io_info));
   DESC_SIZE.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //COUNT
   ACF_DataDesc COUNT;
   lpIo = (acf_io_info*)((int32_t)pArgs+10*sizeof(acf_io_info));
   COUNT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //DESCRIPTOR_OUT
   ACF_DataDesc DESCRIPTOR_OUT;
   lpIo = (acf_io_info*)((int32_t)pArgs+11*sizeof(acf_io_info));
   DESCRIPTOR_OUT.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   COMPUTE_BRIEF_DESCRIPTOR lProcess;
   lProcess.Initialize();
   lProcess.ConnectIndirectInput("INPUT", INPUT_src, INPUT_offset);
   lProcess.ConnectIO("FILTER_TYPE", FILTER_TYPE);
   lProcess.ConnectIO("SMPL_PACKET", SMPL_PACKET);
   lProcess.ConnectIO("NR_PACKETS_UL", NR_PACKETS_UL);
   lProcess.ConnectIO("NR_PACKETS_UR", NR_PACKETS_UR);
   lProcess.ConnectIO("NR_PACKETS_LL", NR_PACKETS_LL);
   lProcess.ConnectIO("NR_PACKETS_LR", NR_PACKETS_LR);
   lProcess.ConnectIO("PATCH_SIZE", PATCH_SIZE);
   lProcess.ConnectIO("DESC_SIZE", DESC_SIZE);
   lProcess.ConnectIO("COUNT", COUNT);
   lProcess.ConnectIO("DESCRIPTOR_OUT", DESCRIPTOR_OUT);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_COMPUTE_BRIEF_DESCRIPTOR_NATIVE_KERNEL_WRAPPER
