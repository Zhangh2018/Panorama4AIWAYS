/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#include <umat.hpp>
#include <acf_process.h>
#include <stdint.h>
#include <icp_data.h>
#include <icp_datatype.h>

#ifdef __STANDALONE__
	#include <printf.h>
#endif

using namespace std;

#define ACF_WAIT_TIMEOUT_IN_US 1000000 //1s

ACF_Process::ACF_Process()
{
   //CGV_REPORT_LOG("APEX_ACF_process_create");
   mProcessStartedFlag = 0;
   mErrorState = 0;
   mRoiInfoL = 0;
   mRoiInfoR = 0;
   mRoiInfoT = 0;
   mRoiInfoB = 0;
   mRoiInfoManualFlag = 0;
   mInitializedFlag = 0;
   mSpatialDepFlag = 0;
   mIoSizeChangeFlag = 0;
   mWaitTimeoutInUs = ACF_WAIT_TIMEOUT_IN_US;

   mSkipPortConsistencyCheck = 0;
}

ACF_Process::~ACF_Process()
{
}

ACF_DataDesc TranslateIcpDataDescToAcfDataDesc(icp::DataDescriptor& lDataDesc)
{
   //TODO: adjust pointers to account for 'dimensions'!!!
   
   ACF_DataDesc lAcfDataDesc (lDataDesc.GetDataPtr(),
                              lDataDesc.GetDataPtrPhys(),
                              lDataDesc.GetWidth(),
                              lDataDesc.GetHeight(),
                              lDataDesc.GetSpanRow(),
                              lDataDesc.GetElementDataType(),
                              lDataDesc.GetElementDimX(),
                              lDataDesc.GetElementDimY(),
                              lDataDesc.GetRoiInfoL(),
                              lDataDesc.GetRoiInfoR(),
                              lDataDesc.GetRoiInfoT(),
                              lDataDesc.GetRoiInfoB());

   return lAcfDataDesc;
}

icp::DATATYPE convertdata2icp(unsigned int cvtype)
{
  cvtype &= 0x00000007;
  
  if (cvtype == VSDK_CV_8U)
  {
    return icp::DATATYPE_08U;
  }
  
  if (cvtype == VSDK_CV_8S)
  {
    return icp::DATATYPE_08S;
  }
  
  if (cvtype == VSDK_CV_16U)
  {
    return icp::DATATYPE_16U;
  }
  
  if (cvtype == VSDK_CV_16S)
  {
    return icp::DATATYPE_16S;
  }
  
  if (cvtype == VSDK_CV_32S)
  {
    return icp::DATATYPE_32S;
  }
  
  return icp::DATATYPE_32U;
}

int convertdata2umat(icp::DATATYPE cvtype)
{
  switch (cvtype)
  {
    case icp::DATATYPE_08U:
      return VSDK_CV_8U;
    case icp::DATATYPE_08S:
      return VSDK_CV_8S;
    case icp::DATATYPE_16U:
      return VSDK_CV_16U;
    case icp::DATATYPE_16S:
      return VSDK_CV_16S;
    case icp::DATATYPE_32S:
      return VSDK_CV_32S;
    default:
      return 0;
  }
}

ACF_DataDesc TranslateVsdkUMatToAcfDataDesc(const vsdk::SUMat& lUmat)
{
   //TODO: adjust pointers to account for 'dimensions'!!!
   
   ACF_DataDesc lAcfDataDesc ((0 == lUmat.u->data) ? 0 : (lUmat.u->data + lUmat.offset),
                              (uint8_t *)lUmat.u->handle + lUmat.offset,
                              lUmat.cols,
                              lUmat.rows,
                              lUmat.step[0],
                              convertdata2icp(lUmat.type()),
                              lUmat.channels(),
                              lUmat.step[2],
                              0,
                              0,
                              0,
                              0);
   return lAcfDataDesc;
}

int32_t ACF_Process::ConnectIO(std::string lPortIdentifier, const ACF_DataDesc& lDataDesc)
{
   //CGV_REPORT_LOG("APEX_ACF_process_connect_io");

   int32_t lRetVal = 0;

   ACF_Process_PortInfo* lpPortInfo = RetPortInfo(lPortIdentifier);

   if (lpPortInfo != 0) //if port was found
   {
      if (lpPortInfo->IsDirect())
      {
         //if the port was previously configured AND the new IO has an updated width or height, set a flag
         if ((1 == lpPortInfo->mConfiguredFlag) &&
             (lpPortInfo->mDataDesc.RetWidth() != lDataDesc.RetWidth() || lpPortInfo->mDataDesc.RetHeight() != lDataDesc.RetHeight()))
         {
            mIoSizeChangeFlag = 1;
         }

         lpPortInfo->mDataDesc = lDataDesc;

         e0_desc lE0DescIo = RetE0Desc(lpPortInfo->mDataDesc.RetElementDataType(),
                                       lpPortInfo->mDataDesc.RetElementDimX(),
                                       lpPortInfo->mDataDesc.RetElementDimY());

         if (E0Compare(&lE0DescIo, &lpPortInfo->mE0))
         {
            lpPortInfo->mConfiguredFlag = 1;
         }
         else
         {
           
           printf("%d, %d, %d\n", lpPortInfo->mE0.dataType, lpPortInfo->mE0.x, lpPortInfo->mE0.y);
           printf("%d, %d, %d\n", lE0DescIo.dataType, lE0DescIo.x,lE0DescIo.y);
            printf("ACF_Process::ConnectIO(%s, ... ) -> e0 mismatch between port and data\n", lPortIdentifier.c_str());
            lRetVal = 1;
            mErrorState = 1;
         }
      }
      else
      {
         printf("ACF_Process::ConnectIO(%s, ... ) -> this port has been flagged for indirect input\n", lPortIdentifier.c_str());
         lRetVal = 1;
         mErrorState = 1;
      }
   }
   else //port couldn't be found
   {
      printf("ACF_Process::ConnectIO(%s, ... ) -> port could not be found\n", lPortIdentifier.c_str());
      lRetVal = 1;
      mErrorState = 1;
   }

   return lRetVal;
}

int32_t ACF_Process::ConnectIO_ROI(std::string lPortIdentifier,
                                   const ACF_DataDesc& lDataDesc,
                                   int32_t lROI_XOffset,
                                   int32_t lROI_YOffset,
                                   int32_t lROI_Width,
                                   int32_t lROI_Height)
{
   int32_t lRetVal = 0;
   ACF_DataDesc lDescRoi;
   
   if (0 == lDescRoi.Init2D_ROI(lDataDesc, lROI_XOffset, lROI_YOffset, lROI_Width, lROI_Height))
   {
      lRetVal = ConnectIO(lPortIdentifier, lDescRoi);
   }
   else
   {
      lRetVal = 1;
      mErrorState = 1;
   }

   return lRetVal;
}

int32_t ACF_Process::ConnectIndirectInput(std::string lPortIdentifier,
                                          const ACF_DataDesc& lSrcData,
                                          const ACF_DataDesc& lChunkOffsetArray)
{
   //CGV_REPORT_LOG("APEX_ACF_process_connect_indirect_input");

   int32_t lRetVal = 0;

   ACF_Process_PortInfo* lpPortInfo = RetPortInfo(lPortIdentifier);

   if (lpPortInfo != 0) //if port was found
   {
      if (lpPortInfo->IsIndirect())
      {
         //if the port was previously configured AND the new IO has an updated width or height, set a flag
         //(only lChunkOffsetArray is considered here since it impacts the 'shape' of the data)
         if ((1 == lpPortInfo->mConfiguredFlag) &&
             (lpPortInfo->mDataDesc.RetWidth() != lChunkOffsetArray.RetWidth() || lpPortInfo->mDataDesc.RetHeight() != lChunkOffsetArray.RetHeight()))
         {
            mIoSizeChangeFlag = 1;
         }

         lpPortInfo->mDataDesc = lSrcData;

         e0_desc lE0DescIo = RetE0Desc(lpPortInfo->mDataDesc.RetElementDataType(),
                                       lpPortInfo->mDataDesc.RetElementDimX(),
                                       lpPortInfo->mDataDesc.RetElementDimY());

         if (E0Compare(&lE0DescIo, &lpPortInfo->mE0))
         {
            if ((lChunkOffsetArray.RetWidth()&0x3) == 0) //HW LIMITATION: number of chunk offset pointers must be a multiple of 4 in the horizontal direction (GOC)
            {
               lpPortInfo->mOffsetArray    = lChunkOffsetArray;
               lpPortInfo->mConfiguredFlag = 1;
            }
            else //width of chunk offset array is NOT a multiple of 4
            {
               printf("ACF_Process::ConnectIndirectInput(%s, ... ) -> width of chunk offset array must be a multiple of 4 (i.e. # of pointers in the horizontal dimension must be a multiple of 4)\n", lPortIdentifier.c_str());
               lRetVal = 1;
               mErrorState = 1;
            }
         }
         else //e0 mismatch
         {
            printf("ACF_Process::ConnectIndirectInput(%s, ... ) -> e0 mismatch between port and source data\n", lPortIdentifier.c_str());
            lRetVal = 1;
            mErrorState = 1;
         }
      }
      else
      {
         printf("ACF_Process::ConnectIndirectInput(%s, ... ) -> this port has not been flagged for indirect input\n", lPortIdentifier.c_str());
         lRetVal = 1;
         mErrorState = 1;
      }
   }
   else //port couldn't be found
   {
      printf("ACF_Process::ConnectIndirectInput(%s, ... ) -> port could not be found\n", lPortIdentifier.c_str());
      lRetVal = 1;
      mErrorState = 1;
   }

   return lRetVal;
}


int32_t ACF_Process::ConnectIO(std::string lPortIdentifier, icp::DataDescriptor& lDataDesc)
{
   ACF_DataDesc lAcfDataDesc = TranslateIcpDataDescToAcfDataDesc(lDataDesc);

   return ConnectIO(lPortIdentifier, lAcfDataDesc);
}

int32_t ACF_Process::ConnectIO(std::string lPortIdentifier, const vsdk::SUMat& lUmat)
{
   ACF_DataDesc lAcfDataDesc = TranslateVsdkUMatToAcfDataDesc(lUmat);

   return ConnectIO(lPortIdentifier, lAcfDataDesc);
}

int32_t ACF_Process::ConnectIO_ROI(std::string lPortIdentifier,
                                   icp::DataDescriptor& lDataDesc,
                                   int32_t ROI_left,
                                   int32_t ROI_top,
                                   int32_t ROI_width,
                                   int32_t ROI_height)
{
   ACF_DataDesc lAcfDataDesc = TranslateIcpDataDescToAcfDataDesc(lDataDesc);

   return ConnectIO_ROI(lPortIdentifier,
                        lAcfDataDesc,
                        ROI_left,
                        ROI_top,
                        ROI_width,
                        ROI_height);
}

int32_t ACF_Process::ConnectIO_ROI(std::string lPortIdentifier,
                                   const vsdk::SUMat& lUmat,
                                   int32_t ROI_left,
                                   int32_t ROI_top,
                                   int32_t ROI_width,
                                   int32_t ROI_height)
{
   ACF_DataDesc lAcfDataDesc = TranslateVsdkUMatToAcfDataDesc(lUmat);

   return ConnectIO_ROI(lPortIdentifier,
                        lAcfDataDesc,
                        ROI_left,
                        ROI_top,
                        ROI_width,
                        ROI_height);
}

int32_t ACF_Process::ConnectIndirectInput(std::string lPortIdentifier,
                                          icp::DataDescriptor& lSrcData,
                                          icp::DataDescriptor& lChunkOffsetArray)
{
   ACF_DataDesc lAcfDataDesc_srcdata = TranslateIcpDataDescToAcfDataDesc(lSrcData);
   ACF_DataDesc lAcfDataDesc_chunkoffsetarray = TranslateIcpDataDescToAcfDataDesc(lChunkOffsetArray);

   return ConnectIndirectInput(lPortIdentifier, lAcfDataDesc_srcdata, lAcfDataDesc_chunkoffsetarray);
}

int32_t ACF_Process::ConnectIndirectInput(std::string lPortIdentifier,
                                          const vsdk::SUMat& lSrcData,
                                          const vsdk::SUMat& lChunkOffsetArray)
{
   ACF_DataDesc lAcfDataDesc_srcdata = TranslateVsdkUMatToAcfDataDesc(lSrcData);
   ACF_DataDesc lAcfDataDesc_chunkoffsetarray = TranslateVsdkUMatToAcfDataDesc(lChunkOffsetArray);

   return ConnectIndirectInput(lPortIdentifier, lAcfDataDesc_srcdata, lAcfDataDesc_chunkoffsetarray);
}


void ACF_Process::SetProcessIdentifier(std::string lProcessIdentifier)
{
   mProcessIdentifier = lProcessIdentifier;
}

std::string ACF_Process::RetProcessIdentifier()
{
   return mProcessIdentifier;
}

void ACF_Process::AddPort(std::string lPortIdentifier,
                          icp::DATATYPE lE0DataType, int32_t lE0x, int32_t lE0y,
                          int32_t lChunkWidth, int32_t lChunkHeight,
                          int32_t lInputOutput,
                          int32_t lNonstaticStatic,
                          int32_t lDirectIndirect,
                          int32_t lNonfixedFixed,
                          int32_t lRegFifo,
                          int32_t lLmIndex,
                          int32_t lSDInfoN,
                          int32_t lSDInfoS,
                          int32_t lSDInfoE,
                          int32_t lSDInfoW,
                          int32_t lVerDepOverlapFlag)
{
   ACF_Process_PortInfo lPortInfo;

   lPortInfo.mPortIdentifier    = lPortIdentifier;
   lPortInfo.mE0.dataType       = lE0DataType;
   lPortInfo.mE0.x              = lE0x;
   lPortInfo.mE0.y              = lE0y;
   lPortInfo.mChunkWidth        = lChunkWidth;
   lPortInfo.mChunkHeight       = lChunkHeight;
   lPortInfo.mInputOutput       = lInputOutput;
   lPortInfo.mNonstaticStatic   = lNonstaticStatic;
   lPortInfo.mDirectIndirect    = lDirectIndirect;
   lPortInfo.mNonfixedFixed     = lNonfixedFixed;
   lPortInfo.mRegFifo           = lRegFifo;
   lPortInfo.mLmIndex           = lLmIndex;
   lPortInfo.mSDInfoN           = lSDInfoN;
   lPortInfo.mSDInfoS           = lSDInfoS;
   lPortInfo.mSDInfoE           = lSDInfoE;
   lPortInfo.mSDInfoW           = lSDInfoW;
   lPortInfo.mVerDepOverlapFlag = lVerDepOverlapFlag;

   mPortInfoVec.push_back(lPortInfo);
}

ACF_Process_PortInfo* ACF_Process::RetPortInfo(std::string lPortIdentifier)
{
   ACF_Process_PortInfo* lpPortInfo = 0;

   for (int32_t i=0; i<(int32_t)mPortInfoVec.size(); i++)
   {
      if (mPortInfoVec[i].mPortIdentifier == lPortIdentifier)
      {
         lpPortInfo = &mPortInfoVec[i];
         break;
      }
   }

   return lpPortInfo;
}

int32_t ACF_Process::SetRoiInfo(int32_t lRoiInfoL, int32_t lRoiInfoR, int32_t lRoiInfoT, int32_t lRoiInfoB)
{
   mRoiInfoL = lRoiInfoL;
   mRoiInfoR = lRoiInfoR;
   mRoiInfoT = lRoiInfoT;
   mRoiInfoB = lRoiInfoB;

   mRoiInfoManualFlag = 1;

   return 0;
}

void ACF_Process::CfgWaitTimeout(int32_t lTimeoutInUs)
{
   mWaitTimeoutInUs = lTimeoutInUs;
}
