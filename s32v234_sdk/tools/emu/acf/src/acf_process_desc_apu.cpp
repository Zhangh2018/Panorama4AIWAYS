/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright (c) 2017-2018 NXP
* All Rights Reserved
*
******************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
/**
* \file APU_ACF.cpp
* \brief APU ACF process description simulation
* \author Igor Aleksandrowicz
* \version
* \date 12-February-2014
****************************************************************************/

#include <stdio.h>
#include "acf_process_desc_apu.hpp"

using namespace APEX2;

#if __GNUG__
#define sprintf_s(buffer, str, ...) sprintf(buffer, str, ##__VA_ARGS__)
#endif

/****************************************************************************
* functions
****************************************************************************/

icp::DATATYPE convertdata2icp(int cvtype)
{
	cvtype &= 0x00000007;
	switch (cvtype)
	{
	case VSDK_CV_8U:
		return icp::DATATYPE_08U;
	case VSDK_CV_8S:
		return icp::DATATYPE_08S;
	case VSDK_CV_16U:
		return icp::DATATYPE_16U;
	case VSDK_CV_16S:
		return icp::DATATYPE_16S;
	case VSDK_CV_32S:
		return icp::DATATYPE_32S;
	default:
		return icp::DATATYPE_32U;
	}
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

ACF_DataDesc TranslateVsdkUMatToAcfDataDesc(vsdk::SUMat& lumat)
{
	//TODO: adjust pointers to account for 'dimensions'!!!

	int32_t lSpan = lumat.step[0] * lumat.step[2];

	ACF_DataDesc lAcfDataDesc((uint8_t *)lumat.u->handle + lumat.offset,
		(uint8_t *)lumat.u->handle + lumat.offset,
		lumat.cols,
		lumat.rows,
		lSpan,
		convertdata2icp(lumat.type()),
		lumat.channels(),
		lumat.step[2],
		0,
		0,
		0,
		0);
	return lAcfDataDesc;
}

ACF_DataDesc TranslateVsdkUMatToAcfDataDesc(vsdk::SUMat& lumatOrig, int32_t lRoiInfoL, int32_t lRoiInfoR, int32_t lRoiInfoT, int32_t lRoiInfoB)
{
	//TODO: adjust pointers to account for 'dimensions'!!!
	vsdk::SUMat lumat(lumatOrig, vsdk::Rect(lRoiInfoL, lRoiInfoT, lRoiInfoR - lRoiInfoL, lRoiInfoB - lRoiInfoT));

	int32_t lSpan = lumat.step[0] * lumat.step[2];

	ACF_DataDesc lAcfDataDesc((uint8_t *)lumat.u->handle + lumat.offset,
		(uint8_t *)lumat.u->handle + lumat.offset,
		lumat.cols,
		lumat.rows,
		lSpan,
		convertdata2icp(lumat.type()),
		lumat.channels(),
		lumat.step[2],
		lRoiInfoL,
		lumat.cols-lRoiInfoR,
		lRoiInfoT,
		lumat.rows-lRoiInfoB);
	return lAcfDataDesc;
}

void ACF_Process_Desc_APU::Initialize(ACF_Graph& graph, const std::string& name)
{
  if (errorOccured)
  {
    return;
  }

  if (_graphUsed != 0)
  {
    errorOccured = true;
    errorMessage = "the process description is already initialized";
    return;
  }

  _graphUsed = &graph;
  this->name = name;
  _graphUsed->Create();
  if (!CheckGraph(true))
  {
    return;
  }

}

void ACF_Process_Desc_APU::SetInputChunkSize(const std::string& portName, int chunkWidth, int chunkHeight)
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return;
  }

  _graphUsed->SetGraphPortTileSize(portName, chunkWidth, chunkHeight);

  //report as a process error rather than a graph error
  CheckGraph(false);
}

void ACF_Process_Desc_APU::CheckCommonRuntimeErrors()
{
  if (errorOccured)
  {
    return;
  }

  if (_graphUsed == 0)
  {
    errorOccured = true;
    errorMessage = "no graph assigned to the process";
    return;
  }
}

bool ACF_Process_Desc_APU::CheckGraph(bool reportAsGraphError)
{
  if (_graphUsed->ErrorOccured())
  {
    errorOccured = true;
    errorMessage = reportAsGraphError ? "graph error: " + _graphUsed->ErrorMessage() : _graphUsed->ErrorMessage();
    return false;
  }

  return true;
}

int ACF_Process_Desc_APU::Initialize()
{
  Create();
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  //can be computed only once
  _graphUsed->MakeSerialPlan();
  _graphUsed->ComputeChunkSizes();
  if (!CheckGraph(true))
  {
    return 1;
  }

  return 0;
}

int ACF_Process_Desc_APU::ConnectIO(const std::string ioName, vsdk::SUMat& lUMat)
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  ACF_DataDesc *acfdd = new ACF_DataDesc(TranslateVsdkUMatToAcfDataDesc(lUMat));
  _graphUsed->ConnectData(ioName, acfdd);
  //report as a process error rather than a graph error
  if (!CheckGraph(false))
  {
    return 1;
  }

  return 0;
}

int ACF_Process_Desc_APU::ConnectIO_ROI(std::string lPortIdentifier,
	vsdk::SUMat& lUMat,
	int ROI_left,
	int ROI_top,
	int ROI_width,
	int ROI_height)
{
  int lRetVal = 0;
	int lRoiInfoR = ROI_left + ROI_width; // verify that the right and bottom are inside the image
	int lRoiInfoB = ROI_top + ROI_height;
	if (ROI_left >= 0          && ROI_top >= 0 &&
      lRoiInfoR <= lUMat.cols && lRoiInfoB <= lUMat.rows)
  {
		ACF_DataDesc *acfdd = new ACF_DataDesc(TranslateVsdkUMatToAcfDataDesc(lUMat, ROI_left, lRoiInfoR, ROI_top, lRoiInfoB));

		CheckCommonRuntimeErrors();
		if (errorOccured)
		{
			return 1;
		}

		_graphUsed->ConnectData(lPortIdentifier, acfdd);
		//report as a process error rather than a graph error
		if (!CheckGraph(false))
		{
			return 1;
		}
	}else{
		char str[150];
		sprintf_s(str
			, "Graph %s Port %s : Roi (left %d,top %d, rgt %d, bot %d) is not inside image (width %d, height %d)"
			, RetProcessIdentifier().c_str(), lPortIdentifier.c_str()
			, ROI_left, ROI_top, lRoiInfoR, lRoiInfoB
			, lUMat.cols, lUMat.rows);
		errorMessage = std::string(str);
		lRetVal = 1;
	}
	return lRetVal;

}


int ACF_Process_Desc_APU::ConnectIndirectInput(const std::string& ioName, vsdk::SUMat& dataDescriptor, vsdk::SUMat& offsetDescriptor)
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  ACF_DataDesc *acfdd = new ACF_DataDesc(TranslateVsdkUMatToAcfDataDesc(dataDescriptor));
  ACF_DataDesc *acfoffdd = new ACF_DataDesc(TranslateVsdkUMatToAcfDataDesc(offsetDescriptor));

  _graphUsed->ConnectIndirectData(ioName, acfdd, acfoffdd);
  //report as a process error rather than a graph error
  if (!CheckGraph(false))
  {
    return 1;
  }

  return 0;
}

int ACF_Process_Desc_APU::FlagInputAsChunkBasedIndirect(const std::string& ioName)
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  _graphUsed->FlagIndirect(ioName);
  //report as a process error rather than a graph error
  if (!CheckGraph(false))
  {
    return 1;
  }

  return 0;
}


int ACF_Process_Desc_APU::Start()
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  //have to be computed for each run (PrepareDataPlaces could be updated only when connecting other data)
  _graphUsed->PrepareDataPlaces();
  _graphUsed->Run();
  if (!CheckGraph(false))
  {
    return 1;
  }

  return 0;
}



int ACF_Process_Desc_APU::Start_Prepare() {
	CheckCommonRuntimeErrors();
	if (errorOccured)
	{
		return 1;
	}

	//have to be computed for each run (PrepareDataPlaces could be updated only when connecting other data)
	_graphUsed->PrepareDataPlaces();


	return 0;
}

int ACF_Process_Desc_APU::Start_Run(){
	CheckCommonRuntimeErrors();
	if (errorOccured)
	{
		return 1;
	}

	_graphUsed->Run();
	if (!CheckGraph(false))
	{
		return 1;
	}

	return 0;
}

int ACF_Process_Desc_APU::Wait()
{
  //this function doesn't really do anything except checking the errors, included for ACF compatibility reasons
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return 1;
  }

  return 0;
}

std::string ACF_Process_Desc_APU::GetExecutionPlanDescription()
{
  CheckCommonRuntimeErrors();
  if (errorOccured)
  {
    return "no plan due to errors\n";
  }

  return _graphUsed->GetPlanDescription();
}

ACF_Graph::DataPlace* ACF_Process_Desc_APU::GetDataPlacePtr(const std::string& kernelInstanceName, const std::string& portName)
{
  if (_graphUsed == 0)
  {
    return 0;
  }
  return _graphUsed->GetDataPlacePtr(kernelInstanceName, portName);
}

bool ACF_Process_Desc_APU::WarningOccured()
{
  return _graphUsed->WarningOccured() || warningOccured;
}

std::string ACF_Process_Desc_APU::WarningMessages()
{
  std::string ret;
  if (_graphUsed->WarningOccured())
  {
    ret += _graphUsed->WarningMessages();
  }
  ret += warningMessages;
  return ret;
}
