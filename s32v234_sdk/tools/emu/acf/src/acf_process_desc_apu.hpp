/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP
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
* \file APU_ACF.h
* \brief APU ACF process description simulation
* \author Igor Aleksandrowicz
* \version
* \date 12-February-2014
****************************************************************************/

#ifndef ACFPROCESSDESCAPU_HPP
#define ACFPROCESSDESCAPU_HPP

#include "acf_graph.hpp"
#include "sumat.hpp"

namespace APEX2

{
  typedef enum _ACF_APU_CFG
  {
    ACF_APU_CFG__DEFAULT,                 ///< APU0 with all CUs and all SMEM
    ACF_APU_CFG__APU_0_CU_0_63_SMEM_0_3,  ///< APU0 with CUs 0-63 and 128K SMEM
    ACF_APU_CFG__APU_0_CU_0_31_SMEM_0_1,  ///< APU0 with CUs 0-31 and 64K  SMEM
    ACF_APU_CFG__APU_1_CU_32_63_SMEM_2_3  ///< APU1 with CUs 32-63 and 64K  SMEM
  } ACF_APU_CFG;

  typedef struct _e0_desc
  {
    icp::DATATYPE dataType;
    uint16_t x;
    uint16_t y;
  } e0_desc;

  typedef struct _AcfProfilingInfo {
    int32_t host_start;     ///< host time (us) spent from the beginning of start() to the triggering of process execution on the APEX (this is 100% host overhead and includes apu loading, resource acquisition, etc.)
    int32_t host_wait;      ///< host time (us) spent in the wait() call (this includes time spent waiting for the process execution to complete, plus a small amount of overhead)
    int32_t apu_total;      ///< total apu time (us) (includes init, processing, idle, and all other overhead)
    int32_t apu_init;       ///< apu time (us) spent on initialization
    int32_t apu_processing; ///< apu time (us) spent on kernel execution + padding + circular buffer management
    int32_t apu_idle;       ///< apu time (us) spent waiting for data transfers to complete (if this is large, the process is likely bandwidth limited)
    int32_t apu_misc;       ///< apu time (us) spent on misc. overhead (control flow, descriptor management, etc.)
  } AcfProfilingInfo;


  // TODO: adapt to emulation. It is just copied from acf
  class ACF_Process_PortInfo
  {
  public:

    ACF_Process_PortInfo()
    {
      mConfiguredFlag = 0;

      mSDInfoN = 0;
      mSDInfoS = 0;
      mSDInfoE = 0;
      mSDInfoW = 0;

      mVerDepOverlapFlag = 0;
    }

    //this port information comes via the offline world to characterize the port
    //and allow sanity checks to be done when the user connects something
    //(e.g. does e0 of the connected image/buffer match the port?)
    std::string mPortIdentifier;
    e0_desc mE0;
    int32_t mChunkWidth;
    int32_t mChunkHeight;
    int32_t mInputOutput;     //0=input, 1=output
    int32_t mNonstaticStatic; //0=nonstatic, 1=static
    int32_t mDirectIndirect;  //0=direct, 1=indirect
    int32_t mNonfixedFixed;   //0=nonfixed, 1=fixed
    int32_t mRegFifo;         //0=regular, 1=fifo
    int32_t mLmIndex;

    //mSDInfo isn't the spatial dep of the port alone; it's the cumulative dependency that must be pulled from the source to satisfy
    //cascading filter dependencies in the ROI use case
    int32_t mSDInfoN;
    int32_t mSDInfoS;
    int32_t mSDInfoE;
    int32_t mSDInfoW;

    int32_t mVerDepOverlapFlag;

    //this information comes from the user at run-time on the host processor
    ACF_DataDesc mDataDesc;    //description of src/dst data buffer
    ACF_DataDesc mOffsetArray; //description of offset array in indirect case
    int32_t mConfiguredFlag;

    bool IsInput()
    {
      if (mInputOutput == 0) return true;
      else return false;
    }

    bool IsOutput()
    {
      if (mInputOutput == 1) return true;
      else return false;
    }

    bool IsDirect()
    {
      if (mDirectIndirect == 0) return true;
      else return false;
    }

    bool IsIndirect()
    {
      if (mDirectIndirect == 1) return true;
      else return false;
    }

    bool IsNonStatic()
    {
      if (mNonstaticStatic == 0) return true;
      else return false;
    }

    bool IsStatic()
    {
      if (mNonstaticStatic == 1) return true;
      else return false;
    }

    bool IsNonFixed()
    {
      if (mNonfixedFixed == 0) return true;
      else return false;
    }

    bool IsFixed()
    {
      if (mNonfixedFixed == 1) return true;
      else return false;
    }

    bool IsReg()
    {
      if (mRegFifo == 0) return true;
      else return false;
    }

    bool IsFifo()
    {
      if (mRegFifo == 1) return true;
      else return false;
    }
  };


//ACF process descriptor implementation
//also contains the final process functionality
class ACF_Process_Desc_APU
{
private:
  ACF_Graph* _graphUsed;
  std::string name;

  bool errorOccured;
  std::string errorMessage;
  bool warningOccured;
  std::string warningMessages;

  void CheckCommonRuntimeErrors();
  //returns false if there were graph errors
  //the error will be reported as a graph error if reportAsGraphError is true, as a process error otherwise
  bool CheckGraph(bool reportAsGraphError);
public:
  ACF_Process_Desc_APU():
    _graphUsed(0),
    errorOccured(false),
    errorMessage(""),
    warningOccured(false),
    warningMessages("")
  {
  }

  virtual ~ACF_Process_Desc_APU()
  {
  }

  std::string RetProcessIdentifier() {
	  if (_graphUsed)
		  return _graphUsed->RetGraphIdentifier();
	  else
		  return std::string();
  }

  //functions from ACF
  virtual void Create() = 0;
  void Initialize(ACF_Graph& graph, const std::string& name);


  /////////////////////////////////////////////////////////////////////////
  /// Sets a user defined chunk size to the specified graph port
  ///
  /// \param[in] graphPortName	- Graph port identifier.
  /// \param[in] chunkWidth     - set the chunk width of the specified port.
  /// \param[in] chunkHeight    - set the chunk height of the specified port.
  ///
  /////////////////////////////////////////////////////////////////////////
  void SetInputChunkSize(const std::string& portName, int chunkWidth, int chunkHeight);

  /////////////////////////////////////////////////////////////////////////
  /// Returns the chunk size of the specified graph port
  ///
  /// \param[in] graphPortName	- Graph port identifier.
  /// \param[out] chunkWidth    - returns the chunk width of the specified port.
  /// \param[out] chunkHeight    - returns the chunk height of the specified port.
  ///
  /// \return  0 if successful, non-zero if an an error occurred.
  /////////////////////////////////////////////////////////////////////////
  int QueryPortChunkSize(const std::string graphPortName, int& chunkWidth, int& chunkHeight) {
	  if (_graphUsed)
		  return _graphUsed->QueryPortChunkSize(graphPortName, chunkWidth, chunkHeight);
	  else
		  return 1;
  };

  //error checking
  bool ErrorOccured(){ return errorOccured; }
  std::string ErrorMessage(){ return errorMessage; }
  bool WarningOccured();
  //multiple warning messages can be present (but only one error message)
  std::string WarningMessages();

  //functions from ACF
  int Initialize();

  /////////////////////////////////////////////////////////////////////////
  /// Connect the data region described by "dataDescriptor" to graph
  /// port "ioName".
  ///
  /// \param[in] ioName			- Graph port identifier.
  /// \param[in] dataDescriptor - Description of contiguous data region.
  ///
  /// \return  0 if successful, non-zero if an an error occurred.
  /////////////////////////////////////////////////////////////////////////
  int ConnectIO(const std::string ioName, vsdk::SUMat& dataDescriptor);

  /////////////////////////////////////////////////////////////////////////
  /// Connect the region of interest (ROI) described by "lContigDataDesc",
  /// "lROI_XOffset", "lROI_YOffset", "lROI_Width", and "lROI_Height"
  /// to graph port "lPortIdentifier".
  ///
  /// \param[in] lPortIdentifier - Graph port identifier.
  /// \param[in] lContigDataDesc - Description of contiguous data region that 'contains' the ROI.
  /// \param[in] lROI_XOffset	 - The X offset of the top left corner of the ROI (relative to the top left
  /// corner of the region described by "lContigDataDesc").
  /// \param[in] lROI_YOffset	-  The Y offset of the top left corner of the ROI (relative to the top left
  /// corner of the region described by "lContigDataDesc").
  /// \param[in] lROI_Width		- The width of the ROI.
  /// \param[in] lROI_Height	- The height of the ROI.
  ///
  /// \return  0 if successful, non-zero if an an error occurred.
  /////////////////////////////////////////////////////////////////////////
  int ConnectIO_ROI(std::string lPortIdentifier,
	  vsdk::SUMat& lUMat,
	  int ROI_left,
	  int ROI_top,
	  int ROI_width,
	  int ROI_height);

  int ConnectIndirectInput(const std::string& ioName, vsdk::SUMat& umat, vsdk::SUMat& offsetUMat);

  int FlagInputAsChunkBasedIndirect(const std::string& ioName);
  int Start();
  int Wait();

  int Start_Prepare();

  int Start_Run();

  int Wait_NoReset(){
	  return Wait();
  }

  int NoWait_Reset(){
	  return 0;
  }

  //get the execution plan of the graph
  std::string GetExecutionPlanDescription();

  //get the intermediate result (DataPlace) from inside a graph, returns null if nothing found
  //doesn't set an error!
  ACF_Graph::DataPlace* GetDataPlacePtr(const std::string& kernelInstanceName, const std::string& portName);
  ACF_Graph::DataPlace* GetDataPlacePtr(const std::string& graphPortName)
  {
    return GetDataPlacePtr("", graphPortName);
  }


  /// ADDED for compatibility to ACF library
  void DumpApuProfilingInfoToStdout(){}
  void DumpHostProfilingInfoToStdout(){}
  AcfProfilingInfo RetAcfProfilingInfo(){ return AcfProfilingInfo(); }
  ACF_Process_PortInfo* RetPortInfo(std::string& aPortID) { return NULL;/*_graphUsed->GraphPort(aPortID); */ }
  int32_t SelectApuConfiguration(ACF_APU_CFG aConfig, int aID){ return 0; }
  void FlagInputAsVerticalSdOverlap(std::string){};
  int32_t SelectScenario(std::string lPortIdentifier, int32_t lChunkWidth, int32_t lChunkHeight)
  {
    SetInputChunkSize(lPortIdentifier, lChunkWidth, lChunkHeight);
    return (int32_t) (errorOccured == false);
  }
  int32_t OverridePortChunkSize(std::string lPortIdentifier, int32_t lChunkWidth, int32_t lChunkHeight)
  {
    SetInputChunkSize(lPortIdentifier, lChunkWidth, lChunkHeight);
    return (int32_t) (errorOccured == false);
  }

  }; // class ACF_Process_Desc_APU
}
typedef APEX2::ACF_Process_Desc_APU ACF_Process_APU;

#endif /* ACFPROCESSDESCAPU_HPP */
