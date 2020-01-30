/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2016 Freescale Semiconductor
* Copyright 2017-2018 NXP 
* All Rights Reserved
*
*****************************************************************************
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
* \brief APU ACF graph simulation
* \author Igor Aleksandrowicz
* \version
* \date 30-January-2014
****************************************************************************/

#ifndef ACFGRAPH_HPP
#define ACFGRAPH_HPP

#include "acf_kernel.hpp"
#include "acf_datadesc.hpp"
//#include <icp_data.h>
//using namespace icp;

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <list>
using std::string;
using std::vector;
using std::map;
using std::unordered_map;
using std::pair;
using std::list;

using namespace std;

namespace APEX2
{

class ACF_Graph
{
public:

  //structure for storing data at graph internal connections (each kernel input or output port has its DataPlace)
  struct DataPlace
  {
	  unsigned int tileWidth;
	  unsigned int tileHeight;
	  unsigned int tileCount;
	  bool isInitialized;
	  
	  //One padding is used for all directions, it's equal to the largest of the directional spatial data dependencies.
	  //Doesn't matter when using the stride, wouldn't be equal to real ACF padding anyways (ACF merges, multiple spatial dependencies in a graph to one)
	  int padding;
	  int paddingVert;
	  PortDataType portDataType;

	  unsigned int e0Width;
	  unsigned int e0Height;

	  unsigned int ekWidth;
	  unsigned int ekHeight;
	  
	  bool vectorData;
	  bool fixedData;
	  bool staticData;

	  int actualSize;
	  void* dataPtr;

	  bool connected;

    DataPlace() :
		tileWidth(0), tileHeight(0), tileCount(1), isInitialized(false), padding(0), paddingVert(0), portDataType(d08u), e0Width(1), e0Height(1), ekWidth(1), ekHeight(1), vectorData(true), fixedData(false), staticData(false), actualSize(0), dataPtr(0), connected(false)
    {}

    void RequestMemory(int requestedSize)
    {
      if (requestedSize != actualSize)
      {
		  if (dataPtr != NULL) {
			  delete[]((unsigned char*)dataPtr);
		  }

        dataPtr = (void*)(new unsigned char[requestedSize]);
        actualSize = requestedSize;
      }
    }

    ~DataPlace()
    {
      delete[]((unsigned char*)dataPtr);
    }
  };

  std::string RetGraphIdentifier() {
	  return identifier;
  }

private:
  std::string identifier;

  //map of kernel instances present in the graph
  typedef map<std::string, ACF_Kernel> KernelMapType;
  KernelMapType kernels;

  //structure used for storing graph input and output ports
  struct GraphPortInfo
  {
    int chunkWidth;
    int chunkHeight;
    __port port;
    bool isInitialized;

    GraphPortInfo():
		chunkWidth(1), chunkHeight(1), isInitialized(false)
    {}

    GraphPortInfo(const __port& port):
		chunkWidth(1), chunkHeight(1), port(port), isInitialized(false)
    {}
  };

  //map of graph ports
  typedef unordered_map<std::string, GraphPortInfo> GraphPortInfoType;
  unordered_map<std::string, GraphPortInfo> graphPorts;
  list<std::string> lazyInitialized_graphPorts;

  //structure for storing data at graph inputs and outputs
  class GraphDataInfo
  {
  public:
    bool isIndirect;
    //data can be different than the original data, allocation have to be handled manually
	//data will be equal to original data when the data are spanning all CUs or different to data and allocated in the lib otherwise, then they have to be allocated and deallocated in the lib
    ACF_DataDesc* data;
    ACF_DataDesc* offsets;

    //original data will be different from *data when width < CU count * chunk width (because we need to add the non-active CU's data)
    //original data are pointers to host memory, should never be deleted inside the lib!
    ACF_DataDesc* originalData;
    ACF_DataDesc* originalOffsets;

    //number of active CUs implied by this graph input
    int activeCUCount;

    GraphDataInfo():
      isIndirect(false), data(0), offsets(0), originalData(0), originalOffsets(0), activeCUCount(0)
    {}

    ~GraphDataInfo()
    { 
    }

  };

  //map of graph I/O data
  typedef unordered_map<std::string, GraphDataInfo> GraphDataType;
  GraphDataType graphData;

  //structure used for handling ports inside the graph
  struct PortDescriptor
  {
    //kernelInstanceName = "" means it's a graph port
    std::string kernelInstanceName;
    __port port;

    bool operator==(const PortDescriptor& other) const
    {
      return kernelInstanceName == other.kernelInstanceName && port.identifier.identifier == other.port.identifier.identifier;
    }

    bool IsGraphPort()
    {
      return kernelInstanceName == "";
    }
  };

  typedef  ACF_Graph::PortDescriptor ACF_Port;

  //port -> port connection
  struct Connection
  {
    PortDescriptor a;
    PortDescriptor b;

    Connection(PortDescriptor a, PortDescriptor b):a(a), b(b){}

    bool operator==(const Connection& other) const
    {
      return a == other.a && b == other.b;
    }
  };
  //all graph connections
  vector<Connection> connections;

  //data place database
  typedef pair<std::string, std::string> DataPlaceKey;
  typedef map<DataPlaceKey, DataPlace> DataPlaceMapType;
  DataPlaceMapType dataPlaceMap;

  //error handling data
  bool errorOccured;
  std::string errorMessage;
  bool warningOccured;
  std::string warningMessages;

  //one step of a graph execution plan
  struct PlanElement
  {
    //kernel instance, can be NULL - e.g. for the first propagation
    KernelMapType::iterator kernelInstance;
    //kernel data place indices
    vector<DataPlaceKey> kernelDataPlaces;
    //data place propagations which should occur after running the element
    vector<pair<DataPlaceKey, DataPlaceKey>> dataPlacePropagations;
  };

  //graph execution plan
  vector<PlanElement> kernelPlan;
public:
  ACF_Graph()
	  : errorOccured(false)
    , errorMessage("")
    , warningOccured(false)
    , warningMessages("")
  {
  }


  virtual ~ACF_Graph();

  void SetIdentifier(const std::string& identifier)
  {
    if (errorOccured)
    {
      return;
    }
    this->identifier = identifier;
  }

  ACF_Port* SetKernelPortOutputDelay(std::string lKernelIdentifier, std::string lPortIdentifier, int32_t lOutputDelay){ return NULL; }; //THIS IS TEMPORARY


  std::string RetIdentifier() const
  {
	  return identifier;
  }

  //from ACF
  void AddKernel(const std::string& kernelInstanceName, const std::string& kernelName);
  void AddInputPort(const std::string& inputPortName);
  void AddOutputPort(const std::string& outputPortName);
  //get a port descriptor that can be used in the Connect function
  PortDescriptor GraphPort(const std::string& graphPortName);
  PortDescriptor KernelPort(const std::string& kernelInstanceName, const std::string& kernelPortName);
  void Connect(PortDescriptor a, PortDescriptor b);

  //error handling
  bool ErrorOccured(){ return errorOccured; }
  std::string ErrorMessage(){ return errorMessage; }
  bool WarningOccured(){ return warningOccured; }
  std::string WarningMessages(){ return warningMessages; }
  int QueryPortChunkSize(const std::string& graphPortName, int& chunkWidth, int& chunkHeight);

  virtual void Create() = 0;

private:
  std::string GetPlanDescription();

  //get the intermediate result from inside a graph, returns null if nothing found
  //doesn't set the error!
  DataPlace* GetDataPlacePtr(const std::string& kernelInstanceName, const std::string& portName);

  void SetGraphPortTileSize(const std::string& graphPortName, int tileWidth, int tileHeight);

  //creates the execution plan
  bool MakeSerialPlan();

  bool AutoIntializeChunkSizes(const std::string& graphPortName, ACF_DataDesc* data, int vectorWidth);

  //Test if chunk width is conforming with ACF chunk widths
  bool TestChunkWidth(unsigned int chunkW, std::string& errMsg);

  //graph inputs setting functions
  void ConnectData(const std::string& graphPortName, ACF_DataDesc* data);
  void ConnectIndirectData(const std::string& graphPortName, ACF_DataDesc* data, ACF_DataDesc* offsets);
  void FlagIndirect(const std::string& graphPortName);

  //compute the chunks sizes startnig from graph inputs
  void ComputeChunkSizes();

  //prepare the connection data places
  void PrepareDataPlaces();
  //run the functions
  void Run();

  friend class ACF_Process_Desc_APU;

  //help function for handling active/inactive CUs by resizing data descriptors
  static void CopyChangeWidth(ACF_DataDesc& destination, ACF_DataDesc& source);
};

}

#endif /* ACFGRAPH_HPP */


