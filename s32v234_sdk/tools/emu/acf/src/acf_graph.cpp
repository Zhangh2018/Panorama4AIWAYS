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
/**
* \file APU_ACF.cpp
* \brief APU ACF graph simulation
* \author Igor Aleksandrowicz
* \version
* \date 31-January-2014
****************************************************************************/

#include "acf_graph.hpp"
#include <algorithm>
#include <unordered_set>
#include <cstdio>
#include "apu_config.hpp"
#include "apu_extras.hpp"
#include "apu_cond.hpp"
using std::unordered_set;
using std::make_pair;
using std::max;

using namespace APEX2;
using namespace std;

#if __GNUG__
#define sprintf_s(buffer, str, ...) sprintf(buffer, str, ##__VA_ARGS__)
#endif

#include "acf_lib.hpp"

/****************************************************************************
* functions
****************************************************************************/
void ACF_Graph::AddKernel(const std::string& kernelInstanceName, const std::string& kernelName)
{
  if (errorOccured)
  {
    return;
  }

  //validation checks
  
  if (kernels.count(kernelInstanceName) > 0)
  {
    errorOccured = true;
    errorMessage = "duplicated kernel instance: " + kernelName;
    return;
  }

  ACF_Kernel kernelPrototype = ACF_Kernel::GetPrototypeFromDatabase(kernelName);
  if (kernelPrototype.GetName() != kernelName)
  {
    errorOccured = true;
    errorMessage = "undefined kernel: " + kernelName;
    return;
  }

  if (kernelPrototype.ErrorOccured())
  {
    //propagate the kernel error
    errorOccured = true;
    errorMessage = "invalid kernel (errors in the kernel): " + kernelName + ", kernel error: " + kernelPrototype.ErrorMessage();
    return;
  }

  //add the kernel instance into the map
  kernels[kernelInstanceName] = kernelPrototype;

  //create DataPlaces for all the kernel instance ports
  for (int i = 0; i < kernelPrototype.GetPortCount(); ++i)
  {
    DataPlace dataPlace;
	
    //use the maximal padding needed
    int padding = kernelPrototype.GetPort(i).spatial_dep.dependencies[0];
    padding = max(padding, kernelPrototype.GetPort(i).spatial_dep.dependencies[1]);
    int paddingVert = max(0, kernelPrototype.GetPort(i).spatial_dep.dependencies[2]);
    paddingVert = max(paddingVert, kernelPrototype.GetPort(i).spatial_dep.dependencies[3]);
    dataPlace.padding = padding;
	dataPlace.paddingVert = paddingVert;

    dataPlace.portDataType = kernelPrototype.GetPort(i).e0_data_type.dataType;

    dataPlace.e0Width = kernelPrototype.GetPort(i).e0_size.size[0];
    dataPlace.e0Height = kernelPrototype.GetPort(i).e0_size.size[1];

    dataPlace.ekWidth = kernelPrototype.GetPort(i).ek_size.size[0];
    dataPlace.ekHeight = kernelPrototype.GetPort(i).ek_size.size[1];

    Attributes attributes = kernelPrototype.GetPort(i).attributes.attributes;
    dataPlace.vectorData = IsVectorAttribute(attributes);
    dataPlace.fixedData = IsFixedAttribute(attributes);
    dataPlace.staticData = IsStaticAttribute(attributes);

    //fixed ports' tile sizes are defined by ek
    if (IsFixedAttribute(kernelPrototype.GetPort(i).attributes.attributes))
    {
      dataPlace.tileWidth = dataPlace.ekWidth;
      dataPlace.tileHeight = dataPlace.ekHeight;
    }

    dataPlaceMap[make_pair(kernelInstanceName, kernelPrototype.GetPort(i).identifier.identifier)] = dataPlace;
  }
}

void ACF_Graph::AddInputPort(const std::string& inputPortName)
{
  if (errorOccured)
  {
    return;
  }

  //validation check
  if (graphPorts.count(inputPortName) > 0)
  {
    errorOccured = true;
    errorMessage = "duplicated input graph port: " + inputPortName;
    return;
  }

  //create new input graph port
  GraphPortInfo graphPortInfo(__port(__index(0), __identifier(inputPortName), __attributes(ACF_ATTR_VEC_IN), __spatial_dep(0, 0, 0, 0), __e0_data_type(d08u), __e0_size(0, 0), __ek_size(0, 0)));
  graphPorts[inputPortName] = graphPortInfo;

  //create the corresponding DataPlace
  dataPlaceMap[make_pair("", inputPortName)] = DataPlace();
}

void ACF_Graph::AddOutputPort(const std::string& outputPortName)
{
  if (errorOccured)
  {
    return;
  }

  //validation check
  if (graphPorts.count(outputPortName) > 0)
  {
    errorOccured = true;
    errorMessage = "duplicated output graph port: " + outputPortName;
    return;
  }

  //create new output graph port
  __port port(__index(0), __identifier(outputPortName), __attributes(ACF_ATTR_VEC_OUT), __spatial_dep(0, 0, 0, 0), __e0_data_type(d08u), __e0_size(0, 0), __ek_size(0, 0));
  graphPorts[outputPortName] = port;

  //create the corresponding DataPlace
  dataPlaceMap[make_pair("", outputPortName)] = DataPlace();
}

ACF_Graph::PortDescriptor ACF_Graph::GraphPort(const std::string& graphPortName)
{
  ACF_Graph::PortDescriptor portDescriptor;
  //kernel name is a an empty string for graph ports
  portDescriptor.kernelInstanceName = "";
  if (errorOccured)
  {
    return portDescriptor;
  }
  
  //return the info from the database or report an error if not found
  unordered_map<std::string, GraphPortInfo>::iterator it = graphPorts.find(graphPortName);
  if (it != graphPorts.end())
  {
    portDescriptor.port = graphPorts[graphPortName].port;
  }
  else
  {
    errorOccured = true;
    errorMessage = "undefined graph port: " + graphPortName;
    portDescriptor.port.identifier.identifier = "";
  }
  return portDescriptor;
}

ACF_Graph::PortDescriptor ACF_Graph::KernelPort(const std::string& kernelInstanceName, const std::string& kernelPortName)
{
  ACF_Graph::PortDescriptor portDescriptor;
  portDescriptor.kernelInstanceName = kernelInstanceName;
  if (errorOccured)
  {
    return portDescriptor;
  }
  
  //find the kernel
  KernelMapType::iterator it = kernels.find(kernelInstanceName);
  if (it != kernels.end())
  {
    bool portFound = false;
    //search the kernel's ports
    for (int i = 0; i < MAX_KN_PORTS; ++i)
    {
      if (it->second.GetPort(i).identifier.identifier == kernelPortName)
      {
        portDescriptor.port = it->second.GetPort(i);
        portFound = true;
        break;
      }
    }
    if (!portFound)
    {
      errorOccured = true;
      errorMessage = "undefined port: " + kernelPortName + " in the kernel instance: " + kernelInstanceName;
    }
  }
  else
  {
    errorOccured = true;
    errorMessage = "undefined kernel instance: " + kernelInstanceName;
  }
  return portDescriptor;
}

void ACF_Graph::Connect(ACF_Graph::PortDescriptor a, ACF_Graph::PortDescriptor b)
{
  if (errorOccured)
  {
    return;
  }

  //validate the connection itself
  Connection connection = Connection(a, b);
  if (connection.a.kernelInstanceName == connection.b.kernelInstanceName && !connection.a.kernelInstanceName.empty())
  {
    errorOccured = true;
    errorMessage = "cycle on the kernel instance: " + connection.a.kernelInstanceName;
    return;
  }

  if (connection.a.kernelInstanceName.empty() && connection.b.kernelInstanceName.empty())
  {
    errorOccured = true;
    errorMessage = "can't connect 2 graph ports: " + connection.a.port.identifier.identifier + " -> " + connection.b.port.identifier.identifier;
    return;
  }

  //validate the connection against other connections
  for (unsigned int i = 0; i < connections.size(); ++i)
  {
   std::string separatorA = connection.a.kernelInstanceName == "" ? "" : ".";
   std::string separatorB = connection.b.kernelInstanceName == "" ? "" : ".";

    if (connections[i] == connection)
    {
      errorOccured = true;
      errorMessage = "duplicated connection: " +
        connection.a.kernelInstanceName + separatorA + connection.a.port.identifier.identifier +
        " -> " +
        connection.b.kernelInstanceName + separatorB + connection.b.port.identifier.identifier;
      return;
    }

    if (connections[i].b == connection.b)
    {
      errorOccured = true;
      errorMessage = "two connections to port: " +
        connection.b.kernelInstanceName + separatorB + connection.b.port.identifier.identifier;
      return;
    }
  }

  DataPlaceKey dpAK = make_pair(connection.a.kernelInstanceName, connection.a.port.identifier.identifier);
  DataPlaceKey dpBK = make_pair(connection.b.kernelInstanceName, connection.b.port.identifier.identifier);
  DataPlace& dpA = dataPlaceMap[dpAK];
  DataPlace& dpB = dataPlaceMap[dpBK];
  dpA.connected = true;
  dpB.connected = true;
  //prepare the graph port data places here, kernel port data places are defined by the kernels' metadata
  if (connection.a.IsGraphPort())
  {
    dpA.vectorData = IsVectorAttribute(connection.b.port.attributes.attributes);
    dpA.staticData = IsStaticAttribute(connection.b.port.attributes.attributes);
    dpA.fixedData = IsFixedAttribute(connection.b.port.attributes.attributes);
    dpA.portDataType = connection.b.port.e0_data_type.dataType;
    dpA.e0Width = connection.b.port.e0_size.size[0];
    dpA.e0Height = connection.b.port.e0_size.size[1];
    dpA.ekWidth = connection.b.port.ek_size.size[0];
    dpA.ekHeight = connection.b.port.ek_size.size[1];
    GraphPortInfo& graphPort = graphPorts[connection.a.port.identifier.identifier];
    graphPort.port.attributes = ReturnAttribute(true, dpA.vectorData, dpA.staticData, dpA.fixedData);
	graphPort.port.e0_data_type.dataType = connection.b.port.e0_data_type.dataType;
  }
  if (connection.b.IsGraphPort())
  {
    dpB.vectorData = IsVectorAttribute(connection.a.port.attributes.attributes);
    dpB.staticData = IsStaticAttribute(connection.a.port.attributes.attributes);
    dpB.fixedData = IsFixedAttribute(connection.a.port.attributes.attributes);
    dpB.portDataType = connection.a.port.e0_data_type.dataType;
    dpB.e0Width = connection.a.port.e0_size.size[0];
    dpB.e0Height = connection.a.port.e0_size.size[1];
    dpB.ekWidth = connection.a.port.ek_size.size[0];
    dpB.ekHeight = connection.a.port.ek_size.size[1];
    GraphPortInfo& graphPort = graphPorts[connection.b.port.identifier.identifier];
    graphPort.port.attributes = ReturnAttribute(false, dpB.vectorData, dpB.staticData, dpB.fixedData);
	graphPort.port.e0_data_type.dataType = connection.a.port.e0_data_type.dataType;
  }
  //add the connection to the database
  connections.push_back(connection);
}

void ACF_Graph::SetGraphPortTileSize(const std::string& graphPortName, int tileWidth, int tileHeight)
{
  if (errorOccured)
  {
    return;
  }

  //validate
  if (graphPorts.count(graphPortName) < 1)
  {
    errorOccured = true;
    errorMessage = "cannot set the chunk size for an undefined graph port: " + graphPortName;
    return;
  }

  graphPorts[graphPortName].isInitialized = true;

  if (!IsInputAttribute(graphPorts[graphPortName].port.attributes.attributes))
  {
    errorOccured = true;
    errorMessage = "cannot set the chunk size for a non-input graph port: " + graphPortName;
    return;
  }

  if (tileWidth < 1 || tileHeight < 1)
  {
    errorOccured = true;
    errorMessage = "invalid input chunk size: " + graphPortName;
    return;
  }

  //set the tile widths
  graphPorts[graphPortName].chunkWidth = tileWidth;
  graphPorts[graphPortName].chunkHeight = tileHeight;
}

bool ACF_Graph::MakeSerialPlan()
{
  if (errorOccured)
  {
    return false;
  }

  //check whether there are any graph inputs and outputs
  {
    int inputGraphPortCount = 0;
    int outputGraphPortCount = 0;
    for (GraphPortInfoType::iterator it = graphPorts.begin(); it != graphPorts.end(); ++it)
    {
      Attributes attributes = it->second.port.attributes.attributes;
      if (IsOutputAttribute(attributes))
      {
        ++outputGraphPortCount;
      }
      else if (IsInputAttribute(attributes))
      {
        ++inputGraphPortCount;
      }
    }
    if (inputGraphPortCount == 0)
    {
      errorOccured = true;
      errorMessage = "no graph input";
      return false;
    }
    if (outputGraphPortCount == 0)
    {
      errorOccured = true;
      errorMessage = "no graph output";
      return false;
    }
  }

  //topological sort http://en.wikipedia.org/wiki/Topological_sorting
  //not using sets of objects because of the const/updating issues
  struct GraphNode
  {
    int id;
    KernelMapType::iterator kernelInstance;
	//set of indices of predecessor nodes
    unordered_set<int> prev;
	//set of indices of successor nodes
    unordered_set<int> next;
  };

  //prepare the nodes
  vector<GraphNode> allGraphNodes;
  //kernel name to node id translation
  unordered_map<std::string, int> graphNodeMap;
  int index = 0;
  for (KernelMapType::iterator it = kernels.begin(); it != kernels.end(); ++it)
  {
    GraphNode n;
    n.id = index;
    n.kernelInstance = it;
    allGraphNodes.push_back(n);
    graphNodeMap[it->first] = index;
    ++index;
  }
  
  //prepare the edges, i.e. fill the prev and next node sets
  for (unsigned int ki = 0; ki < allGraphNodes.size(); ++ki)
  {
    KernelMapType::iterator kernelIt = allGraphNodes[ki].kernelInstance;
    for (unsigned int ci = 0; ci < connections.size(); ++ci)
    {
      if (connections[ci].a.kernelInstanceName == kernelIt->first)
      {
        std::string connectKernelName = connections[ci].b.kernelInstanceName;
        if (!connectKernelName.empty())
        {
          int connectGraphNodeID = graphNodeMap[connectKernelName];
          allGraphNodes[ki].next.insert(connectGraphNodeID);
        }
      }
      if (connections[ci].b.kernelInstanceName == kernelIt->first)
      {
        std::string connectKernelName = connections[ci].a.kernelInstanceName;
        if (!connectKernelName.empty())
        {
          int connectGraphNodeID = graphNodeMap[connectKernelName];
          allGraphNodes[ki].prev.insert(connectGraphNodeID);
        }
      }
    }
  }

  //first plan element - propagate graph inputs
  PlanElement firstPlanElement;
  //no kernel for the first element
  firstPlanElement.kernelInstance = kernels.end();
  for (unsigned int i = 0; i < connections.size(); ++i)
  {
    //for each connection with empty input kernel name (i.e. connections from graph ports)
    if (connections[i].a.kernelInstanceName == "")
    {
      DataPlaceKey inputGraphDataPlaceKey = make_pair("", connections[i].a.port.identifier.identifier);
      DataPlaceKey inputKernelDataPlaceKey = make_pair(connections[i].b.kernelInstanceName, connections[i].b.port.identifier.identifier);
	  //propagate the graph input to the kernel input
      firstPlanElement.dataPlacePropagations.push_back(make_pair(inputGraphDataPlaceKey, inputKernelDataPlaceKey));
    }
  }
  //don't use the first element if there are no graph inputs
  if (!firstPlanElement.dataPlacePropagations.empty())
  {
    kernelPlan.push_back(firstPlanElement);
  }

  //core topological sort
  unordered_set<int> remainingNodes;
  unordered_set<int> processedNodes;

  //add the nodes into pending nodes
  for (unsigned int i = 0; i < allGraphNodes.size(); ++i)
  {
    remainingNodes.insert(int(i));
  }

  while (!remainingNodes.empty())
  {
    bool nextNodeFound = false;
    //for the remaining kernel instances
    for (unordered_set<int>::iterator it = remainingNodes.begin(); it != remainingNodes.end(); ++it)
    {
      bool process = true;
      //check if there are some unprocessed predecessors
      for (unordered_set<int>::iterator it2 = allGraphNodes[*it].prev.begin(); it2 != allGraphNodes[*it].prev.end(); ++it2)
      {
        if (remainingNodes.count(*it2))
        {
          process = false;
          break;
        }
      }

      //there aren't, we found a node which we can add to our plan
      if (process)
      {
        processedNodes.insert(*it);
		
        //initialize the plan element
        PlanElement planElement;
        planElement.kernelInstance = allGraphNodes[*it].kernelInstance;
        std::string kernelInstanceName = planElement.kernelInstance->first;
        //handle dataPlace transforms
        for (int i = 0; i < planElement.kernelInstance->second.GetPortCount(); ++i)
        {
          std::string portIdentifier = planElement.kernelInstance->second.GetPort(i).identifier.identifier;
          DataPlaceKey dataPlaceKey = make_pair(kernelInstanceName, portIdentifier);
          //add the data place corresponding to the port to the plan
          planElement.kernelDataPlaces.push_back(dataPlaceKey);
          //find the DataPlace-DataPlace propagations corresponding to the kernel output connections
          for (unsigned int i = 0; i < connections.size(); ++i)
          {
            if (connections[i].a.kernelInstanceName == kernelInstanceName && connections[i].a.port.identifier.identifier == portIdentifier)
            {
              DataPlaceKey secondKey = make_pair(connections[i].b.kernelInstanceName, connections[i].b.port.identifier.identifier);
              //add the propagation into the plan element
              planElement.dataPlacePropagations.push_back(make_pair(dataPlaceKey, secondKey));
            }
          }
        }
        kernelPlan.push_back(planElement);
        remainingNodes.erase(it);
        nextNodeFound = true;
        break;
      }
    }
	
    //we couldn't use any of the remaining nodes, therefore the graph contains a cycle
    if (!nextNodeFound)
    {
      errorOccured = true;
      errorMessage = "cycles found in the graph: " + identifier;
      kernelPlan.clear();
      return false;
    }
  }

  return true;
}

std::string ACF_Graph::GetPlanDescription()
{
  if (errorOccured)
  {
    return "no plan due to errors\n";
  }

  //create a text description of the plan
  std::string description;
  for (unsigned int i = 0; i < kernelPlan.size(); ++i)
  {
    KernelMapType::iterator kernelInstance = kernelPlan[i].kernelInstance;
    if (kernelInstance != kernels.end())
    {
      description.append("run the kernel: ");
      description.append(kernelPlan[i].kernelInstance->first);
      description.append("\n");
    }
    for (unsigned int j = 0; j < kernelPlan[i].dataPlacePropagations.size(); ++j)
    {
      description.append("propagate: ");
      std::string firstKernelName = kernelPlan[i].dataPlacePropagations[j].first.first;
      //don't include kernel name and . when dealing with graph port
      if (firstKernelName != "")
      {
        description.append(firstKernelName);
        description.append(".");
      }
      description.append(kernelPlan[i].dataPlacePropagations[j].first.second);

      description.append(" -> ");

      std::string secondKernelName = kernelPlan[i].dataPlacePropagations[j].second.first;
      //don't include kernel name and . when dealing with a graph port
      if (secondKernelName != "")
      {
        description.append(secondKernelName);
        description.append(".");
      }
      description.append(kernelPlan[i].dataPlacePropagations[j].second.second);
      description.append("\n");
    }
  }

  return description;
}


bool ACF_Graph::TestChunkWidth(unsigned int dpChunkWidth, std::string& errMsg)
{
	const unsigned int _allowedChunkWidth[] = { 
		2, 4, 6, 8,
		10, 12, 14, 16,
		18, 20, 22, 24,
		26, 28, 30, 32,
		36, 40, 44, 48,
		52, 56, 60, 64,
		72, 80, 88, 96,
		104, 112, 120, 128, 256, 512 };

	const unsigned int _lastChunkW = sizeof(_allowedChunkWidth) / sizeof(int);

	static string chunkStr;
	if (chunkStr.empty()) {
		chunkStr = " { \n   ";
		for (unsigned int i = 0; i<_lastChunkW; ++i) {
			chunkStr = chunkStr + std::to_string(_allowedChunkWidth[i]);
			if (i % 4 == 0) {
				chunkStr = chunkStr + ",\n   ";
			}
			else {
				if (i < _lastChunkW) {
					chunkStr = chunkStr + ", ";
				}
			}
		}
		chunkStr = chunkStr + "\n }";
	}

	bool allowedChunkWidthFound = false;

	if (dpChunkWidth > _allowedChunkWidth[_lastChunkW - 1]) {
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "data chunk width %d is too large maximum is %d", dpChunkWidth, _allowedChunkWidth[_lastChunkW - 1]);
		errMsg = std::string(tmp);
		return false;
	}

	for (unsigned int chwIdx = 0; chwIdx < _lastChunkW && dpChunkWidth >= _allowedChunkWidth[chwIdx]; ++chwIdx){
		if (dpChunkWidth == _allowedChunkWidth[chwIdx]) {
			allowedChunkWidthFound = true;
			break;
		}
	}

	if (!allowedChunkWidthFound) {
		char tmp[400];
		sprintf_s(tmp, "data chunk width %d is inappropriate. Allowed chunk widths are %s"
			, dpChunkWidth, chunkStr.c_str());
		errMsg = std::string(tmp);
	}

	return allowedChunkWidthFound;
}


bool ACF_Graph::AutoIntializeChunkSizes(const std::string& graphPortName, ACF_DataDesc* data, int vectorWidth) {
	bool isError = false;
	GraphPortInfo& gpi = graphPorts[graphPortName];
		
	int   dw = data->RetWidth(), dh = data->RetHeight()
        , dE0_x = data->RetElementDimX()//, dE0_y = data->RetElementDimY()
		, elemSz = RetDataTypeSizeInBytes(data->RetElementDataType())
		, pixelSzInBytes = elemSz*dE0_x;

	int totWidth = dw, totHeight = dh; 

	//These are multiples of pixels (i.e dE0_x * szOfData ) 
	int nrPixelsInChunk = 0; 
	std::string errMsg;

	bool chunkWFound = false;
	// find the lowest chunk width possible, if the image is not covering/divisible to the total number of available CU's
	for (int nrCUs = vectorWidth; nrCUs > 1; --nrCUs) {
		bool divisible = totWidth%nrCUs == 0;		
		int tmpNrPixelsInChunk = divisible ? totWidth / nrCUs : 0;
		divisible &= TestChunkWidth(tmpNrPixelsInChunk * pixelSzInBytes, errMsg);
		if (divisible) {
			chunkWFound = true;
			nrPixelsInChunk = tmpNrPixelsInChunk;
			break;
		}
	}

	if (!chunkWFound)
	{
		errorMessage = errMsg;
		errorOccured = true;
		return errorOccured;
	}

	// find the lowest chunk height possible
	int chunkHeight = totHeight % 2 == 0 ? 2 : (totHeight % 3 == 0 ? 3 : (totHeight % 5 == 0 ? 5 : (totHeight % 7 == 0 ? 7 : (totHeight % 7 == 0 ? 7 : (totHeight % 11 == 0 ? 11 : (totHeight % 13 == 0 ? 13 : 1))))));
	
	gpi.chunkWidth = nrPixelsInChunk;
	gpi.chunkHeight= chunkHeight;
	gpi.isInitialized = true;

	ComputeChunkSizes(); // TBD: might be inefficient here, because it is called for each not initialized graph port. 

	return isError;
}

int ACF_Graph::QueryPortChunkSize(const std::string& graphPortName, int& chunkWidth, int& chunkHeight) {
	if (graphPorts.count(graphPortName) < 1) {
		errorOccured = true;
		errorMessage = std::string("undefined graph port: ") + graphPortName;		
	}

	if (errorOccured)
	{
		return 1;
	}

	GraphPortInfo& gpi = graphPorts[graphPortName];
	chunkWidth = gpi.chunkWidth;
	chunkHeight = gpi.chunkHeight;
	return 0;
}

void ACF_Graph::ConnectData(const std::string& graphPortName, ACF_DataDesc* data)
{
  if (errorOccured)
  {
    return;
  }

  if (graphPorts.count(graphPortName) < 1)
  {
    errorOccured = true;
    errorMessage = "undefined graph port: " + graphPortName;
  }
  else
  {
	  Attributes attr = graphPorts.find(graphPortName)->second.port.attributes.attributes;

    GraphDataInfo& info = graphData[graphPortName];
    //delete the intermediate data if they were different than the original data
    if (info.data != info.originalData)
    {
	  free(info.data->RetDataPtr());
	  delete info.data;
      info.data = 0;
    }

    //get the corresponding DataPlace
    DataPlace& dp = dataPlaceMap[make_pair("", graphPortName)];
    int vectorWidth = dp.vectorData ? VSIZE : 1;
	if ((IsInputAttribute(attr) && !dp.isInitialized) && vectorWidth > 1) {
		AutoIntializeChunkSizes(graphPortName, data, vectorWidth);

		list<std::string>::iterator lazy_it;
		for (lazy_it = lazyInitialized_graphPorts.begin(); lazy_it != lazyInitialized_graphPorts.end(); ++lazy_it) {
			std::string grPName = *lazy_it;
			if (dataPlaceMap[make_pair("", grPName)].isInitialized) {
				//lazyInitialized_graphPorts.remove(*lazy_it);
				GraphDataInfo& grIn = graphData[grPName];
				ConnectData(grPName, grIn.data);				
			}
		}
	}

	if (!IsInputAttribute(attr) && !dp.isInitialized) {
		lazyInitialized_graphPorts.push_back(graphPortName);
		info.data = data;
		info.originalData = data;
		return;
	}

    unsigned int dpTileSpan = dp.tileWidth * dp.e0Width;
    unsigned int dpSliceHorSpan = dpTileSpan * vectorWidth;
    unsigned int dpSliceVerSpan = dp.tileHeight * dp.e0Height;
    unsigned int ddHorSpan = data->RetWidth() * data->RetElementDimX();
    unsigned int ddVerSpan = data->RetHeight() * data->RetElementDimY();
	unsigned int dpChunkElems = dp.tileWidth * dp.e0Width;
	unsigned int dpChunkWidth = dpChunkElems * RetDataTypeSizeInBytes(data->RetElementDataType());

	const int maxNrChunkElems = 128; // As in ACF


	if (dpChunkWidth % 2 != 0 && !dp.fixedData){
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "data chunk width %d is inappropriate for port %s. It has to be a multiple of 2", dpChunkWidth, graphPortName.c_str());
		//errorMessage = "input data type doesn't match the graph port data type: " + graphPortName;
		errorMessage = std::string(tmp);
		return;
	}
	
	if (dpChunkElems > maxNrChunkElems && !dp.fixedData) {
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "number of chunk  elems %d is too large maximum is %d for port %s ", dpChunkElems, maxNrChunkElems, graphPortName.c_str());
		//errorMessage = "input data type doesn't match the graph port data type: " + graphPortName;
		errorMessage = std::string(tmp);
		return;
	}


	if (!dp.fixedData) {
		if (!TestChunkWidth(dpChunkWidth, errorMessage))
		{
			errorMessage = errorMessage + " for port " + graphPortName;
			errorOccured = true;
			return;
		}

	}

	icp::DATATYPE dataDT = data->RetElementDataType();
	if (dp.portDataType != static_cast<PortDataType>(dataDT))
	{
		if (!((dp.portDataType == d32u || dp.portDataType == d32s) && (dataDT == ICP_DATATYPE_32S || dataDT == ICP_DATATYPE_32U))) {
			errorOccured = true;
			char tmp[400];
			sprintf_s(tmp, "input data type %d doesn't match the graph port data type %d: %s ", data->RetElementDataType(), dp.portDataType, graphPortName.c_str());
			//errorMessage = "input data type doesn't match the graph port data type: " + graphPortName;

			errorMessage = std::string(tmp);
			return;
		}
	}

    //check width for scalar data
    if (!dp.vectorData && (unsigned int) data->RetWidth() != dp.tileWidth)
    {
	char tmp[400];
	 errorOccured = true;
	  sprintf_s(tmp, "scalar input data width %d doesn't match the chunk width %d: %s", data->RetWidth(), dp.tileWidth, graphPortName.c_str());
	  errorMessage = std::string(tmp);
      return;
    }
    //check width compatibility for vector data
    else if (dp.vectorData && data->RetWidth() % dp.tileWidth != 0)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "vector input data width %d  is not divisible by chunk width %d : %s", data->RetWidth(), dp.tileWidth, graphPortName.c_str());
	  errorMessage = std::string(tmp);
      return;
    }
    //check whether there is enough CUs to handle the width
    else
    {
	 if (dp.vectorData) {
	 //info.activeCUCount = data->GetWidth() * data->GetElementDimX()/ dpTileSpan;
	  info.activeCUCount = data->RetWidth() / dp.tileWidth;
      if (info.activeCUCount > APU_VSIZE)
      {
        errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "vector input data width %d (== dataWidth/tileWidth, i.e.%d/%d ) is too large, max is %d: %s", info.activeCUCount, data->RetWidth(), dp.tileWidth, APU_VSIZE, graphPortName.c_str());
		errorMessage = std::string(tmp);
        return;
      }
    }
    }

    //direct/indirect mismatch
    if (info.isIndirect)
    {
      errorOccured = true;
      errorMessage = "cannot connect direct data to an indirect output: " + graphPortName;
    }
    //element data type mismatch
	else if ((unsigned int) RetDataTypeSizeInBytes(data->RetElementDataType()) != PortDataTypeSize(dp.portDataType))
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "incompatible data element type size (%d) / kernel element data type size (%d) : %s ", RetDataTypeSizeInBytes(data->RetElementDataType()), PortDataTypeSize(dp.portDataType), graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //wrong element data size
    else if ((unsigned int) data->RetElementDimX() != dp.e0Width) 
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "data element size x %d doesn't match kernel e0 width %d for port: %s ", data->RetElementDimX(), dp.e0Width, graphPortName.c_str());
	  errorMessage = std::string(tmp);
	}
	//wrong element data size
    else if ((unsigned int) data->RetElementDimY() != dp.e0Height)
	{
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "data element size y %d doesn't match kernel e0 height %d for port: %s ", data->RetElementDimY(), dp.e0Height, graphPortName.c_str());
		errorMessage = std::string(tmp);
	}
    //chunk height compatibility check
    else if ((!dp.staticData && ((ddVerSpan % dpSliceVerSpan) != 0)) )
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "the data height %d is not divisible by the chunk height %d for port: %s", ddVerSpan, dpSliceVerSpan, graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
	//chunk height compatibility check
	else if ((!dp.staticData && (dpSliceVerSpan > ddVerSpan)))
	{
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "the data height %d is lower than the chunk height %d for port: %s ", ddVerSpan, dpSliceVerSpan, graphPortName.c_str());
		errorMessage = std::string(tmp);
	}
    //everything ok
    else
    {
      info.originalData = data;
      //create temporary data which span all CUs if the width implies some CUs shouldn't be active
	  if (dp.vectorData && dpSliceHorSpan != ddHorSpan)
      { //if data has depth > 1 (i.e. is a volume), then flatten the volume to a big image. CAUTION: this does not work for very deep volumes, due to memory shortage!
		  void *ptr = malloc((dp.tileWidth* APU_VSIZE)*data->RetHeight()*data->RetElementDimX()*RetDataTypeSizeInBytes(data->RetElementDataType()));
		  info.data = new ACF_DataDesc((uint8_t *)ptr,
									   (uint8_t *)ptr,
									   dp.tileWidth* APU_VSIZE,
									   data->RetHeight(),
									   dp.tileWidth* APU_VSIZE * data->RetElementDimX()*RetDataTypeSizeInBytes(data->RetElementDataType()),
									   data->RetElementDataType(),
									   data->RetElementDimX(),
									   data->RetElementDimY(),
									   0,
									   0,
									   0,
									   0);

        //info.data = new DataDescriptor(dp.tileWidth* APU_VSIZE, data->GetHeight()*data->GetDimensions(), data->GetElementDataType(), data->GetElementDimX(), data->GetElementDimY());
        //info.data->SetFreeOnExit(true);
      }
      else
      {
        info.data = data;
      }
    }
  }

}

//TODO: more checks for scalar data? vector data only?
void ACF_Graph::ConnectIndirectData(const std::string& graphPortName, ACF_DataDesc* data, ACF_DataDesc* offsets)
{
  if (errorOccured)
  {
    return;
  }

  if (graphPorts.count(graphPortName) < 1)
  {
    errorOccured = true;
    errorMessage = "undefined graph port: " + graphPortName;
  }
  else
  {
    GraphDataInfo& info = graphData[graphPortName];
    //delete the intermediate offsets if they were different than the original offsets
    if (info.offsets != info.originalOffsets)
    {
	  free(info.offsets->RetDataPtr());
      delete info.offsets;
      info.offsets = 0;
    }

    info.activeCUCount = (int)offsets->RetWidth();
    //get the corresponding DataPlace
    DataPlace& dp = dataPlaceMap[make_pair("", graphPortName)];
    int vectorWidth = dp.vectorData ? APU_VSIZE : 1;
    //direct//indirect mismatch
    if (!info.isIndirect)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp,"graph input is not indirect: %s", graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //element data type mismatch
    else if( (unsigned int) RetDataTypeSizeInBytes(data->RetElementDataType()) != PortDataTypeSize(dp.portDataType))
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "incompatible data type size (%d)/kernel element data type size(%d): %s", RetDataTypeSizeInBytes(data->RetElementDataType()), PortDataTypeSize(dp.portDataType), graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //wrong element data size
    else if( (unsigned int) data->RetElementDimX() != dp.e0Width)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "data element size x %d doesn't match kernel e0 width %d:  %s", data->RetElementDimX(), dp.e0Width, graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
	//wrong element data size
    else if ( (unsigned int) data->RetElementDimY() != dp.e0Height)
	{
		errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "data element size y %d doesn't match kernel e0 height %d:  %s", data->RetElementDimY(), dp.e0Height, graphPortName.c_str());
		errorMessage = std::string(tmp);
	}
    //offsets have to be 32-bit wide
    else if (offsets->RetElementDataType() != icp::DATATYPE_32S && offsets->RetElementDataType() != icp::DATATYPE_32U)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "only 32-bit offsets supported: %s", graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //offsets have to have 1x1 elements
    else if (offsets->RetElementDimX() != 1 || offsets->RetElementDimY() != 1)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "only 1x1 element offsets supported: %s", graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //only non-static vector data are supported for indirect inputs
    else if (dp.staticData || !dp.vectorData)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "indirect data not supported for scalar or static data: %s", graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //check whether there is enough CUs to handle the width
    else if ((int)offsets->RetWidth() > vectorWidth)
    {
      errorOccured = true;
	  char tmp[400];
	  sprintf_s(tmp, "offset data width %d too high, max is %d : %s", offsets->RetWidth(), vectorWidth, graphPortName.c_str());
	  errorMessage = std::string(tmp);
    }
    //everything ok
    else
    {
      info.data = data;
      info.originalData = data;
      info.originalOffsets = offsets;

      //create temporary data which span all CUs if the width implies some CUs shouldn't be active
	  if ((int)offsets->RetWidth() < vectorWidth)
      {
		  void *ptr = malloc(vectorWidth*offsets->RetHeight()*offsets->RetElementDimX()*RetDataTypeSizeInBytes(offsets->RetElementDataType()));
		info.offsets = new ACF_DataDesc((uint8_t *)ptr,
			  (uint8_t *)ptr,
			  vectorWidth,
			  offsets->RetHeight(),
			  vectorWidth * offsets->RetElementDimX()*RetDataTypeSizeInBytes(offsets->RetElementDataType()),
			  offsets->RetElementDataType(),
			  offsets->RetElementDimX(),
			  offsets->RetElementDimY(),
			  0,
			  0,
			  0,
			  0);
      }
      else
      {
        info.offsets = offsets;
      }
    }
  }
}

void ACF_Graph::FlagIndirect(const std::string& graphPortName)
{
  if (errorOccured)
  {
    return;
  }

  //only already defined input ports can be flagged as indirect
  
  if (graphPorts.count(graphPortName) < 1)
  {
    errorOccured = true;
    errorMessage = "undefined graph port: " + graphPortName;
  }
  else
  {
    if (!IsInputAttribute(graphPorts[graphPortName].port.attributes.attributes))
    {
      errorOccured = true;
      errorMessage = "non-input port cannot be flagged as indirect: " + graphPortName;
    }
    else
    {
      graphData[graphPortName].isIndirect = true;
    }
  }
}

//greatest common divisor
static int gcd(int a, int b)
{
    for (;;)
    {
        if (a == 0) return b;
        b %= a;
        if (b == 0) return a;
        a %= b;
    }
   // return 1;
}

//least common multiple
static int lcm(int a, int b)
{
    int temp = gcd(a, b);

    return temp ? (a / temp * b) : 0;
}


void ACF_Graph::ComputeChunkSizes()
{
  if (errorOccured)
  {
    return;
  }

  //propagate the input graph port info to corresponding DataPlaces
  for (GraphPortInfoType::iterator it = graphPorts.begin(); it != graphPorts.end(); ++it)
  {
    Attributes attributes = it->second.port.attributes.attributes;
    //TODO: attributes as bit masks?
    if (IsInputAttribute(attributes))
    {
      DataPlace& dp = dataPlaceMap[make_pair("", it->first)];
      //fixed chunk sizes are defined by ek
      if (!IsFixedAttribute(attributes))
      {
        dp.tileWidth = it->second.chunkWidth;
        dp.tileHeight = it->second.chunkHeight;
        dp.isInitialized = it->second.isInitialized;
      }
      else
      {
        dp.tileWidth = dp.ekWidth;
        dp.tileHeight = dp.ekHeight;
        dp.isInitialized = true;
      }
    }
  }

  //propagate chunk size info using the execution plan
  for (unsigned int i = 0; i < kernelPlan.size(); ++i)
  {
    //if plan element contains kernel execution
    if (kernelPlan[i].kernelInstance != kernels.end())
    {
      ACF_Kernel& kp = kernelPlan[i].kernelInstance->second;
      std::string instanceName = kernelPlan[i].kernelInstance->first;

      unsigned int tileWidth = 1;
      unsigned int tileHeight = 1;

      //input k-element sizes to use as numerators and denominators
      int kWidthNum = 1;
      int kWidthDenom = 1;
      int kHeightNum = 1;
      int kHeightDenom = 1;

	  bool atLeastOneIsInitialized = false;

	  for (int j = 0; j < kp.GetPortCount(); ++j)
	  {
        Attributes attributes = kp.GetPort(j).attributes.attributes;
        if (IsInputAttribute(attributes))
        {
          DataPlace& dp = dataPlaceMap[make_pair(instanceName, kp.GetPort(j).identifier.identifier)];
          //set the default tile size for the kernel to the maximum of all input tile sizes
          if (!IsFixedAttribute(attributes))
          {
            tileWidth = max(tileWidth, dp.tileWidth);
            //set the k-element size numerators to the least common multiples of the output ek sizes
            kWidthNum = lcm(kWidthNum, dp.ekWidth);
          }
          if (!IsStaticAttribute(attributes)) {
            tileHeight = max(tileHeight, dp.tileHeight);
            kHeightNum = lcm(kHeightNum, dp.ekHeight);
            atLeastOneIsInitialized |= dp.isInitialized;
          }
        }

        if (IsOutputAttribute(attributes))
        {
          DataPlace& dp = dataPlaceMap[make_pair(instanceName, kp.GetPort(j).identifier.identifier)];
          if (!IsFixedAttribute(attributes))
          {
            //set the k-element size denominators to the least common multiples of the output ek sizes
            kWidthDenom = lcm(kWidthDenom, dp.ekWidth);
          }

          if (!IsStaticAttribute(attributes))
          {
            kHeightDenom = lcm(kHeightDenom, dp.ekHeight);
          }
        }
      }

      int multTileHeight = tileHeight * kHeightDenom;
      int multTileWidth = tileWidth * kWidthDenom;

      for (int j = 0; j < kp.GetPortCount(); ++j)
      {
        Attributes attr = kp.GetPort(j).attributes.attributes;
        //try to divide the kernel output kernels' tile sizes (width + height) by (kNum/kDenom)
        if (IsOutputAttribute(attr))
        {
          DataPlace& dp = dataPlaceMap[make_pair(instanceName, kp.GetPort(j).identifier.identifier)];
         if (!IsStaticAttribute(attr))
         {
            if ( multTileHeight % /*kHeightNum*/dp.ekHeight != 0)
            {
              char tmp[400];
              sprintf_s(tmp, "input ek height %d not divisible with output element ek height %d: %s ", multTileHeight, dp.ekHeight, kp.GetName().c_str());
              errorMessage = std::string(tmp);
              errorOccured = true;
              return;
            }
            dp.tileHeight = multTileHeight / kHeightNum; // TODO: maybe BUG: shouldn't it rather be /dp.ekHeight
            dp.isInitialized = atLeastOneIsInitialized;
          }
          else
          {
            if (!IsFixedAttribute(attr))
            {
              dp.tileHeight = tileHeight * kHeightDenom / kHeightNum;
            }
          }

          if(!IsFixedAttribute(attr))
          {
            if ((multTileWidth % /*kWidthNum*/dp.ekWidth != 0)/* || (multTileHeight % kHeightNum != 0)*/)
            {
              char tmp[400];
              sprintf_s(tmp, "input ek width %d not divisible with output element ek width %d: %s ", multTileWidth,  kWidthNum, kp.GetName().c_str());
              errorMessage = std::string(tmp);
              errorOccured = true;
              return;
            }
            dp.tileWidth = multTileWidth / kWidthNum;// TODO: maybe BUG: shouldn't it rather be /dp.ekWidth
            dp.isInitialized = atLeastOneIsInitialized;
          }
          else
          {
		    dp.isInitialized = true;
          }
        }
      }
    }

    //use the planned propagation to copy the tile sizes into connected kernels' inputs
    for (unsigned int j = 0; j < kernelPlan[i].dataPlacePropagations.size(); ++j)
    {
      std::string a = kernelPlan[i].dataPlacePropagations[j].first.first + "." + kernelPlan[i].dataPlacePropagations[j].first.second;
      std::string b = kernelPlan[i].dataPlacePropagations[j].second.first + "." + kernelPlan[i].dataPlacePropagations[j].second.second;

      DataPlace& dpA = dataPlaceMap[kernelPlan[i].dataPlacePropagations[j].first];
      DataPlace& dpB = dataPlaceMap[kernelPlan[i].dataPlacePropagations[j].second];

      dpB.tileWidth = dpA.tileWidth;
      dpB.tileHeight = dpA.tileHeight;
	  dpB.isInitialized = dpA.isInitialized;	  
    }
  }

  //propagate the tile size to output graph ports
  for (GraphPortInfoType::iterator it = graphPorts.begin(); it != graphPorts.end(); ++it)
  {
    Attributes attributes = it->second.port.attributes.attributes;
    // if (IsOutputAttribute(attributes))
    {
      DataPlace& dp = dataPlaceMap[make_pair("", it->first)];
      it->second.chunkWidth = dp.tileWidth;
      it->second.chunkHeight = dp.tileHeight;
	  it->second.isInitialized = dp.isInitialized;
    }
  }
}

void ACF_Graph::PrepareDataPlaces()
{
  if (errorOccured)
  {
    return;
  }

  //check whether all input ports are connected to data
  if (graphData.size() != graphPorts.size())
  {
    for (GraphPortInfoType::iterator gpIt = graphPorts.begin(); gpIt != graphPorts.end(); ++gpIt)
    {
      if (graphData.count(gpIt->first) < 1)
      {
        errorOccured = true;
        errorMessage = "data not connected to the graph port: " + gpIt->first;
        return;
      }
    }
  }

  //compute the number of iterations (data tiles) for graph input data
  unsigned int iterations = 0;

  for (GraphDataType::iterator it = graphData.begin(); it != graphData.end(); ++it)
  {
	ACF_DataDesc* pdd = it->second.data;
    bool isIndirect = it->second.isIndirect;
	ACF_DataDesc* offsets = it->second.offsets;

    GraphPortInfo& gpi = graphPorts[it->first];
    Attributes attributes = gpi.port.attributes.attributes;
    //number of iterations implied by the port
    unsigned int height = isIndirect ? offsets->RetHeight() * gpi.chunkHeight : pdd->RetHeight();
    unsigned int portIterations = height / gpi.chunkHeight;
    //iteration count should match across the inputs, static data should be ignored for this check
    if ((portIterations != iterations) && (iterations != 0) && (!IsStaticAttribute(attributes)))
    {
        errorOccured = true;
        char tmp[400];
        sprintf_s(tmp, "incompatible input graph port configurations (portIterations = dataHeight/chunkHeight, i.e. %d = %d/%d) ==> leads to slice count conflict: %d iterations: %s", portIterations, height, gpi.chunkHeight, iterations, it->first.c_str());
        errorMessage = std::string(tmp);
        return;
    }
    else
    {
	  //update the iteration count (won't be changed after setting it once)
      if (!IsStaticAttribute(attributes))
      {
        iterations = portIterations;
      }
    }

    //fixed data shouldn't be checked
    if (IsFixedAttribute(attributes) && !isIndirect)
    {
      continue;
    }

    unsigned int width = isIndirect ? offsets->RetWidth() * gpi.chunkWidth : pdd->RetWidth();
    //connected data/port inconsistencies
    if ((width % gpi.chunkWidth != 0) || (height % gpi.chunkHeight != 0))
    {
      errorOccured = true;
      char tmp[400];
      sprintf_s(tmp, "graph port data size (%d,%d) not divisible by the port chunk size (%d,%d): %s ", width, height, gpi.chunkWidth, gpi.chunkHeight, it->first.c_str());
      errorMessage = std::string(tmp);
      return;
    }

  }
  iterations = iterations == 0 ? 1 : iterations;

  //allocate the memory for all DataPlaces
  for (DataPlaceMapType::iterator it = dataPlaceMap.begin(); it != dataPlaceMap.end(); ++it)
  {
    it->second.tileCount = it->second.staticData /*|| it->second.fixedData */? 1 : iterations;
    int allocateTiles = it->second.tileCount;
    int layoutCUCount = it->second.vectorData ? VSIZE : 1;
    //compute the amount of memory needed
    int dataSize = 1;
    if (it->second.fixedData)
    {
      dataSize = it->second.e0Width * it->second.e0Height * it->second.ekWidth * it->second.ekHeight * PortDataTypeSize(it->second.portDataType) * layoutCUCount * allocateTiles;
    }
    else
    {
      dataSize = allocateTiles *
      CMEMArraySize(it->second.tileWidth, it->second.tileHeight, it->second.padding, it->second.paddingVert, layoutCUCount) *
      PortDataTypeSize(it->second.portDataType) * it->second.e0Width * it->second.e0Height;
    }
    //allocate the memory or don't do anything if the data are already allocated and have the correct size
    if (it->second.actualSize != dataSize)
    {
      it->second.RequestMemory(dataSize);
    }
  }
}

void ACF_Graph::Run()
{
  if (errorOccured)
  {
    return;
  }

  int activeCUCount = -1, activeCUCountFixed = -1;

  //propagate input DataDescriptor data to graph input DataPlaces
  for (GraphDataType::iterator it = graphData.begin(); it != graphData.end(); ++it)
  {
    Attributes attributes = graphPorts[it->first].port.attributes.attributes;

    //check and update the active CU count, the count can be set only once and has to match across non-static vector inputs
    if (IsVectorAttribute(attributes) /*&& !IsFixedAttribute(attributes)*/) // vectorial fixed attributes can have activeCUCount < VSIZE
    {
      if (activeCUCount < 0)
      {
        activeCUCount = it->second.activeCUCount;
      }
      else if (it->second.activeCUCount != activeCUCount)
      {
        errorOccured = true;
		char tmp[400];
		sprintf_s(tmp, "inconsistent CU counts %d vs. %d implied by graph inputs: %s", it->second.activeCUCount, activeCUCount, it->first.c_str());
		errorMessage = std::string(tmp);
        return;
      }
    }
	activeCUCountFixed = IsFixedAttribute(attributes)  && IsVectorAttribute(attributes) && (activeCUCount <= 0) ? APU_VSIZE : activeCUCount;

	if (IsInputAttribute(attributes))
	{
		//get information needed for transforming the data from raw format to CMEM format
		DataPlace& dp = dataPlaceMap[make_pair("", it->first)];
		void* dataPlacePtr = dp.dataPtr;
		GraphDataInfo gdi = it->second;
		ACF_DataDesc* dd = gdi.data;
		void* inputDataPtr = dd->RetDataPtr();
		void* offsetsDataPtr = gdi.isIndirect ? gdi.offsets->RetDataPtr() : 0;
		int dpWidth = dp.e0Width * (dp.fixedData ? dp.ekWidth : dp.tileWidth);
		int dpHeight = dp.tileCount * dp.e0Height * (dp.fixedData ? dp.ekHeight : dp.tileHeight);
		int dpPadding = dp.padding;
		int dpPaddingVert = dp.paddingVert;
		int layoutCUCount = dp.vectorData ? VSIZE : 1;
		unsigned int ddWidth = dd->RetWidth() * dd->RetElementDimX();
		unsigned int ddHeight = dd->RetHeight() * dd->RetElementDimY();
		std::string message;
		APEX2OperationResult result = APEX2_OK;
		/*if (IsFixedAttribute(attributes) && !gdi.isIndirect) {
			if (dd != gdi.originalData)
			{
				CopyChangeWidth(*dd, *gdi.originalData);
			}
			memcpy(dataPlacePtr, inputDataPtr, dp.actualSize);
		}
		else*/ {
			//handle inputs which are not spanning all CUs, use data and originalData pointers
			if (dd != gdi.originalData)
			{
				CopyChangeWidth(*dd, *gdi.originalData);
			}
			if (gdi.offsets != 0 && gdi.offsets != gdi.originalOffsets)
			{
				CopyChangeWidth(*gdi.offsets, *gdi.originalOffsets);
			}

			//copy the data from the input DataDescriptors to input graph DataPlaces including changing the format from raw to CMEM
			//handle both direct inputs and indirect inputs
			switch (dd->RetElementDataType())
			{
			case (icp::DATATYPE_08U) :
			case (icp::DATATYPE_08S) :
								if (!gdi.isIndirect)
								{
									//result = ArrayToCMEMData<int08u>((int08u*)dataPlacePtr, (int08u*)inputDataPtr, dpWidth, dpHeight, dpPadding, layoutCUCount, ddWidth, &message);
									result = ArrayToCMEMDataSym<int08u>((int08u*)dataPlacePtr, (int08u*)inputDataPtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount, ddWidth, &message);
								}
								else
								{
									result = ArrayToCMEMDataIndirect<int08u, int32u>((int08u*)dataPlacePtr, (int08u*)inputDataPtr, dpWidth, dp.tileHeight, dp.tileCount, layoutCUCount, ddWidth, ddHeight, (int32u*)offsetsDataPtr, &message);
								}
								break;
			case (icp::DATATYPE_16U) :
			case (icp::DATATYPE_16S) :
								if (!gdi.isIndirect)
								{
									// result = ArrayToCMEMData<int16u>((int16u*)dataPlacePtr, (int16u*)inputDataPtr, dpWidth, dpHeight, dpPadding, layoutCUCount, ddWidth, &message);
									result = ArrayToCMEMDataSym<int16u>((int16u*)dataPlacePtr, (int16u*)inputDataPtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount, ddWidth, &message);
								}
								else
								{
									result = ArrayToCMEMDataIndirect<int16u, int32u>((int16u*)dataPlacePtr, (int16u*)inputDataPtr, dpWidth, dp.tileHeight, dp.tileCount, layoutCUCount, ddWidth, ddHeight, (int32u*)offsetsDataPtr, &message);
								}
								break;
			case (icp::DATATYPE_32U) :
			case (icp::DATATYPE_32S) :
								if (!gdi.isIndirect)
								{
									//result = ArrayToCMEMData<int32u>((int32u*)dataPlacePtr, (int32u*)inputDataPtr, dpWidth, dpHeight, dpPadding, layoutCUCount, ddWidth, &message);
									result = ArrayToCMEMDataSym<int32u>((int32u*)dataPlacePtr, (int32u*)inputDataPtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount, ddWidth, &message);
								}
								else
								{
									result = ArrayToCMEMDataIndirect<int32u, int32u>((int32u*)dataPlacePtr, (int32u*)inputDataPtr, dpWidth, dp.tileHeight, dp.tileCount, layoutCUCount, ddWidth, ddHeight, (int32u*)offsetsDataPtr, &message);
								}
								break;
			}
			//check whether an error or a warning occurred during transforming the data
			if (result == APEX2_ERROR)
			{
				errorOccured = true;
				errorMessage = message;
				return;
			}
			else if (result == APEX2_WARNING)
			{
				warningOccured = true;
				warningMessages += message + "\n";
			}
		}
	}
	activeCUCount = activeCUCount <= 0 ? activeCUCountFixed : activeCUCount;
  }

  //execute the kernel plan
  for (unsigned int i = 0; i < kernelPlan.size(); ++i)
  {
    //if plan element contains kernel execution
    if (kernelPlan[i].kernelInstance != kernels.end())
    {
      ACF_Kernel& kp = kernelPlan[i].kernelInstance->second;
      std::string instanceName = kernelPlan[i].kernelInstance->first;

      kernel_io_desc kernelIO[MAX_KN_PORTS];

      //compute the iteration count and fill in the kernel_io_desc structures
      unsigned int iterations = 1;
      for (int j = 0; j < kp.GetPortCount(); ++j)
      {
        DataPlace& dp = dataPlaceMap[make_pair(instanceName, kp.GetPort(j).identifier.identifier)];
        kernelIO[j].chunkWidth = dp.tileWidth;
        kernelIO[j].chunkHeight = dp.tileHeight;
        int dataTypeSize = PortDataTypeSize(dp.portDataType);
        kernelIO[j].chunkSpan = (dp.tileWidth + 2 * dp.padding) * dataTypeSize * dp.e0Width;
        int layoutCUCount = IsVectorAttribute(kp.GetPort(j).attributes.attributes) ? VSIZE : 1;
        unsigned char* startPtr = (unsigned char*)dp.dataPtr;
        startPtr += (dp.paddingVert * kernelIO[j].chunkSpan * dp.e0Height + dp.padding * dataTypeSize * dp.e0Width) * layoutCUCount;
        kernelIO[j].pMem = (void*)startPtr;
        iterations = max(dp.tileCount, iterations);
      }


	  {
		  //////////////////////////////////////////////////////////////////////////////////
		  // This is a critical section. Lock the VectorConditionController for other threads, 
		  // for the global condition stack and other global variables not to be corrupted by other threads
		  StartCriticalSection();


		  //set the ACF-accessible info variables
		  _tileWidthInChunks = activeCUCount;
		  _tileFirstCUID = 0;
		  _numInputIterations = iterations;
		  _cuArrayWidth = VSIZE;
		  
		  VectorConditionController::Instance().Init();


		  //iterate through slices
		  for (unsigned int tileNum = 0; tileNum < iterations; ++tileNum)
		  {
			  //set the ACF-accessible info variables
			  _firstTile = (tileNum == 0);
			  _lastTile = (tileNum == iterations - 1);

			  try{
				  //run the kernel using the filled kernel_io_desc structures
				  kp.Run(
					  kernelIO[0], kernelIO[1], kernelIO[2], kernelIO[3], kernelIO[4], kernelIO[5], kernelIO[6], kernelIO[7],
					  kernelIO[8], kernelIO[9], kernelIO[10], kernelIO[11], kernelIO[12], kernelIO[13], kernelIO[14], kernelIO[15]);
			  }
			  catch (...){
				  errorOccured = true;
				  errorMessage = std::basic_string<char>("Error running kernel ") + kp.GetName() + std::string("at iteration ") + std::to_string(tileNum);
				  return;
			  }


			  //update the kernel_io_desc pointers for the next iteration
			  for (int j = 0; j < kp.GetPortCount(); ++j)
			  {
				  DataPlace& dp = dataPlaceMap[make_pair(instanceName, kp.GetPort(j).identifier.identifier)];
				  int layoutCUCount = IsVectorAttribute(kp.GetPort(j).attributes.attributes) ? VSIZE : 1;
				  unsigned char* ptr = (unsigned char*)kernelIO[j].pMem;
				  bool st = IsStaticAttribute(kp.GetPort(j).attributes.attributes);
				  if (!st)
				  {
					  ptr += kernelIO[j].chunkHeight * kernelIO[j].chunkSpan * dp.e0Height * layoutCUCount;
				  }
				  kernelIO[j].pMem = (void*)ptr;
			  }
		  }

		  //set the ACF-accessible info variables
		  _firstTile = false;
		  _lastTile = false;
		  _tileWidthInChunks = 0;
		  _tileFirstCUID = 0;
		  _numInputIterations = 0;
		  _cuArrayWidth = 0;

		  EndCriticalSection();
		  // ///////////////// END of Critical Section
	  }
   }

    //kernel outputs to connected kernels' inputs data propagations
    for (unsigned int j = 0; j < kernelPlan[i].dataPlacePropagations.size(); ++j)
    {
      DataPlace& dpA = dataPlaceMap[kernelPlan[i].dataPlacePropagations[j].first];
      DataPlace& dpB = dataPlaceMap[kernelPlan[i].dataPlacePropagations[j].second];

	  int dpAHeight = 0, dpAWidth = 0, dpBHeight = 0, dpBWidth = 0;

	  //compute the variables needed for transferring the data between the kernel output DataPlaces and connected kernels' input DataPlaces
	  if (dpA.fixedData)
	  {
		  dpAHeight = dpA.e0Height * dpA.ekHeight * (dpA.staticData ? 1 : dpA.tileCount);
		  dpAWidth = dpA.e0Width * dpA.ekWidth;
	  }
	  else
	  {
		  dpAHeight = dpA.e0Height * (dpA.staticData ? dpA.tileHeight : dpA.tileHeight * dpA.tileCount);
		  dpAWidth = dpA.tileWidth * dpA.e0Width;
	  }

	  if (dpB.fixedData)
	  {
		  dpBHeight = dpB.e0Height * dpB.ekHeight * (dpB.staticData ? 1 : dpB.tileCount);
		  dpBWidth = dpB.e0Width * dpB.ekWidth;
	  }
	  else
	  {
		  dpBHeight = dpB.e0Height * (dpB.staticData ? dpB.tileHeight : dpB.tileHeight * dpB.tileCount);
		  dpBWidth = dpB.tileWidth * dpB.e0Width;
	  }
      
      int dpAPaddingX = dpA.padding * dpA.e0Width;
      int dpAPaddingY = dpA.paddingVert * dpA.e0Height;
      int dpBPaddingX = dpB.padding * dpB.e0Width;
      int dpBPaddingY = dpB.paddingVert * dpB.e0Height;
      int paddingsA[4] = {dpAPaddingX, dpAPaddingX, dpAPaddingY, dpAPaddingY};
      int paddingsB[4] = {dpBPaddingX, dpBPaddingX, dpBPaddingY, dpBPaddingY};
      int granularityX = dpB.e0Width;
      int granularityY = dpB.e0Height;

	  int layoutCUCount = dpB.vectorData ? VSIZE : 1;
	  int inImgW = activeCUCount * dpAWidth;

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      auto getOrigDD = [this](DataPlaceKey& dpk) -> ACF_DataDesc* {
          ACF_DataDesc* dd = 0;
          string s = dpk.first;

          if (!s.compare("")) {
              auto itGraphData = graphData.find(dpk.second);
              if (itGraphData != graphData.end()) {
                  dd = itGraphData->second.originalData;
              }
          }

          return dd;
      };

      ACF_DataDesc* ddA = getOrigDD(kernelPlan[i].dataPlacePropagations[j].first);
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //copy the data and change the padding if necessary
      switch (dpB.portDataType)
      {
        case d08u:
        case d08s:
          ChangePaddingCMEM<int08u>((int08u*)dpB.dataPtr, (int08u*)dpA.dataPtr, ddA, inImgW, dpBWidth, dpBHeight, paddingsB, paddingsA, layoutCUCount, granularityX, granularityY);
          break;
        case d16u:
        case d16s:
			ChangePaddingCMEM<int16u>((int16u*)dpB.dataPtr, (int16u*)dpA.dataPtr, ddA, inImgW, dpBWidth, dpBHeight, paddingsB, paddingsA, layoutCUCount, granularityX, granularityY);
          break;
        case d32u:
        case d32s:
            ChangePaddingCMEM<int32u>((int32u*)dpB.dataPtr, (int32u*)dpA.dataPtr, ddA, inImgW, dpBWidth, dpBHeight, paddingsB, paddingsA, layoutCUCount, granularityX, granularityY);
          break;
      }
    }
  }

  //propagate output data to graph outputs
  for (GraphDataType::iterator it = graphData.begin(); it != graphData.end(); ++it)
  {
    Attributes attributes = graphPorts[it->first].port.attributes.attributes;
    if (IsOutputAttribute(attributes))
    {
      //get information needed for transforming the data from CMEM format to raw format
      DataPlace& dp = dataPlaceMap[make_pair("", it->first)];
      void* dataPlacePtr = dp.dataPtr;
      GraphDataInfo gdi = it->second;
      ACF_DataDesc * dd = gdi.data;
      void* outputDataPtr = dd->RetDataPtr();
      void* offsetsDataPtr = gdi.isIndirect ? gdi.offsets->RetDataPtr() : 0;
      int dpWidth = dp.e0Width * (dp.fixedData ? dp.ekWidth : dp.tileWidth);
      int dpHeight = dp.tileCount * dp.e0Height * (dp.fixedData ? dp.ekHeight : dp.tileHeight);
      int dpPadding = dp.padding;
	  int dpPaddingVert = dp.paddingVert;
      int layoutCUCount = dp.vectorData ? VSIZE : 1;

      //copy the data from the output graph DataPlaces to output DataDescriptors including changing the format from CMEM to raw
      switch (dd->RetElementDataType())
      {
	  case (icp::DATATYPE_08U):
	  case (icp::DATATYPE_08S) :
		  CMEMDataToArray<int08u>((int08u*)outputDataPtr, (int08u*)dataPlacePtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount);
          break;
	  case (icp::DATATYPE_16U) :
	  case (icp::DATATYPE_16S) :
          CMEMDataToArray<int16u>((int16u*)outputDataPtr, (int16u*)dataPlacePtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount);
          break;
	  case (icp::DATATYPE_32U) :
	  case (icp::DATATYPE_32S) :
          CMEMDataToArray<int32u>((int32u*)outputDataPtr, (int32u*)dataPlacePtr, dpWidth, dpHeight, dpPadding, dpPaddingVert, layoutCUCount);
          break;
      }

      //handle outputs which are not spanning all CUs, use data and originalData pointers
      if (dd != gdi.originalData)
      {
        CopyChangeWidth(*gdi.originalData, *dd);
      }
    }
  }
}

ACF_Graph::DataPlace* ACF_Graph::GetDataPlacePtr(const std::string& kernelInstanceName, const std::string& portName)
{
  //return a pointer to requested DataPlace (debug functionality)
  DataPlaceKey dpKey = make_pair(kernelInstanceName, portName);
  DataPlaceMapType::iterator it = dataPlaceMap.find(dpKey);
  if (it != dataPlaceMap.end())
  {
    return &(it->second);
  }
  return 0;
}

ACF_Graph::~ACF_Graph()
{
  //deallocate the extra data used during handling the input/output data which don't span all CUs
  for (GraphDataType::iterator it = graphData.begin(); it != graphData.end(); ++it)
  {
    ACF_DataDesc* data = it->second.data;
	ACF_DataDesc* originalData = it->second.originalData;
	ACF_DataDesc* offsets = it->second.offsets;
	ACF_DataDesc* originalOffsets = it->second.originalOffsets;

	if (data != originalData && data != 0)
	{
		free(data->RetDataPtr());
		delete data;
		data = 0;
	}

    if (offsets != originalOffsets)
    {
		free(offsets->RetDataPtr());
	    delete offsets;
	    offsets = 0;
    }

	if (originalData != 0)
	{
		delete originalData;
		originalData = 0;
	}

	if (originalOffsets != 0)
	{
		delete originalOffsets;
		originalOffsets = 0;
	}
  }
}

void ACF_Graph::CopyChangeWidth(ACF_DataDesc& destination, ACF_DataDesc& source)
{
  //copy the data and include extra columns corresponding to non-active CUs
  int destinationElementsX = destination.RetWidth();
  int depth = 1;// destination.RetDimensions();
  int destinationElementsY = destination.RetHeight()*depth;

  int sourceElementsX = source.RetWidth();
  int elementBytesX = destination.RetElementDimX() * RetDataTypeSizeInBytes(destination.RetElementDataType());
  int elementSizeY = destination.RetElementDimY();
  uint8_t* srcPtr = (uint8_t*)source.RetDataPtr();
  uint8_t* dstPtr = (uint8_t*)destination.RetDataPtr();
	  for (int by = 0; by < destinationElementsY * elementSizeY; ++by)
	  {
		  uint8_t* srcRow = &(srcPtr[by * (source.RetSpan() / source.RetElementDimY())]);
		  uint8_t* dstRow = &(dstPtr[by * (destination.RetSpan() / destination.RetElementDimY())]);
		  for (int rowElementIndex = 0; rowElementIndex < (int)destinationElementsX; ++rowElementIndex)
		  {
			  int srcRowElementIndex = rowElementIndex < sourceElementsX ? rowElementIndex : sourceElementsX - 1;
			  uint8_t* srcE = &(srcRow[srcRowElementIndex * elementBytesX]);
			  uint8_t* dstE = &(dstRow[rowElementIndex * elementBytesX]);
			  for (int bi = 0; bi < elementBytesX; ++bi)
			  {
				  dstE[bi] = srcE[bi];
			  }
		  }
	  }
}
