/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
* Copyright 2017 NXP 
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

#ifndef ACFPORT_HPP
#define ACFPORT_HPP

#include <acf_common.h>
#include <stdint.h>
#include <string>

//forward declarations
class ACF_Node;

#define ACF_MAX_NUM_FWD_DIR_CONNECTIONS_PER_PORT 100

typedef enum _ACF_PORT_TYPE
{
   ACF_PORT_TYPE_UNDEFINED,
   ACF_PORT_TYPE_INPUT,
   ACF_PORT_TYPE_OUTPUT
} ACF_PORT_TYPE;

class ACF_PortInfo
{
public:
   ACF_PortInfo();
   ~ACF_PortInfo();

   int32_t RetMaxVertDep();
   int32_t HasSpatialDep();

   e0_desc mE0;
   ek_desc mEK;
   sd_desc mSD;

   int32_t mAttrDmem;
   int32_t mAttrStatic;
   int32_t mAttrFixed;
   int32_t mAttrFifo;

   int32_t mOutputDelayInLines; //in lines of e0
   int32_t mOutputDelayInIter;  //in lines of e0
   
   //delay associated with the port; consumption delay in case of kernel input or DMA output,
   //production delay in case of kernel output or DMA input (in iterations).  this is currently
   //used for determining if input DMAs can be delayed.
   int32_t mDelay; 

   int32_t AttrCmem();
   int32_t AttrDmem();
   int32_t AttrStatic();
   int32_t AttrFixed();
   int32_t AttrFifo();
};

class ACF_Port
{
   friend class ACF_Kernel;
   friend class ACF_Graph;

public:

   ACF_Port();
   ~ACF_Port();

   void SetIdentifier(std::string lIdentifier);
   std::string& RetIdentifier();
   void SetParentNode(ACF_Node* lpParentNode);
   void SetPortType(ACF_PORT_TYPE lPortType); //ACF_PORT_TYPE_INPUT or ACF_PORT_TYPE_OUTPUT
   void SetDelay (int32_t lDelayInIter);
   ACF_PORT_TYPE RetPortType();
   ACF_Node* RetParentNodePtr();
   int32_t RetNumFwdDirConnections();
   ACF_Port* RetFwdDirConnection(int32_t lIndex);
   int32_t ConnectTo(ACF_Port* lpPort);
   ACF_Port* RetBacktrackConnection();
   ACF_PortInfo& RetPortInfo();

private:

   std::string mIdentifier;   //port identifier
   ACF_PORT_TYPE mPortType;   //ACF_PORT_TYPE_INPUT or ACF_PORT_TYPE_OUTPUT
   ACF_Node* mpParentNode;    //pointer to parent node (a port belongs to a single parent node)
   int32_t mNumFwdDirConnections; //number of forward directed connections
   ACF_Port* mFwdDirConnectionArray[ACF_MAX_NUM_FWD_DIR_CONNECTIONS_PER_PORT];
   ACF_Port* mpBacktrackConnection; //applicable to input ports only (points back to the output port feeding it)
   ACF_PortInfo mPortInfo;
};

#endif /* ACFPORT_HPP */
