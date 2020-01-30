/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

#ifndef ACFNODE_HPP
#define ACFNODE_HPP

#include <acf_port.hpp>
#include <stdint.h>

#define ACF_MAX_NUM_PORTS_PER_NODE        36

typedef enum _ACF_NODE_TYPE
{
   ACF_NODE_TYPE_UNDEFINED,
   ACF_NODE_TYPE_KERNEL,
   ACF_NODE_TYPE_GRAPH
} ACF_NODE_TYPE;

class ACF_Node
{
public:

   ACF_Node();
   ~ACF_Node();

   ACF_Port* AddInputPort(std::string lPortIdentifier);  //add an input port with the identifier 'lPortIdentifier' to the node
   ACF_Port* AddOutputPort(std::string lPortIdentifier); //add an output port with the identifier 'lPortIdentifier' to the node
   ACF_Port* Port(std::string lPortIdentifier);     //return pointer to port specified by 'lPortIdentifier'
   int32_t SetIdentifier(std::string lNodeIdentifier); //set identifier associated with the node to 'lNodeIdentifier'
   std::string& RetIdentifier();
   int32_t RetNumInputPorts();
   int32_t RetNumOutputPorts();
   int32_t RetNumPorts();
   ACF_Port* InputPort(int32_t lIndex);
   ACF_Port* OutputPort(int32_t lIndex);

   virtual ACF_NODE_TYPE RetNodeType() = 0;

private:

   int32_t IsPortIdentifierUnique(std::string lPortIdentifier);

   std::string mIdentifier; //node identifier

   int32_t mNumInputPorts;
   ACF_Port* mInputPortArray; //allocated on heap w/ new

   int32_t mNumOutputPorts;
   ACF_Port* mOutputPortArray; //allocated on heap w/ new
};

#endif /* ACFNODE_HPP */
