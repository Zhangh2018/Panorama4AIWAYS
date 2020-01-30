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

#ifndef ACFKERNEL_HPP
#define ACFKERNEL_HPP

#include <acf_node.hpp>
#include <stdint.h>
#include <map>

class ACF_Kernel : public ACF_Node
{

public:

   ACF_Kernel();
   ~ACF_Kernel();

   int32_t Initialize(std::string lKernelDatabaseIdentifier);
   ACF_NODE_TYPE RetNodeType();
   std::string RetDBIdentifier();
   ACF_Port* RetPortBasedOnParamIndex(int32_t lParamIndex);

   std::string mDBIdentifier;
   int32_t mCascadeDepth;
   int32_t mLaunchDelay; //in iterations
   int32_t mExtraIterToSatisfyOd; //takes into account OD for all output ports (used to adjust how many times the kernel must be executed)
   ed_desc mED;
   ed_desc mBaseED;

   std::map<int32_t, ACF_Port*> mKernelParamIndexToPortMap; //map kernel parameter index to port
};

#endif /* ACFKERNEL_HPP */
