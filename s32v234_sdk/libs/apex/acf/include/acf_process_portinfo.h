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

#ifndef ACFPROCESSPORTINFO_H
#define ACFPROCESSPORTINFO_H

#include "acf_datadesc.hpp"
#include <icp_datatype.h>
#include <acf_common.h>
#include <stdint.h>
#include <string>

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

#endif /* ACFPROCESSPORTINFO_H */
