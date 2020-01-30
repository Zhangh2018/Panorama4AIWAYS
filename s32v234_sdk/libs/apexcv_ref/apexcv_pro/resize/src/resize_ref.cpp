/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2015-2017 NXP
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

// ACF HOST LIB
#include <icp_data.h>
#include <oal.h>

#include <iostream>
#include <cstdio>

#include "reference_resize.h"
#include <resize_ref.h>
#include <cmath>

///// C++ interface //////////////////////////////////////////////////
namespace ref{

Resize::Resize():mSrcImagePtr(0),mDestImagePtr(0),mInitState(0)
{
}

Resize::~Resize()
{
}


int Resize::process(vsdk::SUMat& src,
                    vsdk::SUMat& dst)
{
   int intResult = -1;

   if(dst.empty() == true)
   {
      vsdk::SUMat lDest_umat_temp0(dst.rows,
                                  dst.step[0],
                                  VSDK_CV_8UC1);
      vsdk::Rect lRect(0,
                       0,
                       dst.cols,
                       dst.rows);

      vsdk::SUMat lDest_umat_temp1(lDest_umat_temp0, lRect);

      dst = lDest_umat_temp1;
   }

   if (src.cols == dst.cols &&
       src.rows == dst.rows)
   {
      dst = src;
      intResult = 0;
   }
   else if (src.cols == dst.cols)
   {
      vertical_Scale_08U((uint8_t*)(src.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED).data), src.cols, src.rows, src.step[0],
                         (uint8_t*)(dst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data), dst.cols, dst.rows, dst.step[0]);
      intResult = 0;
   }
   else if (src.rows == dst.rows)
   {
      horizontal_Scale_08U((uint8_t*)(src.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED).data), src.cols, src.rows, src.step[0],
                           (uint8_t*)(dst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data), dst.cols, dst.rows, dst.step[0]);
      intResult = 0;
  }
   else
   {
      if (0.75*(float)src.cols*(float)src.rows+(float)(src.cols*dst.rows) < (float)(src.cols*src.rows)+(float)dst.cols*(float)src.rows*0.75)
      {
         void* tmp_OAL = OAL_MemoryAllocFlag(dst.rows * src.cols, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

         if (tmp_OAL != NULL)
         {
            vertical_Scale_08U((uint8_t*)(src.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED).data), src.cols, src.rows, src.cols,
                               (uint8_t*)OAL_MemoryReturnAddress(tmp_OAL, ACCESS_CH_WB), src.cols, dst.rows, src.cols);
            horizontal_Scale_08U((uint8_t*)OAL_MemoryReturnAddress(tmp_OAL, ACCESS_CH_WB), src.cols, dst.rows, src.cols,
                                 (uint8_t*)(dst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data), dst.cols, dst.rows, dst.cols);

            OAL_MemoryFree(tmp_OAL);

            intResult = 0;
         }
      }
      else
      {
         void* tmp_OAL = OAL_MemoryAllocFlag(dst.cols * src.rows, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

         if (tmp_OAL != NULL)
         {
            horizontal_Scale_08U((uint8_t*)(src.getMat(vsdk::ACCESS_RW  | OAL_USAGE_CACHED).data), src.cols, src.rows, src.cols,
                                 (uint8_t*)OAL_MemoryReturnAddress(tmp_OAL, ACCESS_CH_WB), dst.cols, src.rows, dst.cols);
            vertical_Scale_08U((uint8_t*)OAL_MemoryReturnAddress(tmp_OAL, ACCESS_CH_WB), dst.cols, src.rows, dst.cols,
                               (uint8_t*)(dst.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED).data), dst.cols, dst.rows, dst.cols);
            OAL_MemoryFree(tmp_OAL);

            intResult = 0;
         }
      }
   }

   //dst_pad_oal = dst.GetDataPtr();

   return intResult;
}

}
