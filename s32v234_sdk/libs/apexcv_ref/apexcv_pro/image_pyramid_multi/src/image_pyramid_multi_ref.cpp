/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2017 NXP
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
/*!****************************************************************************
*  \file pyramid_multi.cpp
*  \brief Reference kernels for image pyramid_multi creation algorithm
*******************************************************************************/
#include "image_pyramid_multi_ref.h"
#include "image_pyramid_ref.h"
#include <oal.h>
#include <stdio.h>

template<typename T>
static void create_padded_image(T *src     /*!< Source memory buffer. */,
                                T *dst      /*!< Destination memory buffer. */,
                                int32_t width     /*!< Source image width. */,
                                int32_t height    /*!< Source image height. */,
                                int32_t span      /*!< Source image span in pixels. */,
                                int32_t top       /*!< Padding size in +y direction. */,
                                int32_t bottom    /*!< Padding size in -y direction. */,
                                int32_t left      /*!< Padding size in -x direction. */,
                                int32_t right     /*!< Padding size in +x direction. */
)
{
   int sstr = span;
   int dstr = width+left+right;
   T *buf = dst + top*dstr;

   for (int h = 0; h < height; h++) // For every line
   {
      int w;
      for (w = 0; w < left; w++)
      {
         buf[h*dstr+w] = src[h*sstr];           // Left destination pixels replicate first pixel on source line
      }
      for (; w < left+width; w++)
      {
         buf[h*dstr+w] = src[h*sstr+(w-left)];  // Middle destination pixels replicate corresponding pixel on source line
      }
      for (; w < dstr; w++)
      {
         buf[h*dstr+w] = src[h*sstr+width-1];   // Right destination pixels replicate last pixel on source line
      }
   }

   for (int w = 0; w < dstr; w++)   // For every column
   {
      for (int h = 0; h < top; h++)
      {
         dst[h*dstr+w] = buf[w];                // Top destination pixels replicate corresponding pixel on first source line
      }

      for (int h = top+height; h < top+height+bottom; h++)
      {
         dst[h*dstr+w] = buf[(height-1)*dstr+w];// Top destination pixels replicate corresponding pixel on first source line
      }
   }
}



static int _run_reference_pyramid_down(vsdk::SMat  rSrcDd,
                                       vsdk::SMat  rDstDd)
{
   int lRetVal = 0;

   uint8_t* src = (uint8_t*)(rSrcDd.data);
   int lSrcWidth    = rSrcDd.cols;
   int lSrcHeight   = rSrcDd.rows;
   int lSrcSpan      = rSrcDd.step[0];
   uint8_t* dst = (uint8_t*)(rDstDd.data);

   void *lpSrcPaddedOal = OAL_MemoryAllocFlag((lSrcWidth+4)*lSrcHeight,   OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpMidOal       = OAL_MemoryAllocFlag(lSrcWidth*lSrcHeight*2,     OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpMidPaddedOal = OAL_MemoryAllocFlag(lSrcWidth*(lSrcHeight+4)*2, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);

   if (lpSrcPaddedOal &&
       lpMidOal       &&
       lpMidPaddedOal)
   {
      uint8_t *srcPadded = (uint8_t *)OAL_MemoryReturnAddress(lpSrcPaddedOal, ACCESS_CH_WB);
      int16_t *mid       = (int16_t *)OAL_MemoryReturnAddress(lpMidOal, ACCESS_CH_WB);
      int16_t *midPadded = (int16_t *)OAL_MemoryReturnAddress(lpMidPaddedOal, ACCESS_CH_WB);

      int32_t sstr = lSrcWidth+2+2;
      int32_t dstr = lSrcWidth;

      create_padded_image(src, srcPadded, lSrcWidth, lSrcHeight, lSrcSpan, 0, 0, 2, 2);


      uint8_t *tempSrcPadded = srcPadded + 2;
      int16_t *tempMid = mid;

      for (int h = 0; h < lSrcHeight; h++)   // For every line
      {
         for (int w = 0; w < lSrcWidth; w++) // For every column
         {
            horizontal_gaus(&tempSrcPadded[w], sstr, &tempMid[w], dstr, 1, 1);
         }
         tempSrcPadded += sstr;
         tempMid += dstr;
      }

      create_padded_image(mid, midPadded, lSrcWidth, lSrcHeight, lSrcWidth, 2, 2, 0, 0);
      sstr = lSrcWidth;
      dstr = rDstDd.step[0];
      int16_t *tempMidPadded = midPadded + 2*lSrcWidth;
      for (int h = 0; h < lSrcHeight>>1; h++)
      {
         for (int w = 0; w < lSrcWidth>>1; w++)
         {
            vertical_gaus_and_reduce(&tempMidPadded[w<<1], sstr, &dst[w], dstr, 1, 1);
         }
         tempMidPadded += sstr*2;
         dst += dstr;
      }
   }
   else
   {
      lRetVal = 1;
   }

   //! [free memory]
   if (lpSrcPaddedOal)     OAL_MemoryFree(lpSrcPaddedOal);
   if (lpMidOal)           OAL_MemoryFree(lpMidOal);
   if (lpMidPaddedOal)     OAL_MemoryFree(lpMidPaddedOal);
   //! [free memory]

   return lRetVal;
}



namespace ref {



PyramidMultiCreation::PyramidMultiCreation()
{
}



PyramidMultiCreation::~PyramidMultiCreation()
{
}



int PyramidMultiCreation::process(vsdk::SUMat  src,   /*!< Source memory buffer. Datatype is ICP_DATATYPE_08U. */
                                  vsdk::SUMat*  dst,   /*!< Destination memory buffer. */
                                  bool isPyramidUp)
{
   int intResult = -1;

   if (isPyramidUp == false)
   {
      vsdk::SUMat* lDd[MAX_PYRAMID_MULTI_LEVELS_REF+1] = { &src, 0 };

      for(uint32_t i = 1; i <= MAX_PYRAMID_MULTI_LEVELS_REF; ++i)
      {
         lDd[i] = &(dst[i-1]);

         intResult = _run_reference_pyramid_down(lDd[i-1]->getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED),
                                                 lDd[i]->getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED));
      }
   }

   return intResult;
}


} /* namespace apex */

