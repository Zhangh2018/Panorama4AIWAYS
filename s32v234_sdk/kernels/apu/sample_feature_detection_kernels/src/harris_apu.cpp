
/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2013 Freescale Semiconductor;
* Copyright (c) 2017 NXP
*  All Rights Reserved
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
* \file harris_apu.cpp
* \brief harris corner detector implementation for APEX
* \author Igor Aleksandrowicz
* \version
* \date
****************************************************************************/
#ifdef ACF_KERNEL_IMPLEMENTATION

#include "harris_apu.h"

/*****************************************************************************
* functions
*****************************************************************************/
static int computeLog2(int aNr)
{
  int lLog2 = 0;
  while ((aNr >>= 1) > 0) {
    lLog2++;
  }

  return lLog2;
}

void apuHarrisResponse(vec16u* apResponse,
  vec16s* apGradX, vec16s* apGradY, vec16s* apGradXSqrTmp, vec16s* apGradYSqrTmp, vec16s* apGradXYTmp,
  int aBlockWidth, int aBlockHeight, int aStride, int k, int responseBitShift, int aWindowSize, int thresh, bool isFirstSlice)
{

  int startRow = isFirstSlice ? -aWindowSize : aWindowSize;
  // this is the divisor after forming the sum. It is the log2(area of the summing window)
  static int lLog2Wnd = 1;

  // from [-wnd, wnd]
  if (isFirstSlice) 
  {
    lLog2Wnd = (1 + computeLog2(aWindowSize)) << 1;// because we have (2*aWindowSize)^2 elements to sum up.
  }


  //from [wnd, h-wnd]
  vec16s *gradX = apGradX + -aWindowSize* aStride
       , *gradY = apGradY + -aWindowSize* aStride;

  vec16s* gradX2 = apGradXSqrTmp + aWindowSize;
  vec16s* gradY2 = apGradYSqrTmp + aWindowSize;
  vec16s* gradXY = apGradXYTmp + aWindowSize;
  
  // compute square directly on inputs 
  for (int y = -aWindowSize; y < aBlockHeight + aWindowSize ; ++y)
  {
    for (int x = -aWindowSize; x < aBlockWidth + aWindowSize; ++x)
    {
      vec16s gx = gradX[x] >> 1; // take half, to avoid overflow
      vec16s gy = gradY[x] >> 1;
      
      gradXY[x] = gx*gy;
      gradX2[x] = gx*gx;
      gradY2[x] = gy*gy;
    }
    gradX2 += aStride;
    gradY2 += aStride;
    gradXY += aStride; 
    gradX += aStride;
    gradY += aStride;
  }



  //rewind
  gradX2 = apGradXSqrTmp + aWindowSize + aWindowSize* aStride;
  gradY2 = apGradYSqrTmp + aWindowSize + aWindowSize* aStride;
  gradXY = apGradXYTmp + aWindowSize + aWindowSize* aStride;
  vec32s vthresh = thresh;

  vec16u *out = apResponse;
  for (int y = 0; y < aBlockHeight; ++y)
  {
	  for (int x = 0; x < aBlockWidth; ++x)
    {
      vec32u sumGx2 = 0;
      vec32u sumGy2 = 0;
      vec32s sumGxy = 0;

      int index = (y - aWindowSize) * aStride + x;
      vec16s* crtGX2 = gradX2 + index ;
      vec16s* crtGY2 = gradY2 + index ;
      vec16s* crtGXY = gradXY + index ;

      // sum over the window
      for (int wy = -aWindowSize; wy < aWindowSize; ++wy)
      {

        for (int wx = -aWindowSize; wx <  aWindowSize; ++wx)
        {          
		      vec32u tx = (vec32u)((vec16u)crtGX2[wx]);
		      vec32u ty = (vec32u)((vec16u)crtGY2[wx]);
		      sumGx2 += tx;
		      sumGy2 += ty;
          sumGxy += (vec32s)crtGXY[wx];
        }
        crtGX2 += aStride;
        crtGY2 += aStride;
        crtGXY += aStride;
      }

      sumGxy >>= lLog2Wnd;
      sumGx2 >>= lLog2Wnd;
      sumGy2 >>= lLog2Wnd;
      vec32u trace = (sumGx2 + sumGy2) >> 1; //Trace is T/2, trace^2 will be /256
      vec32s det = (vec32s)(sumGx2  * sumGy2 ) - (vec32s)(sumGxy * sumGxy);

      vec32s response = det - (vec32s)((k * trace * trace) >> 6 ); //  shifted to the right, in order to have integer k=[10,42]
		
      response >>= responseBitShift;
      const vec32s upperBnd = 65535;
      const vec32s zeros = 0; 
      response = vselect((vec32s)response, zeros, response > zeros);
      response = vselect((vec32s)response, zeros, response > vthresh);
      response = vselect((vec32s)response, upperBnd, response < upperBnd);
	    out[x] = (vec16u)response;
    }

    out += aBlockWidth;
   }
}




#endif //#ifdef ACF_KERNEL_IMPLEMENTATION 
