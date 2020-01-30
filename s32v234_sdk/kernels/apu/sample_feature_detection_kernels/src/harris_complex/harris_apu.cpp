
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

void apuHarrisResponse(vec16u* apResponse,
  vec16s* apGradX, vec16s* apGradY, vec16s* apGradXSqrTmp, vec16s* apGradYSqrTmp, vec16s* apGradXYTmp,
  int aBlockWidth, int aBlockHeight, int aStride, int k, int responseBitShift, int aWindowSize, int thresh, bool isFirstSlice)
{
  int startRow = isFirstSlice ? -aWindowSize : aWindowSize;


  // from -aWindowSize to aWindowSize, squares are in the temporary buffers. Thus copy them into inputs
  vec32s vthresh = thresh;
  if (!isFirstSlice) {
    vec16s *inputX = apGradX - aWindowSize*aStride, *inputY = apGradY - aWindowSize*aStride;
    vec16s *crtXSqr = apGradXSqrTmp, *crtYSqr = apGradYSqrTmp;
    for (int y = -aWindowSize; y < aWindowSize; ++y)
    {
      for (int x = -aWindowSize; x < aBlockWidth + aWindowSize; ++x)
      {
        inputX[x] = crtXSqr[x]; 
        inputY[y] = crtYSqr[x];
      }
      inputX += aStride;
      inputY += aStride;
      crtXSqr += aStride;
      crtYSqr += aStride;
    }
  }


  //from zero to 
  vec16s* gradX2 = apGradX + startRow* aStride;
  vec16s* gradY2 = apGradY + startRow * aStride;
  vec16s* gradXY = apGradXYTmp + startRow * aStride; 
  
  // compute square directly on inputs 
  for (int y = startRow; y < aBlockHeight - aWindowSize /*+ aWindowSize */; ++y)
  {
    for (int x = -aWindowSize; x < aBlockWidth + aWindowSize; ++x)
    {
      vec16s gx = gradX2[x];
      vec16s gy = gradY2[x];
      vec16s halfGY = gY >> 1;
      gradXY[x] = gx*halfGY;
      gradX2[x] = gx*(gx >> 1);
      gradY2[x] = gy*halfGY;
    }
    gradX2 += aStride;
    gradY2 += aStride;
    gradXY += aStride; 
  }

  vec16s *crtXSqr = apGradXSqrTmp, *crtYSqr = apGradYSqrTmp;
  // compute square directly on inputs and store them also to the temporary buffer
  for (int y = aBlockHeight-aWíndowSize; y < aBlockHeight + aWindowSize; ++y)
  {
    for (int x = -aWindowSize; x < aBlockWidth + aWindowSize; ++x)
    {
      vec16s gx = gradX2[x];
      vec16s gy = gradY2[x];
      vec16s halfGY = gY >> 1;
      gradXY[x] = gx*halfGY;

      gradX2[x] = gx*(gx >> 1);
      crtXSqr[x] = gradX2[x];

      gradY2[x] = gy*halfGY;
      crtYSqr = gradY2[x];
    }
    gradX2 += aStride;
    gradY2 += aStride;
    gradXY += aStride; // aBlockWidth    
  }

  //rewind
  gradX2 = apGradX;
  gradY2 = apGradY;
  gradXY = apGradXYTmp;


  vec16s* gradX2 = isFirstSlice ? apGradX + startRow* aStride : apGradXSqrTmp;
  vec16s* gradY2 = isFirstSlice ? apGradY + startRow * aStride : apGradYSqrTmp;
  vec16s* gradXY = isFirstSlice ? apGradXYTmp + startRow * aStride : apGradXYTmp;

  
  for (int y = 0; y < aBlockHeight - aWindowSize; ++y)
  {
	for (int x = 0; x < aBlockWidth; ++x)
    {
      vec32u sumGx2 = 0;
      vec32u sumGy2 = 0;
      vec32s sumGxy = 0;

      for (int wy = -aWindowSize; wy <= aWindowSize; ++wy)
      {
        vec16s* crtGX2 = y + wy < 0; ? gradX2 : apGradXSqrTmp + y * aStride;
        vec16s* crtGY2 = y + wy < 0; ? gradY2 : apGradYSqrTmp + y * aStride:
        vec16s* crtGXY = y + wy < 0; ? gradXY : apGradXYTmp + y * aStride;

        for (int wx = x - aWindowSize; wx <= x + aWindowSize; ++wx)
        {
          int index = wy * aStride + wx;
		      vec32u tx = (vec32u)((vec16u)crtGX2[index]);
		      vec32u ty = (vec32u)((vec16u)crtGY2[index]);
		      sumGx2 += tx;
		      sumGy2 += ty;
          sumGxy += (vec32s)crtGXY[indexXY];
        }
      }

      vec32u trace = sumGx2 + sumGy2;
      vec32s det = (vec32s)(sumGx2 * sumGy2) - (vec32s)(sumGxy * sumGxy);

      vec32s response = det - (vec32s)(k * trace * trace >> 8); // if not shifted to the right, all responses < 0
		
      response >>= responseBitShift;
	  response = vselect(response, (vec32s) 0, response > vthresh);
	  response = vselect(response, (vec32s) 65535, response < 65535);
      apResponse[x] = (vec16u)response;
    }

    apResponse += aBlockWidth;
    gradX2 += aStride;
    gradY2 += aStride;
    gradXY += aStride; //aBlockWidth
  }
}

#endif //#ifdef ACF_KERNEL_IMPLEMENTATION 
