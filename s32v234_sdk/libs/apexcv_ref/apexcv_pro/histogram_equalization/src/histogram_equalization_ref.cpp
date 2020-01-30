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
/*!*********************************************************************************
*  \file apexcv_histogram_ref.cpp
*  \brief Reference code for histogram kernels
***********************************************************************************/

#include <stdint.h>

#include <histogram_equalization_ref.h>

/* Histogram generation */
/* Required to have the hst values set to zero before hand */
void gen_histogram_ref(uint32_t* hst, const uint8_t* src, int ss, int sw, int sh)
{
  for (int y = 0; y < sh; ++y)
  {
    for (int x = 0; x < sw; ++x)
    {
      ++hst[src[y*ss + x]];
    }
  }
}

/* Reference LUT generation, using standard formula */
void generate_lut_ref(uint8_t* lut, uint32_t* hist, int aNumPixels){
  int i = 0;

  /* find first bin with data */
  while (!hist[i])
  {
    i++;
  }
  /* keep divisor */
  int iDelta = aNumPixels - hist[i];
  /* start accumulating pixel count and skip first element
     so we can have (acc - min) / (total - min)
     where sum = (acc - min) //substract by omission
     and   delta = (total - min)
     */
  uint32_t sum = 0;
  lut[i++] = 0;
  for (; i < 256; i++)
  {
    sum += hist[i];
    lut[i] = (uint8_t)((((double)(sum * 255)) / iDelta) + 0.5f);//0.5 is used to round
  }
}

/* Reference LUT generation, with no floating point and division */
void apu_generate_lut_impl_ref
(
uint8_t* lut,
uint32_t* hist,
uint32_t numPixels
)
{
  int i = 0;
  uint32_t firstBin = hist[0];
  /* find first bin with data */
  while (!firstBin)
  {
    firstBin = hist[++i];
  }
  /* start accumulating pixel count and skip first element
  so we can have (acc - min) / (total - min)
  where sum = (acc - min) //substract by omission
  and   divisor = (total - min)
  */
  lut[i] = 0;
  i++;
  /* Code below implements without using division:
  lut[i] = (uint8_t)((float)(sum * 255)/(numPixels - firstBin) + .5);
  */
  int tgtBin = 0;
  uint32_t sum = 0;
  uint32_t divisor;
  uint32_t compareValue;
  divisor = numPixels - firstBin;
  compareValue = (divisor + 1) >> 1;
  for (; i < 256; i++)
  {
    sum += (hist[i] * 255);
    while (sum >= compareValue)
    {
      compareValue += divisor;
      tgtBin++;
    }
    lut[i] = tgtBin;
  }
}


/* Apply the LUT to the image */
void normalize_image_ref(uint8_t* dst, uint8_t* src, uint8_t* lut, int numPixels){
  for (int i = 0; i < numPixels; i++)
    dst[i] = (uint8_t)lut[src[i]];
}