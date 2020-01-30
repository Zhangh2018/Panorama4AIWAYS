/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2011-2017 NXP
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

/*******************************************************************************
 *  Project : Maple
 *  Description :
 *     Reference Code for dewarp [and decimation]
 *     This code has been split from CIMG_RefDewarp.cpp.  This code should
 *    remain independant of ICP/ICL, while portions which use the 
 *    functions and structures belong in CIMG_RefDewarp.cpp.
 *****************************************************************************
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "icp_data.h"
#include "apexcv_pro_remap.h"			// for apexcv::INTER_TYPE


#include "cimg_refdewarp.h"


const uint32_t SLICE_HEIGHT = 4;

 // RGBA8888
#define BYTES_PER_PIX_R_G_B     (4)
#define BYTES_PER_PIX_R_G_B_SHIFT     (2)
#define BYTES_PER_PIX_GREYSCALE (1)

// This value was tuned  for previous architecture.
#define NN_TILE_WIDTH  (160)               



//
// Grayscale bilinear interpolation
//
// There is no selection of neighbours like in YUV422
// DeltaX and DeltaY is a fraction coded over the full 8 bytes
//
void fixedPointBilinGrayscale( unsigned int A, unsigned int B, unsigned int C, unsigned int D, 
                               unsigned int deltaX, unsigned int deltaY, uint8_t & Y)
{
   uint32_t Mtop, Mbot;
   // Luminance interpolation in X and Y axis
   // Statistics with a random image
   //Stats were taken prior to making changes to dewarpOpenCVgreyscale
   // where the generation of mapX and mapY was rounded by 128
   // which caused the max error to go down to 5.
   //
   // Ytop/bottom = Yn-1 + DeltaX *(Yn - Yn-1) 
   // Y = Ytop + DeltaY *(Ybot - Ytop) 
   //
   //Mtop = (A + (((deltaX * (B-A)) + 128)>>8));
   //Mbot = (C + (((deltaX * (D-C)) + 128)>>8));                    // These are not exactly the same operation in X and Y
   //Y = (uint8_t)((((Mtop<<8) + ((Mbot-Mtop)*deltaY))+128) >>8);   // I wonder if this explains JIRA ICPSDK-466 for YUV422
   //--------------------
   //# err: 179541
   //Max err: 6
   //sum SQ err: 338605
   //sum err: -2675
   //--------------------


   //Mtop = (((A<<8) + (deltaX * (B-A)) + 128)>>8);
   //Mbot = (((C<<8) + (deltaX * (D-C)) + 128)>>8);
   //Y = (uint8_t)((((Mtop<<8) + ((Mbot-Mtop)*deltaY))+128) >>8);
   //--------------------
   //# err: 179541
   //Max err: 6
   //sum SQ err: 338605
   //sum err: -2675
   //--------------------
   

   //Mtop = (A + (((deltaX * (B-A)) + 128)>>8));
   //Mbot = (C + (((deltaX * (D-C)) + 128)>>8));
   //Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+128)>>8));
   //--------------------
   //# err: 179541
   //Max err: 6
   //sum SQ err: 338605
   //sum err: -2675
   //--------------------

   
   //Mtop = (A + (((deltaX * (B-A)) + 0)>>8));
   //Mbot = (C + (((deltaX * (D-C)) + 0)>>8));
   //Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+0)>>8));
   //--------------------
   //# err: 235799
   //Max err: 7
   //sum SQ err: 636885
   //sum err: 301777
   //--------------------

   
   //Mtop = (A + (((deltaX * (B-A)) + 128)>>8));
   //Mbot = (C + (((deltaX * (D-C)) + 128)>>8));
   //Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+0)>>8));
   //--------------------
   //# err: 197553
   //Max err: 6
   //sum SQ err: 412894
   //sum err: 149742
   //--------------------
   
   
   //Mtop = (A + (((deltaX * (B-A)) + 0)>>8));
   //Mbot = (C + (((deltaX * (D-C)) + 0)>>8));
   //Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+128)>>8));
   //--------------------
   //# err: 197766
   //Max err: 6
   //sum SQ err: 412658
   //sum err: 149258
   //--------------------
   
  
   // Currently does not match .sel code, see below.
   Mtop = (A + (((deltaX * (B-A)) + 128)>>8));
   Mbot = (C + (((deltaX * (D-C)) + 128)>>8));
   Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+128)>>8));
}

// matches .sel for RGB
void fixedPointBilinRGB( unsigned int A, unsigned int B, unsigned int C, unsigned int D, 
                               unsigned int deltaX, unsigned int deltaY, uint8_t & Y)
{
   uint32_t Mtop, Mbot;
   Mtop = (A + (((deltaX * (B-A)) + 0)>>8));
   Mbot = (C + (((deltaX * (D-C)) + 0)>>8));
   Y = (uint8_t)(Mtop + ((((Mbot-Mtop)*deltaY)+128)>>8));
}



// The image is arranged as tiles
// tileWidth * imageHeight
unsigned int perBlockLocationNoApex(unsigned int pop, unsigned int imageHeight, unsigned int tileWidth, unsigned int imageWidth, unsigned int imageStride)
{
   unsigned int blockSize = tileWidth * imageHeight;
   unsigned int blockNumber = pop / blockSize;
   unsigned int blockWidth = tileWidth;
   // Last tile may have a shorter width than tileWidth
   if ((blockNumber+1) * tileWidth > imageWidth) {
      blockWidth = imageWidth - (blockNumber*tileWidth);
   }
   // top left block corner
   unsigned int blockStart = blockNumber * blockSize;
   unsigned int tile_y = (pop - blockStart) / blockWidth;
   unsigned int tile_x = (pop - blockStart) % blockWidth;
   unsigned int location = ((blockNumber * tileWidth) + tile_x) + (imageStride * tile_y);

   return location;
}

void
simulateGrayDewarpNearNeighbour( 
                uint8_t* dst,
                uint8_t* src,
                const uint32_t* offset,
                const uint8_t* /*delta*/,
                int /*inputStride*/, int /*inputHeight*/,
                int outputWidth, int outputHeight,
                int /*outputStride*/,
                int /*sliceHeight*/)
{
   // Place a black pixel in the top left corner 
//   uint32_t * srcCorner = (uint32_t *) src;
//   uint32_t backupCornerGray = *srcCorner;
//   *srcCorner = 0;

   const int imgSize = outputWidth * outputHeight * BYTES_PER_PIX_GREYSCALE;
   const int offsetSize = outputWidth * outputHeight;

   memset(dst, 0, imgSize);
   unsigned char * srcPtr = src;
   unsigned char * dstPtr = dst;
//   unsigned int idx;
   for (int pop = 0; pop<offsetSize; pop++) {
      unsigned int srcOffset = offset[pop];
//      idx = perBlockLocationNoApex(pop, outputHeight, (unsigned int)NN_TILE_WIDTH, outputWidth, outputStride);
      // ACF2 - no mangling of offset image
      int idx = pop;
      dstPtr[idx] = srcPtr[srcOffset];
   }
//   *srcCorner = backupCornerGray;
}


void
simulateGrayDewarpBilinear( 
                uint8_t* dst,
                uint8_t* src,
                const uint32_t* offset,
                const uint8_t* delta,
                int inputStride, int /*inputHeight*/,
                int outputWidth, int outputHeight,
                int outputStride,
                int /*sliceHeight*/)
{
   // Place a black pixel in the top left corner 
//   uint32_t * srcCorner = (uint32_t *) src;
//   uint32_t backupCornerGray[2] = {srcCorner[0], srcCorner[inputStride>>2]};
//   srcCorner[0] = 0;
//   srcCorner[inputStride>>2] = 0;
  const uint8_t* pdelta = delta;
  const uint32_t* poffset = offset;

//   const int offsetSize = outputWidth * outputHeight;
  for (int j = 0; j < outputHeight; j++)
  {
    for (int i = 0; i < outputWidth; i++)
    {
      unsigned int srcOffset = *poffset++;

      unsigned char srcA = src[srcOffset];
      unsigned char srcB = src[srcOffset + 1];
      unsigned char srcC = src[srcOffset + inputStride];
      unsigned char srcD = src[srcOffset + 1 + inputStride];

      //      int idx = perBlockLocation(pop, DEFAULT_SLICE_HEIGHT, BLOCK_WIDTH, outputWidth, outputStride);
      // ACF2 - no mangling of offset image
      unsigned int deltaX = *pdelta++;
      unsigned int deltaY = *pdelta++;

      uint8_t lum;
      //      fixedPointBilinGrayscale( srcA, srcB, srcC, srcD, deltaX, deltaY, lum);
      fixedPointBilinRGB(srcA, srcB, srcC, srcD, deltaX, deltaY, lum);
      dst[i] = lum;
    }
    dst += outputStride;
  }
//   srcCorner[0] = backupCornerGray[0];
//   srcCorner[inputStride>>2] = backupCornerGray[1];
}


void
simulateGrayDewarp( 
                uint8_t* dst,
                uint8_t* src,
                const uint32_t* offset,
                const uint8_t* delta,
//                ICL_MAP_MODE mapMode,
                apexcv::Remap::INTER_TYPE mapMode,
                int inputStride, int inputHeight,
                int outputWidth, int outputHeight,
                int outputStride,
                int sliceHeight)
{
   // only apexcv::INTER_LINEAR and apexcv::INTER_NN supported
   if (apexcv::Remap::INTER_LINEAR == mapMode) {
      simulateGrayDewarpBilinear(dst,
                                 src,
                                 offset,
                                 delta,
                                 inputStride, inputHeight,
                                 outputWidth, outputHeight,
                                 outputStride,
                                 sliceHeight);
   } else {
      simulateGrayDewarpNearNeighbour(dst,
                                 src,
                                 offset,
                                 delta,
                                 inputStride, inputHeight,
                                 outputWidth, outputHeight,
                                 outputStride,
                                 sliceHeight);
   }
}

// RGB - treat pixels as 8-bit entities, and apply offset/delta to 4 consecutive 8-bit entities.
int
simulateRGBDewarpNearNeighbour( 
                uint32_t* dst,
                uint32_t* src,
                const uint32_t* offset,
                const uint8_t* /*delta*/,
                int /*inputStride*/, int /*inputHeight*/,
                int outputWidth, int outputHeight,
                int outputStride,
                int /*sliceHeight*/)
{
   // Place a black pixel in the top left corner 
   uint32_t * srcCorner = (uint32_t *) src;
   uint32_t backupCornerGray = *srcCorner;
   *srcCorner = 0;

   const int imgSize = outputWidth * outputHeight * BYTES_PER_PIX_R_G_B;
   const int offsetSize = outputWidth * outputHeight;

   memset(dst, 0, imgSize);
   unsigned char * srcPtr = (unsigned char *)src;
   unsigned char * dstPtr = (unsigned char *)dst;
   unsigned int idx;
   for (int pop = 0; pop<offsetSize; pop++) {
      unsigned int srcOffset = offset[pop];
      idx = perBlockLocationNoApex(pop, outputHeight, (unsigned int)NN_TILE_WIDTH, outputWidth, outputStride);
      dstPtr[idx] = srcPtr[srcOffset];
   }
   *srcCorner = backupCornerGray;
   return 0;
}


int
simulateRGBDewarpBilinear( 
                uint8_t* dst,
                uint8_t* src,
                const uint32_t* offset,
                const uint8_t* delta,
                int inputStride, int /*inputHeight*/,
                int outputWidth, int outputHeight,
                int /*outputStride*/,
                int /*sliceHeight*/)
{
   // Place a black pixel in the top left corner 
   uint32_t * srcCorner = (uint32_t *) src;
   uint32_t backupCornerGray[2] = {srcCorner[0], srcCorner[inputStride>>2]};
//   srcCorner[0] = 0;
//   srcCorner[inputStride>>2] = 0;

   // Outer loop, one interation per 32-bit RGBA pixel
   const int offsetSize = outputWidth * outputHeight;
   for (int pop = 0; pop<offsetSize; pop++) {
//      int idx = perBlockLocation(pop, DEFAULT_SLICE_HEIGHT, BLOCK_WIDTH, outputWidth, outputStride);
      // ACF2 - no mangling of offset image
      int idx = pop;

      unsigned int deltaX = delta[2*idx];
      unsigned int deltaY = delta[1+(2*idx)];
     unsigned int srcOffset = offset[pop];

     // inner loop, treat the current pixel as 4 8-bit entities
     for (int element=0; element < BYTES_PER_PIX_R_G_B; element++ ) {
        unsigned char srcA = src[srcOffset*BYTES_PER_PIX_R_G_B + element];
        unsigned char srcB = src[(srcOffset+1)*BYTES_PER_PIX_R_G_B + element];
        unsigned char srcC = src[(srcOffset+inputStride)*BYTES_PER_PIX_R_G_B + element];
        unsigned char srcD = src[(srcOffset+1+inputStride)*BYTES_PER_PIX_R_G_B + element];
   
        uint8_t lum;
//		  fixedPointBilinGrayscale( srcA, srcB, srcC, srcD, deltaX, deltaY, lum);
        fixedPointBilinRGB( srcA, srcB, srcC, srcD, deltaX, deltaY, lum);
        dst[idx*BYTES_PER_PIX_R_G_B + element] = lum;
     }
   }
   srcCorner[0] = backupCornerGray[0];
   srcCorner[inputStride>>2] = backupCornerGray[1];
   return 0;
}


int
simulateRGBDewarp( 
                uint8_t* dst,
                uint8_t* src,
                const uint32_t* offset,
                const uint8_t* delta,
//                ICL_MAP_MODE mapMode,
                apexcv::Remap::INTER_TYPE mapMode,
                int inputStride, int inputHeight,
                int outputWidth, int outputHeight,
                int outputStride,
                int sliceHeight)
{
   // only apexcv::INTER_LINEAR and apexcv::INTER_NN supported
   if (apexcv::Remap::INTER_LINEAR == mapMode) {
      return simulateRGBDewarpBilinear(dst,
                                 src,
                                 offset,
                                 delta,
                                 inputStride, inputHeight,
                                 outputWidth, outputHeight,
                                 outputStride,
                                 sliceHeight);
   } else {
      return simulateRGBDewarpNearNeighbour((uint32_t*)dst,
                                 (uint32_t*)src,
                                 offset,
                                 delta,
                                 inputStride, inputHeight,
                                 outputWidth, outputHeight,
                                 outputStride,
                                 sliceHeight);
   }
}
