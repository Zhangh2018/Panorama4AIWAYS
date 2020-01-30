#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "remap_transforms.h"

int32_t floatMapTo1DOffset_ref(
		const float *floatMap, // stride assumed to be width?
		const int32_t imageWidth,
		const int32_t imageHeight,
		uint32_t* oneDimOffset,
		uint8_t* delta,
  const int32_t srcWidth,
  const int32_t srcHeight,
  const int32_t srcStride)
{   
   // todo - please confirm type of stride
   int offsetStride = imageWidth;
   int deltaStride = imageWidth*2; 

   // ---BUILD offset and delta images---
   // NB: offset is ACF1-style offset, not local offset.  /todo revisit this
   // Based on 8-bit delta encoding.  Unused for Nearest Neighbour
   const float scl = 256.0; // used to compute 8-bit delta value
   const float rnd = (0.5 / scl);

   for (int y = 0; y < imageHeight; ++y) {
      for (int x = 0; x < imageWidth; ++x) {
         // If fractional part is > delta value 0xff, round up offset count
         float sx = floatMap[2*(y*imageWidth + x)] + rnd;
         float sy = floatMap[2*(y*imageWidth + x) + 1] + rnd;

         // Padding rules...
         if (sx < 0.0)
            sx = 0.0;
         if (sx > (srcWidth - 1))
            sx = (srcWidth - 1);
         if (sy < 0.0)
            sy = 0.0;
         // source image height is not known
         if (sy > (srcHeight - 1))
            sy = (srcHeight - 1);

         // bilinear interpolation
         int32_t xi = (int32_t) floor(sx);
         int32_t xf = (int32_t) floor(scl * (sx - (float) xi));
         int32_t yi = (int32_t) floor(sy);
         int32_t yf = (int32_t) floor(scl * (sy - (float) yi));

         const uint32_t offsetVal = (yi * srcStride) + xi;
         oneDimOffset[x] = offsetVal;
		 // Resize delta values
         delta[2 * x + 0] = xf;
         delta[2 * x + 1] = yf;
      }
      oneDimOffset += offsetStride;
	  delta += deltaStride;    // 2 values per pixel
   }

   return 0;
}

// Determine the source block size required to accommodate the given destination block size, given the 1-d offset table (derived from Q16.16 map)
int32_t getSourceBlockSize_ref(
      uint32_t* oneDimOffset,
	  uint32_t dstImageWidth,
	  uint32_t dstImageHeight,
      const int32_t dstBW,
      const int32_t dstBH,
      const int32_t srcStride,
      int32_t &srcBW,
      int32_t &srcBH)
{
   int32_t rVal = 0;

   // Initialize source block size - largest block seen so far
   srcBW = 0;
   srcBH = 0;

   // Areas not processed in the "last" block - right or bottom
   int excessPixelsRight = dstImageWidth % dstBW;
   int excessPixelsBottom = dstImageHeight % dstBH;

   // For each block
   for (uint32_t i = 0; i < dstImageHeight; i += dstBH) {
      int currentBlockHeight = dstBH;
      if ((i > (dstImageHeight - dstBH)) && (excessPixelsBottom > 0)) {
         currentBlockHeight = excessPixelsBottom;
      }
      for (uint32_t j = 0; j < dstImageWidth; j += dstBW) {
         unsigned int currentBlockWidth = dstBW;
         if ((j > (dstImageWidth - dstBW)) && (excessPixelsRight > 0)) {
            currentBlockWidth = excessPixelsRight;
         }

         // Min and max X and Y values observed, in source image offsets
         unsigned int xMin = 0xffffffff;				// set to very large integer
         unsigned int xMax = 0;
         unsigned int yMin = 0xffffffff;				// set to very large integer
         unsigned int yMax = 0;

         // We are parsing each offset in the destination block
         int blkIndex = i * dstImageWidth + j;

         // For each pixel in the block
         for (int k = 0; k < currentBlockHeight; k++) {
            // Offset of first pixel in block row
            int offsetIndex = blkIndex + k * dstImageWidth;

            for (unsigned int l = 0; l < currentBlockWidth; l++) {

               unsigned int xMap = (oneDimOffset[offsetIndex])
                     % srcStride;
               unsigned int yMap = (oneDimOffset[offsetIndex])
                     / srcStride;

               // Compare with current min/max
               if (xMap < xMin) {
                  xMin = xMap;
               }
               if (xMap > xMax) {
                  xMax = xMap;
               }

               if (yMap < yMin) {
                  yMin = yMap;
               }
               if (yMap > yMax) {
                  yMax = yMap;
               }

               offsetIndex++;
            } // next pixel in row
         } // next pixel row in block

         // Block size
         int blockW = xMax - xMin + 1;
         int blockH = yMax - yMin + 1;

         if (blockW > srcBW) {
            srcBW = blockW;
         }
         if (blockH > srcBH) {
            srcBH = blockH;
         }

      } // next block in row
   } // next row of blocks

   return rVal;
}

int32_t getSourceBlockOffsets_ref(
		uint32_t* oneDimOffset,
		uint32_t dstImageWidth,
		uint32_t dstImageHeight,
		const int32_t dstBW,
		const int32_t dstBH,
		const int32_t srcStride,
		uint32_t* blockOffset)
{
   unsigned int rVal = 0;

   // Offset of the top left corner of the block
   uint32_t offsetTL = 0;
   
   // RetIcpDataTypeSizeInBytes(lSrc.RetContigDataDesc(0).RetElementDataType())
   // todo fix this
   //unsigned int sourceDataSize = 4;

   // used to parse and populate sourceOffsetList;
   int srcOffsetIndex = 0;

   // Areas not processed in the "last" block - right or bottom
   unsigned int excessPixelsRight = dstImageWidth % dstBW;
   unsigned int excessPixelsBottom = dstImageHeight % dstBH;

   // For each block
   for (unsigned int i = 0; i < dstImageHeight; i += dstBH) {
      unsigned int currentBlockHeight = dstBH;
      if ((i > (dstImageHeight - dstBH)) && (excessPixelsBottom > 0)) {
         currentBlockHeight = excessPixelsBottom;
      }
      for (unsigned int j = 0; j < dstImageWidth; j += dstBW) {
         unsigned int currentBlockWidth = dstBW;
         if ((j > (dstImageWidth - dstBW)) && (excessPixelsRight > 0)) {
            currentBlockWidth = excessPixelsRight;
         }

         // Min and max X and Y values observed, in source image offsets
         unsigned int offsetMinX = 0xffffffff;		// set to very large integer
         unsigned int offsetMinY = 0xffffffff;		// set to very large integer

         // We are parsing each offset in the destination block
         int blkIndex = i * dstImageWidth + j;

         // For each pixel in the block
         for (unsigned int k = 0; k < currentBlockHeight; k++) {
            // Offset of first pixel in block row
            int offsetIndex = blkIndex + k * dstImageWidth;

            for (unsigned int l = 0; l < currentBlockWidth; l++) {
               unsigned int xMap = (oneDimOffset[offsetIndex])
                     % srcStride;
               unsigned int yMap = (oneDimOffset[offsetIndex])
                     / srcStride;

               if ((xMap < offsetMinX) || (yMap < offsetMinY)) {
                  if (xMap < offsetMinX) {offsetMinX = xMap;}
                  if (yMap < offsetMinY) {offsetMinY = yMap;}

                  // NB: this might not even be one of the offsets in the block
                  offsetTL = offsetMinY * srcStride + offsetMinX;
               }
               offsetIndex++;
            } // next pixel in row
         } // next pixel row in block

         // Determine results of block processing
         blockOffset[srcOffsetIndex++] = offsetTL;

      } // next block in row
   } // next row of blocks

   return rVal;
}


int32_t OneDOffsetToLocalOffset_ref(
         uint32_t* oneDimOffset,
		 int dstImageWidth,
		 int dstImageHeight,
         uint32_t* blockOffset,
         int dstBlockWidth,
         int dstBlockHeight,
         int srcBlockWidth,
         int srcStride,
         uint16_t* localOffset)
{
	/*
   uint32_t *srcBlockOffsets = (uint32_t*)lBlockOffset.RetDataPtr();
   uint32_t *offsetPtr       = (uint32_t*)l1DOffset.RetDataPtr();
   uint16_t *optr            = (uint16_t*)lLocalOffset.RetDataPtr();
   */

   // Takes advantage of the fact that 1DOffset and Dest images are the same size
   const unsigned int numBlocksX = (dstImageWidth + dstBlockWidth - 1) / dstBlockWidth;

   // for each pixel, probably single loop only needed?  but need to know my block number
   for (int y = 0; y < dstImageHeight; ++y) {
      unsigned int by = y / dstBlockHeight;

      for (int x = 0; x < dstImageWidth; ++x) {
         // Convert offset to 1-D block offset by:
         // - determine which source block to use
         // - subtract the offset of the top left of the current source block
         // - calucalte x, y offsets to top left of current source block by:
         //    offX = (offset - TLoffset) % stride   offY = (offset - TLoffset) / stride
         // - blockoffset = offY * blockstride + offX

         unsigned int bx = x / dstBlockWidth;

         // offset of the top left pixel of the source block
         unsigned int offsetTL = blockOffset[by * numBlocksX + bx];

         // Because offsetVal = (yi * srcStride) + xi;
         unsigned int offBlockX = (*oneDimOffset - offsetTL) % srcStride;
         unsigned int offBlockY = (*oneDimOffset - offsetTL) / srcStride;

         *localOffset = offBlockY * srcBlockWidth + offBlockX;

         oneDimOffset++;
         localOffset++;
      }
   }

   return 0;
}
