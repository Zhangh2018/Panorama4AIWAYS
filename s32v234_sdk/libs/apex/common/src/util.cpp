/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
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

#include <stdio.h>
#include <stdint.h>
#include "util.h"

int08u
cmem_read(vec08u __cmem* src, int cu)
{
    vec08u vtmp = src[0];
    int08u stmp = vget(vtmp, cu);
    return stmp;
}

int16u
cmem_read(vec16u __cmem* src, int cu)
{
    vec16u vtmp = src[0];
    int16u stmp = vget(vtmp, cu);
    return stmp;
}

void
cmem_write(vec08u __cmem* dst, int cu, int08u val)
{
    vec08u vtmp = dst[0];
    vec08u vval = val;
    vtmp = vput(vtmp, vval, cu);
    dst[0] = vtmp;
}

void
cmem_write(vec16u __cmem* dst, int cu, int16u val)
{
    vec16u vtmp = dst[0];
    vec16u vval = val;
    vtmp = vput(vtmp, vval, cu);
    dst[0] = vtmp;
}

void
TransferTileToCmem(unsigned char* input,
                  vec08u __cmem* dst,
                  int blkWidth,
                  int blkHeight,
                  int blkSpan,
                  int tileWidth,
                  int tileStride,
                  int tileOffsetX,
                  int tileOffsetY)
{
   for (int i=0; i<tileWidth; i+=blkWidth)
   {
      for (int j=0; j<blkHeight; j++)
      {
         for (int k=0; k<blkWidth; k++)
         {
            cmem_write(&dst[j*blkSpan+k], i/blkWidth, input[(j+tileOffsetY)*tileStride + i + k + tileOffsetX]);
         }
      }
   }
}

void
TransferTileToCmem(unsigned short* input,
                   vec16u __cmem* dst,
                   int blkWidth,
                   int blkHeight,
                   int blkSpan,
                   int tileWidth,
                   int tileStride,
                   int tileOffsetX,
                   int tileOffsetY)
{
   for (int i=0; i<tileWidth; i+=blkWidth)
   {
      for (int j=0; j<blkHeight; j++)
      {
         for (int k=0; k<blkWidth; k++)
         {
            cmem_write(&dst[j*blkSpan+k], i/blkWidth, input[(j+tileOffsetY)*tileStride + i + k + tileOffsetX]);
         }
      }
   }
}

void
CheckAgainstReference(int tileWidth, int tileStride, int tileHeight, int blkWidth, unsigned char* ref, vec08u __cmem* vec, int errorThreshold)
{
   int totalMismatchCnt = 0;
   int mismatchCnt      = 0;
   
   for (int i=0; i<tileWidth; i+=blkWidth)
   {
      for (int j=0; j<tileHeight; j++)
      {
         for (int k=0; k<blkWidth; k++)
         {
            int08u refVal  = ref[j*tileStride + i + k];
            int08u cmemVal = cmem_read(&vec[j*blkWidth+k], i/blkWidth);
            if (refVal != cmemVal)
            {
               totalMismatchCnt ++;
               if (((refVal-cmemVal) > errorThreshold) || ((cmemVal-refVal) > errorThreshold))
               {
                  mismatchCnt ++;
                  printf("Mismatch at sample (%3d,%3d) [REF = %3d, CMEM = %3d]\n", i+k, j, refVal, cmemVal);
               }
            } 
         }
      }
   }
   
   printf("%d mismatches detected ",totalMismatchCnt, mismatchCnt, errorThreshold);
   if (totalMismatchCnt)
      printf("(%d exceed allowable error threshold of %d)", mismatchCnt, errorThreshold);
   printf("\n");  
}

void
CheckAgainstReference_YUV422(int tileWidth, int tileStride, int tileHeight, int blkWidth, unsigned short* ref, vec16u __cmem* vec, int errorThreshold)
{
   int totalMismatchCnt = 0;
   int mismatchCnt      = 0;
   
   for (int i=0; i<tileWidth; i+=blkWidth)
   {
      for (int j=0; j<tileHeight; j++)
      {
         for (int k=0; k<blkWidth; k++)
         {
            int16u refVal  = ref[j*tileStride + i + k];
            int16u cmemVal = cmem_read(&vec[j*blkWidth+k], i/blkWidth);
            
            //check luma and chroma separately
            int08u refLum  = refVal>>8;
            int08u refChr  = refVal&0xFF;
            int08u cmemLum = cmemVal>>8;
            int08u cmemChr = cmemVal&0xFF;
            
            if ((refLum != cmemLum) || (refChr != cmemChr))
            {
               totalMismatchCnt ++;
               
               int08u lumDif = cmemLum - refLum;
               if (refLum > cmemLum)
                  lumDif = refLum - cmemLum;
   
               int08u chrDif = cmemChr - refChr;
               if (refChr > cmemChr)
                  chrDif = refChr - cmemChr;

               if (lumDif>errorThreshold || chrDif>errorThreshold)
               {
                  mismatchCnt ++;
                  printf("Mismatch at sample (%3d,%3d) [REF = %3x, CMEM = %3x]\n", i+k, j, refVal, cmemVal);
               }
            }
         }
      }
   }
   
   printf("%d mismatches detected ",totalMismatchCnt, mismatchCnt, errorThreshold);
   if (totalMismatchCnt)
      printf("(%d exceed allowable error threshold of %d)", mismatchCnt, errorThreshold);
   printf("\n");  
}

void
SetupPadding(vec08u __cmem* src,
             vec08u __cmem* padTop,
             vec08u __cmem* padBot,
             vec08u __cmem* padLeft,
             vec08u __cmem* padRight,
             int blkWidth,
             int blkHeight,
             int blkSpan,
             int filterSize)
{
   int padSize = filterSize>>1;
   
   //POPULATE PADDING BUFFERS [in this example, edges are all 'image borders' so padding will be replicated samples]
   
   //pad top and bottom [replication for this example]
   for (int i=0; i < padSize; i++) //height
   {
      for(int j=0; j<blkWidth; j++) //width
      {
         padTop[i*blkWidth + j] = src[j];
         padBot[i*blkWidth + j] = src[blkHeight*blkSpan-blkSpan + j];
      }
   }
   
   //pad left and right [replication for this example]
   for(int i=0; i<blkHeight; i++) //height
   {
      for (int j=0; j < padSize; j++) //width
      {
         padLeft [(i+padSize)*padSize + j] = src[i*blkSpan];
         padRight[(i+padSize)*padSize + j] = src[i*blkSpan + blkWidth - 1];
      } 
   }
   
   //pad left and right corners [replication for this example]
   for (int i=0; i<padSize; i++) //height
   {
      for (int j=0; j<padSize; j++) //width
      {
         padLeft[i*padSize+j] = src[0];
         padLeft[(padSize+blkHeight+i)*padSize + j] = src[blkHeight*blkSpan-blkSpan];
         
         padRight[i*padSize+j] = src[blkWidth-1];
         padRight[(padSize+blkHeight+i)*padSize + j] = src[blkHeight*blkSpan-blkSpan + blkWidth - 1];
      }
   } 
}

void
GenBlockPadding (vec08u __cmem* src,
                 vec08u __cmem* padTop,
                 vec08u __cmem* padBot,
                 vec08u __cmem* padLeft,
                 vec08u __cmem* padRight,
                 vec08u __cmem* blk,
                 const int blkWidth,
                 const int blkHeight,
                 const int blkSpan,
                 const int padSize)
{
   /*
   blk: (where X is the src data and a-h represent padding at specific locations)
   a b b b b c
   d X X X X e
   d X X X X e
   d X X X X e
   d X X X X e
   f g g g g h
   */
   
   //copy blk into tmp region
   /*
   for (int i=0; i < blkHeight; i++)
   {
      for (int j=0; j < blkWidth; j++)
      {
         blk[(padSize+i)*blkSpan + padSize + j] = src[i*blkWidth + j];
      }
   }
   */
   
   //fill in top and bottom padding (b and g)
   for (int j=0; j < blkWidth; j++)
   {
      for (int i=0; i < padSize; i++) chess_prepare_for_pipelining
      {
         blk[i*blkSpan + padSize + j]                     = padTop[i*blkWidth + j]; //top
         blk[(padSize+blkHeight+i)*blkSpan + padSize + j] = padBot[i*blkWidth + j]; //bottom
      }
   }
   
   //fill in left and right padding for source data (d and e)
   for (int i=0; i < blkHeight; i++)
   {
      for (int j=0; j < padSize; j++)
      {
         blk[(padSize+i)*blkSpan + j]                     = vmsr(src[i*blkSpan + blkWidth - (padSize-j)]); //left
         blk[(padSize+i)*blkSpan + blkSpan - (padSize-j)] = vmsl(src[i*blkSpan + j]);                      //right
      } 
   }
   
   //fill in left and right padding for corners (a, c, f, and h)
   for (int i=0; i < padSize; i++)
   {
      for (int j=0; j < padSize; j++)
      {
         blk[i*blkSpan + j]                     = vmsr(padTop[i*blkWidth + blkWidth - (padSize-j)]); //left
         blk[i*blkSpan + blkSpan - (padSize-j)] = vmsl(padTop[i*blkWidth + j]);                      //right
         
         blk[(padSize+blkHeight+i)*blkSpan + j]                     = vmsr(padBot[i*blkWidth + blkWidth - (padSize-j)]); //left
         blk[(padSize+blkHeight+i)*blkSpan + blkSpan - (padSize-j)] = vmsl(padBot[i*blkWidth + j]);                      //right
      } 
   }
   
   //fill in padding for leftmost and rightmost CUs (image edges or tile edges)
   vec08u __cmem lmask;
   vec08u __cmem rmask;
   cmem_write (&lmask,  0, 1);
   cmem_write (&rmask, 31, 1);
   
   for (int i=0; i<(blkHeight + 2*padSize); i++)
   {
      for (int j=0; j < padSize; j++)
      {
         vif (lmask == 1)
            blk[i*blkSpan+j] = padLeft[i*padSize + j];
      }
   }
   
   for (int i=0; i<(blkHeight + 2*padSize); i++)
   {
      for (int j=0; j < padSize; j++)
      {
         vif (rmask == 1)
            blk[i*blkSpan + blkSpan - (padSize-j)] = padRight[i*padSize + j];
      }
   } 
}

void
GenBlockPadding_HARDCODED (vec08u __cmem* src,
                           vec08u __cmem* padTop,
                           vec08u __cmem* padBot,
                           vec08u __cmem* padLeft,
                           vec08u __cmem* padRight,
                           vec08u __cmem* blk)
{
   
   const int blkWidth  = 8;
   const int blkHeight = 4;
   const int blkSpan   = 10;
   const int padSize   = 1;
   /*
   blk: (where X is the src data and a-h represent padding at specific locations)
   a b b b b c
   d X X X X e
   d X X X X e
   d X X X X e
   d X X X X e
   f g g g g h
   */
   
   //fill in top and bottom padding (b and g)
   for (int j=0; j < blkWidth; j++)
   {
      for (int i=0; i < padSize; i++)
      {
         blk[i*blkSpan + padSize + j]                     = padTop[i*blkWidth + j]; //top
         blk[(padSize+blkHeight+i)*blkSpan + padSize + j] = padBot[i*blkWidth + j]; //bottom
      }
   }
   
   //fill in left and right padding for source data (d and e)
   for (int i=0; i < blkHeight; i++)
   {
      for (int j=0; j < padSize; j++)
      {
         blk[(padSize+i)*blkSpan + j]                     = vmsr(src[i*blkSpan + blkWidth - (padSize-j)]); //left
         blk[(padSize+i)*blkSpan + blkSpan - (padSize-j)] = vmsl(src[i*blkSpan + j]);                      //right
      } 
   }
   
   //fill in left and right padding for corners (a, c, f, and h)
   for (int i=0; i < padSize; i++)
   {
      for (int j=0; j < padSize; j++)
      {
         blk[i*blkSpan + j]                     = vmsr(padTop[i*blkWidth + blkWidth - (padSize-j)]); //left
         blk[i*blkSpan + blkSpan - (padSize-j)] = vmsl(padTop[i*blkWidth + j]);                      //right
         
         blk[(padSize+blkHeight+i)*blkSpan + j]                     = vmsr(padBot[i*blkWidth + blkWidth - (padSize-j)]); //left
         blk[(padSize+blkHeight+i)*blkSpan + blkSpan - (padSize-j)] = vmsl(padBot[i*blkWidth + j]);                      //right
      } 
   }
   
   //fill in padding for leftmost and rightmost CUs (image edges or tile edges)
   vec08u __cmem lmask;
   vec08u __cmem rmask;
   cmem_write (&lmask,  0, 1);
   cmem_write (&rmask, 31, 1);
   
   for (int i=0; i<(blkHeight + 2*padSize); i++)
   {
      for (int j=0; j < padSize; j++)
      {
         vif (lmask == 1)
            blk[i*blkSpan+j] = padLeft[i*padSize + j];
      }
   }
   
   for (int i=0; i<(blkHeight + 2*padSize); i++)
   {
      for (int j=0; j < padSize; j++)
      {
         vif (rmask == 1)
            blk[i*blkSpan + blkSpan - (padSize-j)] = padRight[i*padSize + j];
      }
   } 
}

#if (INLINE_VDIV == 0)
vec16s vdiv(vec16s dividend, vec08u divisor, int n)
{
   vbool negsign = (dividend < 0);
   
   vif (negsign)
   {
      dividend = -dividend;
   }
   vendif
   
   vec16s quotient  = 0;
   vec16s remainder = dividend;
      
   for (int N = n-1; N >= 0; N--)
   {
      vif((remainder >> N) >= divisor)
      {
         remainder -= (divisor << N);
         quotient  += (1 << N);
      }
      vendif
   }
   
   vif(negsign)
      quotient = -quotient;
   vendif
   
   return quotient;
}
#endif
