/**************************************************************************************************
 * 
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015, Digital Media Professionals. All rights reserved.
 * Copyright 2017-2018 NXP 
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
 **************************************************************************************************/
 
#ifdef ACF_KERNEL_IMPLEMENTATION
#include "lbp_apu.h"

static const int16u LBP_RADIUS = 2;
static const int16u LBP_NEIGHBORS = 8;
static const int16u LBP_GRID_X = 8;
static const int16u LBP_GRID_Y = 8;
static const int16u LBP_HIST_LENGTH = (1<<LBP_NEIGHBORS);
static const int16u LOCAL_ACUM_NUM = 8;


static inline vec08u _calculate_local_binary_pattern(vec08u * input, const int16u stride)
{
   /* The local binary pattern is a visual descriptor used for various 
   * classifications in computer vision. It is one of the most simple and powerful
   * texture classificators. The algorithm is simple, define a circular region
   * around the center interest point and then for each pixel in this regions
   * compare its grayscale value with the center.
   * The circular dots that fall in between pixels are interpolated, see t1, t3, t5 and t7
   */
   vec16s out = 0;   
   vec16u center = (vec16u) input[0];
   vec16u centerScaled = (vec16u) (center << 7);
   
   vec16u t1 = (31 * (vec16u) input[-2*stride+1] + 22 * (vec16u) input[-2*stride+2] + 44 * (vec16u) input[-stride+1]  + 31 * (vec16u) input[-stride+2]);
   vec16u t3 = (22 * (vec16u) input[-2*stride-2] + 31 * (vec16u) input[-2*stride-1] + 31 * (vec16u) input[-stride-2]  + 44 * (vec16u) input[-stride-1]);
   vec16u t5 = (31 * (vec16u) input[stride-2]    + 44 * (vec16u) input[stride-1]    + 22 * (vec16u) input[2*stride-2] + 31 * (vec16u) input[2*stride-1]);
   vec16u t7 = (44 * (vec16u) input[stride+1]    + 31 * (vec16u) input[stride+2]    + 31 * (vec16u) input[2*stride+1] + 22 * (vec16u) input[2*stride+2]);
   
   out = vslc(out, (t7>=centerScaled));
   out = vslc(out, ((vec16u) input[2*stride]>=center));
   out = vslc(out, (t5>=centerScaled));
   out = vslc(out, ((vec16u) input[-2]>=center));
   out = vslc(out, (t3>=centerScaled));
   out = vslc(out, ((vec16u) input[-2*stride]>=center));
   out = vslc(out, (t1>=centerScaled));
   out = vslc(out, ((vec16u) input[2]>=center));

   return (vec08u) out;
}

static inline vec16u _calculate_distance(vec08u src0, vec08u src1)
{
   /* metric = (|a - b|)^2/(a + b) */
   vec16u diff = vabs_diff((vec16u) src0, (vec16u) src1);
   vec16u deno = (vec16u) src0 + (vec16u) src1;
   vec16u metric = (diff * diff)/deno;
   vec16u dist = vselect((vec16u) 0, metric, deno==0);
   return dist;
}



/*!
   Calculate LBP descriptor.
   (unsigned 8-bit -> unsigned 8-bit)
   
   \param lpvIn         - [Input]   Pointer to the input image data.
   \param lStrideIn     - [Input]   Stride of the input image data.
   \param lpvOut        - [Output]  Pointer to the output descriptor data.
   \param lChunkWidth   - [Input]   Processing chunk width.
   \parma lChunkHeight  - [Input]   Processing chunk height.
 */
void apu_lbp_extract_descriptor(
   vec08u *lpvIn,
   int16u lStrideIn,
   vec08u *lpvOut,
   int16u lChunkWidth,
   int16u lChunkHeight,
   int32_t numCU
)
{  
  /*  Steps to compute the lbp descriptor:
  *   1. For each pixel in the chunk, follow its neighbors in a circular pattern
  *       and compare each value with the center       
  *    2. Where the center pixel's value is greater than the selected pixel,
  *       write "0" otherwise write "1" in the bit map
  *    3. Compute the histogram over the entire chunk, this will be a 256-byte
  *      feature vector  
  */

   /* Clear the histogram accumulator */
   for (int16u i = 0; i < LBP_HIST_LENGTH; i++)
      lpvOut[i] = 0;
   
   /* skip margin (= radius pixel) */
   int16u lOffset = LBP_RADIUS*lStrideIn+LBP_RADIUS;
   lpvIn += lOffset;
   
   for (int16u h = 0; h < lChunkHeight; h++) chess_loop_range(1,)
   {
      for (int16u w = 0; w < lChunkWidth; w++)  chess_loop_range(1,)
      {
         /* Compute LBP of the pixel */
         vec08u lbp = _calculate_local_binary_pattern(&lpvIn[w], lStrideIn);
         
         /* Create the histogram */
         vec08u d = vload(lpvOut, lbp);
         vstore(lpvOut, lbp, d+1);
      }
      lpvIn += lStrideIn;
   }
}


/*!
   Compare two descriptors.
   (unsigned 8-bit -> signed 32-bit)
   
   \param lpvIn0        - [Input]   Pointer to the input image0 data.
   \param lpvIn1        - [Input]   Pointer to the input image1 data.
   \param lpvOut        - [Output]  Pointer to the histogram distance buffer.
   \param lCompareSize  - [Input]   Comparing array size.
 */
void apu_lbp_compare_histogram(
   vec08u *lpvIn0,
   vec08u *lpvIn1,
   vec32s *lpvOut,
   int16u lCompareSize,
   int32_t numCU
)
{   
   vec32s tmp = *lpvOut;
   for (int16u i = 0; i < lCompareSize; i++) chess_loop_range(1,)
   {
      tmp += (vec32s) _calculate_distance(lpvIn0[i], lpvIn1[i]);
    }
    *lpvOut = tmp;
}

// misa's implementation
void apu_lbp_compare_histogram_misa(
   vec08u *lpvIn0,
   vec08u *lpvIn1,
   vec32s *lpvOut,
   int16u lCompareSize,
   int32_t numCU
)
{
  vec32s tmp = *lpvOut;
  for (int16u i = 0; i < lCompareSize; i++) chess_loop_range(1,)
  {
    tmp += (vec32s) _calculate_distance(lpvIn0[i], lpvIn1[i]);
  }
  *lpvOut = tmp;
}


/*!
   Accumulate local value in each CU to obtain one scalar value.
   (signed 32-bit -> signed 32-bit)   
   \param lpvIn         - [Input]   Pointer to the local value.
   \param lpvOut        - [Output]  Pointer to the accumulated value.
 */
void apu_lbp_accumulate_distance(
   vec32s lpvIn,
   int32s *lpvOut,
   int32_t numCU
)
{
   vec32s localAcum = lpvIn;
   lpvOut[0] = 0;

   // move block sum value to left CU and add local distance LOCAL_ACUM_NUM-1 times
   for (int16u i = 0; i < (LOCAL_ACUM_NUM-1); i++)
   {
      localAcum = vmrl(localAcum, 0);
      localAcum += lpvIn;
    }
    
   // collect APU_VSIZE>>3 block sum values to merge into one global sum value
   for (int16u i = 0; i < (numCU>>3); i++)
   {
      lpvOut[0] += vget(localAcum, (i<<3));
   }
}
#endif
