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
*  \file apu_histogram.c
*  \brief APU histogram kernels.
***********************************************************************************/

#ifdef ACF_KERNEL_IMPLEMENTATION


#include <stdint.h>
#include "histogram_apu.h"

#define HIST_BIN_NBR     (256)

/*!
   Initialize Global & Local Histograms (CMEM)

   \param l_hist - [Output] Local Histograms (CMEM)
   \param g_hist - [Output] Global Histogram (DMEM)
   \param nbr    - [Input]  Number of Bins
 */
void
apu_hist_init(vec16u* l_hst, uint32_t* g_hst, int nbr)
{
   for (int k=0; k<nbr; ++k) chess_loop_range(1,)
   {
      l_hst[k] = 0;
      g_hst[k] = 0;
   }
}

/*!
   Initialize Global & Local Histograms (CMEM) - HIST_BIN_NBR bins

   \param l_hist - [Output] Local Histograms (CMEM)
   \param g_hist - [Output] Global Histogram (DMEM)
 */
void
apu_hist_init(vec16u* l_hst, uint32_t* g_hst)
{
   apu_hist_init(l_hst, g_hst, HIST_BIN_NBR);
}

// Local Histogram (1D)
//    Maximum: 2^16-1 /CU - enough for 1920x1080 on 32 CU - image with single shade.
/*!
   Calculate a local histogram for an image tile.  
   This input image is 8 bits per pixel, 
   and the histogram must have at enough bins to contain maximum source value 
   (256 bins, for [0-255]).

   \param lhist - [Output] 256-bin Local Histogram (CMEM)
   \param src   - [Input]  Source Pointer buffer
   \param sstr    - [Input]  Source Stride
   \param bw    - [Input]  Source Width, in pixels (must be a multiple of 2, 16-bit aligned)
   \param bh    - [Input]  Source Height
 */
void
apu_hist1_l(vec16u* lhist, const vec08u* src, int sstr, int bw, int bh)
{
   // data must be aligned 16-bit
   /* This code does not work with emulation library due to Memory organization in vectors<vectors of CU's> and not CU_vectors<data_vectors>, as it is needed here
   */
#ifndef APEX2_EMULATE
   const vec16u* s = (const vec16u*)src;
   sstr >>= 1;
   // Local Histogram
   for (int y = 0; y<bh; ++y) 
   {
      vec16u v  = s[0];
      for (int x=1; x<=(bw>>1); ++x) chess_loop_range(1,)
                                    chess_prepare_for_pipelining
      {
         const vec16s v0 = (vec16s)vmul_ulul(v, 1);
         const vec16s v1 = (vec16s)vmul_uhul(v, 1);
         v  = s[x];
         const vec16u d0 = vload(lhist, v0) + 1;
         const vec16u d1 = vload(lhist, v1) + 1;
         vstore(lhist, v0, d0);
         vstore(lhist, v1, d1 + (vec16u)(v0==v1));
      }

      s += sstr;
   }
#else
  const vec08u* s = src;
  for (int y = 0; y<bh; ++y)
  {
     for (int x = 0; x<bw; ++x)
    {
      vec08u v = s[x];
      const vec16u d = vload(lhist, v) + 1; vstore(lhist, v, d);
    }
    s += sstr;
  }

#endif
}

// Global Histogram (1D) (Reduction)  (using CU for intermediate results)
/*!
   Accumulate in Global Histogram (DMEM) all the Local Histograms (CMEM)

   \param ghist - [Output] 256-bin Global Histogram (DMEM)
   \param lhist - [Input]  256-bin Local  Histogram (CMEM)
   \param bcu   - [Input]  ID of first CU used
   \param ecu   - [Input]  ID of first unused CU (last used + 1)
   
 */
void
apu_hist1_gn(uint32_t* ghist, const vec16u __cmem* lhist, int bcu, int ecu)
{
   // Histogram Bin Reduction
   for (int k=0; k<HIST_BIN_NBR; k+=8)
   {  
      
        vec16u lhist_b0 = lhist[k + 0];
        vec16u lhist_b1 = lhist[k + 1];
        vec16u lhist_b2 = lhist[k + 2];
        vec16u lhist_b3 = lhist[k + 3];
        vec16u lhist_b4 = lhist[k + 4];
        vec16u lhist_b5 = lhist[k + 5];
        vec16u lhist_b6 = lhist[k + 6];
        vec16u lhist_b7 = lhist[k + 7];
        /* Temporary global accumulator */
        uint32_t acc0 = ghist[k + 0];
        uint32_t acc1 = ghist[k + 1];
        uint32_t acc2 = ghist[k + 2];
        uint32_t acc3 = ghist[k + 3];
        uint32_t acc4 = ghist[k + 4];
        uint32_t acc5 = ghist[k + 5];
        uint32_t acc6 = ghist[k + 6];
        uint32_t acc7 = ghist[k + 7];
        
        for (int cu_id = bcu; cu_id < ecu; ++cu_id)  chess_loop_range(1,)
        {
            acc0 += vget(lhist_b0, cu_id);
            acc1 += vget(lhist_b1, cu_id);
            acc2 += vget(lhist_b2, cu_id);
            acc3 += vget(lhist_b3, cu_id);
            acc4 += vget(lhist_b4, cu_id);
            acc5 += vget(lhist_b5, cu_id);
            acc6 += vget(lhist_b6, cu_id);
            acc7 += vget(lhist_b7, cu_id);
        }
        ghist[k + 0] = acc0;
        ghist[k + 1] = acc1;
        ghist[k + 2] = acc2;
        ghist[k + 3] = acc3;
        ghist[k + 4] = acc4;
        ghist[k + 5] = acc5;
        ghist[k + 6] = acc6;
        ghist[k + 7] = acc7;
   }
}

/*!
   Accumulate in Global Mean (DMEM) all the Local Mean (CMEM)

   \param src   - [Input]  Source Pointer buffer
   \param tmp   - [Output]  Template for caculate
   \param sz    - [Input]  Size of chunk
 */
void
apu_mean1_l(vec08u* src, vec32u* tmp, uint32_t sz)
{
    //since there is no spatial dep, input chunk can be accessed linearly
    for (int i = 0; i < sz; i++)
    {
      tmp[0] += (vec32u)src[i];
    }
}

/*!
   Accumulate in Global Mean (DMEM) all the Local Mean (CMEM)

   \param bcu   - [Input]  ID of first CU used
   \param ecu   - [Input]  ID of first unused CU (last used + 1)
   \param vsum  - [Input]  Caculated value for output
   \param gmean - [Output] 256-bin Global Mean (DMEM)
 */
void
apu_mean1_gn(int bcu, int ecu, vec32u vsum, uint32_t* gmean)
{
    uint32_t lSum = 0;
    //since there is no spatial dep, input chunk can be accessed linearly
    for (int ci = bcu; ci < ecu; ci++)
    {
        lSum += vget(vsum, ci);
    }

    gmean[0] = lSum;
}

/*!
   Accumulate in Global Mean (DMEM) all the Local Mean (CMEM)

   \param src0       - [Input]   Source Pointer buffer0
   \param src1       - [Input]   Source Pointer buffer1
   \param g_stddev   - [Output]  Local stddev (CMEM)
   \param sz         - [Input]   Chunk size
 */
void
apu_stddev_l(vec08u* src0, uint8_t* src1, vec32u* g_stddev, uint32_t sz)
{
    vec32u vsum = g_stddev[0];
    vec32u vsqsum = g_stddev[1];
    uint8_t scale = src1[0];

    //since there is no spatial dep, input chunk can be accessed linearly
    for (int i = 0; i < sz; i++)
    {
        vec08u in = src0[i];
        vec16u inSq = ((vec16u)in) * ((vec16u)in);
        vsum += (vec32u)in;
        vsqsum += (vec32u)(inSq >> scale);
    }

    g_stddev[0] = vsum;
    g_stddev[1] = vsqsum;
}

#endif /* ACF_KERNEL_IMPLEMENTATION */
