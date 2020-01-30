/*****************************************************************************
*
* NXP Confidential Proprietary
*
* Copyright (c) 2013-2017 NXP
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
*  \file pro_blockmatching_ref.cpp
*  \brief Reference kernels for block matching algorithm
***********************************************************************************/
#include <blockmatching_ref.h>
#include <stdlib.h>
#include <stdint.h>

int BlockmatchingRef(uint32_t* dst, uint16_t* dst_score,
                      uint8_t* src_template, int ststr,
                      int32_t* template_offsets,
                      uint8_t* src_window, int swstr,
                      int32_t* window_offsets,
                      int num_points,
                      int tsx, int tsy,
                      int wsx, int wsy
)
{
   for (int i = 0; i < num_points; ++i) {
      uint32_t min = 0xFFFF;
      int16_t x = 0;
      int16_t y = 0;
      
      uint8_t* pTmp = &src_template[template_offsets[i]];
      uint8_t* pWin = &src_window[window_offsets[i]];
      
      for (int wy = 0; wy <= wsy-tsy; ++wy) {
      for (int wx = 0; wx <= wsx-tsx; ++wx) {
         uint32_t sum = 0;
         for (int ty = 0; ty < tsy; ++ty) {
         for (int tx = 0; tx < tsx; ++tx) {
            int16_t diff = pTmp[ty*ststr+tx] - pWin[(wy+ty)*swstr+(wx+tx)];
            sum += abs(diff);
            if (sum > 0xFFFF) {
               sum = 0xFFFF;
            }
         }
         }
         if (sum < min) {
            min = sum;
            x = wx;
            y = wy;
         }
      }
      }
      dst[i] = ((y&0xFFFF)<<16) | (x&0xFFFF);
      dst_score[i] = min;
   }
   
   return 0;
}


// Assumes the input point and output point locations given are the "center" of interest
int BlockmatchingRef(uint32_t* pOutput0,
                      uint8_t* pOutput0Status,
                      uint8_t* pInputTemp, int ist, int iwt, int iht,
                      uint8_t* pInputWin, int isw, int iww, int ihw,
                      uint32_t* pPointsArray,
                      int sadThreshold,
                      int num_points,
                      int tsx, int tsy,
                      int wsx, int wsy,
                      int ncu_x, int ncu_y
)
{
   // search and template must be multiple of 4, template must be less than search window
   if (   (tsx&0x3) || (tsy&0x3) || (wsx&0x3) || (wsy&0x3) || (tsx >= wsx) || (tsy >= wsy)
       || (wsx > iww) || (wsy > ihw) || (tsx > iwt) || (tsy > iht)) {
      return 1;
   }
   
   if (sadThreshold > 0xFFFF) {
      sadThreshold = 0xFFFF;
   }
   
   // Determine the effective search window
   const int overlap_x = wsx - tsx + 1;
   const int overlap_y = wsy - tsy + 1;
   const int eff_x = overlap_x * (ncu_x-1) + wsx;
   const int eff_y = overlap_y * (ncu_y-1) + wsy;
   int ex = eff_x;
   int ey = eff_y;
   
   if (ex > iww) {
      int nx = (iww - wsx) / overlap_x + 1;
      ex = overlap_x*(nx - 1) + wsx;
   }
   
   if (ey > ihw) {
      int ny = (ihw - wsy) / overlap_y + 1;
      ey = overlap_y*(ny - 1) + wsy;
   }
   
   // Create the offsets for each point
   for (int i = 0; i < num_points; ++i) {
      // Get the starting locations
      int x = pPointsArray[i]&0xFFFF;
      int y = pPointsArray[i]>>16;
      int start_x = x - ex/2;
      int start_y = y - ey/2;
      if (start_x < 0) {
         start_x = 0;
      }
      if (start_y < 0) {
         start_y = 0;
      }
      if (start_x + ex > iww) {
         start_x = iww - ex;
      }
      if (start_y + ey > ihw) {
         start_y = ihw - ey;
      }
      
      int32_t win_offset = start_y*isw + start_x;
      int32_t src_offset = (y - tsy/2)*ist + (x-tsx/2);
      if (src_offset < 0 || src_offset > (iht-1)*ist+iwt-1) {
         src_offset = 0;
      }
      uint16_t score = 0xFFFF;
      
      // Do the block matching
      BlockmatchingRef(&pOutput0[i], &score,
                        pInputTemp, ist,
                        &src_offset,
                        pInputWin, isw,
                        &win_offset,
                        1,
                        tsx, tsy,
                        ex, ey);
                        
      if (   (score < sadThreshold)
          && ((x-tsx/2) >= 0) && ((x+tsx/2) <= iwt)
          && ((y-tsy/2) >= 0) && ((y+tsy/2) <= iht))
      {
         pOutput0Status[i] = 1;
         int dx = pOutput0[i]&0xFFFF;
         int dy = pOutput0[i]>>16;
         pOutput0[i] = ((start_y + dy + tsy/2)<<16) | (start_x + dx + tsx/2);
      } else {
         pOutput0Status[i] = 0;
         pOutput0[i] = 0;
      }
   }
   
   return 0;
}
