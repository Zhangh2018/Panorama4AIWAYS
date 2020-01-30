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
*  \file apexcv_affine_transform_ref.cpp
*  \brief Reference code for affine transform
***********************************************************************************/

#include <stdio.h>
#include <string>

#include <icp_data.h>
#include <oal.h>

#include <affine_ref.h>

#define AB_BITS         10
#define INTER_BITS      5
#define AB_SCALE        (1<<AB_BITS)
#define INTER_TAB_SIZE  32

static void _calc_inv_matrix(
               int *dst,   // [out] inverse matrix
         const int *src    // [in] transform matrix
)
{
   int D = src[0]*src[4] - src[1]*src[3];
   D >>= AB_BITS;
   if (D==0)
      for (int i = 0; i < 6; i++)
         dst[i] = 0;
   
   int delta_d = D>>1;
   int d0 = ((src[4]<<INTER_BITS)+delta_d)/D;
   int d1 = ((src[1]<<INTER_BITS)+delta_d)/D;
   int d3 = ((src[3]<<INTER_BITS)+delta_d)/D;
   int d4 = ((src[0]<<INTER_BITS)+delta_d)/D;
   dst[2] = (-d0*src[2] + d1*src[5])>>AB_BITS;
   dst[5] = (d3*src[2] - d4*src[5])>>AB_BITS;
   dst[0] = d0;
   dst[1] = -d1;
   dst[3] = -d3;
   dst[4] = d4;
}

static int _calc_offset_base(
               int coef,   // [in] scale
               uint16_t y, // [in] offset
               int delta   // [in] bias
)
{
   int ta = (int)y * coef;
   return ta + delta;
}

static int _calc_offset_coord(
               int coord,  // [in] bias
               uint16_t x, // [in] offset 
               int coef    // [in] scale
)
{
   int ta = (int)(x * coef);
   return coord + ta;
}

static void _calc_chunk_offset(
               int *dst_gcoord,        // [out] global offset destination buffer
               int16_t *dst_lcoord,    // [out] local offset destination buffer
               uint16_t *dst_alpha,    // [out] alpha destination buffer
         const int *mat,               // [in] inverse matrix
         const uint16_t *offset,       // [in] global offset
               int offset_str,         // [in] stride of global offset
               int src_width,          // [in] source image width
               int src_height,         // [in] source image height
               int dstr,               // [in] stride of destination buffer
               int chunk_width,        // [in] chunk width
               int chunk_height        // [in] chunk height
)
{
   int src_w_b = (src_width-1)<<INTER_BITS;
   int src_h_b = (src_height-1)<<INTER_BITS;
   int tx[2];
   int ty[2];
   int gx = 0;
   int gy = 0;

   for (int h = 0; h < chunk_height; h++)
   {
      int X0 = _calc_offset_base(mat[1], offset[1], mat[2]);
      int Y0 = _calc_offset_base(mat[4], offset[1], mat[5]);

      for (int w = 0; w < (chunk_width>>1); w++)
      {
         for (int k = 0; k < 2; k++)
         {
            tx[k] = _calc_offset_coord(X0, offset[(w<<2)+(k<<1)], mat[0]);
            ty[k] = _calc_offset_coord(Y0, offset[(w<<2)+(k<<1)], mat[3]);
            // clamp to border
            if (tx[k] > src_w_b)
            {
               tx[k] = src_w_b;
            }
            else if (tx[k] < 0)
            {
               tx[k] = 0;
            }
            if (ty[k] > src_h_b)
            {
               ty[k] = src_h_b;
            }
            else if (ty[k] < 0)
            {
               ty[k] = 0;
            }
            dst_alpha[(w<<2)+(k<<1)+0] = (uint16_t)(tx[k]&(INTER_TAB_SIZE-1))>>1;
            dst_alpha[(w<<2)+(k<<1)+1] = (uint16_t)(ty[k]&(INTER_TAB_SIZE-1))>>1;

            tx[k] >>= INTER_BITS;
            ty[k] >>= INTER_BITS;
         }
         gx = (tx[0]<tx[1]) ? tx[0] : tx[1];
         gy = (ty[0]<ty[1]) ? ty[0] : ty[1];
         dst_gcoord[w] = gy * src_width + gx;
         dst_lcoord[(w<<2)+0] = (int16_t)(tx[0] - gx);
         dst_lcoord[(w<<2)+1] = (int16_t)(ty[0] - gy);
         dst_lcoord[(w<<2)+2] = (int16_t)(tx[1] - gx);
         dst_lcoord[(w<<2)+3] = (int16_t)(ty[1] - gy);
      }
      offset += offset_str;
      dst_gcoord += dstr>>1;
      dst_lcoord += dstr<<1;
      dst_alpha += dstr<<1;
   }
}

static void _process_calc_chunk_offset(
               int *dst_gcoord,        // [out] chunk offset array destination buffer
               int16_t *dst_lcoord,    // [out] local offset destination buffer
               uint16_t *dst_alpha,    // [out] bilinear coefficient destination buffer
         const int *matrix,            // [in] inverse matrix
         const uint16_t *offset,       // [in] input offset buffer
               int width,              // [in] process width
               int height,              // [in] process height
               int num_cu)
{
   int offset_str = width*2;
   int dstr = width;
   int chunk_width = width/num_cu;
   int chunk_height = 16;

   for (int h = 0; h < height/chunk_height; h++)
   {
      for (int c = 0; c < width; c+=chunk_width)
      {
         // calculate chunk offset
         _calc_chunk_offset(&dst_gcoord[c>>1], &dst_lcoord[c<<1], &dst_alpha[c<<1], matrix, &offset[c<<1], offset_str, width, height, dstr, chunk_width, chunk_height);
      }
      offset += offset_str*chunk_height;
      dst_gcoord += (dstr>>1)*chunk_height;
      dst_lcoord += (dstr<<1)*chunk_height;
      dst_alpha += (dstr<<1)*chunk_height;
   }
}

static uint8_t _interpolate(
      const uint8_t *src,     // [in] source image
            int sstr,         // [in] stride of source image
      const uint16_t *alpha   // [in] bilinear interpolation coefficient
)
{
   uint16_t dx = alpha[0];
   uint16_t dy = alpha[1];
   uint16_t sx = 16 - alpha[0];
   uint16_t sy = 16 - alpha[1];
   uint16_t ti = src[0]*sx*sy + src[1]*dx*sy + src[sstr]*sx*dy + src[sstr+1]*dx*dy;

   return (uint8_t)(((ti>>7)+1)>>1);
}

static void _bilinear_interpolation(
               uint8_t *dst,     // [out] destination image
         const uint8_t *src,     // [in] source image
               int sstr,         // [in] stride of source image
         const int16_t *coord,   // [in] local offset
         const uint16_t *alpha   // [in] bilinear coef
)
{
   for (int w = 0; w < 2; w++)
   {
      dst[w] = _interpolate(&src[coord[(w<<1)+1]*sstr + coord[w<<1]], sstr, &alpha[w<<1]);
   }
}

static void 
_process_bilinear_interpolation(
               uint8_t *dst,     // [out] destination image buffer
               int dstr,         // [in] stride of destination image
         const uint8_t *src,     // [in] source image
               int sstr,         // [in] stride of source image
               int width,        // [in] process width
               int height,       // [in] process height
         const int *gcoord,      // [in] chunk offset array
         const int16_t *lcoord,  // [in] local offset
         const uint16_t *delta   // [in] bilinear coef
)
{
   for (int h = 0; h < height; h++)
   {
      for (int c = 0; c < width; c+=2)
      {
         // bilinear interpolation
         _bilinear_interpolation(&dst[c], &src[gcoord[c>>1]], sstr, &lcoord[c<<1], &delta[c<<1]);
      }
      gcoord += sstr>>1;
      lcoord += sstr<<1;
      delta += sstr<<1;
      dst += dstr;
   }
}

int 
ref_affine_transform(
               uint8_t* dst,
         const uint8_t* src,
               int bw, int bh,
         const int* matrix,
         const uint16_t* offset,
         int num_cu)
{
   int lRetVal = 0;
   icp::DataDescriptor lInvMat;
   icp::DataDescriptor lGlobal;
   icp::DataDescriptor lLocal;
   icp::DataDescriptor lAlpha;
   
   void *lpInvMatOal = OAL_MemoryAllocFlag(6*4, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpGlobalOal = OAL_MemoryAllocFlag((bw/2)*bh*4, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpLocalOal = OAL_MemoryAllocFlag(bw*bh*2*2, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpAlphaOal = OAL_MemoryAllocFlag(bw*bh*2*2, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   
   if (lpInvMatOal && lpGlobalOal && lpLocalOal && lpAlphaOal)
   {
      lInvMat.InitManual(1, 1, OAL_MemoryReturnAddress(lpInvMatOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(lpInvMatOal, ACCESS_PHY),
         icp::DATATYPE_32S, 6, 1);
      lGlobal.InitManual(bw/2, bh, OAL_MemoryReturnAddress(lpGlobalOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(lpGlobalOal, ACCESS_PHY),
         icp::DATATYPE_32S, 1, 1);
      lLocal.InitManual(bw, bh, OAL_MemoryReturnAddress(lpLocalOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(lpLocalOal, ACCESS_PHY),
         icp::DATATYPE_16S, 2, 1);
      lAlpha.InitManual(bw, bh, OAL_MemoryReturnAddress(lpAlphaOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(lpAlphaOal, ACCESS_PHY),
         icp::DATATYPE_16U, 2, 1);
         
      int* inv_mat      = (int*)lInvMat.GetDataPtr();
      int* global       = (int*)lGlobal.GetDataPtr();
      int16_t* local    = (int16_t*)lLocal.GetDataPtr();
      uint16_t* alpha   = (uint16_t*)lAlpha.GetDataPtr();
      
//! [ref]
      _calc_inv_matrix(inv_mat, matrix);
      _process_calc_chunk_offset(global, local, alpha, inv_mat, offset, bw, bh, num_cu);
      _process_bilinear_interpolation(dst, bw, src, bw, bw, bh, global, local, alpha);
//! [ref]
   }
   else
   {
      lRetVal = 1;
      printf("Error: Cannot allocate memory!!!\n");
   }
//! [free reference memory]
   if (lpInvMatOal)  OAL_MemoryFree(lpInvMatOal);
   if (lpGlobalOal)  OAL_MemoryFree(lpGlobalOal);
   if (lpLocalOal)   OAL_MemoryFree(lpLocalOal);
   if (lpAlphaOal)   OAL_MemoryFree(lpAlphaOal);
//! [free reference memory]
   return lRetVal;
}

