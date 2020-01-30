/*****************************************************************************
 * *
 * * NXP Confidential Proprietary
 * *
 * * Copyright (c) 2015-2018 NXP Semiconductor;
 * * All Rights Reserved
 * *
 * *****************************************************************************
 * *
 * * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * * THE POSSIBILITY OF SUCH DAMAGE.
 * *
 * ****************************************************************************/

#include "lkpyramid_ref.h"
#include "image_pyramid_ref.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define LKPYRAMID_FLOATING_COOR_Q    8
#define LKPYRAMID_MAX_TRACKING_PT    1024

#define LKPYRAMID_PATCH_BORDER 6
#define IMAGE_PAD 16

#define DEBUG_PRINT_OUT 0


/* Fix-point math simulation */
static uint64_t getA(uint64_t x){return (x>>56)&0xFF;}
static uint64_t getB(uint64_t x){return (x>>48)&0xFF;}
static uint64_t getM(uint64_t x){return (x>>56)&0xFF;}
static uint64_t getN(uint64_t x){return (x>>48)&0xFF;}

static void create_padded_image(uint8_t *src     /*!< Source memory buffer. */,
                        uint8_t *dst      /*!< Destination memory buffer. */,
                        int32_t width     /*!< Source image width. */,
                        int32_t height    /*!< Source image height. */,
                        int32_t top       /*!< Padding size in +y direction. */,
                        int32_t bottom    /*!< Padding size in -y direction. */,
                        int32_t left      /*!< Padding size in -x direction. */,
                        int32_t right     /*!< Padding size in +x direction. */
)
{
   int sstr = width;
   int dstr = width+left+right;
   uint8_t *buf = dst + top*dstr;

   for (int h = 0; h < height; h++)
   {
      int w;
      for (w = 0; w < left; w++)
      {
         buf[h*dstr+w] = src[h*sstr];
      }
      for (; w < left+width; w++)
      {
         buf[h*dstr+w] = src[h*sstr+(w-left)];
      }
      for (; w < dstr; w++)
      {
         buf[h*dstr+w] = src[h*sstr+width-1];
      }
   }

   for (int w = 0; w < dstr; w++)
   {
      for (int h = 0; h < top; h++)
      {
         dst[h*dstr+w] = buf[w];
      }

      for (int h = top+height; h < top+height+bottom; h++)
      {
         dst[h*dstr+w] = buf[(height-1)*dstr+w];
      }
   }
}

static void create_padded_image(int16_t *src     /*!< Source memory buffer. */,
                        int16_t *dst      /*!< Destination memory buffer. */,
                        int32_t width     /*!< Source image width. */,
                        int32_t height    /*!< Source image height. */,
                        int32_t top       /*!< Padding size in +y direction. */,
                        int32_t bottom    /*!< Padding size in -y direction. */,
                        int32_t left      /*!< Padding size in -x direction. */,
                        int32_t right     /*!< Padding size in +x direction. */
)
{
   int sstr = width;
   int dstr = width+left+right;
    int16_t *buf = dst + top*dstr;

   for (int h = 0; h < height; h++)
   {
      int w;
      for (w = 0; w < left; w++)
      {
         buf[h*dstr+w] = src[h*sstr];
      }
      for (; w < left+width; w++)
      {
         buf[h*dstr+w] = src[h*sstr+(w-left)];
      }
      for (; w < dstr; w++)
      {
         buf[h*dstr+w] = src[h*sstr+width-1];
      }
   }

   for (int w = 0; w < dstr; w++)
   {
      for (int h = 0; h < top; h++)
      {
         dst[h*dstr+w] = buf[w];
      }

      for (int h = top+height; h < top+height+bottom; h++)
      {
         dst[h*dstr+w] = buf[(height-1)*dstr+w];
      }
   }
}

static int _run_reference_pyramid_down(uint8_t *src, int width, int height, uint8_t *dst)
{
   int lRetVal = 0;

   void *lpSrcPaddedOal = OAL_MemoryAllocFlag((width+4)*height, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpMidOal = OAL_MemoryAllocFlag(width*height*2, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   void *lpMidPaddedOal = OAL_MemoryAllocFlag(width*(height+4)*2, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   if (lpSrcPaddedOal && lpMidOal && lpMidPaddedOal)
   {
      uint8_t *srcPadded = (uint8_t *)OAL_MemoryReturnAddress(lpSrcPaddedOal, ACCESS_CH_WB);
      int16_t *mid = (int16_t *)OAL_MemoryReturnAddress(lpMidOal, ACCESS_CH_WB);
      int16_t *midPadded = (int16_t *)OAL_MemoryReturnAddress(lpMidPaddedOal, ACCESS_CH_WB);

      int32_t sstr = width+2+2;
      int32_t dstr = width;
      create_padded_image(src, srcPadded, width, height, 0, 0, 2, 2);
      uint8_t *tempSrcPadded = srcPadded + 2;
      int16_t *tempMid = mid;
      for (int h = 0; h < height; h++)
      {
         for (int w = 0; w < width; w++)
         {
            horizontal_gaus(&tempSrcPadded[w], sstr, &tempMid[w], dstr, 1, 1);
         }
         tempSrcPadded += sstr;
         tempMid += dstr;
      }

      create_padded_image(mid, midPadded, width, height, 2, 2, 0, 0);
      sstr = width;
      dstr = width>>1;
      int16_t *tempMidPadded = midPadded + 2*width;
      for (int h = 0; h < height>>1; h++)
      {
         for (int w = 0; w < width>>1; w++)
         {
            vertical_gaus_and_reduce(&tempMidPadded[w<<1], sstr, &dst[w], dstr, 1, 1);
         }
         tempMidPadded += sstr*2;
         dst += dstr;
      }
   }
   else
   {
      lRetVal = 1;
   }

   //! [free memory]
   if (lpSrcPaddedOal)     OAL_MemoryFree(lpSrcPaddedOal);
   if (lpMidOal)           OAL_MemoryFree(lpMidOal);
   if (lpMidPaddedOal)     OAL_MemoryFree(lpMidPaddedOal);
   //! [free memory]
   
   return lRetVal;
}

static int count_leading_zero(uint16_t x)
{
   uint8_t n = 0;
   for (int i = 0; i < 16; i++)
   {
      if (x & 0x8000) break;
      n++;
      x <<= 1;
   }
   return n;
}

static int count_leading_zero(uint32_t x)
{
   uint8_t n = 0;
   for (int i = 0; i < 32; i++)
   {
      if (x & 0x80000000) break;
      n++;
      x <<= 1;
   }
   return n;
}


static uint64_t mul1(uint64_t abcd, uint64_t mnop){
   uint64_t am = getA(abcd)*getM(mnop) << 48;
   uint64_t an = getA(abcd)*getN(mnop) << 40;
   uint64_t bm = getB(abcd)*getM(mnop) << 40;

   return (am  +
            an +
            bm  ) & 0xFFFF000000000000;
}
static uint64_t mul2(uint64_t abcd, uint64_t mnop){
   uint64_t am = getA(abcd)*getM(mnop) << 48;
   uint64_t an = getA(abcd)*getN(mnop) << 40;

   return (am  +
            an)  & 0xFFFF000000000000;
}

static uint16_t fxrecip12(uint16_t d){
   uint64_t dn = (uint64_t)d << 48;
   uint64_t x1 = (dn<<1) & 0xFFFE000000000000ULL;
   uint64_t k0, k1, k2, s0, s1;

   if(dn > (46006ULL<<48)){
      s1 = 3;
      s0 = 2;
      k1 =   181ULL<<56;
      k2 = 30945ULL<<48;
      k0 = 63962ULL<<48;

      k0 -= 9ULL<<48;
   }else{
      s1 = 1;
      s0 = 4;
      k1 =    31ULL<<56;
      k2 = 58691ULL<<48;
      k0 = 65532ULL<<48;

      k0 -= 9ULL<<48;
   }

   uint64_t x2   = mul1(x1, x1);
   uint64_t k1x1 = mul2(k1, x1);
   uint64_t a    = k0 - x1 + ((k1x1 >> s0) & 0xFFFF000000000000ULL);
   uint64_t b    = k2 - ((x1 >> s1) & 0xFFFF000000000000ULL);
   uint64_t bx2  = mul1(b, x2);
   uint64_t abx2 = a+bx2;
   return   abx2 >> 48;
}

static void ref_lkpyramid_calc_offset(icp::DataDescriptor& SrcDesc, icp::DataDescriptor& CoorDesc, icp::DataDescriptor& OffsDesc, int half_win, int pad)
{
   int32_t* p_offset = (int32_t*)OffsDesc.GetDataPtr();
   int32_t* p_coor   = (int32_t*)CoorDesc.GetDataPtr();

   for (uint32_t i = 0; i < CoorDesc.GetWidth(); i++)
   {
      int32_t x = p_coor[i*CoorDesc.GetElementDimX()];
      int32_t y = p_coor[i*CoorDesc.GetElementDimX()+1];
      int32_t i_x = x >> LKPYRAMID_FLOATING_COOR_Q;
      int32_t i_y = y >> LKPYRAMID_FLOATING_COOR_Q;
      p_offset[i] = SrcDesc.GetSpan() * i_y + RetDataTypeSizeInBytes(SrcDesc.GetElementDataType())*i_x;
      p_offset[i] -= (SrcDesc.GetSpan()*(half_win+pad) + (half_win+pad)*RetDataTypeSizeInBytes(SrcDesc.GetElementDataType()));
   }
}

static inline void ref_inline_lkpyramid_bilinear_08u(
      uint8_t* dst, int dstr,
      uint8_t* src, int sstr,
      uint8_t* frac_xy,
      int bw, int bh,
      int shift)
{
	uint16_t unit = 0x1 << shift;

	uint8_t *pc = frac_xy;
	uint16_t coef_a = (uint16_t)*pc++;
	uint16_t coef_b = (uint16_t)*pc;

   /* early exit */
   if (coef_a == 0 && coef_b == 0)
   {
	   for (int i = 0; i < bh; i++)
	   {
		   uint8_t *p0 = src;
		   uint8_t *pd = dst;
		   for (int j = 0; j < bw; j++)
		   {
            *pd++ = *p0++;  
         }
         src += sstr;
         dst += dstr;
      }
		return;
   }   

	uint16_t iw00 = (unit-coef_a)*(unit-coef_b);
	uint16_t iw01 = coef_a*(unit-coef_b);
	uint16_t iw10 = (unit-coef_a)*coef_b;
	uint16_t iw11 = (unit<<shift) - iw00 - iw01 - iw10;

	for (int i = 0; i < bh; i++)
	{
		uint8_t *p0 = src;
		uint8_t *pd = dst;
		for (int j = 0; j < bw; j++)
		{
			uint8_t  *p1 = p0++;
			uint16_t val = (uint16_t)*p1++;
			uint32_t sum = 1<<(2*shift-1);
			sum += iw00*val;
			val = (uint16_t)*p1;
			sum += iw01*val;
			p1 += sstr;
			val = (uint16_t)*p1--;
			sum += iw11*val;
			val = (uint16_t)*p1;
			sum += iw10*val;
			sum = sum>>16;
			*pd++ = (uint8_t)sum;
		}
      src += sstr;
      dst += dstr;
	}
}

static void ref_lkpyramid_bilinear_08u(
      uint8_t* dst, int dstr,
      uint8_t* src, int sstr,
      int32_t* coor,
      int bw, int bh,
      int shift)
{
   int16_t mask = 0x0;
   for (int i = 0; i < LKPYRAMID_FLOATING_COOR_Q; i++)
   {
      mask = mask | (1<<i);
   }
   uint8_t frac[2] = {(uint8_t)(coor[0] & mask), (uint8_t)(coor[1] & mask)};

   ref_inline_lkpyramid_bilinear_08u(
            dst, dstr,
            src, sstr,
            frac,
            bw, bh,
            shift);
}

static void ref_flt_centraldx_3x3(
			int16_t* dst, int dstr, 
			uint8_t* src, int sstr, 
			int bw, int bh,
			int8_t k0, int8_t k1, int8_t k2)
{
	for (int y = 0; y < bh; y++)
	{
		for (int x = 0; x < bw; x++)
		{
			int16_t tmp = ((int16_t)k0)*src[(y-1)*sstr+x+1];
			tmp += (int16_t)k1*src[(y)*sstr+x+1];
			tmp += (int16_t)k2*src[(y+1)*sstr+x+1];
			tmp -= (int16_t)k0*src[(y-1)*sstr+x-1];
			tmp -= (int16_t)k1*src[(y)*sstr+x-1];
			tmp -= (int16_t)k2*src[(y+1)*sstr+x-1];

			dst[y*dstr+x] = (int16_t)tmp;
		}
	}	
}

static void ref_flt_centraldy_3x3(
			int16_t* dst, int dstr, 
			uint8_t* src, int sstr, 
			int bw, int bh,
			int8_t k0, int8_t k1, int8_t k2)
{
	for (int y = 0; y < bh; y++)
	{
		for (int x = 0; x < bw; x++)
		{
			int16_t tmp = ((int16_t)k0)*src[(y+1)*sstr+x-1];
			tmp += (int16_t)k1*src[(y+1)*sstr+x];
			tmp += (int16_t)k2*src[(y+1)*sstr+x+1];
			tmp -= (int16_t)k0*src[(y-1)*sstr+x-1];
			tmp -= (int16_t)k1*src[(y-1)*sstr+x];
			tmp -= (int16_t)k2*src[(y-1)*sstr+x+1];

			dst[y*dstr+x] = (int16_t)tmp;
		}
	}
}

static void ref_lkpyramid_core(
      int32_t* coor_xy_o,                  /* output updated coordinate */
      uint8_t* srcpatch, int patchstride,  /* patch */
      uint8_t* srctempl, int templstride,  /* template */
      int16_t* srcdx,    int dxstride,     /* template x gradient */
      int16_t* srcdy,    int dystride,     /* template y gradient */
      int32_t* coor_xy_i,                  /* input coordinate */
      int bw_templ, int bh_templ,          /* template block width, height */
      int , int ,          /* patch    block width, height */
      int mNumIter,
      FILE* )
{
   int16_t* srcdt_oal    = (int16_t*)OAL_MemoryAllocFlag(sizeof(int16_t)*bw_templ*bh_templ, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   uint8_t* patchwin_oal = (uint8_t*)OAL_MemoryAllocFlag(bw_templ*bh_templ, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   int16_t* srcdt    = (int16_t*)OAL_MemoryReturnAddress(srcdt_oal, ACCESS_CH_WB);
   uint8_t* patchwin = (uint8_t*)OAL_MemoryReturnAddress(patchwin_oal, ACCESS_CH_WB);
   int dtstride = bw_templ;
   int patchwinstride = bw_templ;


   int16_t A[3]; /* [dxdx, dxdy, dydy] */
   int16_t B[2]; /* [dxdt, dydt] */
   int16_t inv_detA = 0;

	int16_t disp_xy[2] = { (int16_t)(coor_xy_i[0]&0xFF), (int16_t)(coor_xy_i[1]&0xFF)};

   int16_t min_a_clz;
   int16_t min_b_clz;
   int16_t recip_shift;
   int16_t disp_shift;

   {
      int32_t AA[4] = {0, 0, 0, 0};
      for (int i = 0; i < bw_templ; i++)
      {
         for (int j = 0; j < bh_templ; j++)
         {
            int16_t dx = (*(srcdx+i*dxstride+j));
            int16_t dy = (*(srcdy+i*dystride+j));
            AA[0] += dx*dx;
            AA[1] += dx*dy;
            AA[2] += dy*dy;
         }
      }
   
      /* normalize matrix A */
      {
         
         min_a_clz = count_leading_zero((uint16_t)((abs(AA[0])>>16) | (abs(AA[1])>>16) | (abs(AA[2]) >> 16))) - 1;
         if(min_a_clz < 0)  min_a_clz = 0;
         if(min_a_clz > 31) min_a_clz = 31;
         A[0] = ((AA[0]<<min_a_clz) + (1<<15)) >> 16;
         A[1] = ((AA[1]<<min_a_clz) + (1<<15)) >> 16;
         A[2] = ((AA[2]<<min_a_clz) + (1<<15)) >> 16;
      }
   
      AA[3] = A[0]*A[2]-A[1]*A[1];
   
      /* reciprocal of detA */
      /* normalize detA */
      { 
         recip_shift = count_leading_zero((uint32_t)(abs(AA[3])));
         uint32_t d = ((abs(AA[3])<<recip_shift) + (1<<15)) >> 16;
         uint16_t recip = fxrecip12(d);
         /* right shift 1 bit to make the room for the sign bit */
         inv_detA = (int16_t)(recip>>1);
         if (AA[3] < 0)
            inv_detA = -inv_detA;
      }
   }


   coor_xy_o[3] = 1;
   for (int iter = 0; iter < mNumIter; iter++)
   { 
      int16_t i_x = disp_xy[0] >> LKPYRAMID_FLOATING_COOR_Q;
      int16_t i_y = disp_xy[1] >> LKPYRAMID_FLOATING_COOR_Q;
      int16_t offset = i_y * patchstride + i_x;
      int16_t mask = 0x0;
      for (int i = 0; i < LKPYRAMID_FLOATING_COOR_Q; i++)
      {
         mask = mask | (1<<i);
      }
      uint8_t frac[2] = {(uint8_t)(disp_xy[0] & mask), (uint8_t)(disp_xy[1] & mask)};

      ref_inline_lkpyramid_bilinear_08u(
         patchwin, patchwinstride,
         srcpatch+offset, patchstride, 
         frac,
         bw_templ, bh_templ,
         LKPYRAMID_FLOATING_COOR_Q);
         
      /* sub for dt */
      int16_t *dt_row = srcdt;
      uint8_t *patchwin_row = patchwin;
      uint8_t *srctempl_row = srctempl;
      for (int i = 0; i < bh_templ; i++)
      {
         int16_t *dst = dt_row;
         uint8_t *psrc = patchwin_row;
         uint8_t *tsrc = srctempl_row;
         for (int j = 0; j < bw_templ; j++)
         {
           *dst++ = (*psrc++) - (*tsrc++);
         }
         dt_row += dtstride;
         patchwin_row += patchwinstride;
         srctempl_row += templstride;
      }
    
#if DEBUG_PRINT_OUT
//      if (iter == 0)
      {
         for (int i = 0; i < bh_patch; i++)
         {
            fprintf(tmpFile, "patch, ");
            for (int j = 0; j < bw_patch; j++)
            {
               fprintf(tmpFile, "%d, ", *((srcpatch-patchstride*((bh_patch-bw_templ)/2)-((bh_patch-bw_templ)/2))+i*patchstride+j));
            }
            if (i < bw_templ)
            {
               fprintf(tmpFile, "templ, ");
               for (int j = 0; j < bw_templ; j++)
               {
                  fprintf(tmpFile, "%d, ", *(srctempl+i*templstride+j));
               }
               fprintf(tmpFile, "patchwin, ");
               for (int j = 0; j < bw_templ; j++)
               {
                  fprintf(tmpFile, "%d, ", *(patchwin+i*patchwinstride+j));
               }
               fprintf(tmpFile, "dt, ");
               for (int j = 0; j < bw_templ; j++)
               {
                  fprintf(tmpFile, "%d, ", *(srcdt+i*dtstride+j));
               }
               fprintf(tmpFile, "dx, ");
               for (int j = 0; j < bw_templ; j++)
               {
                  fprintf(tmpFile, "%d, ", *(srcdx+i*dxstride+j));
               }
               fprintf(tmpFile, "dy, ");
               for (int j = 0; j < bw_templ; j++)
               {
                  fprintf(tmpFile, "%d, ", *(srcdy+i*dystride+j));
               }
               fprintf(tmpFile, "\n");
            }
            else
            {
               fprintf(tmpFile, "\n");
            }
         }
      }
#endif

      {
         int16_t dx;
         int16_t dy;
         int16_t dt;
         int32_t sum_B0 = 0;
         int32_t sum_B1 = 0;
       
         for (int i = 0; i < bw_templ; i++)
         {
            for (int j = 0; j < bh_templ; j++)
            {
               dx = *(srcdx+i*dxstride+j);
               dy = *(srcdy+i*dystride+j);
               dt = *(srcdt+i*dtstride+j);
   
               sum_B0 += dx*dt;
               sum_B1 += dy*dt;
            }
         }
         min_b_clz = count_leading_zero((uint16_t)((abs(sum_B0)>>16) | (abs(sum_B1)>>16))) - 1;
         min_b_clz = min_b_clz < 0  ? 0  : min_b_clz;
         min_b_clz = min_b_clz > 31 ? 31 : min_b_clz;
   

         B[0] = ((sum_B0<<min_b_clz) + (1<<15)) >> 16;
         B[1] = ((sum_B1<<min_b_clz) + (1<<15)) >> 16;
      }

      {
         int32_t delta_0 = A[1]*B[1] - A[2]*B[0];
         int32_t delta_1 = A[1]*B[0] - A[0]*B[1];

         disp_shift = count_leading_zero((uint16_t)((abs(delta_0)>>16) | (abs(delta_1)>>16))) - 1;
         if(disp_shift < 0)  disp_shift = 0;
         if(disp_shift > 31) disp_shift = 31;
         delta_0 = ((delta_0<<disp_shift) + (1<<15)) >> 16;
         delta_1 = ((delta_1<<disp_shift) + (1<<15)) >> 16;
         delta_0 = delta_0*inv_detA;
         delta_1 = delta_1*inv_detA;


         if (17 + min_b_clz + disp_shift > min_a_clz + recip_shift)
         {
            int scaling = 17 + min_b_clz + disp_shift - (min_a_clz + recip_shift);
            scaling = scaling > 31 ? 31 : scaling;
            delta_0 >>= scaling;
				delta_1 >>= scaling;
         }
         else
         {
            int scaling = (min_a_clz + recip_shift) - (17 + min_b_clz + disp_shift);
            scaling = scaling > 31 ? 31 : scaling;
            delta_0 <<= ((min_a_clz + recip_shift) - (17 + min_b_clz + disp_shift));
            delta_1 <<= ((min_a_clz + recip_shift) - (17 + min_b_clz + disp_shift));
         }
         disp_xy[0] += (delta_0);
         disp_xy[1] += (delta_1);
   
   #if DEBUG_PRINT_OUT
   //      if (iter == 0)
         {
            fprintf(tmpFile, "A, %d, %d, %d, ", A[0], A[1], A[2]);
            fprintf(tmpFile, "B, %d, %d,\n", B[0], B[1]);
            fprintf(tmpFile, "det_A, %d, inv_detA, %d, delta, %d, %d, disp_xy, %d, %d, [, %.5f, %.5f, ]\n", 
               det_A, inv_detA, A[1]*B[1] - A[2]*B[0], A[1]*B[0] - A[0]*B[1],
               disp_xy[0], disp_xy[1], (float)disp_xy[0]/(1<<LKPYRAMID_FLOATING_COOR_Q), (float)disp_xy[1]/(1<<LKPYRAMID_FLOATING_COOR_Q));
         }
   #endif
   
         for (int i = 0; i < 2; i++)
         {
            if (disp_xy[i] < ((-LKPYRAMID_PATCH_BORDER)<<(LKPYRAMID_FLOATING_COOR_Q)))
               disp_xy[i]  = ((-LKPYRAMID_PATCH_BORDER)<<(LKPYRAMID_FLOATING_COOR_Q));
            if (disp_xy[i] > (LKPYRAMID_PATCH_BORDER<<(LKPYRAMID_FLOATING_COOR_Q)))
               disp_xy[i]  = (LKPYRAMID_PATCH_BORDER<<(LKPYRAMID_FLOATING_COOR_Q));
         }
   
         if (disp_xy[0] == (-LKPYRAMID_PATCH_BORDER)<<(LKPYRAMID_FLOATING_COOR_Q) ||
               disp_xy[0] == LKPYRAMID_PATCH_BORDER<<(LKPYRAMID_FLOATING_COOR_Q)   ||
               disp_xy[1] == (-LKPYRAMID_PATCH_BORDER)<<(LKPYRAMID_FLOATING_COOR_Q) ||
               disp_xy[1] == LKPYRAMID_PATCH_BORDER<<(LKPYRAMID_FLOATING_COOR_Q))
         {
            coor_xy_o[3] = 0;
         }
      }
   }
#if DEBUG_PRINT_OUT
   fprintf(tmpFile, "\n\n");
#endif
   coor_xy_o[0] = (coor_xy_i[0]&0xFFFFFF00) + disp_xy[0];
   coor_xy_o[1] = (coor_xy_i[1]&0xFFFFFF00) + disp_xy[1];

   int32_t sum_dt = 0;
   int16_t dt; 
   for (int i = 0; i < bw_templ; i++)
   {
      for (int j = 0; j < bh_templ; j++)
      {
         dt = *(srcdt+i*dtstride+j);
         sum_dt += (dt < 0 ? -dt : dt);
      }
   }
   coor_xy_o[2] = sum_dt;

   OAL_MemoryFree(srcdt_oal);
   OAL_MemoryFree(patchwin_oal);
}

typedef struct RefLKPyramidResult
{
   int index;
   int status;
   float x_0;
   float y_0;
   float x_1;
   float y_1;
   float err;
} RefLKPyramidResult;

LKPyramidRef::LKPyramidRef():mInit(false), mPyrLayers(0), mBoxSize(0), mNumIter(0)
{
   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
   mSrc0AllOal  = NULL;
   mSrc1AllOal  = NULL;
}

LKPyramidRef::~LKPyramidRef()
{
   Release();
}

void LKPyramidRef::Release()
{
   if (mCoor0Oal)    OAL_MemoryFree(mCoor0Oal);
   if (mCoor1Oal)    OAL_MemoryFree(mCoor1Oal);
   if (mCoor1Oal_O)  OAL_MemoryFree(mCoor1Oal_O);
   if (mSrc0offsOal) OAL_MemoryFree(mSrc0offsOal);
   if (mSrc1offsOal) OAL_MemoryFree(mSrc1offsOal);
   if (mSrc0AllOal)  OAL_MemoryFree(mSrc0AllOal);
   if (mSrc1AllOal)  OAL_MemoryFree(mSrc1AllOal);

   mInit = false;
   mPyrLayers = 0;
   mBoxSize = 0;
   mNumIter = 0;
}

int LKPyramidRef::Initialize (
               icp::DataDescriptor& aSrc0,
               icp::DataDescriptor& ,
               icp::DataDescriptor& aCoor0,
               icp::DataDescriptor& aCoor1,
               icp::DataDescriptor& aCoor1_o,
               int aPyrLayers,
               int aBoxSize,
               int aNumIter,
               int aReqPadding
)
{
   int reval = LKPYRAMID_REF_SUCCESS;

   if (mInit)
   {
      return LKPYRAMID_REF_ALREADY_INITIALIZED;
   }
   mInit = true;

   /* Initialize pyramid layer, box size and interation# */
   if (aPyrLayers >= 0 &&
       (aBoxSize == 3 || aBoxSize == 5 || aBoxSize == 7 || aBoxSize == 9) &&
       aNumIter > 0)
   {
      mPyrLayers  = aPyrLayers;
      mBoxSize    = aBoxSize;
      mNumIter    = aNumIter;
      mReqPadding = aReqPadding;
   
      mWidth = aSrc0.GetWidth();
      mHeight= aSrc0.GetHeight();

      mK0 = 3;
      mK1 = 10;
      mK2 = 3;
   }
   else 
   {
      return LKPYRAMID_REF_INVALID_ARG;
   }

   /* Allocate buffers */
   mCoor0Oal   = OAL_MemoryAllocFlag(RetDataTypeSizeInBytes(aCoor0.GetElementDataType())*aCoor0.GetElementDimX()*aCoor0.GetWidth(), 
                  OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   mCoor1Oal   = OAL_MemoryAllocFlag(RetDataTypeSizeInBytes(aCoor1.GetElementDataType())*aCoor1.GetElementDimX()*aCoor1.GetWidth(), 
                  OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   mCoor1Oal_O = OAL_MemoryAllocFlag(RetDataTypeSizeInBytes(aCoor1_o.GetElementDataType())*aCoor1_o.GetElementDimX()*aCoor1.GetWidth(),
                  OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

   mCoor0Desc.InitManual(aCoor0.GetWidth(),
                           OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_CH_WB),   OAL_MemoryReturnAddress(mCoor0Oal, ACCESS_PHY),
                           aCoor0.GetElementDataType(),
                           aCoor0.GetElementDimX());
   mCoor1Desc.InitManual(aCoor1.GetWidth(),
                           OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_CH_WB),   OAL_MemoryReturnAddress(mCoor1Oal, ACCESS_PHY),
                           aCoor1.GetElementDataType(),
                           aCoor1.GetElementDimX());
   mCoor1Desc_O.InitManual(aCoor1_o.GetWidth(),
                           OAL_MemoryReturnAddress(mCoor1Oal_O, ACCESS_CH_WB), OAL_MemoryReturnAddress(mCoor1Oal_O, ACCESS_PHY),
                           aCoor1_o.GetElementDataType(),
                           aCoor1_o.GetElementDimX());

   {
      mSrc0offsOal = OAL_MemoryAllocFlag(RetDataTypeSizeInBytes(icp::DATATYPE_32S)*aCoor0.GetWidth(), 
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
      mSrc1offsOal = OAL_MemoryAllocFlag(RetDataTypeSizeInBytes(icp::DATATYPE_32S)*aCoor1.GetWidth(), 
                           OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
      mSrc0Offs.InitManual(aCoor0.GetWidth(), 
                           OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY), 
                           icp::DATATYPE_32S, 
                           1);
      mSrc1Offs.InitManual(aCoor0.GetWidth(), 
                          OAL_MemoryReturnAddress( mSrc1offsOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY), 
                           icp::DATATYPE_32S, 
                           1);
   }

   mSpan =  mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID;
   mSrc0AllOal = OAL_MemoryAllocFlag((mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD),
                     OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   mSrc1AllOal = OAL_MemoryAllocFlag((mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD),
                     OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
   memset(OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB), 0, (mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD));
   memset(OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB), 0, (mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD));
   {

      mSrc0AllDesc.InitManual(mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID, mHeight+(mHeight>>1)+4*IMAGE_PAD,
                              OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB),
                              OAL_MemoryReturnAddress(mSrc0AllOal,  ACCESS_PHY),
                              icp::DATATYPE_08U,
                              1, 1);
      mSrc1AllDesc.InitManual(mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID, mHeight+(mHeight>>1)+4*IMAGE_PAD,
                              OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB),
                              OAL_MemoryReturnAddress(mSrc1AllOal,  ACCESS_PHY),
                              icp::DATATYPE_08U,
                              1, 1);

      int c_width = mWidth; int c_height = mHeight;
      uint8_t *pSrc0    = (uint8_t*)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_CH_WB) + mSpan*IMAGE_PAD + IMAGE_PAD;
      uint8_t *pSrc1    = (uint8_t*)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_CH_WB) + mSpan*IMAGE_PAD + IMAGE_PAD;
      uint8_t *pSrc0Phy = (uint8_t*)OAL_MemoryReturnAddress(mSrc0AllOal, ACCESS_PHY) + mSpan*IMAGE_PAD + IMAGE_PAD;
      uint8_t *pSrc1Phy = (uint8_t*)OAL_MemoryReturnAddress(mSrc1AllOal, ACCESS_PHY) + mSpan*IMAGE_PAD + IMAGE_PAD;
      for (int i = 0; i <= MAX_NUM_PYRAMID; i++)
      {
         mSrc0PyrDesc[i].InitManual(c_width, c_height, pSrc0, pSrc0Phy, icp::DATATYPE_08U, 1, 1);
         mSrc1PyrDesc[i].InitManual(c_width, c_height, pSrc1, pSrc1Phy, icp::DATATYPE_08U, 1, 1);
         mSrc0PyrDesc[i].SetSpan(mSpan);
         mSrc1PyrDesc[i].SetSpan(mSpan);
         if (i == 0)
         {
            pSrc0    += ((c_height+2*IMAGE_PAD)*mSpan);
            pSrc1    += ((c_height+2*IMAGE_PAD)*mSpan);
            pSrc0Phy += ((c_height+2*IMAGE_PAD)*mSpan);
            pSrc1Phy += ((c_height+2*IMAGE_PAD)*mSpan);
         }
         else
         {
            pSrc0    += (c_width+2*IMAGE_PAD);
            pSrc1    += (c_width+2*IMAGE_PAD);
            pSrc0Phy += (c_width+2*IMAGE_PAD);
            pSrc1Phy += (c_width+2*IMAGE_PAD);
         }
         c_width >>= 1; c_height >>= 1;
      }
   }

   return reval;
}

int LKPyramidRef::Process(
               icp::DataDescriptor& aSrc0,
               icp::DataDescriptor& aSrc1,
               icp::DataDescriptor& aCoor0,
               icp::DataDescriptor& aCoor1,
               icp::DataDescriptor& aCoor1_o
)
{
   int reval = 0;


   if (mInit)
   {
      uint8_t *s = (uint8_t*)aSrc0.GetDataPtr();
      uint8_t *p = (uint8_t*)mSrc0PyrDesc[0].GetDataPtr();
      int src_span   = aSrc0.GetSpan();

      for (int y = 0; y < (int)mHeight; y++)
      {
         memcpy(p, s, mWidth);
         p += mSpan;
         s += src_span;
      }
      s = (uint8_t*)aSrc1.GetDataPtr();
      p = (uint8_t*)mSrc1PyrDesc[0].GetDataPtr();
      src_span   = aSrc1.GetSpan();
      for (int y = 0; y < (int)mHeight; y++)
      {
         memcpy(p, s, mWidth);
         p += mSpan;
         s += src_span;
      }

      int c_width  = mWidth;
      int c_height = mHeight;
      for (int i = 0; i < MAX_NUM_PYRAMID; i++)
      {
         void  *s_oal = OAL_MemoryAllocFlag(c_width*c_height, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
         void  *p_oal = OAL_MemoryAllocFlag((c_width/2)*(c_height/2), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
         uint8_t *s = (uint8_t*)OAL_MemoryReturnAddress(s_oal, ACCESS_CH_WB);
         uint8_t *p = (uint8_t*)OAL_MemoryReturnAddress(p_oal, ACCESS_CH_WB);
         for (int y = 0; y < (int)c_height; y++)
         {
            memcpy(s+y*c_width, (uint8_t *)mSrc0PyrDesc[i].GetDataPtr()+y*mSrc0PyrDesc[i].GetSpan(), c_width);
         } 
         _run_reference_pyramid_down(s, c_width, c_height, p);
         for (int y = 0; y < (int)(c_height/2); y++)
         {
            memcpy((uint8_t *)mSrc0PyrDesc[i+1].GetDataPtr()+y*mSrc0PyrDesc[i+1].GetSpan(), p+y*(c_width/2), c_width/2);
         } 

         for (int y = 0; y < (int)c_height; y++)
         {
            memcpy(s+y*c_width, (uint8_t *)mSrc1PyrDesc[i].GetDataPtr()+y*mSrc1PyrDesc[i].GetSpan(), c_width);
         } 
         _run_reference_pyramid_down(s, c_width, c_height, p);
         for (int y = 0; y < (int)c_height/2; y++)
         {
            memcpy((uint8_t *)mSrc1PyrDesc[i+1].GetDataPtr()+y*mSrc1PyrDesc[i+1].GetSpan(), p+y*(c_width/2), c_width/2);
         } 
         OAL_MemoryFree(s_oal);
         OAL_MemoryFree(p_oal);
         c_width >>= 1; c_height >>= 1;
      }

      /* perform padding */
      if (mReqPadding)
      {
         for (int i = 0; i <= mPyrLayers; i++)
         {
            mSrc0PyrDesc[i].E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
            mSrc1PyrDesc[i].E0Replicate(IMAGE_PAD, IMAGE_PAD, IMAGE_PAD, IMAGE_PAD);
         }
      }

#if DEBUG_PRINT_OUT
      {
         FILE *tmpF = fopen("data/apexcv_pro/apexcv_pro_lkpyramid/output/src0all_ref.raw", "wb");
         fwrite(mSrc0AllDesc.GetDataPtr(), (mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD), 1, tmpF);
         fclose(tmpF);

         tmpF = fopen("data/apexcv_pro/apexcv_pro_lkpyramid/output/src1all_ref.raw", "wb");
         fwrite(mSrc1AllDesc.GetDataPtr(), (mWidth+2*IMAGE_PAD*MAX_NUM_PYRAMID)*(mHeight+(mHeight>>1)+4*IMAGE_PAD), 1, tmpF);
         fclose(tmpF);

         for (int i = 0; i <= mPyrLayers; i++)
         {
            char fname[256];
            sprintf(fname, "data/apexcv_pro/apexcv_pro_lkpyramid/output/src0.%d_ref.raw", i);
            tmpF = fopen(fname, "wb");
            uint8_t *s = (uint8_t*)mSrc0PyrDesc[i].GetDataPtr();
            for (int y = 0; y < (int)mSrc0PyrDesc[i].GetHeight(); y++)
            {
               fwrite(s, mSrc0PyrDesc[i].GetWidth(), 1, tmpF);
               s += mSrc0PyrDesc[i].GetSpan();
            }
            fclose(tmpF);

            sprintf(fname, "data/apexcv_pro/apexcv_pro_lkpyramid/output/src0.%d_ref.csv", i);
            tmpF = fopen(fname, "wb");
            s = (uint8_t*)mSrc0PyrDesc[i].GetDataPtr();
            fprintf(tmpF, ", ");
            for (int x = 0; x < (int)mSrc0PyrDesc[i].GetWidth(); x++)
            {
               fprintf(tmpF, "%d, ", x);
            }
            fprintf(tmpF, "\n");
            for (int y = 0; y < (int)mSrc0PyrDesc[i].GetHeight(); y++)
            {
               fprintf(tmpF, "%d, ", y);
               for (int x = 0; x < (int)mSrc0PyrDesc[i].GetWidth(); x++)
               {
                  fprintf(tmpF, "%d, ", s[x]);
               }
               fprintf(tmpF, "\n");
               s += mSrc0PyrDesc[i].GetSpan();
            }
            fclose(tmpF);

            sprintf(fname, "data/apexcv_pro/apexcv_pro_lkpyramid/output/src1.%d_ref.raw", i);
            tmpF = fopen(fname, "wb");
            s = (uint8_t*)mSrc1PyrDesc[i].GetDataPtr();
            for (int y = 0; y < (int)mSrc1PyrDesc[i].GetHeight(); y++)
            {
               fwrite(s, mSrc1PyrDesc[i].GetWidth(), 1, tmpF);
               s += mSrc1PyrDesc[i].GetSpan();
            }
            fclose(tmpF);
            sprintf(fname, "data/apexcv_pro/apexcv_pro_lkpyramid/output/src1.%d_ref.csv", i);
            tmpF = fopen(fname, "wb");
            s = (uint8_t*)mSrc1PyrDesc[i].GetDataPtr();
            fprintf(tmpF, ", ");
            for (int x = 0; x < (int)mSrc1PyrDesc[i].GetWidth(); x++)
            {
               fprintf(tmpF, "%d, ", x);
            }
            fprintf(tmpF, "\n");
            for (int y = 0; y < (int)mSrc1PyrDesc[i].GetHeight(); y++)
            {
               fprintf(tmpF, "%d, ", y);
               for (int x = 0; x < (int)mSrc1PyrDesc[i].GetWidth(); x++)
               {
                  fprintf(tmpF, "%d, ", s[x]);
               }
               fprintf(tmpF, "\n");
               s += mSrc1PyrDesc[i].GetSpan();
            }
            fclose(tmpF);
         }
      }
#endif

      /* iterate pyramids */
      for (int pyr = mPyrLayers; pyr >= 0; pyr--)
      {
         memcpy(mCoor0Desc.GetDataPtr(), aCoor0.GetDataPtr(), RetDataTypeSizeInBytes(aCoor0.GetElementDataType())*aCoor0.GetElementDimX()*aCoor0.GetWidth());
         memcpy(mCoor1Desc.GetDataPtr(), aCoor1.GetDataPtr(), RetDataTypeSizeInBytes(aCoor1.GetElementDataType())*aCoor1.GetElementDimX()*aCoor1.GetWidth());

         int32_t* p   = (int32_t*) aCoor0.GetDataPtr();
         int32_t* s   = (int32_t*)mCoor0Desc.GetDataPtr();
         for (uint32_t i = 0; i < aCoor0.GetWidth(); i++)
         {
            int32_t x = p[i*aCoor0.GetElementDimX()]   >> pyr;
            int32_t y = p[i*aCoor0.GetElementDimX()+1] >> pyr;
            s[i*mCoor0Desc.GetElementDimX()  ] = x;
            s[i*mCoor0Desc.GetElementDimX()+1] = y;
         }

         s   = (int32_t*)mCoor1Desc.GetDataPtr();
         if (pyr == mPyrLayers)
         {
            p   = (int32_t*) aCoor1.GetDataPtr();
            for (uint32_t i = 0; i < aCoor1.GetWidth(); i++)
            {
               int32_t x = p[i*aCoor1.GetElementDimX()]   >> pyr;
               int32_t y = p[i*aCoor1.GetElementDimX()+1] >> pyr;
               s[i*mCoor1Desc.GetElementDimX()  ] = x;
               s[i*mCoor1Desc.GetElementDimX()+1] = y;
            }
         }
         else
         {
            p   = (int32_t*) mCoor1Desc_O.GetDataPtr();
            for (uint32_t i = 0; i < mCoor1Desc_O.GetWidth(); i++)
            {
               int32_t x = p[i*mCoor1Desc_O.GetElementDimX()]   << 1;
               int32_t y = p[i*mCoor1Desc_O.GetElementDimX()+1] << 1;
               s[i*mCoor1Desc.GetElementDimX()  ] = x;
               s[i*mCoor1Desc.GetElementDimX()+1] = y;
            }
         }

         ref_lkpyramid_calc_offset(mSrc0PyrDesc[pyr], mCoor0Desc, mSrc0Offs, mBoxSize>>1, 1); // pad 1 for 3x3 scharr gradient
         ref_lkpyramid_calc_offset(mSrc1PyrDesc[pyr], mCoor1Desc, mSrc1Offs, mBoxSize>>1, 0); 

         if (mBoxSize == 3)
         {
         }
         else if (mBoxSize == 5)
         {
         }
         else if (mBoxSize == 7)
         {
            /* inter-kernel buffers - cmem */
            void *templ_interpolated_oal = OAL_MemoryAllocFlag((mBoxSize+2)*(mBoxSize+2), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
            void *gradient_dx_oal = OAL_MemoryAllocFlag(mBoxSize*mBoxSize*sizeof(int16_t), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
            void *gradient_dy_oal = OAL_MemoryAllocFlag(mBoxSize*mBoxSize*sizeof(int16_t), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
            void *templ_interpolated = OAL_MemoryReturnAddress(templ_interpolated_oal, ACCESS_CH_WB);
            void *gradient_dx = OAL_MemoryReturnAddress(gradient_dx_oal, ACCESS_CH_WB);
            void *gradient_dy = OAL_MemoryReturnAddress(gradient_dy_oal, ACCESS_CH_WB);
#if DEBUG_PRINT_OUT
         	FILE* tmpFile = fopen("data/apexcv_pro/apexcv_pro_lkpyramid/output/ref_template_dump.csv", "wb");
#endif
            for (uint32_t pt_idx = 0; pt_idx < mCoor0Desc.GetWidth(); pt_idx++)
            {
#if DEBUG_PRINT_OUT
               {
                  int32_t* coor_xy_0 = (int32_t*)mCoor0Desc.GetDataPtr()+mCoor0Desc.GetElementDimX()*pt_idx;
                  int32_t* coor_xy_1 = (int32_t*)mCoor1Desc.GetDataPtr()+mCoor1Desc.GetElementDimX()*pt_idx;
                  fprintf(tmpFile, "coor_0 %d %d coor_1 %d %d\n", 
                        coor_xy_0[0]/(1<<LKPYRAMID_FLOATING_COOR_Q), 
                        coor_xy_0[1]/(1<<LKPYRAMID_FLOATING_COOR_Q), 
                        coor_xy_1[0]/(1<<LKPYRAMID_FLOATING_COOR_Q), 
                        coor_xy_1[1]/(1<<LKPYRAMID_FLOATING_COOR_Q));
               }
#endif
               ref_lkpyramid_bilinear_08u(
                  (uint8_t*)templ_interpolated, mBoxSize+2,  
                  (uint8_t*)mSrc0PyrDesc[pyr].GetDataPtr()+(*((int32_t*)mSrc0Offs.GetDataPtr()+pt_idx)), mSrc0PyrDesc[pyr].GetSpan(), 
                  (int32_t*)mCoor0Desc.GetDataPtr()+mCoor0Desc.GetElementDimX()*pt_idx,
                  mBoxSize+2, mBoxSize+2,
                  LKPYRAMID_FLOATING_COOR_Q);

#if DEBUG_PRINT_OUT
               if (pt_idx == 0 && pyr == 1)
               {
                  printf("template:\n");
                  uint8_t *p = (uint8_t*)mSrc0PyrDesc[pyr].GetDataPtr()+(*((int32_t*)mSrc0Offs.GetDataPtr()+pt_idx));
                  for (int i = 0; i < mBoxSize+2; i++)
                  {
                     printf("%3d ", p[i]);
                  }                  
                  printf("\n"); p += mSrc0PyrDesc[pyr].GetSpan();
               }
#endif
   
               ref_flt_centraldx_3x3(
                  (int16_t*)gradient_dx, mBoxSize,
                  (uint8_t*)templ_interpolated+(mBoxSize+2)*1+1, (mBoxSize+2),
                  mBoxSize, mBoxSize,
                  mK0, mK1, mK2);
   
               ref_flt_centraldy_3x3(
                  (int16_t*)gradient_dy, mBoxSize,
                  (uint8_t*)templ_interpolated+(mBoxSize+2)*1+1, (mBoxSize+2),
                  mBoxSize, mBoxSize,
                  mK0, mK1, mK2);
   
#if DEBUG_PRINT_OUT
               for (int i = 0; i < mBoxSize+2; i++)
               {
                  fprintf(tmpFile, "templ, ");
                  for (int j = 0; j < mBoxSize+2; j++)
                  {
                     fprintf(tmpFile, "%d, ", *((uint8_t*)templ_interpolated+i*(mBoxSize+2)+j));
                  }
                  if (i < mBoxSize)
                  {
                  	fprintf(tmpFile, "dx, ");
                  	for (int j = 0; j < mBoxSize; j++)
                  	{
                  	   fprintf(tmpFile, "%d, ", *((int16_t*)gradient_dx+i*mBoxSize+j));
                  	}
                  	fprintf(tmpFile, "dy, ");
                  	for (int j = 0; j < mBoxSize; j++)
                  	{
                  	   fprintf(tmpFile, "%d, ", *((int16_t*)gradient_dy+i*mBoxSize+j));
                  	}
                  	fprintf(tmpFile, "\n");
                  }
                  else 
                  {
                  	fprintf(tmpFile, "\n");
                  }
               }
               fprintf(tmpFile, "\n\n");
#endif
   
               int32_t* coor_xy_o = (int32_t*)mCoor1Desc_O.GetDataPtr()+mCoor1Desc_O.GetElementDimX()*pt_idx;
               uint8_t* srcpatch = (uint8_t*)mSrc1PyrDesc[pyr].GetDataPtr()+(*((int32_t*)mSrc1Offs.GetDataPtr()+pt_idx));
               uint8_t* srctempl = (uint8_t*)templ_interpolated+1*9+1;
               int32_t* coor_xy_i = (int32_t*)mCoor1Desc.GetDataPtr()+mCoor1Desc.GetElementDimX()*pt_idx;
   
               ref_lkpyramid_core(
                  coor_xy_o,
                  srcpatch, mSrc1PyrDesc[pyr].GetSpan(),
                  srctempl, mBoxSize+2,
                  (int16_t*)gradient_dx, mBoxSize,
                  (int16_t*)gradient_dy, mBoxSize,
                  coor_xy_i,
                  mBoxSize, mBoxSize,
                  mBoxSize+2*LKPYRAMID_PATCH_BORDER, mBoxSize+2*LKPYRAMID_PATCH_BORDER,
                  mNumIter,
#if DEBUG_PRINT_OUT
                  tmpFile
#else
                  NULL
#endif
               );               
            }
   
#if DEBUG_PRINT_OUT
            fclose(tmpFile);
#endif
            if (templ_interpolated_oal) OAL_MemoryFree(templ_interpolated_oal);
            if (gradient_dx_oal) OAL_MemoryFree(gradient_dx_oal);
            if (gradient_dy_oal) OAL_MemoryFree(gradient_dy_oal);
         }
         else  /* == 9 */
         {
         }
      }
   }
   else
   {
      reval = 1;
      return reval;
   }
   {
      int32_t *p   = (int32_t*) mCoor1Desc_O.GetDataPtr();
      for (uint32_t i = 0; i < mCoor1Desc_O.GetWidth(); i++)
      {
         int32_t x = p[i*mCoor1Desc_O.GetElementDimX()]   << 1;
         int32_t y = p[i*mCoor1Desc_O.GetElementDimX()+1] << 1;
         p[i*mCoor1Desc_O.GetElementDimX()  ] = x>>1;
         p[i*mCoor1Desc_O.GetElementDimX()+1] = y>>1;
      }
   }
   memcpy(aCoor1_o.GetDataPtr(), mCoor1Desc_O.GetDataPtr(), RetDataTypeSizeInBytes(aCoor1_o.GetElementDataType())*aCoor1_o.GetElementDimX()*aCoor1_o.GetWidth());

   return reval;
}

