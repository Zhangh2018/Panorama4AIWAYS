/*****************************************************************************
 * *
 * * NXP Confidential Proprietary
 * *
 * * Copyright (c) 2015 NXP Semiconductor;
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

#include "lktracker_ref.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __STANDALONE__
#include "t32term.h"
#define DD_LKTRACKER_PATH_DST "C:\\vsdk\\data\\apexcv_pro\\apexcv_pro_lktracker\\output\\"
#endif

#define LKTRACKER_FLOATING_COOR_Q    8
#define LKTRACKER_MAX_TRACKING_PT    1024

#define LKTRACKER_PATCH_BORDER 6

#define DEBUG_PRINT_OUT 0


/* Fix-point math simulation */
static uint64_t getA(uint64_t x){return (x>>56)&0xFF;}
static uint64_t getB(uint64_t x){return (x>>48)&0xFF;}
static uint64_t getC(uint64_t x){return (x>>40)&0xFF;}
static uint64_t getD(uint64_t x){return (x>>32)&0xFF;}
static uint64_t getE(uint64_t x){return (x>>24)&0xFF;}
static uint64_t getF(uint64_t x){return (x>>16)&0xFF;}
static uint64_t getG(uint64_t x){return (x>> 8)&0xFF;}
static uint64_t getH(uint64_t x){return (x>> 0)&0xFF;}
static uint64_t getM(uint64_t x){return (x>>56)&0xFF;}
static uint64_t getN(uint64_t x){return (x>>48)&0xFF;}
static uint64_t getO(uint64_t x){return (x>>40)&0xFF;}
static uint64_t getP(uint64_t x){return (x>>32)&0xFF;}
static uint64_t getQ(uint64_t x){return (x>>24)&0xFF;}
static uint64_t getR(uint64_t x){return (x>>16)&0xFF;}
static uint64_t getS(uint64_t x){return (x>> 8)&0xFF;}
static uint64_t getT(uint64_t x){return (x>> 0)&0xFF;}

static int count_leading_zero(int16_t x)
{
   uint8_t n = 0;
   uint16_t abs_x = (uint16_t)x;
   if (x < 0)
   {
      abs_x = ~x;
   }
   for (int i = 0; i < 16; i++)
   {
      if (abs_x & 0x8000) break;
      n++;
      abs_x <<= 1;
   }
   n = n-1;
   return n;
}

static int count_leading_zero(int32_t /*in*/)
{
	uint8_t n = 0;
   return n;
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


static uint64_t mul1(uint64_t abcd, uint64_t mnop)
{
   uint64_t am = getA(abcd)*getM(mnop) << 48;
   uint64_t an = getA(abcd)*getN(mnop) << 40;
   uint64_t bm = getB(abcd)*getM(mnop) << 40;

   return (am  +
            an +
            bm  ) & 0xFFFF000000000000;
}

static uint64_t mul2(uint64_t abcd, uint64_t mnop)
{
   uint64_t am = getA(abcd)*getM(mnop) << 48;
   uint64_t an = getA(abcd)*getN(mnop) << 40;

   return (am  +
            an)  & 0xFFFF000000000000;
}

static uint16_t fxrecip12(uint16_t d)
{
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

static void ref_lktracker_calc_offset(icp::DataDescriptor& SrcDesc, icp::Feature32SDescriptor& CoorDesc, icp::DataDescriptor& OffsDesc, icp::DataDescriptor& FracDesc, int half_win, int pad)
{
   int32_t* p_offset = (int32_t*)OffsDesc.GetDataPtr();
   uint8_t* p_frac   = (uint8_t*)FracDesc.GetDataPtr();

   for (int i = 0; i < CoorDesc.GetCount(); i++)
   {
      int32_t x = CoorDesc.GetFeature(i).position.x;
      int32_t y = CoorDesc.GetFeature(i).position.y;
      int32_t i_x = x >> LKTRACKER_FLOATING_COOR_Q;
      int32_t i_y = y >> LKTRACKER_FLOATING_COOR_Q;
      p_offset[i] = SrcDesc.GetSpan() * i_y + RetDataTypeSizeInBytes(SrcDesc.GetElementDataType())*i_x;
      p_offset[i] -= (SrcDesc.GetSpan()*(half_win+pad) + (half_win+pad)*RetDataTypeSizeInBytes(SrcDesc.GetElementDataType()));

      int16_t mask = 0x0;
      for (int i = 0; i < LKTRACKER_FLOATING_COOR_Q; i++)
      {
         mask = mask | (1<<i);
      }
      p_frac[i*FracDesc.GetElementDimX()]   = (uint8_t)(x & mask);
      p_frac[i*FracDesc.GetElementDimX()+1] = (uint8_t)(y & mask);
   }
}

static void ref_lktracker_calc_offset(icp::DataDescriptor& SrcDesc, icp::Feature32SDescriptor& CoorDesc, icp::DataDescriptor& OffsDesc, int half_win, int pad)
{
   int32_t* p_offset = (int32_t*)OffsDesc.GetDataPtr();

   for (int i = 0; i < CoorDesc.GetCount(); i++)
   {
      int32_t x = CoorDesc.GetFeature(i).position.x;
      int32_t y = CoorDesc.GetFeature(i).position.y;
      int32_t i_x = x >> LKTRACKER_FLOATING_COOR_Q;
      int32_t i_y = y >> LKTRACKER_FLOATING_COOR_Q;
      p_offset[i] = SrcDesc.GetSpan() * i_y + RetDataTypeSizeInBytes(SrcDesc.GetElementDataType())*i_x;
      p_offset[i] -= (SrcDesc.GetSpan()*(half_win+pad) + (half_win+pad)*RetDataTypeSizeInBytes(SrcDesc.GetElementDataType()));
   }
}

static inline void ref_inline_lktracker_bilinear_08u(
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

void ref_lktracker_bilinear_08u(
      uint8_t* dst, int dstr,
      uint8_t* src, int sstr,
      uint8_t* frac_xy,
      int bw, int bh,
      int shift)
{

   ref_inline_lktracker_bilinear_08u(
            dst, dstr,
            src, sstr,
            frac_xy,
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

void ref_lktracker_core(
      int32_t* coor_xy_o,                  /* output updated coordinate */
      uint8_t* srcpatch, int patchstride,  /* patch */
      uint8_t* srctempl, int templstride,  /* template */
      int16_t* srcdx,    int dxstride,     /* template x gradient */
      int16_t* srcdy,    int dystride,     /* template y gradient */
      int32_t* coor_xy_i,                  /* input coordinate */
      int bw_templ, int bh_templ,          /* template block width, height */
      int bw_patch, int bh_patch,          /* patch    block width, height */
      int mNumIter, 
      int max_x, int max_y,
      FILE* tmpFile)
{
   int16_t* srcdt_oal = (int16_t*)OAL_MemoryAllocFlag(sizeof(int16_t)*bw_templ*bh_templ, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   int16_t* srcdt    = (int16_t*)OAL_MemoryReturnAddress(srcdt_oal, ACCESS_CH_WB);
   uint8_t* patchwin_oal = (uint8_t*)OAL_MemoryAllocFlag(bw_templ*bh_templ, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   uint8_t* patchwin = (uint8_t*)OAL_MemoryReturnAddress(patchwin_oal, ACCESS_CH_WB);
   int dtstride = bw_templ;
   int patchwinstride = bw_templ;


   int16_t A[3]; /* [dxdx, dxdy, dydy] */
   int16_t B[2]; /* [dxdt, dydt] */
   int16_t det_A = 0;
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
         min_a_clz < 0  ? 0  : min_a_clz;
         min_a_clz > 31 ? 31 : min_a_clz;
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

   int clz_a = 0;

   coor_xy_o[3] = 1;
   for (int iter = 0; iter < mNumIter; iter++)
   { 
      int16_t i_x = disp_xy[0] >> LKTRACKER_FLOATING_COOR_Q;
      int16_t i_y = disp_xy[1] >> LKTRACKER_FLOATING_COOR_Q;
      int16_t offset = i_y * patchstride + i_x;
      int16_t mask = 0x0;
      for (int i = 0; i < LKTRACKER_FLOATING_COOR_Q; i++)
      {
         mask = mask | (1<<i);
      }
      uint8_t frac[2] = {(uint8_t)(disp_xy[0] & mask), (uint8_t)(disp_xy[1] & mask)};

      ref_inline_lktracker_bilinear_08u(
         patchwin, patchwinstride,
         srcpatch+offset, patchstride, 
         frac,
         bw_templ, bh_templ,
         LKTRACKER_FLOATING_COOR_Q);
         
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
               {srcdt
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
   
         int clz_b = 0;
         B[0] = ((sum_B0<<min_b_clz) + (1<<15)) >> 16;
         B[1] = ((sum_B1<<min_b_clz) + (1<<15)) >> 16;
      }

      {
         int32_t delta_0 = A[1]*B[1] - A[2]*B[0];
         int32_t delta_1 = A[1]*B[0] - A[0]*B[1];

         disp_shift = count_leading_zero((uint16_t)((abs(delta_0)>>16) | (abs(delta_1)>>16))) - 1;
         disp_shift < 0  ? 0  : disp_shift;
         disp_shift > 31 ? 31 : disp_shift;
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
               disp_xy[0], disp_xy[1], (float)disp_xy[0]/(1<<LKTRACKER_FLOATING_COOR_Q), (float)disp_xy[1]/(1<<LKTRACKER_FLOATING_COOR_Q));
         }
   #endif
   
         for (int i = 0; i < 2; i++)
         {
            if (disp_xy[i] < ((-LKTRACKER_PATCH_BORDER)<<LKTRACKER_FLOATING_COOR_Q))
               disp_xy[i]  = ((-LKTRACKER_PATCH_BORDER)<<LKTRACKER_FLOATING_COOR_Q);
            if (disp_xy[i] > (LKTRACKER_PATCH_BORDER<<LKTRACKER_FLOATING_COOR_Q))
               disp_xy[i]  = (LKTRACKER_PATCH_BORDER<<LKTRACKER_FLOATING_COOR_Q);
         }
   
         if (disp_xy[0] == (-LKTRACKER_PATCH_BORDER)<<LKTRACKER_FLOATING_COOR_Q ||
               disp_xy[0] == LKTRACKER_PATCH_BORDER<<LKTRACKER_FLOATING_COOR_Q   ||
               disp_xy[1] == (-LKTRACKER_PATCH_BORDER)<<LKTRACKER_FLOATING_COOR_Q ||
               disp_xy[1] == LKTRACKER_PATCH_BORDER<<LKTRACKER_FLOATING_COOR_Q)
         {
            coor_xy_o[3] = 0;
            break;
         }
      }
   }
#if DEBUG_PRINT_OUT
   fprintf(tmpFile, "\n\n");
#endif
   coor_xy_o[0] = (coor_xy_i[0] & 0xFFFFFF00) + disp_xy[0];
   coor_xy_o[1] = (coor_xy_i[1] & 0xFFFFFF00) + disp_xy[1];

   if (coor_xy_o[0] > (max_x << LKTRACKER_FLOATING_COOR_Q)) coor_xy_o[0] = (max_x << LKTRACKER_FLOATING_COOR_Q);
   if (coor_xy_o[0] < 0)     coor_xy_o[0] = 0;
   if (coor_xy_o[1] > (max_y << LKTRACKER_FLOATING_COOR_Q)) coor_xy_o[1] = (max_y << LKTRACKER_FLOATING_COOR_Q);
   if (coor_xy_o[1] < 0)     coor_xy_o[1] = 0;

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

typedef struct RefLKTrackerResult
{
   int index;
   int status;
   float x_0;
   float y_0;
   float x_1;
   float y_1;
   float err;
} RefLKTrackerResult;

LKTrackerRef::LKTrackerRef():mInit(false), mPyrLayers(0), mBoxSize(0), mNumIter(0)
{
   mK0Oal = NULL;
   mK1Oal = NULL;
   mK2Oal = NULL;
   mFracxy0Oal  = NULL;
   mSrc0offsOal = NULL;
   mSrc1offsOal = NULL;
}

LKTrackerRef::~LKTrackerRef()
{
   Release();
}

void LKTrackerRef::Release()
{
   if (mK0Oal) OAL_MemoryFree(mK0Oal);
   if (mK1Oal) OAL_MemoryFree(mK1Oal);
   if (mK2Oal) OAL_MemoryFree(mK2Oal);
   if (mFracxy0Oal)  OAL_MemoryFree(mFracxy0Oal);
   if (mSrc0offsOal) OAL_MemoryFree(mSrc0offsOal);
   if (mSrc1offsOal) OAL_MemoryFree(mSrc1offsOal);

   mInit = false;
   mPyrLayers = 0;
   mBoxSize = 0;
   mNumIter = 0;
}

int LKTrackerRef::Initialize (
               vsdk::SMat& aSrc0,
               vsdk::SMat& aSrc1,
               icp::Feature32SDescriptor& aCoor0,
               icp::Feature32SDescriptor& aCoor1,
               icp::Feature32SDescriptor& aCoor1_o,
               int aBoxSize,
               int aNumIter
)
{
   int reval = LKTRACKER_REF_SUCCESS;

   if (mInit)
   {
      return LKTRACKER_REF_ALREADY_INITIALIZED;
   }
   mInit = true;

   /* Initialize pyramid layer, box size and interation# */
   if ((aBoxSize == 3 || aBoxSize == 5 || aBoxSize == 7 || aBoxSize == 9) &&
      aNumIter > 0)
   {
      mBoxSize   = aBoxSize;
      mNumIter   = aNumIter;
   }
   else {
      return LKTRACKER_REF_INVALID_ARG;
   }

   /* Allocate buffers */
   if (aSrc0.type() == VSDK_CV_8UC1 &&
      aSrc1.type() == VSDK_CV_8UC1)
   {
      //int width  = aSrc0.GetWidth();
      //int height = aSrc0.GetHeight();

      mCoor0Desc.Init(aCoor0.GetDataPtr(), aCoor0.GetDataPtrPhys(),aCoor0.GetSize());
      mCoor0Desc.SetCount(aCoor0.GetCount());
      mCoor1Desc.Init(aCoor1.GetDataPtr(), aCoor1.GetDataPtrPhys(),aCoor1.GetSize());
      mCoor1Desc.SetCount(aCoor1.GetCount());
      mCoor1Desc_O.Init(aCoor1_o.GetDataPtr(), aCoor1_o.GetDataPtrPhys(),aCoor1_o.GetSize());
      mCoor1Desc_O.SetCount(aCoor1_o.GetCount());

      /* Scharr gradient coef */
      {
         mK0Desc.Init(1, 1, icp::DATATYPE_08S);
         mK1Desc.Init(1, 1, icp::DATATYPE_08S);
         mK2Desc.Init(1, 1, icp::DATATYPE_08S);
         mK0Oal = mK0Desc.GetDataPtrPhys();
         mK1Oal = mK1Desc.GetDataPtrPhys();
         mK2Oal = mK2Desc.GetDataPtrPhys();
         *((int8_t*)OAL_MemoryReturnAddress(mK0Oal, ACCESS_CH_WB)) = 3;
         *((int8_t*)OAL_MemoryReturnAddress(mK1Oal, ACCESS_CH_WB)) = 10;
         *((int8_t*)OAL_MemoryReturnAddress(mK2Oal, ACCESS_CH_WB)) = 3;
         if (!(mK0Oal && mK1Oal && mK2Oal))
         {
            return LKTRACKER_REF_OUT_OF_MEMORY;
         }
      }

      /* source images */
      {
         mSrc0Desc.InitManual(aSrc0.cols, aSrc0.rows,
                                    aSrc0.data, aSrc0.u->handle + (aSrc0.data - aSrc0.datastart),
                                    icp::DATATYPE_08U, 1, 1);
         mSrc1Desc.InitManual(aSrc1.cols, aSrc1.rows,
                                    aSrc1.data, aSrc1.u->handle + (aSrc1.data - aSrc1.datastart),
                                    icp::DATATYPE_08U, 1, 1);
         mSrc0Desc.SetSpan(aSrc0.step[0]);
         mSrc1Desc.SetSpan(aSrc1.step[0]);
      }

      {
         mFracxy0Oal = OAL_MemoryAllocFlag(sizeof(uint8_t) * 2 * aCoor0.GetSize(), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mFracXY0Desc.InitManual(aCoor0.GetSize(), OAL_MemoryReturnAddress(mFracxy0Oal, ACCESS_CH_WB), OAL_MemoryReturnAddress(mFracxy0Oal, ACCESS_PHY), icp::DATATYPE_08U, 2);
      }

      {
         mSrc0offsOal = OAL_MemoryAllocFlag(sizeof(int32_t) * aCoor0.GetSize(), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc1offsOal = OAL_MemoryAllocFlag(sizeof(int32_t) * aCoor0.GetSize(), OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
         mSrc0Offs.InitManual(aCoor0.GetSize(), OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(mSrc0offsOal, ACCESS_PHY), icp::DATATYPE_32S, 1);
         mSrc1Offs.InitManual(aCoor0.GetSize(), OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_CH_WB), OAL_MemoryReturnAddress(mSrc1offsOal, ACCESS_PHY), icp::DATATYPE_32S, 1);
      }
   }

   return reval;

}

int LKTrackerRef::Process()
{
   int reval = 0;
   int PAD = 16;

#if DEBUG_PRINT_OUT
   {
      char filename[256];
      sprintf(filename, "%ssrc0_img.csv", DD_LKTRACKER_PATH_DST);
   	FILE* tmpFile = fopen(filename, "wb");
   	fprintf(tmpFile, ", ");
   	for (int col = 0; col < mSrc0Desc.GetWidth(); col++)
   	{
   	  fprintf(tmpFile, "%d, ", col);
   	}
   	fprintf(tmpFile, "\n");
		for (int row = 0; row < mSrc0Desc.GetHeight(); row++)
   	{
   	   fprintf(tmpFile, "%d, ", row);
   	   for (int col = 0; col < mSrc0Desc.GetWidth(); col++)
   	   {
   	      fprintf(tmpFile, "%d, ", *((uint8_t*)mSrc0Desc.GetDataPtr()+row*mSrc0Desc.GetSpan()+col));
   	   }
   	   fprintf(tmpFile, "\n");
   	}
   	fclose(tmpFile);

      sprintf(filename, "%ssrc1_img.csv", DD_LKTRACKER_PATH_DST);
   	tmpFile = fopen(filename, "wb");
   	fprintf(tmpFile, ", ");
   	for (int col = 0; col < mSrc1Desc.GetWidth(); col++)
   	{
   	  fprintf(tmpFile, "%d, ", col);
   	}
   	fprintf(tmpFile, "\n");
		for (int row = 0; row < mSrc1Desc.GetHeight(); row++)
   	{
   	   fprintf(tmpFile, "%d, ", row);
   	   for (int col = 0; col < mSrc1Desc.GetWidth(); col++)
   	   {
   	      fprintf(tmpFile, "%d, ", *((uint8_t*)mSrc1Desc.GetDataPtr()+row*mSrc1Desc.GetSpan()+col));
   	   }
   	   fprintf(tmpFile, "\n");
   	}
   	fclose(tmpFile);
   }
#endif

   /* allocated pad memory for worst case */
   const uint32_t cSrcXPadOal_SizeInBytes = (mSrc0Desc.GetWidth()+PAD*2)*(mSrc0Desc.GetHeight()+PAD*2);

   void *src0_pad_oal = OAL_MemoryAllocFlag(cSrcXPadOal_SizeInBytes, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);
   void *src1_pad_oal = OAL_MemoryAllocFlag(cSrcXPadOal_SizeInBytes, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE) | OAL_MEMORY_FLAG_CONTIGUOUS);

   const uintptr_t cSrc1PadOal_End = ((uintptr_t)src1_pad_oal) + cSrcXPadOal_SizeInBytes;

   if (!(src0_pad_oal && src1_pad_oal))
   {
      if (src0_pad_oal != 0) OAL_MemoryFree(src0_pad_oal);
      if (src1_pad_oal != 0) OAL_MemoryFree(src1_pad_oal);
      reval = 1;
      return reval;
   }

   if (mInit)
   {
      /* pad source */
      icp::DataDescriptor Src0PadDesc;
      icp::DataDescriptor Src1PadDesc;
      Src0PadDesc.InitManual(mSrc0Desc.GetWidth(), mSrc0Desc.GetHeight(),
                              (uint8_t*)OAL_MemoryReturnAddress(src0_pad_oal, ACCESS_CH_WB) + PAD + PAD*(mSrc0Desc.GetWidth()+2*PAD),
                              (uint8_t*)OAL_MemoryReturnAddress(src0_pad_oal, ACCESS_PHY) + PAD + PAD*(mSrc0Desc.GetWidth()+2*PAD),
                              icp::DATATYPE_08U,
                              1, 1);
      Src0PadDesc.SetSpan(mSrc0Desc.GetWidth()+2*PAD);
      Src1PadDesc.InitManual(mSrc1Desc.GetWidth(), mSrc1Desc.GetHeight(),
                              (uint8_t*)OAL_MemoryReturnAddress(src1_pad_oal, ACCESS_CH_WB) + PAD + PAD*(mSrc1Desc.GetWidth()+2*PAD),
                              (uint8_t*)OAL_MemoryReturnAddress(src1_pad_oal, ACCESS_PHY) + PAD + PAD*(mSrc1Desc.GetWidth()+2*PAD),
                              icp::DATATYPE_08U,
                              1, 1);
      Src1PadDesc.SetSpan(mSrc1Desc.GetWidth()+2*PAD);

      uint8_t *p = (uint8_t*)Src0PadDesc.GetDataPtr();
      uint8_t *s = (uint8_t*)mSrc0Desc.GetDataPtr();
      for (int y = 0; y < mSrc0Desc.GetHeight(); y++)
      {
         memcpy(p, s, mSrc0Desc.GetWidth());
         p += Src0PadDesc.GetSpan();
         s += mSrc0Desc.GetSpan();
      }
      Src0PadDesc.E0Replicate(PAD, PAD, PAD, PAD);

      p = (uint8_t*)Src1PadDesc.GetDataPtr();
      s = (uint8_t*)mSrc1Desc.GetDataPtr();
      for (int y = 0; y < mSrc1Desc.GetHeight(); y++)
      {
         memcpy(p, s, mSrc1Desc.GetWidth());
         p += Src1PadDesc.GetSpan();
         s += mSrc1Desc.GetSpan();
      }
      Src1PadDesc.E0Replicate(PAD, PAD, PAD, PAD);

#if DEBUG_PRINT_OUT
      {
         char filename[256];
         sprintf(filename, "%ssrc0_img_padded.csv", DD_LKTRACKER_PATH_DST);
      	FILE* tmpFile = fopen(filename, "wb");
      	fprintf(tmpFile, ", ");
      	for (int col = 0; col < Src0PadDesc.GetWidth(); col++)
      	{
      	  fprintf(tmpFile, "%d, ", col);
      	}
      	fprintf(tmpFile, "\n");
	   	for (int row = 0; row < Src0PadDesc.GetHeight(); row++)
      	{
      	   fprintf(tmpFile, "%d, ", row);
      	   for (int col = 0; col < Src0PadDesc.GetWidth(); col++)
      	   {
      	      fprintf(tmpFile, "%d, ", *((uint8_t*)Src0PadDesc.GetDataPtr()+row*Src0PadDesc.GetSpan()+col));
      	   }
      	   fprintf(tmpFile, "\n");
      	}
      	fclose(tmpFile);

         sprintf(filename, "%ssrc1_img_padded.csv", DD_LKTRACKER_PATH_DST);
      	tmpFile = fopen(filename, "wb");
      	fprintf(tmpFile, ", ");
      	for (int col = 0; col < Src1PadDesc.GetWidth(); col++)
      	{
      	  fprintf(tmpFile, "%d, ", col);
      	}
      	fprintf(tmpFile, "\n");
	   	for (int row = 0; row < Src1PadDesc.GetHeight(); row++)
      	{
      	   fprintf(tmpFile, "%d, ", row);
      	   for (int col = 0; col < Src1PadDesc.GetWidth(); col++)
      	   {
      	      fprintf(tmpFile, "%d, ", *((uint8_t*)Src1PadDesc.GetDataPtr()+row*Src1PadDesc.GetSpan()+col));
      	   }
      	   fprintf(tmpFile, "\n");
      	}
      	fclose(tmpFile);
      }
#endif

      ref_lktracker_calc_offset(Src0PadDesc, mCoor0Desc, mSrc0Offs, mFracXY0Desc, mBoxSize>>1, 1); // pad 1 for 3x3 scharr gradient
      ref_lktracker_calc_offset(Src1PadDesc, mCoor1Desc, mSrc1Offs, mBoxSize>>1, 0); 

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
         char filename[256];
         sprintf(filename, "%stemplate_dump.csv", DD_LKTRACKER_PATH_DST);
      	FILE* tmpFile = fopen(filename, "wb");
#endif

         for (int pt_idx = 0; pt_idx < mCoor0Desc.GetCount(); pt_idx++)
         {
#if DEBUG_PRINT_OUT
            {
               int32_t* coor_xy_0 = (int32_t*)&(mCoor0Desc.GetFeature(pt_idx));
               int32_t* coor_xy_1 = (int32_t*)&(mCoor1Desc.GetFeature(pt_idx));
               fprintf(tmpFile, "coor_0 %d %d coor_1 %d %d\n", 
                     coor_xy_0[0]/(1<<LKTRACKER_FLOATING_COOR_Q), 
                     coor_xy_0[1]/(1<<LKTRACKER_FLOATING_COOR_Q), 
                     coor_xy_1[0]/(1<<LKTRACKER_FLOATING_COOR_Q), 
                     coor_xy_1[1]/(1<<LKTRACKER_FLOATING_COOR_Q));
            }
#endif
            ref_lktracker_bilinear_08u(
               (uint8_t*)templ_interpolated, mBoxSize+2,  
               (uint8_t*)Src0PadDesc.GetDataPtr()+(*((int32_t*)mSrc0Offs.GetDataPtr()+pt_idx)), Src0PadDesc.GetSpan(), 
               (uint8_t*)mFracXY0Desc.GetDataPtr()+pt_idx*mFracXY0Desc.GetElementDimX(),
               mBoxSize+2, mBoxSize+2,
               LKTRACKER_FLOATING_COOR_Q);

            ref_flt_centraldx_3x3(
               (int16_t*)gradient_dx, mBoxSize,
               (uint8_t*)templ_interpolated+(mBoxSize+2)*1+1, (mBoxSize+2),
               mBoxSize, mBoxSize,
               *((int8_t*)mK0Desc.GetDataPtr()), 
               *((int8_t*)mK1Desc.GetDataPtr()),
               *((int8_t*)mK2Desc.GetDataPtr()));

            ref_flt_centraldy_3x3(
               (int16_t*)gradient_dy, mBoxSize,
               (uint8_t*)templ_interpolated+(mBoxSize+2)*1+1, (mBoxSize+2),
               mBoxSize, mBoxSize,
               *((int8_t*)mK0Desc.GetDataPtr()), 
               *((int8_t*)mK1Desc.GetDataPtr()),
               *((int8_t*)mK2Desc.GetDataPtr()));

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

            const int32_t cSrc1Offs_at_pt_idx = (*((int32_t*)mSrc1Offs.GetDataPtr()+pt_idx));
            int32_t* coor_xy_o = (int32_t*)&(mCoor1Desc_O.GetFeature(pt_idx).position);
            uint8_t* srcpatch = (uint8_t*)Src1PadDesc.GetDataPtr()+cSrc1Offs_at_pt_idx;
            uint8_t* srctempl = (uint8_t*)templ_interpolated+1*9+1;
            int32_t* coor_xy_i = (int32_t*)&(mCoor1Desc.GetFeature(pt_idx).position);

            if((uintptr_t)srcpatch < (uintptr_t)OAL_MemoryReturnAddress(src1_pad_oal, ACCESS_CH_WB) ||
               (uintptr_t)srcpatch >= (uintptr_t)OAL_MemoryReturnAddress(src1_pad_oal, ACCESS_CH_WB) + cSrcXPadOal_SizeInBytes )
            {
               printf("BAD!\n");
            }

            ref_lktracker_core(
               coor_xy_o,
               srcpatch, Src1PadDesc.GetSpan(),
               srctempl, mBoxSize+2,
               (int16_t*)gradient_dx, mBoxSize,
               (int16_t*)gradient_dy, mBoxSize,
               coor_xy_i,
               mBoxSize, mBoxSize,
               mBoxSize+2*LKTRACKER_PATCH_BORDER, mBoxSize+2*LKTRACKER_PATCH_BORDER,
               mNumIter,
               mSrc1Desc.GetWidth(), mSrc1Desc.GetHeight(),
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
   else
   {
      if (src0_pad_oal != 0) OAL_MemoryFree(src0_pad_oal);
      if (src1_pad_oal != 0) OAL_MemoryFree(src1_pad_oal);
      reval = 1;
      return reval;
   }

   if (src0_pad_oal != 0) OAL_MemoryFree(src0_pad_oal);
   if (src1_pad_oal != 0) OAL_MemoryFree(src1_pad_oal);
   return reval;
}

