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

#include "apu_test_util.h"
#include <stdio.h>
#include <stdlib.h>

#define APU_TIL_COMPARE(TAG, VECT, SCAL)    \
int \
apu_til_compare_##TAG ( \
   const VECT* dst, int dstr, \
   const SCAL* ref, int sstr, \
   int tw, int /*th*/, \
   int bw, int bh, \
   int errmax) \
{ \
   int rval  = 0; \
   int etot  = 0; \
   int ethr  = 0; \
    \
   for (int by=0, ty=0; by<bh; ++by, ++ty) \
   { \
     for (int bx=0; bx<bw; ++bx) \
     { \
       const VECT d = dst[by*dstr + bx]; \
       for (int c=0, tx=bx; (c<32) && (tx<tw); ++c, tx+=bw) \
       { \
         SCAL ddst = vget(d, c); \
         SCAL dref = ref[ty*sstr + tx]; \
          \
         if (ddst != dref) \
         { \
            ++etot; \
            int ddif = ddst - dref; if (ddif < 0) ddif = -ddif; \
            if (ddif > errmax) \
            { \
              ++ethr; \
              printf(" (%04X != %04X) (REF[%3i][%3i] != CMEM[%2i][%2i][%2i]) (%5i != %5i)\n",  \
                     dref, ddst,  \
                     tx, ty, \
                     c, bx, by, \
                     dref, ddst); \
            } \
         } \
       } \
     } \
   } \
    \
   if (etot > 0) \
   { \
     printf(" *** ERROR: (%6i) / TOTAL(%6i) \n", ethr, etot); \
   } \
    \
   return ethr; \
}

#define APU_PAD_IMAGE(TAG, TYPE) \
void \
apu_pad_image_##TAG( \
TYPE* dst, const TYPE* src, int sw, int sh, int pw, int ph) \
{ \
    const TYPE* s = src; \
    TYPE* d = dst; \
    if (ph < 0) ph = 0; \
    if (pw < 0) pw = 0; \
     \
    int spanw = 2*pw+sw; \
     \
    if (pw > 0) { \
        d += ph*spanw; \
        for (int i = 0; i < sh; ++i) { \
            for (int j = 0; j < pw; ++j) \
                d[j] = s[0]; \
             \
            for (int j = 0; j < sw; ++j) \
                d[j+pw] = s[j]; \
             \
            for (int j = 0; j < pw; ++j) \
                d[pw + sw + j] = s[sw-1]; \
                 \
            d += spanw; \
            s += sw; \
        } \
    } else { \
        d += ph*sw; \
        for (int i = 0; i < sh; ++i, d+=sw, s+=sw) \
            for (int j = 0; j < sw; ++j) \
                d[j] = s[j]; \
    } \
     \
    if (ph > 0) { \
        d = dst; \
        s = dst + ph*spanw; \
        for (int i = 0; i < ph; ++i) { \
            for (int j = 0; j < spanw; ++j) { \
                d[j] = s[j]; \
                 \
                d[(sh+ph)*spanw+j] = s[(sh-1)*spanw+j]; \
            } \
            d += spanw; \
        } \
    } \
} \
 \
void \
apu_pad_image_##TAG( \
TYPE* dst, const TYPE* src, int sstr, int sw, int sh, int pw, int ph) \
{ \
    const TYPE* s = src; \
    TYPE* d = dst; \
    if (ph < 0) ph = 0; \
    if (pw < 0) pw = 0; \
     \
    int spanw = 2*pw+sw; \
     \
    if (pw > 0) { \
        d += ph*spanw; \
        for (int i = 0; i < sh; ++i) { \
            for (int j = 0; j < pw; ++j) \
                d[j] = s[0]; \
             \
            for (int j = 0; j < sw; ++j) \
                d[j+pw] = s[j]; \
             \
            for (int j = 0; j < pw; ++j) \
                d[pw + sw + j] = s[sw-1]; \
                 \
            d += spanw; \
            s += sstr; \
        } \
    } else { \
        d += ph*sw; \
        for (int i = 0; i < sh; ++i, d+=sw, s+=sstr) \
            for (int j = 0; j < sw; ++j) \
                d[j] = s[j]; \
    } \
     \
    if (ph > 0) { \
        d = dst; \
        s = dst + ph*spanw; \
        for (int i = 0; i < ph; ++i) { \
            for (int j = 0; j < spanw; ++j) { \
                d[j] = s[j]; \
                 \
                d[(sh+ph)*spanw+j] = s[(sh-1)*spanw+j]; \
            } \
            d += spanw; \
        } \
    } \
}

/*!
 Vertical padding with pixel replication (in place).
 
   \param blk  - [Input/Output]  Block pointer
   \param sstr - [Input]         Block stride
   \param bw   - [Input]  Block width
   \param bh   - [Input]  Block height 
   \param bpx  - [Input]  Border padding size (x-dir)
   \param bpy  - [Input]  Border padding size (y-dir)
 */
#define APU_TIL_PADVER(TAG, TYPE) \
void \
apu_til_padver_##TAG(TYPE* blk, int sstr, int bw, int bh, int bpx, int bpy) \
{ \
   TYPE * s0 = blk; \
   TYPE * sh = blk + (bh-1)*sstr; \
 \
   for (int by=1; by<=bpy; ++by) \
   { \
      TYPE * st = s0 - by*sstr; \
      TYPE * sb = sh + by*sstr; \
       \
      for (int bx=-bpx; bx<(bw+bpx); ++bx) \
      { \
         st[bx] = s0[bx]; \
         sb[bx] = sh[bx]; \
      } \
   } \
}

#define APU_MEM_CLEAR(TAG, TYPE) \
void \
apu_mem_clear_##TAG(TYPE*  dst, int dstr, int bw, int bh) \
{ \
   int ncu = apuGetNumberOfCUs(); \
   for (int by=0; by<bh; ++by) \
   { \
      for (int bx=0; bx<bw; ++bx) \
      { \
         TYPE d = 0; \
         for (int c=0; c<ncu; ++c) \
         { \
            d = vput(d, 0, c); \
         } \
         dst[bx] = d; \
      } \
      dst += dstr; \
   } \
}

/*!
 Data Comparison (CMEM & DMEM) using ranges for comparing approximation
 
   \param dst     - [Input]  Result block pointer     (CMEM)
   \param dstr    - [Input]  Result block stride
   \param ref     - [Input]  Reference tile pointer  (DMEM)
   \param sstr    - [Input]  Reference tile stride
   \param tw      - [Input]  Tile width
   \param th      - [Input]  Tile height 
   \param bw      - [Input]  Block width
   \param bh      - [Input]  Block height 
   \param errmax  - [Input]  Maximum number of errors (display)
   \param window  - [Input]  Range window value
 */
int
apu_til_compare_range(
   const vec08u*  dst, int dstr, 
   const uint8_t* ref, int sstr, 
   int tw, int /*th*/, 
   int bw, int bh, 
   int errmax, int window)
{
   int rval  = 0;
   int etot  = 0;
   int ethr  = 0;
   
   vec16s d;
   int16_t ddst, dref;
   
   for (int by=0, ty=0; by<bh; ++by, ++ty)
   {
      for (int bx=0; bx<bw; ++bx)
      {
         d = (vec16s)dst[by*dstr + bx];
         for (int c=0, tx=bx; (c<32) && (tx<tw); ++c, tx+=bw)
         {
            ddst = vget(d, c);
            dref = ref[ty*sstr + tx];
            
             if (ddst < dref - window || ddst > dref + window)
            {
               ++etot;
               int16_t ddif = ddst - dref; if (ddif < 0) ddif = -ddif;
               if (ddif > errmax)
               {
                  ++ethr;
                  printf(" (%02X != %02X) (REF[%3i][%3i] != CMEM[%2i][%2i][%2i]) (%3i != %3i)\n", 
                           dref, ddst, 
                           tx, ty,
                           c, bx, by,
                           dref, ddst);
               }
            }
         }
      }
   }
   
   if (etot > 0)
   {
      printf(" *** ERROR: (%6i) / TOTAL(%6i) \n", ethr, etot);
   }
   
   return ethr;
}

/*!
 Data Comparison (CMEM & DMEM) using ranges for comparing approximation
 
   \param dst     - [Input]  Result block pointer     (CMEM)
   \param dstr    - [Input]  Result block stride
   \param ref     - [Input]  Reference tile pointer  (DMEM)
   \param sstr    - [Input]  Reference tile stride
   \param tw      - [Input]  Tile width
   \param th      - [Input]  Tile height 
   \param bw      - [Input]  Block width
   \param bh      - [Input]  Block height 
   \param errmax  - [Input]  Maximum number of errors (display)
   \param window  - [Input]  Range window value
 */
int
apu_til_compare_range(
   const vec16u*  dst, int dstr, 
   const uint16_t* ref, int sstr, 
   int tw, int /*th*/, 
   int bw, int bh, 
   int errmax, int window)
{
   int rval  = 0;
   int etot  = 0;
   int ethr  = 0;
   
   vec16u d;
   uint16_t ddst, dref;
   
   for (int by=0, ty=0; by<bh; ++by, ++ty)
   {
      for (int bx=0; bx<bw; ++bx)
      {
         d = dst[by*dstr + bx];
         for (int c=0, tx=bx; (c<32) && (tx<tw); ++c, tx+=bw)
         {
            ddst = vget(d, c);
            dref = ref[ty*sstr + tx];
            
             if (ddst < dref - window || ddst > dref + window)
            {
               ++etot;
               uint16_t ddif = ddst - dref; if (ddif < 0) ddif = -ddif;
               if (ddif > errmax)
               {
                  ++ethr;
                  printf(" (%02X != %02X) (REF[%3i][%3i] != CMEM[%2i][%2i][%2i]) (%3i != %3i)\n", 
                           dref, ddst, 
                           tx, ty,
                           c, bx, by,
                           dref, ddst);
               }
            }
         }
      }
   }
   
   if (etot > 0)
   {
      printf(" *** ERROR: (%6i) / TOTAL(%6i) \n", ethr, etot);
   }
   
   return ethr;
}

APU_TIL_COMPARE(32, vec32s, int32s)
APU_TIL_COMPARE(32, vec32u, int32u)
APU_TIL_COMPARE(16, vec16u, int16u)
APU_TIL_COMPARE(16, vec16s, int16s)
APU_TIL_COMPARE(08, vec08u, int08u)
APU_TIL_COMPARE(08, vec08s, int08s)

APU_TIL_PADVER(16, vec16s)
APU_TIL_PADVER(16, vec16u)
APU_TIL_PADVER(08, vec08s)
APU_TIL_PADVER(08, vec08u)

APU_PAD_IMAGE(08, int08u)
APU_PAD_IMAGE(16, int16u)
APU_PAD_IMAGE(32, int32u)

APU_MEM_CLEAR(32, vec32s)
APU_MEM_CLEAR(32, vec32u)
APU_MEM_CLEAR(16, vec16s)
APU_MEM_CLEAR(16, vec16u)
APU_MEM_CLEAR(08, vec08s)
APU_MEM_CLEAR(08, vec08u)

void
apu_indirect_input_08(
   vec08u* dst, int dstr,
   const int08u* src, int sstr, const int32s *offset,
   int cw, int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; by++)
   {
      for (int bx=0; bx<bw; bx++)
      {
         vec08u d = 0;
         for (int c=0; (c<ncu) && (c<cw); c++)
         {
            d = vput(d, src[offset[c]+bx+by*sstr], c);
         }
         dst[bx] = d;
      }
      dst += dstr;
   }
}

void
apu_indirect_input_08(
   vec08u* dst, int dstr,
   const int08u* src, int sstr, const int32s *offset,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; by++)
   {
      for (int bx=0; bx<bw; bx++)
      {
         vec08u d = 0;
         for (int c=0; (c<ncu); c++)
         {
            d = vput(d, src[offset[c]+bx+by*sstr], c);
         }
         dst[bx] = d;
      }
      dst += dstr;
   }
}

void
apu_indirect_input_16(
   vec16u* dst, int dstr,
   const int16u* src, int sstr, const int32s *offset,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; by++)
   {
      for (int bx=0; bx<bw; bx++)
      {
         vec16u d = 0;
         for (int c=0; c<ncu; c++)
         {
            d = vput(d, src[(offset[c]>>1)+bx+by*sstr], c);
         }
         dst[bx] = d;
      }
      dst += dstr;
   }
}

void
apu_indirect_input_16(
   vec16s* dst, int dstr,
   const int16s* src, int sstr, const int32s *offset,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; by++)
   {
      for (int bx=0; bx<bw; bx++)
      {
         vec16s d = 0;
         for (int c=0; c<ncu; c++)
         {
            d = vput(d, src[(offset[c]>>1)+bx+by*sstr], c);
         }
         dst[bx] = d;
      }
      dst += dstr;
   }
}

void
apu_indirect_input_32(
   vec32u* dst, int dstr,
   const int32u* src, int sstr, const int32s *offset,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; by++)
   {
      for (int bx=0; bx<bw; bx++)
      {
         vec32u d = 0;
         for (int c=0; c<ncu; c++)
         {
            d = vput(d, src[(offset[c]>>2)+bx+by*sstr], c);
         }
         dst[bx] = d;
      }
      dst += dstr;
   }
}

// [To be deprecated]
#define APU_TIL_TRANSFER(TAG, VECT, SCAL)    \
void \
apu_til_transfer_##TAG( \
   VECT*  dst, int dstr, \
   const SCAL* src, int sstr, \
   int tw, \
   int bw, int bh, int flt_size_x, int flt_size_y) \
{ \
   int ncu = apuGetNumberOfCUs(); \
   int pad_size_x = ((flt_size_x)>>1)<<1; \
   for (int by=0; by<(bh+((flt_size_y>>1)<<1)); ++by) \
   { \
     for (int bx=0; bx<(bw+pad_size_x); ++bx) \
     { \
       VECT d = 0; \
       for (int c=0, tx=bx; (c<ncu) && (tx<(tw+pad_size_x)); ++c, tx+=bw) \
       { \
         d = vput(d, src[tx], c); \
       } \
       dst[bx] = d; \
     } \
     src += sstr+pad_size_x; \
     dst += dstr+pad_size_x; \
   } \
}


// To be deprecated
APU_TIL_TRANSFER(32, vec32s, int32s)
APU_TIL_TRANSFER(32, vec32u, int32u)
APU_TIL_TRANSFER(16, vec16u, int16u)
APU_TIL_TRANSFER(16, vec16s, int16s)
APU_TIL_TRANSFER(08, vec08u, int08u)
APU_TIL_TRANSFER(08, vec08s, int08s) 

// Assume we have enough data from the source to fill the destination vector
void apu_til_transfer_08(
   vec08u*  dst, int dstr,
   const int08u* src, int sstr,
   int tw, int /*th*/,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; ++by)  chess_loop_range(1, ) {
      vec08u* restrict pd = dst;
      for (int bx=0; bx<bw; bx+=2) chess_loop_range(1, ) {
         vec08u d1 = 0;
         vec08u d2 = 0;
         const int08u* restrict ps1 = src+bx;
         const int08u* restrict ps2 = src+bx+1;
         for (int c = 0, tx=bw; c<ncu && tx<=tw; ++c,tx+=bw)  chess_loop_range(1, ) {
            d1 = vput(d1, *ps1, c);
            ps1 += bw;
            d2 = vput(d2, *ps2, c);
            ps2 += bw;
         }
         *pd++ = d1;
         *pd++ = d2;
      }
      src += sstr;
      dst += dstr;
   }
}

void apu_til_transfer_16(
   vec16u*  dst, int dstr,
   const int16u* src, int sstr,
   int tw, int /*th*/,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; ++by)  chess_loop_range(1, ) {
      vec16u* restrict pd = dst;
      for (int bx=0; bx<bw; bx+=2) chess_loop_range(1, ) {
         vec16u d1 = 0;
         vec16u d2 = 0;
         const int16u* restrict ps1 = src+bx;
         const int16u* restrict ps2 = src+bx+1;
         for (int c = 0, tx=bw; c<ncu && tx<=tw; ++c,tx+=bw)  chess_loop_range(1, ) {
            d1 = vput(d1, *ps1, c);
            ps1 += bw;
            d2 = vput(d2, *ps2, c);
            ps2 += bw;
         }
         *pd++ = d1;
         *pd++ = d2;
      }
      src += sstr;
      dst += dstr;
   }
}

void apu_til_transfer_32(
   vec32u*  dst, int dstr,
   const int32u* src, int sstr,
   int tw, int /*th*/,
   int bw, int bh)
{
   int ncu = apuGetNumberOfCUs();
   for (int by=0; by<bh; ++by)  chess_loop_range(1, ) {
      vec32u* restrict pd = dst;
      for (int bx=0; bx<bw; bx+=2) chess_loop_range(1, ) {
         vec32u d1 = 0;
         vec32u d2 = 0;
         const int32u* restrict ps1 = src+bx;
         const int32u* restrict ps2 = src+bx+1;
         for (int c = 0, tx=bw; c<ncu && tx<=tw; ++c,tx+=bw)  chess_loop_range(1, ) {
            d1 = vput(d1, *ps1, c);
            ps1 += bw;
            d2 = vput(d2, *ps2, c);
            ps2 += bw;
         }
         *pd++ = d1;
         *pd++ = d2;
      }
      src += sstr;
      dst += dstr;
   }
}

void apu_til_transfer_08(
            int08u*  dst, int dstr,
      const vec08u* src, int sstr,
            int tw, int /*th*/,
            int bw, int bh
)
{
   int ncu = apuGetNumberOfCUs();
   for (int h = 0; h < bh; ++h) chess_loop_range(1, ) {
      int08u* restrict pd = dst;
      for (int j = 0; j < ncu; ++j) chess_loop_range(1, ) {
         const vec08u* restrict ps = src;
         for (int k = 0,tx=j*bw; k<bw && tx<=tw; k+=2,tx+=2) chess_loop_range(1, ) {
            *pd++ = vget(*ps++, j);
            *pd++ = vget(*ps++, j);
         }
      }
      dst += dstr;
      src += sstr;
   }
}

void apu_til_transfer_16(
            int16u*  dst, int dstr,
      const vec16u* src, int sstr,
            int tw, int /*th*/,
            int bw, int bh
)
{
   int ncu = apuGetNumberOfCUs();
   for (int h = 0; h < bh; ++h) chess_loop_range(1, ) {
      int16u* restrict pd = dst;
      for (int j = 0; j < ncu; ++j) chess_loop_range(1, ) {
         const vec16u* restrict ps = src;
         for (int k = 0,tx=j*bw; k<bw && tx<=tw; k+=2,tx+=2) chess_loop_range(1, ) {
            *pd++ = vget(*ps++, j);
            *pd++ = vget(*ps++, j);
         }
      }
      dst += dstr;
      src += sstr;
   }
}

void apu_til_transfer_32(
            int32u*  dst, int dstr,
      const vec32u* src, int sstr,
            int tw, int /*th*/,
            int bw, int bh
)
{
   int ncu = apuGetNumberOfCUs();
   for (int h = 0; h < bh; ++h) chess_loop_range(1, ) {
      int32u* restrict pd = dst;
      for (int j = 0; j < ncu; ++j) chess_loop_range(1, ) {
         const vec32u* restrict ps = src;
         for (int k = 0,tx=j*bw; k<bw && tx<=tw; k+=2,tx+=2) chess_loop_range(1, ) {
            *pd++ = vget(*ps++, j);
            *pd++ = vget(*ps++, j);
         }
      }
      dst += dstr;
      src += sstr;
   }
}

void saveImage(char* fn, void *p_src, uint32_t iw, uint32_t ih, uint32_t is, uint8_t bpp)
{
   FILE *fp = fopen(fn, "wb");
   
   if (fp != NULL)
   {
      uint8_t* tptr = (uint8_t*)p_src;
      for (uint32_t h=0; h<ih; h++)
      {
         fwrite (tptr, bpp, iw, fp);
         tptr += is*bpp;
      }
      fclose(fp);
   }
}

void readImage(char* fn, uint8_t *p_u8Src, uint32_t iw, uint32_t ih, uint32_t is, uint8_t bpp)
{
   FILE* fp = fopen(fn, "rb");
   if (fp != NULL)
   {
      for (uint32_t i=0; i<ih; i++)
      {
         fread((p_u8Src+i*is*bpp), 1, iw*bpp, fp);
      }
      fclose(fp);
   }
}

// What is a template?
// There is very little difference between the vec08u, vec16u, vec32u types
// having the same code for all gives pretty optimal solutions already.

#define APU_TIL_PADHOR_E0X(tag, VECT)                                                        \
void apu_til_padhor_##tag ##_e0x(VECT* blk, int sstr, int bw, int bh, int e0x, int leftPadd, int rightPadd)\
{                                                                                            \
   VECT* restrict sl;                                                                        \
   VECT* restrict sl2;                                                                       \
   VECT* restrict sr;                                                                        \
   VECT* restrict sr2;                                                                       \
   VECT vrp, vlp;                                                                            \
   VECT vr, vl;                                                                              \
                                                                                             \
   int bx;                                                                                   \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                                       \
      sl = blk-1;                                                                            \
      sl2 = blk;                                                                             \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                       \
      sr2 = blk + e0x*(bw-1) + e0x;                                                          \
                                                                                             \
      for (bx=0; bx<leftPadd*e0x;) {                                                         \
         VECT* restrict tmp = blk + e0x - 1;                                                 \
         vrp = *tmp;                                                                         \
         vr = *sr;                                                                           \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                         \
            vrp = vmrl(vrp);                                                                 \
            vr = vmrr(vr, vrp);                                                              \
            *sl-- = vr;                                                                      \
            vrp = *--tmp;                                                                    \
            vr = *--sr;                                                                      \
         }                                                                                   \
      }                                                                                      \
      for (int br=0; br<rightPadd*e0x;) {                                                    \
         VECT* restrict tmp2 = blk + e0x*(bw-1);                                             \
         vlp = *tmp2;                                                                        \
         vl = *sl2;                                                                          \
         for (int e = 0; e < e0x; ++e, ++br) chess_loop_range(1, ) {                         \
            vlp = vmrr(vlp);                                                                 \
            vl = vmrl(vl, vlp);                                                              \
            *sr2++ = vl;                                                                     \
            vlp = *++tmp2;                                                                   \
            vl = *++sl2;                                                                     \
         }                                                                                   \
      }                                                                                      \
      blk += sstr;                                                                           \
   }                                                                                         \
}

#define APU_TIL_PADHOR_SYM_E0X(tag, VECT)                                                    \
void apu_til_padhor_##tag ##_e0x(VECT* blk, int sstr, int bw, int bh, int e0x, int padx)           \
{                                                                                            \
   VECT* restrict sl;                                                                        \
   VECT* restrict sl2;                                                                       \
   VECT* restrict sr;                                                                        \
   VECT* restrict sr2;                                                                       \
   VECT vrp, vlp;                                                                            \
   VECT vr, vl;                                                                              \
                                                                                             \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                                       \
      sl = blk-1;                                                                            \
      sl2 = blk;                                                                             \
      /* Compiler *should* handle common sub calculations for us */                          \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                       \
      sr2 = blk + e0x*(bw-1) + e0x;                                                          \
      VECT* restrict tmp = blk + e0x - 1;                                                    \
      VECT* restrict tmp2 = blk + e0x*(bw-1);                                                \
      blk += sstr;                                                                           \
                                                                                             \
      for (int bx=0; bx<padx*e0x;) chess_loop_range(1, ) {                                   \
         /* Manually unroll loads to help the compiler optimize the loop */                  \
         vrp = *tmp;                                                                         \
         vr = *sr;                                                                           \
         vlp = *tmp2;                                                                        \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                         \
            /* The compiler *should* do the reordering for us to make it optimal */          \
            /* Rotates should happen in same instruction as loads */                         \
                                                                                             \
            /* Left Pad */                                                                   \
            vrp = vmrl(vrp);                                                                 \
            vr = vmrr(vr, vrp);                                                              \
            *sl-- = vr;                                                                      \
            vrp = *--tmp;                                                                    \
            vr = *--sr;                                                                      \
            vl = *sl2++;                                                                     \
                                                                                             \
            /* Right Pad */                                                                  \
            vlp = vmrr(vlp);                                                                 \
            vl = vmrl(vl, vlp);                                                              \
            *sr2++ = vl;                                                                     \
            vlp = *++tmp2;                                                                   \
         }                                                                                   \
      }                                                                                      \
   }                                                                                         \
}

#define APU_TIL_PADHOR_H2_E0X(tag, VECT)                                                     \
void apu_til_padhor_##tag ##_h2_e0x(VECT* blk, int sstr, int bw, int bh, int e0x, int leftPadd, int rightPadd)\
{                                                                                            \
   /* Pretty much just duplicating code of apu_til_padhor_16 */                              \
   /* Looks ugly, but gives better CC                        */                              \
   VECT* restrict sl;                                                                        \
   VECT* restrict sl_2;                                                                      \
   VECT* restrict sl2;                                                                       \
   VECT* restrict sl2_2;                                                                     \
   VECT* restrict sr;                                                                        \
   VECT* restrict sr_2;                                                                      \
   VECT* restrict sr2;                                                                       \
   VECT* restrict sr2_2;                                                                     \
   VECT vrp, vlp;                                                                            \
   VECT vrp_2, vlp_2;                                                                        \
   VECT vr, vl;                                                                              \
   VECT vr_2, vl_2;                                                                          \
                                                                                             \
   for (int by=0; by<bh; by+=2) chess_loop_range(1, ) {                                      \
      /* Compiler *should* handle common sub calculations for us */                          \
      /* Row 1 */                                                                            \
      sl = blk-1;                                                                            \
      sl2 = blk;                                                                             \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                       \
      sr2 = blk + e0x*(bw-1) + e0x;                                                          \
      VECT* restrict tmp = blk + e0x - 1;                                                    \
      VECT* restrict tmp2 = blk + e0x*(bw-1);                                                \
      blk += sstr;                                                                           \
                                                                                             \
      /* Row 2 */                                                                            \
      sl_2 = blk-1;                                                                          \
      sl2_2 = blk;                                                                           \
      sr_2 = blk + e0x*(bw-1) + e0x - 1;                                                     \
      sr2_2 = blk + e0x*(bw-1) + e0x;                                                        \
      VECT* restrict tmp_2 = blk + e0x - 1;                                                  \
      VECT* restrict tmp2_2 = blk + e0x*(bw-1);                                              \
      blk += sstr;                                                                           \
                                                                                             \
      for (int bx=0; bx<leftPadd*e0x;) chess_loop_range(1, ) {                               \
         /* Manually unroll loads to help the compiler optimize the loop */                  \
         vrp = *tmp;                                                                         \
         vrp_2 = *tmp_2;                                                                     \
         vr = *sr;                                                                           \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                         \
            /* The compiler *should* do the reordering for us to make it optimal */          \
            /* Rotates should happen in same instruction as loads */                         \
            /* Row 1 Left Pad */                                                             \
            vrp = vmrl(vrp);                                                                 \
            vr = vmrr(vr, vrp);                                                              \
            *sl-- = vr;                                                                      \
            vrp = *--tmp;                                                                    \
            vr = *--sr;                                                                      \
                                                                                             \
            /* Row 2 Left Pad */                                                             \
            vr_2 = *sr_2--;                                                                  \
                                                                                             \
            vrp_2 = vmrl(vrp_2);                                                             \
            vr_2 = vmrr(vr_2, vrp_2);                                                        \
            *sl_2-- = vr_2;                                                                  \
            vrp_2 = *--tmp_2;                                                                \
         }                                                                                   \
      }                                                                                      \
                                                                                             \
      for (int bx=0; bx<rightPadd*e0x;) {                                                    \
         /* Manually unroll loads to help the compiler optimize the loop */                  \
         vlp = *tmp2;                                                                        \
         vlp_2 = *tmp2_2;                                                                    \
         vl = *sl2;                                                                          \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                         \
            /* The compiler *should* do the reordering for us to make it optimal */          \
            /* Rotates should happen in same instruction as loads */                         \
            /* Row 1 Right Pad */                                                            \
            vlp = vmrr(vlp);                                                                 \
            vl = vmrl(vl, vlp);                                                              \
            *sr2++ = vl;                                                                     \
            vlp = *++tmp2;                                                                   \
            vl = *++sl2;                                                                     \
            vl_2 = *sl2_2++;                                                                 \
                                                                                             \
            /* Row 2 Right Pad */                                                            \
            vlp_2 = vmrr(vlp_2);                                                             \
            vl_2 = vmrl(vl_2, vlp_2);                                                        \
            *sr2_2++ = vl_2;                                                                 \
            vlp_2 = *++tmp2_2;                                                               \
         }                                                                                   \
      }                                                                                      \
   }                                                                                         \
}

#define APU_TIL_PADHOR_SYM_H2_E0X(tag, VECT)                                                 \
void apu_til_padhor_##tag ##_h2_e0x(VECT* blk, int sstr, int bw, int bh, int e0x, int padx)  \
{                                                                                            \
   /* Pretty much just duplicating code of apu_til_padhor_16 */                              \
   /* Looks ugly, but gives better CC                        */                              \
   VECT* restrict sl;                                                                        \
   VECT* restrict sl_2;                                                                      \
   VECT* restrict sl2;                                                                       \
   VECT* restrict sl2_2;                                                                     \
   VECT* restrict sr;                                                                        \
   VECT* restrict sr_2;                                                                      \
   VECT* restrict sr2;                                                                       \
   VECT* restrict sr2_2;                                                                     \
   VECT vrp, vlp;                                                                            \
   VECT vrp_2, vlp_2;                                                                        \
   VECT vr, vl;                                                                              \
   VECT vr_2, vl_2;                                                                          \
                                                                                             \
   for (int by=0; by<bh; by+=2) chess_loop_range(1, ) {                                      \
      /* Compiler *should* handle common sub calculations for us */                          \
      /* Row 1 */                                                                            \
      sl = blk-1;                                                                            \
      sl2 = blk;                                                                             \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                       \
      sr2 = blk + e0x*(bw-1) + e0x;                                                          \
      VECT* restrict tmp = blk + e0x - 1;                                                    \
      VECT* restrict tmp2 = blk + e0x*(bw-1);                                                \
      blk += sstr;                                                                           \
                                                                                             \
      /* Row 2 */                                                                            \
      sl_2 = blk-1;                                                                          \
      sl2_2 = blk;                                                                           \
      sr_2 = blk + e0x*(bw-1) + e0x - 1;                                                     \
      sr2_2 = blk + e0x*(bw-1) + e0x;                                                        \
      VECT* restrict tmp_2 = blk + e0x - 1;                                                  \
      VECT* restrict tmp2_2 = blk + e0x*(bw-1);                                              \
      blk += sstr;                                                                           \
                                                                                             \
      for (int bx=0; bx<padx*e0x;) chess_loop_range(1, ) {                                   \
         /* Manually unroll loads to help the compiler optimize the loop */                  \
         vrp = *tmp;                                                                         \
         vr = *sr;                                                                           \
         vlp = *tmp2;                                                                        \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                         \
            /* The compiler *should* do the reordering for us to make it optimal */          \
            /* Rotates should happen in same instruction as loads */                         \
                                                                                             \
            /* Row 1 Left Pad */                                                             \
            vrp = vmrl(vrp);                                                                 \
            vr = vmrr(vr, vrp);                                                              \
            *sl-- = vr;                                                                      \
            vrp = *--tmp;                                                                    \
            vl = *sl2++;                                                                     \
            vr = *--sr;                                                                      \
                                                                                             \
            /* Row 1 Right Pad */                                                            \
            vlp = vmrr(vlp);                                                                 \
            vl = vmrl(vl, vlp);                                                              \
            *sr2++ = vl;                                                                     \
            vlp = *++tmp2;                                                                   \
                                                                                             \
            /* Row 2 Left Pad */                                                             \
            vrp_2 = *tmp_2--;                                                                \
            vr_2 = *sr_2--;                                                                  \
            vlp_2 = *tmp2_2++;                                                               \
            vl_2 = *sl2_2++;                                                                 \
                                                                                             \
            vrp_2 = vmrl(vrp_2);                                                             \
            vr_2 = vmrr(vr_2, vrp_2);                                                        \
            *sl_2-- = vr_2;                                                                  \
                                                                                             \
            /* Row 2 Right Pad */                                                            \
            vlp_2 = vmrr(vlp_2);                                                             \
            vl_2 = vmrl(vl_2, vlp_2);                                                        \
            *sr2_2++ = vl_2;                                                                 \
         }                                                                                   \
      }                                                                                      \
   }                                                                                         \
}

#define APU_TIL_PADHOR(tag, VECT)                                                            \
void apu_til_padhor_##tag(VECT* blk, int sstr, int bw, int bh, int leftPadd, int rightPadd)  \
{                                                                                            \
   VECT vr, vl ,vr2, vl2;                                                                    \
   VECT *sl, *sr, *sl2, *sr2;                                                                \
   VECT vrp, vlp;                                                                            \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                                       \
      sl = blk;                                                                              \
      sr = blk + (bw-1);                                                                     \
                                                                                             \
      vrp = *sl--;                                                                           \
      vrp = vmrl(vrp);                                                                       \
      vlp = *sr;                                                                             \
      vlp = vmrr(vlp);                                                                       \
      vr, vl;                                                                                \
      for (int bx=0; bx<leftPadd; ++bx) {                                                    \
         vr = *sr--;                                                                         \
         vr = vmrr(vr, vrp);                                                                 \
         *sl-- = vr;                                                                         \
      }                                                                                      \
      sl += leftPadd;                                                                        \
      sr += leftPadd;                                                                        \
      for (int bx=0; bx<rightPadd; ++bx) {                                                   \
         vl = *++sl;                                                                         \
         vl = vmrl(vl, vlp);                                                                 \
         *++sr = vl;                                                                         \
      }                                                                                      \
      blk += sstr;                                                                           \
   }                                                                                         \
}

#define APU_TIL_PADHOR_H2(tag, VECT)                                                         \
void apu_til_padhor_##tag ##_h2(VECT* blk, int sstr, int bw, int bh, int leftPadd, int rightPadd)\
{                                                                                            \
   VECT vr, vl ,vr2, vl2;                                                                    \
   VECT *sl, *sr, *sl2, *sr2;                                                                \
   VECT vrp, vlp;                                                                            \
   VECT vrp2, vlp2;                                                                          \
   for (int by=0; by<bh; by+=2) chess_loop_range(1, ) {                                      \
      sl = blk;                                                                              \
      sr = blk + (bw-1);                                                                     \
      sl2 = sl + sstr;                                                                       \
      sr2 = sr + sstr;                                                                       \
                                                                                             \
      vrp = *sl--; vrp = vmrl(vrp);                                                          \
      vrp2 = *sl2--; vrp2 = vmrl(vrp2);                                                      \
                                                                                             \
      vlp = *sr; vlp = vmrr(vlp);                                                            \
      vlp2 = *sr2; vlp2 = vmrr(vlp2);                                                        \
                                                                                             \
      vr, vl, vr2, vl2;                                                                      \
      for (int bx=0; bx<leftPadd; ++bx) {                                                    \
         vr = *sr--;                                                                         \
         vr2 = *sr2--;                                                                       \
         vr = vmrr(vr, vrp);                                                                 \
         vr2 = vmrr(vr2, vrp2);                                                              \
         *sl-- = vr;                                                                         \
         *sl2-- = vr2;                                                                       \
      }                                                                                      \
                                                                                             \
      sl += leftPadd;                                                                        \
      sl2 += leftPadd;                                                                       \
      sr += leftPadd;                                                                        \
      sr2 += leftPadd;                                                                       \
      for (int bx=0; bx<rightPadd; ++bx) {                                                   \
         vl = *++sl;                                                                         \
         vl2 = *++sl2;                                                                       \
         vl = vmrl(vl, vlp);                                                                 \
         vl2 = vmrl(vl2, vlp2);                                                              \
         *++sr = vl;                                                                         \
         *++sr2 = vl2;                                                                       \
      }                                                                                      \
      blk += 2*sstr;                                                                         \
   }                                                                                         \
}

#define APU_TIL_PADHOR_SYM(tag, VECT)                                   \
void apu_til_padhor_##tag(VECT* blk, int sstr, int bw, int bh, int pad) \
{                                                                       \
   VECT vr, vl;                                                         \
   VECT *sl, *sr, *sl2, *sr2;                                           \
   VECT vrp, vlp;                                                       \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                  \
      sl = blk;                                                         \
      sr = blk + (bw-1);                                                \
      sl2 = sl;                                                         \
      sr2 = sr;                                                         \
                                                                        \
      vrp = *sl--; vrp = vmrl(vrp);                                     \
      vlp = *sr2++; vlp = vmrr(vlp);                                    \
                                                                        \
      for (int bx=0; bx<pad; ++bx) chess_loop_range(1, ) {              \
         vr = *sr--;                                                    \
         vl = *sl2++;                                                   \
         vr = vmrr(vr, vrp);                                            \
         vl = vmrl(vl, vlp);                                            \
         *sl-- = vr;                                                    \
         *sr2++ = vl;                                                   \
      }                                                                 \
      blk += sstr;                                                      \
   }                                                                    \
}

#define APU_TIL_PADHOR_SYM_H2(tag, VECT)                                \
void apu_til_padhor_ ##tag ##_h2(VECT* blk, int sstr, int bw, int bh, int pad)\
{                                                                       \
   VECT vr, vl ,vr2, vl2;                                               \
   VECT vr_2, vl_2 ,vr2_2, vl2_2;                                       \
   VECT *sl, *sr, *sl2, *sr2;                                           \
   VECT *sl_2, *sr_2, *sl2_2, *sr2_2;                                   \
   VECT vrp, vlp;                                                       \
   VECT vrp_2, vlp_2;                                                   \
   for (int by=0; by<bh; by+=2) chess_loop_range(1, ) {                 \
      sl = blk;                                                         \
      sl_2 = sl+sstr;                                                   \
                                                                        \
      sr = blk + (bw-1);                                                \
      sr_2 = sr+sstr;                                                   \
                                                                        \
      sl2 = sl;                                                         \
      sl2_2 = sl_2;                                                     \
                                                                        \
      sr2 = sr;                                                         \
      sr2_2 = sr_2;                                                     \
                                                                        \
      vrp = *sl--; vrp = vmrl(vrp);                                     \
      vrp_2 = *sl_2--; vrp_2 = vmrl(vrp_2);                             \
                                                                        \
      vlp = *sr2++; vlp = vmrr(vlp);                                    \
      vlp_2 = *sr2_2++; vlp_2 = vmrr(vlp_2);                            \
                                                                        \
      for (int bx=0; bx<pad; ++bx) chess_loop_range(1, ) {              \
         vr = *sr--;                                                    \
         vr_2 = *sr_2--;                                                \
                                                                        \
         vl = *sl2++;                                                   \
         vl_2 = *sl2_2++;                                               \
                                                                        \
         vr = vmrr(vr, vrp);                                            \
         vr_2 = vmrr(vr_2, vrp_2);                                      \
                                                                        \
         vl = vmrl(vl, vlp);                                            \
         vl_2 = vmrl(vl_2, vlp_2);                                      \
                                                                        \
         *sl-- = vr;                                                    \
         *sl_2-- = vr_2;                                                \
                                                                        \
         *sr2++ = vl;                                                   \
         *sr2_2++ = vl_2;                                               \
      }                                                                 \
      blk += 2*sstr;                                                    \
   }                                                                    \
}

APU_TIL_PADHOR(08, vec08u)
APU_TIL_PADHOR(16, vec16u)
APU_TIL_PADHOR(32, vec32u)

APU_TIL_PADHOR_H2(08, vec08u)
APU_TIL_PADHOR_H2(16, vec16u)
APU_TIL_PADHOR_H2(32, vec32u)

APU_TIL_PADHOR_SYM(08, vec08u)
APU_TIL_PADHOR_SYM(16, vec16u)
APU_TIL_PADHOR_SYM(32, vec32u)

APU_TIL_PADHOR_SYM_H2(08, vec08u)
APU_TIL_PADHOR_SYM_H2(16, vec16u)


APU_TIL_PADHOR_E0X(08, vec08u)
APU_TIL_PADHOR_E0X(16, vec16u)
APU_TIL_PADHOR_E0X(32, vec32u)

APU_TIL_PADHOR_H2_E0X(08, vec08u)
APU_TIL_PADHOR_H2_E0X(16, vec16u)
APU_TIL_PADHOR_H2_E0X(32, vec32u)

APU_TIL_PADHOR_SYM_E0X(08, vec08u)
APU_TIL_PADHOR_SYM_E0X(16, vec16u)

APU_TIL_PADHOR_SYM_H2_E0X(08, vec08u)
APU_TIL_PADHOR_SYM_H2_E0X(16, vec16u)

void apu_til_copy_right(
               vec08u* restrict dst, int dspan,
         const vec08u* restrict src, int sspan, int sw,
               int widthInBytes, int height,
               vec16u mask
)
{
   if (!(widthInBytes&&height)) {
      return;
   }
   if (vany((vbool)(mask==1))) {
      // If the addresses are 16-bit aligned and the copy bytes is multiple of 2
      // we can use the more efficient 16-bit copies
      if (!((sw&0x1) | (widthInBytes&0x1) | (((intptr_t)src)&0x1) | (((intptr_t)dst)&0x1))) {
         widthInBytes >>= 1;
         vec16u* src16 = (vec16u*)src;
         vec16u* dst16 = (vec16u*)dst;
         sspan >>= 1;
         dspan >>= 1;
         sw >>= 1;
         if (!((sw&0x1) | (widthInBytes&0x1))) {
            apu_til_copy_right_16_w2(dst16, dspan, src16, sspan, sw, widthInBytes, height, mask);
         } else {
            apu_til_copy_right_16(dst16, dspan, src16, sspan, sw, widthInBytes, height, mask);
         }
      } else {
         if (!((sw&0x1) | (widthInBytes&0x1))) {
            apu_til_copy_right_08_w2(dst, dspan, src, sspan, sw, widthInBytes, height, mask);
         } else {
            apu_til_copy_right_08(dst, dspan, src, sspan, sw, widthInBytes, height, mask);
         }
      }
   }
}


#define APU_TIL_COPY_RIGHT(tag, VECT)                                      \
void apu_til_copy_right_##tag(                                             \
               VECT* restrict dst, int dstr,                               \
         const VECT* restrict src, int sstr, int sw,                       \
               int bw, int bh,                                             \
               vec16u mask                                                 \
)                                                                          \
{                                                                          \
   const VECT* restrict ps;                                                \
   VECT* restrict pd;                                                      \
                                                                           \
   for (int i = 0; i < bh; ++i) chess_loop_range(1, ) {                    \
      pd = dst;                                                            \
      dst += dstr;                                                         \
      for (int j = 0; j < bw;) chess_loop_range(1, ) {                     \
         ps = src;                                                         \
         VECT val = *ps++;                                                 \
         for (int k = 0; k < sw; k++,j++) chess_loop_range(1, ) {          \
            vif ((vbool)(mask==1)) {                                      \
               *pd = val;                                                  \
            vendif}                                                        \
            val = *ps++;                                                   \
            pd++;                                                          \
         }                                                                 \
      }                                                                    \
      src += sstr;                                                         \
   }                                                                       \
}

#define APU_TIL_COPY_RIGHT_W2(tag, VECT)                                   \
void apu_til_copy_right_##tag ##_w2(                                       \
               VECT* restrict dst, int dstr,                               \
         const VECT* restrict src, int sstr, int sw,                       \
               int bw, int bh,                                             \
               vec16u mask                                                 \
)                                                                          \
{                                                                          \
   const VECT* restrict ps;                                                \
   VECT* restrict pd;                                                      \
   VECT* restrict pd2;                                                     \
                                                                           \
   for (int i = 0; i < bh; ++i) chess_loop_range(1, ) {                    \
      pd = dst;                                                            \
      pd2 = dst+1;                                                         \
      dst += dstr;                                                         \
      for (int j = 0; j < bw;) chess_loop_range(1, ) {                     \
         ps = src;                                                         \
         VECT val = *ps++;                                                 \
         VECT val2 = *ps++;                                                \
         for (int k = 0; k < sw; k+=2,j+=2) chess_loop_range(1, ) {        \
            vif ((vbool)(mask==1)) {                                      \
               *pd = val;                                                  \
               *pd2 = val2;                                                \
            vendif}                                                        \
            val = *ps++;                                                   \
            val2 = *ps++;                                                  \
            pd+=2;                                                         \
            pd2+=2;                                                        \
         }                                                                 \
      }                                                                    \
      src += sstr;                                                         \
   }                                                                       \
}

APU_TIL_COPY_RIGHT(08, vec08u)
APU_TIL_COPY_RIGHT(16, vec16u)
APU_TIL_COPY_RIGHT(32, vec32u)
APU_TIL_COPY_RIGHT_W2(08, vec08u)
APU_TIL_COPY_RIGHT_W2(16, vec16u)
APU_TIL_COPY_RIGHT_W2(32, vec32u)

void apu_til_copy(
               vec08u* restrict dst, int dspan,
         const vec08u* restrict src, int sspan, int sh,
               int widthInBytes, int height
)
{
   if (!(widthInBytes&&height)) {
      return;
   }
   
   if (!((widthInBytes&0x1) || (((intptr_t)src)&0x1) || (((intptr_t)dst)&0x1))) {
      widthInBytes >>= 1;
      vec16u* src16 = (vec16u*)src;
      vec16u* dst16 = (vec16u*)dst;
      sspan >>= 1;
      dspan >>= 1;
      // Ideally we want to make 4 loads and 4 stores in each loop
      // This is because there is a 3 instruction delay before
      // the loaded value can be used.
      if (!(widthInBytes&0x1)) {
         // if (height&0x1) {
            apu_til_copy_16_w2(dst16, dspan, src16, sspan, sh, widthInBytes, height);
         // } else {
            // apu_til_copy_16_w2_h2(dst16, dspan, src16, sspan, sh, widthInBytes, height);
         // }
      } else {
         apu_til_copy_16(dst16, dspan, src16, sspan, sh, widthInBytes, height);
      }
   } else {
      // Ideally we want to make 4 loads and 4 stores in each loop
      // This is because there is a 3 instruction delay before
      // the loaded value can be used.
      if (!(widthInBytes&0x1)) {
         // if (height&0x1) {
            apu_til_copy_08_w2(dst, dspan, src, sspan, sh, widthInBytes, height);
         // } else {
            // apu_til_copy_08_w2_h2(dst, dspan, src, sspan, sh, widthInBytes, height);
         // }
      } else {
         apu_til_copy_08(dst, dspan, src, sspan, sh, widthInBytes, height);
      }
   }
}

#define APU_TIL_COPY_W2_H2(tag, VECT)                             \
void apu_til_copy_##tag ##_w2_h2(                                 \
               VECT* restrict dst, int dstr,                      \
         const VECT* restrict src, int sstr, int sh,              \
               int bw, int bh                                     \
)                                                                 \
{                                                                 \
   const VECT* restrict orig;                                     \
   const VECT* restrict ps;                                       \
   const VECT* restrict ps2;                                      \
   VECT* restrict pd;                                             \
   VECT* restrict pd2;                                            \
                                                                  \
   orig = src;                                                    \
   for (int i = 0; i < bh; ) chess_loop_range(1, ) {              \
      src = orig;                                                 \
      for (int k = 0; k < sh; k+=2,i+=2) chess_loop_range(1, ) {  \
         pd = dst;                                                \
         pd2 = dst+dstr;                                          \
         ps = src;                                                \
         src += sstr;                                             \
         ps2 = src;                                               \
         src += sstr;                                             \
         for (int j = 0; j < bw; j+=2) chess_loop_range(1, ) {    \
            *pd++ = *ps++;                                        \
            *pd++ = *ps++;                                        \
            *pd2++ = *ps2++;                                      \
            *pd2++ = *ps2++;                                      \
         }                                                        \
         dst += 2*dstr;                                           \
      }                                                           \
   }                                                              \
}

#define APU_TIL_COPY_W2(tag, VECT)                                \
void apu_til_copy_##tag ##_w2(                                    \
               VECT* restrict dst, int dstr,                      \
         const VECT* restrict src, int sstr, int sh,              \
               int bw, int bh                                     \
)                                                                 \
{                                                                 \
   const VECT* restrict orig;                                     \
   const VECT* restrict ps;                                       \
   VECT* restrict pd;                                             \
                                                                  \
   orig = src;                                                    \
   for (int i = 0; i < bh; ) chess_loop_range(1, ) {              \
      src = orig;                                                 \
      for (int k = 0; k < sh; ++k,++i) chess_loop_range(1, ) {    \
        pd = dst;                                                 \
         ps = src;                                                \
         dst += dstr;                                             \
         src += sstr;                                             \
         for (int j = 0; j < bw; j+=2) chess_loop_range(1, ) {    \
            *pd++ = *ps++;                                        \
            *pd++ = *ps++;                                        \
         }                                                        \
      }                                                           \
   }                                                              \
}

#define APU_TIL_COPY(tag, VECT)                                   \
void apu_til_copy_##tag (                                         \
               VECT* restrict dst, int dstr,                      \
         const VECT* restrict src, int sstr, int sh,              \
               int bw, int bh                                     \
)                                                                 \
{                                                                 \
   const VECT* restrict orig;                                     \
   const VECT* restrict ps;                                       \
   VECT* restrict pd;                                             \
                                                                  \
   orig = src;                                                    \
   for (int i = 0; i < bh; ) chess_loop_range(1, ) {              \
      src = orig;                                                 \
      for (int k = 0; k < sh; ++k,++i) chess_loop_range(1, ) {    \
         pd = dst;                                                \
         ps = src;                                                \
         src += sstr;                                             \
         dst += dstr;                                             \
         for (int j = 0; j < bw; ++j) chess_loop_range(1, ) {     \
            *pd++ = *ps++;                                        \
         }                                                        \
      }                                                           \
   }                                                              \
}

APU_TIL_COPY(08, vec08u)
APU_TIL_COPY_W2(08, vec08u)
// APU_TIL_COPY_W2_H2(08, vec08u)

APU_TIL_COPY(16, vec16u)
APU_TIL_COPY_W2(16, vec16u)
// APU_TIL_COPY_W2_H2(16, vec16u)

APU_TIL_COPY(32, vec32u)
APU_TIL_COPY_W2(32, vec32u)
// APU_TIL_COPY_W2_H2(32, vec32u)

#if 0
//CURRENTLY ONLY SUPPORTS PADDING THAT REACHES AS FAR AS A SINGLE CU (need to modify if I want it to work with 2 CUs away)
#define APU_BLK_PADHOR(tag, VECT)                                                     \
void apu_blk_padhor_##tag(VECT* blk, VECT* leftWrap, VECT* rightWrap,                 \
                          int bw, int bh, int bs, int e0x, int leftPad, int rightPad) \
{                                                                                     \
   VECT* restrict sl;                                                                 \
   VECT* restrict sl2;                                                                \
   VECT* restrict sr;                                                                 \
   VECT* restrict sr2;                                                                \
   VECT vrp, vlp;                                                                     \
   VECT vr, vl;                                                                       \
                                                                                      \
   int bx;                                                                            \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                                \
      sl = blk-1;                                                                     \
      sl2 = blk;                                                                      \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                \
      sr2 = blk + e0x*(bw-1) + e0x;                                                   \
                                                                                      \
      for (bx=0; bx<leftPad*e0x;) {                                                   \
         VECT* restrict tmp = leftWrap + e0x - 1;                                     \
         vrp = *tmp;                                                                  \
         vr = *sr;                                                                    \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                  \
            vrp = vmrl(vrp);                                                          \
            vr = vmrr(vr, vrp);                                                       \
            *sl-- = vr;                                                               \
            vrp = *--tmp;                                                             \
            vr = *--sr;                                                               \
         }                                                                            \
      }                                                                               \
      for (int br=0; br<rightPad*e0x;) {                                              \
         VECT* restrict tmp2 = rightWrap + e0x*(bw-1);                                \
         vlp = *tmp2;                                                                 \
         vl = *sl2;                                                                   \
         for (int e = 0; e < e0x; ++e, ++br) chess_loop_range(1, ) {                  \
            vlp = vmrr(vlp);                                                          \
            vl = vmrl(vl, vlp);                                                       \
            *sr2++ = vl;                                                              \
            vlp = *++tmp2;                                                            \
            vl = *++sl2;                                                              \
         }                                                                            \
      }                                                                               \
      leftWrap += bs;                                                                 \
      rightWrap += bs;                                                                \
      blk += bs;                                                                      \
   }                                                                                  \
}
#endif

//NOTE: following seems to work for doing proper padding between wrapped lines (doesn't do left replication though ...)
//CURRENTLY ONLY SUPPORTS PADDING THAT REACHES AS FAR AS A SINGLE CU
#define APU_BLK_PADHOR(tag, VECT)                                                     \
void apu_blk_padhor_##tag(VECT* blk, VECT* leftWrap, VECT* rightWrap,                 \
                          int bw, int bh, int bs, int e0x, int leftPad, int rightPad) \
{                                                                                     \
   VECT* restrict sl;                                                                 \
   VECT* restrict sl2;                                                                \
   VECT* restrict sr;                                                                 \
   VECT* restrict sr2;                                                                \
   VECT vrp, vlp;                                                                     \
   VECT vr, vl;                                                                       \
                                                                                      \
   int bx;                                                                            \
   for (int by=0; by<bh; ++by) chess_loop_range(1, ) {                                \
      sl = blk-1;                                                                     \
      sl2 = blk;                                                                      \
      sr = blk + e0x*(bw-1) + e0x - 1;                                                \
      sr2 = blk + e0x*(bw-1) + e0x;                                                   \
                                                                                      \
      for (bx=0; bx<leftPad*e0x;) {                                                   \
         VECT* restrict tmp = leftWrap - bx + e0x - 1;                                \
         vrp = *tmp;                                                                  \
         vr = *sr;                                                                    \
         for (int e = 0; e < e0x; ++e, ++bx) chess_loop_range(1, ) {                  \
            /*vrp = vmrl(vrp);*/                                                      \
            vr = vmrr(vr, vrp);                                                       \
            *sl-- = vr;                                                               \
            vrp = *--tmp;                                                             \
            vr = *--sr;                                                               \
         }                                                                            \
      }                                                                               \
      for (int br=0; br<rightPad*e0x;) {                                              \
         VECT* restrict tmp2 = rightWrap + br;                                        \
         vlp = *tmp2;                                                                 \
         vl = *sl2;                                                                   \
         for (int e = 0; e < e0x; ++e, ++br) chess_loop_range(1, ) {                  \
            /*vlp = vmrr(vlp);*/                                                      \
            vl = vmrl(vl, vlp);                                                       \
            *sr2++ = vl;                                                              \
            vlp = *++tmp2;                                                            \
            vl = *++sl2;                                                              \
         }                                                                            \
      }                                                                               \
      leftWrap += bs;                                                                 \
      rightWrap += bs;                                                                \
      blk += bs;                                                                      \
   }                                                                                  \
}

APU_BLK_PADHOR(08, vec08u)
APU_BLK_PADHOR(16, vec16u)
APU_BLK_PADHOR(32, vec32u)

#define APU_BLK_ZERO(tag, VECT)                                           \
void apu_blk_zero_##tag (                                                 \
               VECT* restrict dst, int dstr, int bw, int bh, vec16u mask) \
{                                                                         \
   VECT* restrict pd;                                                     \
                                                                          \
   for (int i = 0; i < bh; ++i) chess_loop_range(1, ) {                   \
      pd = dst;                                                           \
      dst += dstr;                                                        \
      for (int j = 0; j < bw; ++j) chess_loop_range(1, ) {                \
         vif ((vbool)(mask==1)); {                                        \
            *pd++ = 0;                                                    \
         vendif}                                                          \
      }                                                                   \
   }                                                                      \
}

APU_BLK_ZERO(08, vec08u)
APU_BLK_ZERO(16, vec16u)
APU_BLK_ZERO(32, vec32u)
