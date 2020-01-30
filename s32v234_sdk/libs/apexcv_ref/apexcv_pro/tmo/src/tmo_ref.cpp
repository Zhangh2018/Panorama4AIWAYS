/**************************************************************************************************
 *
 * NXP Confidential Proprietary
 *
 * Copyright (c) 2015, Digital Media Professionals. 
 * Copyright (c) 2016-2017 NXP
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

#include "tmo_ref.h"
#include <math.h>
#include <stdio.h>
#include <tmo_lut.h>

#define RGB_DATA_SIZE         3
#define RGBE_DATA_SIZE        4
#define LOG_SHIFT             8
#define EXP2_SHIFT            8
#define TABLE_ELE_NUM         256
#define ROUND_MASK            0xff

static inline short __round_n_shift(int value, unsigned char mask, unsigned char shift_bits)
{
   return (((value & mask) < 0x7f) ? (value >> shift_bits) : ((value >> shift_bits) + 1));
}

static inline unsigned char __select(unsigned char a, unsigned char b, bool condition)
{
   return((condition) ? a : b);
}

static inline signed char __select(signed char a, signed char b, bool condition)
{
   return((condition) ? a : b);
}

static inline unsigned int __select(unsigned int a, unsigned int b, bool condition)
{
   return((condition) ? a : b);
}

static inline int __iselect(int a, int b, bool condition)
{
   return((condition) ? a : b);
}

static inline int __find_log2(unsigned int value)
{
   int count = 0;
   for (int i = 0; i < 32; i++)
   {
      if (value != 0)  count++;
      value >>= 1;
   }

   return (count - 1);
}

static inline short __ceil_n_shift(int value, unsigned char mask, unsigned char shift_bits)
{
   return ((value & mask) ? ((value >> shift_bits) + 1) : (value >> shift_bits));
}

static inline signed int __clamp(signed int value, signed int left_range, signed int right_range)
{
   return ((value < left_range) ? left_range : ((value > right_range) ? right_range : value));
}

static inline unsigned char ___max(unsigned char a, unsigned char b, unsigned char c)
{
   return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
}

static inline unsigned char ___max(unsigned char a, unsigned char b)
{
   return (a > b) ? a : b;
}

static inline unsigned int ___abs(signed int val)
{
   return (unsigned int)((val < 0) ? -val : val);
}

static inline void _calc_world_lum(
   unsigned char *ie,      //[in] exponent part of the intermediate format
   unsigned char *im,      //[in] mantissa part of the intermediate format
   unsigned char *we,      //[out]exponent part of world luminance
   unsigned char *wm       //[out]mantissa part of world luminance
   )
{
   // calculation of world luminance
   // ML = 0.27*(Rm+0.5)*2^Re + 0.67*(Gm+0.5)*2^Ge + 0.06*(Bm+0.5)*2^Be
   // Lwe = log2(ML) - 8
   // Lwm = ML*2^(-Lwe)

   static const short s_k0 = 4424;//0.27*(1<<14)
   static const short s_k1 = 10977;//0.67*(1<<14)
   static const short s_k2 = 983;//0.06*(1<<14)

   // ML = 0.27*(Rm+0.5)*2^Re + 0.67*(Gm+0.5)*2^Ge + 0.06*(Bm+0.5)*2^Be
   unsigned char maxE = ___max(ie[0], ie[1], ie[2]);
   signed int ML = ((s_k0 * ((im[0] << 1) + 1)) >> (maxE - ie[0]))
      + ((s_k1 * ((im[1] << 1) + 1)) >> (maxE - ie[1]))
      + ((s_k2 * ((im[2] << 1) + 1)) >> (maxE - ie[2]));
   ML >>= 15;
   unsigned short tmp = log_table[__clamp(ML, 0, 255)];

   // Lwe = log2(ML) - 8
   unsigned char te = __ceil_n_shift(tmp, ROUND_MASK, LOG_SHIFT) + maxE - 8;
   we[0] = __select(0, te, maxE == 0);

   // Lwm = ML*2^(-Lwe)
   unsigned char tm = __clamp(ML << (maxE - we[0]), 0, 255);
   wm[0] = __select(0, tm, maxE == 0);
}

static inline void _calc_scaled_lum(
   unsigned char we,       //[in] exponent part of world luminance
   unsigned char wm,       //[in] mantissa part of world luminance
   unsigned char geo_me,   //[in] exponent part of geometric mean
   unsigned char geo_mm,   //[in] mantissa part of geometric mean
   unsigned char s_key,    //[in] key value
   unsigned char *se,      //[out]exponent part of scaled luminance
   unsigned char *sm       //[out]mantissa part of scaled luminance
   )
{
   // calculation of scaled luminance
   // AL = key*(Lwm+0.5)/(Gwm+0.5)
   // Le = log2(AL)+Lwe-Gwe+128
   // Lm = AL*2^(136+Lwe-Le-Gwe)

   // AL = (key*(Lwm+0.5)/(Gwm+0.5))<<DIV_SHIFT, where DIV_SHIFT=2
   unsigned int AL = (((s_key * (((int)wm << 1) + 1)) * division_param[geo_mm * 2]) >> division_param[geo_mm * 2 + 1]);

   // Le = log2(AL)+Lwe-Gwe+128-KEY_VALUE_SHIFT
   int Le = log_table[__clamp(AL >> 2, 0, 255)] + ((-8 + we - geo_me + 128 + 1) << LOG_SHIFT);
   Le = __ceil_n_shift(Le, ROUND_MASK, LOG_SHIFT);
   se[0] = (unsigned char)__clamp(Le, 0, 255);

   // Lm = AL*2^(136+Lwe-Le-Gwe-DIV_SHIFT-KEY_VALUE_SHIFT)
   int shift = 136 + we - se[0] - geo_me - 10;
   AL = __select(AL >> ___abs(shift), AL << shift, shift < 0);
   sm[0] = __select(se[0], (unsigned char)__clamp(AL, 0, 255), (Le < 0) || (Le>255));

   return;
}

static inline void _calc_disp_lum(
   unsigned char se,    //[in]exponent part of scaled luminance
   unsigned char sm,    //[in]mantissa part of scaled luminance
   unsigned char *de,   //[out] exponent part of display luminance
   unsigned char *dm    //[out] mantissa part of display luminance
   )
{
   // calculation of display luminance
   // FL = (Lm + 0.5) / (Lm + 0.5 + 2^(136 - Le))
   // Lde = log2(FL) + 128
   // Ldm = FL*2^(136 - Lde)
   signed int Lde;
   unsigned int Ldm;
   signed short tse = 136 - (signed short)se;
   signed short sm2 = (sm << 1) + 1;

   // calculation falls into one of the following 3 cases:
   // Case 1: (136-Le) > 15
   // FL can be approximated as: FL = (Lm + 0.5) / (2^(136-Le)), because 2^(136-Le) is very large
   // Lde = log2(Lm+0.5) - 136 + Le + 128
   // Ldm = (Lm + 0.5)*2^(Le - Lde)
   //
   // Case 2: (136-Le) < -8
   // FL can be approximated as 1, becasue 2^(136-Le) is very small and can be ignored
   // Lde = 128
   // Ldm = 255
   //
   // Case 3: -8 <= (136-Le) <= 15
   // in this case, Lde and Ldm can be calculated with fixed-point arithmetic
   if (tse > 15 || tse < -8)//Case 1 || Case 2
   {
      // Lde = log2(Lm) - 136 + Le + 128
      // Ldm = (Lm+0.5)*2^(Le-Lde)
      Lde = log_table[sm] + ((-136 + se + 128) << LOG_SHIFT);

      // Output calculated Lde for Case 1, 128 for Case 2
      de[0] = __select((unsigned char)__clamp(__ceil_n_shift(Lde, ROUND_MASK, LOG_SHIFT), 0, 255), se, tse > 15);
      // Output calculated Ldm for Case 1, 255 for Case 2
      dm[0] = __select((unsigned char)__clamp(((sm2 << (se - de[0])) >> 1), 0, 255), 255, tse > 15);
   }
   else//Case 3
   {
      // Lde = log2(Lm+0.5) - log2(Lm + 0.5 + 2^(136-Le))
      // if (136-Le)<0, Lde = log2(Lm+0.5) - (log2( (Lm+0.5)<<abs(136-Le+1) + (1<<1) ) - abs(136-Le+1)) + 128
      // if (136-Le)>=0, Lde = log2(Lm+0.5) - (log2( ((Lm+0.5)<<1) + (1<<(136-Le+1) ) - 1) + 128
      unsigned short a = __select((unsigned char)___abs(tse) + 1, (unsigned char)1, tse < 0);
      unsigned int base = __clamp((sm2 << (a - 1)) + (1 << (tse + a)), 0, 0xffff);
      int log_resl = __find_log2(base * base);//The result will be more precise to calculate base*base.
      Lde = ((log_table[sm] << 1) - ((log_resl - ((a + 128) << 1)) << LOG_SHIFT)) >> 1;//log2(base*base) = 2 * log2(base)
      Lde = __ceil_n_shift(Lde, ROUND_MASK, LOG_SHIFT);
      de[0] = __clamp(Lde, 0, 255);

      // Ldm = (Lm+0.5) * 2^(136-Lde) / (Lm + 0.5 + 2^(136-Le))
      unsigned int div = __select((unsigned int)0, (unsigned int)1 << (tse + 1), tse < -1);
      Ldm = (sm2 << (136 - Lde)) / (sm2 + div);
      dm[0] = __select(de[0], (unsigned char)__clamp(Ldm, 0, 255), Lde < 0 || Lde>255);
   }
}

static inline unsigned char _calc_final_rgb(
   unsigned char de,    //[in] exponent part of display luminance
   unsigned char dm,    //[in] mantissa part of display luminance
   unsigned char we,    //[in] exponent part of world luminance
   unsigned char wm,    //[in] mantissa part of world luminance
   unsigned char ie,    //[in] exponent part of the intermediate format
   unsigned char im     //[in] mantissa part of the intermediate format
   )
{
   // calculation of final RGB value
   // RL = (Ldm+0.5)*(Cm+0.5)/(Lwm+0.5)
   // Ci = round(255*RL*2^(Ce+Lde-Lwe-136))

   // RL = (Ldm+0.5)*(Cm+0.5)/(Lwm+0.5), division replaced by mult+shift
   // LUT is created to produce the following division result: when B and C are given, LUT returns A, where A = (B<<1)/C
   // Therefore, the following calculation is actually computing:
   // RL' = ((((Ldm+0.5)<<1)*((Cm+0.5)<<1))<<1)/(Lwm+0.5), thus RL' = RL<<3
   unsigned RL = (((((dm << 1) + 1) * ((im << 1) + 1))*division_param[wm * 2]) >> division_param[wm * 2 + 1]);

   // expo = Ce+Lde-Lwe-136
   short expo = __clamp(ie + de - we - 136, -31, 31);

   // Ci = (round((255<<5)*RL'*2^expo)>>8)
   unsigned int color = __select((RL * 8160) >> ___abs(expo), (RL * 8160) << expo, expo < 0);
   color = __round_n_shift(color, ROUND_MASK, LOG_SHIFT);

   return (__clamp(color, 0, 255));
}

void calc_world_lum_rgbe(
   unsigned char *src,     //[in] source image
   int sstr,               //[in] stride of source buffer
   unsigned char *ie,      //[in] exponent part of intermediate format
   unsigned char *im,      //[in] mantissa part of intermediate format
   int istr,               //[in] stride of intermediate format
   unsigned char *we,      //[out]exponent part of world luminance
   unsigned char *wm,      //[out]mantissa part of world luminance
   int wstr,               //[in] stride of world luminance
   signed int *sum_we,     //[out]accumulation of world luminance exponent part
   signed int *sum_wm,     //[out]accumulation of world luminance mantissa part 
   unsigned int *count,    //[out]number of non-zero world luminance exponent part
   int chunk_width,        //[in] chunk width
   int chunk_height        //[in] chunk height
   )
{
   signed int local_sum_we = 0;
   signed int local_sum_wm = 0;
   unsigned int local_count = 0;

   for (int h = 0; h < chunk_height; h++)
   {
      for (int w = 0; w < chunk_width; w++)
      {
         // convert RGBE format to intermediate format
         for (int k = 0; k < RGB_DATA_SIZE; k++)
         {
            ie[w*RGB_DATA_SIZE + k] = src[w*RGBE_DATA_SIZE + 3];
            im[w*RGB_DATA_SIZE + k] = src[w*RGBE_DATA_SIZE + k];
         }

         // calculate world liminance
         _calc_world_lum(&ie[w*RGB_DATA_SIZE], &im[w*RGB_DATA_SIZE], &we[w], &wm[w]);

         if (we[w] != 0)//Only non-zero Lwe are used. 
         {
            // ƒ°(Lwe - 136)
            // ƒ°log2(Lwm + 0.5)
            local_sum_we += ((signed int)we[w] - 136);
            local_sum_wm += (signed int)log_table[wm[w]];
            local_count++;
         }
      }
      src += sstr;
      ie += istr;
      im += istr;
      we += wstr;
      wm += wstr;
   }
   sum_we[0] += local_sum_we;
   sum_wm[0] += local_sum_wm;
   count[0] += local_count;
}

void calc_world_lum_openexr(
   unsigned short *src, //[in] source image
   int sstr,            //[in] stride of source buffer
   unsigned char *ie,   //[in] exponent part of intermediate format
   unsigned char *im,   //[in] mantissa part of intermediate format
   int istr,            //[in] stride of intermediate format
   unsigned char *we,   //[out]exponent part of world luminance
   unsigned char *wm,   //[out]mantissa part of world luminance
   int wstr,            //[in] stride of world luminance
   signed int *sum_we,  //[out]accumulation of world luminance exponent part
   signed int *sum_wm,  //[out]accumulation of world luminance mantissa part 
   unsigned int *count, //[out]number of non-zero world luminance exponent part
   int chunk_width,     //[in] chunk width
   int chunk_height     //[in] chunk height
   )
{
   signed int local_sum_we = 0;
   signed int local_sum_wm = 0;
   unsigned int local_count = 0;
   for (int h = 0; h < chunk_height; h++)
   {
      for (int w = 0; w < chunk_width; w++)
      {
         // convert OpenEXR format to intermediate format
         for (int k = 0; k < RGB_DATA_SIZE; k++)
         {
            ie[w*RGB_DATA_SIZE + k] = (unsigned char)((src[w*RGB_DATA_SIZE + k] >> 10) & 0x1f) + 128 - 16;
            im[w*RGB_DATA_SIZE + k] = (unsigned char)((src[w*RGB_DATA_SIZE + k] & 0x3ff) >> 3) | 0x80;
         }

         // calculate world luminance
         _calc_world_lum(&ie[w*RGB_DATA_SIZE], &im[w*RGB_DATA_SIZE], &we[w], &wm[w]);

         if (we[w] != 0)//Only non-zero Lwe are used. 
         {
            // ƒ°(Lwe - 136)
            // ƒ°log2(Lwm + 0.5)
            local_sum_we += ((signed int)we[w] - 136);
            local_sum_wm += (signed int)log_table[wm[w]];
            local_count++;
         }
      }
      src += sstr;
      ie += istr;
      im += istr;
      we += wstr;
      wm += wstr;
   }
   sum_we[0] += local_sum_we;
   sum_wm[0] += local_sum_wm;
   count[0] += local_count;
}

void calc_geo_mean(
   signed int global_sum_we,     // [in] sum of world luminance exponent
   signed int global_sum_wm,     // [in] sum of world luminance mantissa
   unsigned int global_count,    // [in] world luminance pixel count
   unsigned char *geo_me,        // [out] geometric exponent mean
   unsigned char *geo_mm         // [out] geometric mantissa mean
   )
{
   // calculation of geometric mean
   // SLwe = 1/N*ƒ°(Lwe - 136)
   // SLwm = 1/N*ƒ°log2(Lwm + 0.5)
   // Gwe = SLwm + SLwe + 128
   // Gwm = 2^(SLwm+SLwe-Gwe+136)
   int SLwe = (int)((float)(global_sum_we << LOG_SHIFT) / (float)global_count);
   int SLwm = global_sum_wm / global_count;

   geo_me[0] = __ceil_n_shift(SLwm + SLwe + (128 << LOG_SHIFT), ROUND_MASK, LOG_SHIFT);

   //+----------------------------------------------------------------------+
   //|exp_table is generated as follows:                                    |
   //|exp_table[i] = (2^(7 + i/256)) * 256;                                 |
   //|we want to calculate 2 to the power of X(X is between 7.0 and 8.0)    |
   //|but since we cannot use fraction, X has been upscaled by 256 times    |
   //|Then the table index i can be calculated as follows:                  |
   //|i = X - 7 * 256= X - 1792                                             |
   //+----------------------------------------------------------------------+
   unsigned short exp2_index = __clamp(SLwm + SLwe + ((-geo_me[0] + 136 - 7) << LOG_SHIFT), 0, 255);
   geo_mm[0] = exp_table[exp2_index];
}

void ldr_transform(
   unsigned char geo_me,   //[in] exponent part of geometric mean
   unsigned char geo_mm,   //[in] mantissa part of geometric mean
   unsigned char *ie,      //[in] exponent part of intermediate format
   unsigned char *im,      //[in] mantissa part of intermediate format
   int sstr,               //[in] stride of intermediate format image
   unsigned char *we,      //[in] exponent part of world luminance
   unsigned char *wm,      //[in] mantissa part of world luminace
   int wstr,               //[in] stride of world luminance image
   unsigned char key_value,//[in] key value
   unsigned char *dst,     //[out]destination buffer
   int dstr,               //[in] stride of destination buffer
   int chunk_width,        //[in] chunk width
   int chunk_height        //[in] chunk height
   )
{
   unsigned char se = 0;
   unsigned char sm = 0;
   unsigned char de = 0;
   unsigned char dm = 0;

   for (int h = 0; h < chunk_height; h++)
   {
      for (int w = 0; w < chunk_width; w++)
      {
         //calculate scaled luminance
         _calc_scaled_lum(we[w], wm[w], geo_me, geo_mm, key_value, &se, &sm);

         //calculate display luminance
         _calc_disp_lum(se, sm, &de, &dm);

         for (int k = 0; k < RGB_DATA_SIZE; k++)
         {
            dst[w*RGB_DATA_SIZE + k] = _calc_final_rgb(de, dm, we[w], wm[w], ie[w*RGB_DATA_SIZE + k], im[w*RGB_DATA_SIZE + k]);
         }
      }
      ie += sstr;
      im += sstr;
      we += wstr;
      wm += wstr;
      dst += dstr;
   }
}
