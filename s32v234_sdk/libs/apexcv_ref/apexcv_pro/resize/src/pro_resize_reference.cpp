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

#define __STDC_LIMIT_MACROS
#include <stdint.h> 
 
 // Prototypes
int GetCOL(uint8_t* s, uint8_t* dst, int b, int x, int pad, int sh, int ss);
int PutCOL(uint8_t* d, uint8_t* src, int b, int x, int dh, int ds);
int GetROW(uint8_t* s, uint8_t* dst, int b, int y, int pad, int sw, int ss);
int PutROW(uint8_t* d, const unsigned char* src, int b, int y, int dw, int ds);
int
POLY_INTEGER(
    uint8_t* pdst,
    const uint8_t* psrc,
    int ssiz,              //!< source Size   (OROI)
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd);             //!< Filter Results Rounding Bias
int
HRSZ_RSZ_POLY(
    uint8_t* pdst,
    const uint8_t* psrc,
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd);             //!< Filter Results Rounding Bias
int
VRSZ_RSZ_POLY(
    uint8_t* pdst,
    const uint8_t* psrc,
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd);             //!< Filter Results Rounding Bias
const short* RSZ_GET_POLY(int i);

// const and defines
// 10 matches the HRSZ scaler hardware implementation
#define HRSZ_Q_F           (16)
// It's possible VRSZ_Q_F should match VRSZ_FIXED_SHIFT in APU_resize.c
#define VRSZ_Q_F           (16)
#define HRSZ_BORDER_CONSTANT  1
#define HRSZ_BORDER_REPLICATE 2



 
 /*!
   Load a grayscale image (U8) and scale ().
 */
int
vertical_Scale_08U(uint8_t *src, int sw, int sh, int ss,
                   uint8_t *dst, int dw, int dh, int ds)
{
   int rval = 0;


   // temporary line buffer (4K) (input, processing and output)
   const int wpad =   16; // padding width
   const int wmax = 4096 + 2*wpad;

   uint8_t IBUF[wmax];
   uint8_t OBUF[wmax];

   // intput format
   const int16_t ifmt_scl = 7;
   const int16_t ifmt_off = 0;
   int      rsz_mod = -1;
   int      rsz_phs = -1;
   int      rsz_siz =  0;
   const int16_t* rsz_val;// =  0;

   if       (sh >= 2*dh)
   {
      rsz_mod = 2;
      rsz_phs = 4; // 1
      rsz_siz = 8;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   else if  (sh  >   dh)
   {
      rsz_mod = 1;
      rsz_phs = 8; // 3
      rsz_siz = 4;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   else if  (sh  <   dh)
   {
      rsz_mod = 0;
      rsz_phs = 8; // 3
      rsz_siz = 4;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   const int32_t rsz_scl   =  8;
   const int32_t rsz_rnd   = (1<<(rsz_scl-1));

   const int rsz_one    = (1 << VRSZ_Q_F);
   const int rsz_sinc   = (sh * rsz_one) / dh;
   const int rsz_sbeg   = (rsz_sinc >> 1);
   const int rsz_send   = rsz_sbeg + ((dh-1)*rsz_sinc);

   // output format
   const int16_t ofmt_scl = 7;
   const int16_t ofmt_off = (1 << (ofmt_scl -1));

   {
      for (int y=0; y<dw; ++y)
      {
         GetCOL(src, &IBUF[wpad], 0, y, wpad, sh, ss); //Performs padding also.

         // resize
         if (rsz_mod >= 0)
         {
           VRSZ_RSZ_POLY(&OBUF[wpad], &IBUF[wpad], dh,
                  rsz_sbeg, rsz_sinc,
                  rsz_val, rsz_phs, rsz_siz,
                  rsz_scl, rsz_rnd);
         }
         // scale: ((2^8)-1) / ((2^15)-1) ~= 1 / (2^7)
         PutCOL(dst, &OBUF[wpad], 0, y, dh, ds);
      }
   }


   return rval;
}
int
horizontal_Scale_08U(uint8_t *src, int sw, int sh, int ss,
                     uint8_t *dst, int dw, int dh, int ds)
{
   int rval = 0;


   // temporary line buffer (4K) (input, processing and output)
   const int wpad =   16;
   const int wmax = 4096 + 2*wpad;

   uint8_t IBUF[wmax];
   uint8_t OBUF[wmax];
   int16_t PBUF[16][wmax];

   // intput format
   const int16_t ifmt_scl = 7;
   const int16_t ifmt_off = 0;

   // filter (11-tap, symmetric filter)
   // const int16_t flt_val[] = {  1, 10, 45, 120,  210, 252,  210, 120, 45, 10,  1};
   // const int16_t flt_val[] = { -1,  2, -8,  32, -119, 444, -119,  32, -8,  2, -1 };
   // const int16_t flt_siz   = 11;
   // const int16_t flt_scl   =  8;
   // const int16_t flt_rnd   = (1<<(flt_scl-1));

   //
         int      rsz_mod = -1;
         int      rsz_phs = -1;
         int      rsz_siz =  0;
   const int16_t* rsz_val =  0;

   if       (sw >= 2*dw)
   {
      rsz_mod = 2;
      rsz_phs = 4; // 1
      rsz_siz = 8;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   else if  (sw  >   dw)
   {
      rsz_mod = 1;
      rsz_phs = 8; // 3
      rsz_siz = 4;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   else if  (sw  <   dw)
   {
      rsz_mod = 0;
      rsz_phs = 8; // 3
      rsz_siz = 4;
      rsz_val = RSZ_GET_POLY(rsz_mod);
   }
   const int16_t rsz_scl   =  8;
   const int16_t rsz_rnd   = (1<<(rsz_scl-1));

   const int rsz_one    = (1 << HRSZ_Q_F);
   const int rsz_sinc   = (sw * rsz_one) / dw;    // as per vertical
   const int rsz_sbeg   = (rsz_sinc >> 1);
   const int rsz_send   = rsz_sbeg + ((dw-1)*rsz_sinc);

   // output format
   const int16_t ofmt_scl = 7;
   const int16_t ofmt_off = (1 << (ofmt_scl -1));

   {
      for (int y=0; y<dh; ++y)
      {
         GetROW(src, &IBUF[wpad], 0, y, wpad, sw, ss);

         if (rsz_mod >= 0)
         {
            HRSZ_RSZ_POLY(&OBUF[wpad], &IBUF[wpad], dw,
                  rsz_sbeg, rsz_sinc,
                  rsz_val, rsz_phs, rsz_siz,
                  rsz_scl, rsz_rnd);

         }

         PutROW(dst, &OBUF[wpad], 0, y, dw, ds);
      }
   }

   return rval;
}

///


int GetCOL(uint8_t* s, uint8_t* dst, int b, int x, int pad, int sh, int ss)
{
   int rval = 0;

   const uint8_t* src = s + b + x*1;

   const int h = sh;
   int y;
   for (y=0; y<h; ++y)
   {
      dst[y] =  src[0];
      src   += ss;
   }

   // PADDING
   if (pad>0)
   {
      for (y=-pad; y<0; ++y)
         dst[y] = dst[0];
      for (y=0; y<pad; ++y)
         dst[h+y] = dst[h-1];
   }

   return rval;
}

int PutCOL(uint8_t* d, uint8_t* src, int b, int x, int dh, int ds)
{
   int rval = 0;

        unsigned char* dst = d + b + x*1;

   const int h = dh;
   int y;
   for (y=0; y<h; ++y)
   {
      dst[0] =  src[y];
      dst   += ds;
   }

   return rval;
}

int GetROW(uint8_t* s, uint8_t* dst, int b, int y, int pad, int sw, int ss)
{
   int rval = 0;

   const unsigned char* src = s + b + y*ss;

   const int w = sw;
   int x;
   for (x=0; x<w; ++x)
   {
      dst[x] =  src[0];
      src   += 1;       // 1 for 8-bit image
   }

   // PADDING
   if (pad>0)
   {
      for (x=-pad; x<0; ++x)
         dst[x] = dst[0];
      for (x=0; x<pad; ++x)
         dst[w+x] = dst[w-1];
   }

   return rval;
}

int PutROW(uint8_t* d, const unsigned char* src, int b, int y, int dw, int ds)
{
   int rval = 0;

   unsigned char* dst = d + b + y*ds;

   const int w = dw;
   int x;
   for (x=0; x<w; ++x)
   {
      dst[0] =  src[x];
      dst   += 1;          // for 8-bit image
   }

   return rval;
}

// perform vertical resize
int
POLY_INTEGER(
    uint8_t* pdst,
    const uint8_t* psrc,
    int ssiz,              //!< source Size   (OROI)
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd)              //!< Filter Results Rounding Bias
{
   int rval = 0;

   // saturation value
   const int32_t dmin = 0;
   const int32_t dmax = UINT8_MAX;

   const int     dscl = VRSZ_Q_F;

   // conversion of fphs
   switch (rphs)
   {
   case  8:  rphs = 3; break; // filter size:  4
   case  4:  rphs = 2; break; // filter size:  8
   case  2:  rphs = 1; break; // filter size: 16
   case  1:                   // filter size: 32
   // n/a
   default:
            rphs = 0;
      break;
   }

   // source/destination positions
   int S, D;

   // correction of starting point
   S = sbeg - (((rsiz-1) >> 1)<<dscl);
   for (D=0; D<dsiz; D++, S += sinc)
   {
	  const int FB = S >> dscl;
      // filter
      int32_t d = 0;
      for (int fk=0; fk<rsiz; ++fk)//////////////////////////////////**************************************
         d += ((int32_t)fbnk[fk] * (int32_t)psrc[FB+fk]);

      // rounding and scaling
      d  += rrnd;
      d >>= rscl;

      // saturation
      if       (d < dmin) d = dmin;
      else if  (d > dmax) d = dmax;

      pdst[D] = d;
   }

   return rval;
}


// Same as VRSZ_RSZ_POLY except for the dscl value.  TODO fix this - use 1 function with 1 extra parameter
int
VRSZ_RSZ_POLY(
    uint8_t* pdst,
    const uint8_t* psrc,
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd)              //!< Filter Results Rounding Bias
{
   int rval = 0;

   // saturation value
   const int32_t dmin = 0;
   const int32_t dmax = UINT8_MAX;

   const int32_t dscl = VRSZ_Q_F;

   // conversion of fphs
   switch (rphs)
   {
   case  8:  rphs = 3; break; // filter size:  4
   case  4:  rphs = 2; break; // filter size:  8
   case  2:  rphs = 1; break; // filter size: 16
   case  1:                   // filter size: 32
   // n/a
   default:
            rphs = 0;
      break;
   }

   // source/destination positions
   int32_t S, D;

   // correction of starting point
   S = sbeg - (((rsiz-1) >> 1) << dscl);

   for (D=0; D<dsiz; ++D, S += sinc)
   {
      // extraction of interpolation parameters
      const int32_t FB = (S  >> dscl);             // filter source (integer)
      const int32_t FF = (S  & ((1 << dscl)-1));   // filter source (fraction)
      const int32_t FP = (FF >> (dscl - rphs));    // filter phase

      // input selection
      const uint8_t* s   = psrc +  FB;
      // filter selection
      const int16_t* flt = fbnk + (FP * rsiz);

      // filter
      int32_t d = 0;
      for (int fk=0; fk<rsiz; ++fk)
         d += ((int32_t)flt[fk] * (int32_t)s[fk]);
         
      // rounding and scaling
      d  += rrnd;
      d >>= rscl;


      // saturation
      if       (d < dmin) d = dmin;
      else if  (d > dmax) d = dmax;

      pdst[D] = d;
   }
   
   return rval;
}



/*!
   Generic Resizing using Polyphase Filter.
      - Filter Coefficients  S10 [ -512, 511 ]
      - Internal Accumulator S32 (28-bit signed).
      - Rounding toward minus infinity (with bias)
      - Saturation           S16 [ -INT16_MAX, +INT16_MAX ].
 */
int
HRSZ_RSZ_POLY(
    uint8_t* pdst,
    const uint8_t* psrc,
    int dsiz,              //!< Destination Size   (OROI)
    int sbeg,              //!< Source Begin       (IROI)
    int sinc,              //!< Source Increment
    const int16_t* fbnk,   //!< Filter Bank Coefficients
    int rphs,              //!< Filter Bank Number of Phases
    int rsiz,              //!< Filter Bank Size (per phase)
    int rscl,              //!< Filter Results Scaling
    int rrnd)              //!< Filter Results Rounding Bias
{
   int rval = 0;

   // saturation value
   const int32_t dmin = 0;
   const int32_t dmax = UINT8_MAX;

   const int     dscl = HRSZ_Q_F;

   // conversion of fphs
   switch (rphs)
   {
   case  8:  rphs = 3; break; // filter size:  4
   case  4:  rphs = 2; break; // filter size:  8
   case  2:  rphs = 1; break; // filter size: 16
   case  1:                   // filter size: 32
   // n/a
   default:
            rphs = 0;
      break;
   }

   // source/destination positions
   int S, D;

   // correction of starting point
   S = sbeg - (((rsiz-1) >> 1) << dscl);

   for (D=0; D<dsiz; ++D, S += sinc)
   {
      // extraction of interpolation parameters
      const int32_t FB = (S  >> dscl);             // filter source (integer)
      const int32_t FF = (S  & ((1 << dscl)-1));   // filter source (fraction)
      const int32_t FP = (FF >> (dscl - rphs));    // filter phase

      // input selection
      const uint8_t* s   = psrc +  FB;
      // filter selection
      const int16_t* flt = fbnk + (FP * rsiz);

      // filter
      int32_t d = 0;
      for (int fk=0; fk<rsiz; ++fk)
         d += ((int32_t)flt[fk] * (int32_t)s[fk]);

      // rounding and scaling
      d  += rrnd;
      d >>= rscl;

      // saturation
      if       (d < dmin) d = dmin;
      else if  (d > dmax) d = dmax;

      pdst[D] = d;
   }
   
   return rval;
}



/*!
 Polyphase Filter Coefficients for Resizing (Filter Bank).
 \todo:
    the filter bank should provide:
       - number of phases;
       - number of taps (size) of filter;
       - polyphase filter coefficient values;
       - filter scale and rounding;
 */
const short*
RSZ_GET_POLY(int i)
{
   static const short sSCL_RSZ_FLT[][32] =
   {
      // FILTERS
      // const short FUP[] =
      {   0, 256,  0,   0,
        -11, 245,  23,  -1,
        -17, 220,  58,  -5,
        -18, 184, 100, -10,\
        -15, 143, 143, -15,
        -10, 100, 184, -18,
         -5,  58, 220, -17,
         -1,  23, 245, -11,
      },

      // const short FD1[] =
      { 28, 200,  28, 0,
        15, 194,  47, 0,
         7, 178,  71, 0,
         2, 156,  98, 0,
         0, 128, 128, 0,
         0,  98, 156, 2,
         0,  71, 178, 7,
         0,  47, 194, 15,
      },

      // const short FD2[] =
      {  2, 32, 94,  94, 32,  2, 0, 0,
         0, 19, 80, 105, 47,  5, 0, 0,
         0, 11, 64, 106, 64, 11, 0, 0,
         0,  5, 47, 105, 80, 19, 0, 0,
      },
   };

   return sSCL_RSZ_FLT[i];
}
