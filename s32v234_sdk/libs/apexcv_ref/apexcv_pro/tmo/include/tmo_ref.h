/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2015, Digital Media Professionals. All rights reserved.
* Copyright (c) 2016 Freescale Semiconductor
* Copyright 2017 NXP 
* All Rights Reserved
*
******************************************************************************
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



#ifndef TMOAPEXREF_H
#define TMOAPEXREF_H

void calc_world_lum_rgbe(
   unsigned char *src,     //[in] source image
   int sstr,            //[in] stride of source buffer
   unsigned char *ie,      //[in] exponent part of intermediate format
   unsigned char *im,      //[in] mantissa part of intermediate format
   int istr,            //[in] stride of intermediate format
   unsigned char *we,      //[out]exponent part of world luminance
   unsigned char *wm,      //[out]mantissa part of world luminance
   int wstr,            //[in] stride of world luminance
   signed int *sum_we,     //[out]accumulation of world luminance exponent part
   signed int *sum_wm,     //[out]accumulation of world luminance mantissa part 
   unsigned int *count, //[out]number of non-zero world luminance exponent part
   int chunk_width,     //[in] chunk width
   int chunk_height     //[in] chunk height
   );

void calc_world_lum_openexr(
   unsigned short *src, //[in] source image
   int sstr,            //[in] stride of source buffer
   unsigned char *ie,      //[in] exponent part of intermediate format
   unsigned char *im,      //[in] mantissa part of intermediate format
   int istr,            //[in] stride of intermediate format
   unsigned char *we,      //[out]exponent part of world luminance
   unsigned char *wm,      //[out]mantissa part of world luminance
   int wstr,            //[in] stride of world luminance
   signed int *sum_we,     //[out]accumulation of world luminance exponent part
   signed int *sum_wm,     //[out]accumulation of world luminance mantissa part 
   unsigned int *count, //[out]number of non-zero world luminance exponent part
   int chunk_width,     //[in] chunk width
   int chunk_height     //[in] chunk height
   );

void calc_geo_mean(
   signed int global_sum_we,     // [in] sum of world luminance exponent
   signed int global_sum_wm,     // [in] sum of world luminance mantissa
   unsigned int global_count,    // [in] world luminance pixel count
   unsigned char *geo_me,        // [out] geometric exponent mean
   unsigned char *geo_mm         // [out] geometric mantissa mean
   );

void ldr_transform(
   unsigned char geo_me,   //[in] exponent part of geometric mean
   unsigned char geo_mm,   //[in] mantissa part of geometric mean
   unsigned char *ie,      //[in] exponent part of intermediate format
   unsigned char *im,      //[in] mantissa part of intermediate format
   int sstr,            //[in] stride of intermediate format image
   unsigned char *we,      //[in] exponent part of world luminance
   unsigned char *wm,      //[in] mantissa part of world luminace
   int wstr,            //[in] stride of world luminance image
   unsigned char key_value,//[in] key value
   unsigned char *dst,     //[out]destination buffer
   int dstr,            //[in] stride of destination buffer
   int chunk_width,     //[in] chunk width
   int chunk_height     //[in] chunk height
   );



#endif /* TMOAPEXREF_H */
