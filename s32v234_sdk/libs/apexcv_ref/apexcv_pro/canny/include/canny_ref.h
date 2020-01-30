/*****************************************************************************
* 
* NXP Confidential Proprietary
*
* Copyright (c) 2014-2016 Freescale Semiconductor
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

/*!*********************************************************************************
*  \file apexcv_canny_ref.hpp
*  \brief Reference kernels for Canny edge detection algorithm
***********************************************************************************/

#ifndef CANNYREF_H
#define CANNYREF_H
#include <stdint.h>

template<typename T>
void pad_image(T* dst, const T* src, int sw, int sh, int pw, int ph);

template<typename T>
void pad_image_rep(T* blk, int width, int height, int x_pad, int y_pad);

void ref_canny_suppress(
               uint8_t* dEdges, int estr,
         const uint16_t* sMag, int mstr,
         const int16_t* dx, int xstr,
         const int16_t* dy, int ystr,
               int bw, int bh,
               uint16_t low, uint16_t high);

void ref_canny_create_image(
               uint8_t* dst, int dstr,
         const uint8_t* src, int sstr,
         const uint8_t* edge, int estr,
               int bw, int bh);

void ref_canny_sobel_3x3_x(
            int16_t* dst, int dstr,
      const uint8_t* src, int sstr,
      int bw, int bh);

void ref_canny_sobel_3x3_y(
            int16_t* dst, int dstr,
      const uint8_t* src, int sstr,
      int bw, int bh);
      
void ref_canny_magnitude(
               uint16_t* dst, int dstr,
         const int16_t* srcx, int sstrx,
         const int16_t* srcy, int sstry,
               int bw, int bh);

#endif /* CANNYREF_H */
