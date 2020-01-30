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
*  \file apexcv_interpolation_ref.hpp
*  \brief Reference code for interpolation kernels
***********************************************************************************/

#ifndef INTERPOLATIONREF_H
#define INTERPOLATIONREF_H

#include <stdint.h>

/*!
   Applies a Grayscale Linear Interpolation.

   \param dst       - [Output] Destination block pointer
   \param dstr      - [Input]  Destination block stride
   \param src       - [Input]  Source block pointer
   \param sstr      - [Input]  Source block stride
   \param bw        - [Input]  Block width
   \param bh        - [Input]  Block height
   \param delta     - [Input]  Delta X Values
   */
void
interp_linear_grayscale_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const uint8_t* delta);

/*!
   Applies a Grayscale Bilinear Interpolation.

   \param dst       - [Output] Destination block pointer
   \param dstr      - [Input]  Destination block stride
   \param src       - [Input]  Source block pointer
   \param sstr      - [Input]  Source block stride
   \param bw        - [Input]  Block width
   \param bh        - [Input]  Block height
   \param delta     - [Input]  Delta XY Values
   */
void
interp_bilinear_grayscale_ref(
uint8_t* dst, int dstr,
const uint8_t* src, int sstr,
int bw, int bh,
const uint8_t* delta);

void interp_bicubic_grayscale_ref(
  uint8_t* dst, int dstr,
  const uint8_t* src, int sstr,
  int bw, int bh,
  const uint8_t x_offset, const uint8_t y_offset);


#endif /* INTERPOLATIONREF_H */
