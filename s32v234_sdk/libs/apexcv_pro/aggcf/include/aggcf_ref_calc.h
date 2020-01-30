/*****************************************************************************
* 
* NXP Confidential Proprietary
*
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

#ifndef AGGCFREFCALC_H
#define AGGCFREFCALC_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef int16_t   fx16p0;
typedef uint16_t ufx16p0;
typedef uint16_t ufx16p1;
typedef uint16_t ufx16p2;
/* 32-bit */
typedef uint32_t ufx32p0;
typedef int32_t   fx32p0;
typedef uint32_t ufx32p1;
typedef uint32_t ufx32p2;
/* 64-bit */
typedef uint64_t ufx64p0;
typedef int64_t   fx64p0;
typedef uint64_t ufx64p1;

uint16_t * image_pad(uint16_t *src, int srcwidth, int srcheight, int left, int right, int up, int bot);

void fxgradMag_c_row(uint16_t* L, uint16_t* U, uint16_t* V, int32_t instride, int32_t outstride,
    uint16_t* M, uint16_t* O,
    int32_t h, int32_t w,
    int32_t *Gx, int32_t *Gy, uint32_t *M2, float &diff);


void fxgradHist_c_row_trilinear(uint16_t* M, uint16_t* O,
    uint16_t* bin0, uint16_t *bin1, uint16_t *bin2, uint16_t *bin3, uint16_t *bin4, uint16_t *bin5,
    int32_t h, int32_t w, int instride, int32_t outstride,
    int32_t wb, int32_t hb, int32_t nb, int32_t bin, int32_t nOrientations, int32_t softBin, int32_t full,
    uint32_t *M0, uint32_t*M1, uint32_t*O0, uint32_t *O1);


void triangleFilt1_c_row(uint16_t* in, uint16_t* out, int32_t h, int32_t w, int32_t instride, int32_t outstride, uint32_t *T, int32_t p, int32_t pad);

void apu_aggcf_rgb2luv(
    uint32_t *rgbImgIn,
    uint16_t *lImgOut,
    uint16_t *uImgOut,
    uint16_t *vImgOut,
    uint16_t strideIn,
    uint16_t strideOut,
    uint16_t Width,
    uint16_t Height
    );
    
void aggcf_pyramid_downscale_ref(uint16_t *in, uint16_t *out, int width, int height, int scale);
    



#endif /* AGGCFREFCALC_H */


