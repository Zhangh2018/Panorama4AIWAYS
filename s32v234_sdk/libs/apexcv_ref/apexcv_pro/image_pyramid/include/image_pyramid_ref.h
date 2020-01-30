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
*  \file pyramid_apex_ref.h
*  \brief Reference kernels for image pyramid creation algorithm
***********************************************************************************/

#ifndef PYRAMIDAPEXREF_H
#define PYRAMIDAPEXREF_H

void horizontal_gaus_and_expand(
   unsigned char *src,  // [in] ptr to src image
   int sstr,            // [in] stride of src image
   short *dst,          // [out] ptr to dst image
   int dstr,            // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
);

void vertical_gaus(
   short *src,          // [in] ptr to src image
   int sstr,            // [in] stride of src image
   unsigned char *dst,  // [out] ptr to dst image
   int dstr,            // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
);

void horizontal_gaus(
   unsigned char *src,  // [in] ptr to src image
   int sstr,            // [in] stride of src image
   short *dst,          // [out] ptr to dst image
   int dstr,            // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
);

void vertical_gaus_and_reduce(
   short *src,          // [in] ptr to src image
   int sstr,            // [in] stride of src image
   unsigned char *dst,  // [out] ptr to dst image
   int dstr,            // [in] stride of dst image
   int width,        // [in] width
   int height        // [in] height
);

#endif /* PYRAMIDAPEXREF_H */
